"""Adapters for validation rollout interaction with Gym-like environments."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from srb.interfaces.sim_to_real.validation.numeric import ensure_finite
from srb.utils.failures import (
    FailureCategory,
    FailureInfo,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
    SafetyViolationError,
    classify_exception,
    normalize_health_event,
)


@dataclass(frozen=True)
class ValidationStep:
    """Normalized result of one validation environment step."""

    obs: Any
    reward: float
    terminated: bool
    truncated: bool
    done: bool
    info: dict[str, Any]


_NON_SUCCESS_HEALTH_CATEGORIES = frozenset(
    {
        FailureCategory.INVALID_INPUT,
        FailureCategory.INTEGRITY_FAILURE,
        FailureCategory.SAFETY_VIOLATION,
        FailureCategory.INTERNAL_INVARIANT,
    }
)


def _typed_health_failure(value: object) -> FailureInfo | None:
    """Parse a complete typed failure without guessing from partial fields."""
    if isinstance(value, HealthEvent):
        return value.failure
    if not isinstance(value, Mapping):
        return None
    failure_value = value.get("failure")
    if isinstance(failure_value, FailureInfo):
        return failure_value
    if not isinstance(failure_value, Mapping):
        return None
    try:
        return FailureInfo.from_mapping(failure_value)
    except (TypeError, ValueError):
        # A malformed inner mapping is not a typed carrier.  The caller will
        # classify the whole malformed event as an internal invariant.
        return None


def _health_category_from_value(
    value: object, failure: FailureInfo | None
) -> FailureCategory | None:
    """Infer high-confidence non-success health categories."""
    if failure is not None:
        if failure.category in _NON_SUCCESS_HEALTH_CATEGORIES:
            return failure.category
        return None
    if not isinstance(value, Mapping):
        return None
    mapping = value
    code = mapping.get("kind", mapping.get("code"))
    if not isinstance(code, str):
        return None
    normalized = code.lower().replace("-", "_").replace(".", "_")
    if normalized in {
        "health_check_error",
        "health_check_exception",
        "validation_health_check_exception",
    }:
        return FailureCategory.INTERNAL_INVARIANT
    return None


def _health_failure(
    *,
    category: FailureCategory,
    code: str,
    message: str,
    component: str,
    details: Mapping[str, object],
) -> FailureInfo:
    return FailureInfo(
        category=category,
        code=code,
        message=message or code,
        component=component,
        details=dict(details),
        retryable=False,
        fallback_allowed=False,
        evidence_valid=False,
    )


def _health_boundary_event(
    *,
    code: str,
    message: str,
    component: str,
    details: Mapping[str, object],
    category: FailureCategory = FailureCategory.INTERNAL_INVARIANT,
    failure: FailureInfo | None = None,
) -> HealthEvent:
    if failure is None:
        failure = _health_failure(
            category=category,
            code=code,
            message=message,
            component=component,
            details=details,
        )
    failure_category = failure.category
    severity = (
        HealthSeverity.CRITICAL
        if failure_category is FailureCategory.SAFETY_VIOLATION
        else HealthSeverity.ERROR
    )
    return HealthEvent(
        status=HealthStatus.UNHEALTHY,
        severity=severity,
        component=component,
        code=code,
        message=message or code,
        details=details,
        failure=failure,
    )


def _force_non_success_health(
    event: HealthEvent, category: FailureCategory
) -> HealthEvent:
    """Make inferred invalid/integrity/safety/internal events fail closed."""
    failure = event.failure
    if failure is None or failure.category is not category:
        failure = _health_failure(
            category=category,
            code=f"validation.health.{category.value}",
            message=event.message,
            component=event.component,
            details={"health_code": event.code},
        )
    if event.status is HealthStatus.UNHEALTHY and event.failure is failure:
        return event
    severity = (
        HealthSeverity.CRITICAL
        if category is FailureCategory.SAFETY_VIOLATION
        else HealthSeverity.ERROR
    )
    return HealthEvent(
        status=HealthStatus.UNHEALTHY,
        severity=severity,
        component=event.component,
        code=event.code,
        message=event.message,
        details=event.details,
        failure=failure,
    )


def normalize_validation_health_event(
    value: object, *, component: str = "validation"
) -> HealthEvent | None:
    """Normalize adapter health and fail closed on malformed input.

    Shared by the rollout adapter and episode runner, including test doubles.
    """
    category: FailureCategory | None = None
    parsed_failure: FailureInfo | None = None
    try:
        # Mapping implementations are adapter-owned code. Their ``get`` or
        # iteration methods may raise, so inference must be inside the same
        # fail-closed guard as canonical normalization.
        parsed_failure = _typed_health_failure(value)
        category = _health_category_from_value(value, parsed_failure)
        event = normalize_health_event(value, component=component)
    except Exception as exc:  # noqa: BLE001 - evidence boundary must report
        details = {
            "exception_type": type(exc).__name__,
            "error": str(exc),
            "value_type": type(value).__name__,
        }
        if parsed_failure is not None:
            return _health_boundary_event(
                code=parsed_failure.code,
                message=parsed_failure.message,
                component=parsed_failure.component,
                details=details,
                category=parsed_failure.category,
                failure=parsed_failure,
            )
        return _health_boundary_event(
            code="validation.health_event_malformed",
            message=f"malformed health event: {type(exc).__name__}: {exc}",
            component=component,
            details=details,
            category=category or FailureCategory.INTERNAL_INVARIANT,
        )
    if event is None:
        return None
    if category in _NON_SUCCESS_HEALTH_CATEGORIES:
        return _force_non_success_health(event, category)
    return event


def _health_check_exception_event(iface: object, exc: Exception) -> HealthEvent:
    interface_name = type(iface).__name__
    code = "validation.health_check_exception"
    details = {
        "interface": interface_name,
        "exception_type": type(exc).__name__,
        "error": str(exc),
    }
    failure = classify_exception(
        exc,
        code=code,
        component=interface_name,
        details=details,
    )
    return _health_boundary_event(
        code=failure.code,
        message=failure.message,
        component=failure.component,
        details=details,
        category=failure.category,
        failure=failure,
    )


def _action_clipping_failure(
    *,
    branch: str,
    action_shape: tuple[int, ...] | None,
    space_shape: tuple[int, ...] | None,
    cause: BaseException,
) -> SafetyViolationError:
    cause_type = f"{type(cause).__module__}.{type(cause).__qualname__}"
    return SafetyViolationError(
        code="validation.rollout.action_clipping",
        message=(
            f"cannot safely clip {branch} action shape {action_shape} "
            f"to action space shape {space_shape}"
        ),
        component="validation.rollout",
        details={
            "branch": branch,
            "action_shape": action_shape,
            "space_shape": space_shape,
            "cause_type": cause_type,
            "error": str(cause),
        },
    )


def _shape_tuple(value: Any) -> tuple[int, ...] | None:
    shape = getattr(value, "shape", None)
    if shape is None:
        return None
    return tuple(int(dimension) for dimension in shape)


class ValidationRolloutAdapter:
    """Normalize environment interaction for validation harnesses.

    Accepts legacy Gym 4-tuples and Gymnasium 5-tuples.
    """

    def __init__(self, env: Any) -> None:
        self._env = env

    def reset(self) -> Any:
        return self._normalize_reset(self._env.reset())

    def step(self, action: Any) -> ValidationStep:
        ensure_finite(action, context="policy action")
        adapted = self._adapt_action(action)
        # A finite float64 value can overflow to infinity in float32.
        ensure_finite(adapted, context="adapted policy action")
        result = self._env.step(adapted)
        if not isinstance(result, tuple):
            raise ValueError(  # noqa: TRY004 - preserve the adapter's public error
                f"validation env.step() expected tuple, got {type(result).__name__}"
            )
        if len(result) == 4:
            obs, reward, done, info = result
            terminated = self._as_bool(done)
            truncated = False
        elif len(result) == 5:
            obs, reward, terminated_raw, truncated_raw, info = result
            terminated = self._as_bool(terminated_raw)
            truncated = self._as_bool(truncated_raw)
        else:
            raise ValueError(
                f"validation env.step() expected 4 or 5 values, got {len(result)}"
            )
        return ValidationStep(
            obs=obs,
            reward=self._as_float(reward),
            terminated=terminated,
            truncated=truncated,
            done=terminated or truncated,
            info=self._as_info(info),
        )

    def poll_health(self) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        for iface in self._hardware_interfaces():
            try:
                raw_event = iface.health()
            except Exception as exc:  # noqa: BLE001 - report without stopping rollout
                events.append(_health_check_exception_event(iface, exc).to_mapping())
                continue
            event = normalize_validation_health_event(
                raw_event,
                component=type(iface).__name__,
            )
            if event is not None:
                events.append(event.to_mapping())
        return events

    def summarise_obs(self, obs: Any) -> dict[str, Any]:
        if isinstance(obs, dict):
            return {
                key: (value.tolist() if hasattr(value, "tolist") else value)
                for key, value in obs.items()
                if isinstance(value, (int, float, list, tuple))
                or hasattr(value, "tolist")
            }
        return {}

    def abort_session(self, reason: str) -> None:
        """Propagate a session abort to the real env or its hardware sinks."""

        candidates = [self._env]
        unwrapped = getattr(self._env, "unwrapped", None)
        if unwrapped is not None and unwrapped is not self._env:
            candidates.append(unwrapped)
        for env in candidates:
            abort = getattr(env, "abort_session", None)
            if callable(abort):
                abort(reason)
                return

        errors: list[Exception] = []
        for iface in self._hardware_interfaces():
            safe_stop = getattr(iface, "safe_stop", None)
            if not callable(safe_stop):
                continue
            try:
                safe_stop()
            except Exception as exc:  # noqa: BLE001 - stop remaining interfaces
                errors.append(exc)
        if errors:
            if len(errors) == 1:
                raise errors[0]
            raise ExceptionGroup("validation rollout safe stop failed", errors)

    def hold_motion(self, reason: str) -> None:
        """Zero every action sink between episodes without a session latch."""

        candidates = [self._env]
        unwrapped = getattr(self._env, "unwrapped", None)
        if unwrapped is not None and unwrapped is not self._env:
            candidates.append(unwrapped)
        for env in candidates:
            hold = getattr(env, "hold_motion", None)
            if callable(hold):
                hold(reason)
                return

        errors: list[Exception] = []
        for iface in self._hardware_interfaces():
            pause = getattr(iface, "pause", None)
            if not callable(pause):
                continue
            if getattr(iface, "safe_stop_issued", False) or getattr(
                iface, "is_paused", False
            ):
                continue
            try:
                pause()
            except Exception as exc:  # noqa: BLE001 - hold remaining interfaces
                errors.append(exc)
        if errors:
            if len(errors) == 1:
                raise errors[0]
            raise ExceptionGroup("validation rollout motion hold failed", errors)

    @staticmethod
    def _normalize_reset(result: Any) -> Any:
        if isinstance(result, tuple) and len(result) == 2:
            obs, _info = result
            return obs
        return result

    def _hardware_interfaces(self) -> Any:
        candidates = [self._env]
        unwrapped = getattr(self._env, "unwrapped", None)
        if unwrapped is not None and unwrapped is not self._env:
            candidates.append(unwrapped)
        for env in candidates:
            interfaces = getattr(env, "hardware_interfaces", None)
            if interfaces:
                return interfaces
        return []

    def _adapt_action(self, action: Any) -> Any:
        """Match NumPy policy output to Isaac-style tensor environments."""
        device = self._env_device()
        if device is None:
            return self._clip_action_to_space(action)
        num_envs = self._env_num_envs()
        vectorized = self._env_is_vector_env()
        space = self._env_single_action_space()
        space_shape = _shape_tuple(space) if space is not None else None
        tensor: Any | None = None
        try:
            import torch
        except ImportError:  # pragma: no cover - validation envs normally use torch
            return self._clip_action_to_space(action)
        try:
            if hasattr(action, "to"):
                tensor = action.to(device=device, dtype=torch.float32)
            else:
                tensor = torch.as_tensor(
                    np.array(action, dtype=np.float32, copy=True),
                    dtype=torch.float32,
                    device=device,
                )
            tensor = self._normalize_tensor_action_shape(
                tensor,
                vectorized=vectorized,
                num_envs=num_envs,
                space_shape=space_shape,
            )
            return self._clip_tensor_action_to_space(
                tensor,
                batched=vectorized and num_envs is not None,
            )
        except SafetyViolationError:
            raise
        except (OverflowError, TypeError, ValueError, RuntimeError) as exc:
            action_shape = (
                _shape_tuple(tensor) if tensor is not None else _shape_tuple(action)
            )
            raise _action_clipping_failure(
                branch="tensor",
                action_shape=action_shape,
                space_shape=space_shape,
                cause=exc,
            ) from exc

    def _normalize_tensor_action_shape(
        self,
        tensor: Any,
        *,
        vectorized: bool,
        num_envs: int | None,
        space_shape: tuple[int, ...] | None,
    ) -> Any:
        action_shape = _shape_tuple(tensor)
        if action_shape is None or space_shape is None:
            if vectorized and num_envs is not None and action_shape is not None:
                if action_shape[:1] != (num_envs,):
                    return tensor.reshape(num_envs, -1)
            return tensor

        if not vectorized or num_envs is None:
            if action_shape != space_shape:
                raise ValueError(
                    f"scalar action shape must be {space_shape}, got {action_shape}"
                )
            return tensor

        expected_shape = (num_envs, *space_shape)
        if num_envs == 1 and action_shape == space_shape:
            return tensor.reshape(expected_shape)
        if action_shape == expected_shape:
            return tensor
        if action_shape[:1] != (num_envs,):
            # Keep the legacy reshape boundary so an indivisible flat vector
            # is reported as a typed safety failure rather than a raw Torch
            # RuntimeError.  A successful reshape is still rejected below:
            # vectorized callers must provide the exact batched shape.
            tensor.reshape(num_envs, -1)
        raise ValueError(
            f"vector action shape must be {expected_shape}, got {action_shape}"
        )

    def _clip_action_to_space(self, action: Any) -> Any:
        space = self._env_single_action_space()
        if space is None:
            return action
        array: np.ndarray | None = None
        low: np.ndarray | None = None
        space_shape: tuple[int, ...] | None = None
        try:
            array = np.asarray(action, dtype=np.float32)
            space_shape = _shape_tuple(space)
            if space_shape is not None:
                array = self._normalize_numpy_action_shape(
                    array,
                    vectorized=self._env_is_vector_env(),
                    num_envs=self._env_num_envs(),
                    space_shape=space_shape,
                )
            if not hasattr(space, "low") or not hasattr(space, "high"):
                return array
            low = np.asarray(space.low, dtype=np.float32)
            high = np.asarray(space.high, dtype=np.float32)
            clipped = np.clip(array, low, high)
            if clipped.shape != array.shape:
                raise ValueError(
                    f"clipping changed action shape from {array.shape} "
                    f"to {clipped.shape}"
                )
            return clipped
        except (OverflowError, TypeError, ValueError) as exc:
            raise _action_clipping_failure(
                branch="numpy",
                action_shape=_shape_tuple(array) if array is not None else None,
                space_shape=space_shape
                or (_shape_tuple(low) if low is not None else None),
                cause=exc,
            ) from exc

    def _normalize_numpy_action_shape(
        self,
        array: np.ndarray,
        *,
        vectorized: bool,
        num_envs: int | None,
        space_shape: tuple[int, ...],
    ) -> np.ndarray:
        action_shape = tuple(array.shape)
        if not vectorized or num_envs is None:
            if action_shape != space_shape:
                raise ValueError(
                    f"scalar action shape must be {space_shape}, got {action_shape}"
                )
            return array
        expected_shape = (num_envs, *space_shape)
        if num_envs == 1 and action_shape == space_shape:
            return array.reshape(expected_shape)
        if action_shape != expected_shape:
            raise ValueError(
                f"vector action shape must be {expected_shape}, got {action_shape}"
            )
        return array

    def _clip_tensor_action_to_space(self, tensor: Any, *, batched: bool) -> Any:
        space = self._env_single_action_space()
        if space is None or not hasattr(space, "low") or not hasattr(space, "high"):
            return tensor
        try:
            import torch
        except ImportError:  # pragma: no cover
            return tensor
        low: Any | None = None
        space_shape: tuple[int, ...] | None = None
        try:
            low_array = np.asarray(space.low)
            high_array = np.asarray(space.high)
            space_shape = _shape_tuple(space) or _shape_tuple(low_array)
            low = torch.as_tensor(low_array, dtype=tensor.dtype, device=tensor.device)
            high = torch.as_tensor(high_array, dtype=tensor.dtype, device=tensor.device)
            if batched:
                low = low.reshape((1, *low.shape))
                high = high.reshape((1, *high.shape))
            clipped = torch.clamp(tensor, min=low, max=high)
            if clipped.shape != tensor.shape:
                raise ValueError(
                    f"clipping changed action shape from {tuple(tensor.shape)} "
                    f"to {tuple(clipped.shape)}"
                )
            return clipped
        except (OverflowError, TypeError, ValueError, RuntimeError) as exc:
            raise _action_clipping_failure(
                branch="tensor",
                action_shape=_shape_tuple(tensor),
                space_shape=space_shape,
                cause=exc,
            ) from exc

    def _env_device(self) -> Any | None:
        for env in (self._env, getattr(self._env, "unwrapped", None)):
            if env is None:
                continue
            device = getattr(env, "device", None)
            if device is not None:
                return device
        return None

    def _env_num_envs(self) -> int | None:
        for env in (self._env, getattr(self._env, "unwrapped", None)):
            if env is None:
                continue
            value = getattr(env, "num_envs", None)
            if value is not None:
                return int(value)
        return None

    def _env_is_vector_env(self) -> bool:
        for env in (self._env, getattr(self._env, "unwrapped", None)):
            if env is None:
                continue
            value = getattr(env, "is_vector_env", None)
            if isinstance(value, bool):
                return value
        # Preserve the historical tensor-adapter fallback for test doubles
        # and legacy environments that expose device/num_envs only.
        return True

    def _env_single_action_space(self) -> Any | None:
        outer = self._env
        base = getattr(outer, "unwrapped", None)
        outer_single = getattr(outer, "single_action_space", None)
        if outer_single is not None:
            return outer_single

        outer_action = getattr(outer, "action_space", None)
        if base is None or base is outer:
            if outer_action is not None:
                return outer_action
            return getattr(base, "single_action_space", None)

        base_single = getattr(base, "single_action_space", None)
        base_action = getattr(base, "action_space", None)
        if outer_action is not None:
            if outer_action is base_action and base_single is not None:
                return base_single
            # A distinct wrapper action space is an intentional override.
            return outer_action
        if base_single is not None:
            return base_single
        return base_action

    @staticmethod
    def _as_info(info: Any) -> dict[str, Any]:
        return {} if info is None else dict(info)

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        normalized = ValidationRolloutAdapter._to_numpy_like(value)
        array = np.asarray(normalized)
        if array.shape == ():
            return bool(array.item())
        return bool(np.any(array))

    @staticmethod
    def _as_float(value: Any) -> float:
        if isinstance(value, (int, float)):
            return float(value)
        normalized = ValidationRolloutAdapter._to_numpy_like(value)
        array = np.asarray(normalized, dtype=np.float64)
        if array.size != 1:
            raise ValueError(
                f"validation env.step() expected scalar reward, got shape {array.shape}"
            )
        return float(array.reshape(-1)[0])

    @staticmethod
    def _to_numpy_like(value: Any) -> Any:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            try:
                return value.numpy()
            except TypeError:
                return value
        return value
