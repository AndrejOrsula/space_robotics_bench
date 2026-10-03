from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Literal, TypeAlias

import torch

from srb._typing.step_return import StepReturn

DelayRangeCfg: TypeAlias = int | tuple[int, int] | list[int]
ObservationDict: TypeAlias = dict[str, dict[str, torch.Tensor]]
EnvIds: TypeAlias = torch.Tensor | Sequence[int]
DelayDiagnosticLevel: TypeAlias = Literal["info", "warning"]


# torch.Generator seeds must fit in an unsigned 64-bit integer.
_SEED_MODULUS = 2**64


def _channel_seed(seed: int, channel_index: int) -> int:
    """Offset observation seeds so channels use independent RNG streams."""
    return (int(seed) + 1 + int(channel_index)) % _SEED_MODULUS


def delay_buffer_length(max_delay_steps: int) -> int:
    """Return ring-buffer length needed to represent delays in [0, max]."""
    if isinstance(max_delay_steps, bool) or not isinstance(max_delay_steps, int):
        raise ValueError(
            f"max_delay_steps must be a non-negative int, got {max_delay_steps!r}"
        )
    if max_delay_steps < 0:
        raise ValueError(f"max_delay_steps must be non-negative, got {max_delay_steps}")
    return max_delay_steps + 1


def delayed_history_read_indices(
    *,
    write_index: int,
    delay_steps: torch.Tensor,
    history_length: int,
) -> torch.Tensor:
    if history_length <= 0:
        raise ValueError(f"history_length must be positive, got {history_length}")
    return (write_index - delay_steps + history_length) % history_length


def _normalize_delay_env_ids(
    env_ids: EnvIds,
    *,
    num_envs: int,
    device: torch.device | str,
) -> torch.Tensor:
    if isinstance(env_ids, torch.Tensor):
        if env_ids.dtype == torch.bool:
            if env_ids.shape != (num_envs,):
                raise ValueError(
                    f"boolean env_ids mask must have shape {(num_envs,)}, "
                    f"got {tuple(env_ids.shape)}"
                )
            return torch.nonzero(
                env_ids.to(device=device),
                as_tuple=False,
            ).flatten()
        normalized = env_ids.to(device=device, dtype=torch.long)
    else:
        normalized = torch.as_tensor(env_ids, device=device, dtype=torch.long)
    if normalized.ndim == 0:
        normalized = normalized.reshape(1)
    # Negative indices would silently wrap onto the last environments' slots.
    if normalized.numel() and bool(((normalized < 0) | (normalized >= num_envs)).any()):
        raise ValueError(
            f"env_ids must lie in [0, {num_envs}), got {normalized.tolist()}"
        )
    return normalized


def reset_delay_steps(
    delay_steps: torch.Tensor,
    env_ids: EnvIds,
    delay_range: DelayRange,
    *,
    generator: torch.Generator | None = None,
) -> torch.Tensor:
    """Sample delay steps for selected environments and return normalized ids."""
    normalized_env_ids = _normalize_delay_env_ids(
        env_ids,
        num_envs=int(delay_steps.shape[0]),
        device=delay_steps.device,
    )
    if normalized_env_ids.numel() == 0:
        return normalized_env_ids
    delay_steps[normalized_env_ids] = torch.randint(
        delay_range.min_steps,
        delay_range.max_steps + 1,
        (normalized_env_ids.numel(),),
        generator=generator,
        device=delay_steps.device,
        dtype=torch.long,
    )
    return normalized_env_ids


def fill_reset_history(
    history: torch.Tensor,
    env_ids: EnvIds,
    latest_value: torch.Tensor | None = None,
) -> torch.Tensor:
    """Replicate the latest value through selected env history slots."""
    if history.ndim < 2:
        raise ValueError(f"history must be at least 2-D, got {tuple(history.shape)}")
    num_envs = int(history.shape[1])
    value_shape = tuple(int(dim) for dim in history.shape[2:])
    normalized_env_ids = _normalize_delay_env_ids(
        env_ids,
        num_envs=num_envs,
        device=history.device,
    )
    if normalized_env_ids.numel() == 0:
        return normalized_env_ids
    if latest_value is None:
        history[:, normalized_env_ids] = 0
        return normalized_env_ids
    if tuple(latest_value.shape) != (num_envs, *value_shape):
        raise ValueError(
            f"latest_value shape must be {(num_envs, *value_shape)}, "
            f"got {tuple(latest_value.shape)}"
        )

    latest_env_ids = normalized_env_ids.to(device=latest_value.device)
    scrubbed = torch.nan_to_num(
        latest_value[latest_env_ids].to(device=history.device, dtype=history.dtype),
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    history[:, normalized_env_ids] = scrubbed.unsqueeze(0)
    return normalized_env_ids


def _validate_delay_history(
    history: torch.Tensor,
    *,
    write_index: int,
    delay_steps: torch.Tensor,
) -> tuple[int, int, int]:
    if history.ndim < 2:
        raise ValueError(f"history must be at least 2-D, got {tuple(history.shape)}")
    history_length = int(history.shape[0])
    if history_length <= 0:
        raise ValueError(f"history length must be positive, got {history_length}")
    num_envs = int(history.shape[1])
    if delay_steps.ndim != 1 or int(delay_steps.shape[0]) != num_envs:
        raise ValueError(
            f"delay_steps must have shape {(num_envs,)}, got {tuple(delay_steps.shape)}"
        )

    normalized_write_index = int(write_index)
    if not 0 <= normalized_write_index < history_length:
        raise ValueError(
            f"write_index must be in [0, {history_length}), got {write_index!r}"
        )
    return history_length, num_envs, normalized_write_index


def _write_delay_history(
    history: torch.Tensor,
    value: torch.Tensor,
    *,
    write_index: int,
    scrub_non_finite: bool,
) -> None:
    expected_shape = tuple(int(dim) for dim in history.shape[1:])
    if tuple(value.shape) != expected_shape:
        raise ValueError(
            f"value shape must be {expected_shape}, got {tuple(value.shape)}"
        )
    stored = value.to(device=history.device, dtype=history.dtype)
    if scrub_non_finite:
        stored = torch.nan_to_num(stored, nan=0.0, posinf=0.0, neginf=0.0)
    history[write_index] = stored


def _read_delay_history(
    history: torch.Tensor,
    *,
    write_index: int,
    delay_steps: torch.Tensor,
) -> torch.Tensor:
    history_length, num_envs, normalized_write_index = _validate_delay_history(
        history,
        write_index=write_index,
        delay_steps=delay_steps,
    )
    read_indices = delayed_history_read_indices(
        write_index=normalized_write_index,
        delay_steps=delay_steps.to(device=history.device, dtype=torch.long),
        history_length=history_length,
    )
    env_indices = torch.arange(num_envs, device=history.device)
    return history[read_indices, env_indices].clone()


def push_and_read_delay_history(
    history: torch.Tensor,
    value: torch.Tensor,
    *,
    write_index: int,
    delay_steps: torch.Tensor,
    scrub_non_finite: bool = True,
) -> tuple[torch.Tensor, int]:
    """Push values into history and read delayed values.

    Preserve non-finite observations for the step-return safety check. Actions
    may be scrubbed, but observations must remain visible to that check.
    """
    history_length, _, normalized_write_index = _validate_delay_history(
        history,
        write_index=write_index,
        delay_steps=delay_steps,
    )
    _write_delay_history(
        history,
        value,
        write_index=normalized_write_index,
        scrub_non_finite=scrub_non_finite,
    )
    delayed = _read_delay_history(
        history,
        write_index=normalized_write_index,
        delay_steps=delay_steps,
    )
    next_write_index = (normalized_write_index + 1) % history_length
    return delayed, next_write_index


def push_and_read_observation_delay_history(
    history: ObservationDict,
    observation: ObservationDict,
    *,
    write_index: int,
    delay_steps: torch.Tensor,
) -> tuple[ObservationDict, int]:
    """Push current observations into nested history and read delayed observations."""
    history_length: int | None = None
    normalized_write_index: int | None = None

    for group in history.values():
        for history_buffer in group.values():
            buffer_length, _, buffer_write_index = _validate_delay_history(
                history_buffer,
                write_index=write_index,
                delay_steps=delay_steps,
            )
            if history_length is None:
                history_length = buffer_length
                normalized_write_index = buffer_write_index
            elif history_length != buffer_length:
                raise ValueError(
                    "all observation history buffers must share the same length"
                )

    if history_length is None or normalized_write_index is None:
        return {}, int(write_index)

    for category, group in observation.items():
        history_group = history[category]
        for key, value in group.items():
            _write_delay_history(
                history_group[key],
                value,
                write_index=normalized_write_index,
                scrub_non_finite=False,
            )

    delayed_observation: ObservationDict = {}
    for category, group in history.items():
        delayed_observation[category] = {}
        for key, history_buffer in group.items():
            delayed_observation[category][key] = _read_delay_history(
                history_buffer,
                write_index=normalized_write_index,
                delay_steps=delay_steps,
            )

    next_write_index = (normalized_write_index + 1) % history_length
    return delayed_observation, next_write_index


def randomize_delay_steps_on_boundary(
    delay_steps: torch.Tensor,
    delay_range: DelayRange,
    *,
    current_time: float,
    step_dt: float,
    change_freq: float,
    change_prob: float,
    generator: torch.Generator | None = None,
) -> bool:
    """Randomly move per-env delay steps by one on configured step boundaries."""
    if delay_steps.ndim != 1:
        raise ValueError(f"delay_steps must be 1-D, got {tuple(delay_steps.shape)}")
    if change_prob <= 0.0 or delay_range.min_steps >= delay_range.max_steps:
        return False
    if change_freq <= 0.0:
        raise ValueError(f"change_freq must be positive, got {change_freq!r}")
    if step_dt <= 0.0:
        # step_dt == 0 would make ``current_time % change_freq >= step_dt`` always
        # true, silently disabling boundary randomization. Surface the misconfig.
        raise ValueError(f"step_dt must be positive, got {step_dt!r}")
    if current_time % change_freq >= step_dt:
        return False

    dist = torch.rand(
        (int(delay_steps.shape[0]),),
        device=delay_steps.device,
        generator=generator,
    )
    decrease = (delay_steps > delay_range.min_steps) & (dist < change_prob)
    increase = (
        (delay_steps < delay_range.max_steps) & ~decrease & (dist > (1.0 - change_prob))
    )
    delay_steps[decrease] -= 1
    delay_steps[increase] += 1
    return bool((decrease | increase).any().item())


def _validate_steps(min_steps: int, max_steps: int) -> None:
    if (
        isinstance(min_steps, bool)
        or isinstance(max_steps, bool)
        or not isinstance(min_steps, int)
        or not isinstance(max_steps, int)
    ):
        raise ValueError("Delay steps must be integers, not bool")
    if min_steps < 0 or max_steps < 0:
        raise ValueError("Delay steps must be non-negative")
    if min_steps > max_steps:
        raise ValueError(
            f"min_steps must be <= max_steps, got {min_steps} > {max_steps}"
        )


@dataclass(frozen=True)
class DelayRange:
    min_steps: int
    max_steps: int

    def __post_init__(self) -> None:
        _validate_steps(self.min_steps, self.max_steps)

    @classmethod
    def from_cfg(cls, cfg: DelayRangeCfg) -> DelayRange:
        if isinstance(cfg, bool):
            raise ValueError("Delay range cannot be bool")
        if isinstance(cfg, int):
            return cls(cfg, cfg)
        if isinstance(cfg, tuple | list):
            if len(cfg) != 2:
                raise ValueError(f"Delay range must have length 2, got {len(cfg)}")
            min_steps, max_steps = cfg
            if (
                isinstance(min_steps, bool)
                or isinstance(max_steps, bool)
                or not isinstance(min_steps, int)
                or not isinstance(max_steps, int)
            ):
                raise ValueError("Delay range values must be integers")
            return cls(min_steps, max_steps)
        raise ValueError(f"Unsupported delay range config: {cfg!r}")

    def enabled(self) -> bool:
        return self.max_steps > 0


@dataclass(frozen=True)
class DelayChannelConfig:
    delay_range: DelayRange
    change_freq: float
    change_prob: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.change_freq) or self.change_freq <= 0.0:
            raise ValueError(
                f"change_freq must be finite and positive, got {self.change_freq!r}"
            )
        if not math.isfinite(self.change_prob) or not 0.0 <= self.change_prob <= 1.0:
            raise ValueError(
                f"change_prob must be finite and in [0, 1], got {self.change_prob!r}"
            )

    @classmethod
    def from_cfg(
        cls,
        cfg: object,
        *,
        steps_attr: str,
        change_freq_attr: str,
        change_prob_attr: str,
    ) -> DelayChannelConfig:
        return cls(
            delay_range=DelayRange.from_cfg(getattr(cfg, steps_attr, 0)),
            change_freq=float(getattr(cfg, change_freq_attr, 1.0)),
            change_prob=float(getattr(cfg, change_prob_attr, 0.0)),
        )

    @property
    def history_length(self) -> int:
        return delay_buffer_length(self.delay_range.max_steps)

    def enabled(self) -> bool:
        return self.delay_range.enabled()


@dataclass(frozen=True)
class DelayRuntimeConfig:
    """Normalized action/observation delay policy shared by SRB env runtimes."""

    seed: int
    action: DelayChannelConfig
    observation: DelayChannelConfig

    @classmethod
    def from_cfg(cls, cfg: object) -> DelayRuntimeConfig:
        seed = getattr(cfg, "seed", None)
        if seed is None:
            seed = 0
        return cls(
            seed=int(seed),
            action=DelayChannelConfig.from_cfg(
                cfg,
                steps_attr="action_delay_steps",
                change_freq_attr="action_delay_on_step_change_freq",
                change_prob_attr="action_delay_on_step_change_prob",
            ),
            observation=DelayChannelConfig.from_cfg(
                cfg,
                steps_attr="observation_delay_steps",
                change_freq_attr="observation_delay_on_step_change_freq",
                change_prob_attr="observation_delay_on_step_change_prob",
            ),
        )

    def enabled(self) -> bool:
        return self.action.enabled() or self.observation.enabled()

    def make_generator(self, device: torch.device | str) -> torch.Generator | None:
        if not self.enabled():
            return None
        return torch.Generator(device=device).manual_seed(self.seed)


@dataclass(frozen=True)
class DelayActivationDiagnostic:
    level: DelayDiagnosticLevel
    message: str


@dataclass(frozen=True)
class DelayActivationPlan:
    """Activation policy for action/observation delay in an env runtime."""

    delay_config: DelayRuntimeConfig
    use_step_return_runtime: bool
    use_legacy_action_delay: bool
    diagnostics: tuple[DelayActivationDiagnostic, ...] = ()

    def build_step_return_runtime(
        self,
        cfg: object,
        step_return: StepReturn | None,
        *,
        action_dim: int,
        num_envs: int,
        device: torch.device | str,
        actions_enabled: bool,
    ) -> DelayRuntime | None:
        if not self.use_step_return_runtime:
            return None
        if step_return is None:
            raise ValueError("step_return is required for StepReturn delay runtime")
        return DelayRuntime.from_step_return(
            cfg,
            step_return,
            action_dim=action_dim,
            num_envs=num_envs,
            device=device,
            actions_enabled=actions_enabled,
            delay_config=self.delay_config,
        )

    def build_legacy_action_delay(
        self,
        *,
        action_dim: int,
        num_envs: int,
        device: torch.device | str,
    ) -> ActionDelay | None:
        if not self.use_legacy_action_delay:
            return None
        return ActionDelay.from_config(
            self.delay_config.action,
            action_dim=action_dim,
            num_envs=num_envs,
            device=device,
            seed=self.delay_config.seed,
        )

    def emit_diagnostics(self, logger: object) -> None:
        for diagnostic in self.diagnostics:
            log = getattr(logger, diagnostic.level)
            log(diagnostic.message)


def plan_delay_runtime_activation(
    cfg: object,
    *,
    step_return_workflow: bool,
    actions_enabled: bool,
    action_dim: int,
) -> DelayActivationPlan:
    """Plan delay runtime activation and diagnostics from env capabilities."""
    if isinstance(action_dim, bool) or int(action_dim) < 0:
        raise ValueError(f"action_dim must be a non-negative int, got {action_dim!r}")

    normalized_action_dim = int(action_dim)
    delay_config = DelayRuntimeConfig.from_cfg(cfg)
    diagnostics: list[DelayActivationDiagnostic] = []

    if delay_config.action.enabled():
        if actions_enabled and normalized_action_dim > 0:
            diagnostics.append(
                DelayActivationDiagnostic(
                    "info",
                    "Action delay of maximum "
                    f"{delay_config.action.delay_range.max_steps} agent steps enabled.",
                )
            )
        else:
            diagnostics.append(
                DelayActivationDiagnostic(
                    "warning",
                    "Action delay was requested but no action manager or action "
                    "dimensions are configured; the delay will be ignored.",
                )
            )

    if delay_config.observation.enabled():
        if step_return_workflow:
            diagnostics.append(
                DelayActivationDiagnostic(
                    "info",
                    "Observation delay of maximum "
                    f"{delay_config.observation.delay_range.max_steps} "
                    "agent steps enabled.",
                )
            )
        else:
            diagnostics.append(
                DelayActivationDiagnostic(
                    "warning",
                    "Observation delay was requested but the legacy observation "
                    "workflow does not produce StepReturn observations; the delay "
                    "will be ignored.",
                )
            )

    return DelayActivationPlan(
        delay_config=delay_config,
        use_step_return_runtime=step_return_workflow,
        use_legacy_action_delay=not step_return_workflow and actions_enabled,
        diagnostics=tuple(diagnostics),
    )


class TensorDelayBuffer:
    def __init__(
        self,
        *,
        max_delay_steps: int,
        value_shape: tuple[int, ...],
        num_envs: int,
        device: torch.device | str,
        dtype: torch.dtype,
        delay_range: DelayRange,
        seed: int,
    ) -> None:
        history_length = delay_buffer_length(max_delay_steps)
        if isinstance(num_envs, bool) or not isinstance(num_envs, int) or num_envs <= 0:
            raise ValueError(f"num_envs must be a positive int, got {num_envs!r}")
        if not isinstance(value_shape, tuple) or any(
            isinstance(dim, bool) or not isinstance(dim, int) or dim <= 0
            for dim in value_shape
        ):
            raise ValueError(
                f"value_shape must be a tuple of positive ints, got {value_shape!r}"
            )
        if delay_range.max_steps > max_delay_steps:
            raise ValueError(
                "delay_range.max_steps must be <= max_delay_steps, "
                f"got {delay_range.max_steps} > {max_delay_steps}"
            )

        self.max_delay_steps = max_delay_steps
        self.value_shape = value_shape
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.dtype = dtype
        self.delay_range = delay_range
        self.history_length = history_length
        self.history = torch.zeros(
            (self.history_length, num_envs, *value_shape),
            device=self.device,
            dtype=dtype,
        )
        self.ptr = 0
        self.delay_steps = torch.zeros(num_envs, device=self.device, dtype=torch.long)
        self.generator = torch.Generator(device=self.device)
        self.generator.manual_seed(seed)

    @classmethod
    def from_channel_config(
        cls,
        config: DelayChannelConfig,
        *,
        value_shape: tuple[int, ...],
        num_envs: int,
        device: torch.device | str,
        dtype: torch.dtype,
        seed: int,
    ) -> TensorDelayBuffer:
        """Build a buffer from a channel config and reset all envs once."""
        buf = cls(
            max_delay_steps=config.delay_range.max_steps,
            value_shape=value_shape,
            num_envs=num_envs,
            device=device,
            dtype=dtype,
            delay_range=config.delay_range,
            seed=seed,
        )
        buf.reset(torch.arange(num_envs, device=buf.device, dtype=torch.long))
        return buf

    def reset(self, env_ids: EnvIds) -> None:
        reset_delay_steps(
            self.delay_steps,
            env_ids,
            self.delay_range,
            generator=self.generator,
        )

    def manual_seed(self, seed: int) -> None:
        self.generator.manual_seed(int(seed))

    def randomize_delay_steps_on_boundary(
        self,
        *,
        current_time: float,
        step_dt: float,
        change_freq: float,
        change_prob: float,
    ) -> bool:
        return randomize_delay_steps_on_boundary(
            self.delay_steps,
            self.delay_range,
            current_time=current_time,
            step_dt=step_dt,
            change_freq=change_freq,
            change_prob=change_prob,
            generator=self.generator,
        )

    def fill_reset(
        self, env_ids: EnvIds, latest_value: torch.Tensor | None = None
    ) -> None:
        fill_reset_history(self.history, env_ids, latest_value)

    def push_and_read(
        self, value: torch.Tensor, *, scrub_non_finite: bool = True
    ) -> torch.Tensor:
        delayed, self.ptr = push_and_read_delay_history(
            self.history,
            value,
            write_index=self.ptr,
            delay_steps=self.delay_steps,
            scrub_non_finite=scrub_non_finite,
        )
        return delayed


class ActionDelay:
    def __init__(self, buffer: TensorDelayBuffer | None) -> None:
        self.buffer = buffer

    @classmethod
    def from_config(
        cls,
        config: DelayChannelConfig,
        *,
        action_dim: int,
        num_envs: int,
        device: torch.device | str,
        dtype: torch.dtype = torch.float32,
        seed: int,
    ) -> ActionDelay:
        if not config.enabled() or action_dim <= 0:
            return cls(None)
        return cls(
            TensorDelayBuffer.from_channel_config(
                config,
                value_shape=(action_dim,),
                num_envs=num_envs,
                device=device,
                dtype=dtype,
                seed=seed,
            )
        )

    def enabled(self) -> bool:
        return self.buffer is not None

    def reset(self, env_ids: EnvIds) -> None:
        if self.buffer is None:
            return
        self.buffer.reset(env_ids)
        # A reset starts a new episode. Zero-fill its history so the previous
        # episode's last command cannot leak across the reset boundary.
        self.buffer.fill_reset(env_ids)

    def manual_seed(self, seed: int) -> None:
        if self.buffer is not None:
            self.buffer.manual_seed(seed)

    def process(
        self,
        actions: torch.Tensor,
        *,
        current_time: float,
        step_dt: float,
        change_freq: float,
        change_prob: float,
    ) -> torch.Tensor:
        if self.buffer is None:
            return actions
        delayed = self.buffer.push_and_read(actions)
        self.buffer.randomize_delay_steps_on_boundary(
            current_time=current_time,
            step_dt=step_dt,
            change_freq=change_freq,
            change_prob=change_prob,
        )
        return delayed


class ObservationDelay:
    def __init__(
        self,
        buffers: dict[tuple[str, str], TensorDelayBuffer] | None = None,
    ) -> None:
        self.buffers = buffers or {}

    @classmethod
    def from_config(
        cls,
        config: DelayChannelConfig,
        step_return: StepReturn,
        *,
        num_envs: int,
        seed: int,
    ) -> ObservationDelay:
        if not config.enabled():
            return cls()

        buffers: dict[tuple[str, str], TensorDelayBuffer] = {}
        channel_index = 0
        for category, group in step_return.observation.items():
            for key, value in group.items():
                buffers[(category, key)] = TensorDelayBuffer.from_channel_config(
                    config,
                    value_shape=tuple(value.shape[1:]),
                    num_envs=num_envs,
                    device=value.device,
                    dtype=value.dtype,
                    # Decorrelate per-channel RNG so observation channels do not
                    # share an identical delay sequence. Action delay uses the
                    # base seed; observation channels are offset by 1 + index.
                    seed=_channel_seed(seed, channel_index),
                )
                channel_index += 1
        return cls(buffers)

    def enabled(self) -> bool:
        return bool(self.buffers)

    def reset(
        self,
        env_ids: EnvIds,
        latest_observation: ObservationDict | None,
    ) -> None:
        for buffer in self.buffers.values():
            buffer.reset(env_ids)
        if latest_observation is not None:
            self.fill_reset(env_ids, latest_observation)

    def fill_reset(
        self,
        env_ids: EnvIds,
        observation: ObservationDict,
    ) -> None:
        for (category, key), buffer in self.buffers.items():
            if category not in observation or key not in observation[category]:
                continue
            buffer.fill_reset(env_ids, observation[category][key])

    def manual_seed(self, seed: int) -> None:
        for channel_index, buffer in enumerate(self.buffers.values()):
            buffer.manual_seed(_channel_seed(seed, channel_index))

    def apply(
        self,
        step_return: StepReturn,
        *,
        current_time: float,
        step_dt: float,
        change_freq: float,
        change_prob: float,
    ) -> StepReturn:
        if not self.buffers:
            return step_return

        delayed_observation: ObservationDict = {}
        for category, group in step_return.observation.items():
            delayed_observation[category] = {}
            for key, value in group.items():
                buffer = self.buffers.get((category, key))
                # Do NOT scrub non-finite observations here: the delay write runs
                # before scrub_non_finite_step_return, whose terminate-on-non-finite
                # safety net would otherwise never see (and reset) a blown-up env.
                delayed_observation[category][key] = (
                    buffer.push_and_read(value, scrub_non_finite=False)
                    if buffer is not None
                    else value
                )
        for buffer in self.buffers.values():
            buffer.randomize_delay_steps_on_boundary(
                current_time=current_time,
                step_dt=step_dt,
                change_freq=change_freq,
                change_prob=change_prob,
            )
        return StepReturn(
            delayed_observation,
            step_return.reward,
            step_return.termination,
            step_return.truncation,
            step_return.info,
        )


class DelayRuntime:
    """Own action/observation delay state for SRB env runtimes."""

    def __init__(
        self,
        *,
        cfg: object,
        delay_config: DelayRuntimeConfig,
        action_delay: ActionDelay | None,
        observation_delay: ObservationDelay,
        raw_observation: ObservationDict,
    ) -> None:
        self.cfg = cfg
        self.delay_config = delay_config
        self._action_delay = action_delay
        self._observation_delay = observation_delay
        self._raw_observation: ObservationDict | None = raw_observation
        # Env subsets whose observation-delay history must be refilled with a
        # fresh observation on the NEXT process_observation. Populated by
        # out-of-band per-env resets (e.g. ROS /envX/reset) via
        # mark_reset_reseed; consumed and cleared in process_observation.
        self._pending_reset_reseed: list[EnvIds] = []

    @classmethod
    def from_step_return(
        cls,
        cfg: object,
        step_return: StepReturn,
        *,
        action_dim: int,
        num_envs: int,
        device: torch.device | str,
        actions_enabled: bool,
        delay_config: DelayRuntimeConfig | None = None,
    ) -> DelayRuntime:
        delay_config = delay_config or DelayRuntimeConfig.from_cfg(cfg)
        action_delay = None
        if actions_enabled:
            action_delay = ActionDelay.from_config(
                delay_config.action,
                action_dim=action_dim,
                num_envs=num_envs,
                device=device,
                seed=delay_config.seed,
            )

        return cls(
            cfg=cfg,
            delay_config=delay_config,
            action_delay=action_delay,
            observation_delay=ObservationDelay.from_config(
                delay_config.observation,
                step_return,
                num_envs=num_envs,
                seed=delay_config.seed,
            ),
            raw_observation=step_return.observation,
        )

    def reset(self, env_ids: EnvIds) -> None:
        if self._action_delay is not None:
            self._action_delay.reset(env_ids)
        self._observation_delay.reset(env_ids, self._raw_observation)

    def manual_seed(self, seed: int) -> None:
        if self._action_delay is not None:
            self._action_delay.manual_seed(seed)
        self._observation_delay.manual_seed(seed)

    def process_action(
        self,
        actions: torch.Tensor,
        *,
        current_time: float,
        step_dt: float,
    ) -> torch.Tensor:
        if self._action_delay is None:
            return actions
        return self._action_delay.process(
            actions,
            current_time=current_time,
            step_dt=step_dt,
            change_freq=self.delay_config.action.change_freq,
            change_prob=self.delay_config.action.change_prob,
        )

    def process_reset_observation(
        self,
        env_ids: EnvIds,
        step_return: StepReturn,
        *,
        current_time: float,
        step_dt: float,
    ) -> StepReturn:
        self._raw_observation = step_return.observation
        self._observation_delay.fill_reset(env_ids, step_return.observation)
        return self.process_observation(
            step_return,
            current_time=current_time,
            step_dt=step_dt,
        )

    def reseed_reset_observation(
        self,
        env_ids: EnvIds,
        step_return: StepReturn,
    ) -> StepReturn:
        """Refill selected histories from fresh reset observations without advancing
        buffers.
        """
        normalized_env_ids = _normalize_delay_env_ids(
            env_ids, num_envs=int(step_return.termination.shape[0]), device="cpu"
        )
        self._observation_delay.fill_reset(normalized_env_ids, step_return.observation)
        if self._raw_observation is None:
            self._raw_observation = step_return.observation
        else:
            from .outcome_observation import splice_nested_rows

            self._raw_observation = splice_nested_rows(
                self._raw_observation,
                step_return.observation,
                normalized_env_ids,
            )
        return step_return

    @property
    def has_observation_delay(self) -> bool:
        """Whether any observation channel has an active delay buffer."""
        return self._observation_delay.enabled()

    def mark_reset_reseed(self, env_ids: EnvIds) -> None:
        """Refill selected histories on the next fresh observation after reset.

        Immediate extraction would advance task counters for every environment;
        deferred refill avoids that and prevents replaying the prior episode.
        This is a no-op when observation delay is disabled.
        """
        if not self._observation_delay.enabled():
            return
        self._pending_reset_reseed.append(env_ids)

    def process_observation(
        self,
        step_return: StepReturn,
        *,
        current_time: float,
        step_dt: float,
    ) -> StepReturn:
        self._raw_observation = step_return.observation
        if self._pending_reset_reseed:
            # Refill the deferred per-env-reset history from this fresh
            # observation before the delayed read. Merge overlapping reset
            # requests so each env's history is filled only once.
            num_envs = next(iter(self._observation_delay.buffers.values())).num_envs
            pending_env_ids = torch.unique(
                torch.cat(
                    [
                        _normalize_delay_env_ids(
                            env_ids,
                            num_envs=num_envs,
                            device="cpu",
                        )
                        for env_ids in self._pending_reset_reseed
                    ]
                )
            )
            self._observation_delay.fill_reset(
                pending_env_ids,
                step_return.observation,
            )
            self._pending_reset_reseed = []
        return self._observation_delay.apply(
            step_return,
            current_time=current_time,
            step_dt=step_dt,
            change_freq=self.delay_config.observation.change_freq,
            change_prob=self.delay_config.observation.change_prob,
        )


def resolve_delay_step_dt(env: Any) -> float:
    """Return the configured agent-step duration used by delay transforms.

    Delays are expressed in ``cfg.agent_rate``; ``env.step_dt`` derives from
    simulation timing. ``BaseEnvCfg`` requires integral agent/simulation rate
    ratios, so both durations agree, but the configured delay quantum remains
    the source of truth.
    """

    resolver = getattr(env, "_delay_step_dt", None)
    if callable(resolver):
        return float(resolver())
    agent_rate = getattr(getattr(env, "cfg", None), "agent_rate", None)
    if isinstance(agent_rate, (int, float)) and not isinstance(agent_rate, bool):
        return float(agent_rate)
    return 1.0


__all__ = [
    "ActionDelay",
    "DelayActivationDiagnostic",
    "DelayActivationPlan",
    "DelayChannelConfig",
    "DelayDiagnosticLevel",
    "DelayRuntime",
    "DelayRuntimeConfig",
    "DelayRange",
    "ObservationDelay",
    "TensorDelayBuffer",
    "delay_buffer_length",
    "delayed_history_read_indices",
    "fill_reset_history",
    "push_and_read_delay_history",
    "push_and_read_observation_delay_history",
    "plan_delay_runtime_activation",
    "randomize_delay_steps_on_boundary",
    "reset_delay_steps",
    "resolve_delay_step_dt",
]
