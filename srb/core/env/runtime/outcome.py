from __future__ import annotations

import json
import math
from collections.abc import Callable
from dataclasses import dataclass
from numbers import Integral, Real
from typing import Any

import torch

from srb._typing.step_return import StepReturn
from srb.core.reward.outcome import RewardOutcomeRuntime

from .episode_info import EPISODE_METRICS_KEY, EPISODE_SUCCESS_KEY
from .observation_layout import ObservationLayout
from .outcome_core import StepOutcomeCore, process_step_outcome_core
from .outcome_observation import (
    ObservationBuilder,
    build_reset_observation,
    build_step_observation,
    final_observation,
)
from .step_return import validate_step_return

NonFiniteWarning = Callable[[torch.Tensor, tuple[str, ...]], None]


@dataclass(frozen=True)
class ResetOutcome:
    step_return: StepReturn
    observation: dict[str, torch.Tensor]
    info: dict[str, Any]


@dataclass(frozen=True)
class StepOutcome:
    step_return: StepReturn
    observation: dict[str, torch.Tensor]
    reward: torch.Tensor
    terminated: torch.Tensor
    truncated: torch.Tensor
    info: dict[str, Any]
    extras: dict[str, Any]
    reset_mask: torch.Tensor
    reset_ids: torch.Tensor
    final_observation: dict[str, torch.Tensor] | None
    invalid_envs: torch.Tensor
    invalid_keys: tuple[str, ...]


def process_reset_outcome(
    step_return: StepReturn,
    *,
    num_envs: int,
    observation_layout: ObservationLayout | None = None,
    device: torch.device | str | None = None,
) -> ResetOutcome:
    """Validate and flatten a reset step return.

    ``device`` enforces the residency invariant. It defaults to
    ``None`` -- i.e. unenforced -- because this helper predates the invariant
    and is a public seam for out-of-tree callers. It has no in-tree caller of
    its own: the owned reset path goes through
    ``process_step_outcome_core``, which always passes the environment's
    device.
    """
    validate_step_return(step_return, num_envs=num_envs, device=device)
    observation = build_reset_observation(
        step_return,
        observation_layout=observation_layout,
    )
    return ResetOutcome(
        step_return=step_return,
        observation=observation,
        info=dict(step_return.info or {}),
    )


def process_step_outcome(
    step_return: StepReturn,
    *,
    num_envs: int,
    device: torch.device | str,
    include_reward_terms: bool = False,
    reward_runtime: RewardOutcomeRuntime | None = None,
    observation_layout: ObservationLayout | None = None,
    observation_builder: ObservationBuilder | None = None,
) -> StepOutcome:
    core = process_step_outcome_core(
        step_return,
        num_envs=num_envs,
        device=device,
        include_reward_terms=include_reward_terms,
        reward_runtime=reward_runtime,
    )

    observation = build_step_observation(
        core.step_return,
        observation_layout=observation_layout,
        observation_builder=observation_builder,
    )

    return StepOutcome(
        step_return=core.step_return,
        observation=observation,
        reward=core.reward,
        terminated=core.terminated,
        truncated=core.truncated,
        info=core.info,
        extras=core.extras,
        reset_mask=core.reset_mask,
        reset_ids=core.reset_ids,
        final_observation=final_observation(observation, core.reset_ids),
        invalid_envs=core.invalid_envs,
        invalid_keys=core.invalid_keys,
    )


class DirectStepOutcomeAdapter:
    """Cache one outcome for DirectEnv's split reward, done, and observation hooks."""

    def __init__(
        self,
        *,
        num_envs: int,
        device: torch.device | str,
        reward_runtime: RewardOutcomeRuntime,
        observation_builder: ObservationBuilder,
        warn_non_finite: NonFiniteWarning | None = None,
    ) -> None:
        self._num_envs = num_envs
        self._device = device
        self._reward_runtime = reward_runtime
        self._observation_builder = observation_builder
        self._warn_non_finite = warn_non_finite
        self._outcome: StepOutcomeCore | None = None
        self._observation: dict[str, torch.Tensor] | None = None
        self._episode_return = torch.zeros(
            num_envs,
            dtype=torch.float32,
            device=device,
        )
        # Envs whose episode ended at the cached outcome and were already
        # popped. Their next episode has not stepped yet, so a second pop
        # before ``begin_step`` (an explicit reset) must not replay that row.
        self._consumed_env_ids: set[int] = set()

    @property
    def has_outcome(self) -> bool:
        return self._outcome is not None

    def clear(self) -> None:
        self._outcome = None
        self._observation = None
        self._episode_return.zero_()
        self._consumed_env_ids.clear()

    def begin_step(
        self,
        step_return: StepReturn,
        *,
        include_reward_terms: bool = False,
    ) -> StepOutcomeCore:
        outcome = process_step_outcome_core(
            step_return,
            num_envs=self._num_envs,
            device=self._device,
            include_reward_terms=include_reward_terms,
            reward_runtime=self._reward_runtime,
        )
        self._outcome = outcome
        self._observation = None
        self._consumed_env_ids.clear()
        self._episode_return += outcome.reward.to(device=self._episode_return.device)
        if outcome.invalid_keys and self._warn_non_finite is not None:
            self._warn_non_finite(outcome.invalid_envs, outcome.invalid_keys)
        return outcome

    def dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        outcome = self._require_outcome()
        return outcome.terminated, outcome.truncated

    def reward(self) -> torch.Tensor:
        return self._require_outcome().reward

    def extras(self) -> dict[str, Any]:
        return self._require_outcome().extras

    def observation(
        self,
        *,
        event_observations: dict[str, torch.Tensor] | None = None,
    ) -> dict[str, torch.Tensor]:
        if self._observation is None:
            step_return = self._require_outcome().step_return
            if event_observations is not None:
                self._observation = self._observation_builder(
                    step_return,
                    event_observations=event_observations,
                )
            else:
                self._observation = self._observation_builder(step_return)
        return self._observation

    def pop_episode_recording_outcomes(
        self,
        env_ids: Any,
    ) -> dict[int, dict[str, Any]]:
        """Return per-env recording metadata for reset episodes and clear totals.

        An env reset without an env done at the cached outcome (an explicit
        reset of an in-progress episode) is reported as truncated: the
        operator or collector cut it. An env already popped at this outcome
        has started a new episode that has not stepped, so it reports an
        empty truncated episode instead of replaying the consumed row.
        """
        outcome = self._require_outcome()
        normalized_env_ids = tuple(
            _normalize_env_ids(env_ids, num_envs=self._episode_return.numel())
        )
        outputs: dict[int, dict[str, Any]] = {}
        for env_id in normalized_env_ids:
            if env_id in self._consumed_env_ids:
                outputs[env_id] = {
                    "success_flag": 0,
                    "reward_summary": json.dumps(
                        {
                            "total": 0.0,
                            "final_reward": 0.0,
                            "terms": {},
                            "terminated": False,
                            "truncated": True,
                        },
                        sort_keys=True,
                        separators=(",", ":"),
                        allow_nan=False,
                    ),
                }
                continue
            terminated = _bool_at(outcome.terminated, env_id)
            truncated = _bool_at(outcome.truncated, env_id)
            summary = {
                "total": _float_at(self._episode_return, env_id),
                "final_reward": _float_at(outcome.reward, env_id),
                "terms": {
                    name: _float_at(value, env_id)
                    for name, value in outcome.step_return.reward.items()
                },
                "terminated": terminated,
                "truncated": truncated or not terminated,
            }
            metrics = _episode_metrics_at(outcome.info, env_id)
            if metrics:
                summary["metrics"] = metrics
            outputs[env_id] = {
                "success_flag": int(
                    EPISODE_SUCCESS_KEY in outcome.info
                    and _bool_at(outcome.info[EPISODE_SUCCESS_KEY], env_id)
                ),
                "reward_summary": json.dumps(
                    summary,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ),
            }
        self._consumed_env_ids.update(normalized_env_ids)
        if normalized_env_ids:
            indices = torch.tensor(
                normalized_env_ids,
                dtype=torch.long,
                device=self._episode_return.device,
            )
            self._episode_return[indices] = 0.0
        return outputs

    def reset_episode_return(self, env_ids: Any) -> None:
        """Zero accumulated episode return for ``env_ids`` on episode reset.

        ``pop_episode_recording_outcomes`` only zeroes when an episode recorder is
        configured; this keeps ``_episode_return`` bounded on the no-recorder path.
        """
        normalized_env_ids = _normalize_env_ids(
            env_ids, num_envs=self._episode_return.numel()
        )
        if not normalized_env_ids:
            return
        indices = torch.tensor(
            normalized_env_ids,
            dtype=torch.long,
            device=self._episode_return.device,
        )
        self._episode_return[indices] = 0.0

    def _require_outcome(self) -> StepOutcomeCore:
        if self._outcome is None:
            raise RuntimeError("DirectStepOutcomeAdapter has no active step outcome")
        return self._outcome


def _normalize_env_ids(env_ids: Any, *, num_envs: int) -> list[int]:
    if env_ids is None:
        return list(range(num_envs))
    if torch.is_tensor(env_ids):
        return [int(v) for v in env_ids.detach().cpu().flatten().tolist()]
    return [int(v) for v in env_ids]


def _env_value(value: Any, env_id: int) -> Any:
    if torch.is_tensor(value):
        if value.ndim == 0:
            return value.detach().cpu().item()
        selected = value[env_id].detach().cpu()
        if selected.numel() == 1:
            return selected.item()
        return selected.tolist()
    if isinstance(value, (list, tuple)):
        if not value:
            return None
        if len(value) > env_id:
            return value[env_id]
    return value


def _float_at(value: Any, env_id: int) -> float:
    selected = _env_value(value, env_id)
    try:
        return float(selected)
    except (TypeError, ValueError):
        return 0.0


def _bool_at(value: Any, env_id: int) -> bool:
    return bool(_env_value(value, env_id))


def _episode_metrics_at(info: dict[str, Any], env_id: int) -> dict[str, Any]:
    raw_metrics = info.get(EPISODE_METRICS_KEY)
    if not isinstance(raw_metrics, dict):
        return {}
    return {
        str(name): _json_safe(_env_value(value, env_id))
        for name, value in raw_metrics.items()
    }


def _json_safe(value: Any) -> Any:
    if torch.is_tensor(value):
        if value.ndim == 0:
            return value.detach().cpu().item()
        return value.detach().cpu().tolist()
    if isinstance(value, bool):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_json_safe(item) for item in value]
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    try:
        json.dumps(value)
    except TypeError:
        return str(value)
    return value


__all__ = [
    "DirectStepOutcomeAdapter",
    "NonFiniteWarning",
    "ObservationBuilder",
    "ResetOutcome",
    "StepOutcomeCore",
    "StepOutcome",
    "process_reset_outcome",
    "process_step_outcome",
]
