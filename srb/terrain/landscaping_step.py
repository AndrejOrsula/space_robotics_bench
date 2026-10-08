"""Evaluate landscaping reward, success, and metrics without Isaac imports.

The single stationary reward uses fixed oracle grading cells (MAE or excess
volume). Observable sustained success is reported at truncation; diagnostics
do not contribute to reward.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import torch

from srb.terrain.landscaping_actions import LANDSCAPING_DRIVE_ACTION
from srb.terrain.landscaping_metrics import update_stuck_counter
from srb.terrain.landscaping_success import (
    LandscapingStateQuality,
    SustainedSuccessTracker,
)


@dataclass(frozen=True, slots=True)
class LandscapingStepEvaluatorCfg:
    """Configuration needed to evaluate one landscaping step."""

    stuck_velocity_threshold: float
    stuck_command_threshold: float
    stuck_window_steps: int
    #: Select grading MAE or excess volume. The latter requires evaluators
    #: built with ``cell_size_m``.
    reward_metric: str = "grading_mae"
    #: Analytic initial excess volume (m^3) used to normalize that reward.
    excess_reference_m3: float = 1.0

    def __post_init__(self) -> None:
        if self.reward_metric not in ("grading_mae", "excess_volume"):
            raise ValueError(
                f"unknown reward_metric {self.reward_metric!r}; "
                "expected 'grading_mae' or 'excess_volume'"
            )
        if (
            not math.isfinite(self.excess_reference_m3)
            or self.excess_reference_m3 <= 0.0
        ):
            raise ValueError("excess_reference_m3 must be finite and positive")
        if isinstance(self.stuck_window_steps, bool) or self.stuck_window_steps < 1:
            raise ValueError("stuck_window_steps must be at least 1")


@dataclass(frozen=True, slots=True)
class LandscapingStepSnapshot:
    """Step inputs. Oracle maps drive reward; observable maps drive evaluation.

    Repeated observable sequence numbers indicate held frames and do not
    advance sustained success.
    """

    oracle_current_map: torch.Tensor
    oracle_target_map: torch.Tensor
    observable_current_map: torch.Tensor
    observable_target_map: torch.Tensor
    observable_sequence: torch.Tensor
    mission_pose: torch.Tensor
    base_vel_b: torch.Tensor
    base_rpy: torch.Tensor
    prev_action: torch.Tensor
    particle_velocity_magnitude: torch.Tensor
    wall_force: torch.Tensor
    truncation: torch.Tensor


@dataclass(frozen=True, slots=True)
class LandscapingStepEvaluation:
    """Reward, success, and metrics produced for one landscaping step."""

    reward_terms: dict[str, torch.Tensor]
    success: torch.Tensor
    metrics: dict[str, torch.Tensor | float]


class LandscapingStepEvaluator:
    """Evaluate oracle reward and observable success with separate trackers."""

    def __init__(
        self,
        *,
        num_envs: int,
        device: torch.device | str,
        oracle_quality: LandscapingStateQuality,
        observable_quality: LandscapingStateQuality,
        cfg: LandscapingStepEvaluatorCfg,
    ) -> None:
        self.cfg = cfg
        self._device = torch.device(device)
        self._num_envs = int(num_envs)
        self._oracle_quality = oracle_quality
        self._observable_quality = observable_quality
        self._previous_action: torch.Tensor | None = None
        self._stuck_counter = torch.zeros(
            self._num_envs, device=self._device, dtype=torch.long
        )
        window = int(observable_quality.params.sustained_frames)
        self._oracle_success_tracker = SustainedSuccessTracker(
            num_envs=self._num_envs,
            window_steps=int(oracle_quality.params.sustained_frames),
            device=self._device,
        )
        self._observable_success_tracker = SustainedSuccessTracker(
            num_envs=self._num_envs,
            window_steps=window,
            device=self._device,
        )

    def reset(self, env_ids: Sequence[int] | torch.Tensor | None = None) -> None:
        """Reset per-env evaluator state for all or selected environments."""
        if env_ids is None:
            self._stuck_counter.zero_()
            self._oracle_success_tracker.reset()
            self._observable_success_tracker.reset()
            if self._previous_action is not None:
                self._previous_action.zero_()
            return

        ids = torch.as_tensor(env_ids, device=self._device, dtype=torch.long)
        if ids.numel() == 0:
            return
        self._stuck_counter[ids] = 0
        self._oracle_success_tracker.reset(ids)
        self._observable_success_tracker.reset(ids)
        if self._previous_action is not None:
            self._previous_action[ids] = 0.0

    def evaluate(self, snapshot: LandscapingStepSnapshot) -> LandscapingStepEvaluation:
        """Evaluate reward, success, and info metrics for one step."""
        cfg = self.cfg
        previous_action = self._previous_action_for(snapshot.prev_action)
        rover_xy = snapshot.mission_pose[..., :2]

        oracle = self._oracle_quality.evaluate(
            snapshot.oracle_current_map,
            snapshot.oracle_target_map,
            rover_xy=rover_xy,
        )
        observable = self._observable_quality.evaluate(
            snapshot.observable_current_map,
            snapshot.observable_target_map,
            rover_xy=rover_xy,
        )

        # Oracle updates every policy step; observable updates on new frames.
        oracle_sustained = self._oracle_success_tracker.update(
            oracle.fraction_within_tolerance,
            threshold=self._oracle_quality.params.fraction_within_tolerance,
        )
        observable_sustained = self._observable_success_tracker.update(
            observable.fraction_within_tolerance,
            threshold=self._observable_quality.params.fraction_within_tolerance,
            sequence=snapshot.observable_sequence,
        )

        # --- Named metrics only (no reward contribution) -------------------
        action_rate = (snapshot.prev_action - previous_action).pow(2).mean(dim=-1)
        particle_chaos = torch.tanh(snapshot.particle_velocity_magnitude.squeeze(-1))
        tip_magnitude = snapshot.base_rpy[..., :2].abs().sum(-1)
        self._stuck_counter, stuck_score = update_stuck_counter(
            stuck_counter=self._stuck_counter,
            base_vel_b=snapshot.base_vel_b,
            prev_action=snapshot.prev_action[
                ...,
                LANDSCAPING_DRIVE_ACTION.linear_action_index : (
                    LANDSCAPING_DRIVE_ACTION.linear_action_index + 1
                ),
            ],
            velocity_threshold=cfg.stuck_velocity_threshold,
            command_threshold=cfg.stuck_command_threshold,
            window_steps=cfg.stuck_window_steps,
        )

        self._previous_action = snapshot.prev_action.clone().detach()
        # Success is an evaluation metric at the truncation boundary, never a
        # termination signal and never a reward term.
        success = snapshot.truncation & observable_sustained
        if self.cfg.reward_metric == "excess_volume":
            if oracle.excess_volume_m3 is None:
                raise ValueError(
                    "reward_metric='excess_volume' requires quality evaluators "
                    "built with cell_size_m"
                )
            quality_term = 1.0 / (
                1.0 + oracle.excess_volume_m3 / self.cfg.excess_reference_m3
            )
        else:
            quality_term = oracle.quality
        excess_metrics = {}
        if oracle.excess_volume_m3 is not None:
            excess_metrics["oracle_excess_volume_m3"] = oracle.excess_volume_m3
        if observable.excess_volume_m3 is not None:
            excess_metrics["observable_excess_volume_m3"] = observable.excess_volume_m3
        return LandscapingStepEvaluation(
            reward_terms={"quality": quality_term},
            success=success,
            metrics={
                **excess_metrics,
                "oracle_grading_mae_m": oracle.grading_mae_m,
                "oracle_fraction_within_tolerance": oracle.fraction_within_tolerance,
                "oracle_quality": oracle.quality,
                "oracle_local_mae_m": oracle.local_mae_m,
                "oracle_instantaneous_success": oracle.instantaneous_success,
                "oracle_sustained_success": oracle_sustained,
                "observable_grading_mae_m": observable.grading_mae_m,
                "observable_fraction_within_tolerance": (
                    observable.fraction_within_tolerance
                ),
                "observable_quality": observable.quality,
                "observable_local_mae_m": observable.local_mae_m,
                "observable_instantaneous_success": observable.instantaneous_success,
                "observable_sustained_success": observable_sustained,
                "action_rate": action_rate,
                "particle_chaos": particle_chaos,
                "rover_tip_magnitude": tip_magnitude,
                "stuck_score": stuck_score,
                "wall_force": snapshot.wall_force,
                "success_tolerance_m": self._observable_quality.params.success_tolerance_m,
                "success_fraction_threshold": (
                    self._observable_quality.params.fraction_within_tolerance
                ),
            },
        )

    def _previous_action_for(self, prev_action: torch.Tensor) -> torch.Tensor:
        if (
            self._previous_action is None
            or self._previous_action.shape != prev_action.shape
        ):
            self._previous_action = torch.zeros_like(prev_action)
        return self._previous_action


__all__ = [
    "LandscapingStepEvaluation",
    "LandscapingStepEvaluator",
    "LandscapingStepEvaluatorCfg",
    "LandscapingStepSnapshot",
]
