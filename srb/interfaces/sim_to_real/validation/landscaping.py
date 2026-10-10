"""Evaluate real landscaping from accepted heightmap frames and a verified target.

Reuses simulator grading and manifest cells; uses no oracle map and fabricates
no frames. Rejects stale or mis-calibrated data with
:class:`~srb.interfaces.sim_to_real.core.task_evaluator.RealSessionAbort`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

from srb.interfaces.sim_to_real.core.hardware import DeploymentCapability
from srb.interfaces.sim_to_real.core.task_evaluator import (
    CalibrationMismatchAbort,
    DeploymentIdentity,
    ManifestMismatchAbort,
    RealSessionAbort,
    RealTaskEvaluation,
    SensorIncompleteAbort,
    SensorStaleAbort,
)
from srb.terrain.landscaping_contract import (
    LANDSCAPING_IO_CONTRACT,
    LandscapingIOContract,
    TargetMapManifest,
    load_crater_target_manifest,
    verify_target_manifest,
)
from srb.terrain.landscaping_external_heightmap import (
    ExternalHeightmapBatch,
    ExternalHeightmapError,
    ExternalHeightmapStaleError,
    validate_external_heightmap_batch,
    validate_external_heightmap_target,
    validate_policy_time,
)
from srb.terrain.landscaping_success import (
    LandscapingStateQuality,
    StateQualityParams,
    SustainedSuccessTracker,
)

#: Recorded in model artifacts; deployment rejects a different policy.
LANDSCAPING_NORMALIZATION_POLICY = (
    "metric observation units; upstream Dreamer symlog transform; no running normalizer"
)


@dataclass(frozen=True, slots=True)
class LandscapingAcceptedFrame:
    """Accepted frame, policy read time, and calibration attestation.

    ``batch`` has one row per environment; deployments use one. Read time uses
    the batch clock, and ``calibration_version`` identifies the adapter transform.
    """

    batch: ExternalHeightmapBatch
    policy_time_s: float
    calibration_version: str


def landscaping_session_abort(exc: BaseException) -> RealSessionAbort | None:
    """Map heightmap failures consistently; return ``None`` for other errors."""

    if isinstance(exc, ExternalHeightmapStaleError):
        abort: RealSessionAbort = SensorStaleAbort(str(exc))
    elif isinstance(exc, ExternalHeightmapError):
        # These failures mean no complete frame exists for this step.
        abort = SensorIncompleteAbort(str(exc))
    else:
        return None
    abort.__cause__ = exc
    return abort


class LandscapingRealEvaluator:
    """Evaluate observable landscaping metrics for real sessions.

    Claims only ``landscaping.observable_task_evaluator``; hardware adapters
    own mapper and robot capabilities.
    """

    DEPLOYMENT_CLAIMS: frozenset[DeploymentCapability] = frozenset(
        {DeploymentCapability.LANDSCAPING_OBSERVABLE_TASK_EVALUATOR}
    )

    def __init__(
        self,
        *,
        horizon_steps: int,
        manifest: TargetMapManifest | None = None,
        contract: LandscapingIOContract = LANDSCAPING_IO_CONTRACT,
        local_radius_m: float = 1.0,
        device: torch.device | str = "cpu",
    ) -> None:
        if horizon_steps < 1:
            raise ValueError("horizon_steps must be positive")
        self._horizon_steps = int(horizon_steps)
        self._contract = contract
        self._manifest = (
            manifest if manifest is not None else load_crater_target_manifest()
        )
        if self._manifest.io_schema_fingerprint != contract.fingerprint():
            raise ManifestMismatchAbort(
                "target manifest was written against IO-schema fingerprint "
                f"{self._manifest.io_schema_fingerprint!r}, deployed contract is "
                f"{contract.fingerprint()!r}"
            )
        # Full hash/geometry/frame/datum verification of the static target.
        self._materialization = verify_target_manifest(
            self._manifest, contract=contract
        )
        validate_external_heightmap_target(
            self._materialization.target,
            contract=contract,
        )
        self._device = torch.device(device)
        self._target = torch.from_numpy(
            np.array(self._materialization.target, dtype=np.float32)
        ).to(self._device)

        geometry = contract.geometry
        grading = np.asarray(self._materialization.grading_indices)
        centres_x = np.asarray(geometry.cell_centres(0))
        centres_y = np.asarray(geometry.cell_centres(1))
        centres = np.stack(
            [centres_x[grading[:, 0]], centres_y[grading[:, 1]]], axis=-1
        )
        self._quality = LandscapingStateQuality(
            params=StateQualityParams(
                success_tolerance_m=float(self._manifest.success.success_tolerance_m),
                fraction_within_tolerance=float(
                    self._manifest.success.fraction_within_tolerance
                ),
                sustained_frames=int(self._manifest.success.sustained_frames),
                manifest_sha256=self._manifest.manifest_sha256,
                io_schema_fingerprint=contract.fingerprint(),
            ),
            map_shape=geometry.source_shape,
            grading_indices=torch.from_numpy(grading.copy()),
            cell_centres_xy=torch.from_numpy(centres.astype(np.float32)),
            local_radius_m=local_radius_m,
            # The mound gate uses excess volume in simulation and deployment;
            # pass the simulator's cell-size argument so it is observable here.
            cell_size_m=float(geometry.cell_size_m),
            device=self._device,
        )
        self._tracker = SustainedSuccessTracker(
            num_envs=1,
            window_steps=int(self._manifest.success.sustained_frames),
            device=self._device,
        )
        self._central_roi_indices = self._build_central_roi_indices()
        # The static ROI indices are reused to avoid rebuilding the tensor each step.
        self._central_roi_index_tensor = torch.from_numpy(
            self._central_roi_indices.copy()
        ).to(self._device)
        self._max_age_s: float = 0.0
        self._last_metrics: dict[str, float] | None = None
        self._reset_episode_state()

    # -- identity ---------------------------------------------------------

    @property
    def deployment_capabilities(self) -> frozenset[DeploymentCapability]:
        return self.DEPLOYMENT_CLAIMS

    @property
    def horizon_steps(self) -> int:
        return self._horizon_steps

    @property
    def manifest(self) -> TargetMapManifest:
        return self._manifest

    @property
    def identity(self) -> DeploymentIdentity:
        return DeploymentIdentity(
            task_id=self._manifest.task_id,
            io_schema_fingerprint=self._contract.fingerprint(),
            target_manifest_sha256=self._manifest.manifest_sha256,
            reset_layout_sha256=self._manifest.reset_layout_sha256,
            normalization_policy=LANDSCAPING_NORMALIZATION_POLICY,
        )

    @property
    def grading_cell_count(self) -> int:
        return self._quality.grading_cell_count

    @property
    def central_roi_cell_count(self) -> int:
        return int(self._central_roi_indices.shape[0])

    # -- evaluation -------------------------------------------------------

    def reset(self) -> None:
        """Start an episode and clear prior episode state."""

        self._tracker.reset()
        self._reset_episode_state()

    def validate_frame(self, frame: LandscapingAcceptedFrame) -> None:
        """Validate a frame without advancing metrics or trackers."""

        if not isinstance(frame, LandscapingAcceptedFrame):
            raise TypeError(
                "landscaping evaluation needs a LandscapingAcceptedFrame, got "
                f"{type(frame).__name__}"
            )
        if frame.batch.batch_size != 1:
            raise ValueError(
                "a real landscaping deployment evaluates exactly one rover, "
                f"got a batch of {frame.batch.batch_size}"
            )
        try:
            validate_external_heightmap_batch(
                frame.batch,
                contract=self._contract,
                require_canonical_geometry=True,
                require_contract_metadata=False,
            )
        except ExternalHeightmapError as exc:
            raise SensorIncompleteAbort(str(exc)) from exc
        self._verify_frame_identity(frame)
        self._frame_age_s(frame)

    def evaluate(
        self, *, frame: LandscapingAcceptedFrame, action: np.ndarray
    ) -> RealTaskEvaluation:
        """Grade an accepted frame against the verified target."""

        self.validate_frame(frame)
        batch = frame.batch
        age_s = self._frame_age_s(frame)

        current = torch.from_numpy(np.array(batch.elevation, dtype=np.float32)).to(
            self._device
        )
        rover_xy = torch.from_numpy(
            np.array(batch.mission_pose[:, :2], dtype=np.float32)
        ).to(self._device)
        result = self._quality.evaluate(current, self._target, rover_xy=rover_xy)
        sustained = self._tracker.update(
            result.fraction_within_tolerance,
            threshold=self._quality.params.fraction_within_tolerance,
            sequence=torch.from_numpy(np.asarray(batch.sequence, dtype=np.int64)).to(
                self._device
            ),
        )
        central_roi_mae = self._central_roi_mae(current)
        action_delta = self._action_delta(action)

        if result.excess_volume_m3 is None:
            raise SensorIncompleteAbort(
                "the observable quality evaluator produced no excess volume; "
                "it was constructed without a cell size, so the mound gate "
                "metric cannot be reported for this session"
            )
        metrics = {
            "grading_mae_m": float(result.grading_mae_m[0]),
            "excess_volume_m3": float(result.excess_volume_m3[0]),
            "fraction_within_tolerance": float(result.fraction_within_tolerance[0]),
            "stationary_quality": float(result.quality[0]),
            "central_roi_mae_m": central_roi_mae,
            "local_mae_m": float(result.local_mae_m[0]),
            "instantaneous_success": float(result.instantaneous_success[0]),
            "sustained_success": float(sustained[0]),
            "map_age_s": age_s,
            "action_delta": action_delta,
        }
        self._steps += 1
        self._action_delta_sum += action_delta
        self._max_age_s = max(self._max_age_s, age_s)
        self._last_metrics = metrics
        return RealTaskEvaluation(
            reward=float(result.quality[0]),
            success=bool(sustained[0]),
            metrics=metrics,
        )

    def final_info(self) -> dict[str, Any]:
        """Return final grading evidence for the session report."""

        if self._last_metrics is None:
            raise SensorIncompleteAbort(
                "no accepted external frame was evaluated in this episode; "
                "there is no observable evidence to report"
            )
        last = self._last_metrics
        mission_frame = self._manifest.mission_frame
        return {
            # Grading and diagnostics, all from accepted frames only.
            "final_grading_mae_m": last["grading_mae_m"],
            # The mound task's gate metric, measured on accepted frames only.
            "final_excess_volume_m3": last["excess_volume_m3"],
            "final_fraction_within_tolerance": last["fraction_within_tolerance"],
            "final_stationary_quality": last["stationary_quality"],
            "diagnostic_central_roi_mae_m": last["central_roi_mae_m"],
            "diagnostic_local_mae_m": last["local_mae_m"],
            "action_smoothness": self._action_delta_sum / max(self._steps, 1),
            "map_age_s": last["map_age_s"],
            "max_map_age_s": self._max_age_s,
            "sustained_success": bool(last["sustained_success"]),
            "instantaneous_success": bool(last["instantaneous_success"]),
            "evaluated_steps": self._steps,
            # Identity: four distinct hashes plus the calibration attestation.
            "task_id": self._manifest.task_id,
            "io_schema_fingerprint": self._contract.fingerprint(),
            "target_manifest_sha256": self._manifest.manifest_sha256,
            "target_map_sha256": self._manifest.target_map_sha256,
            "reset_layout_sha256": self._manifest.reset_layout_sha256,
            "calibration_version": mission_frame.calibration_version,
            "mission_frame_id": mission_frame.mission_frame_id,
            "elevation_datum_id": mission_frame.elevation_datum_id,
            "normalization_policy": LANDSCAPING_NORMALIZATION_POLICY,
            # Saved success parameters the grading above was judged against.
            "grading_cell_count": self.grading_cell_count,
            "central_roi_cell_count": self.central_roi_cell_count,
            "success_tolerance_m": float(self._manifest.success.success_tolerance_m),
            "success_fraction_threshold": float(
                self._manifest.success.fraction_within_tolerance
            ),
            "sustained_frames": int(self._manifest.success.sustained_frames),
        }

    # -- internals --------------------------------------------------------

    def _reset_episode_state(self) -> None:
        self._steps = 0
        self._action_delta_sum = 0.0
        self._max_age_s = 0.0
        self._last_metrics = None
        self._previous_action: np.ndarray | None = None

    def _build_central_roi_indices(self) -> np.ndarray:
        geometry = self._contract.geometry
        r0, c0 = geometry.roi_index_start
        r1, c1 = geometry.roi_index_stop
        rows = np.arange(r0, r1, dtype=np.int64)
        cols = np.arange(c0, c1, dtype=np.int64)
        width = int(geometry.source_shape[1])
        flat = (rows[:, None] * width + cols[None, :]).reshape(-1)
        flat.setflags(write=False)
        return flat

    def _central_roi_mae(self, current: torch.Tensor) -> float:
        indices = self._central_roi_index_tensor
        cur = current.reshape(current.shape[0], -1)[:, indices]
        des = self._target.reshape(1, -1)[:, indices]
        return float((cur - des).abs().mean(dim=-1)[0])

    def _action_delta(self, action: np.ndarray) -> float:
        flat = np.asarray(action, dtype=np.float32).reshape(-1)
        previous = (
            np.zeros_like(flat)
            if self._previous_action is None
            else self._previous_action
        )
        if previous.shape != flat.shape:
            previous = np.zeros_like(flat)
        delta = float(np.mean(np.square(flat - previous)))
        self._previous_action = flat.copy()
        return delta

    def _verify_frame_identity(self, frame: LandscapingAcceptedFrame) -> None:
        batch = frame.batch
        mission_frame = self._manifest.mission_frame
        if batch.manifest_hashes[0] != self._manifest.manifest_sha256:
            raise ManifestMismatchAbort(
                "accepted frame was produced against target manifest "
                f"{batch.manifest_hashes[0]!r}, deployed manifest is "
                f"{self._manifest.manifest_sha256!r}"
            )
        if frame.calibration_version != mission_frame.calibration_version:
            raise CalibrationMismatchAbort(
                "external source attests calibration "
                f"{frame.calibration_version!r}, deployed manifest declares "
                f"{mission_frame.calibration_version!r}"
            )
        if batch.mission_frame_id != mission_frame.mission_frame_id:
            raise CalibrationMismatchAbort(
                f"accepted frame is in mission frame {batch.mission_frame_id!r}, "
                f"deployed manifest declares {mission_frame.mission_frame_id!r}"
            )
        if batch.elevation_datum_id != mission_frame.elevation_datum_id:
            raise CalibrationMismatchAbort(
                "accepted frame uses elevation datum "
                f"{batch.elevation_datum_id!r}, deployed manifest declares "
                f"{mission_frame.elevation_datum_id!r}"
            )
        if batch.clock != self._contract.freshness.clock:
            raise CalibrationMismatchAbort(
                f"accepted frame is stamped on clock {batch.clock!r}, deployed "
                f"contract declares {self._contract.freshness.clock!r}"
            )

    def _frame_age_s(self, frame: LandscapingAcceptedFrame) -> float:
        try:
            ages = validate_policy_time(
                frame.batch,
                frame.policy_time_s,
                contract=self._contract,
                require_canonical_geometry=True,
                require_contract_metadata=False,
            )
        except ExternalHeightmapStaleError as exc:
            age = float(exc.ages_s[0])
            raise SensorStaleAbort(
                f"accepted frame age {age:.3f} s exceeds the hard stale cutoff "
                f"{self._contract.freshness.stale_cutoff_s:.3f} s; a stale map "
                "is a safe-stop and session abort, never normal data"
            ) from exc
        except ExternalHeightmapError as exc:
            try:
                age = float(frame.policy_time_s) - float(frame.batch.timestamp_s[0])
            except (TypeError, ValueError):
                age = float("nan")
            raise SensorIncompleteAbort(
                f"accepted frame age {age} s is not a valid synchronized age "
                "(map/pose/velocity must share one accepted source timestamp "
                "at or before the policy read time)"
            ) from exc
        return float(ages[0])


__all__ = [
    "LANDSCAPING_NORMALIZATION_POLICY",
    "LandscapingAcceptedFrame",
    "LandscapingRealEvaluator",
    "landscaping_session_abort",
]
