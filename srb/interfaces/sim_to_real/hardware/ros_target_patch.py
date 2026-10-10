"""Sample a static world target map into a robot-centric heightmap patch.

Loads an ``N x N`` float32 map at ``start()``, tracks ``world_frame ->
base_frame``, then samples the map at body-frame cell centres and converts
world elevations to attach-frame ``+z``. The ``proprio_dyn/target_patch_local``
observation has shape ``(ny, nx)`` and follows the sim heightmap convention:
rows point forward, columns lateral, and out-of-map coordinates clamp to the
edge. Failed TF lookups preserve the last observation. The map is static for
the mission and is not reloaded on reset; ROS imports are deferred to start."""

from __future__ import annotations

import math
import os
import time
from collections.abc import Callable, Sequence

import gymnasium
import numpy as np
from pydantic import Field

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.terrain.landscaping_heightmap import (
    DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY,
    LandscapingHeightmapGeometry,
    sample_single_global_map_at_heightmap_np,
    world_patch_heights_to_attach_z_np,
)
from srb.terrain.landscaping_observations import (
    LANDSCAPING_TARGET_PATCH_OBSERVATION,
    landscaping_target_patch_zero_observation,
)
from srb.terrain.landscaping_pose import (
    quat_xyzw_to_yaw,
)
from srb.utils import logging
from srb.utils.failures import HealthEvent, HealthSeverity, HealthStatus
from srb.utils.resource_budget import MAX_TARGET_MAP_BYTES, inspect_npy_array


class RosTargetPatchInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for the static target-patch hardware interface."""

    name: str = "ros_target_patch"
    """Hardware interface instance name (inherited slot)."""

    target_map_path: str = ""
    """Filesystem path to a 2D ``(N, N)`` float32 numpy array saved via
    ``np.save``. ``start()`` asserts the file exists and loads it. The array
    is indexed as ``target_map[x_idx, y_idx]`` where the first axis maps to
    world +x and the second to world +y. Cell values are world-frame
    elevations in metres, using the same vertical datum as ``world_frame``."""
    target_map_extent_m: float = Field(default=4.0, gt=0.0, allow_inf_nan=False)
    """Physical side length the loaded map spans, in metres (square)."""
    target_map_center_world_xy: tuple[float, float] = (0.0, 0.0)
    """World-frame coordinate of the loaded map's centre cell."""

    world_frame: str = "world"
    """Mocap world (fixed) frame id (parent of the tf2 lookup)."""
    base_frame: str = "base_link"
    """Robot body frame id tracked by the mocap (child of the tf2 lookup)."""
    tf_timeout_duration: float = 0.05
    """Max seconds to wait when querying tf2."""
    max_tf_age_s: float = Field(default=1.0, gt=0.0, allow_inf_nan=False)
    """Freshness gate: when the last successful tf lookup is older than this,
    the interface reports unhealthy so the stale target patch cannot silently
    feed the policy or the evaluator through a dispatch."""

    patch_nx: int = DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.nx
    """Lateral cells of the emitted patch (must match ``LANDSCAPING_HEIGHTMAP_SPEC.nx``)."""
    patch_ny: int = DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.ny
    """Forward rows of the emitted patch (must match ``LANDSCAPING_HEIGHTMAP_SPEC.ny``)."""
    patch_cell_size_m: float = DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.cell_size_m
    """Size of each patch cell, in metres (must match ``LANDSCAPING_HEIGHTMAP_SPEC.cell_size_m``)."""
    patch_forward_offset_m: float = (
        DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.forward_offset_m
    )
    """Body-frame +x offset of the patch origin, in metres (must match
    ``LANDSCAPING_HEIGHTMAP_SPEC.forward_offset_m``)."""
    patch_lateral_offset_m: float = (
        DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.lateral_offset_m
    )
    """Body-frame +y offset of the patch origin, in metres (must match
    ``LANDSCAPING_HEIGHTMAP_SPEC.lateral_offset_m``)."""


class RosTargetPatchInterface(HardwareInterface):
    """Emits ``proprio_dyn/target_patch_local`` by bilinear-sampling a static
    world-frame target heightmap at ego-centric cell centres."""

    cfg: RosTargetPatchInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (
        (
            LANDSCAPING_TARGET_PATCH_OBSERVATION.key,
            LANDSCAPING_TARGET_PATCH_OBSERVATION.name,
        ),
    )

    def __init__(
        self,
        cfg: RosTargetPatchInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        cfg = cfg or RosTargetPatchInterfaceCfg()
        self._clock = clock
        self._last_tf_s: float | None = None
        super().__init__(cfg)

        self._geometry = LandscapingHeightmapGeometry(
            nx=int(self.cfg.patch_nx),
            ny=int(self.cfg.patch_ny),
            cell_size_m=float(self.cfg.patch_cell_size_m),
            forward_offset_m=float(self.cfg.patch_forward_offset_m),
            lateral_offset_m=float(self.cfg.patch_lateral_offset_m),
        )

        # Output buffer returned via the ``observation`` property. Zero-init
        # until the first successful sync()/reset() replaces it.
        self.obs: dict[str, np.ndarray] = landscaping_target_patch_zero_observation(
            geometry=self._geometry
        )

        # Static target map (populated in ``start()``).
        self._target_map: np.ndarray | None = None

        # ROS handles (set in ``start``).
        self.tf_buffer = None
        self._tf_listener = None
        self._tf_timeout = None

        # Pre-compute ego-centric cell-centre coordinates in body frame.
        # Rows = +x (forward), cols = +y (lateral). Matches the sim helper
        # torch heightmap helper byte-for-byte.
        self._cells_body_xy: np.ndarray = self._geometry.body_xy_grid_np()

    # ----------------------------------------------------------------- lifecycle

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener

        # --- Load the static target map -------------------------------------
        target_map_path = (
            self.cfg.target_map_path
            or os.environ.get("SRB_TARGET_PATCH_MAP_PATH", "")
            or os.environ.get("SRB_TARGET_MAP_PATH", "")
        )
        if not target_map_path:
            raise ValueError(
                f"[{self.name}] cfg.target_map_path must be non-empty "
                "or SRB_TARGET_PATCH_MAP_PATH must point to a target map"
            )
        if not os.path.isfile(target_map_path):
            raise FileNotFoundError(
                f"[{self.name}] target_map_path does not exist: {target_map_path}"
            )
        # A resource ceiling applies BEFORE allocation. `target_map_path`
        # comes from `SRB_TARGET_PATCH_MAP_PATH`, i.e. outside the process,
        # and `np.load` allocates whatever the header declares.
        # `inspect_npy_array` validates the header and the exact payload size
        # without materialising the array.
        inspect_npy_array(
            target_map_path,
            resource=f"[{self.name}] target map",
            max_array_bytes=MAX_TARGET_MAP_BYTES,
        )
        target_map = np.load(target_map_path).astype(np.float32)
        if target_map.ndim != 2:
            raise ValueError(
                f"[{self.name}] expected 2D target map, got shape {target_map.shape}"
            )
        if target_map.shape[0] != target_map.shape[1]:
            raise ValueError(
                f"[{self.name}] expected square target map, got shape {target_map.shape}"
            )
        if target_map.shape[0] == 0:
            raise ValueError(f"[{self.name}] target map must have non-zero side length")
        if not np.all(np.isfinite(target_map)):
            raise ValueError(f"[{self.name}] target map contains non-finite values")
        self._target_map = target_map

        # --- tf2 listener ---------------------------------------------------
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self.ros_node)
        self._tf_timeout = Duration(
            seconds=math.floor(self.cfg.tf_timeout_duration),
            nanoseconds=int((self.cfg.tf_timeout_duration % 1) * 1e9),
        )

        logging.info(
            f"[{self.name}] Loaded target map shape={target_map.shape} "
            f"extent={self.cfg.target_map_extent_m} m "
            f"centre={self.cfg.target_map_center_world_xy}; "
            f"listening for tf: {self.cfg.world_frame} -> "
            f"{self.cfg.base_frame}"
        )

    def close(self):
        super().close()
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer.clear()
            self.tf_buffer = None

    def reset(self):
        super().reset()
        # Re-zero the obs buffer; do NOT re-load the target map (it is static
        # for the whole mission).
        self.obs.update(
            landscaping_target_patch_zero_observation(geometry=self._geometry)
        )
        # The zeroed observation is not backed by a TF sample; stay unhealthy
        # until the next successful lookup.
        self._last_tf_s = None

    # -------------------------------------------------------------- main stepping

    def _tf_age_s(self) -> float | None:
        if self._last_tf_s is None:
            return None
        return max(0.0, self._clock() - self._last_tf_s)

    @staticmethod
    def _stamp_seconds(stamp: object) -> float | None:
        if stamp is None:
            return None
        if hasattr(stamp, "nanoseconds"):
            try:
                value = float(getattr(stamp, "nanoseconds")) / 1e9
            except (TypeError, ValueError, OverflowError):
                return None
            return value if math.isfinite(value) else None
        try:
            sec = float(getattr(stamp, "sec"))
            nanosec = float(getattr(stamp, "nanosec", 0.0))
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        value = sec + nanosec / 1e9
        return value if math.isfinite(value) else None

    def health(self) -> HealthEvent:
        age_s = self._tf_age_s()
        if age_s is None:
            state, message = "pre_message", "waiting for the first tf lookup"
        elif age_s > self.cfg.max_tf_age_s:
            state, message = "stale", "target patch pose is stale"
        else:
            state, message = "healthy", "target patch pose is fresh"
        healthy = state == "healthy"
        return HealthEvent(
            status=HealthStatus.HEALTHY if healthy else HealthStatus.UNHEALTHY,
            severity=HealthSeverity.INFO if healthy else HealthSeverity.ERROR,
            component=self.name,
            code=f"{self.name}.{state}",
            message=message,
            details={
                "state": state,
                "ready": healthy,
                "age_ms": None if age_s is None else age_s * 1000.0,
                "max_tf_age_ms": self.cfg.max_tf_age_s * 1000.0,
            },
        )

    def sync(self):
        super().sync()
        if self.tf_buffer is None or self._target_map is None:
            return

        from rclpy.time import Time

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.cfg.world_frame,
                self.cfg.base_frame,
                Time(),
                self._tf_timeout,
            )
        except Exception as e:  # noqa: BLE001 - tf not ready yet is normal at startup
            # Hold last-good obs to avoid flicker during short tf dropouts;
            # ``health()`` turns unhealthy once the hold exceeds max_tf_age_s.
            logging.debug(
                f"[{self.name}] tf2 lookup {self.cfg.world_frame} -> "
                f"{self.cfg.base_frame} failed: {e}"
            )
            return

        stamp_s = self._stamp_seconds(
            getattr(getattr(tf_stamped, "header", None), "stamp", None)
        )
        # A zero stamp is tf2's static transform, valid at every time; only
        # stamped (dynamic) samples are aged against ROS time.
        if stamp_s is not None and stamp_s != 0.0:
            try:
                ros_now_s = self._stamp_seconds(self.ros_node.get_clock().now())
            except (AttributeError, OSError, RuntimeError, TypeError, ValueError):
                ros_now_s = None
            stamp_age_s = None if ros_now_s is None else ros_now_s - stamp_s
            if stamp_age_s is None or abs(stamp_age_s) > self.cfg.max_tf_age_s:
                logging.debug(
                    f"[{self.name}] ignoring stale or invalid tf sample; "
                    f"stamp_age_s={stamp_age_s}"
                )
                return
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        x = float(t.x)
        y = float(t.y)
        z = float(t.z)
        quat_xyzw = (float(q.x), float(q.y), float(q.z), float(q.w))
        if (
            not np.all(np.isfinite((x, y, z, *quat_xyzw)))
            or np.linalg.norm(quat_xyzw) <= np.finfo(np.float64).eps
        ):
            logging.warning(f"[{self.name}] ignoring non-finite or invalid tf sample")
            return
        yaw = quat_xyzw_to_yaw(*quat_xyzw)

        # --- Bilinear-sample the target map on the shared heightmap geometry
        target_map = self._target_map  # (N, N) float32
        out = sample_single_global_map_at_heightmap_np(
            global_map=target_map,
            robot_xy=(x, y),
            robot_yaw=yaw,
            map_center_world_xy=self.cfg.target_map_center_world_xy,
            geometry=self._geometry,
            extent=float(self.cfg.target_map_extent_m),
        )
        out = world_patch_heights_to_attach_z_np(
            patch_heights_world=out,
            robot_position_world_xyz=(x, y, z),
            robot_quat_xyzw=quat_xyzw,
            geometry=self._geometry,
        )

        if not np.all(np.isfinite(out)):
            logging.warning(f"[{self.name}] ignoring non-finite target patch sample")
            return

        self._last_tf_s = self._clock()
        self.obs[LANDSCAPING_TARGET_PATCH_OBSERVATION.key] = out.astype(
            np.float32,
            copy=False,
        )

    # -------------------------------------------------------------------- property

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self.obs)

    # --------------------------------------------------------------------- helpers
