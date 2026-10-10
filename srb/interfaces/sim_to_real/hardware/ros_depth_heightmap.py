"""Project RealSense depth into an attach-frame heightmap grid.

Produces ``proprio_dyn/heightmap`` and ``proprio_dyn/heightmap_valid_mask``
observations with shape ``(ny, nx)``. ROS imports are deferred until
``start()`` so the module can load without a ROS installation.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Sequence

import gymnasium
import numpy as np
import torch
from pydantic import Field, model_validator

from srb.core.sensor.heightmap import (
    HeightmapSpec,
    fuse_heightmap_views,
    project_depth_to_heightmap,
)
from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.terrain.landscaping_observations import (
    LANDSCAPING_HEIGHTMAP_OBSERVATION,
    LANDSCAPING_HEIGHTMAP_VALID_MASK_OBSERVATION,
    landscaping_heightmap_fill_observation,
    landscaping_heightmap_observation,
)
from srb.terrain.landscaping_pose import (
    quat_xyzw_to_rotmat_np,
)
from srb.utils import logging
from srb.utils.failures import HealthEvent, HealthSeverity, HealthStatus

if TYPE_CHECKING:
    from sensor_msgs.msg import CameraInfo, Image


class RosDepthHeightmapInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for the depth -> heightmap hardware interface."""

    depth_topic: str = "/camera/depth/image_rect_raw"
    """Topic carrying ``sensor_msgs/Image`` depth frames (``16UC1`` or ``32FC1``)."""
    camera_info_topic: str = "/camera/camera_info"
    """Topic carrying ``sensor_msgs/CameraInfo`` for the depth camera."""
    wrist_depth_topic: str | None = None
    """Optional second depth topic, usually a wrist camera."""
    wrist_camera_info_topic: str | None = None
    """Optional second CameraInfo topic. Enabled only with ``wrist_depth_topic``."""
    attach_frame: str = "base_link"
    """TF frame the heightmap is expressed in (target of the tf2 lookup)."""
    tf_timeout_duration: float = 0.05
    """Max seconds to wait when querying tf2."""
    spec: HeightmapSpec = HeightmapSpec()
    """Grid layout parameters (nx, ny, cell_size, offsets, ...)."""
    max_depth_age_s: float = Field(default=0.5, gt=0.0, allow_inf_nan=False)
    """Maximum monotonic age of every required successfully decoded frame and
    of the transform last looked up at that frame's header stamp."""
    min_valid_pixel_fraction: float = Field(
        default=0.0, ge=0.0, le=1.0, allow_inf_nan=False
    )
    """Minimum fraction of finite positive depth pixels; zero still requires one."""

    @model_validator(mode="after")
    def _validate_frames(self) -> RosDepthHeightmapInterfaceCfg:
        if self.spec.frame != self.attach_frame:
            raise ValueError("spec.frame must match attach_frame")
        return self


@dataclass
class _DepthStreamState:
    name: str
    depth_topic: str
    camera_info_topic: str
    depth: torch.Tensor | None = None
    intrinsics: torch.Tensor | None = None
    tf_matrix: torch.Tensor | None = None
    camera_frame: str | None = None
    camera_info_size: tuple[int, int] | None = None
    depth_frame: str | None = None
    last_depth_stamp: object | None = None
    last_depth_s: float | None = None
    tf_stamp_s: float | None = None
    last_tf_s: float | None = None
    depth_error: str | None = None
    camera_info_error: str | None = None
    tf_error: str | None = None
    projection_error: str | None = None
    depth_sub: object | None = None
    camera_info_sub: object | None = None


class RosDepthHeightmapInterface(HardwareInterface):
    """Projects RealSense depth frames into an attach-frame heightmap grid."""

    cfg: RosDepthHeightmapInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (
        (
            LANDSCAPING_HEIGHTMAP_OBSERVATION.key,
            LANDSCAPING_HEIGHTMAP_OBSERVATION.name,
        ),
        (
            LANDSCAPING_HEIGHTMAP_VALID_MASK_OBSERVATION.key,
            LANDSCAPING_HEIGHTMAP_VALID_MASK_OBSERVATION.name,
        ),
    )

    def __init__(
        self,
        cfg: RosDepthHeightmapInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        if cfg is None:
            cfg = RosDepthHeightmapInterfaceCfg()
        super().__init__(cfg)
        self._clock = clock
        # ROS callbacks run on executor threads concurrently with ``sync``.
        self._state_lock = threading.Lock()

        self._primary_stream = _DepthStreamState(
            name="primary",
            depth_topic=cfg.depth_topic,
            camera_info_topic=cfg.camera_info_topic,
        )
        self._streams: list[_DepthStreamState] = [self._primary_stream]
        if bool(cfg.wrist_depth_topic) != bool(cfg.wrist_camera_info_topic):
            logging.warning(
                "[RosDepthHeightmap] wrist camera disabled because both "
                "wrist_depth_topic and wrist_camera_info_topic are required"
            )
        elif cfg.wrist_depth_topic and cfg.wrist_camera_info_topic:
            self._streams.append(
                _DepthStreamState(
                    name="wrist",
                    depth_topic=cfg.wrist_depth_topic,
                    camera_info_topic=cfg.wrist_camera_info_topic,
                )
            )

        # Output buffer returned via the ``observation`` property.
        self.obs: dict[str, np.ndarray] = self._empty_obs()

        # ROS handles (set in ``start``).
        self._tf_buffer = None
        self._tf_listener = None
        self._tf_timeout = None

    # ----------------------------------------------------------------- lifecycle

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.duration import Duration
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import Buffer, TransformListener

        # Depth: sensor_data profile (BEST_EFFORT + VOLATILE).
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # CameraInfo: default reliable.
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for stream in self._streams:
            stream.depth_sub = self.ros_node.create_subscription(
                Image,
                stream.depth_topic,
                lambda msg, _stream=stream: self._locked(
                    self._depth_callback, msg, _stream
                ),
                depth_qos,
            )
            logging.info(
                f"[{self.name}] Subscribing to {stream.name} depth topic: "
                f"{stream.depth_topic}"
            )

            stream.camera_info_sub = self.ros_node.create_subscription(
                CameraInfo,
                stream.camera_info_topic,
                lambda msg, _stream=stream: self._locked(
                    self._camera_info_callback, msg, _stream
                ),
                info_qos,
            )
            logging.info(
                f"[{self.name}] Subscribing to {stream.name} camera_info topic: "
                f"{stream.camera_info_topic}"
            )

        # tf2 listener.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.ros_node)
        self._tf_timeout = Duration(
            seconds=math.floor(self.cfg.tf_timeout_duration),
            nanoseconds=int((self.cfg.tf_timeout_duration % 1) * 1e9),
        )

    def close(self):
        super().close()
        for stream in self._streams:
            if stream.depth_sub is not None:
                stream.depth_sub.destroy()
                stream.depth_sub = None
            if stream.camera_info_sub is not None:
                stream.camera_info_sub.destroy()
                stream.camera_info_sub = None
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        if self._tf_buffer is not None:
            self._tf_buffer.clear()
            self._tf_buffer = None

    def health(self) -> HealthEvent:
        """Report complete depth/calibration/TF readiness and freshness."""

        now = self._clock()
        stream_details: dict[str, object] = {}
        failure: tuple[str, str, str] | None = None
        with self._state_lock:
            for stream in self._streams:
                age_s = (
                    None
                    if stream.last_depth_s is None
                    else max(0.0, now - stream.last_depth_s)
                )
                tf_age_s = (
                    None
                    if stream.tf_matrix is None or stream.last_tf_s is None
                    else max(0.0, now - stream.last_tf_s)
                )
                missing = tuple(
                    name
                    for name, value in (
                        ("depth", stream.depth),
                        ("camera_info", stream.intrinsics),
                        ("transform", stream.tf_matrix),
                    )
                    if value is None
                )
                errors = tuple(
                    value
                    for value in (
                        stream.depth_error,
                        stream.camera_info_error,
                        stream.tf_error,
                        stream.projection_error,
                    )
                    if value is not None
                )
                stream_details[stream.name] = {
                    "missing": missing,
                    "errors": errors,
                    "age_ms": None if age_s is None else age_s * 1000.0,
                    "tf_age_ms": None if tf_age_s is None else tf_age_s * 1000.0,
                    "tf_stamp_s": stream.tf_stamp_s,
                }
                if failure is None and errors:
                    failure = ("failure", stream.name, errors[0])
                elif failure is None and missing:
                    failure = (
                        "pre_message" if "depth" in missing else "incomplete",
                        stream.name,
                        "missing " + ", ".join(missing),
                    )
                elif (
                    failure is None
                    and age_s is not None
                    and age_s > self.cfg.max_depth_age_s
                ):
                    failure = ("stale", stream.name, f"depth age is {age_s:.3f} s")
                elif (
                    failure is None
                    and tf_age_s is not None
                    and tf_age_s > self.cfg.max_depth_age_s
                ):
                    failure = (
                        "stale",
                        stream.name,
                        f"transform age is {tf_age_s:.3f} s",
                    )

        if failure is None:
            state = "healthy"
            status = HealthStatus.HEALTHY
            severity = HealthSeverity.INFO
            message = "all depth streams have current decoded frames and transforms"
        else:
            state, stream_name, reason = failure
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            message = f"depth stream '{stream_name}' is not ready: {reason}"
        return HealthEvent(
            status=status,
            severity=severity,
            component=self.name,
            code=f"{self.name}.{state}",
            message=message,
            details={
                "state": state,
                "streams": stream_details,
                "max_depth_age_ms": self.cfg.max_depth_age_s * 1000.0,
                "ready": status is HealthStatus.HEALTHY,
            },
        )

    def reset(self):
        super().reset()
        with self._state_lock:
            for stream in self._streams:
                stream.depth = None
                stream.intrinsics = None
                stream.tf_matrix = None
                stream.camera_info_size = None
                stream.depth_frame = None
                stream.last_depth_stamp = None
                stream.last_depth_s = None
                stream.tf_stamp_s = None
                stream.last_tf_s = None
                stream.depth_error = None
                stream.camera_info_error = None
                stream.tf_error = None
                stream.projection_error = None
        self.obs = self._empty_obs()

    # -------------------------------------------------------------- main stepping

    def sync(self):
        super().sync()
        views: list[tuple[torch.Tensor, torch.Tensor]] = []
        with self._state_lock:
            for stream in self._streams:
                self._refresh_tf(stream)
                mismatch = self._camera_info_mismatch(stream)
                if mismatch is not None:
                    # Mismatched intrinsics or optical frames produce wrong heights.
                    stream.projection_error = mismatch
                    continue
                try:
                    projected = self._project_stream(stream)
                except Exception as exc:  # noqa: BLE001 - projection is a trust seam
                    stream.projection_error = (
                        f"heightmap projection failed: {type(exc).__name__}: {exc}"
                    )
                    projected = None
                if projected is not None:
                    stream.projection_error = None
                    views.append(projected)

        if not views:
            # Missing data -> emit a fill grid so consumers see stable shape.
            self.obs = self._empty_obs()
            return

        heights, valid = fuse_heightmap_views(
            views,
            fill_value=float(self.cfg.spec.max_range_m),
            shape=(int(self.cfg.spec.ny), int(self.cfg.spec.nx)),
        )

        self.obs = landscaping_heightmap_observation(
            heightmap=heights.detach().cpu().numpy(),
            valid_mask=valid.detach().cpu().numpy(),
        )

    # -------------------------------------------------------------------- property

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self._empty_obs())

    # --------------------------------------------------------------------- helpers

    def _empty_obs(self) -> dict[str, np.ndarray]:
        ny = int(self.cfg.spec.ny)
        nx = int(self.cfg.spec.nx)
        fill = float(self.cfg.spec.max_range_m)
        return landscaping_heightmap_fill_observation(
            shape=(ny, nx),
            fill_value=fill,
        )

    @staticmethod
    def _camera_info_mismatch(stream: _DepthStreamState) -> str | None:
        """Describe a depth/CameraInfo disagreement, or ``None`` if consistent."""
        if stream.depth is None or stream.intrinsics is None:
            return None
        depth_size = (int(stream.depth.shape[0]), int(stream.depth.shape[1]))
        if stream.camera_info_size != depth_size:
            info_h, info_w = stream.camera_info_size or (0, 0)
            return (
                f"depth image resolution {depth_size[1]}x{depth_size[0]} does not "
                f"match CameraInfo resolution {info_w}x{info_h}"
            )
        if (
            stream.depth_frame
            and stream.camera_frame
            and stream.depth_frame != stream.camera_frame
        ):
            return (
                f"depth image frame '{stream.depth_frame}' does not match "
                f"CameraInfo frame '{stream.camera_frame}'"
            )
        return None

    @staticmethod
    def _stamp_to_s(stamp: object | None) -> float | None:
        """Convert a ``builtin_interfaces/Time`` to seconds; ``None`` if absent."""
        try:
            sec = int(getattr(stamp, "sec"))
            nanosec = int(getattr(stamp, "nanosec"))
        except (AttributeError, TypeError, ValueError):
            return None
        return sec + nanosec * 1e-9

    # ---- Callbacks ----

    def _project_stream(
        self, stream: _DepthStreamState
    ) -> tuple[torch.Tensor, torch.Tensor] | None:
        if (
            stream.depth is None
            or stream.intrinsics is None
            or stream.tf_matrix is None
        ):
            return None
        return project_depth_to_heightmap(
            stream.depth,
            stream.intrinsics,
            stream.tf_matrix,
            self.cfg.spec,
        )

    def _locked(
        self,
        callback: Callable[[object, _DepthStreamState], None],
        msg: object,
        stream: _DepthStreamState,
    ) -> None:
        with self._state_lock:
            callback(msg, stream)

    def _depth_callback(self, msg: "Image", stream: _DepthStreamState | None = None):
        """Decode depth image into a (H, W) float32 torch tensor in metres."""
        stream = stream or self._primary_stream
        try:
            from cv_bridge import CvBridge  # type: ignore[import-untyped]

            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except ImportError:
            cv_image = self._numpy_imgmsg_to_array(msg)
            if cv_image is None:
                stream.depth = None
                stream.last_depth_s = None
                stream.depth_error = "manual depth decode returned no frame"
                return
        except Exception as e:  # noqa: BLE001 - log and drop frame
            logging.warning(f"[{self.name}] Failed to decode depth image: {e}")
            stream.depth = None
            stream.last_depth_s = None
            stream.depth_error = f"depth decode failed: {type(e).__name__}: {e}"
            return

        depth_np = self._depth_to_metres(cv_image, msg.encoding)
        if depth_np is None:
            stream.depth = None
            stream.last_depth_s = None
            stream.depth_error = "decoded depth frame has an unsupported shape"
            return
        valid_depth = np.isfinite(depth_np) & (depth_np > 0.0)
        valid_pixel_count = int(np.count_nonzero(valid_depth))
        valid_pixel_fraction = valid_pixel_count / max(1, depth_np.size)
        if (
            valid_pixel_count == 0
            or valid_pixel_fraction < self.cfg.min_valid_pixel_fraction
        ):
            stream.depth = None
            stream.last_depth_stamp = None
            stream.depth_frame = None
            stream.last_depth_s = None
            stream.depth_error = (
                f"depth frame has too few valid depth pixels: "
                f"{valid_pixel_count}/{depth_np.size} "
                f"({valid_pixel_fraction:.1%}); minimum is "
                f"{self.cfg.min_valid_pixel_fraction:.1%} and at least one pixel"
            )
            return
        # REP 118 permits NaN and infinities for pixels without a return.
        # Convert them to the +inf sentinel masked by the projection helper;
        # malformed frames are rejected above.
        invalid = ~np.isfinite(depth_np)
        if invalid.any():
            depth_np = np.where(invalid, np.float32(np.inf), depth_np).astype(
                np.float32, copy=False
            )

        stream.depth = torch.from_numpy(depth_np).to(dtype=torch.float32, device="cpu")
        header = getattr(msg, "header", None)
        stream.last_depth_stamp = getattr(header, "stamp", None)
        stream.depth_frame = getattr(header, "frame_id", "") or None
        stream.last_depth_s = self._clock()
        stream.depth_error = None

    def _camera_info_callback(
        self, msg: "CameraInfo", stream: _DepthStreamState | None = None
    ):
        """Extract 3x3 intrinsics and cache the optical frame id."""
        stream = stream or self._primary_stream
        try:
            k = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        except Exception as e:  # noqa: BLE001 - log and drop
            logging.warning(f"[{self.name}] Bad CameraInfo.k: {e}")
            stream.intrinsics = None
            stream.camera_info_error = f"invalid CameraInfo.k: {type(e).__name__}: {e}"
            return
        if not np.all(np.isfinite(k)) or float(k[0, 0]) <= 0.0 or float(k[1, 1]) <= 0.0:
            stream.intrinsics = None
            stream.camera_info_error = "CameraInfo.k must be finite with positive fx/fy"
            return
        stream.intrinsics = torch.from_numpy(k).to(dtype=torch.float32, device="cpu")
        stream.camera_info_size = (
            int(getattr(msg, "height", 0) or 0),
            int(getattr(msg, "width", 0) or 0),
        )
        stream.camera_info_error = None

        frame_id = getattr(msg.header, "frame_id", "") or ""
        if frame_id:
            stream.camera_frame = frame_id

    def _refresh_tf(self, stream: _DepthStreamState | None = None):
        """Look up attach_frame <- camera_optical_frame at the depth stamp.

        The transform is queried at the cached depth image's header stamp so
        the projection pairs each frame with the pose it was captured at.  A
        missing or zero stamp is rejected: tf2 would treat it as "latest" and
        silently pair the frame with an unrelated pose.
        """
        stream = stream or self._primary_stream
        if self._tf_buffer is None or not stream.camera_frame:
            return
        if stream.depth is None:
            return

        depth_stamp = stream.last_depth_stamp
        depth_stamp_s = self._stamp_to_s(depth_stamp)
        if depth_stamp_s is None or depth_stamp_s <= 0.0:
            stream.tf_matrix = None
            stream.tf_stamp_s = None
            stream.tf_error = (
                "depth image has no usable header stamp for the transform lookup"
            )
            return

        from rclpy.time import Time

        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self.cfg.attach_frame,
                stream.camera_frame,
                Time.from_msg(depth_stamp),
                self._tf_timeout,
            )
        except Exception as e:  # noqa: BLE001 - tf not ready yet is normal at startup
            logging.debug(
                f"[{self.name}] tf2 lookup {self.cfg.attach_frame} <- "
                f"{stream.camera_frame} at {depth_stamp_s:.9f} failed: {e}"
            )
            stream.tf_matrix = None
            stream.tf_stamp_s = None
            stream.tf_error = f"transform lookup failed: {type(e).__name__}: {e}"
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        # The rotation helper maps a zero-norm quaternion to identity.
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not 1e-12 <= norm_sq < math.inf:
            stream.tf_matrix = None
            stream.tf_stamp_s = None
            stream.tf_error = "transform rotation is not a valid quaternion"
            return
        tf = np.eye(4, dtype=np.float32)
        tf[:3, :3] = quat_xyzw_to_rotmat_np(q.x, q.y, q.z, q.w)
        tf[:3, 3] = (t.x, t.y, t.z)

        if not np.all(np.isfinite(tf)):
            stream.tf_matrix = None
            stream.tf_stamp_s = None
            stream.tf_error = "transform contains non-finite values"
            return

        stream.tf_matrix = torch.from_numpy(tf).to(dtype=torch.float32, device="cpu")
        tf_stamp_s = self._stamp_to_s(
            getattr(getattr(tf_stamped, "header", None), "stamp", None)
        )
        stream.tf_stamp_s = depth_stamp_s if tf_stamp_s is None else tf_stamp_s
        stream.last_tf_s = self._clock()
        stream.tf_error = None

    @property
    def _depth(self) -> torch.Tensor | None:
        return self._primary_stream.depth

    @_depth.setter
    def _depth(self, value: torch.Tensor | None) -> None:
        self._primary_stream.depth = value

    @property
    def _intrinsics(self) -> torch.Tensor | None:
        return self._primary_stream.intrinsics

    @_intrinsics.setter
    def _intrinsics(self, value: torch.Tensor | None) -> None:
        self._primary_stream.intrinsics = value

    @property
    def _tf_matrix(self) -> torch.Tensor | None:
        return self._primary_stream.tf_matrix

    @_tf_matrix.setter
    def _tf_matrix(self, value: torch.Tensor | None) -> None:
        self._primary_stream.tf_matrix = value

    @property
    def _camera_frame(self) -> str | None:
        return self._primary_stream.camera_frame

    @_camera_frame.setter
    def _camera_frame(self, value: str | None) -> None:
        self._primary_stream.camera_frame = value

    @property
    def _last_depth_stamp(self) -> object | None:
        return self._primary_stream.last_depth_stamp

    @_last_depth_stamp.setter
    def _last_depth_stamp(self, value: object | None) -> None:
        self._primary_stream.last_depth_stamp = value

    # ---- Depth decoding ----

    @staticmethod
    def _depth_to_metres(array: np.ndarray, encoding: str) -> np.ndarray | None:
        """Normalise depth array to float32 metres, regardless of source encoding."""
        if array is None:
            return None
        if array.ndim == 3 and array.shape[-1] == 1:
            array = array[..., 0]
        if array.ndim != 2:
            logging.warning(f"[RosDepthHeightmap] Unexpected depth shape {array.shape}")
            return None

        enc = (encoding or "").lower()
        if array.dtype == np.uint16 or enc == "16uc1":
            # RealSense publishes millimetres in 16UC1.
            return array.astype(np.float32) / 1000.0
        if array.dtype == np.float32 or enc == "32fc1":
            return array.astype(np.float32)
        # Fallback: cast & hope caller provided metres.
        return array.astype(np.float32)

    @staticmethod
    def _numpy_imgmsg_to_array(msg: "Image") -> np.ndarray | None:
        """Minimal cv_bridge-free decode for sensor_msgs/Image (depth only)."""
        if not msg.data:
            return None
        enc = (msg.encoding or "").lower()
        if "fc" in enc:
            dtype = np.float32
        elif "16" in enc:
            dtype = np.uint16
        else:
            dtype = np.uint8
        try:
            wire_dtype = np.dtype(dtype).newbyteorder(
                ">" if getattr(msg, "is_bigendian", False) else "<"
            )
            itemsize = wire_dtype.itemsize
            # Channels come from the encoding (``...C<n>``); ``step`` may
            # include row padding, so it only gives the row stride.
            _, sep, suffix = enc.rpartition("c")
            channels = int(suffix) if sep and suffix.isdigit() else 1
            row_bytes = msg.width * channels * itemsize
            step = msg.step or row_bytes
            if step < row_bytes:
                raise ValueError(f"step {step} < row size {row_bytes}")
            rows = np.frombuffer(msg.data, dtype=np.uint8, count=step * msg.height)
            rows = np.ascontiguousarray(rows.reshape(msg.height, step)[:, :row_bytes])
            shape = (
                (msg.height, msg.width, channels)
                if channels > 1
                else (msg.height, msg.width)
            )
            # Decode with the wire byte order, then copy into native order.
            return (
                rows.view(wire_dtype)
                .reshape(shape)
                .astype(wire_dtype.newbyteorder("="))
            )
        except (ValueError, TypeError) as e:
            logging.error(f"[RosDepthHeightmap] Failed manual decode: {e}")
            return None
