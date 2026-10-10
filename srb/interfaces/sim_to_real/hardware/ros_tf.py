import math
import time
from collections.abc import Callable
from enum import Enum, auto
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, Sequence, Set

import gymnasium
import numpy as np
from pydantic import BaseModel, Field
from typing_extensions import Self

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
    observation_spaces_from_values,
)
from srb.utils import logging
from srb.utils.failures import HealthEvent, HealthSeverity, HealthStatus

from ._rotation import quat_to_rotmat as _quat_to_rotmat

if TYPE_CHECKING:
    from geometry_msgs.msg import Quaternion


class PositionRepresentation(Enum):
    POS_3D = auto()
    POS_2D = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class RotationRepresentation(Enum):
    QUAT = auto()
    ROTMAT = auto()
    ROT_6D = auto()
    ROT_2D_TRIG_YAW = auto()
    ROTMAT_COL_X = auto()
    ROTMAT_COL_Y = auto()
    ROTMAT_COL_Z = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class StaticTransformCfg(BaseModel):
    frame_id: str
    child_frame_id: str
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    #: Either a quaternion in SRB's ``(x, y, z, w)`` order or ``(roll, pitch,
    #: yaw)`` in radians.
    rotation: tuple[float, float, float] | tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )


class RosTfInterfaceCfg(HardwareInterfaceCfg):
    timeout_duration: float = Field(default=0.05, gt=0.0, allow_inf_nan=False)
    discovery_interval: float = 2.0
    position_repr: Sequence[PositionRepresentation] = (PositionRepresentation.POS_3D,)
    rotation_repr: Sequence[RotationRepresentation] = (RotationRepresentation.ROT_6D,)

    allowlist: Sequence[str] = ()
    blocklist: Sequence[str] = ("map", "odom")
    prefix_striplist: Sequence[str] = ()

    static_transforms: Sequence[StaticTransformCfg] = ()

    # Dynamic TF is optional for the generic adapter: callers that use TF as a
    # policy input declare the pairs that must be fresh before dispatch.  A
    # configured static pair is exempt from this age gate, including a valid
    # ROS zero timestamp.
    required_dynamic_frame_pairs: Sequence[tuple[str, str]] = ()
    # Pairs whose observation keys are declared at construction without a
    # freshness gate.  Required pairs are always declared too, so RealEnv can
    # validate every policy observation route before ROS start.
    observation_frame_pairs: Sequence[tuple[str, str]] = ()
    max_transform_age_s: float = Field(default=0.5, gt=0.0, allow_inf_nan=False)
    clock_jump_tolerance_s: float = Field(default=0.0, ge=0.0, allow_inf_nan=False)


class RosTfInterface(HardwareInterface):
    cfg: RosTfInterfaceCfg

    def __init__(
        self,
        cfg: RosTfInterfaceCfg | None = None,
        *,
        ros_clock: Callable[[], object] | None = None,
    ):
        if cfg is None:
            cfg = RosTfInterfaceCfg()
        super().__init__(cfg)

        self.last_discovery_time: float = 0.0
        self.discovered_frames: Set[str] = set()
        self.obs: dict[str, np.ndarray] = {}
        self.tf_buffer = None
        self.tf_listener = None
        self.static_tf_broadcaster = None
        self.tf_timeout_duration = None
        # Keep the fallback narrow until ``start()`` records tf2's concrete
        # TransformException class.  Native tf2 lookup failures are transport
        # state, not fatal adapter errors, and must reach ``health()``.
        self._transform_exception: type[BaseException] = LookupError
        self._ros_clock = ros_clock
        self._last_ros_now_s: float | None = None
        self._clock_invalid_reason: str | None = None
        self._clock_recovery_pending: bool = False
        self._pair_state: dict[tuple[str, str], dict[str, object]] = {
            pair: {
                "lookup_success": False,
                "last_stamp_s": None,
                "last_receive_s": None,
                "last_error": None,
                "stamp_valid": False,
            }
            for pair in self._required_dynamic_pairs()
        }
        # Declare configured observation keys before start(): RealEnv builds
        # its observation routes before acquiring ROS.
        for source_frame_full, target_frame_full in self._declared_pairs():
            self._ensure_observation_pair(source_frame_full, target_frame_full)

    def _required_dynamic_pairs(self) -> tuple[tuple[str, str], ...]:
        return self._validated_pairs(
            self.cfg.required_dynamic_frame_pairs,
            field="required_dynamic_frame_pairs",
        )

    def _observation_pairs(self) -> tuple[tuple[str, str], ...]:
        return self._validated_pairs(
            self.cfg.observation_frame_pairs,
            field="observation_frame_pairs",
        )

    def _declared_pairs(self) -> tuple[tuple[str, str], ...]:
        """Required pairs, then observation-only pairs, without duplicates."""

        return tuple(
            dict.fromkeys((*self._required_dynamic_pairs(), *self._observation_pairs()))
        )

    @staticmethod
    def _validated_pairs(
        raw_pairs: Sequence[tuple[str, str]], *, field: str
    ) -> tuple[tuple[str, str], ...]:
        pairs: list[tuple[str, str]] = []
        seen: set[tuple[str, str]] = set()
        for raw_pair in raw_pairs:
            if len(raw_pair) != 2:
                raise ValueError(
                    f"{field} entries must contain exactly "
                    f"(target_frame, source_frame), got {raw_pair!r}"
                )
            pair = (str(raw_pair[0]).strip(), str(raw_pair[1]).strip())
            if not pair[0] or not pair[1] or pair[0] == pair[1]:
                raise ValueError(
                    f"{field} entries must contain two "
                    f"distinct non-empty frame names, got {raw_pair!r}"
                )
            if pair not in seen:
                seen.add(pair)
                pairs.append(pair)
        return tuple(pairs)

    def _configured_static_pairs(self) -> frozenset[tuple[str, str]]:
        return frozenset(
            pair
            for static_tf in self.cfg.static_transforms
            for pair in (
                (static_tf.frame_id, static_tf.child_frame_id),
                (static_tf.child_frame_id, static_tf.frame_id),
            )
        )

    def start(self, **kwargs):
        super().start(**kwargs)
        from geometry_msgs.msg import Transform, TransformStamped, Vector3
        from rclpy.duration import Duration
        from std_msgs.msg import Header
        from tf2_ros import (
            Buffer,
            StaticTransformBroadcaster,
            TransformException,
            TransformListener,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.ros_node)
        self._transform_exception = TransformException
        self.tf_timeout_duration = Duration(
            seconds=math.floor(self.cfg.timeout_duration),
            nanoseconds=int((self.cfg.timeout_duration % 1) * 1e9),
        )
        self._last_ros_now_s = None
        self._clock_invalid_reason = None
        self._clock_recovery_pending = False
        self._pair_state = {
            pair: {
                "lookup_success": False,
                "last_stamp_s": None,
                "last_receive_s": None,
                "last_error": None,
                "stamp_valid": False,
            }
            for pair in self._required_dynamic_pairs()
        }

        # Publish static transforms
        if self.cfg.static_transforms:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self.ros_node)
            for msg in (
                TransformStamped(
                    header=Header(
                        stamp=self.ros_node.get_clock().now().to_msg(),
                        frame_id=static_tf.frame_id,
                    ),
                    child_frame_id=static_tf.child_frame_id,
                    transform=Transform(
                        translation=Vector3(
                            x=static_tf.translation[0],
                            y=static_tf.translation[1],
                            z=static_tf.translation[2],
                        ),
                        rotation=self._static_rotation_to_quat(static_tf.rotation),
                    ),
                )
                for static_tf in self.cfg.static_transforms
            ):
                self.static_tf_broadcaster.sendTransform(msg)
                logging.info(
                    f"[{self.name}] Published static transform: {msg.header.frame_id} -> {msg.child_frame_id}"
                )

    def close(self):
        errors: list[Exception] = []
        listener = self.tf_listener
        if listener is not None:
            try:
                listener.unregister()
            except (
                AttributeError,
                ExceptionGroup,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                errors.append(exc)
            else:
                self.tf_listener = None
        buffer = self.tf_buffer
        if buffer is not None and self.tf_listener is None:
            try:
                buffer.clear()
            except (
                AttributeError,
                ExceptionGroup,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                errors.append(exc)
            else:
                self.tf_buffer = None
        if errors:
            if len(errors) == 1:
                raise errors[0]
            raise ExceptionGroup("ROS TF cleanup failed", errors)
        super().close()

    def sync(self):
        super().sync()
        self._discover_frames()
        self._update_transforms()

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs.copy()

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return observation_spaces_from_values(self.obs)

    @property
    def info(self) -> dict[str, Any]:
        return {
            "ros_tf/frames": self.discovered_frames,
            "ros_tf/required_dynamic_frame_pairs": self._required_dynamic_pairs(),
            "ros_tf/health": self.health(),
        }

    def health(self) -> HealthEvent | None:
        """Report freshness of required dynamic TF pairs.

        Compare TF stamps with ROS time. Monotonic time is receipt-time
        diagnostic only and is never subtracted from a ROS timestamp.
        """

        required_pairs = self._required_dynamic_pairs()
        if not required_pairs:
            return None
        if self.tf_buffer is None:
            required_labels = tuple(self._pair_label(pair) for pair in required_pairs)
            return HealthEvent(
                status=HealthStatus.UNHEALTHY,
                severity=HealthSeverity.ERROR,
                component=self.name,
                code=f"{self.name}.transport_unavailable",
                message="required dynamic TF pairs have no active TF buffer",
                details={
                    "state": "transport_unavailable",
                    "pairs": {
                        label: {
                            "state": "transport_unavailable",
                            "lookup_success": False,
                            "stamp_s": None,
                            "age_ms": None,
                            "last_receive_s": None,
                            "last_error": None,
                            "ready": False,
                        }
                        for label in required_labels
                    },
                    "required_pairs": required_labels,
                    "max_transform_age_ms": self.cfg.max_transform_age_s * 1000.0,
                    "clock_invalid_reason": None,
                    "ready": False,
                },
            )

        static_pairs = self._configured_static_pairs()
        now_s = self._ros_now_seconds()
        pair_details: dict[str, dict[str, object]] = {}
        unhealthy: list[tuple[tuple[str, str], str, str]] = []
        for pair in required_pairs:
            state = self._pair_state.setdefault(
                pair,
                {
                    "lookup_success": False,
                    "last_stamp_s": None,
                    "last_receive_s": None,
                    "last_error": None,
                    "stamp_valid": False,
                },
            )
            label = self._pair_label(pair)
            if pair in static_pairs:
                static_ready = bool(state["lookup_success"])
                pair_details[label] = {
                    "state": "static" if static_ready else "static_unavailable",
                    "lookup_success": static_ready,
                    "stamp_s": state["last_stamp_s"],
                    "age_ms": None,
                    "last_receive_s": state["last_receive_s"],
                    "last_error": state["last_error"],
                    "ready": static_ready,
                }
                if not static_ready:
                    unhealthy.append(
                        (
                            pair,
                            "static_unavailable",
                            str(
                                state["last_error"]
                                or "configured static transform has not been accepted"
                            ),
                        )
                    )
                continue

            pair_state = "healthy"
            reason = "dynamic transform is fresh"
            age_s: float | None = None
            stamp_s = state["last_stamp_s"]
            if self._clock_invalid_reason is not None or now_s is None:
                pair_state = "clock_invalid"
                reason = self._clock_invalid_reason or "ROS clock is unavailable"
            elif not bool(state["lookup_success"]):
                pair_state = "pre_message"
                reason = "waiting for the first required transform"
                if state["last_error"]:
                    pair_state = "lookup_failure"
                    reason = str(state["last_error"])
            elif not bool(state["stamp_valid"]) or stamp_s is None:
                pair_state = "invalid_stamp"
                reason = "required dynamic transform has no valid ROS timestamp"
            else:
                age_s = now_s - float(stamp_s)
                if age_s < -self.cfg.clock_jump_tolerance_s:
                    pair_state = "clock_invalid"
                    reason = "transform timestamp is ahead of the ROS clock"
                elif age_s > self.cfg.max_transform_age_s:
                    pair_state = "stale"
                    reason = (
                        f"required transform age {age_s:.3f} s exceeds "
                        f"{self.cfg.max_transform_age_s:.3f} s"
                    )
            pair_details[label] = {
                "state": pair_state,
                "lookup_success": bool(state["lookup_success"]),
                "stamp_s": stamp_s,
                "age_ms": None if age_s is None else max(0.0, age_s) * 1000.0,
                "last_receive_s": state["last_receive_s"],
                "last_error": state["last_error"],
                "ready": pair_state == "healthy",
            }
            if pair_state != "healthy":
                unhealthy.append((pair, pair_state, reason))

        if unhealthy:
            pair, state, reason = unhealthy[0]
            message = f"TF pair {self._pair_label(pair)} is not ready: {reason}"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            state_name = state
        else:
            message = "all required dynamic TF pairs are fresh"
            status = HealthStatus.HEALTHY
            severity = HealthSeverity.INFO
            state_name = "healthy"
        return HealthEvent(
            status=status,
            severity=severity,
            component=self.name,
            code=f"{self.name}.{state_name}",
            message=message,
            details={
                "state": state_name,
                "pairs": pair_details,
                "required_pairs": tuple(
                    self._pair_label(pair) for pair in required_pairs
                ),
                "max_transform_age_ms": self.cfg.max_transform_age_s * 1000.0,
                "clock_invalid_reason": self._clock_invalid_reason,
                "ready": status is HealthStatus.HEALTHY,
            },
        )

    def _discover_frames(self):
        if self.tf_buffer is None:
            return
        current_time = time.time()
        if current_time - self.last_discovery_time < self.cfg.discovery_interval:
            return
        self.last_discovery_time = current_time

        new_frames = set()
        for line in self.tf_buffer.all_frames_as_string().split("\n"):
            # Output format: 'Frame <> exists with parent <>.'
            line: str = line.removeprefix("Frame ").removesuffix(".")
            if not line:
                continue
            try:
                child_frame, parent_frame = line.split(" exists with parent ", 1)
            except ValueError:
                logging.warning(
                    f"[{self.name}] Ignoring malformed TF frame line: {line}"
                )
                continue
            if not child_frame or not parent_frame:
                continue
            child_frame = child_frame.strip()
            parent_frame = parent_frame.strip()
            if (
                child_frame
                and self._is_frame_allowed(child_frame)
                and child_frame not in self.discovered_frames
            ):
                new_frames.add(child_frame)
            if (
                parent_frame
                and self._is_frame_allowed(parent_frame)
                and parent_frame not in self.discovered_frames
            ):
                new_frames.add(parent_frame)

        if new_frames:
            self._initialize_new_frames(new_frames)
            logging.info(
                f"[{self.name}] Discovered new frames: {', '.join(new_frames)}"
            )

    def _is_frame_allowed(self, frame_name: str) -> bool:
        if self.cfg.allowlist:
            return frame_name in self.cfg.allowlist

        if self.cfg.blocklist:
            return frame_name not in self.cfg.blocklist

        return True

    def _get_stripped_frame_name(self, frame_name: str) -> str:
        for prefix in self.cfg.prefix_striplist:
            if frame_name.startswith(prefix):
                return frame_name.removeprefix(prefix)
        return frame_name

    def _initialize_new_frames(self, new_frames: Set[str]):
        self.discovered_frames.update(new_frames)
        for source_frame_full in self.discovered_frames:
            for target_frame_full in self.discovered_frames:
                # We need to check against full names for discovery logic
                if (source_frame_full == target_frame_full) or (
                    source_frame_full not in new_frames
                    and target_frame_full not in new_frames
                ):
                    continue
                # Seed only: a declared pair whose frames are discovered later
                # keeps its latest measured value.
                self._ensure_observation_pair(source_frame_full, target_frame_full)

    @staticmethod
    def _pair_label(pair: tuple[str, str]) -> str:
        return f"{pair[0]} -> {pair[1]}"

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
            sec_value = getattr(stamp, "sec", None)
            if sec_value is None:
                sec_value = getattr(stamp, "secs")
            nanosec_value = getattr(stamp, "nanosec", None)
            if nanosec_value is None:
                nanosec_value = getattr(stamp, "nsec", 0.0)
            sec = float(sec_value)
            nanosec = float(nanosec_value)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        value = sec + nanosec / 1e9
        return value if math.isfinite(value) else None

    def _ros_now_seconds(self) -> float | None:
        try:
            if self._ros_clock is not None:
                raw_now = self._ros_clock()
            else:
                node = getattr(self, "_ros_node", None)
                if node is None:
                    return None
                raw_now = node.get_clock().now()
            now_s = self._stamp_seconds(raw_now)
            if now_s is None and isinstance(raw_now, (int, float)):
                now_s = float(raw_now)
            if now_s is None or not math.isfinite(now_s):
                self._clock_invalid_reason = "ROS clock returned a non-finite time"
                return None
        except (
            AttributeError,
            OSError,
            OverflowError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as exc:
            self._clock_invalid_reason = (
                f"ROS clock read failed: {type(exc).__name__}: {exc}"
            )
            return None

        previous = self._last_ros_now_s
        if previous is not None and now_s < previous - self.cfg.clock_jump_tolerance_s:
            self._clock_invalid_reason = (
                f"ROS clock moved backwards from {previous:.9f} to {now_s:.9f}"
            )
            self._clock_recovery_pending = True
        elif (
            self._clock_invalid_reason is not None
            and not self._clock_recovery_pending
            and (previous is None or now_s >= previous)
        ):
            # A subsequent non-decreasing sample establishes a usable clock
            # again.  The old transform still has to pass the age gate.
            self._clock_invalid_reason = None
        self._last_ros_now_s = now_s
        return now_s

    def _ensure_observation_pair(
        self, source_frame_full: str, target_frame_full: str
    ) -> tuple[str, str]:
        source_frame = self._get_stripped_frame_name(source_frame_full)
        target_frame = self._get_stripped_frame_name(target_frame_full)
        if PositionRepresentation.POS_3D in self.cfg.position_repr:
            self.obs.setdefault(
                f"state/tf_pos_{source_frame}_to_{target_frame}",
                np.zeros(3, dtype=np.float32),
            )
        if PositionRepresentation.POS_2D in self.cfg.position_repr:
            self.obs.setdefault(
                f"state/tf_pos2d_{source_frame}_to_{target_frame}",
                np.zeros(2, dtype=np.float32),
            )
        if RotationRepresentation.QUAT in self.cfg.rotation_repr:
            self.obs.setdefault(
                f"state/tf_quat_{source_frame}_to_{target_frame}",
                np.array((0.0, 0.0, 0.0, 1.0), dtype=np.float32),
            )
        if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
            self.obs.setdefault(
                f"state/tf_rotmat_{source_frame}_to_{target_frame}",
                np.eye(3, dtype=np.float32).flatten(),
            )
        if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
            # Identity: the first two columns of eye(3), as _rotmat_to_rot6d.
            self.obs.setdefault(
                f"state/tf_rot6d_{source_frame}_to_{target_frame}",
                np.array((1.0, 0.0, 0.0, 1.0, 0.0, 0.0), dtype=np.float32),
            )
        if RotationRepresentation.ROT_2D_TRIG_YAW in self.cfg.rotation_repr:
            self.obs.setdefault(
                f"state/tf_rot2dtrigyaw_{source_frame}_to_{target_frame}",
                np.array((0.0, 1.0), dtype=np.float32),
            )
        if RotationRepresentation.ROTMAT_COL_X in self.cfg.rotation_repr:
            self.obs.setdefault(
                f"state/tf_rotmatcolx_{source_frame}_to_{target_frame}",
                np.array((1.0, 0.0, 0.0), dtype=np.float32),
            )
        if RotationRepresentation.ROTMAT_COL_Y in self.cfg.rotation_repr:
            self.obs.setdefault(
                f"state/tf_rotmatcoly_{source_frame}_to_{target_frame}",
                np.array((0.0, 1.0, 0.0), dtype=np.float32),
            )
        if RotationRepresentation.ROTMAT_COL_Z in self.cfg.rotation_repr:
            self.obs.setdefault(
                f"state/tf_rotmatcolz_{source_frame}_to_{target_frame}",
                np.array((0.0, 0.0, 1.0), dtype=np.float32),
            )
        return source_frame, target_frame

    def _update_transforms(self):
        from rclpy.time import Time

        if self.tf_buffer is None:
            return
        required_pairs = set(self._required_dynamic_pairs())
        pairs = set(self._declared_pairs())
        pairs.update(
            (source_frame_full, target_frame_full)
            for source_frame_full in self.discovered_frames
            for target_frame_full in self.discovered_frames
            if source_frame_full != target_frame_full
        )
        now_s = self._ros_now_seconds()
        receive_s = time.monotonic()
        for source_frame_full, target_frame_full in sorted(pairs):
            source_frame, target_frame = self._ensure_observation_pair(
                source_frame_full, target_frame_full
            )
            pair = (source_frame_full, target_frame_full)
            state = self._pair_state.setdefault(
                pair,
                {
                    "lookup_success": False,
                    "last_stamp_s": None,
                    "last_receive_s": None,
                    "last_error": None,
                    "stamp_valid": False,
                },
            )
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    source_frame_full,
                    target_frame_full,
                    Time(),
                    self.tf_timeout_duration,
                )
            except (
                self._transform_exception,
                AttributeError,
                ExceptionGroup,
                LookupError,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                state["lookup_success"] = False
                state["last_error"] = f"{type(exc).__name__}: {exc}"
                if pair in required_pairs:
                    logging.warning(
                        f"[{self.name}] Failed to get transform from "
                        f"{source_frame_full} to {target_frame_full}: {exc}"
                    )
                continue

            state["last_receive_s"] = receive_s
            try:
                if not self._transform_is_finite(tf_stamped):
                    raise ValueError("transform contains non-finite values")
            except (
                AttributeError,
                ExceptionGroup,
                OSError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                state["lookup_success"] = False
                state["last_error"] = f"{type(exc).__name__}: {exc}"
                if pair in required_pairs:
                    logging.warning(
                        f"[{self.name}] Ignoring invalid transform from "
                        f"{source_frame_full} to {target_frame_full}: {exc}"
                    )
                continue

            state["lookup_success"] = True
            state["last_error"] = None
            stamp_s = self._stamp_seconds(
                getattr(getattr(tf_stamped, "header", None), "stamp", None)
            )
            state["last_stamp_s"] = stamp_s
            state["stamp_valid"] = stamp_s is not None and stamp_s > 0.0
            if (
                pair in required_pairs
                and pair not in self._configured_static_pairs()
                and stamp_s is not None
                and now_s is not None
                and stamp_s > now_s + self.cfg.clock_jump_tolerance_s
            ):
                self._clock_invalid_reason = (
                    f"transform {self._pair_label(pair)} is ahead of the ROS clock"
                )
                self._clock_recovery_pending = True
            elif (
                pair in required_pairs
                and self._clock_recovery_pending
                and stamp_s is not None
                and now_s is not None
                and 0.0 < stamp_s <= now_s + self.cfg.clock_jump_tolerance_s
            ):
                # A post-jump fresh sample is required before the clock is
                # trusted again.  A repeated health() call is insufficient.
                self._clock_recovery_pending = False

            self._write_transform_observation(
                tf_stamped,
                source_frame,
                target_frame,
            )

    @staticmethod
    def _transform_is_finite(tf_stamped: object) -> bool:
        transform = tf_stamped.transform
        translation = transform.translation
        rotation = transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        quaternion_norm = sum(
            float(value) ** 2
            for value in (
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
        )
        return quaternion_norm > 0.0 and math.isfinite(quaternion_norm)

    def _write_transform_observation(
        self,
        tf_stamped: object,
        source_frame: str,
        target_frame: str,
    ) -> None:
        transform = tf_stamped.transform
        translation = transform.translation
        rotation = transform.rotation
        quaternion_norm = math.sqrt(
            float(rotation.x) ** 2
            + float(rotation.y) ** 2
            + float(rotation.z) ** 2
            + float(rotation.w) ** 2
        )
        if not math.isfinite(quaternion_norm) or quaternion_norm == 0.0:
            raise ValueError("transform quaternion must have a finite, nonzero norm")
        normalized_rotation = SimpleNamespace(
            x=float(rotation.x) / quaternion_norm,
            y=float(rotation.y) / quaternion_norm,
            z=float(rotation.z) / quaternion_norm,
            w=float(rotation.w) / quaternion_norm,
        )
        if PositionRepresentation.POS_3D in self.cfg.position_repr:
            self.obs[f"state/tf_pos_{source_frame}_to_{target_frame}"] = np.array(
                (translation.x, translation.y, translation.z), dtype=np.float32
            )
        if PositionRepresentation.POS_2D in self.cfg.position_repr:
            self.obs[f"state/tf_pos2d_{source_frame}_to_{target_frame}"] = np.array(
                (translation.x, translation.y), dtype=np.float32
            )
        rotmat: np.ndarray | None = None
        if RotationRepresentation.QUAT in self.cfg.rotation_repr:
            self.obs[f"state/tf_quat_{source_frame}_to_{target_frame}"] = np.array(
                (
                    normalized_rotation.x,
                    normalized_rotation.y,
                    normalized_rotation.z,
                    normalized_rotation.w,
                ),
                dtype=np.float32,
            )
        if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
            rotmat = _quat_to_rotmat(normalized_rotation)
            # Declared and seeded as a flat row-major (9,) vector.
            self.obs[f"state/tf_rotmat_{source_frame}_to_{target_frame}"] = (
                rotmat.flatten()
            )
        if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
            if rotmat is None:
                rotmat = _quat_to_rotmat(normalized_rotation)
            self.obs[f"state/tf_rot6d_{source_frame}_to_{target_frame}"] = (
                self._rotmat_to_rot6d(rotmat)
            )
        if RotationRepresentation.ROT_2D_TRIG_YAW in self.cfg.rotation_repr:
            yaw = self._quat_to_yaw(normalized_rotation)
            self.obs[f"state/tf_rot2dtrigyaw_{source_frame}_to_{target_frame}"] = (
                np.array((np.sin(yaw), np.cos(yaw)), dtype=np.float32)
            )
        if RotationRepresentation.ROTMAT_COL_X in self.cfg.rotation_repr:
            if rotmat is None:
                rotmat = _quat_to_rotmat(normalized_rotation)
            self.obs[f"state/tf_rotmatcolx_{source_frame}_to_{target_frame}"] = rotmat[
                :, 0
            ]
        if RotationRepresentation.ROTMAT_COL_Y in self.cfg.rotation_repr:
            if rotmat is None:
                rotmat = _quat_to_rotmat(normalized_rotation)
            self.obs[f"state/tf_rotmatcoly_{source_frame}_to_{target_frame}"] = rotmat[
                :, 1
            ]
        if RotationRepresentation.ROTMAT_COL_Z in self.cfg.rotation_repr:
            if rotmat is None:
                rotmat = _quat_to_rotmat(normalized_rotation)
            self.obs[f"state/tf_rotmatcolz_{source_frame}_to_{target_frame}"] = rotmat[
                :, 2
            ]

    @staticmethod
    def _static_rotation_to_quat(
        rotation: tuple[float, float, float] | tuple[float, float, float, float],
    ) -> "Quaternion":
        if len(rotation) == 3:
            return RosTfInterface._rpy_to_quat(*rotation)

        from geometry_msgs.msg import Quaternion

        x, y, z, w = rotation
        return Quaternion(x=x, y=y, z=z, w=w)

    @staticmethod
    def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> "Quaternion":
        from geometry_msgs.msg import Quaternion

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)

        return Quaternion(
            w=cy * cr * cp + sy * sr * sp,
            x=cy * sr * cp - sy * cr * sp,
            y=cy * cr * sp + sy * sr * cp,
            z=sy * cr * cp - cy * sr * sp,
        )

    @staticmethod
    def _quat_to_yaw(quat: "Quaternion") -> float:
        return math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
        )

    @staticmethod
    def _rotmat_to_rot6d(rotmat: np.ndarray) -> np.ndarray:
        return rotmat[:, :2].flatten()
