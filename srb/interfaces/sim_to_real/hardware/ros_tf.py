import math
import time
from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Dict, Sequence, Set

import numpy
from typing_extensions import Self

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging

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
    QUAT_WXYZ = auto()
    ROTMAT = auto()
    ROT_6D = auto()
    ROT_2D_TRIG_YAW = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class RosTfInterfaceCfg(HardwareInterfaceCfg):
    timeout_duration: float = 0.2
    discovery_interval: float = 1.0
    position_repr: Sequence[PositionRepresentation] = (PositionRepresentation.POS_2D,)
    rotation_repr: Sequence[RotationRepresentation] = (
        RotationRepresentation.ROT_2D_TRIG_YAW,
    )

    # TODO[high]: Remove hardcoded values
    allowlist: Sequence[str] = ("target", "robot")
    blocklist: Sequence[str] = ("world", "map")


class RosTfInterface(HardwareInterface):
    cfg: RosTfInterfaceCfg

    def __init__(self, cfg: RosTfInterfaceCfg = RosTfInterfaceCfg()):
        super().__init__(cfg)

        self.last_discovery_time: float = 0.0
        self.discovered_frames: Set[str] = set()
        self.obs: Dict[str, numpy.ndarray] = {}

    def start(self, **kwargs):
        super().start(**kwargs)
        from rclpy.duration import Duration
        from tf2_ros import Buffer, TransformListener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.ros_node)
        self.tf_timeout_duration = Duration(
            seconds=math.floor(self.cfg.timeout_duration),
            nanoseconds=int((self.cfg.timeout_duration % 1) * 1e9),
        )

    def close(self):
        super().close()
        self.tf_listener.unregister()
        self.tf_buffer.clear()

    def sync(self):
        super().sync()
        self._discover_frames()
        self._update_transforms()

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return self.obs.copy()

    @property
    def info(self) -> Dict[str, Any]:
        return {
            "ros_tf/frames": self.discovered_frames,
        }

    def _discover_frames(self):
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
            child_frame, parent_frame = line.split(" exists with parent ", 1)
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

    def _initialize_new_frames(self, new_frames: Set[str]):
        self.discovered_frames.update(new_frames)
        for source_frame in self.discovered_frames:
            for target_frame in self.discovered_frames:
                if (source_frame == target_frame) or (
                    source_frame not in new_frames and target_frame not in new_frames
                ):
                    continue

                ## Position
                if PositionRepresentation.POS_3D in self.cfg.position_repr:
                    # 3D position observation (x, y, z)
                    self.obs[f"state/tf_pos_{source_frame}_to_{target_frame}"] = (
                        numpy.zeros(3, dtype=numpy.float32)
                    )
                if PositionRepresentation.POS_2D in self.cfg.position_repr:
                    # 2D position observation (x, y)
                    self.obs[f"state/tf_pos2d_{source_frame}_to_{target_frame}"] = (
                        numpy.zeros(2, dtype=numpy.float32)
                    )

                ## Rotation
                if RotationRepresentation.QUAT_WXYZ in self.cfg.rotation_repr:
                    # Quaternion observation
                    self.obs[f"state/tf_quat_{source_frame}_to_{target_frame}"] = (
                        numpy.array((1.0, 0.0, 0.0, 0.0), dtype=numpy.float32)
                    )
                if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
                    # Rotation matrix observation
                    self.obs[f"state/tf_rotmat_{source_frame}_to_{target_frame}"] = (
                        numpy.eye(3, dtype=numpy.float32).flatten()
                    )
                if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
                    # 6D rotation observation
                    self.obs[f"state/tf_rot6d_{source_frame}_to_{target_frame}"] = (
                        numpy.array((1.0, 0.0, 0.0, 0.0, 1.0, 0.0), dtype=numpy.float32)
                    )
                if RotationRepresentation.ROT_2D_TRIG_YAW in self.cfg.rotation_repr:
                    # 2D rotation observation (sin(yaw), cos(yaw))
                    self.obs[
                        f"state/tf_rot2dtrigyaw_{source_frame}_to_{target_frame}"
                    ] = numpy.array((0.0, 1.0), dtype=numpy.float32)

    def _update_transforms(self):
        current_time = self.ros_node.get_clock().now()
        for source_frame in self.discovered_frames:
            for target_frame in self.discovered_frames:
                if source_frame == target_frame:
                    continue

                try:
                    tf_stamped = self.tf_buffer.lookup_transform(
                        source_frame,
                        target_frame,
                        current_time,
                        timeout=self.tf_timeout_duration,
                    )
                except Exception as e:
                    logging.warning(
                        f"[{self.name}] Failed to get transform from {source_frame} to {target_frame}: {e}"
                    )
                    continue

                ## Position
                if PositionRepresentation.POS_3D in self.cfg.position_repr:
                    self.obs[f"state/tf_pos_{source_frame}_to_{target_frame}"] = (
                        numpy.array(
                            (
                                tf_stamped.transform.translation.x,
                                tf_stamped.transform.translation.y,
                                tf_stamped.transform.translation.z,
                            ),
                            dtype=numpy.float32,
                        )
                    )
                if PositionRepresentation.POS_2D in self.cfg.position_repr:
                    self.obs[f"state/tf_pos2d_{source_frame}_to_{target_frame}"] = (
                        numpy.array(
                            (
                                tf_stamped.transform.translation.x,
                                tf_stamped.transform.translation.y,
                            ),
                            dtype=numpy.float32,
                        )
                    )

                ## Rotation
                if RotationRepresentation.QUAT_WXYZ in self.cfg.rotation_repr:
                    self.obs[f"state/tf_quat_{source_frame}_to_{target_frame}"] = (
                        numpy.array(
                            (
                                tf_stamped.transform.rotation.w,
                                tf_stamped.transform.rotation.x,
                                tf_stamped.transform.rotation.y,
                                tf_stamped.transform.rotation.z,
                            ),
                            dtype=numpy.float32,
                        )
                    )
                if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
                    self.obs[f"state/tf_rotmat_{source_frame}_to_{target_frame}"] = (
                        self._quat_to_rotmat(tf_stamped.transform.rotation)
                    )
                if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
                    self.obs[f"state/tf_rot6d_{source_frame}_to_{target_frame}"] = (
                        self._rotmat_to_rot6d(
                            self._quat_to_rotmat(tf_stamped.transform.rotation)
                        )
                    )
                if RotationRepresentation.ROT_2D_TRIG_YAW in self.cfg.rotation_repr:
                    yaw = self._quat_to_yaw(tf_stamped.transform.rotation)
                    self.obs[
                        f"state/tf_rot2dtrigyaw_{source_frame}_to_{target_frame}"
                    ] = numpy.array(
                        (numpy.sin(yaw), numpy.cos(yaw)), dtype=numpy.float32
                    )

    @staticmethod
    def _quat_to_yaw(quat: "Quaternion") -> float:
        return math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
        )

    @staticmethod
    def _quat_to_rotmat(quat: "Quaternion") -> numpy.ndarray:
        r, i, j, k = quat.w, quat.x, quat.y, quat.z
        two_s = 2.0 / (r * r + i * i + j * j + k * k)
        return numpy.array(
            (
                1 - two_s * (j * j + k * k),
                two_s * (i * j - k * r),
                two_s * (i * k + j * r),
                two_s * (i * j + k * r),
                1 - two_s * (i * i + k * k),
                two_s * (j * k - i * r),
                two_s * (i * k - j * r),
                two_s * (j * k + i * r),
                1 - two_s * (i * i + j * j),
            ),
            dtype=numpy.float32,
        ).reshape((3, 3))

    @staticmethod
    def _rotmat_to_rot6d(rotmat: numpy.ndarray) -> numpy.ndarray:
        return rotmat[:, :2].flatten()
