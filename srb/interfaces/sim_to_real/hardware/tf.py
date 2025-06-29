import math
import time
from enum import Enum, auto
from typing import Any, Dict, Sequence, Set

import numpy
from typing_extensions import Self

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging


class RotationRepresentation(Enum):
    ROTMAT = auto()
    ROT_6D = auto()
    QUAT_WXYZ = auto()

    def __str__(self) -> str:
        return self.name.lower()

    @classmethod
    def from_str(cls, string: str) -> Self | None:
        return next(
            (variant for variant in cls if string.upper() == variant.name), None
        )


class TfInterfaceCfg(HardwareInterfaceCfg):
    timeout_duration: float = 0.2
    discovery_interval: float = 1.0
    rotation_repr: Sequence[RotationRepresentation] = (RotationRepresentation.ROT_6D,)

    allowlist: Sequence[str] = ()
    blocklist: Sequence[str] = ("world", "map")


class TfInterface(HardwareInterface):
    cfg: TfInterfaceCfg

    def __init__(self, cfg: TfInterfaceCfg = TfInterfaceCfg()):
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

    def _discover_frames(self):
        current_time = time.time()
        if current_time - self.last_discovery_time < self.cfg.discovery_interval:
            return
        self.last_discovery_time = current_time

        new_frames = set()
        for line in self.tf_buffer.all_frames_as_string().split("\n"):
            if line.strip():
                frame_name = line.split()[0].strip(":")
                if (
                    frame_name
                    and self._is_frame_allowed(frame_name)
                    and frame_name not in self.discovered_frames
                ):
                    new_frames.add(frame_name)

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
                self.obs[f"state/tf_pos_{source_frame}_to_{target_frame}"] = (
                    numpy.zeros(3, dtype=numpy.float32)
                )

                ## Rotation
                if RotationRepresentation.ROTMAT in self.cfg.rotation_repr:
                    # Rotation matrix observation
                    self.obs[f"state/tf_rotmat_{source_frame}_to_{target_frame}"] = (
                        numpy.eye(3, dtype=numpy.float32).flatten()
                    )
                if RotationRepresentation.ROT_6D in self.cfg.rotation_repr:
                    # 6D rotation observation
                    self.obs[f"state/tf_rot6d_{source_frame}_to_{target_frame}"] = (
                        numpy.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=numpy.float32)
                    )
                if RotationRepresentation.QUAT_WXYZ in self.cfg.rotation_repr:
                    # Quaternion observation
                    self.obs[f"state/tf_quat_{source_frame}_to_{target_frame}"] = (
                        numpy.array([1.0, 0.0, 0.0, 0.0], dtype=numpy.float32)
                    )

    def _update_transforms(self):
        current_time = self.ros_node.get_clock().now()
        for source_frame in self.discovered_frames:
            for target_frame in self.discovered_frames:
                if source_frame == target_frame:
                    continue

                tf_stamped = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    current_time,
                    timeout=self.tf_timeout_duration,
                )
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

    def _quaternion_to_rot6d(self, quat: numpy.ndarray) -> numpy.ndarray:
        # Normalize quaternion
        quat = quat / numpy.linalg.norm(quat)
        w, x, y, z = quat

        # Convert to rotation matrix
        rot_matrix = numpy.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ],
            dtype=numpy.float32,
        )

        # Extract first two columns for 6D representation
        rot6d = rot_matrix[:, :2].flatten()
        return rot6d

    @staticmethod
    def _quat_to_rotmat(quat) -> numpy.ndarray:
        r, i, j, k = quat[0], quat[1], quat[2], quat[3]
        two_s = 2.0 / (quat * quat).sum()
        return numpy.stack(
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
            )
        ).reshape((3, 3))

    @staticmethod
    def _rotmat_to_rot6d(rotmat: numpy.ndarray) -> numpy.ndarray:
        return rotmat[:, :2].reshape((6,))

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        return self.obs.copy()

    @property
    def info(self) -> Dict[str, Any]:
        return {
            "frames": self.discovered_frames,
        }
