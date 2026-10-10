"""Pure-numeric rotation helpers shared by the ROS hardware drivers."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from geometry_msgs.msg import Quaternion


def quat_to_rotmat(quat: Quaternion) -> np.ndarray:
    # Standard (x, y, z, w) → 3x3 rotation matrix. Matches scipy
    # Rotation.from_quat([x, y, z, w]).as_matrix() and tf2 transformations.
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    two_s = 2.0 / (x * x + y * y + z * z + w * w)
    return np.array(
        (
            1 - two_s * (y * y + z * z),
            two_s * (x * y - w * z),
            two_s * (x * z + w * y),
            two_s * (x * y + w * z),
            1 - two_s * (x * x + z * z),
            two_s * (y * z - w * x),
            two_s * (x * z - w * y),
            two_s * (y * z + w * x),
            1 - two_s * (x * x + y * y),
        ),
        dtype=np.float32,
    ).reshape((3, 3))
