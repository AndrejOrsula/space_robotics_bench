import numpy as np
from isaaclab.devices import Se3Gamepad as __Se3Gamepad
from isaaclab.devices import Se3GamepadCfg
from scipy.spatial.transform import Rotation


class GamepadTeleopInterface(__Se3Gamepad):
    def __init__(
        self,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 1.6,
    ):
        super().__init__(
            Se3GamepadCfg(
                pos_sensitivity=pos_sensitivity,
                rot_sensitivity=rot_sensitivity,
            )
        )

    def advance(self) -> tuple[np.ndarray, bool]:
        command = super().advance().cpu().numpy()
        pose = command[:6].copy()
        # The base returns an axis-angle rotvec; SRB consumers expect the
        # XYZ Euler angles it was built from (as for the keyboard).
        pose[3:6] = Rotation.from_rotvec(command[3:6]).as_euler("XYZ")
        return pose, bool(command[6] < 0.0)
