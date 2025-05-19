import threading
import time
from collections.abc import Callable
from typing import List

import numpy
import pyspacemouse
from scipy.spatial.transform import Rotation

from srb.interfaces.teleop import DeviceBase
from srb.utils import logging


# Driver: https://github.com/FreeSpacenav/spacenavd
class SpacemouseTeleopInterface(DeviceBase):
    def __init__(
        self,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
        rate: float = 1000.0,
    ):
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        self.sleep_rate = 1.0 / rate

        # Command buffers
        self._close_gripper = False
        self._delta_pos = numpy.zeros(3)  # (x, y, z)
        self._delta_rot = numpy.zeros(3)  # (roll, pitch, yaw)
        self._additional_callbacks = {}

        # Open the device
        try:
            success = pyspacemouse.open(
                dof_callback=self._cb_dof,  # type: ignore
                button_callback=self._cb_button,  # type: ignore
            )
            if success:
                # Run a background thread for the device
                self._thread = threading.Thread(target=self._run_device)
                self._thread.daemon = True
                self._thread.start()
            else:
                logging.error(
                    "Failed to open a SpaceMouse device. Is it connected?",
                )
        except Exception as e:
            logging.error(
                f"Failed to open a SpaceMouse device. Is it connected?\n{e}",
            )

    def __del__(self):
        if hasattr(self, "_thread"):
            self._thread.join()

    def __str__(self) -> str:
        msg = f"Spacemouse Controller ({self.__class__.__name__})\n"
        msg += "\tToggle gripper (alternative): Right button\n"
        msg += "\tReset: Left button\n"
        return msg

    def reset(self):
        self._close_gripper = False
        self._delta_pos = numpy.zeros(3)
        self._delta_rot = numpy.zeros(3)

    def add_callback(self, key: str, func: Callable):
        if key not in ["L", "R", "LR"]:
            raise ValueError(
                f"Only left (L), right (R), and right-left (LR) buttons supported. Provided: {key}."
            )
        self._additional_callbacks[key] = func

    def advance(self) -> tuple[numpy.ndarray, bool]:
        rot_vec = Rotation.from_euler("XYZ", self._delta_rot).as_rotvec()
        return numpy.concatenate([self._delta_pos, rot_vec]), self._close_gripper

    def _run_device(self):
        while True:
            _state = pyspacemouse.read()
            time.sleep(self.sleep_rate)

    def _cb_dof(self, state: pyspacemouse.SpaceNavigator):
        self._delta_pos = numpy.array(
            [
                state.y * self.pos_sensitivity,
                -state.x * self.pos_sensitivity,
                state.z * self.pos_sensitivity,
            ]
        )
        self._delta_rot = numpy.array(
            [
                -state.roll * self.rot_sensitivity,
                -state.pitch * self.rot_sensitivity,
                -state.yaw * self.rot_sensitivity,
            ]
        )

    def _cb_button(self, state: pyspacemouse.SpaceNavigator, buttons: List[bool]):
        if buttons[0]:
            self.reset()
            if "L" in self._additional_callbacks.keys():
                self._additional_callbacks["L"]()
        if buttons[1]:
            self._close_gripper = not self._close_gripper
            if "R" in self._additional_callbacks.keys():
                self._additional_callbacks["R"]()
        if all(buttons):
            if "LR" in self._additional_callbacks.keys():
                self._additional_callbacks["LR"]()
