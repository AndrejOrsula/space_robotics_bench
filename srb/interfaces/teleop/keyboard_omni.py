import weakref
from collections.abc import Callable

import carb
import numpy as np
import omni
from isaaclab.devices import Se3Keyboard as __Se3Keyboard
from isaaclab.devices import Se3KeyboardCfg


class OmniKeyboardTeleopInterface(__Se3Keyboard):
    def __init__(
        self,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
    ):
        super().__init__(
            Se3KeyboardCfg(
                pos_sensitivity=pos_sensitivity,
                rot_sensitivity=rot_sensitivity,
            )
        )

    def advance(self) -> tuple[np.ndarray, bool]:
        command = super().advance()
        pose = command[:6].cpu().numpy().copy()
        # The base returns an axis-angle rotvec, but SRB consumers expect Euler
        # angles; _delta_rot retains the accumulated roll, pitch, and yaw.
        pose[3:6] = np.asarray(self._delta_rot, dtype=pose.dtype)
        return pose, command[6].item() < 0.0

    def __str__(self) -> str:
        msg = super().__str__()

        msg += "\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tAdditional controls:\n"
        msg += "\tToggle gripper (alternative): R\n"
        return msg

    def _on_keyboard_event(self, event, *args, **kwargs) -> bool:
        ret = super()._on_keyboard_event(event, *args, **kwargs)

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:  # type: ignore[attr-defined]
            if event.input.name == "R":
                self._close_gripper = not self._close_gripper

        return ret


class EventOmniKeyboardTeleopInterface:
    def __init__(self, callbacks: dict[str, Callable] | None = None):
        if callbacks is None:
            callbacks = {}
        self._appwindow = omni.appwindow.get_default_app_window()  # type: ignore[attr-defined]
        self._input = carb.input.acquire_input_interface()  # type: ignore[attr-defined]
        self._keyboard = self._appwindow.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            # Bind a weak proxy now; capturing ``self`` would create a cycle.
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(  # noqa: B008
                event, *args
            ),
        )
        self._additional_callbacks = callbacks

    def __del__(self):
        self._input.unsubscribe_from_keyboard_events(self._keyboard, self._keyboard_sub)
        self._keyboard_sub = None

    def add_callback(self, key: str, func: Callable):
        self._additional_callbacks[key] = func

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:  # type: ignore[attr-defined]
            if event.input.name in self._additional_callbacks:
                self._additional_callbacks[event.input.name]()
        return True
