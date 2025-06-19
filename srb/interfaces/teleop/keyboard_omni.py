import weakref
from collections.abc import Callable

import carb
import omni
from isaaclab.devices import Se3Keyboard as __Se3Keyboard


class OmniKeyboardTeleopInterface(__Se3Keyboard):
    def __str__(self) -> str:
        msg = super().__str__()

        msg += "\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tAdditional controls:\n"
        msg += "\tToggle gripper (alternative): R\n"
        return msg

    def _on_keyboard_event(self, event, *args, **kwargs) -> bool:
        ret = super()._on_keyboard_event(event, *args, **kwargs)

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:  # type: ignore
            if event.input.name == "R":
                self._close_gripper = not self._close_gripper

        return ret


class EventOmniKeyboardTeleopInterface:
    def __init__(self, callbacks: dict[str, Callable] = {}):
        self._appwindow = omni.appwindow.get_default_app_window()  # type: ignore
        self._input = carb.input.acquire_input_interface()  # type: ignore
        self._keyboard = self._appwindow.get_keyboard()
        self._keyboard_sub = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            lambda event, *args, obj=weakref.proxy(self): obj._on_keyboard_event(
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
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:  # type: ignore
            if event.input.name in self._additional_callbacks:
                self._additional_callbacks[event.input.name]()
        return True
