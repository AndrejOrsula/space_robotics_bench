from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Callable
from importlib import import_module
from typing import TYPE_CHECKING

import numpy as np

from srb.utils import logging
from srb.utils.failures import attempt_cleanup

if TYPE_CHECKING:
    import pyspacemouse


def __getattr__(name: str):
    if name == "pyspacemouse":
        return import_module(name)
    raise AttributeError(name)


def _driver():
    """Import the optional driver on first use (F-INTERFACES-08).

    A caller-bound module attribute (used by tests) takes precedence.
    """
    driver = globals().get("pyspacemouse")
    return driver if driver is not None else import_module("pyspacemouse")


# Driver: https://github.com/FreeSpacenav/spacenavd
class SpacemouseImplementation:
    def __init__(
        self,
        pos_sensitivity: float = 1.0,
        rot_sensitivity: float = 3.1415927,
        rate: float = 1000.0,
        _strict: bool = False,
        motion_timeout: float = 0.5,
    ):
        """Create a SpaceMouse reader.

        Cached motion expires after ``motion_timeout`` seconds without a
        successful read. Freshness uses reads because some devices stop
        reporting while the puck is held steady. Default: 0.5 seconds.
        """
        # Validate the Hz rate before setup; invalid values break thread pacing.
        if isinstance(rate, bool) or not math.isfinite(rate) or rate <= 0.0:
            raise ValueError(
                f"SpaceMouse rate must be finite and positive, got {rate!r}"
            )
        if (
            isinstance(motion_timeout, bool)
            or not math.isfinite(motion_timeout)
            or motion_timeout <= 0.0
        ):
            raise ValueError(
                "SpaceMouse motion timeout must be finite and positive, "
                f"got {motion_timeout!r}"
            )
        try:
            _driver()
        except ImportError as exc:
            raise ImportError(
                "SpaceMouse teleop requires pyspacemouse; "
                "install the 'spacemouse' extra"
            ) from exc
        self._strict = _strict
        self._device_open = False
        self._device_handle = None
        self._hid_handle = None
        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity
        self.sleep_rate = 1.0 / rate
        self.motion_timeout = motion_timeout
        self._clock = time.monotonic

        # Command buffers
        self._close_gripper = False
        self._delta_pos = np.zeros(3)  # (x, y, z)
        self._delta_rot = np.zeros(3)  # (roll, pitch, yaw)
        self._last_successful_read: float | None = None
        self._additional_callbacks = {}
        self._prev_buttons = (False, False)
        self._running = False
        self._closed = False
        self._motion_lock = threading.RLock()
        self._motion_generation = 0
        self._active_read_generation: int | None = None
        self._reader_thread_ident: int | None = None

        # Open the device
        try:
            if self._strict:
                handle = self._open_native_device()
            else:
                handle = _driver().open(
                    dof_callback=self._cb_dof,  # type: ignore[misc]
                    button_callback=self._cb_button,  # type: ignore[misc]
                )
            if handle:
                self._device_open = True
                if self._strict:
                    self._device_handle = handle
                # Run a background thread for the device
                with self._motion_lock:
                    self._running = True
                self._thread = threading.Thread(target=self._run_device)
                self._thread.daemon = True
                self._thread.start()
            else:
                logging.error("Failed to open a SpaceMouse device. Is it connected?")
                if self._strict:
                    raise RuntimeError("Failed to open a SpaceMouse device")
        except Exception as e:
            logging.error(f"Failed to open a SpaceMouse device. Is it connected?\n{e}")
            if self._strict:
                raise RuntimeError("Failed to open a SpaceMouse device") from e

    def _open_native_device(self):
        """Adopt the driver's spec and HID before any fallible open operation."""
        pyspacemouse = _driver()
        device_names = pyspacemouse.list_devices()
        if not device_names:
            raise RuntimeError("No connected or supported SpaceMouse devices found.")

        device_name = device_names[0]
        spec = pyspacemouse.device_specs[device_name]
        for hid in pyspacemouse.Enumeration().find() or ():
            if (hid.vendor_id, hid.product_id) != tuple(spec.hid_id):
                continue

            handle = copy.deepcopy(spec)
            handle.device = hid
            handle.dof_callback = self._cb_dof
            handle.button_callback = self._cb_button
            handle.set_nonblocking_loop = True

            # Retain both owners before open, which may fail after HID.open().
            self._device_handle = handle
            self._hid_handle = hid
            handle.open()
            hid.set_nonblocking(True)
            return handle
        return None

    def __del__(self):
        self.close()

    def close(self) -> None:
        motion_lock = getattr(self, "_motion_lock", None)
        if motion_lock is None:
            # Initialization did not reach device open, so nothing needs release.
            self._running = False
            self._closed = True
            return
        with motion_lock:
            self._running = False
            self._closed = True
            self._motion_generation = getattr(self, "_motion_generation", 0) + 1
            self._clear_motion_locked()
        strict = getattr(self, "_strict", False)
        errors = []
        # Stop the reader before freeing a handle it may still use.
        thread = getattr(self, "_thread", None)
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=0.25)
            if thread.is_alive():
                errors.append(RuntimeError("SpaceMouse reader thread did not stop"))
        if errors:
            # Keep the device open for retry; the reader may still use it.
            if strict:
                raise ExceptionGroup("SpaceMouse cleanup incomplete", errors)
            logging.error("SpaceMouse reader thread did not stop; device left open")
            return
        if strict:
            handle = getattr(self, "_device_handle", None)
            if handle is not None:
                _, error = attempt_cleanup(handle.close)
                if isinstance(error, Exception):
                    errors.append(error)
                elif error is not None:
                    raise error
                else:
                    self._device_handle = None
                    self._hid_handle = None
                    self._device_open = False
        else:
            _, error = attempt_cleanup(_driver().close)
            if error is not None:
                if not isinstance(error, Exception):
                    raise error
                errors.append(error)
            else:
                self._device_open = False
        if errors:
            if strict:
                raise ExceptionGroup("SpaceMouse cleanup incomplete", errors)
            logging.error("SpaceMouse cleanup incomplete: %s", errors[0])

    def __str__(self) -> str:
        msg = f"Spacemouse Controller ({self.__class__.__name__})\n"
        msg += "\tToggle gripper (alternative): Right button\n"
        msg += "\tReset: Left button\n"
        return msg

    def reset(self):
        with self._motion_lock:
            if self._closed:
                return
            self._motion_generation += 1
            self._reset_state_locked()

    def _clear_motion_locked(self) -> None:
        self._delta_pos = np.zeros(3)
        self._delta_rot = np.zeros(3)
        self._last_successful_read = None

    def _reset_state_locked(self) -> None:
        self._close_gripper = False
        self._clear_motion_locked()

    def _callback_allowed_locked(self) -> bool:
        return (
            not self._closed
            and self._active_read_generation is not None
            and self._active_read_generation == self._motion_generation
            and self._reader_thread_ident == threading.get_ident()
        )

    def _finish_read(self, generation: int, *, failed: bool) -> None:
        with self._motion_lock:
            if self._active_read_generation == generation:
                self._active_read_generation = None
                self._reader_thread_ident = None
            if self._closed or self._motion_generation != generation:
                return
            if failed:
                self._clear_motion_locked()
            else:
                self._last_successful_read = self._clock()

    def add_callback(self, key: str, func: Callable):
        if key not in ["L", "R", "LR"]:
            raise ValueError(
                f"Only left (L), right (R), and right-left (LR) buttons supported. Provided: {key}."
            )
        with self._motion_lock:
            self._additional_callbacks[key] = func

    def advance(self) -> tuple[np.ndarray, bool]:
        # Return Euler deltas expected by ROS, haptic, keyboard, policy, and
        # direct consumers; axis-angle conversion corrupts multi-axis rotation.
        with self._motion_lock:
            if (
                self._last_successful_read is not None
                and self._clock() - self._last_successful_read >= self.motion_timeout
            ):
                self._clear_motion_locked()
            return (
                np.concatenate([self._delta_pos, self._delta_rot]),
                self._close_gripper,
            )

    def _run_device(self):
        while True:
            with self._motion_lock:
                if not self._running or self._closed:
                    return
                generation = self._motion_generation
                self._active_read_generation = generation
                self._reader_thread_ident = threading.get_ident()

            try:
                if self._strict:
                    handle = self._device_handle
                    state = handle.read() if handle is not None else None
                else:
                    state = _driver().read()
            except Exception as exc:
                self._finish_read(generation, failed=True)
                logging.error(f"Failed to read SpaceMouse input.\n{exc}")
            else:
                self._finish_read(generation, failed=state is None)
            time.sleep(self.sleep_rate)

    def _cb_dof(self, state: pyspacemouse.SpaceNavigator):
        with self._motion_lock:
            if not self._callback_allowed_locked():
                return
            self._delta_pos = np.array(
                [
                    state.y * self.pos_sensitivity,
                    -state.x * self.pos_sensitivity,
                    state.z * self.pos_sensitivity,
                ]
            )
            self._delta_rot = np.array(
                [
                    -state.roll * self.rot_sensitivity,
                    -state.pitch * self.rot_sensitivity,
                    -state.yaw * self.rot_sensitivity,
                ]
            )
            self._last_successful_read = self._clock()

    def _cb_button(self, state: pyspacemouse.SpaceNavigator, buttons: list[bool]):
        with self._motion_lock:
            # The driver reports the full button vector on every button change;
            # act on rising edges so a held button does not repeat its action.
            left = bool(buttons[0])
            right = len(buttons) > 1 and bool(buttons[1])
            prev_left, prev_right = self._prev_buttons
            self._prev_buttons = (left, right)
            left_pressed = left and not prev_left
            right_pressed = right and not prev_right
            both_pressed = left and right and not (prev_left and prev_right)
            if not self._callback_allowed_locked():
                return
            left_callback = (
                self._additional_callbacks.get("L") if left_pressed else None
            )
            if left_pressed:
                self._motion_generation += 1
                self._reset_state_locked()
            button_generation = self._motion_generation
        if left_callback is not None:
            left_callback()

        with self._motion_lock:
            if self._closed or self._motion_generation != button_generation:
                return
            right_callback = (
                self._additional_callbacks.get("R") if right_pressed else None
            )
            if right_pressed:
                self._close_gripper = not self._close_gripper
        if right_callback is not None:
            right_callback()

        with self._motion_lock:
            if (
                self._closed
                or self._motion_generation != button_generation
                or not both_pressed
            ):
                return
            both_callback = self._additional_callbacks.get("LR")
        if both_callback is not None:
            both_callback()
