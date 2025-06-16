from functools import cache, cached_property
from typing import Any, Dict, Iterable, Mapping, Sequence

import numpy

from srb.utils import logging


class HardwareInterface:
    ACTION_KEYS: Sequence[str] = ()
    CUSTOM_ALIASES: Sequence[Sequence[str]] = ()

    def __init__(self):
        pass

    @property
    def name(self) -> str:
        return self.__class__.__name__

    def start(self, rate: float = 1.0 / 50.0):
        logging.info(f"[{self.name}] Start")
        self._rate = rate

    def sync(self):
        logging.trace(f"[{self.name}] Sync")

    def reset(self):
        logging.debug(f"[{self.name}] Reset")

    def close(self):
        logging.info(f"[{self.name}] Close")

    def apply_action(self, action: Dict[str, numpy.ndarray]):
        raise NotImplementedError()

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        raise NotImplementedError()

    @property
    def reward(self) -> float:
        raise NotImplementedError()

    @property
    def termination(self) -> bool:
        raise NotImplementedError()

    @property
    def info(self) -> Dict[str, Any]:
        return {}

    ### Internal logic ###

    @cached_property
    def rate(self) -> float:
        if not hasattr(self, "_rate"):
            raise RuntimeError(
                f"Function '{self.name}.start()' must be called before accessing the rate."
            )
        return self._rate

    @cached_property
    def action_key_map(self) -> Mapping[str, str]:
        if not self._has_io_action:
            return {}
        return self._map_aliases(self.ACTION_KEYS)

    @cached_property
    def observation_key_map(self) -> Mapping[str, str]:
        if not self._has_io_observation:
            return {}
        return self._map_aliases(self.observation.keys())

    @cached_property
    def _has_io_action(self) -> bool:
        try:
            self.apply_action({})
        except NotImplementedError:
            return False
        return bool(self.ACTION_KEYS)

    @cached_property
    def _has_io_observation(self) -> bool:
        try:
            _ = self.observation
        except NotImplementedError:
            return False
        return True

    @cached_property
    def _has_io_reward(self) -> bool:
        try:
            _ = self.reward
        except NotImplementedError:
            return False
        return True

    @cached_property
    def _has_io_termination(self) -> bool:
        try:
            _ = self.termination
        except NotImplementedError:
            return False
        return True

    __COMMON_ALIASES: Sequence[Sequence[str]] = (
        ("accel", "acceleration", "accelerometer"),
        ("cmd_vel", "velocity_command"),
        ("depth", "depth_image", "depth_img", "depth_map"),
        ("ee_displacement", "ee_velocity"),
        ("ft", "ft_sensor", "force_torque_sensor"),
        ("gripper", "gripper_toggle"),
        ("gyro", "gyroscope", "angular_velocity"),
        ("img", "image", "rgb", "rgb_image", "rgb_img"),
        ("imu", "inertial_measurement_unit"),
        ("joint_position", "joint_positions"),
        ("joint_state", "joint_states"),
        ("joint_torque", "joint_torques"),
        ("joint_velocity", "joint_velocities"),
    )

    def _map_aliases(self, value: Iterable[str]) -> Mapping[str, str]:
        return {self._map_alias(val): val for val in value}

    def _map_alias(self, value: str) -> str:
        return self.__map_alias_cached(
            value, self.CUSTOM_ALIASES, self.__COMMON_ALIASES
        )

    @cache
    @staticmethod
    def __map_alias_cached(
        value: str,
        custom_aliases: Sequence[Sequence[str]],
        common_aliases: Sequence[Sequence[str]],
    ) -> str:
        for alias in custom_aliases:
            if value in alias:
                return alias[0]
        else:
            for alias in common_aliases:
                if value in alias:
                    return alias[0]
            else:
                return value

    def __str__(self) -> str:
        out = f"[{self.name}]\n" + f"  Rate: {self._rate} Hz\n"
        out += (
            "  Actions: "
            + (
                f"{','.join(self.action_key_map.keys())}"
                if self._has_io_action
                else "No"
            )
            + "\n"
        )
        out += (
            "  Observations: "
            + (
                f"{','.join(self.observation_key_map.keys())}"
                if self._has_io_observation
                else "No"
            )
            + "\n"
        )
        out += "  Reward: " + ("Yes" if self._has_io_reward else "No") + "\n"
        out += "  Termination: " + ("Yes" if self._has_io_termination else "No") + "\n"

        return out
