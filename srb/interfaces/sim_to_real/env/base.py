import time
from typing import Any, Dict, List, Mapping, Sequence, SupportsFloat, Tuple

import gymnasium
import numpy

from srb.interfaces.sim_to_real import HardwareInterface
from srb.utils import logging


class RealEnv(gymnasium.Env):
    ACTION_SPACE: Dict[str, gymnasium.Space] = {}
    OBSERVATION_SPACE: Dict[str, gymnasium.Space] = {}
    ACTION_RATE: float = 1.0 / 50.0

    _MIN_SLEEP_TIME: float = 1.0 / 1000.0
    _FREQ_EST_EMA_ALPHA: float = 0.9
    _FREQ_EST_EMA_BETA: float = 1.0 - _FREQ_EST_EMA_ALPHA

    def __init__(self, hardware: Sequence[HardwareInterface], **kwargs):
        super().__init__(**kwargs)

        # Categorize all hardware interfaces
        self._hardware: List[HardwareInterface] = list(hardware)
        self._sink_action: List[HardwareInterface] = []
        self._src_observation: List[HardwareInterface] = []
        self._src_reward: List[HardwareInterface] = []
        self._src_termination: List[HardwareInterface] = []
        for i, hw in enumerate(self._hardware):
            logging.info(f"Hardware interface #{i}: {hw.name}")
            if hw._has_io_action:
                self._sink_action.append(hw)
            if hw._has_io_observation:
                self._src_observation.append(hw)
            if hw._has_io_reward:
                self._src_reward.append(hw)
            if hw._has_io_termination:
                self._src_termination.append(hw)
        logging.info(
            f"Action interfaces: {', '.join(hw.name for hw in self._sink_action)}\n"
            f"Observation interfaces: {', '.join(hw.name for hw in self._src_observation)}\n"
            f"Reward interfaces: {', '.join(hw.name for hw in self._src_reward)}\n"
            f"Termination interfaces: {', '.join(hw.name for hw in self._src_termination)}"
        )

        # Map each action and observation to a single hardware interface
        self._hardware_action_map: Dict[HardwareInterface, List[Tuple[str, str]]] = {}
        for action_key, action_space in self.ACTION_SPACE.items():
            _found_action_hw: HardwareInterface | None = None
            for hw in self._sink_action:
                alias_key = hw._map_alias(action_key)
                for kw_alias_key, hw_target_key in hw.action_key_map.items():
                    if kw_alias_key is alias_key:
                        if _found_action_hw is not None:
                            raise ValueError(
                                f'Action key "{action_key}" must have a unique hardware interface mapping but two were found: {_found_action_hw.name} and {hw.name}'
                            )
                        if hw not in self._hardware_action_map:
                            self._hardware_action_map[hw] = []
                        self._hardware_action_map[hw].append(
                            (
                                action_key,
                                hw_target_key,
                            )
                        )
                        _found_action_hw = hw

                        hw_action_space = hw.SUPPORTED_ACTION_SPACES[hw_target_key]
                        if not action_space.shape == hw_action_space.shape:
                            raise ValueError(
                                f'Action "{action_key}" from hardware "{hw.name}" does not match expected space "{action_space}" with its shape "{hw_action_space.shape}"'
                            )
                        if not action_space.dtype == hw_action_space.dtype:
                            raise ValueError(
                                f'Action "{action_key}" from hardware "{hw.name}" does not match expected dtype "{action_space.dtype}" with its dtype "{hw_action_space.dtype}"'
                            )
            if _found_action_hw is None:
                raise ValueError(
                    f'Action key "{action_key}" must be mapped to a hardware interface but no matches were found'
                )
        self._hardware_observation_map: Dict[
            HardwareInterface, List[Tuple[str, str]]
        ] = {}
        for obs_key, obs_space in self.OBSERVATION_SPACE.items():
            _found_obs_hw: HardwareInterface | None = None
            for hw in self._src_observation:
                alias_key = hw._map_alias(obs_key)
                for kw_alias_key, hw_target_key in hw.observation_key_map.items():
                    if kw_alias_key is alias_key:
                        if _found_obs_hw is not None:
                            raise ValueError(
                                f'Observation key "{obs_key}" must have a unique hardware interface mapping but two were found: {_found_obs_hw.name} and {hw.name}'
                            )
                        if hw not in self._hardware_observation_map:
                            self._hardware_observation_map[hw] = []
                        self._hardware_observation_map[hw].append(
                            (
                                obs_key,
                                hw_target_key,
                            )
                        )
                        _found_obs_hw = hw

                        if not obs_space.contains(hw.observation[hw_target_key]):
                            raise ValueError(
                                f'Observation "{obs_key}" from hardware "{hw.name}" does not match expected space "{obs_space}" with its shape "{hw.observation[hw_target_key].shape}" and dtype "{hw.observation[hw_target_key].dtype}"'
                            )
            if _found_obs_hw is None:
                raise ValueError(
                    f'Observation key "{obs_key}" must be mapped to a hardware interface but no matches were found'
                )

        # Start all hardware interfaces
        for hw in self._hardware:
            hw.start(rate=self.ACTION_RATE)
            hw.sync()

        # Misc
        self._extract_time_average: float = 0.0

    def step(
        self, action: Dict[str, numpy.ndarray] | numpy.ndarray
    ) -> Tuple[
        Dict[str, numpy.ndarray],
        SupportsFloat,
        bool,
        bool,
        Dict[str, Any],
    ]:
        # Apply action
        pre_action_time: float = time.time()
        if isinstance(action, Mapping):
            for hw, keys in self._hardware_action_map.items():
                hw.apply_action({key_pair[1]: action[key_pair[0]] for key_pair in keys})
        else:
            for hw in self._sink_action:
                hw.apply_action({"obs": action})

        # Maintain constant action rate
        action_time: float = time.time() - pre_action_time
        sleep_time: float = self.ACTION_RATE - action_time - self._extract_time_average
        if sleep_time > self._MIN_SLEEP_TIME:
            time.sleep(sleep_time)
        else:
            logging.warning(
                f"Action rate of {self.ACTION_RATE} cannot be maintained with a remaining sleep time of {sleep_time:.3f} s (below the minimum threshold of {self._MIN_SLEEP_TIME:.3f} s). The actual action rate closer to {(1.0 / (action_time + self._extract_time_average)):.3f} Hz..."
            )

        # Extract observations, rewards, terminations, and info
        pre_extract_time: float = time.time()
        for hw in self._hardware:
            hw.sync()
        observation: Dict[str, numpy.ndarray] = {}
        for hw, keys in self._hardware_observation_map.items():
            observation.update(
                {key_pair[1]: hw.observation[key_pair[0]] for key_pair in keys}
            )
        reward: float = 0.0
        for hw in self._src_reward:
            reward += hw.reward
        terminated: bool = False
        for hw in self._src_termination:
            if hw.termination:
                terminated = True
                break
        info: Dict[str, Any] = {hw.name: hw.info for hw in self._hardware}
        self._extract_time_average = (
            self._FREQ_EST_EMA_ALPHA * self._extract_time_average
            + self._FREQ_EST_EMA_BETA * (time.time() - pre_extract_time)
        )

        # Reset episode if terminated
        if terminated:
            logging.info("Resetting environment due to termination")
            self.reset()

        return observation, reward, terminated, False, info

    def reset(self, **kwargs) -> Tuple[Dict[str, numpy.ndarray], Dict[str, Any]]:
        super().reset(**kwargs)

        # Reset all hardware interfaces
        for hw in self._hardware:
            hw.reset()

        # Extract initial observations
        observation: Dict[str, numpy.ndarray] = {}
        for hw, keys in self._hardware_observation_map.items():
            observation.update(
                {key_pair[1]: hw.observation[key_pair[0]] for key_pair in keys}
            )

        # Extract info
        info: Dict[str, Any] = {hw.name: hw.info for hw in self._hardware}

        return observation, info

    def close(self):
        super().close()

        # Close all hardware interfaces
        for hw in self._hardware:
            hw.close()
