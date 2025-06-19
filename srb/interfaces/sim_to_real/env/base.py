import time
from threading import Thread
from typing import Any, ClassVar, Dict, List, Mapping, Sequence, SupportsFloat, Tuple

import gymnasium
import numpy
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as RosNode

from srb.interfaces.sim_to_real.hardware import HardwareInterface
from srb.utils import logging


class RealEnv(gymnasium.Env):
    SINGLE_ACTION_SPACE: ClassVar[gymnasium.Space]
    DETAILED_ACTION_SPACE: ClassVar[gymnasium.spaces.Dict]
    ACTION_ORDERING: ClassVar[Dict[str, slice]]
    ACTION_RATE: ClassVar[float]
    ACTION_SCALE: ClassVar[Dict[str, float]]

    SINGLE_OBSERVATION_SPACE: ClassVar[gymnasium.Space]
    DETAILED_OBSERVATION_SPACE: ClassVar[gymnasium.spaces.Dict]

    ROBOT: ClassVar[str]

    _MIN_SLEEP_DURATION: ClassVar[float] = 1.0 / 1000.0
    _FREQ_EST_EMA_ALPHA: ClassVar[float] = 0.9

    def __init__(
        self,
        hardware: Sequence[HardwareInterface],
        ros_node: RosNode | None = None,
        **kwargs,
    ):
        super().__init__(**kwargs)

        # Map extracted spaces to standard interfaces
        self._single_action_space = self.SINGLE_ACTION_SPACE
        self._action_space = gymnasium.vector.utils.batch_space(
            self.single_action_space, 1
        )
        self._single_observation_space = self.SINGLE_OBSERVATION_SPACE
        self._observation_space = gymnasium.vector.utils.batch_space(
            self.single_observation_space, 1
        )

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

        # Map each action to a single hardware interface
        self._hardware_action_map: Dict[HardwareInterface, List[Tuple[str, str]]] = {}
        if isinstance(self.DETAILED_ACTION_SPACE, gymnasium.spaces.Dict):
            for action_key, action_space in self.DETAILED_ACTION_SPACE.spaces.items():
                self._map_action_to_hardware(action_key, action_space)
        else:
            raise ValueError(
                f"Unexpected action space type: {self.DETAILED_ACTION_SPACE}"
            )

        # Map each observation to a single hardware interface
        self._hardware_observation_map: Dict[
            HardwareInterface, List[Tuple[str, str]]
        ] = {}
        if isinstance(self.DETAILED_OBSERVATION_SPACE, gymnasium.spaces.Dict):
            self._map_observations_recursive("", self.DETAILED_OBSERVATION_SPACE.spaces)
        else:
            raise ValueError(
                f"Unexpected observation space type: {self.DETAILED_OBSERVATION_SPACE}"
            )

        # Initialize ROS node
        if not ros_node:
            rclpy.init()
            self._ros_node = RosNode(
                "real", namespace="srb", start_parameter_services=False
            )
        else:
            self._ros_node = ros_node

        # Start all hardware interfaces
        for hw in self._hardware:
            if hw in self._hardware_action_map.keys():
                action_scale = {}
                for action_key, hw_target_key in self._hardware_action_map[hw]:
                    # Handle hierarchical action keys (e.g., "robot__thrust" -> "thrust")
                    base_action_key = (
                        action_key.split("__")[-1] if "__" in action_key else action_key
                    )
                    action_scale[hw_target_key] = self.ACTION_SCALE.get(
                        action_key, self.ACTION_SCALE.get(base_action_key, 1.0)
                    )
            else:
                action_scale = {}
            hw.start(
                action_rate=self.ACTION_RATE,
                action_scale=action_scale,
                ros_node=self._ros_node,
            )
            hw.sync()

        # Misc
        self._extract_duration_ema: float = 0.0

        # Spin up ROS executor
        if not ros_node:
            self.__ros_executor = MultiThreadedExecutor(num_threads=2)
            self.__ros_executor.add_node(self._ros_node)
            self.__ros_thread = Thread(target=self.__ros_executor.spin)
            self.__ros_thread.daemon = True
            self.__ros_thread.start()

    def step(
        self, action: Dict[str, numpy.ndarray] | numpy.ndarray
    ) -> Tuple[Dict[str, numpy.ndarray], SupportsFloat, bool, bool, Dict[str, Any]]:
        pre_action_time: float = time.time()

        # Structure the action
        if isinstance(action, numpy.ndarray):
            assert action.shape == self.single_action_space.shape, (
                f"Action shape {action.shape} does not match expected shape {self.single_action_space.shape}"
            )
            action = {
                action_key: action[action_range]
                for action_key, action_range in self.ACTION_ORDERING.items()
            }
        elif not (
            isinstance(action, Mapping)
            and all(isinstance(v, numpy.ndarray) for v in action.values())
        ):
            raise ValueError(
                f"Action must either be a numpy array with shape {self.single_action_space.shape} or a mapping of action keys to numpy arrays, but got: {action}"
            )

        # Apply action
        for hw, keys in self._hardware_action_map.items():
            hw.apply_action(
                {
                    hw_target_key: action[action_key]
                    for action_key, hw_target_key in keys
                }
            )

        # Maintain constant action rate
        action_duration: float = time.time() - pre_action_time
        sleep_duration: float = (
            self.ACTION_RATE - action_duration - self._extract_duration_ema
        )
        if sleep_duration > self._MIN_SLEEP_DURATION:
            time.sleep(sleep_duration)
        else:
            logging.warning(
                f"Action rate of {self.ACTION_RATE} cannot be maintained with a remaining sleep time of {sleep_duration:.3f} s (below the minimum threshold of {self._MIN_SLEEP_DURATION:.3f} s). The actual action rate closer to {(1.0 / (action_duration + self._extract_duration_ema)):.3f} Hz..."
            )

        # Extract observations, rewards, terminations, and info
        pre_extract_time: float = time.time()
        for hw in self._hardware:
            hw.sync()

        # Build hierarchical observation structure
        observation = self._build_observation_structure()

        reward: float = 0.0
        for hw in self._src_reward:
            reward += hw.reward
        terminated: bool = False
        for hw in self._src_termination:
            if hw.termination:
                terminated = True
                break
        info: Dict[str, Any] = {hw.name: hw.info for hw in self._hardware}
        self._extract_duration_ema = (
            self._FREQ_EST_EMA_ALPHA * self._extract_duration_ema
            + (1.0 - self._FREQ_EST_EMA_ALPHA) * (time.time() - pre_extract_time)
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
        observation = self._build_observation_structure()

        # Extract info
        info: Dict[str, Any] = {hw.name: hw.info for hw in self._hardware}

        return observation, info

    def close(self):
        super().close()

        # Close all hardware interfaces
        for hw in self._hardware:
            hw.close()

        # Shutdown ROS node if it was created by this environment
        if hasattr(self, "__ros_executor"):
            self.__ros_executor.shutdown()
            self.__ros_thread.join()
            rclpy.shutdown()

    def __del__(self):
        self.close()

    @property
    def single_action_space(self) -> gymnasium.Space:
        return self._single_action_space

    @property
    def action_space(self) -> gymnasium.Space:
        return self._action_space

    @property
    def single_observation_space(self) -> gymnasium.Space:
        return self._single_observation_space

    @property
    def observation_space(self) -> gymnasium.Space:
        return self._observation_space

    def _map_action_to_hardware(self, action_key: str, action_space: gymnasium.Space):
        _found_action_hw: HardwareInterface | None = None

        # Extract the base action key (e.g., "robot/thrust" -> "thrust")
        base_action_key = action_key.split("/")[-1] if "/" in action_key else action_key

        for hw in self._sink_action:
            # Try both the full action key and the base action key
            for key_to_try in [action_key, base_action_key]:
                alias_key = hw._map_alias(key_to_try)
                for kw_alias_key, hw_target_key in hw.action_key_map.items():
                    if kw_alias_key == alias_key:
                        if _found_action_hw is not None:
                            raise ValueError(
                                f'Action key "{action_key}" must have a unique hardware interface mapping but two were found: {_found_action_hw.name} and {hw.name}'
                            )
                        _found_action_hw = hw

                        if hw not in self._hardware_action_map:
                            self._hardware_action_map[hw] = []
                        self._hardware_action_map[hw].append(
                            (
                                action_key,
                                hw_target_key,
                            )
                        )

                        hw_action_space = hw.supported_action_spaces.spaces[
                            hw_target_key
                        ]
                        if not action_space.shape == hw_action_space.shape:
                            raise ValueError(
                                f'Action "{action_key}" from hardware "{hw.name}" does not match expected space "{action_space}" with its shape "{hw_action_space.shape}"'
                            )
                        if not action_space.dtype == hw_action_space.dtype:
                            raise ValueError(
                                f'Action "{action_key}" from hardware "{hw.name}" does not match expected dtype "{action_space.dtype}" with its dtype "{hw_action_space.dtype}"'
                            )
                        break
                if _found_action_hw is not None:
                    break

        if _found_action_hw is None:
            raise ValueError(
                f'Action key "{action_key}" must be mapped to a hardware interface but no matches were found'
            )

    def _map_observations_recursive(
        self, prefix: str, obs_spaces: Dict[str, gymnasium.Space]
    ):
        for obs_key, obs_space in obs_spaces.items():
            assert "/" not in obs_key
            full_key = f"{prefix}/{obs_key}" if prefix else obs_key
            if isinstance(obs_space, gymnasium.spaces.Dict):
                self._map_observations_recursive(full_key, obs_space.spaces)
            else:
                self._map_observation_to_hardware(full_key, obs_space)

    def _map_observation_to_hardware(self, obs_key: str, obs_space: gymnasium.Space):
        _found_obs_hw: HardwareInterface | None = None
        for hw in self._src_observation:
            alias_key = hw._map_alias(obs_key)
            for kw_alias_key, hw_target_key in hw.observation_key_map.items():
                if kw_alias_key == alias_key:
                    if _found_obs_hw is not None:
                        raise ValueError(
                            f'Observation key "{obs_key}" must have a unique hardware interface mapping but two were found: {_found_obs_hw.name} and {hw.name}'
                        )
                    _found_obs_hw = hw

                    if hw not in self._hardware_observation_map:
                        self._hardware_observation_map[hw] = []
                    self._hardware_observation_map[hw].append((obs_key, hw_target_key))

                    if not obs_space.contains(hw.observation[hw_target_key]):
                        raise ValueError(
                            f'Observation "{obs_key}" from hardware "{hw.name}" does not match expected space "{obs_space}" with its shape "{hw.observation[hw_target_key].shape}" and dtype "{hw.observation[hw_target_key].dtype}"'
                        )

        if _found_obs_hw is None:
            raise ValueError(
                f'Observation key "{obs_key}" must be mapped to a hardware interface but no matches were found'
            )

    def _build_observation_structure(self) -> Dict[str, Any]:
        observation = {}
        for hw, keys in self._hardware_observation_map.items():
            for obs_key, hw_target_key in keys:
                key_parts = obs_key.split("/")
                current_dict = observation
                for part in key_parts[:-1]:
                    if part not in current_dict:
                        current_dict[part] = {}
                    current_dict = current_dict[part]
                current_dict[key_parts[-1]] = hw.observation[hw_target_key]

        return self._flatten_observations(observation)

    @staticmethod
    def _flatten_observations(
        obs_dict: Dict[str, Dict[str, numpy.ndarray]],
    ) -> Dict[str, numpy.ndarray]:
        return {
            obs_cat: numpy.concatenate(
                [
                    obs_group[obs_key].reshape((-1,))
                    for obs_key in sorted(obs_group.keys())
                ],
                axis=0,
            )
            for obs_cat, obs_group in obs_dict.items()
        }
