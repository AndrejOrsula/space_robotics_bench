import time
from functools import cached_property
from threading import Thread
from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Dict,
    List,
    Mapping,
    Sequence,
    SupportsFloat,
    Tuple,
)

import gymnasium
import numpy
import torch

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceRegistry,
)
from srb.utils import logging

if TYPE_CHECKING:
    from rclpy.node import Node as RosNode


class RealEnv(gymnasium.Env):
    SINGLE_ACTION_SPACE: ClassVar[gymnasium.Space]
    DETAILED_ACTION_SPACE: ClassVar[gymnasium.spaces.Dict]
    ACTION_ORDERING: ClassVar[Dict[str, slice]]
    ACTION_RATE: ClassVar[float]
    ACTION_SCALE: ClassVar[Dict[str, float]]

    SINGLE_OBSERVATION_SPACE: ClassVar[gymnasium.Space]
    DETAILED_OBSERVATION_SPACE: ClassVar[gymnasium.spaces.Dict]

    SIM_ROBOT: ClassVar[str]
    HARDWARE: ClassVar[Sequence[str]]

    # TODO[mid]: Explore better options for rate limiting
    LIMIT_RATE: ClassVar[bool] = True
    MEASURE_PERFORMANCE: ClassVar[bool] = not LIMIT_RATE

    _HOT_START_DURATION: ClassVar[float] = 5.0
    _HOT_START_ITERATIONS: ClassVar[int] = 10
    _MIN_SLEEP_DURATION: ClassVar[float] = 1.0 / 1000.0
    _PAUSE_SLEEP_DURATION: ClassVar[float] = 1.0 / 100.0
    _FREQ_EST_EMA_ALPHA: ClassVar[float] = 0.9
    _PERF_LOG_INTERVAL_S: ClassVar[float] = 10.0
    _PERF_LOG_INTERVAL_STEPS: ClassVar[int] = 100

    def __init__(
        self,
        hardware: Sequence[HardwareInterface | str],
        ros_node: "RosNode | None" = None,
        **kwargs,
    ):
        super().__init__()

        # Map extracted spaces to standard interfaces
        self._single_action_space = self.SINGLE_ACTION_SPACE
        self._action_space = gymnasium.vector.utils.batch_space(
            self.single_action_space, 1
        )
        self._single_observation_space = self.SINGLE_OBSERVATION_SPACE
        self._observation_space = gymnasium.vector.utils.batch_space(
            self.single_observation_space, 1
        )

        # Create hardware interfaces
        self._hardware: Sequence[HardwareInterface] = tuple(
            set(
                (
                    HardwareInterfaceRegistry.get_by_name(hw)()  # type: ignore
                    if isinstance(hw, str)
                    else hw
                )
                for hw in (hardware or self.HARDWARE)
            )
        )

        # Categorize all hardware interfaces action sinks
        self._sink_action: Sequence[HardwareInterface] = []
        for i, hw in enumerate(self._hardware, 1):
            logging.info(f"Hardware interface #{i}: {hw.name}")
            if hw._has_io_action:
                self._sink_action.append(hw)
        self._sink_action = tuple(self._sink_action)
        self._hardware_action_map: Dict[
            HardwareInterface, Sequence[Tuple[str, str]]
        ] = {}

        # Map each action to a single hardware interface
        if isinstance(self.DETAILED_ACTION_SPACE, gymnasium.spaces.Dict):
            for action_key, action_space in self.DETAILED_ACTION_SPACE.spaces.items():
                self._map_action_to_hardware(action_key, action_space)
        else:
            raise ValueError(
                f"Unexpected action space type: {self.DETAILED_ACTION_SPACE}"
            )
        self._hardware_action_map = {
            hw: tuple(keys) for hw, keys in self._hardware_action_map.items()
        }

        # Initialize ROS node
        if not ros_node:
            import rclpy
            from rclpy.node import Node as RosNode

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
                    action_scale.update(self.ACTION_SCALE)
            else:
                action_scale = {}
            hw.start(
                action_rate=self.ACTION_RATE,
                action_scale=action_scale,
                ros_node=self._ros_node,
            )

        # Spin up ROS executor
        if not ros_node:
            from rclpy.executors import MultiThreadedExecutor

            self.__ros_executor = MultiThreadedExecutor(num_threads=2)
            self.__ros_executor.add_node(self._ros_node)
            self.__ros_thread = Thread(target=self.__ros_executor.spin)
            self.__ros_thread.daemon = True
            self.__ros_thread.start()

        # Hot-start all hardware interfaces to ensure they are ready
        hot_start_sleep_period = self._HOT_START_DURATION / self._HOT_START_ITERATIONS
        for _ in range(self._HOT_START_ITERATIONS):
            time.sleep(hot_start_sleep_period)
            for hw in self._hardware:
                hw.sync()

        # Categorize all hardware interface sources (observations, rewards, terminations)
        self._src_observation: Sequence[HardwareInterface] = []
        self._src_reward: Sequence[HardwareInterface] = []
        self._src_termination: Sequence[HardwareInterface] = []
        self._src_pause: Sequence[HardwareInterface] = []
        self._src_resume: Sequence[HardwareInterface] = []
        for hw in self._hardware:
            if hw._has_io_observation:
                self._src_observation.append(hw)
            if hw._has_io_reward:
                self._src_reward.append(hw)
            if hw._has_io_termination:
                self._src_termination.append(hw)
            if hw._has_io_pause:
                self._src_pause.append(hw)
            if hw._has_io_resume:
                self._src_resume.append(hw)
        self._src_observation = tuple(self._src_observation)
        self._src_reward = tuple(self._src_reward)
        self._src_termination = tuple(self._src_termination)
        self._src_pause = tuple(self._src_pause)
        self._src_resume = tuple(self._src_resume)
        logging.info(
            f"Action interfaces: {', '.join(hw.name for hw in self._sink_action)}\n"
            f"Observation interfaces: {', '.join(hw.name for hw in self._src_observation)}\n"
            f"Reward interfaces: {', '.join(hw.name for hw in self._src_reward)}\n"
            f"Termination interfaces: {', '.join(hw.name for hw in self._src_termination)}\n"
            f"Pause interfaces: {', '.join(hw.name for hw in self._src_pause)}\n"
            f"Resume interfaces: {', '.join(hw.name for hw in self._src_resume)}"
        )

        # Map each observation to a single hardware interface
        self._hardware_observation_map: Dict[
            HardwareInterface, Sequence[Tuple[str, str]]
        ] = {}
        if isinstance(self.DETAILED_OBSERVATION_SPACE, gymnasium.spaces.Dict):
            self._map_observations_recursive("", self.DETAILED_OBSERVATION_SPACE.spaces)
        else:
            raise ValueError(
                f"Unexpected observation space type: {self.DETAILED_OBSERVATION_SPACE}"
            )
        self._hardware_observation_map = {
            hw: tuple(keys) for hw, keys in self._hardware_observation_map.items()
        }

        # Misc
        self._is_running: bool = True
        self._extract_duration_ema: float = 0.0
        self._last_step_time: float = 0.0
        self._step_freq_ema: float = 0.0
        self._perf_log_last_time: float = 0.0
        self._perf_log_last_step: int = 0
        self._step_count: int = 0

    def step(
        self,
        action: numpy.ndarray | torch.Tensor | Dict[str, numpy.ndarray | torch.Tensor],
    ) -> Tuple[Dict[str, numpy.ndarray], SupportsFloat, bool, bool, Dict[str, Any]]:
        # Measure performance
        if self.MEASURE_PERFORMANCE:
            current_time = time.time()
            if self._last_step_time > 0:
                step_duration = current_time - self._last_step_time
                if step_duration > 0:
                    step_freq = 1.0 / step_duration
                    self._step_freq_ema = (
                        self._FREQ_EST_EMA_ALPHA * self._step_freq_ema
                        + (1.0 - self._FREQ_EST_EMA_ALPHA) * step_freq
                    )
            self._step_count += 1

            # Log performance
            if (
                current_time - self._perf_log_last_time > self._PERF_LOG_INTERVAL_S
                and self._step_count - self._perf_log_last_step
                > self._PERF_LOG_INTERVAL_STEPS
            ):
                if self._step_freq_ema > 0:
                    latency_ms = (1.0 / self._step_freq_ema) * 1000
                    logging.info(
                        f"Controller frequency: {self._step_freq_ema:.2f} Hz "
                        f"({latency_ms:.2f} ms)"
                    )
                else:
                    logging.info(f"Controller frequency: {self._step_freq_ema:.2f} Hz")
                self._perf_log_last_time = current_time
                self._perf_log_last_step = self._step_count

        # Handle pause and resume signals
        if self._is_running:
            for hw in self._src_pause:
                if hw.pause_signal:
                    logging.info(f"Pause signal received from {hw.name}")
                    for hw in self._hardware:
                        hw.pause()
                    self._is_running = False
                    for hw in self._src_resume:
                        _discard = hw.resume_signal
                    break
        if not self._is_running:
            while True:
                time.sleep(self._PAUSE_SLEEP_DURATION)
                for hw in self._src_resume:
                    hw.sync()
                    if hw.resume_signal:
                        logging.info(f"Resume signal received from {hw.name}")
                        for hw in self._hardware:
                            hw.resume()
                        self._is_running = True
                        for hw in self._src_pause:
                            _discard = hw.pause_signal
                        break
                else:
                    continue
                break

        # Structure the action
        if self.LIMIT_RATE:
            pre_action_time: float = time.time()
        if isinstance(action, Mapping):
            if any(isinstance(v, torch.Tensor) for v in action.values()):
                action = {
                    action_key: v.detach().cpu().numpy()
                    if isinstance(v, torch.Tensor)
                    else v
                    for action_key, v in action.items()
                }
            assert all(isinstance(v, numpy.ndarray) for v in action.values()), (
                f"Action must be a mapping of action keys to numpy arrays, but got: {action}"
            )
            assert set(action.keys()) == set(self.ACTION_ORDERING.keys()), (
                f"Action keys {set(action.keys())} do not match expected keys {set(self.ACTION_ORDERING.keys())}"
            )
        else:
            if isinstance(action, torch.Tensor):
                action = action.detach().cpu().numpy()
            assert isinstance(action, numpy.ndarray), (
                f"Action must be a numpy array or a mapping, but got: {type(action)}"
            )
            if action.ndim != 1:
                action = action.reshape(-1)
            assert action.shape == self.single_action_space.shape, (
                f"Action shape {action.shape} does not match expected shape {self.single_action_space.shape}"
            )
            action = {
                action_key: action[action_range]
                for action_key, action_range in self.ACTION_ORDERING.items()
            }

        # Apply action
        for hw, keys in self._hardware_action_map.items():
            hw.apply_action(
                {  # type: ignore
                    hw_target_key: action[action_key]
                    for action_key, hw_target_key in keys
                }
            )

        # Maintain constant action rate
        if self.LIMIT_RATE:
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
        if self.LIMIT_RATE:
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
        if self.MEASURE_PERFORMANCE:
            info["controller_freq"] = self._step_freq_ema
            if self._step_freq_ema > 0:
                info["controller_latency_ms"] = (1.0 / self._step_freq_ema) * 1000
        if self.LIMIT_RATE:
            self._extract_duration_ema = (
                self._FREQ_EST_EMA_ALPHA * self._extract_duration_ema
                + (1.0 - self._FREQ_EST_EMA_ALPHA) * (time.time() - pre_extract_time)
            )

        # Reset episode if terminated
        if terminated:
            logging.info("Resetting environment due to termination")
            self.reset()

        if self.MEASURE_PERFORMANCE:
            self._last_step_time = time.time()

        return observation, reward, terminated, False, info

    def reset(self, **kwargs) -> Tuple[Dict[str, numpy.ndarray], Dict[str, Any]]:
        super().reset(**kwargs)

        # Reset all hardware interfaces
        for hw in self._hardware:
            hw.reset()

        # Reset performance metrics
        self._last_step_time = 0.0
        self._step_freq_ema = 0.0
        self._perf_log_last_time = 0.0
        self._perf_log_last_step = 0
        self._step_count = 0

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
            import rclpy

            self.__ros_executor.shutdown()
            self.__ros_thread.join()
            rclpy.shutdown()

    def seed(self, seed: int | None = None) -> List[int | None]:
        return [seed]

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
                        self._hardware_action_map[hw].append(  # type: ignore
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
                    self._hardware_observation_map[hw].append(  # type: ignore
                        (obs_key, hw_target_key)
                    )

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

        return RealEnv._flatten_observations(observation)

    @staticmethod
    def _flatten_observations(
        obs_dict: Dict[str, Dict[str, numpy.ndarray]],
    ) -> Dict[str, numpy.ndarray]:
        return {
            obs_cat: obs_group
            if isinstance(obs_group, numpy.ndarray)
            else numpy.concatenate(
                [
                    obs_group[obs_key].reshape((-1,))
                    for obs_key in sorted(obs_group.keys())
                ],
                axis=0,
            )
            for obs_cat, obs_group in obs_dict.items()
        }

    ## Properties for compatibility with simulation environments ##

    @cached_property
    def device(self) -> torch.device:
        return torch.device("cpu")

    @property
    def num_envs(self) -> int:
        return 1
