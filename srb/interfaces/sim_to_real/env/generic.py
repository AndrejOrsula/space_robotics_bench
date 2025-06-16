from typing import Callable

import gymnasium as gym

from srb.core.env import GroundEnvCfg  # Import the config classes

from .interfaces.base_interface import HardwareInterface
from .interfaces.ros_interface import RosHardwareInterface

# This is our factory for creating hardware interfaces
HARDWARE_INTERFACE_MAP = {
    # Could map robot names from the config to their interface class and topics
    "any": {
        "class": RosHardwareInterface,
        "topics": {
            "action": {"topic": "/cmd_vel", "type": "Twist", "shape": [2]},
            "observations": {
                "robot_odom": {"topic": "/odom", "type": "Odometry"},
                "robot_imu": {"topic": "/imu/data", "type": "Imu"},
            },
        },
    }
}


class GenericHIL_Env(gym.Env):
    """
    A generic Hardware-in-the-Loop Gymnasium Environment.
    It is configured using the same TaskCfg as the simulation and uses
    HardwareInterface objects to communicate with the real world.
    """

    def __init__(self, cfg: GroundEnvCfg, compute_fn: Callable):
        super().__init__()
        self.cfg = cfg
        self.compute_fn = compute_fn  # The JIT-scripted reward function
        self.device = "cpu"  # Real-world envs typically run on CPU

        # --- Use the factory to create hardware interfaces ---
        self.interfaces: Dict[str, HardwareInterface] = {}
        # A more sophisticated version would inspect cfg.scene to find assets
        # For now, we assume one main robot.
        robot_interface_config = HARDWARE_INTERFACE_MAP["any"]
        self.robot_interface = robot_interface_config["class"](
            robot_name="robot",
            topic_configs=robot_interface_config["topics"],
            device=self.device,
        )
        self.robot_interface.setup()

        # Define action and observation spaces from config/compute_fn
        # This is complex; for now, we can hardcode based on the task
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        # The observation space is a dictionary matching the state keys
        self.observation_space = gym.spaces.Dict(
            {
                "vel_lin_robot": gym.spaces.Box(-np.inf, np.inf, (3,)),
                "vel_ang_robot": gym.spaces.Box(-np.inf, np.inf, (3,)),
                "tf_pos_robot_to_target": gym.spaces.Box(-np.inf, np.inf, (2,)),
                # ... and so on for all keys in the "state" dict
            }
        )

        # State buffers
        self.episode_length_buf = torch.zeros(1, device=self.device)
        self.action_manager = {
            "action": torch.zeros(1, 2),
            "prev_action": torch.zeros(1, 2),
        }

    def step(self, action: np.ndarray):
        action_tensor = torch.from_numpy(action).unsqueeze(0).to(self.device)
        self.action_manager["prev_action"] = self.action_manager["action"]
        self.action_manager["action"] = action_tensor

        # 1. Publish action to the hardware
        self.robot_interface.publish_action(action_tensor)

        # 2. Wait for one time step
        time.sleep(1.0 / self.cfg.sim.dt)  # Use the same dt as the sim!

        # 3. Gather all observations from all interfaces
        obs_from_hw = self.robot_interface.get_observations()

        # You need to fill in ALL arguments required by compute_fn
        # Some will come from hardware, others are tracked internally (e.g., goal)
        # This is the most complex part of the mapping
        kwargs = {
            "episode_length": self.episode_length_buf,
            "max_episode_length": self.cfg.episode_length_s * self.cfg.sim.dt,
            "truncate_episodes": self.cfg.is_finite_horizon,
            "act_current": self.action_manager["action"],
            "act_previous": self.action_manager["prev_action"],
            **obs_from_hw,
            # Placeholder for values not from the robot (e.g., target position)
            "tf_pos_target": torch.zeros(1, 3, device=self.device),
            "forward_drive_indices": [0],  # Example from nav task
        }

        # 4. Call the original, JIT-scripted reward function!
        step_return = self.compute_fn(**kwargs)

        self.episode_length_buf += 1

        # Extract results and convert to numpy for Gym API
        obs = {
            k: v.squeeze().cpu().numpy()
            for k, v in step_return.observations["state"].items()
        }
        reward = step_return.rewards.sum().item()
        terminated = step_return.terminations.any().item()
        truncated = step_return.truncations.any().item()

        if terminated or truncated:
            self.reset()

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_interface.reset()
        self.episode_length_buf.zero_()
        # Return initial observation
        initial_obs_dict = self.robot_interface.get_observations()
        # This needs to be processed to match the observation space format
        # For simplicity, we return a zeroed observation for the first step
        return {
            k: v.squeeze().cpu().numpy()
            for k, v in self.observation_space.sample().items()
        }, {}

    def close(self):
        self.robot_interface.close()
