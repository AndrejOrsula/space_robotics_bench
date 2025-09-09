from collections import defaultdict, deque
from dataclasses import MISSING
from datetime import datetime
from typing import Sequence

import numpy as np
import torch
from rclpy.time import Duration, Time
from tf2_ros import Buffer, TransformListener

from srb.utils.math import rpy_to_quat

try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: opencv-python not available. Video recording will be disabled.")

from typing import Dict

from torchvision.utils import make_grid, save_image

from srb import assets
from srb._typing import StepReturn
from srb.core.action import ThrustAction  # noqa: F401
from srb.core.asset import (
    AssetVariant,
    ExtravehicularScenery,
    Frame,
    MobileRobot,
    OrbitalRobot,
    Transform,
)
from srb.core.env import (
    OrbitalEnv,
    OrbitalEnvCfg,
    OrbitalEnvVisualExtCfg,
    OrbitalEventCfg,
    OrbitalSceneCfg,
    ViewerCfg,
)
from srb.core.env.common.extension.visual.impl import construct_observation
from srb.core.manager import EventTermCfg, SceneEntityCfg  # noqa: F401
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import apply_external_force_torque, offset_pose_natural  # noqa: F401
from srb.core.sim import ArrowCfg, PreviewSurfaceCfg
from srb.utils import logging
from srb.utils.cfg import configclass
from srb.utils.math import matrix_from_quat, rotmat_to_rot6d, subtract_frame_transforms
from srb.utils.path import SRB_LOGS_DIR
from srb.utils.str import sanitize_cam_name

##############
### Config ###
##############


@configclass
class SceneCfg(OrbitalSceneCfg):
    pass


@configclass
class EventCfg(OrbitalEventCfg):
    target_pose_evolution: EventTermCfg = EventTermCfg(
        func=offset_pose_natural,
        mode="interval",
        interval_range_s=(0.1, 0.1),
        is_global_time=True,
        params={
            "env_attr_name": "_goal",
            "pos_axes": ("x", "y", "z"),
            "pos_step_range": (0.01, 0.1),
            "pos_smoothness": 0.96,
            "pos_step_smoothness": 0.8,
            "pos_bounds": {
                "x": MISSING,
                "y": MISSING,
                "z": MISSING,
            },
            "orient_yaw_only": False,
            "orient_smoothness": 0.8,
        },
    )
    # push_robot: EventTermCfg = EventTermCfg(  # temporarily remove disturbances
    #     func=apply_external_force_torque,
    #     mode="interval",
    #     interval_range_s=(5.0, 20.0),
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "force_range": (-4.0, 4.0),
    #         "torque_range": (-0.19635, 0.19635),
    #     },
    # )


@configclass
class TaskCfg(OrbitalEnvCfg):
    ## Scene
    scene: SceneCfg = SceneCfg()
    stack: bool = True

    ## Assets
    scenery: ExtravehicularScenery | MobileRobot | AssetVariant | None = (
        assets.Gateway()
    )
    # scenery.asset_cfg.init_state.pos = (0.0, 7.0, -10.0)
    # scenery.asset_cfg.init_state.rot = rpy_to_quat(0.0, 0.0, 90.0)
    # TODO: Re-enable collisions with the scenery
    scenery.asset_cfg.spawn.collision_props.collision_enabled = False  # type: ignore
    scenery.asset_cfg.spawn.mesh_collision_props.mesh_approximation = None  # type: ignore
    _scenery: ExtravehicularScenery | None = MISSING  # type: ignore

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    env_rate: float = 1.0 / 40.0
    agent_rate: float = 1.0 / 10.0
    episode_length_s: float = 60.0
    is_finite_horizon: bool = True

    ## Target
    target_pos_range_ratio: float = 0.9
    target_marker_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/target",
        markers={
            "target": ArrowCfg(
                tail_radius=0.02,
                tail_length=0.3,
                head_radius=0.1,
                head_length=0.15,
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    ## Viewer
    viewer: ViewerCfg = ViewerCfg(
        eye=(3.0, -3.0, 3.0),
        lookat=(0.0, 0.0, 0.0),
        origin_type="asset_root",
        asset_name="robot",
    )

    def __post_init__(self):
        super().__post_init__()

        # Event: Waypoint target
        if (
            "hardcoded"
            not in self.events.target_pose_evolution.params["pos_bounds"].keys()  # type: ignore
        ):
            assert self.spacing is not None
            for dim in ("x", "y", "z"):
                self.events.target_pose_evolution.params["pos_bounds"][dim] = (  # type: ignore
                    -50.0,
                    50.0,
                )

        if isinstance(self.robot, OrbitalRobot):
            self.robot.frame_onboard_camera = Frame(
                prim_relpath="cubesat/camera_onboard",
                offset=Transform(
                    pos=(0.0, 0.125, 0.0),
                    rot=rpy_to_quat(0.0, 0.0, 90.0),
                ),
            )
            self._robot = self.robot

        self.scene.scenery.init_state.pos = (4.0, 9.0, -10.0)  # type: ignore
        self.scene.scenery.init_state.rot = rpy_to_quat(0.0, 0.0, 90.0)  # type: ignore


@configclass
class VisualTaskCfg(OrbitalEnvVisualExtCfg, TaskCfg):
    def __post_init__(self):
        TaskCfg.__post_init__(self)
        OrbitalEnvVisualExtCfg.wrap(self, env_cfg=self)


############
### Task ###
############


class Task(OrbitalEnv):
    cfg: VisualTaskCfg

    def __init__(self, cfg: VisualTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        ## Initialize buffers
        self._goal = torch.zeros(self.num_envs, 7, device=self.device)
        self._goal[:, 0:3] = self.scene.env_origins
        self._goal[:, 3] = 1.0

        # Get camera
        self.__cameras = [
            (
                self.scene.sensors[camera_key],
                f"image_{sanitize_cam_name(camera_key)}",
                cfg.cameras_cfg[camera_key].data_types,
                cfg.cameras_cfg[camera_key].spawn.clipping_range,  # type: ignore
            )
            for camera_key in cfg.cameras_cfg.keys()
        ]

        # Video recording setup
        self._video_enabled = True  # TODO: fix make configurable
        self._episode_frames = defaultdict(
            lambda: deque()
        )  # Store frames for each camera
        self._episode_counter = 0
        self._video_fps = 10  # Target FPS for videos (matches agent_rate of 10Hz)
        self._debug_img = False  # Whether to save individual images for debugging

        if not CV2_AVAILABLE and (
            kwargs.get("enable_cameras", False) or kwargs.get("video", False)
        ):
            print(
                "Warning: Video recording requested but opencv-python not available. Install with: pip install opencv-python"
            )
        else:
            print(f"Video recording enabled: {self._video_enabled}")
            self.videos_dir = SRB_LOGS_DIR.joinpath("oc_videos").joinpath(
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            )
            self.videos_dir.mkdir(parents=True, exist_ok=True)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

        ## Reset goal position
        self._goal[env_ids, 0:3] = self.scene.env_origins[env_ids]
        self._goal[env_ids, 3:7] = torch.tensor(
            [1.0, 0.0, 0.0, 0.0], device=self.device
        )

        # Save video for completed episodes and reset frame buffers
        if self._video_enabled and len(env_ids) > 0:
            self._save_episode_videos()
            self._episode_frames.clear()
            self._episode_counter += 1

        if hasattr(self.unwrapped, "ros_node"):
            # ADD ROS SUBSCRIBER TO GET THE WAYPOINTS FROM TRAJ GENERATOR AND UPDATE GOALS
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer, self.unwrapped.ros_node
            )
            logging.info("TF listener initialized.")

    def extract_camera_observations(self) -> Dict[str, torch.Tensor]:
        # Early return if video recording is disabled
        if not self._video_enabled:
            return {}

        # Only construct observations when video recording is enabled
        images = {
            image_key: image
            for camera, image_basename, data_types, clipping_range in self.__cameras
            for image_key, image in construct_observation(
                image_basename=image_basename,
                data_types=data_types,
                clipping_range=clipping_range,  # type: ignore
                merge_channels=False,
                **camera.data.output,
            ).items()
        }

        # Store frames for video creation and optionally save debug images
        if self._video_enabled:
            for image_key, image in images.items():
                # Convert tensor to numpy array suitable for video
                if image.dtype == torch.uint8:
                    # Shape: [1, H, W, C] -> [H, W, C] or [1, H, W] -> [H, W]
                    frame = image[0].cpu().numpy()
                else:
                    # Convert float [0,1] to uint8 [0,255]
                    frame = (image[0].cpu().numpy() * 255).astype(np.uint8)

                # Convert to BGR for OpenCV VideoWriter
                if CV2_AVAILABLE:
                    if frame.ndim == 2 or (frame.ndim == 3 and frame.shape[-1] == 1):
                        # Grayscale to BGR
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    elif frame.shape[-1] == 4:
                        # RGBA to BGR (discards alpha by blending with black background)
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                    elif frame.shape[-1] == 3:
                        # RGB to BGR
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                self._episode_frames[image_key].append(frame)

            # Optional: Still save individual images for debugging (less frequent)
            save_every = 100
            if self._debug_img and self.episode_length_buf[0] % save_every == 0:
                for image_key, image in images.items():
                    file_path = SRB_LOGS_DIR.joinpath(
                        f"inspection_{image_key}_{self.episode_length_buf[0]}.png"
                    ).as_posix()
                    print(f"Saving debug image to {file_path}")

                    # Convert uint8 tensor to float and normalize to [0, 1] range for save_image
                    if image.dtype == torch.uint8:
                        image_normalized = image.float() / 255.0
                    else:
                        image_normalized = image

                    save_image(
                        make_grid(
                            torch.swapaxes(
                                image_normalized.unsqueeze(1), 1, -1
                            ).squeeze(-1),
                            nrow=round(image_normalized.shape[0] ** 0.5),
                        ),
                        file_path,
                    )

        return {}

    def _save_episode_videos(self):
        """Save videos from collected frames for each camera."""
        if not self._episode_frames or not CV2_AVAILABLE:
            return

        for image_key, frames in self._episode_frames.items():
            if len(frames) == 0:
                continue

            video_path = (
                self.videos_dir
                / f"inspection_{image_key}_episode_{self._episode_counter}.mp4"
            )
            print(f"Saving video to {video_path} ({len(frames)} frames)")

            # Get frame dimensions
            height, width = frames[0].shape[:2]

            # Initialize video writer
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(
                str(video_path), fourcc, self._video_fps, (width, height)
            )

            if not video_writer.isOpened():
                print(f"Warning: Could not open video writer for {video_path}")
                continue

            # Write all frames
            for frame in frames:
                video_writer.write(frame)

            video_writer.release()
            print(f"Video saved: {video_path}")

    def extract_step_return(self) -> StepReturn:
        if hasattr(self, "tf_buffer"):
            try:
                tf_stamped = self.tf_buffer.lookup_transform(
                    "srb/env0",
                    "target",
                    Time(),
                    timeout=Duration(
                        seconds=1,
                        nanoseconds=0,
                    ),
                )
                logging.debug(
                    f"Got transform from 'srb/env0' to 'target': {tf_stamped}"
                )
                # self._goal = ... (target transform)
                self._goal[:, 0] = tf_stamped.transform.translation.x
                self._goal[:, 1] = tf_stamped.transform.translation.y
                self._goal[:, 2] = tf_stamped.transform.translation.z
                self._goal[:, 3] = tf_stamped.transform.rotation.w
                self._goal[:, 4] = tf_stamped.transform.rotation.x
                self._goal[:, 5] = tf_stamped.transform.rotation.y
                self._goal[:, 6] = tf_stamped.transform.rotation.z
            except Exception as e:
                logging.warning(
                    f"Failed to get transform from 'srb/env0' to 'target': {e}"
                )

        # get the camera images
        self.extract_camera_observations()
        ## Visualize target
        self._target_marker.visualize(self._goal[:, 0:3], self._goal[:, 3:7])

        # ## Get remaining fuel (if applicable)
        # if self._thrust_action_term_key:
        #     thrust_action_term: ThrustAction = self.action_manager._terms[  # type: ignore
        #         self._thrust_action_term_key
        #     ]
        #     remaining_fuel = (
        #         thrust_action_term.remaining_fuel / thrust_action_term.cfg.fuel_capacity
        #     ).unsqueeze(-1)
        # else:
        #     remaining_fuel = None

        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Root
            tf_pos_robot=self._robot.data.root_pos_w,
            tf_quat_robot=self._robot.data.root_quat_w,
            vel_lin_robot=self._robot.data.root_lin_vel_b,
            vel_ang_robot=self._robot.data.root_ang_vel_b,
            # Transforms (world frame)
            tf_pos_target=self._goal[:, 0:3],
            tf_quat_target=self._goal[:, 3:7],
            # # IMU
            # imu_lin_acc=self._imu_robot.data.lin_acc_b,
            # imu_ang_vel=self._imu_robot.data.ang_vel_b,
            # Fuel
            # remaining_fuel=remaining_fuel,
        )


@torch.jit.script
def _compute_step_return(
    *,
    ## Time
    episode_length: torch.Tensor,
    max_episode_length: int,
    truncate_episodes: bool,
    ## Actions
    act_current: torch.Tensor,
    act_previous: torch.Tensor,
    ## States
    # Root
    tf_pos_robot: torch.Tensor,
    tf_quat_robot: torch.Tensor,
    vel_lin_robot: torch.Tensor,
    vel_ang_robot: torch.Tensor,
    # Transforms (world frame)
    tf_pos_target: torch.Tensor,
    tf_quat_target: torch.Tensor,
    # # IMU
    # imu_lin_acc: torch.Tensor,
    # imu_ang_vel: torch.Tensor,
    # Fuel
    # remaining_fuel: torch.Tensor | None,
) -> StepReturn:
    num_envs = episode_length.size(0)
    # dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Root
    tf_rotmat_robot = matrix_from_quat(tf_quat_robot)
    tf_rot6d_robot = rotmat_to_rot6d(tf_rotmat_robot)

    ## Transforms (world frame)
    # Robot -> Target
    tf_pos_robot_to_target, tf_quat_robot_to_target = subtract_frame_transforms(
        t01=tf_pos_robot,
        q01=tf_quat_robot,
        t02=tf_pos_target,
        q02=tf_quat_target,
    )
    tf_rotmat_robot_to_target = matrix_from_quat(tf_quat_robot_to_target)
    tf_rot6d_robot_to_target = rotmat_to_rot6d(tf_rotmat_robot_to_target)
    dist_robot_to_target = torch.norm(tf_pos_robot_to_target, dim=-1)

    # # Angle between the robot's forward direction and the target's position
    # angle_robot_to_target_pos = torch.acos(
    #     torch.clamp(
    #         tf_pos_robot_to_target[..., 0] / (dist_robot_to_target + 1.0e-6), -1.0, 1.0
    #     )
    # )
    # # Angle of the relative orientation between the robot and the target
    # trace = torch.einsum("...ii->...", tf_rotmat_robot_to_target)
    # angle_robot_to_target_orient = torch.acos(
    #     torch.clamp((trace - 1.0) / 2.0, -1.0, 1.0)
    # )

    # ## Fuel
    # remaining_fuel = (
    #     remaining_fuel
    #     if remaining_fuel is not None
    #     else torch.ones((num_envs, 1), dtype=dtype, device=device)
    # )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -16.0
    _action_rate = torch.mean(torch.square(act_current - act_previous), dim=1)
    penalty_action_rate = WEIGHT_ACTION_RATE * _action_rate

    # Penalty: Action magnitude
    WEIGHT_ACTION_MAGNITUDE = -16.0
    _action_magnitude = torch.mean(torch.square(act_current), dim=1)
    penalty_action_magnitude = WEIGHT_ACTION_MAGNITUDE * _action_magnitude

    # # Penalty: Fuel consumption
    # WEIGHT_FUEL_CONSUMPTION = -8.0
    # penalty_fuel_consumption = WEIGHT_FUEL_CONSUMPTION * torch.square(
    #     1.0 - remaining_fuel.squeeze(-1)
    # )

    # Penalty: Position tracking | Robot <--> Target
    WEIGHT_POSITION_TRACKING = -2.0
    MAX_POSITION_TRACKING_PENALTY = -16.0
    penalty_position_tracking = torch.clamp_min(
        WEIGHT_POSITION_TRACKING * torch.square(dist_robot_to_target),
        min=MAX_POSITION_TRACKING_PENALTY,
    )

    # # Reward: Point towards target | Robot <--> Target
    # WEIGHT_POINT_TOWARDS_TARGET = 1.0
    # TANH_STD_POINT_TOWARDS_TARGET = 0.7854  # 45 deg
    # reward_point_towards_target = WEIGHT_POINT_TOWARDS_TARGET * (
    #     1.0
    #     - torch.tanh(
    #         torch.abs(angle_robot_to_target_pos) / TANH_STD_POINT_TOWARDS_TARGET
    #     )
    # )

    # Reward: Position tracking | Robot <--> Target (precision)
    WEIGHT_POSITION_TRACKING_PRECISION = 16.0
    TANH_STD_POSITION_TRACKING_PRECISION = 0.1
    _position_tracking_precision = 1.0 - torch.tanh(
        dist_robot_to_target / TANH_STD_POSITION_TRACKING_PRECISION
    )
    reward_position_tracking_precision = (
        WEIGHT_POSITION_TRACKING_PRECISION * _position_tracking_precision
    )

    # Reward: Target orientation tracking once position is reached | Robot <--> Target
    WEIGHT_ORIENTATION_TRACKING = 64.0
    TANH_STD_ORIENTATION_TRACKING = 0.25
    orientation_error = torch.linalg.matrix_norm(
        tf_rotmat_robot_to_target
        - torch.eye(3, device=device).unsqueeze(0).expand_as(tf_rotmat_robot_to_target),
        ord="fro",
    )
    _orientation_tracking_precision = _position_tracking_precision * (
        1.0 - torch.tanh(orientation_error / TANH_STD_ORIENTATION_TRACKING)
    )
    reward_orientation_tracking = (
        WEIGHT_ORIENTATION_TRACKING * _orientation_tracking_precision
    )

    # Reward: Action rate at target
    WEIGHT_ACTION_RATE_AT_TARGET = 128.0
    TANH_STD_ACTION_RATE_AT_TARGET = 0.2
    reward_action_rate_at_target = (
        WEIGHT_ACTION_RATE_AT_TARGET
        * _orientation_tracking_precision
        * (1.0 - torch.tanh(_action_rate / TANH_STD_ACTION_RATE_AT_TARGET))
    )

    ##################
    ## Terminations ##
    ##################
    # No termination condition
    termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
    # Truncation
    truncation = (
        episode_length >= max_episode_length
        if truncate_episodes
        else torch.zeros(num_envs, dtype=torch.bool, device=device)
    )

    return StepReturn(
        {
            "state": {
                "act_previous": act_previous,
                "tf_rot6d_robot": tf_rot6d_robot,
                "vel_lin_robot": vel_lin_robot,
                "vel_ang_robot": vel_ang_robot,
                "tf_pos_robot_to_target": tf_pos_robot_to_target,
                "tf_rot6d_robot_to_target": tf_rot6d_robot_to_target,
            },
            # "proprio": {
            #     # "imu_lin_acc": imu_lin_acc,
            #     # "imu_ang_vel": imu_ang_vel,
            #     "remaining_fuel": remaining_fuel,
            # },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_action_magnitude": penalty_action_magnitude,
            # "penalty_fuel_consumption": penalty_fuel_consumption,
            "penalty_position_tracking": penalty_position_tracking,
            # "reward_point_towards_target": reward_point_towards_target,
            "reward_position_tracking_precision": reward_position_tracking_precision,
            "reward_orientation_tracking": reward_orientation_tracking,
            "reward_action_rate_at_target": reward_action_rate_at_target,
        },
        termination,
        truncation,
    )
