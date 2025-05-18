from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    ArticulationCfg,
    AssetBaseCfg,
    AssetVariant,
    Manipulator,
    Object,
    RigidObject,
    RigidObjectCfg,
)
from srb.core.domain import Domain
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import (
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    rpy_to_quat,
    scale_transform,
    subtract_frame_transforms,
)

##############
### Config ###
##############


@configclass
class SceneCfg(ManipulationSceneCfg):
    bolt: RigidObjectCfg = MISSING  # type: ignore
    nut: RigidObjectCfg = MISSING  # type: ignore

    decor: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/decor",
        spawn=UsdFileCfg(
            usd_path=assets.Ingenuity().asset_cfg.spawn.usd_path,  # type: ignore
            collision_props=CollisionPropertiesCfg(collision_enabled=True),
            mesh_collision_props=MeshCollisionPropertiesCfg(mesh_approximation="none"),
            articulation_props=ArticulationRootPropertiesCfg(
                articulation_enabled=False
            ),
            rigid_props=RigidBodyPropertiesCfg(rigid_body_enabled=False),
        ),  # type: ignore
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.65, 0.0, 0.0),
            rot=rpy_to_quat((0.0, 0.0, 45.0)),
        ),
    )


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("bolt"),
            "pose_range": {},
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    ## Scenario
    domain: Domain = Domain.MARS

    ## Assets
    robot: Manipulator | AssetVariant = assets.Franka(
        end_effector=assets.ElectricScrewdriverM5()
    )
    bolt: Object | AssetVariant | None = assets.BoltM8()

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    env_rate: float = 1.0 / 250.0
    episode_length_s: float = 15.0
    is_finite_horizon: bool = True

    def __post_init__(self):
        super().__post_init__()

        # Task setup: Nut from Bolt
        if isinstance(self.bolt, assets.BoltM8):
            nut = assets.NutM8()
            thread_offset = -0.000375
            thread_pitch = 0.00125
        else:
            raise ValueError("Unsupported bolt type")
        # Scene: Nut
        self.scene.nut = nut.asset_cfg
        self.scene.nut.prim_path = "{ENV_REGEX_NS}/nut"
        self.scene.nut.init_state.pos = (
            0.65,
            0.0,
            0.493,
        )
        # Scene: Bolt
        self.scene.bolt = self.bolt.asset_cfg
        self.scene.bolt.prim_path = "{ENV_REGEX_NS}/bolt"
        self.scene.bolt.init_state.pos = (
            0.65,
            0.0,
            0.493 + thread_offset + 3.0 * thread_pitch,
        )
        # Scene: Decor
        self.scene.decor.prim_path = (
            "/World/decor" if self.stack else "{ENV_REGEX_NS}/decor"
        )

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.bolt.prim_path
            ]


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = self.scene["bolt"]
        self._target: RigidObject = self.scene["nut"]

        ## Initialize buffers
        self._offset_pos_bolt_driver_slot = torch.tensor(
            self.cfg.bolt.frame_driver_slot.offset.pos,  # type: ignore
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)

    def extract_step_return(self) -> StepReturn:
        return _compute_step_return(
            ## Time
            episode_length=self.episode_length_buf,
            max_episode_length=self.max_episode_length,
            truncate_episodes=self.cfg.truncate_episodes,
            ## Actions
            act_current=self.action_manager.action,
            act_previous=self.action_manager.prev_action,
            ## States
            # Joints
            joint_pos_robot=self._robot.data.joint_pos,
            joint_pos_limits_robot=(
                self._robot.data.soft_joint_pos_limits
                if torch.all(torch.isfinite(self._robot.data.soft_joint_pos_limits))
                else None
            ),
            joint_pos_end_effector=self._end_effector.data.joint_pos
            if isinstance(self._end_effector, Articulation)
            else None,
            joint_pos_limits_end_effector=(
                self._end_effector.data.soft_joint_pos_limits
                if isinstance(self._end_effector, Articulation)
                and torch.all(
                    torch.isfinite(self._end_effector.data.soft_joint_pos_limits)
                )
                else None
            ),
            joint_acc_robot=self._robot.data.joint_acc,
            joint_applied_torque_robot=self._robot.data.applied_torque,
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            tf_pos_obj=self._obj.data.root_pos_w,
            tf_quat_obj=self._obj.data.root_quat_w,
            tf_pos_target=self._target.data.root_pos_w,
            tf_quat_target=self._target.data.root_quat_w,
            # Object velocity
            vel_ang_obj=self._obj.data.root_ang_vel_w,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            ## Bolt descriptors
            offset_pos_bolt_driver_slot=self._offset_pos_bolt_driver_slot,
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
    # Joints
    joint_pos_robot: torch.Tensor,
    joint_pos_limits_robot: torch.Tensor | None,
    joint_pos_end_effector: torch.Tensor | None,
    joint_pos_limits_end_effector: torch.Tensor | None,
    joint_acc_robot: torch.Tensor,
    joint_applied_torque_robot: torch.Tensor,
    # Kinematics
    fk_pos_end_effector: torch.Tensor,
    fk_quat_end_effector: torch.Tensor,
    # Transforms (world frame)
    tf_pos_end_effector: torch.Tensor,
    tf_quat_end_effector: torch.Tensor,
    tf_pos_obj: torch.Tensor,
    tf_quat_obj: torch.Tensor,
    tf_pos_target: torch.Tensor,
    tf_quat_target: torch.Tensor,
    # Object velocity
    vel_ang_obj: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
    ## Bolt descriptors
    offset_pos_bolt_driver_slot: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    dtype = episode_length.dtype
    device = episode_length.device

    ############
    ## States ##
    ############
    ## Joints
    # Robot joints
    joint_pos_robot_normalized = (
        scale_transform(
            joint_pos_robot,
            joint_pos_limits_robot[:, :, 0],
            joint_pos_limits_robot[:, :, 1],
        )
        if joint_pos_limits_robot is not None
        else joint_pos_robot
    )
    # End-effector joints
    joint_pos_end_effector_normalized = (
        scale_transform(
            joint_pos_end_effector,
            joint_pos_limits_end_effector[:, :, 0],
            joint_pos_limits_end_effector[:, :, 1],
        )
        if joint_pos_end_effector is not None
        and joint_pos_limits_end_effector is not None
        else (
            joint_pos_end_effector
            if joint_pos_end_effector is not None
            else torch.empty((num_envs, 0), dtype=dtype, device=device)
        )
    )

    ## Kinematics
    fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
    fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

    ## Transforms (world frame)
    # End-effector -> Bolt driver slot
    tf_pos_bolt_driver_slot, tf_quat_bolt_driver_slot = combine_frame_transforms(
        t01=tf_pos_obj,
        q01=tf_quat_obj,
        t12=offset_pos_bolt_driver_slot,
    )
    (
        tf_pos_end_effector_to_bolt_driver_slot,
        tf_quat_end_effector_to_bolt_driver_slot,
    ) = subtract_frame_transforms(
        t01=tf_pos_end_effector,
        q01=tf_quat_end_effector,
        t02=tf_pos_bolt_driver_slot,
        q02=tf_quat_bolt_driver_slot,
    )
    tf_rotmat_end_effector_to_bolt_driver_slot = matrix_from_quat(
        tf_quat_end_effector_to_bolt_driver_slot
    )
    tf_rot6d_end_effector_to_bolt_driver_slot = rotmat_to_rot6d(
        tf_rotmat_end_effector_to_bolt_driver_slot
    )
    dist_end_effector_to_bolt_driver_slot = torch.norm(
        tf_pos_end_effector_to_bolt_driver_slot, dim=-1
    )
    # Object -> Target
    tf_pos_obj_to_target, tf_quat_obj_to_target = subtract_frame_transforms(
        t01=tf_pos_obj,
        q01=tf_quat_obj,
        t02=tf_pos_target,
        q02=tf_quat_target,
    )
    tf_rotmat_obj_to_target = matrix_from_quat(tf_quat_obj_to_target)
    tf_rot6d_obj_to_target = rotmat_to_rot6d(tf_rotmat_obj_to_target)

    ## Contacts
    contact_forces_mean_robot = contact_forces_robot.mean(dim=1)
    contact_forces_mean_end_effector = (
        contact_forces_end_effector.mean(dim=1)
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )
    contact_forces_end_effector = (
        contact_forces_end_effector
        if contact_forces_end_effector is not None
        else torch.empty((num_envs, 0), dtype=dtype, device=device)
    )

    #############
    ## Rewards ##
    #############
    # Penalty: Action rate
    WEIGHT_ACTION_RATE = -0.05
    penalty_action_rate = WEIGHT_ACTION_RATE * torch.sum(
        torch.square(act_current - act_previous), dim=1
    )

    # Penalty: Joint torque
    WEIGHT_JOINT_TORQUE = -0.00025
    MAX_JOINT_TORQUE_PENALTY = -4.0
    penalty_joint_torque = torch.clamp_min(
        WEIGHT_JOINT_TORQUE
        * torch.sum(torch.square(joint_applied_torque_robot), dim=1),
        min=MAX_JOINT_TORQUE_PENALTY,
    )

    # Penalty: Joint acceleration
    WEIGHT_JOINT_ACCELERATION = -0.005
    MAX_JOINT_ACCELERATION_PENALTY = -4.0
    penalty_joint_acceleration = torch.clamp_min(
        WEIGHT_JOINT_ACCELERATION * torch.sum(torch.square(joint_acc_robot), dim=1),
        min=MAX_JOINT_ACCELERATION_PENALTY,
    )

    # Penalty: Undesired robot contacts
    WEIGHT_UNDESIRED_ROBOT_CONTACTS = -1.0
    THRESHOLD_UNDESIRED_ROBOT_CONTACTS = 10.0
    penalty_undesired_robot_contacts = WEIGHT_UNDESIRED_ROBOT_CONTACTS * (
        torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]
        > THRESHOLD_UNDESIRED_ROBOT_CONTACTS
    )

    # Reward: End-effector top-down orientation
    WEIGHT_TOP_DOWN_ORIENTATION = 8.0
    TANH_STD_TOP_DOWN_ORIENTATION = 0.025
    top_down_alignment = torch.sum(
        fk_rotmat_end_effector[:, :, 2]
        * torch.tensor((0.0, 0.0, -1.0), device=device)
        .unsqueeze(0)
        .expand(num_envs, 3),
        dim=1,
    )
    reward_top_down_orientation = WEIGHT_TOP_DOWN_ORIENTATION * (
        1.0 - torch.tanh((1.0 - top_down_alignment) / TANH_STD_TOP_DOWN_ORIENTATION)
    )

    # Reward: Object upright orientation
    WEIGHT_OBJECT_TOP_DOWN_ORIENTATION = -64.0
    TANH_STD_OBJECT_TOP_DOWN_ORIENTATION = 0.05
    object_top_down_alignment = torch.sum(
        matrix_from_quat(tf_quat_obj)[:, :, 2]
        * torch.tensor((0.0, 0.0, 1.0), device=device).unsqueeze(0).expand(num_envs, 3),
        dim=1,
    )
    penalty_object_top_down_orientation = WEIGHT_OBJECT_TOP_DOWN_ORIENTATION * (
        torch.tanh(
            (1.0 - object_top_down_alignment) / TANH_STD_OBJECT_TOP_DOWN_ORIENTATION
        )
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT = 12.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT = 0.4
    reward_distance_end_effector_to_bolt_driver_slot = (
        WEIGHT_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT
        * (
            1.0
            - torch.tanh(
                dist_end_effector_to_bolt_driver_slot
                / TANH_STD_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT
            )
        )
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_CLOSE = 32.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_CLOSE = 0.02
    reward_distance_end_effector_to_bolt_driver_slot_close = (
        WEIGHT_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_CLOSE
        * (
            1.0
            - torch.tanh(
                dist_end_effector_to_bolt_driver_slot
                / TANH_STD_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_CLOSE
            )
        )
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_PRECISION = 512.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_PRECISION = 0.005
    reward_distance_end_effector_to_bolt_driver_slot_precision = (
        WEIGHT_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_PRECISION
        * (
            1.0
            - torch.tanh(
                dist_end_effector_to_bolt_driver_slot
                / TANH_STD_DISTANCE_END_EFFECTOR_TO_BOLT_DRIVER_SLOT_PRECISION
            )
        )
    )

    # Reward: Contact object
    WEIGHT_CONTACT = 4.0
    THRESHOLD_CONTACT = 5.0
    reward_contact = (
        WEIGHT_CONTACT
        * (
            torch.mean(
                torch.max(
                    torch.norm(contact_force_matrix_end_effector, dim=-1), dim=-1
                )[0],
                dim=1,
            )
            > THRESHOLD_CONTACT
        )
        if contact_force_matrix_end_effector is not None
        else torch.zeros(num_envs, dtype=dtype, device=device)
    )

    # Reward: Screwing based on angular velocity of the object
    WEIGHT_SCREWING = 128.0
    reward_screwing = WEIGHT_SCREWING * (-vel_ang_obj[:, 2] / torch.pi)

    # Penalty: Distance | Object <--> Target
    WEIGHT_DISTANCE_OBJ_TO_TARGET_TOO_FAR = -256.0
    THRESHOLD_DISTANCE_OBJ_TO_TARGET_TOO_FAR = 0.075
    is_obj_too_far = (
        torch.norm(tf_pos_obj_to_target[:, :2], dim=-1)
        > THRESHOLD_DISTANCE_OBJ_TO_TARGET_TOO_FAR
    )
    penalty_distance_obj_to_target_too_far = (
        WEIGHT_DISTANCE_OBJ_TO_TARGET_TOO_FAR * is_obj_too_far.float()
    )

    # Reward: Distance | Object <--> Target
    WEIGHT_DISTANCE_OBJ_TO_TARGET = 2048.0
    TANH_STD_DISTANCE_OBJ_TO_TARGET = 0.00075
    reward_distance_obj_to_target = (
        WEIGHT_DISTANCE_OBJ_TO_TARGET
        * (~is_obj_too_far).float()
        * (
            1.0
            - torch.tanh(
                torch.clip(-tf_pos_obj_to_target[:, 2], 0.0)
                / TANH_STD_DISTANCE_OBJ_TO_TARGET
            )
        )
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
                "vel_ang_obj": vel_ang_obj,
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                "tf_pos_end_effector_to_bolt_driver_slot": tf_pos_end_effector_to_bolt_driver_slot,
                "tf_rot6d_end_effector_to_bolt_driver_slot": tf_rot6d_end_effector_to_bolt_driver_slot,
                "tf_pos_obj_to_target": tf_pos_obj_to_target,
                "tf_rot6d_obj_to_target": tf_rot6d_obj_to_target,
            },
            "state_dyn": {
                "contact_forces_robot": contact_forces_robot,
                "contact_forces_end_effector": contact_forces_end_effector,
            },
            "proprio": {
                "fk_pos_end_effector": fk_pos_end_effector,
                "fk_rot6d_end_effector": fk_rot6d_end_effector,
            },
            "proprio_dyn": {
                "joint_pos_robot_normalized": joint_pos_robot_normalized,
                "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
                "joint_acc_robot": joint_acc_robot,
                "joint_applied_torque_robot": joint_applied_torque_robot,
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_joint_torque": penalty_joint_torque,
            "penalty_joint_acceleration": penalty_joint_acceleration,
            "penalty_undesired_robot_contacts": penalty_undesired_robot_contacts,
            "reward_top_down_orientation": reward_top_down_orientation,
            "penalty_object_top_down_orientation": penalty_object_top_down_orientation,
            "reward_distance_end_effector_to_bolt_driver_slot": reward_distance_end_effector_to_bolt_driver_slot,
            "reward_distance_end_effector_to_bolt_driver_slot_close": reward_distance_end_effector_to_bolt_driver_slot_close,
            "reward_distance_end_effector_to_bolt_driver_slot_precision": reward_distance_end_effector_to_bolt_driver_slot_precision,
            "reward_contact": reward_contact,
            "reward_screwing": reward_screwing,
            "penalty_distance_obj_to_target_too_far": penalty_distance_obj_to_target_too_far,
            "reward_distance_obj_to_target": reward_distance_obj_to_target,
        },
        termination,
        truncation,
    )
