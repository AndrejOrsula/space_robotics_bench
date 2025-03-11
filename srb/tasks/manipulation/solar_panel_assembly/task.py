from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import StepReturn
from srb.core.asset import (
    Articulation,
    AssetVariant,
    Object,
    RigidObject,
    RigidObjectCfg,
    RigidObjectCollection,
    RigidObjectCollectionCfg,
)
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationSceneCfg,
)
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import (
    reset_collection_root_state_uniform_poisson_disk_2d,
    reset_root_state_uniform,
)
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import PreviewSurfaceCfg
from srb.tasks.manipulation.peg_in_hole_assembly.asset import (
    select_peg_in_hole_assembly,
)
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    scale_transform,
    subtract_frame_transforms,
)

from .asset import select_solar_panel

##############
### Config ###
##############


@configclass
class SceneCfg(ManipulationSceneCfg):
    peg: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore
    )
    hole: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore
    )
    panel: RigidObjectCfg = MISSING  # type: ignore


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: EventTermCfg = EventTermCfg(
        func=reset_collection_root_state_uniform_poisson_disk_2d,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("peg"),
            "pose_range": {
                "x": (
                    -0.2,
                    0.05,
                ),
                "y": (
                    -0.05,
                    0.05,
                ),
                "z": (
                    0.025,
                    0.025,
                ),
                "roll": (torch.pi / 2, torch.pi / 2),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
            "radius": 0.2,
        },
    )
    randomize_panel_state: EventTermCfg = EventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("panel"),
            "pose_range": {
                "x": (-0.1 - 0.025, -0.1 + 0.025),
                "y": (-0.45 - 0.025, -0.45 + 0.025),
                "z": (-0.14, -0.14),
                "roll": (torch.pi, torch.pi),
                "pitch": (-torch.pi + torch.pi / 7, -torch.pi + torch.pi / 7),
                "yaw": (
                    torch.pi / 2 - torch.pi / 32,
                    torch.pi / 2 + torch.pi / 32,
                ),
            },
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    ## Assets
    peg: Object | AssetVariant = AssetVariant.DATASET

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Time
    episode_length_s: float = 50.0
    is_finite_horizon: bool = True

    ## Target - Panel
    panel_tf_pos_target = (0.55, 0.0, 0.0)
    panel_tf_quat_target = (1.0, 0.0, 0.0, 0.0)
    panel_target_marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/panel_target",
        markers={
            "target": assets.SolarPanel().asset_cfg.spawn.replace(  # type: ignore
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        # Task setup: Peg-in-hole assembly
        self.peg_in_hole_assemblies = [
            select_peg_in_hole_assembly(
                self,  # type: ignore
                seed=self.seed,
                prim_path_peg=f"{{ENV_REGEX_NS}}/peg{i}",
                prim_path_hole=f"{{ENV_REGEX_NS}}/hole{i}",
                init_state=RigidObjectCfg.InitialStateCfg(pos=init_pos),
                short_peg=short_peg,
                peg_kwargs={
                    "activate_contact_sensors": True,
                },
            )
            for i, (init_pos, short_peg) in enumerate(
                [
                    ((0.55 + 0.1, 0.2, 0.015), True),
                    ((0.55 + 0.1, -0.2, 0.015), True),
                    ((0.55 - 0.1, 0.2, 0.015), False),
                    ((0.55 - 0.1, -0.2, 0.015), False),
                ]
            )
        ]
        # Scene: Peg
        self.scene.peg.rigid_objects = {
            f"peg{i}": cfg.peg.asset_cfg.replace(  # type: ignore
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.5, 0.0, 0.13),
                )
            )
            for i, cfg in enumerate(self.peg_in_hole_assemblies)
        }
        # Scene: Hole
        self.scene.hole.rigid_objects = {
            f"hole{i}": cfg.hole.asset_cfg
            for i, cfg in enumerate(self.peg_in_hole_assemblies)
        }
        # Scene: Panel
        self.panel_cfg = select_solar_panel(
            self,
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.55, 0.0, 0.015)),
        )
        self.scene.panel = self.panel_cfg.asset_cfg
        self.panel_tf_pos_target = (
            self.panel_tf_pos_target[0] + self.panel_cfg.offset_pos[0],
            self.panel_tf_pos_target[1] + self.panel_cfg.offset_pos[1],
            self.panel_tf_pos_target[2] + self.panel_cfg.offset_pos[2] + 0.015,
        )

        # Sensor: End-effector contacts
        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                f"{{ENV_REGEX_NS}}/peg{i}" for i in range(4)
            ]

        # Update seed & number of variants for procedural assets
        self._update_procedural_assets()


############
### Task ###
############


class Task(ManipulationEnv):
    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._objs: RigidObjectCollection = self.scene["peg"]
        self._targets: RigidObjectCollection = self.scene["hole"]
        self._panel: RigidObject = self.scene["panel"]
        self._panel_target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.panel_target_marker_cfg
        )

        ## Initialize buffers
        self._tf_pos_objs_initial = torch.zeros(
            (self.num_envs, 4, 3),
            dtype=torch.float32,
            device=self.device,
        )
        self._offset_pos_pegs_ends = torch.tensor(
            [self.cfg.peg_in_hole_assemblies[i].peg.offset_pos_ends for i in range(4)],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1, 1)

        self._symmetry_pegs = torch.tensor(
            [self.cfg.peg_in_hole_assemblies[i].peg.symmetry for i in range(4)],
            dtype=torch.int32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._offset_pos_holes_bottom = torch.tensor(
            [
                self.cfg.peg_in_hole_assemblies[i].hole.offset_pos_bottom
                for i in range(4)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._offset_pos_holes_entrance = torch.tensor(
            [
                self.cfg.peg_in_hole_assemblies[i].hole.offset_pos_entrance
                for i in range(4)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._tf_pos_panel_initial = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._tf_pos_panel_target = (
            torch.tensor(
                self.cfg.panel_tf_pos_target, dtype=torch.float32, device=self.device
            ).repeat(self.num_envs, 1)
            + self.scene.env_origins
        )
        self._tf_quat_panel_target = torch.tensor(
            self.cfg.panel_tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        ## Visualize target
        self._panel_target_marker.visualize(
            self._tf_pos_panel_target, self._tf_quat_panel_target
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_objs_initial[env_ids] = self._objs.data.object_pos_w[env_ids]
        self._tf_pos_panel_initial[env_ids] = self._panel.data.root_pos_w[env_ids]

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
            # Kinematics
            fk_pos_end_effector=self._tf_end_effector.data.target_pos_source[:, 0, :],
            fk_quat_end_effector=self._tf_end_effector.data.target_quat_source[:, 0, :],
            # Transforms (world frame)
            tf_pos_end_effector=self._tf_end_effector.data.target_pos_w[:, 0, :],
            tf_quat_end_effector=self._tf_end_effector.data.target_quat_w[:, 0, :],
            tf_pos_objs_initial=self._tf_pos_objs_initial,
            tf_pos_objs=self._objs.data.object_pos_w,
            tf_quat_objs=self._objs.data.object_quat_w,
            tf_pos_targets=self._targets.data.object_pos_w,
            tf_quat_targets=self._targets.data.object_quat_w,
            tf_pos_panel_initial=self._tf_pos_panel_initial,
            tf_pos_panel=self._panel.data.root_com_pos_w,
            tf_quat_panel=self._panel.data.root_com_quat_w,
            tf_pos_panel_target=self._tf_pos_panel_target,
            tf_quat_panel_target=self._tf_quat_panel_target,
            # Contacts
            contact_forces_robot=self._contacts_robot.data.net_forces_w,  # type: ignore
            contact_forces_end_effector=self._contacts_end_effector.data.net_forces_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            contact_force_matrix_end_effector=self._contacts_end_effector.data.force_matrix_w
            if isinstance(self._contacts_end_effector, ContactSensor)
            else None,
            ## Peg-in-hole descriptors
            offset_pos_pegs_ends=self._offset_pos_pegs_ends,
            symmetry_peg=self._symmetry_pegs,
            offset_pos_holes_bottom=self._offset_pos_holes_bottom,
            offset_pos_holes_entrance=self._offset_pos_holes_entrance,
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
    # Kinematics
    fk_pos_end_effector: torch.Tensor,
    fk_quat_end_effector: torch.Tensor,
    # Transforms (world frame)
    tf_pos_end_effector: torch.Tensor,
    tf_quat_end_effector: torch.Tensor,
    tf_pos_objs_initial: torch.Tensor,
    tf_pos_objs: torch.Tensor,
    tf_quat_objs: torch.Tensor,
    tf_pos_targets: torch.Tensor,
    tf_quat_targets: torch.Tensor,
    tf_pos_panel_initial: torch.Tensor,
    tf_pos_panel: torch.Tensor,
    tf_quat_panel: torch.Tensor,
    tf_pos_panel_target: torch.Tensor,
    tf_quat_panel_target: torch.Tensor,
    # Contacts
    contact_forces_robot: torch.Tensor,
    contact_forces_end_effector: torch.Tensor | None,
    contact_force_matrix_end_effector: torch.Tensor | None,
    ## Peg-in-hole descriptors
    offset_pos_pegs_ends: torch.Tensor,
    symmetry_peg: torch.Tensor,
    offset_pos_holes_bottom: torch.Tensor,
    offset_pos_holes_entrance: torch.Tensor,
) -> StepReturn:
    num_envs = episode_length.size(0)
    num_objs = tf_pos_objs.size(1)
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
    # End-effector -> Object
    tf_pos_end_effector_to_objs, tf_quat_end_effector_to_objs = (
        subtract_frame_transforms(
            t01=tf_pos_end_effector.unsqueeze(1).repeat(1, num_objs, 1),
            q01=tf_quat_end_effector.unsqueeze(1).repeat(1, num_objs, 1),
            t02=tf_pos_objs,
            q02=tf_quat_objs,
        )
    )
    tf_rotmat_end_effector_to_objs = matrix_from_quat(tf_quat_end_effector_to_objs)
    tf_rot6d_end_effector_to_objs = rotmat_to_rot6d(tf_rotmat_end_effector_to_objs)

    # Peg -> Peg ends
    _tf_pos_pegs_end0, _ = combine_frame_transforms(
        t01=tf_pos_objs,
        q01=tf_quat_objs,
        t12=offset_pos_pegs_ends[:, :, 0],
    )
    _tf_pos_pegs_end1, _ = combine_frame_transforms(
        t01=tf_pos_objs,
        q01=tf_quat_objs,
        t12=offset_pos_pegs_ends[:, :, 1],
    )
    tf_pos_pegs_ends = torch.stack([_tf_pos_pegs_end0, _tf_pos_pegs_end1], dim=2)

    # Hole -> Hole entrance | bottom
    tf_pos_holes_entrance, _ = combine_frame_transforms(
        t01=tf_pos_targets,
        q01=tf_quat_targets,
        t12=offset_pos_holes_entrance,
    )
    tf_pos_holes_bottom, _ = combine_frame_transforms(
        t01=tf_pos_targets,
        q01=tf_quat_targets,
        t12=offset_pos_holes_bottom,
    )

    # Peg ends -> Hole entrance
    tf_pos_pegs_end0_to_holes_entrance, tf_quat_pegs_to_holes_entrance = (
        subtract_frame_transforms(
            t01=tf_pos_pegs_ends[:, :, 0],
            q01=tf_quat_objs,
            t02=tf_pos_holes_entrance,
            q02=tf_quat_targets,
        )
    )
    tf_pos_pegs_end1_to_holes_entrance, _ = subtract_frame_transforms(
        t01=tf_pos_pegs_ends[:, :, 1],
        q01=tf_quat_objs,
        t02=tf_pos_holes_entrance,
    )
    tf_pos_pegs_ends_to_holes_entrance = torch.stack(
        [tf_pos_pegs_end0_to_holes_entrance, tf_pos_pegs_end1_to_holes_entrance], dim=2
    )
    tf_rotmat_pegs_to_holes = matrix_from_quat(tf_quat_pegs_to_holes_entrance)
    tf_rot6d_pegs_to_holes = rotmat_to_rot6d(tf_rotmat_pegs_to_holes)

    # Peg ends -> Hole bottom
    tf_pos_pegs_end0_to_holes_bottom, _ = subtract_frame_transforms(
        t01=tf_pos_pegs_ends[:, :, 0],
        q01=tf_quat_objs,
        t02=tf_pos_holes_bottom,
    )
    tf_pos_pegs_end1_to_holes_bottom, _ = subtract_frame_transforms(
        t01=tf_pos_pegs_ends[:, :, 1],
        q01=tf_quat_objs,
        t02=tf_pos_holes_bottom,
    )
    tf_pos_pegs_ends_to_holes_bottom = torch.stack(
        [tf_pos_pegs_end0_to_holes_bottom, tf_pos_pegs_end1_to_holes_bottom], dim=1
    )

    # End-effector -> Panel CoM
    pos_robot_ee_to_panel, quat_robot_ee_to_panel = subtract_frame_transforms(
        t01=tf_pos_end_effector,
        q01=tf_quat_end_effector,
        t02=tf_pos_panel,
        q02=tf_quat_panel,
    )
    rotmat_robot_ee_to_panel = matrix_from_quat(quat_robot_ee_to_panel)
    rot6d_robot_ee_to_panel = rotmat_to_rot6d(rotmat_robot_ee_to_panel)

    # Panel CoM -> Panel Target
    pos_panel_to_panel_target, quat_panel_to_panel_target = subtract_frame_transforms(
        t01=tf_pos_panel,
        q01=tf_quat_panel,
        t02=tf_pos_panel_target,
        q02=tf_quat_panel_target,
    )
    rotmat_panel_to_panel_target = matrix_from_quat(quat_panel_to_panel_target)
    rot6d_panel_to_panel_target = rotmat_to_rot6d(rotmat_panel_to_panel_target)

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

    # Penalty: Undesired robot contacts
    WEIGHT_UNDESIRED_ROBOT_CONTACTS = -0.1
    THRESHOLD_UNDESIRED_ROBOT_CONTACTS = 10.0
    penalty_undesired_robot_contacts = WEIGHT_UNDESIRED_ROBOT_CONTACTS * (
        torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]
        > THRESHOLD_UNDESIRED_ROBOT_CONTACTS
    )

    # Reward: Distance | End-effector <--> Object
    WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ = 1.0
    TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ = 0.25
    reward_distance_end_effector_to_objs = WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ * (
        1.0
        - torch.tanh(
            torch.norm(tf_pos_end_effector_to_objs, dim=-1)
            / TANH_STD_DISTANCE_END_EFFECTOR_TO_OBJ
        )
    ).sum(dim=-1)

    # Reward: Grasp object
    WEIGHT_GRASP = 4.0
    THRESHOLD_GRASP = 5.0
    reward_grasp = (
        WEIGHT_GRASP
        * (
            torch.mean(
                torch.max(
                    torch.norm(contact_force_matrix_end_effector, dim=-1), dim=-1
                )[0],
                dim=1,
            )
            > THRESHOLD_GRASP
        )
        if contact_force_matrix_end_effector is not None
        else torch.zeros(num_envs, dtype=dtype, device=device)
    )

    # Reward: Lift object
    WEIGHT_LIFT = 16.0
    HEIGHT_OFFSET_LIFT = 0.5
    HEIGHT_SPAN_LIFT = 0.25
    TANH_STD_HEIGHT_LIFT = 0.1
    reward_lift = WEIGHT_LIFT * (
        1.0
        - torch.tanh(
            (
                torch.abs(
                    tf_pos_objs[:, :, 2]
                    - tf_pos_objs_initial[:, :, 2]
                    - HEIGHT_OFFSET_LIFT
                )
                - HEIGHT_SPAN_LIFT
            ).clamp(min=0.0)
            / TANH_STD_HEIGHT_LIFT
        )
    ).sum(dim=-1)

    # Reward: Alignment | Peg -> Hole | Primary Z axis
    WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY = 8.0
    TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY = 0.5
    _pegs_to_holes_primary_axis_similarity = torch.abs(
        tf_rotmat_pegs_to_holes[:, :, 2, 2]
    )
    reward_align_pegs_to_holes_primary = WEIGHT_ALIGN_PEG_TO_HOLE_PRIMARY * (
        1.0
        - torch.tanh(
            (1.0 - _pegs_to_holes_primary_axis_similarity)
            / TANH_STD_ALIGN_PEG_TO_HOLE_PRIMARY
        )
    ).sum(dim=-1)

    # Reward: Alignment | Peg -> Hole | Secondary XY axes (affected by primary via power)
    WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY = 4.0
    TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY = 0.2
    _pegs_to_holes_yaw = torch.atan2(
        tf_rotmat_pegs_to_holes[:, :, 0, 1], tf_rotmat_pegs_to_holes[:, :, 0, 0]
    )
    _symmetry_step = 2 * torch.pi / symmetry_peg
    _pegs_to_holes_yaw_symmetric_directional = _pegs_to_holes_yaw % _symmetry_step
    # Note: Lines above might result in NaN/inf when `peg_rot_symmetry_n=0` (infinite circular symmetry)
    #       However, the following `torch.where()` will handle this case
    _pegs_to_holes_yaw_symmetric_normalized = torch.where(
        symmetry_peg <= 0,
        0.0,
        torch.min(
            _pegs_to_holes_yaw_symmetric_directional,
            _symmetry_step - _pegs_to_holes_yaw_symmetric_directional,
        )
        / (_symmetry_step / 2.0),
    )
    reward_align_pegs_to_holes_secondary = WEIGHT_ALIGN_PEG_TO_HOLE_SECONDARY * (
        1.0
        - torch.tanh(
            _pegs_to_holes_yaw_symmetric_normalized.pow(
                _pegs_to_holes_primary_axis_similarity
            )
            / TANH_STD_ALIGN_PEG_TO_HOLE_SECONDARY
        )
    ).sum(dim=-1)

    # Reward: Distance | Peg -> Hole entrance
    WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE = 16.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE = 0.05
    reward_distance_pegs_to_holes_entrance = WEIGHT_DISTANCE_PEG_TO_HOLE_ENTRANCE * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(tf_pos_pegs_ends_to_holes_entrance, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_ENTRANCE
        )
    ).sum(dim=-1)

    # Reward: Distance | Peg -> Hole bottom
    WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM = 128.0
    TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM = 0.005
    reward_distance_pegs_to_holes_bottom = WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM * (
        1.0
        - torch.tanh(
            torch.min(torch.norm(tf_pos_pegs_ends_to_holes_bottom, dim=-1), dim=1)[0]
            / TANH_STD_DISTANCE_PEG_TO_HOLE_BOTTOM
        )
    ).sum(dim=-1)

    # Reward: Distance | End-effector <--> Panel
    WEIGHT_DISTANCE_EE_TO_PANEL = 4.0 * WEIGHT_DISTANCE_END_EFFECTOR_TO_OBJ
    TANH_STD_DISTANCE_EE_TO_PANEL = 0.25
    reward_distance_ee_to_panel = WEIGHT_DISTANCE_EE_TO_PANEL * (
        1.0
        - torch.tanh(
            torch.norm(pos_robot_ee_to_panel, dim=-1) / TANH_STD_DISTANCE_EE_TO_PANEL
        )
    )

    # Reward: Panel lifted
    WEIGHT_PANEL_LIFTED = 2 * WEIGHT_LIFT
    HEIGHT_OFFSET_PANEL_LIFTED = 0.5
    HEIGHT_SPAN_PANEL_LIFTED = 0.25
    TANH_STD_HEIGHT_PANEL_LIFTED = 0.1
    panel_target_height_offset = (
        torch.abs(
            tf_pos_panel[:, 2] - tf_pos_panel_initial[:, 2] - HEIGHT_OFFSET_PANEL_LIFTED
        )
        - HEIGHT_SPAN_PANEL_LIFTED
    ).clamp(min=0.0)
    reward_panel_lifted = WEIGHT_PANEL_LIFTED * (
        1.0 - torch.tanh(panel_target_height_offset / TANH_STD_HEIGHT_PANEL_LIFTED)
    )

    # Reward: Distance | Panel <--> Panel Target
    WEIGHT_DISTANCE_PANEL_TO_TARGET = 4.0 * WEIGHT_DISTANCE_PEG_TO_HOLE_BOTTOM
    TANH_STD_DISTANCE_PANEL_TO_TARGET = 0.05
    reward_distance_panel_to_target = WEIGHT_DISTANCE_PANEL_TO_TARGET * (
        1.0
        - torch.tanh(
            torch.norm(pos_panel_to_panel_target, dim=-1)
            / TANH_STD_DISTANCE_PANEL_TO_TARGET
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
                "contact_forces_mean_robot": contact_forces_mean_robot,
                "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                "tf_pos_end_effector_to_objs": tf_pos_end_effector_to_objs,
                "tf_rot6d_end_effector_to_objs": tf_rot6d_end_effector_to_objs,
                "tf_pos_pegs_end0_to_holes_entrance": tf_pos_pegs_end0_to_holes_entrance,
                "tf_pos_pegs_end1_to_holes_entrance": tf_pos_pegs_end1_to_holes_entrance,
                "tf_pos_pegs_end0_to_holes_bottom": tf_pos_pegs_end0_to_holes_bottom,
                "tf_pos_pegs_end1_to_holes_bottom": tf_pos_pegs_end1_to_holes_bottom,
                "tf_rot6d_pegs_to_holes": tf_rot6d_pegs_to_holes,
                "pos_robot_ee_to_panel": pos_robot_ee_to_panel,
                "rot6d_robot_ee_to_panel": rot6d_robot_ee_to_panel,
                "pos_panel_to_panel_target": pos_panel_to_panel_target,
                "rot6d_panel_to_panel_target": rot6d_panel_to_panel_target,
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
            },
        },
        {
            "penalty_action_rate": penalty_action_rate,
            "penalty_undesired_robot_contacts": penalty_undesired_robot_contacts,
            "reward_distance_end_effector_to_objs": reward_distance_end_effector_to_objs,
            "reward_grasp": reward_grasp,
            "reward_lift": reward_lift,
            "reward_align_pegs_to_holes_primary": reward_align_pegs_to_holes_primary,
            "reward_align_pegs_to_holes_secondary": reward_align_pegs_to_holes_secondary,
            "reward_distance_pegs_to_holes_entrance": reward_distance_pegs_to_holes_entrance,
            "reward_distance_pegs_to_holes_bottom": reward_distance_pegs_to_holes_bottom,
            "reward_distance_ee_to_panel": reward_distance_ee_to_panel,
            "reward_panel_lifted": reward_panel_lifted,
            "reward_distance_panel_to_target": reward_distance_panel_to_target,
        },
        termination,
        truncation,
    )
