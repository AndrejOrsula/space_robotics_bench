from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    Articulation,
    RigidObjectCfg,
    RigidObjectCollection,
    RigidObjectCollectionCfg,
)
from srb.core.env import ManipulationEnv
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_collection_root_state_uniform_poisson_disk_2d
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianRewardTerm,
    GaussianTrackingRewardTerm,
    HyperbolicRewardTerm,
    LinearRewardTerm,
)
from srb.core.scene_introspection import get_scene_rigid_object_collection
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    scale_transform,
    subtract_frame_transforms,
)
from srb.utils.sampling import sample_grid

from .asset import select_peg_in_hole_assembly
from .task import EventCfg, SceneCfg, TaskCfg
from .task import RewardCfg as PegInHoleRewardCfg


class MultiRewardCfg(PegInHoleRewardCfg):
    """Multi-peg rewards; object metrics are shaped and summed across pegs."""

    reward_distance_ee_to_obj: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=0.0, scale=0.25
    )
    reward_align_peg_to_hole_primary: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=0.0, target=1.0, scale=0.4)
    )
    reward_align_peg_to_hole_secondary: GaussianRewardTerm = GaussianRewardTerm(
        weight=0.0, scale=0.2
    )
    reward_distance_peg_to_hole_entrance_gradual: HyperbolicRewardTerm = (
        HyperbolicRewardTerm(weight=0.0, scale=0.16)
    )
    reward_distance_peg_to_hole_entrance: GaussianRewardTerm = GaussianRewardTerm(
        weight=0.0, scale=0.04
    )
    reward_distance_peg_to_hole_bottom: GaussianRewardTerm = GaussianRewardTerm(
        weight=0.0, scale=0.003
    )

    reward_distance_ee_to_objs: LinearRewardTerm = LinearRewardTerm(
        weight=2.5, limit=10.0
    )
    """EE proximity, summed across pegs; tanh std 0.2 m."""

    reward_grasp: LinearRewardTerm = LinearRewardTerm(weight=8.0, limit=32.0)
    """Binary grasp reward; 5 N threshold."""

    reward_lift: LinearRewardTerm = LinearRewardTerm(weight=4.0, limit=16.0)
    """Lift reward summed across pegs; tracks 0.2 m lift from initial height
    with a 0.1 m span."""

    reward_align_pegs_to_holes_primary: LinearRewardTerm = LinearRewardTerm(
        weight=8.0, limit=32.0
    )
    """Primary Z-axis alignment, summed across pegs."""

    reward_align_pegs_to_holes_secondary: LinearRewardTerm = LinearRewardTerm(
        weight=4.0, limit=16.0
    )
    """Secondary XY alignment, summed across pegs."""

    reward_distance_pegs_to_holes_entrance_gradual: LinearRewardTerm = LinearRewardTerm(
        weight=8.0, limit=32.0
    )
    """Gradual entrance distance; tanh std 0.16 m."""

    reward_distance_pegs_to_holes_entrance: LinearRewardTerm = LinearRewardTerm(
        weight=32.0, limit=128.0
    )
    """Precision entrance distance; tanh std 0.04 m."""

    reward_distance_pegs_to_holes_bottom: LinearRewardTerm = LinearRewardTerm(
        weight=256.0, limit=1024.0
    )
    """Bottom distance; tanh std 0.002 m."""

    tanh_std_distance_ee_to_objs: float = 0.2
    threshold_grasp: float = 5.0
    height_offset_lift: float = 0.2
    height_span_lift: float = 0.1
    tanh_std_lift: float = 0.05
    tanh_std_align_primary: float = 0.5
    tanh_std_align_secondary: float = 0.2
    tanh_std_entrance_gradual: float = 0.16
    tanh_std_entrance: float = 0.04
    tanh_std_bottom: float = 0.002


@configclass
class MultiSceneCfg(SceneCfg):
    peg: None = None
    pegs: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore[arg-type]
    )
    hole: None = None


@configclass
class MultiEventCfg(EventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_collection_root_state_uniform_poisson_disk_2d,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("pegs"),
            "pose_range": {
                "x": MISSING,
                "y": MISSING,
                "roll": (torch.pi / 2, torch.pi / 2),
                "pitch": (-torch.pi, torch.pi),
                "yaw": (-torch.pi, torch.pi),
            },
            "velocity_range": {},
            "radius": 0.1,
        },
    )


@configclass
class MultiTaskCfg(TaskCfg):
    scene: MultiSceneCfg = MultiSceneCfg()
    num_pegs: int = 4
    hole_spacing: float = 0.15

    events: MultiEventCfg = MultiEventCfg()

    reward: MultiRewardCfg = MultiRewardCfg()

    episode_length_s: float = MISSING  # type: ignore[assignment]
    _base_episode_length_s: float = 10.0

    def __post_init__(self):
        super().__post_init__()

        self.episode_length_s = self.num_pegs * self._base_episode_length_s

        self.scene.peg = None
        self.scene.hole = None
        (num_rows, num_cols), (grid_spacing_pos, grid_spacing_rot) = sample_grid(
            num_instances=self.num_pegs,
            spacing=self.hole_spacing,
            global_pos_offset=(0.5, 0.0, 0.1),
        )
        self.peg_in_hole_assemblies = [
            select_peg_in_hole_assembly(
                self,
                seed=self.seed + (i * self.scene.num_envs),
                prim_path_peg=f"{{ENV_REGEX_NS}}/peg{i}",
                prim_path_hole=f"{{ENV_REGEX_NS}}/hole{i}",
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=grid_spacing_pos[i],
                    rot=grid_spacing_rot[i],
                ),
                peg_kwargs={
                    "activate_contact_sensors": True,
                },
            )
            for i in range(self.num_pegs)
        ]
        self.scene.pegs.rigid_objects = {
            f"peg{i}": cfg.peg.asset_cfg.replace(  # type: ignore[union-attr]
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.5, 0.0, 0.13),
                )
            )
            for i, cfg in enumerate(self.peg_in_hole_assemblies)
        }
        for i, cfg in enumerate(self.peg_in_hole_assemblies):
            setattr(self.scene, f"hole{i}", cfg.hole.asset_cfg)

        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                f"{{ENV_REGEX_NS}}/peg{i}" for i in range(self.num_pegs)
            ]

        self.events.randomize_object_state.params["pose_range"]["x"] = (  # type: ignore[attr-defined]
            -0.5 * (num_rows - 0.5) * self.hole_spacing,
            0.5 * (num_rows - 0.5) * self.hole_spacing,
        )
        self.events.randomize_object_state.params["pose_range"]["y"] = (  # type: ignore[attr-defined]
            -0.5 * (num_cols - 0.5) * self.hole_spacing,
            0.5 * (num_cols - 0.5) * self.hole_spacing,
        )

        self._update_procedural_assets()


class _TaskMixin:
    cfg: MultiTaskCfg

    def __init__(self, cfg: MultiTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._objs: RigidObjectCollection = get_scene_rigid_object_collection(
            self.scene, "pegs"
        )

        hole_positions = []
        hole_quaternions = []
        for i in range(self.cfg.num_pegs):
            init_state = self.cfg.peg_in_hole_assemblies[i].hole.asset_cfg.init_state
            hole_positions.append(
                torch.tensor(init_state.pos, dtype=torch.float32, device=self.device)
            )
            hole_quaternions.append(
                torch.tensor(init_state.rot, dtype=torch.float32, device=self.device)
            )
        self._tf_pos_targets = self.scene.env_origins.unsqueeze(1) + torch.stack(
            hole_positions
        ).unsqueeze(0).expand(self.num_envs, -1, -1)
        self._tf_quat_targets = (
            torch.stack(hole_quaternions)
            .unsqueeze(0)
            .expand(self.num_envs, -1, -1)
            .clone()
        )

        self._tf_pos_objs_initial = torch.zeros(
            (self.num_envs, self.cfg.num_pegs, 3),
            dtype=torch.float32,
            device=self.device,
        )
        self._offset_pos_pegs_ends = torch.tensor(
            [
                self.cfg.peg_in_hole_assemblies[i].peg.offset_pos_ends
                for i in range(self.cfg.num_pegs)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1, 1)

        self._symmetry_pegs = torch.tensor(
            [
                self.cfg.peg_in_hole_assemblies[i].peg.symmetry
                for i in range(self.cfg.num_pegs)
            ],
            dtype=torch.int32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._offset_pos_holes_bottom = torch.tensor(
            [
                self.cfg.peg_in_hole_assemblies[i].hole.offset_pos_bottom
                for i in range(self.cfg.num_pegs)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._offset_pos_holes_entrance = torch.tensor(
            [
                self.cfg.peg_in_hole_assemblies[i].hole.offset_pos_entrance
                for i in range(self.cfg.num_pegs)
            ],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_objs_initial[env_ids] = self._objs.data.object_pos_w[env_ids]

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
        num_objs = self.cfg.num_pegs
        device = self.device
        dtype = torch.float32

        rew = self.cfg.reward

        act_current = self.action_manager.action
        act_previous = self.action_manager.prev_action

        joint_pos_robot = self._robot.data.joint_pos
        joint_pos_limits_robot = self._robot.data.soft_joint_pos_limits
        joint_acc_robot = self._robot.data.joint_acc
        joint_applied_torque_robot = self._robot.data.applied_torque

        joint_pos_robot_normalized = (
            scale_transform(
                joint_pos_robot,
                joint_pos_limits_robot[:, :, 0],
                joint_pos_limits_robot[:, :, 1],
            )
            if torch.all(torch.isfinite(joint_pos_limits_robot))
            else joint_pos_robot
        )

        if isinstance(self._end_effector, Articulation):
            joint_pos_ee = self._end_effector.data.joint_pos
            ee_limits = self._end_effector.data.soft_joint_pos_limits
            joint_pos_end_effector_normalized = (
                scale_transform(joint_pos_ee, ee_limits[:, :, 0], ee_limits[:, :, 1])
                if torch.all(torch.isfinite(ee_limits))
                else joint_pos_ee
            )
        else:
            joint_pos_end_effector_normalized = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )

        fk_pos_end_effector = self._tf_end_effector.data.target_pos_source[:, 0, :]
        fk_quat_end_effector = self._tf_end_effector.data.target_quat_source[:, 0, :]
        fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
        fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

        tf_pos_end_effector = self._tf_end_effector.data.target_pos_w[:, 0, :]
        tf_quat_end_effector = self._tf_end_effector.data.target_quat_w[:, 0, :]
        tf_pos_objs = self._objs.data.object_pos_w
        tf_quat_objs = self._objs.data.object_quat_w

        tf_pos_ee_to_objs, tf_quat_ee_to_objs = subtract_frame_transforms(
            t01=tf_pos_end_effector.unsqueeze(1).repeat(1, num_objs, 1),
            q01=tf_quat_end_effector.unsqueeze(1).repeat(1, num_objs, 1),
            t02=tf_pos_objs,
            q02=tf_quat_objs,
        )
        tf_rotmat_ee_to_objs = matrix_from_quat(tf_quat_ee_to_objs)
        tf_rot6d_ee_to_objs = rotmat_to_rot6d(tf_rotmat_ee_to_objs)

        _tf_pos_pegs_end0, _ = combine_frame_transforms(
            t01=tf_pos_objs,
            q01=tf_quat_objs,
            t12=self._offset_pos_pegs_ends[:, :, 0],
        )
        _tf_pos_pegs_end1, _ = combine_frame_transforms(
            t01=tf_pos_objs,
            q01=tf_quat_objs,
            t12=self._offset_pos_pegs_ends[:, :, 1],
        )
        tf_pos_pegs_ends = torch.stack([_tf_pos_pegs_end0, _tf_pos_pegs_end1], dim=2)

        tf_pos_holes_entrance, _ = combine_frame_transforms(
            t01=self._tf_pos_targets,
            q01=self._tf_quat_targets,
            t12=self._offset_pos_holes_entrance,
        )
        tf_pos_holes_bottom, _ = combine_frame_transforms(
            t01=self._tf_pos_targets,
            q01=self._tf_quat_targets,
            t12=self._offset_pos_holes_bottom,
        )

        tf_pos_pegs_end0_to_holes_entrance, tf_quat_pegs_to_holes_entrance = (
            subtract_frame_transforms(
                t01=tf_pos_pegs_ends[:, :, 0],
                q01=tf_quat_objs,
                t02=tf_pos_holes_entrance,
                q02=self._tf_quat_targets,
            )
        )
        tf_pos_pegs_end1_to_holes_entrance, _ = subtract_frame_transforms(
            t01=tf_pos_pegs_ends[:, :, 1],
            q01=tf_quat_objs,
            t02=tf_pos_holes_entrance,
        )
        tf_pos_pegs_ends_to_holes_entrance = torch.stack(
            [tf_pos_pegs_end0_to_holes_entrance, tf_pos_pegs_end1_to_holes_entrance],
            dim=2,
        )
        tf_rotmat_pegs_to_holes = matrix_from_quat(tf_quat_pegs_to_holes_entrance)
        tf_rot6d_pegs_to_holes = rotmat_to_rot6d(tf_rotmat_pegs_to_holes)

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
            [tf_pos_pegs_end0_to_holes_bottom, tf_pos_pegs_end1_to_holes_bottom],
            dim=2,
        )

        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        has_ee_contacts = isinstance(self._contacts_end_effector, ContactSensor)
        if has_ee_contacts:
            contact_forces_end_effector = self._contacts_end_effector.data.net_forces_w
            contact_forces_mean_end_effector = contact_forces_end_effector.mean(dim=1)
            contact_force_matrix_end_effector = (
                self._contacts_end_effector.data.force_matrix_w
            )
        else:
            contact_forces_end_effector = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )
            contact_forces_mean_end_effector = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )
            contact_force_matrix_end_effector = None

        reward: dict[str, torch.Tensor] = {}

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        reward["penalty_action_rate"] = rew.penalty_action_rate(action_rate_metric)

        joint_torque_metric = torch.sum(torch.square(joint_applied_torque_robot), dim=1)
        reward["penalty_joint_torque"] = rew.penalty_joint_torque(joint_torque_metric)

        joint_acc_metric = torch.sum(torch.square(joint_acc_robot), dim=1)
        reward["penalty_joint_acceleration"] = rew.penalty_joint_acceleration(
            joint_acc_metric
        )

        max_force = torch.max(torch.norm(contact_forces_robot, dim=-1), dim=1)[0]
        reward["penalty_undesired_robot_contacts"] = (
            rew.penalty_undesired_robot_contacts(max_force)
        )

        # World down; the FK frame follows the (possibly tilted) base.
        top_down_alignment = torch.sum(
            matrix_from_quat(tf_quat_end_effector)[:, :, 2]
            * torch.tensor((0.0, 0.0, -1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3),
            dim=1,
        )
        reward["reward_top_down_orientation"] = rew.reward_top_down_orientation(
            top_down_alignment
        )

        _dist_shaped = (
            1.0
            - torch.tanh(
                torch.norm(tf_pos_ee_to_objs, dim=-1) / rew.tanh_std_distance_ee_to_objs
            )
        ).sum(dim=-1)
        reward["reward_distance_ee_to_objs"] = rew.reward_distance_ee_to_objs(
            _dist_shaped
        )

        if contact_force_matrix_end_effector is not None:
            _grasp_metric = (
                torch.mean(
                    torch.max(
                        torch.norm(contact_force_matrix_end_effector, dim=-1),
                        dim=-1,
                    )[0],
                    dim=1,
                )
                > rew.threshold_grasp
            ).float()
        else:
            _grasp_metric = torch.zeros(num_envs, dtype=dtype, device=device)
        reward["reward_grasp"] = rew.reward_grasp(_grasp_metric)

        _lift_shaped = (
            1.0
            - torch.tanh(
                (
                    torch.abs(
                        tf_pos_objs[:, :, 2]
                        - self._tf_pos_objs_initial[:, :, 2]
                        - rew.height_offset_lift
                    )
                    - rew.height_span_lift
                ).clamp(min=0.0)
                / rew.tanh_std_lift
            )
        ).sum(dim=-1)
        reward["reward_lift"] = rew.reward_lift(_lift_shaped)

        _pegs_to_holes_primary_axis_similarity = torch.abs(
            tf_rotmat_pegs_to_holes[:, :, 2, 2]
        )
        _align_primary_shaped = (
            1.0
            - torch.tanh(
                (1.0 - _pegs_to_holes_primary_axis_similarity)
                / rew.tanh_std_align_primary
            )
        ).sum(dim=-1)
        reward["reward_align_pegs_to_holes_primary"] = (
            rew.reward_align_pegs_to_holes_primary(_align_primary_shaped)
        )

        _pegs_to_holes_yaw = torch.atan2(
            tf_rotmat_pegs_to_holes[:, :, 0, 1],
            tf_rotmat_pegs_to_holes[:, :, 0, 0],
        )
        _symmetry_step = 2 * torch.pi / torch.clamp_min(self._symmetry_pegs, 1)
        _pegs_to_holes_yaw_symmetric_directional = _pegs_to_holes_yaw % _symmetry_step
        # Note: Lines above might result in NaN/inf when `peg_rot_symmetry_n=0`
        #       (infinite circular symmetry)
        #       However, the following `torch.where()` will handle this case
        _pegs_to_holes_yaw_symmetric_normalized = torch.where(
            self._symmetry_pegs <= 0,
            0.0,
            torch.min(
                _pegs_to_holes_yaw_symmetric_directional,
                _symmetry_step - _pegs_to_holes_yaw_symmetric_directional,
            )
            / (_symmetry_step / 2.0),
        )
        # Secondary alignment is weighted by primary-axis similarity.
        _secondary_alignment = torch.where(
            _pegs_to_holes_primary_axis_similarity > 0.0,
            _pegs_to_holes_yaw_symmetric_normalized.pow(
                _pegs_to_holes_primary_axis_similarity
            ),
            torch.ones_like(_pegs_to_holes_yaw_symmetric_normalized),
        )
        _align_secondary_shaped = (
            1.0 - torch.tanh(_secondary_alignment / rew.tanh_std_align_secondary)
        ).sum(dim=-1)
        reward["reward_align_pegs_to_holes_secondary"] = (
            rew.reward_align_pegs_to_holes_secondary(_align_secondary_shaped)
        )

        _entrance_gradual_shaped = (
            1.0
            - torch.tanh(
                torch.min(
                    torch.norm(tf_pos_pegs_ends_to_holes_entrance, dim=-1), dim=2
                )[0]
                / rew.tanh_std_entrance_gradual
            )
        ).sum(dim=-1)
        reward["reward_distance_pegs_to_holes_entrance_gradual"] = (
            rew.reward_distance_pegs_to_holes_entrance_gradual(_entrance_gradual_shaped)
        )

        _entrance_shaped = (
            1.0
            - torch.tanh(
                torch.min(
                    torch.norm(tf_pos_pegs_ends_to_holes_entrance, dim=-1), dim=2
                )[0]
                / rew.tanh_std_entrance
            )
        ).sum(dim=-1)
        reward["reward_distance_pegs_to_holes_entrance"] = (
            rew.reward_distance_pegs_to_holes_entrance(_entrance_shaped)
        )

        _bottom_shaped = (
            1.0
            - torch.tanh(
                torch.min(torch.norm(tf_pos_pegs_ends_to_holes_bottom, dim=-1), dim=2)[
                    0
                ]
                / rew.tanh_std_bottom
            )
        ).sum(dim=-1)
        reward["reward_distance_pegs_to_holes_bottom"] = (
            rew.reward_distance_pegs_to_holes_bottom(_bottom_shaped)
        )

        # Zero single-object terms remain in reward output for logging completeness.
        _zero = torch.zeros(num_envs, dtype=dtype, device=device)
        reward["reward_distance_ee_to_obj"] = rew.reward_distance_ee_to_obj(_zero)
        reward["reward_align_peg_to_hole_primary"] = (
            rew.reward_align_peg_to_hole_primary(_zero)
        )
        reward["reward_align_peg_to_hole_secondary"] = (
            rew.reward_align_peg_to_hole_secondary(_zero)
        )
        reward["reward_distance_peg_to_hole_entrance_gradual"] = (
            rew.reward_distance_peg_to_hole_entrance_gradual(_zero)
        )
        reward["reward_distance_peg_to_hole_entrance"] = (
            rew.reward_distance_peg_to_hole_entrance(_zero)
        )
        reward["reward_distance_peg_to_hole_bottom"] = (
            rew.reward_distance_peg_to_hole_bottom(_zero)
        )

        # This task has no termination condition.
        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_end_effector,
                    "tf_pos_end_effector_to_objs": tf_pos_ee_to_objs,
                    "tf_rot6d_end_effector_to_objs": tf_rot6d_ee_to_objs,
                    "tf_pos_pegs_end0_to_holes_entrance": tf_pos_pegs_end0_to_holes_entrance,
                    "tf_pos_pegs_end1_to_holes_entrance": tf_pos_pegs_end1_to_holes_entrance,
                    "tf_pos_pegs_end0_to_holes_bottom": tf_pos_pegs_end0_to_holes_bottom,
                    "tf_pos_pegs_end1_to_holes_bottom": tf_pos_pegs_end1_to_holes_bottom,
                    "tf_rot6d_pegs_to_holes": tf_rot6d_pegs_to_holes,
                },
                ObservationGroup.STATE_DYN: {
                    "contact_forces_robot": contact_forces_robot,
                    "contact_forces_end_effector": contact_forces_end_effector,
                },
                ObservationGroup.PROPRIO: {
                    "fk_pos_end_effector": fk_pos_end_effector,
                    "fk_rot6d_end_effector": fk_rot6d_end_effector,
                },
                ObservationGroup.PROPRIO_DYN: {
                    "joint_pos_robot_normalized": joint_pos_robot_normalized,
                    "joint_pos_end_effector_normalized": joint_pos_end_effector_normalized,
                    "joint_acc_robot": joint_acc_robot,
                    "joint_applied_torque_robot": joint_applied_torque_robot,
                },
            },
            reward,
            termination,
            truncation,
        )


class MultiTask(_TaskMixin, SrbEnv, ManipulationEnv):
    cfg: MultiTaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: MultiTaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)
