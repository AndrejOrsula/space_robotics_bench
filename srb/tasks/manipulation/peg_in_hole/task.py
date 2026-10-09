from dataclasses import MISSING
from typing import Sequence

import torch

from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetBase,
    AssetBaseCfg,
    AssetVariant,
    RigidObject,
    RigidObjectCfg,
    Subterrane,
    Terrain,
)
from srb.core.env import (
    ManipulationEnv,
    ManipulationEnvCfg,
    ManipulationEventCfg,
    ManipulationRewardCfg,
    ManipulationSceneCfg,
)
from srb.core.env.manipulation._step_state import (
    manipulator_observation_tail,
    manipulator_step_state,
)
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import (
    push_by_setting_velocity,  # noqa: F401
    reset_root_state_uniform,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianRewardTerm,
    GaussianTrackingRewardTerm,
    GaussianZoneTrackingRewardTerm,
    HyperbolicRewardTerm,
    SoftThresholdRewardTerm,
)
from srb.core.scene_introspection import get_scene_entity, get_scene_rigid_object
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)

from .asset import select_peg_in_hole_assembly

##############
### Config ###
##############


class RewardCfg(ManipulationRewardCfg):
    """Peg approach, grasp, alignment, and coarse-to-fine insertion rewards."""

    reward_top_down_orientation: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=1.0, target=1.0, scale=0.25)
    )
    """Top-down end-effector orientation with a wide alignment plateau."""

    reward_distance_ee_to_obj: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=4.0, scale=0.25
    )
    """Dense approach guidance; half reward at 25 cm."""

    reward_grasp: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=8.0, threshold=5.0, operator=">", sharpness=1.0
    )
    """Smooth grasp reward around the 5 N contact threshold."""

    reward_lift: GaussianZoneTrackingRewardTerm = GaussianZoneTrackingRewardTerm(
        weight=4.0, target=0.2, tolerance=0.1, scale=0.08
    )
    """Rewards lifting the peg 20 cm, with 10 cm tolerance."""

    reward_align_peg_to_hole_primary: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=8.0, target=1.0, scale=0.4)
    )
    """Rewards primary-axis alignment, with about 20 degrees of tolerance."""

    reward_align_peg_to_hole_secondary: GaussianRewardTerm = GaussianRewardTerm(
        weight=4.0, scale=0.2
    )
    """Rewards symmetry-aware secondary-axis alignment."""

    reward_distance_peg_to_hole_entrance_gradual: HyperbolicRewardTerm = (
        HyperbolicRewardTerm(weight=8.0, scale=0.16)
    )
    """Coarse entrance guidance with a 0.16 m scale."""

    reward_distance_peg_to_hole_entrance: GaussianRewardTerm = GaussianRewardTerm(
        weight=32.0, scale=0.04
    )
    """Close-range entrance reward with a 0.04 m scale."""

    reward_distance_peg_to_hole_bottom: GaussianRewardTerm = GaussianRewardTerm(
        weight=256.0, scale=0.003
    )
    """Precision insertion reward with a 0.003 m scale."""


@configclass
class SceneCfg(ManipulationSceneCfg):
    peg: RigidObjectCfg = MISSING  # type: ignore[assignment]
    hole: AssetBaseCfg = MISSING  # type: ignore[assignment]


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("peg"),
            "pose_range": {
                "x": (-0.3 - 0.025, -0.3 + 0.0125),
                "y": (-0.05, 0.05),
                "roll": (torch.pi / 2, torch.pi / 2),
                "yaw": (
                    torch.pi / 2 - torch.pi / 16,
                    torch.pi / 2 + torch.pi / 16,
                ),
            },
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    """Insert a peg into a matching hole."""

    ## Assets
    scenery: Terrain | Subterrane | AssetVariant | None = AssetVariant.PROCEDURAL
    # The selector pairs each peg with a matching hole, so only variants apply.
    peg: AssetVariant = AssetVariant.DATASET

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    episode_length_s: float = 10.0
    is_finite_horizon: bool = True

    ## Target
    tf_pos_target: tuple[float, float, float] = (0.65, 0.0, 0.02)
    tf_quat_target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        super().__post_init__()

        self.peg_in_hole_assembly = select_peg_in_hole_assembly(
            self,
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=self.tf_pos_target, rot=self.tf_quat_target
            ),
            peg_kwargs={
                "activate_contact_sensors": True,
            },
        )
        self.scene.peg = self.peg_in_hole_assembly.peg.asset_cfg
        self.scene.hole = self.peg_in_hole_assembly.hole.asset_cfg
        # Variants multiply scene clones and consume the procedural budget.

        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                self.scene.peg.prim_path
            ]

        # Refresh procedural asset seeds and variant counts.
        self._update_procedural_assets()


############
### Task ###
############


class _TaskMixin:
    """Align and insert a peg using pose, contact, and depth rewards."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        ## Get scene assets
        self._obj: RigidObject = get_scene_rigid_object(self.scene, "peg")
        self._target: AssetBase = get_scene_entity(self.scene, "hole")

        ## Initialize buffers
        self._tf_pos_obj_initial = torch.zeros(
            (self.num_envs, 3), dtype=torch.float32, device=self.device
        )
        self._offset_pos_peg_ends = torch.tensor(
            self.cfg.peg_in_hole_assembly.peg.offset_pos_ends,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._symmetry_peg = torch.tensor(
            self.cfg.peg_in_hole_assembly.peg.symmetry,
            dtype=torch.int32,
            device=self.device,
        ).repeat(self.num_envs)
        self._offset_pos_hole_bottom = torch.tensor(
            self.cfg.peg_in_hole_assembly.hole.offset_pos_bottom,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._offset_pos_hole_entrance = torch.tensor(
            self.cfg.peg_in_hole_assembly.hole.offset_pos_entrance,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._tf_pos_target = self.scene.env_origins + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_obj_initial[env_ids] = self._obj.data.root_pos_w[env_ids]

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        # Transforms (world frame)
        tf_pos_obj = self._obj.data.root_pos_w
        tf_quat_obj = self._obj.data.root_quat_w

        tf_pos_ee_to_obj, tf_quat_ee_to_obj = subtract_frame_transforms(
            t01=state.tf_pos_ee, q01=state.tf_quat_ee, t02=tf_pos_obj, q02=tf_quat_obj
        )
        tf_rotmat_ee_to_obj = matrix_from_quat(tf_quat_ee_to_obj)
        tf_rot6d_ee_to_obj = rotmat_to_rot6d(tf_rotmat_ee_to_obj)

        # Peg -> Peg ends
        _tf_pos_peg_end0, _ = combine_frame_transforms(
            t01=tf_pos_obj,
            q01=tf_quat_obj,
            t12=self._offset_pos_peg_ends[:, 0],
        )
        _tf_pos_peg_end1, _ = combine_frame_transforms(
            t01=tf_pos_obj,
            q01=tf_quat_obj,
            t12=self._offset_pos_peg_ends[:, 1],
        )
        tf_pos_peg_ends = torch.stack([_tf_pos_peg_end0, _tf_pos_peg_end1], dim=1)

        # Hole -> Hole entrance | bottom
        tf_pos_hole_entrance, _ = combine_frame_transforms(
            t01=self._tf_pos_target,
            q01=self._tf_quat_target,
            t12=self._offset_pos_hole_entrance,
        )
        tf_pos_hole_bottom, _ = combine_frame_transforms(
            t01=self._tf_pos_target,
            q01=self._tf_quat_target,
            t12=self._offset_pos_hole_bottom,
        )

        # Peg ends -> Hole entrance
        tf_pos_peg_end0_to_hole_entrance, tf_quat_peg_to_hole_entrance = (
            subtract_frame_transforms(
                t01=tf_pos_peg_ends[:, 0],
                q01=tf_quat_obj,
                t02=tf_pos_hole_entrance,
                q02=self._tf_quat_target,
            )
        )
        tf_pos_peg_end1_to_hole_entrance, _ = subtract_frame_transforms(
            t01=tf_pos_peg_ends[:, 1],
            q01=tf_quat_obj,
            t02=tf_pos_hole_entrance,
        )
        tf_pos_peg_ends_to_hole_entrance = torch.stack(
            [tf_pos_peg_end0_to_hole_entrance, tf_pos_peg_end1_to_hole_entrance],
            dim=1,
        )
        tf_rotmat_peg_to_hole = matrix_from_quat(tf_quat_peg_to_hole_entrance)
        tf_rot6d_peg_to_hole = rotmat_to_rot6d(tf_rotmat_peg_to_hole)

        # Peg ends -> Hole bottom
        tf_pos_peg_end0_to_hole_bottom, _ = subtract_frame_transforms(
            t01=tf_pos_peg_ends[:, 0],
            q01=tf_quat_obj,
            t02=tf_pos_hole_bottom,
        )
        tf_pos_peg_end1_to_hole_bottom, _ = subtract_frame_transforms(
            t01=tf_pos_peg_ends[:, 1],
            q01=tf_quat_obj,
            t02=tf_pos_hole_bottom,
        )
        tf_pos_peg_ends_to_hole_bottom = torch.stack(
            [tf_pos_peg_end0_to_hole_bottom, tf_pos_peg_end1_to_hole_bottom],
            dim=1,
        )

        # Contacts
        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        has_ee_contacts = isinstance(self._contacts_end_effector, ContactSensor)
        if has_ee_contacts:
            contact_forces_ee = self._contacts_end_effector.data.net_forces_w
            contact_forces_mean_ee = contact_forces_ee.mean(dim=1)
            contact_force_matrix_ee = self._contacts_end_effector.data.force_matrix_w
        else:
            contact_forces_ee = torch.empty((num_envs, 0), dtype=dtype, device=device)
            contact_forces_mean_ee = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )
            contact_force_matrix_ee = None

        #####################
        ## Reward  metrics ##
        #####################
        action_rate_metric = torch.mean(
            torch.square(state.act_current - state.act_previous), dim=1
        )
        joint_torque_metric = torch.sum(
            torch.square(state.joint_applied_torque_robot), dim=1
        )
        joint_acc_metric = torch.sum(torch.square(state.joint_acc_robot), dim=1)
        undesired_contacts_metric = torch.max(
            torch.norm(contact_forces_robot, dim=-1), dim=1
        )[0]

        # Top-down orientation
        down_vec = (
            torch.tensor((0.0, 0.0, -1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3)
        )
        # World down; the FK frame follows the (possibly tilted) base.
        ee_rotmat_w = matrix_from_quat(state.tf_quat_ee)
        top_down_alignment = torch.sum(ee_rotmat_w[:, :, 2] * down_vec, dim=1)

        distance_ee_to_obj = torch.norm(tf_pos_ee_to_obj, dim=-1)

        grasp_metric: torch.Tensor
        if contact_force_matrix_ee is not None:
            grasp_metric = torch.mean(
                torch.max(torch.norm(contact_force_matrix_ee, dim=-1), dim=-1)[0],
                dim=1,
            )
        else:
            grasp_metric = torch.zeros(num_envs, dtype=dtype, device=device)

        lift_height = tf_pos_obj[:, 2] - self._tf_pos_obj_initial[:, 2]

        # Primary Z-axis alignment
        peg_to_hole_primary_sim = torch.abs(tf_rotmat_peg_to_hole[:, 2, 2])

        # Secondary XY-axis alignment (symmetry-aware)
        peg_to_hole_yaw = torch.atan2(
            tf_rotmat_peg_to_hole[:, 0, 1],
            tf_rotmat_peg_to_hole[:, 0, 0],
        )
        symmetry_step = 2.0 * torch.pi / torch.clamp_min(self._symmetry_peg, 1)
        yaw_sym_dir = peg_to_hole_yaw % symmetry_step
        # Note: Lines above might result in NaN/inf when `peg_rot_symmetry_n=0` (infinite circular symmetry)
        yaw_sym_norm = torch.where(
            self._symmetry_peg <= 0,
            torch.zeros_like(peg_to_hole_yaw),
            torch.min(yaw_sym_dir, symmetry_step - yaw_sym_dir) / (symmetry_step / 2.0),
        )
        secondary_alignment_metric = torch.where(
            peg_to_hole_primary_sim > 0.0,
            yaw_sym_norm.pow(peg_to_hole_primary_sim),
            torch.ones_like(yaw_sym_norm),
        )

        dist_peg_to_hole_entrance = torch.min(
            torch.norm(tf_pos_peg_ends_to_hole_entrance, dim=-1), dim=1
        )[0]
        dist_peg_to_hole_bottom = torch.min(
            torch.norm(tf_pos_peg_ends_to_hole_bottom, dim=-1), dim=1
        )[0]

        #############
        ## Rewards ##
        #############
        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_joint_torque": rew.penalty_joint_torque(joint_torque_metric),
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(
                joint_acc_metric
            ),
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                undesired_contacts_metric
            ),
            "reward_top_down_orientation": rew.reward_top_down_orientation(
                top_down_alignment
            ),
            "reward_distance_ee_to_obj": rew.reward_distance_ee_to_obj(
                distance_ee_to_obj
            ),
            "reward_grasp": rew.reward_grasp(grasp_metric),
            "reward_lift": rew.reward_lift(lift_height),
            "reward_align_peg_to_hole_primary": rew.reward_align_peg_to_hole_primary(
                peg_to_hole_primary_sim
            ),
            # Secondary alignment is exponentiated by primary Z alignment.
            "reward_align_peg_to_hole_secondary": rew.reward_align_peg_to_hole_secondary(
                secondary_alignment_metric
            ),
            "reward_distance_peg_to_hole_entrance_gradual": rew.reward_distance_peg_to_hole_entrance_gradual(
                dist_peg_to_hole_entrance
            ),
            "reward_distance_peg_to_hole_entrance": rew.reward_distance_peg_to_hole_entrance(
                dist_peg_to_hole_entrance
            ),
            "reward_distance_peg_to_hole_bottom": rew.reward_distance_peg_to_hole_bottom(
                dist_peg_to_hole_bottom
            ),
        }

        ##################
        ## Terminations ##
        ##################
        # No termination condition
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
                    "contact_forces_mean_end_effector": contact_forces_mean_ee,
                    "tf_pos_end_effector_to_obj": tf_pos_ee_to_obj,
                    "tf_rot6d_end_effector_to_obj": tf_rot6d_ee_to_obj,
                    "tf_pos_peg_end0_to_hole_entrance": tf_pos_peg_end0_to_hole_entrance,
                    "tf_pos_peg_end1_to_hole_entrance": tf_pos_peg_end1_to_hole_entrance,
                    "tf_pos_peg_end0_to_hole_bottom": tf_pos_peg_end0_to_hole_bottom,
                    "tf_pos_peg_end1_to_hole_bottom": tf_pos_peg_end1_to_hole_bottom,
                    "tf_rot6d_peg_to_hole": tf_rot6d_peg_to_hole,
                },
                **manipulator_observation_tail(
                    state, contact_forces_robot, contact_forces_ee
                ),
            },
            reward,
            termination,
            truncation,
        )


class Task(_TaskMixin, SrbEnv, ManipulationEnv):
    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)
