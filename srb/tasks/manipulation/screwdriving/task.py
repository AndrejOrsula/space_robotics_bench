from dataclasses import MISSING

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetBase,
    AssetBaseCfg,
    AssetVariant,
    Manipulator,
    Object,
    RigidObject,
    RigidObjectCfg,
    Terrain,
)
from srb.core.domain import Domain
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
    GaussianTrackingRewardTerm,
    HyperbolicRewardTerm,
    LinearRewardTerm,
    ScaledRewardTerm,
    SoftThresholdRewardTerm,
    ThresholdRewardTerm,
)
from srb.core.scene_introspection import get_scene_entity, get_scene_rigid_object
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    rpy_to_quat,
    subtract_frame_transforms,
)


class RewardCfg(ManipulationRewardCfg):
    """Multi-scale bolt alignment, engagement, and threading rewards."""

    ## Uses standard ManipulationRewardCfg magnitudes

    ## Task-specific rewards
    reward_top_down_orientation: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=8.0, target=1.0, scale=0.04)
    )
    """Top-down orientation with about 2 degrees of tolerance."""

    penalty_object_top_down_orientation: LinearRewardTerm = LinearRewardTerm(
        weight=-64.0, limit=64.0
    )
    """Penalizes bolt tilt during approach."""

    reward_distance_ee_to_bolt_driver_slot: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=12.0, scale=0.4
    )
    """Coarse approach guidance with a 0.4 m scale."""

    reward_distance_ee_to_bolt_driver_slot_close: HyperbolicRewardTerm = (
        HyperbolicRewardTerm(weight=32.0, scale=0.02)
    )
    """Close-range approach guidance with a 0.02 m scale."""

    reward_distance_ee_to_bolt_driver_slot_precision: HyperbolicRewardTerm = (
        HyperbolicRewardTerm(weight=512.0, scale=0.005)
    )
    """Precision slot alignment with a 0.005 m scale."""

    reward_contact: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=4.0, threshold=5.0, operator=">", sharpness=1.0
    )
    """Smooth contact reward around the 5 N threshold."""

    reward_screwing: ScaledRewardTerm = ScaledRewardTerm(weight=128.0, limit=128.0)
    """Signed angular-velocity reward, normalized by pi."""

    penalty_distance_obj_to_target_too_far: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=-256.0, threshold=0.075, operator=">"
    )
    """Penalizes horizontal drift beyond 7.5 cm."""

    reward_distance_obj_to_target: LinearRewardTerm = LinearRewardTerm(
        weight=2048.0, limit=2048.0
    )
    """Rewards bolt depth when near the nut and upright."""

    ## Shaping parameters (not RewardTerms — plain floats)
    object_orientation_tanh_std: float = 0.05
    # Intentionally tight to reward sub-millimetre threading precision.
    distance_obj_to_target_tanh_std: float = 0.00075
    # Require the bolt to remain upright for thread progress to count.
    object_upright_min_alignment: float = 0.95


@configclass
class SceneCfg(ManipulationSceneCfg):
    bolt: RigidObjectCfg = MISSING  # type: ignore[assignment]
    nut: AssetBaseCfg = MISSING  # type: ignore[assignment]
    decor: AssetBaseCfg = assets.Ingenuity().as_asset_base_cfg(disable_collision=True)
    decor.prim_path = "{ENV_REGEX_NS}/decor"
    decor.init_state = AssetBaseCfg.InitialStateCfg(
        pos=(0.65, 0.0, 0.0),
        rot=rpy_to_quat(0.0, 0.0, 45.0),
    )


def _set_nut_spawn_target_pose(
    nut_cfg: AssetBaseCfg,
    target_pos: tuple[float, float, float],
    target_rot: tuple[float, float, float, float],
) -> None:
    """Set the nut spawn pose; quaternion components use xyzw order."""
    nut_cfg.init_state.pos = target_pos
    nut_cfg.init_state.rot = target_rot


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("bolt"),
            "pose_range": {},
            "velocity_range": {},
        },
    )


@configclass
class TaskCfg(ManipulationEnvCfg):
    """Drive a bolt into a threaded nut with an electric screwdriver."""

    ## Scenario
    domain: Domain = Domain.MARS

    ## Assets
    scenery: Terrain | AssetVariant | None = AssetVariant.PROCEDURAL
    robot: Manipulator | AssetVariant = assets.Franka(
        end_effector=assets.ElectricScrewdriverM5()
    )
    bolt: Object | AssetVariant | None = assets.BoltM8()

    ## Scene
    scene: SceneCfg = SceneCfg()

    ## Events
    events: EventCfg = EventCfg()

    ## Reward
    reward: RewardCfg = RewardCfg()

    ## Time
    env_rate: float = 1.0 / 500.0
    episode_length_s: float = 15.0
    is_finite_horizon: bool = True

    ## Target
    tf_pos_target: tuple[float, float, float] = (0.65, 0.0, 0.493)
    tf_quat_target: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        super().__post_init__()

        # Task setup: Nut from Bolt
        if isinstance(self.bolt, assets.BoltM8):
            nut = assets.NutM8()
            thread_offset = -0.000375
            thread_pitch = 0.00125
        else:
            raise ValueError(
                f"screwdriving supports assets.BoltM8, got {self.bolt!r}; "
                "set bolt to assets.BoltM8()"
            )
        # Scene: Nut
        self.scene.nut = nut.asset_cfg
        self.scene.nut.prim_path = "/World/nut" if self.stack else "{ENV_REGEX_NS}/nut"
        _set_nut_spawn_target_pose(
            self.scene.nut, self.tf_pos_target, self.tf_quat_target
        )
        # InteractiveScene replicates this env-scoped asset; variants multiply
        # the clone plan. Start the bolt three pitches above engagement.
        # Scene: Bolt
        self.scene.bolt = self.bolt.asset_cfg
        self.scene.bolt.prim_path = "{ENV_REGEX_NS}/bolt"
        target_x, target_y, target_z = self.tf_pos_target
        self.scene.bolt.init_state.pos = (
            target_x,
            target_y,
            target_z + thread_offset + 3.0 * thread_pitch,
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


def bolt_thread_progress_metrics(
    tf_pos_obj: torch.Tensor,
    tf_pos_target: torch.Tensor,
    obj_top_align: torch.Tensor,
    *,
    too_far_threshold: float,
    tanh_std: float,
    upright_min_alignment: float,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Measure world-horizontal drift and upright-gated insertion progress.

    World axes prevent a toppled bolt from appearing inserted in its own frame.
    Returns ``(horizontal_distance, shaped_progress)``; progress is in ``[0, 1]``
    and is zero when too far away or not upright.
    """
    tf_pos_obj_to_target_w = tf_pos_target - tf_pos_obj
    horizontal_distance = torch.norm(tf_pos_obj_to_target_w[:, :2], dim=-1)
    engaged = (horizontal_distance <= too_far_threshold) & (
        obj_top_align >= upright_min_alignment
    )
    shaped_progress = engaged.float() * (
        1.0 - torch.tanh(torch.clip(-tf_pos_obj_to_target_w[:, 2], 0.0) / tanh_std)
    )
    return horizontal_distance, shaped_progress


class _TaskMixin:
    """Drive a bolt into a threaded nut with pose and rotation tracking."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._obj: RigidObject = get_scene_rigid_object(self.scene, "bolt")
        self._target: AssetBase = get_scene_entity(self.scene, "nut")

        self._offset_pos_bolt_driver_slot = torch.tensor(
            self.cfg.bolt.frame_driver_slot.offset.pos,  # type: ignore[attr-defined]
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._tf_pos_target = self.scene.env_origins + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        tf_pos_obj = self._obj.data.root_pos_w
        tf_quat_obj = self._obj.data.root_quat_w
        vel_ang_obj = self._obj.data.root_ang_vel_w

        tf_pos_bolt_slot, tf_quat_bolt_slot = combine_frame_transforms(
            t01=tf_pos_obj,
            q01=tf_quat_obj,
            t12=self._offset_pos_bolt_driver_slot,
        )
        tf_pos_ee_to_bolt_slot, tf_quat_ee_to_bolt_slot = subtract_frame_transforms(
            t01=state.tf_pos_ee,
            q01=state.tf_quat_ee,
            t02=tf_pos_bolt_slot,
            q02=tf_quat_bolt_slot,
        )
        tf_rotmat_ee_to_bolt_slot = matrix_from_quat(tf_quat_ee_to_bolt_slot)
        tf_rot6d_ee_to_bolt_slot = rotmat_to_rot6d(tf_rotmat_ee_to_bolt_slot)
        dist_ee_to_bolt_slot = torch.norm(tf_pos_ee_to_bolt_slot, dim=-1)

        tf_pos_obj_to_target, tf_quat_obj_to_target = subtract_frame_transforms(
            t01=tf_pos_obj,
            q01=tf_quat_obj,
            t02=self._tf_pos_target,
            q02=self._tf_quat_target,
        )
        tf_rotmat_obj_to_target = matrix_from_quat(tf_quat_obj_to_target)
        tf_rot6d_obj_to_target = rotmat_to_rot6d(tf_rotmat_obj_to_target)

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

        down_vec = (
            torch.tensor((0.0, 0.0, -1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3)
        )
        # World down; the FK frame follows the (possibly tilted) base.
        ee_rotmat_w = matrix_from_quat(state.tf_quat_ee)
        top_down_alignment = torch.sum(ee_rotmat_w[:, :, 2] * down_vec, dim=1)

        up_vec = (
            torch.tensor((0.0, 0.0, 1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3)
        )
        obj_top_align = torch.sum(
            matrix_from_quat(tf_quat_obj)[:, :, 2] * up_vec, dim=1
        )
        obj_tilt_metric = torch.tanh(
            (1.0 - obj_top_align) / rew.object_orientation_tanh_std
        )

        if contact_force_matrix_ee is not None:
            contact_metric = torch.mean(
                torch.max(torch.norm(contact_force_matrix_ee, dim=-1), dim=-1)[0],
                dim=1,
            )
        else:
            contact_metric = torch.zeros(num_envs, dtype=dtype, device=device)

        screwing_metric = -vel_ang_obj[:, 2] / torch.pi

        obj_horiz_dist, obj_target_shaped = bolt_thread_progress_metrics(
            tf_pos_obj,
            self._tf_pos_target,
            obj_top_align,
            too_far_threshold=rew.penalty_distance_obj_to_target_too_far.threshold,
            tanh_std=rew.distance_obj_to_target_tanh_std,
            upright_min_alignment=rew.object_upright_min_alignment,
        )

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
            "penalty_object_top_down_orientation": rew.penalty_object_top_down_orientation(
                obj_tilt_metric
            ),
            "reward_distance_ee_to_bolt_driver_slot": rew.reward_distance_ee_to_bolt_driver_slot(
                dist_ee_to_bolt_slot
            ),
            "reward_distance_ee_to_bolt_driver_slot_close": rew.reward_distance_ee_to_bolt_driver_slot_close(
                dist_ee_to_bolt_slot
            ),
            "reward_distance_ee_to_bolt_driver_slot_precision": rew.reward_distance_ee_to_bolt_driver_slot_precision(
                dist_ee_to_bolt_slot
            ),
            "reward_contact": rew.reward_contact(contact_metric),
            "reward_screwing": rew.reward_screwing(screwing_metric),
            "penalty_distance_obj_to_target_too_far": rew.penalty_distance_obj_to_target_too_far(
                obj_horiz_dist
            ),
            "reward_distance_obj_to_target": rew.reward_distance_obj_to_target(
                obj_target_shaped
            ),
        }

        termination = torch.zeros(num_envs, dtype=torch.bool, device=device)
        truncation = (
            self.episode_length_buf >= self.max_episode_length
            if self.cfg.truncate_episodes
            else torch.zeros(num_envs, dtype=torch.bool, device=device)
        )

        return StepReturn(
            {
                ObservationGroup.STATE: {
                    "vel_ang_obj": vel_ang_obj,
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_ee,
                    "tf_pos_end_effector_to_bolt_driver_slot": tf_pos_ee_to_bolt_slot,
                    "tf_rot6d_end_effector_to_bolt_driver_slot": tf_rot6d_ee_to_bolt_slot,
                    "tf_pos_obj_to_target": tf_pos_obj_to_target,
                    "tf_rot6d_obj_to_target": tf_rot6d_obj_to_target,
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
