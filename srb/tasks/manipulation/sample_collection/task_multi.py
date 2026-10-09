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
from srb.core.marker import VisualizationMarkers
from srb.core.mdp import reset_collection_root_state_uniform_poisson_disk_2d
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import GaussianRewardTerm, LinearRewardTerm
from srb.core.scene_introspection import get_scene_rigid_object_collection
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.utils.cfg import configclass
from srb.utils.math import (
    matrix_from_quat,
    rotmat_to_rot6d,
    scale_transform,
    subtract_frame_transforms,
)

from .asset import select_sample
from .task import EventCfg, SceneCfg, TaskCfg
from .task import RewardCfg as SampleCollectionRewardCfg


class MultiRewardCfg(SampleCollectionRewardCfg):
    """Shape and sum object rewards across samples."""

    # Replaced by multi-object terms.
    reward_distance_ee_to_obj: GaussianRewardTerm = GaussianRewardTerm(
        weight=0.0, scale=0.15
    )
    reward_distance_obj_to_target: GaussianRewardTerm = GaussianRewardTerm(
        weight=0.0, scale=0.15
    )
    penalty_obj_fallen: LinearRewardTerm = LinearRewardTerm(weight=0.0, limit=16.0)

    reward_distance_ee_to_objs: LinearRewardTerm = LinearRewardTerm(
        weight=2.5, limit=10.0
    )
    """Summed end-effector proximity reward; tanh std 0.2 m."""

    reward_grasp: LinearRewardTerm = LinearRewardTerm(weight=4.0, limit=16.0)
    """Binary grasp reward at 5 N contact force."""

    reward_lift: LinearRewardTerm = LinearRewardTerm(weight=8.0, limit=32.0)
    """Summed lift reward tracking a 0.5 m height zone."""

    reward_distance_objs_to_target: LinearRewardTerm = LinearRewardTerm(
        weight=32.0, limit=128.0
    )
    """Summed object-to-target reward; tanh std 0.2 m."""

    tanh_std_distance_ee_to_objs: float = 0.2
    tanh_std_lift: float = 0.1
    height_offset_lift: float = 0.5
    height_span_lift: float = 0.25
    tanh_std_distance_objs_to_target: float = 0.2
    threshold_grasp: float = 5.0


@configclass
class MultiSceneCfg(SceneCfg):
    sample: None = None
    samples: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore[arg-type]
    )


@configclass
class MultiEventCfg(EventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_collection_root_state_uniform_poisson_disk_2d,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("samples"),
            "pose_range": MISSING,
            "velocity_range": MISSING,
            "radius": 0.1,
        },
    )


@configclass
class MultiTaskCfg(TaskCfg):
    scene: MultiSceneCfg = MultiSceneCfg()
    num_samples: int = 3

    events: MultiEventCfg = MultiEventCfg()

    reward: MultiRewardCfg = MultiRewardCfg()

    episode_length_s: float = MISSING  # type: ignore[assignment]
    _base_episode_length_s: float = 7.5

    def __post_init__(self):
        super().__post_init__()
        if type(self.num_samples) is not int or self.num_samples < 1:
            raise ValueError(
                f"num_samples must be an integer >= 1, got {self.num_samples!r}"
            )

        self.episode_length_s = self.num_samples * self._base_episode_length_s

        self.scene.sample = None
        samples = [
            select_sample(
                self,
                seed=self.seed + (i * self.scene.num_envs),
                prim_path=f"{{ENV_REGEX_NS}}/sample{i}",
                asset_cfg=SceneEntityCfg(f"sample{i}"),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.55, 0.0, 0.0)),
                activate_contact_sensors=True,
            )
            for i in range(self.num_samples)
        ]
        self.scene.samples.rigid_objects = {
            f"sample{i}": cfg.asset_cfg for i, cfg in enumerate(samples)
        }

        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                f"{{ENV_REGEX_NS}}/sample{i}" for i in range(self.num_samples)
            ]

        self.events.randomize_object_state.params["pose_range"] = samples[
            0
        ].state_randomizer.params["pose_range"]
        self.events.randomize_object_state.params["velocity_range"] = samples[
            0
        ].state_randomizer.params["velocity_range"]

        self._update_procedural_assets()


class _TaskMixin:
    cfg: MultiTaskCfg

    def __init__(self, cfg: MultiTaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._objs: RigidObjectCollection = get_scene_rigid_object_collection(
            self.scene, "samples"
        )
        self._target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.target_marker_cfg
        )

        self._tf_pos_objs_initial = torch.zeros(
            (self.num_envs, self.cfg.num_samples, 3),
            dtype=torch.float32,
            device=self.device,
        )
        self._tf_pos_target = self.scene.env_origins.unsqueeze(1) + torch.tensor(
            self.cfg.tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, self.cfg.num_samples, 1)
        self._tf_quat_target = torch.tensor(
            self.cfg.tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, self.cfg.num_samples, 1)

        self._target_marker.visualize(
            self._tf_pos_target[:, 0, :], self._tf_quat_target[:, 0, :]
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_objs_initial[env_ids] = self._objs.data.object_com_pos_w[env_ids]

    def extract_step_return(self) -> StepReturn:
        num_envs = self.num_envs
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
                scale_transform(
                    joint_pos_ee,
                    ee_limits[:, :, 0],
                    ee_limits[:, :, 1],
                )
                if torch.all(torch.isfinite(ee_limits))
                else joint_pos_ee
            )
        else:
            joint_pos_end_effector_normalized = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )

        num_objs = self._objs.data.object_com_pos_w.size(1)

        fk_pos_end_effector = self._tf_end_effector.data.target_pos_source[:, 0, :]
        fk_quat_end_effector = self._tf_end_effector.data.target_quat_source[:, 0, :]
        fk_rotmat_end_effector = matrix_from_quat(fk_quat_end_effector)
        fk_rot6d_end_effector = rotmat_to_rot6d(fk_rotmat_end_effector)

        tf_pos_end_effector = self._tf_end_effector.data.target_pos_w[:, 0, :]
        tf_quat_end_effector = self._tf_end_effector.data.target_quat_w[:, 0, :]
        tf_pos_objs = self._objs.data.object_com_pos_w
        tf_quat_objs = self._objs.data.object_com_quat_w

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
        tf_pos_objs_to_target, tf_quat_objs_to_target = subtract_frame_transforms(
            t01=tf_pos_objs,
            q01=tf_quat_objs,
            t02=self._tf_pos_target,
            q02=self._tf_quat_target,
        )
        tf_rotmat_objs_to_target = matrix_from_quat(tf_quat_objs_to_target)
        tf_rot6d_objs_to_target = rotmat_to_rot6d(tf_rotmat_objs_to_target)

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

        action_rate_metric = torch.mean(torch.square(act_current - act_previous), dim=1)
        joint_torque_metric = torch.sum(torch.square(joint_applied_torque_robot), dim=1)
        joint_acc_metric = torch.sum(torch.square(joint_acc_robot), dim=1)
        undesired_contacts_metric = torch.max(
            torch.norm(contact_forces_robot, dim=-1), dim=1
        )[0]

        # World down, like the single-object task; the FK frame follows the base.
        top_down_alignment = torch.sum(
            matrix_from_quat(tf_quat_end_effector)[:, :, 2]
            * torch.tensor((0.0, 0.0, -1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3),
            dim=1,
        )

        _dist_shaped = (
            1.0
            - torch.tanh(
                torch.norm(tf_pos_end_effector_to_objs, dim=-1)
                / rew.tanh_std_distance_ee_to_objs
            )
        ).sum(dim=1)

        if contact_force_matrix_end_effector is not None:
            grasp_metric = (
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
            grasp_metric = torch.zeros(num_envs, dtype=dtype, device=device)

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
        ).sum(dim=1)

        _target_dist_shaped = (
            1.0
            - torch.tanh(
                torch.norm(tf_pos_objs_to_target, dim=-1)
                / rew.tanh_std_distance_objs_to_target
            )
        ).sum(dim=1)

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
            "reward_distance_ee_to_objs": rew.reward_distance_ee_to_objs(_dist_shaped),
            "reward_grasp": rew.reward_grasp(grasp_metric),
            "reward_lift": rew.reward_lift(_lift_shaped),
            "reward_distance_objs_to_target": rew.reward_distance_objs_to_target(
                _target_dist_shaped
            ),
        }

        # Keep disabled single-object terms in logs for compatibility.
        _zero = torch.zeros(num_envs, dtype=dtype, device=device)
        reward["reward_distance_ee_to_obj"] = rew.reward_distance_ee_to_obj(_zero)
        reward["reward_distance_obj_to_target"] = rew.reward_distance_obj_to_target(
            _zero
        )
        reward["penalty_obj_fallen"] = rew.penalty_obj_fallen(_zero)

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
                    "tf_pos_end_effector_to_objs": tf_pos_end_effector_to_objs.flatten(
                        1
                    ),
                    "tf_rot6d_end_effector_to_objs": tf_rot6d_end_effector_to_objs.flatten(
                        1
                    ),
                    "tf_pos_objs_to_target": tf_pos_objs_to_target.flatten(1),
                    "tf_rot6d_objs_to_target": tf_rot6d_objs_to_target.flatten(1),
                },
                ObservationGroup.STATE_DYN: {
                    "contact_forces_robot": contact_forces_robot.flatten(1),
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
