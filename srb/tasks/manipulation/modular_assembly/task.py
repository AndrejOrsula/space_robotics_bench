import math
from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    ModularElement,
    RigidObjectCfg,
    RigidObjectCollection,
    RigidObjectCollectionCfg,
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
from srb.core.env.runtime import episode_outcome_info
from srb.core.env.srb_env import SrbEnv
from srb.core.manager import SceneEntityCfg
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import reset_collection_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianTrackingRewardTerm,
    LinearRewardTerm,
    ThresholdRewardTerm,
)
from srb.core.scene_introspection import get_scene_rigid_object_collection
from srb.core.sensor import ContactSensor, ContactSensorCfg  # noqa: F401
from srb.core.sim import PreviewSurfaceCfg
from srb.utils.cfg import configclass
from srb.utils.failures import InvalidInputError
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    radians,
    rotmat_to_rot6d,
    rpy_to_quat,
    subtract_frame_transforms,
)


class RewardCfg(ManipulationRewardCfg):
    """Assembly rewards: pair alignment, peg insertion, then full-success bonus."""

    penalty_joint_torque: LinearRewardTerm = LinearRewardTerm(weight=0.0, limit=4.0)
    """Disabled; assembly requires sustained torque."""

    penalty_undesired_robot_contacts: ThresholdRewardTerm = ThresholdRewardTerm(
        weight=-32.0, threshold=10.0, operator=">"
    )
    """Four times the default penalty; ignores contact below 10 N."""

    reward_top_down_orientation: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=16.0, target=1.0, scale=0.25)
    )
    """Top-down end-effector orientation; full reward within about 14 degrees."""

    reward_pair_position: LinearRewardTerm = LinearRewardTerm(weight=8.0, limit=80.0)
    """Per-pair position alignment; tanh standard deviation is 0.1 m."""

    reward_pair_orientation: LinearRewardTerm = LinearRewardTerm(weight=2.0, limit=20.0)
    """Per-pair rotation alignment; 0.00025 standard deviation makes it near-binary."""

    reward_peg_hole_entrance_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=8.0, limit=80.0
    )
    """Coarse per-pair guidance toward peg-hole entrances (0.05 m standard deviation)."""

    reward_peg_hole_alignment: LinearRewardTerm = LinearRewardTerm(
        weight=64.0, limit=640.0
    )
    """Precision per-pair insertion reward (0.005 m standard deviation)."""

    reward_success: LinearRewardTerm = LinearRewardTerm(weight=512.0, limit=512.0)
    """Paid every step while every pair meets the position and rotation thresholds."""

    pair_position_tanh_std: float = 0.1
    pair_orientation_tanh_std: float = 0.00025
    peg_hole_entrance_tanh_std: float = 0.05
    peg_hole_alignment_tanh_std: float = 0.005
    success_pos_threshold: float = 0.01
    success_rot_threshold: float = 0.99


@configclass
class SceneCfg(ManipulationSceneCfg):
    env_spacing: float = 10.0
    elements: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore[arg-type]
    )


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_collection_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
        params={
            "asset_cfg": SceneEntityCfg("elements"),
            # TaskCfg narrows the spread-axis range to keep neighbours apart.
            "pose_range": {
                "y": (-0.015, 0.015),
                "x": (-0.015, 0.015),
                "yaw": (-radians(5.0), radians(5.0)),
            },
            "velocity_range": {},
        },
    )
    pass


# Half extents (m) of the widest shipped element, LegoBlock_2x4.
ELEMENT_FOOTPRINT_HALF_EXTENTS_M: tuple[float, float] = (0.0079, 0.0159)
# Gap (m) between neighbours at worst-case reset offsets.
ELEMENT_RESET_CLEARANCE_MARGIN_M: float = 0.0005


def element_footprint_along_axis(
    axis: int,
    yaw: float,
    yaw_range: tuple[float, float],
    half_extents: tuple[float, float] = ELEMENT_FOOTPRINT_HALF_EXTENTS_M,
) -> float:
    """Maximum world-axis extent over the yaw range."""
    hx, hy = half_extents
    lo, hi = yaw + yaw_range[0], yaw + yaw_range[1]
    samples = 180
    widest = 0.0
    for k in range(samples + 1):
        psi = lo + (hi - lo) * k / samples
        c, s = abs(math.cos(psi)), abs(math.sin(psi))
        half = c * hx + s * hy if axis == 0 else s * hx + c * hy
        widest = max(widest, 2.0 * half)
    return widest


def element_reset_offset_limit(
    spread_spacing: float,
    footprint: float,
    *,
    margin: float = ELEMENT_RESET_CLEARANCE_MARGIN_M,
) -> float:
    """Return the largest reset offset preserving neighbour clearance."""
    clearance = spread_spacing - footprint
    if clearance <= 0.0:
        raise ValueError(
            f"modular_assembly: element_init_spacing leaves {spread_spacing:.4f} m "
            f"between element centres, but elements are up to {footprint:.4f} m "
            "wide along that axis, so neighbours intersect at reset; increase "
            "|element_init_spacing|"
        )
    return max(0.0, clearance / 2.0 - margin)


@configclass
class TaskCfg(ManipulationEnvCfg):
    """Configure modular element assembly."""

    gravity: Domain | str | None = Domain.EARTH

    element: ModularElement = assets.LegoBlock_2x4()
    element_count: int = 3
    element_init_pos: tuple[float, float, float] = (0.9, 0.0, 0.25)
    element_init_quat: tuple[float, float, float, float] = rpy_to_quat(0.0, 0.0, -90.0)
    element_init_spacing: float = 0.05

    scene: SceneCfg = SceneCfg()

    events: EventCfg = EventCfg()

    reward: RewardCfg = RewardCfg()

    episode_length_s: float = 20.0
    is_finite_horizon: bool = True

    next_element_marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/next_element",
        markers={},
    )

    def __post_init__(self):
        if self.element_count <= 1:
            raise InvalidInputError(
                f"element_count must be greater than 1, got {self.element_count}"
            )
        super().__post_init__()

        self.next_element_sign = math.copysign(1.0, self.element_init_spacing)

        assembly_spacing = self.element.assembly_spacing
        assembly_axis = max(range(3), key=lambda i: abs(assembly_spacing[i]))
        spread_axis = 1 if assembly_axis != 1 else 0
        spread_spacing = abs(assembly_spacing[spread_axis]) + abs(
            self.element_init_spacing
        )

        # Bound reset offsets by element footprint along the spread axis.
        randomizer = self.events.randomize_object_state
        pose_range = dict(randomizer.params["pose_range"])
        qx, qy, qz, qw = self.element_init_quat
        init_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )
        spread_key = "xyz"[spread_axis]
        offset_limit = element_reset_offset_limit(
            spread_spacing,
            element_footprint_along_axis(
                spread_axis, init_yaw, pose_range.get("yaw", (0.0, 0.0))
            ),
        )
        spread_lo, spread_hi = pose_range.get(spread_key, (0.0, 0.0))
        pose_range[spread_key] = (
            max(spread_lo, -offset_limit),
            min(spread_hi, offset_limit),
        )
        randomizer.params = {**randomizer.params, "pose_range": pose_range}

        self.scene.elements.rigid_objects = {
            f"element{i}": self.element.asset_cfg.replace(  # type: ignore[union-attr]
                prim_path=f"{{ENV_REGEX_NS}}/element{i}",
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=tuple(
                        self.element_init_pos[axis]
                        + self.next_element_sign
                        * spread_spacing
                        * (i - (self.element_count - 1) / 2.0)
                        if axis == spread_axis
                        else self.element_init_pos[axis]
                        for axis in range(3)
                    ),
                    rot=self.element_init_quat,
                ),
            )
            for i in range(self.element_count)
        }

        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                f"{{ENV_REGEX_NS}}/element{i}" for i in range(self.element_count)
            ]

        self.next_element_marker_cfg.markers = {
            "next_element": self.element.asset_cfg.spawn.replace(  # type: ignore[union-attr]
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        }


class _TaskMixin:
    """Pick and place modular elements using peg-hole alignment rewards."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        self._elements: RigidObjectCollection = get_scene_rigid_object_collection(
            self.scene, "elements"
        )
        self._next_element_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.next_element_marker_cfg
        )

        self._offset_pos_element_pegs = torch.tensor(
            self.cfg.element.pegs,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._offset_pos_element_holes = torch.tensor(
            self.cfg.element.holes,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._offset_pos_holes_entrance = torch.tensor(
            self.cfg.element.holes_entrance,
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1, 1)
        self._offset_pos_ideal_assembly_rel = torch.tensor(
            (
                self.cfg.next_element_sign * self.cfg.element.assembly_spacing[0],
                self.cfg.next_element_sign * self.cfg.element.assembly_spacing[1],
                self.cfg.next_element_sign * self.cfg.element.assembly_spacing[2],
            ),
            dtype=torch.float32,
            device=self.device,
        )

        self._next_element_marker.set_visibility(self.cfg.debug_vis)

        # DirectEnv finalization extracts immediately; prime while USD handles exist.
        self._visualize_next_element()

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._visualize_next_element()

    def advance_step_state(self) -> None:
        self._visualize_next_element()

    def _visualize_next_element(
        self,
        pos_threshold: float = 0.01,
        alignment_threshold: float = 0.95,
    ):
        if not self._next_element_marker.is_visible():
            return

        _elements_pose = self._elements.data.object_link_pose_w
        elements_pos = _elements_pose[:, :, :3]
        elements_quat = _elements_pose[:, :, 3:7]
        pos_i = elements_pos[:, :-1, :]
        quat_i = elements_quat[:, :-1, :]
        pos_next = elements_pos[:, 1:, :]
        quat_next = elements_quat[:, 1:, :]
        tf_pos_rel, tf_quat_rel = subtract_frame_transforms(
            t01=pos_i,
            q01=quat_i,
            t02=pos_next,
            q02=quat_next,
        )
        pos_diff = tf_pos_rel - self._offset_pos_ideal_assembly_rel
        pos_condition_met = torch.norm(pos_diff, dim=-1) < pos_threshold
        rot_matrices = matrix_from_quat(tf_quat_rel)
        assembly_axis = max(
            range(3), key=lambda i: abs(self.cfg.element.assembly_spacing[i])
        )
        quat_condition_met = (
            torch.abs(rot_matrices[..., assembly_axis, assembly_axis])
            > alignment_threshold
        )
        continue_condition = pos_condition_met & quat_condition_met
        break_condition = ~continue_condition
        vis_pos_all = torch.zeros((elements_pos.shape[0], 3), device=self.device)
        vis_pos_all[:, 0] = -1000.0
        vis_quat_all = torch.zeros((elements_pos.shape[0], 4), device=self.device)
        vis_quat_all[:, 3] = 1.0  # xyzw identity (markers are xyzw in this fork)
        envs_that_break = torch.any(break_condition, dim=1)

        if torch.any(envs_that_break):
            envs_to_visualize_indices = torch.where(envs_that_break)[0]
            first_break_indices = torch.argmax(
                break_condition[envs_that_break].int(), dim=1
            )
            pos_at_break = elements_pos[envs_to_visualize_indices, first_break_indices]
            quat_at_break = elements_quat[
                envs_to_visualize_indices, first_break_indices
            ]
            expanded_offset_t12 = self._offset_pos_ideal_assembly_rel.expand(
                pos_at_break.shape[0], -1
            )
            vis_pos_break, vis_quat_break = combine_frame_transforms(
                t01=pos_at_break,
                q01=quat_at_break,
                t12=expanded_offset_t12,
            )
            vis_pos_all[envs_to_visualize_indices] = vis_pos_break
            vis_quat_all[envs_to_visualize_indices] = vis_quat_break

        self._next_element_marker.visualize(vis_pos_all, vis_quat_all)

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        ################
        ## Raw states ##
        ################
        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        tf_pos_objs = self._elements.data.object_pos_w
        tf_quat_objs = self._elements.data.object_quat_w
        num_objs = tf_pos_objs.size(1)
        num_pairs = num_objs - 1
        num_pegs = self._offset_pos_element_pegs.size(1)

        tf_pos_ee_to_objs, tf_quat_ee_to_objs = subtract_frame_transforms(
            t01=state.tf_pos_ee.unsqueeze(1).repeat(1, num_objs, 1),
            q01=state.tf_quat_ee.unsqueeze(1).repeat(1, num_objs, 1),
            t02=tf_pos_objs,
            q02=tf_quat_objs,
        )
        tf_rotmat_ee_to_objs = matrix_from_quat(tf_quat_ee_to_objs)
        tf_rot6d_ee_to_objs = rotmat_to_rot6d(tf_rotmat_ee_to_objs)

        pos_i = tf_pos_objs[:, :-1, :]
        quat_i = tf_quat_objs[:, :-1, :]
        pos_next = tf_pos_objs[:, 1:, :]
        quat_next = tf_quat_objs[:, 1:, :]
        tf_pos_rel_pairs, tf_quat_rel_pairs = subtract_frame_transforms(
            t01=pos_i, q01=quat_i, t02=pos_next, q02=quat_next
        )
        tf_rotmat_rel_pairs = matrix_from_quat(tf_quat_rel_pairs)
        tf_rot6d_rel_pairs = rotmat_to_rot6d(tf_rotmat_rel_pairs)

        # Element i's peg mates with element i+1's hole in the assembled pose.
        tf_pos_world_holes, _ = combine_frame_transforms(
            t01=pos_next.unsqueeze(2).repeat(1, 1, num_pegs, 1),
            q01=quat_next.unsqueeze(2).repeat(1, 1, num_pegs, 1),
            t12=self._offset_pos_element_holes.unsqueeze(1).repeat(1, num_pairs, 1, 1),
        )
        tf_pos_world_holes_entrance, _ = combine_frame_transforms(
            t01=pos_next.unsqueeze(2).repeat(1, 1, num_pegs, 1),
            q01=quat_next.unsqueeze(2).repeat(1, 1, num_pegs, 1),
            t12=self._offset_pos_holes_entrance.unsqueeze(1).repeat(1, num_pairs, 1, 1),
        )
        tf_pos_world_pegs, _ = combine_frame_transforms(
            t01=pos_i.unsqueeze(2).repeat(1, 1, num_pegs, 1),
            q01=quat_i.unsqueeze(2).repeat(1, 1, num_pegs, 1),
            t12=self._offset_pos_element_pegs.unsqueeze(1).repeat(1, num_pairs, 1, 1),
        )
        peg_hole_dists = torch.norm(tf_pos_world_pegs - tf_pos_world_holes, dim=-1)
        peg_hole_entrance_dists = torch.norm(
            tf_pos_world_pegs - tf_pos_world_holes_entrance, dim=-1
        )

        contact_forces_robot = self._contacts_robot.data.net_forces_w  # type: ignore[union-attr]
        contact_forces_mean_robot = contact_forces_robot.mean(dim=1)

        has_ee_contacts = isinstance(self._contacts_end_effector, ContactSensor)
        if has_ee_contacts:
            contact_forces_ee = self._contacts_end_effector.data.net_forces_w
            contact_forces_mean_ee = contact_forces_ee.mean(dim=1)
        else:
            contact_forces_ee = torch.empty((num_envs, 0), dtype=dtype, device=device)
            contact_forces_mean_ee = torch.empty(
                (num_envs, 0), dtype=dtype, device=device
            )

        action_rate_metric = torch.mean(
            torch.square(state.act_current - state.act_previous), dim=1
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

        ideal = self._offset_pos_ideal_assembly_rel
        pair_pos_error = torch.norm(tf_pos_rel_pairs - ideal, dim=-1)
        pair_pos_shaped = (
            1.0 - torch.tanh(pair_pos_error / rew.pair_position_tanh_std)
        ).sum(dim=1)

        rotmat_similarity = (
            torch.einsum("...ii->...", tf_rotmat_rel_pairs) + 1.0
        ) / 4.0
        pair_orient_shaped = (
            1.0 - torch.tanh((1.0 - rotmat_similarity) / rew.pair_orientation_tanh_std)
        ).sum(dim=-1)

        peg_hole_entrance_shaped = (
            1.0
            - torch.tanh(
                torch.mean(peg_hole_entrance_dists, dim=-1)
                / rew.peg_hole_entrance_tanh_std
            )
        ).sum(dim=1)

        peg_hole_shaped = (
            1.0
            - torch.tanh(
                torch.mean(peg_hole_dists, dim=-1) / rew.peg_hole_alignment_tanh_std
            )
        ).sum(dim=1)

        is_pos_success = pair_pos_error < rew.success_pos_threshold
        is_rot_success = rotmat_similarity > rew.success_rot_threshold
        is_fully_assembled = torch.all(is_pos_success & is_rot_success, dim=1)

        reward: dict[str, torch.Tensor] = {
            "penalty_action_rate": rew.penalty_action_rate(action_rate_metric),
            "penalty_joint_torque": rew.penalty_joint_torque(
                torch.sum(torch.square(state.joint_applied_torque_robot), dim=1)
            ),
            "penalty_joint_acceleration": rew.penalty_joint_acceleration(
                joint_acc_metric
            ),
            "penalty_undesired_robot_contacts": rew.penalty_undesired_robot_contacts(
                undesired_contacts_metric
            ),
            "reward_top_down_orientation": rew.reward_top_down_orientation(
                top_down_alignment
            ),
            "reward_pair_position": rew.reward_pair_position(pair_pos_shaped),
            "reward_pair_orientation": rew.reward_pair_orientation(pair_orient_shaped),
            "reward_peg_hole_entrance_alignment": rew.reward_peg_hole_entrance_alignment(
                peg_hole_entrance_shaped
            ),
            "reward_peg_hole_alignment": rew.reward_peg_hole_alignment(peg_hole_shaped),
            "reward_success": rew.reward_success(is_fully_assembled.float()),
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
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_ee,
                    "tf_pos_end_effector_to_objs": tf_pos_ee_to_objs.flatten(1),
                    "tf_rot6d_end_effector_to_objs": tf_rot6d_ee_to_objs.flatten(1),
                    "tf_pos_rel_pairs": tf_pos_rel_pairs.flatten(1),
                    "tf_rot6d_rel_pairs": tf_rot6d_rel_pairs.flatten(1),
                    "peg_hole_dists": peg_hole_dists.flatten(1),
                },
                **manipulator_observation_tail(
                    state,
                    contact_forces_robot.flatten(1),
                    contact_forces_ee.flatten(1),
                ),
            },
            reward,
            termination,
            truncation,
            info=episode_outcome_info(success=is_fully_assembled),
        )


class Task(_TaskMixin, SrbEnv, ManipulationEnv):
    """Pick-and-place modular assembly on the owned SRB lifecycle."""

    cfg: TaskCfg
    _srb_direct_runtime = True

    def __init__(self, cfg: TaskCfg, **kwargs):
        runtime_mode = kwargs.pop("runtime_mode", "isaac")
        super().__init__(cfg, runtime_mode=runtime_mode, **kwargs)
