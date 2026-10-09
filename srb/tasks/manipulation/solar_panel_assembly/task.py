from dataclasses import MISSING
from typing import Sequence

import torch

from srb import assets
from srb._typing import ObservationGroup, StepReturn
from srb.core.asset import (
    AssetVariant,
    RigidObject,
    RigidObjectCfg,
    RigidObjectCollection,
    RigidObjectCollectionCfg,
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
from srb.core.marker import VisualizationMarkers, VisualizationMarkersCfg
from srb.core.mdp import (
    reset_collection_root_state_uniform_poisson_disk_2d,
    reset_root_state_uniform,
)
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.reward import (
    GaussianRewardTerm,
    GaussianTrackingRewardTerm,
    GaussianZoneTrackingRewardTerm,
    HyperbolicRewardTerm,
    LinearRewardTerm,
    SoftThresholdRewardTerm,
)
from srb.core.scene_introspection import (
    get_scene_rigid_object,
    get_scene_rigid_object_collection,
)
from srb.core.sensor import ContactSensor, ContactSensorCfg
from srb.core.sim import PreviewSurfaceCfg
from srb.tasks.manipulation.peg_in_hole.asset import select_peg_in_hole_assembly
from srb.utils.cfg import configclass
from srb.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    rotmat_to_rot6d,
    subtract_frame_transforms,
)

from .asset import select_solar_panel

# Configuration


class RewardCfg(ManipulationRewardCfg):
    """Peg metrics are shaped and summed; alignment leads to insertion.

    Panel placement is secondary; inherited penalties use ``ManipulationRewardCfg``.
    """

    # Task rewards
    reward_top_down_orientation: GaussianTrackingRewardTerm = (
        GaussianTrackingRewardTerm(weight=1.0, target=1.0, scale=0.25)
    )
    """Favor a top-down end-effector pose, with a flat reward near vertical."""

    reward_distance_ee_to_objs: LinearRewardTerm = LinearRewardTerm(
        weight=2.5, limit=25.0
    )
    """Dense, pre-shaped end-effector distance summed across pegs."""

    reward_grasp: SoftThresholdRewardTerm = SoftThresholdRewardTerm(
        weight=8.0, threshold=5.0, operator=">", sharpness=1.0
    )
    """Reward grasp contact with a soft threshold at 5 N."""

    reward_lift: LinearRewardTerm = LinearRewardTerm(weight=4.0, limit=40.0)
    """Track each peg's lift zone using a pre-shaped summed metric."""

    reward_align_pegs_to_holes_primary: LinearRewardTerm = LinearRewardTerm(
        weight=8.0, limit=80.0
    )
    """Track peg-hole alignment along the primary Z axis."""

    reward_align_pegs_to_holes_secondary: LinearRewardTerm = LinearRewardTerm(
        weight=4.0, limit=40.0
    )
    """Track symmetric XY alignment after primary-axis alignment."""

    reward_distance_pegs_to_holes_entrance_gradual: LinearRewardTerm = LinearRewardTerm(
        weight=8.0, limit=80.0
    )
    """Provide coarse guidance toward each hole entrance."""

    reward_distance_pegs_to_holes_entrance: LinearRewardTerm = LinearRewardTerm(
        weight=32.0, limit=320.0
    )
    """Guide each peg toward its hole entrance."""

    reward_distance_pegs_to_holes_bottom: LinearRewardTerm = LinearRewardTerm(
        weight=256.0, limit=2560.0
    )
    """Reward insertion by measuring peg distance to each hole bottom."""

    reward_distance_ee_to_panel: HyperbolicRewardTerm = HyperbolicRewardTerm(
        weight=10.0, scale=0.25
    )
    """Guide the end effector toward the panel for pickup."""

    reward_panel_lift: GaussianZoneTrackingRewardTerm = GaussianZoneTrackingRewardTerm(
        weight=16.0, target=0.3, tolerance=0.15, scale=0.1
    )
    """Track the panel's lift zone with a flat reward inside the target range."""

    reward_distance_panel_to_target_gradual: HyperbolicRewardTerm = (
        HyperbolicRewardTerm(weight=32.0, scale=0.1)
    )
    """Provide coarse guidance toward the panel target."""

    reward_distance_panel_to_target: GaussianRewardTerm = GaussianRewardTerm(
        weight=1024.0, scale=0.015
    )
    """Reward precise panel placement and holding at the target."""

    distance_ee_to_objs_tanh_std: float = 0.2
    lift_height_offset: float = 0.2
    lift_height_span: float = 0.1
    lift_tanh_std: float = 0.05
    alignment_primary_tanh_std: float = 0.5
    alignment_secondary_tanh_std: float = 0.2
    distance_entrance_gradual_tanh_std: float = 0.16
    distance_entrance_tanh_std: float = 0.04
    distance_bottom_tanh_std: float = 0.002


@configclass
class SceneCfg(ManipulationSceneCfg):
    peg: RigidObjectCollectionCfg = RigidObjectCollectionCfg(
        rigid_objects=MISSING,  # type: ignore[arg-type]
    )
    panel: RigidObjectCfg = MISSING  # type: ignore[assignment]


@configclass
class EventCfg(ManipulationEventCfg):
    randomize_object_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_collection_root_state_uniform_poisson_disk_2d,
        mode="reset",
        category=RandomizationCategory.TASK,
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
            "radius": 0.05,
        },
    )
    randomize_panel_state: SrbEventTermCfg = SrbEventTermCfg(
        func=reset_root_state_uniform,
        mode="reset",
        category=RandomizationCategory.TASK,
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
    """Configure four-peg insertion into a solar panel frame."""

    # Assets
    scenery: Terrain | AssetVariant | None = AssetVariant.PROCEDURAL
    # The selector pairs each peg with a matching hole, so only variants apply.
    peg: AssetVariant = AssetVariant.DATASET

    # Scene
    scene: SceneCfg = SceneCfg()

    # Events
    events: EventCfg = EventCfg()

    # Reward
    reward: RewardCfg = RewardCfg()

    # Episode timing
    episode_length_s: float = 50.0
    is_finite_horizon: bool = True

    # Panel target
    panel_tf_pos_target = (0.55, 0.0, 0.0)
    panel_tf_quat_target = (0.0, 0.0, 0.0, 1.0)
    panel_target_marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/panel_target",
        markers={
            "target": assets.SolarPanel().asset_cfg.spawn.replace(  # type: ignore[union-attr]
                visual_material=PreviewSurfaceCfg(emissive_color=(0.2, 0.2, 0.8)),
            )
        },
    )

    def __post_init__(self):
        super().__post_init__()

        self.peg_in_hole_assemblies = [
            select_peg_in_hole_assembly(
                self,  # type: ignore[misc]
                seed=self.seed + i,
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
                    ((0.55 + 0.1, 0.2, 0.04), True),
                    ((0.55 + 0.1, -0.2, 0.04), True),
                    ((0.55 - 0.1, 0.2, 0.04), False),
                    ((0.55 - 0.1, -0.2, 0.04), False),
                ]
            )
        ]
        self.scene.peg.rigid_objects = {
            f"peg{i}": cfg.peg.asset_cfg.replace(  # type: ignore[union-attr]
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(0.5, 0.0, 0.13),
                )
            )
            for i, cfg in enumerate(self.peg_in_hole_assemblies)
        }
        for i, cfg in enumerate(self.peg_in_hole_assemblies):
            setattr(self.scene, f"hole{i}", cfg.hole.asset_cfg)
        self.panel_cfg = select_solar_panel(
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.55, 0.0, 0.04)),
        )
        self.scene.panel = self.panel_cfg.asset_cfg
        self.panel_tf_pos_target = (
            self.panel_tf_pos_target[0] + self.panel_cfg.offset_pos[0],
            self.panel_tf_pos_target[1] + self.panel_cfg.offset_pos[1],
            self.panel_tf_pos_target[2] + self.panel_cfg.offset_pos[2] + 0.04,
        )

        if isinstance(self.scene.contacts_end_effector, ContactSensorCfg):
            self.scene.contacts_end_effector.filter_prim_paths_expr = [
                f"{{ENV_REGEX_NS}}/peg{i}" for i in range(4)
            ]

        self._update_procedural_assets()


# Task implementation


class _TaskMixin:
    """Insert four pegs into a panel frame using manipulator joint targets."""

    cfg: TaskCfg

    def __init__(self, cfg: TaskCfg, **kwargs):
        super().__init__(cfg, **kwargs)

        # Scene assets
        self._objs: RigidObjectCollection = get_scene_rigid_object_collection(
            self.scene, "peg"
        )
        self._panel: RigidObject = get_scene_rigid_object(self.scene, "panel")
        self._panel_target_marker: VisualizationMarkers = VisualizationMarkers(
            self.cfg.panel_target_marker_cfg
        )

        # Cache static hole transforms.
        hole_positions = []
        hole_quaternions = []
        for i in range(4):
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
        self._tf_pos_panel_target = self.scene.env_origins + torch.tensor(
            self.cfg.panel_tf_pos_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)
        self._tf_quat_panel_target = torch.tensor(
            self.cfg.panel_tf_quat_target, dtype=torch.float32, device=self.device
        ).repeat(self.num_envs, 1)

        self._panel_target_marker.visualize(
            self._tf_pos_panel_target, self._tf_quat_panel_target
        )

    def _reset_idx(self, env_ids: Sequence[int]):
        super()._reset_idx(env_ids)
        self._tf_pos_objs_initial[env_ids] = self._objs.data.object_pos_w[env_ids]
        self._tf_pos_panel_initial[env_ids] = self._panel.data.root_pos_w[env_ids]

    def extract_step_return(self) -> StepReturn:
        rew = self.cfg.reward

        state = manipulator_step_state(self)
        num_envs, device, dtype = state.num_envs, state.device, state.dtype

        tf_pos_objs = self._objs.data.object_pos_w
        tf_quat_objs = self._objs.data.object_quat_w
        tf_pos_targets = self._tf_pos_targets
        tf_quat_targets = self._tf_quat_targets
        num_objs = tf_pos_objs.size(1)

        tf_pos_ee_to_objs, tf_quat_ee_to_objs = subtract_frame_transforms(
            t01=state.tf_pos_ee.unsqueeze(1).repeat(1, num_objs, 1),
            q01=state.tf_quat_ee.unsqueeze(1).repeat(1, num_objs, 1),
            t02=tf_pos_objs,
            q02=tf_quat_objs,
        )
        tf_rotmat_ee_to_objs = matrix_from_quat(tf_quat_ee_to_objs)
        tf_rot6d_ee_to_objs = rotmat_to_rot6d(tf_rotmat_ee_to_objs)

        # Transform both peg ends into world coordinates.
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

        # Transform hole entrances and bottoms into world coordinates.
        tf_pos_holes_entrance, _ = combine_frame_transforms(
            t01=tf_pos_targets,
            q01=tf_quat_targets,
            t12=self._offset_pos_holes_entrance,
        )
        tf_pos_holes_bottom, _ = combine_frame_transforms(
            t01=tf_pos_targets,
            q01=tf_quat_targets,
            t12=self._offset_pos_holes_bottom,
        )

        # Peg-end offsets to each hole entrance drive distance and alignment.
        tf_pos_end0_to_entrance, tf_quat_pegs_to_holes = subtract_frame_transforms(
            t01=tf_pos_pegs_ends[:, :, 0],
            q01=tf_quat_objs,
            t02=tf_pos_holes_entrance,
            q02=tf_quat_targets,
        )
        tf_pos_end1_to_entrance, _ = subtract_frame_transforms(
            t01=tf_pos_pegs_ends[:, :, 1],
            q01=tf_quat_objs,
            t02=tf_pos_holes_entrance,
        )
        tf_pos_ends_to_entrance = torch.stack(
            [tf_pos_end0_to_entrance, tf_pos_end1_to_entrance], dim=2
        )
        tf_rotmat_pegs_to_holes = matrix_from_quat(tf_quat_pegs_to_holes)
        tf_rot6d_pegs_to_holes = rotmat_to_rot6d(tf_rotmat_pegs_to_holes)

        tf_pos_end0_to_bottom, _ = subtract_frame_transforms(
            t01=tf_pos_pegs_ends[:, :, 0],
            q01=tf_quat_objs,
            t02=tf_pos_holes_bottom,
        )
        tf_pos_end1_to_bottom, _ = subtract_frame_transforms(
            t01=tf_pos_pegs_ends[:, :, 1],
            q01=tf_quat_objs,
            t02=tf_pos_holes_bottom,
        )
        tf_pos_ends_to_bottom = torch.stack(
            [tf_pos_end0_to_bottom, tf_pos_end1_to_bottom], dim=2
        )

        # End-effector and panel target transforms are expressed in local frames.
        pos_ee_to_panel, quat_ee_to_panel = subtract_frame_transforms(
            t01=state.tf_pos_ee,
            q01=state.tf_quat_ee,
            t02=self._panel.data.root_pos_w,
            q02=self._panel.data.root_quat_w,
        )
        rotmat_ee_to_panel = matrix_from_quat(quat_ee_to_panel)
        rot6d_ee_to_panel = rotmat_to_rot6d(rotmat_ee_to_panel)

        pos_panel_to_target, quat_panel_to_target = subtract_frame_transforms(
            t01=self._panel.data.root_pos_w,
            q01=self._panel.data.root_quat_w,
            t02=self._tf_pos_panel_target,
            q02=self._tf_quat_panel_target,
        )
        rotmat_panel_to_target = matrix_from_quat(quat_panel_to_target)
        rot6d_panel_to_target = rotmat_to_rot6d(rotmat_panel_to_target)

        # End-effector contact sensor may be absent on some robot configurations.
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

        # Compare the end-effector Z axis with world down.
        down_vec = (
            torch.tensor((0.0, 0.0, -1.0), device=device)
            .unsqueeze(0)
            .expand(num_envs, 3)
        )
        # World down; the FK frame follows the (possibly tilted) base.
        ee_rotmat_w = matrix_from_quat(state.tf_quat_ee)
        top_down_alignment = torch.sum(ee_rotmat_w[:, :, 2] * down_vec, dim=1)

        # Sum bounded distance shaping across objects.
        ee_to_objs_shaped = (
            1.0
            - torch.tanh(
                torch.norm(tf_pos_ee_to_objs, dim=-1) / rew.distance_ee_to_objs_tanh_std
            )
        ).sum(dim=-1)

        # Contact force matrix supplies the grasp metric when available.
        if contact_force_matrix_ee is not None:
            grasp_metric = torch.mean(
                torch.max(torch.norm(contact_force_matrix_ee, dim=-1), dim=-1)[0],
                dim=1,
            )
        else:
            grasp_metric = torch.zeros(num_envs, dtype=dtype, device=device)

        # Reward each peg within its configured lift-height zone.
        lift_height_error = (
            torch.abs(
                tf_pos_objs[:, :, 2]
                - self._tf_pos_objs_initial[:, :, 2]
                - rew.lift_height_offset
            )
            - rew.lift_height_span
        ).clamp(min=0.0)
        lift_shaped = (1.0 - torch.tanh(lift_height_error / rew.lift_tanh_std)).sum(
            dim=-1
        )

        # Align peg and hole Z axes before scoring rotational symmetry.
        primary_axis_similarity = torch.abs(tf_rotmat_pegs_to_holes[:, :, 2, 2])
        align_primary_shaped = (
            1.0
            - torch.tanh(
                (1.0 - primary_axis_similarity) / rew.alignment_primary_tanh_std
            )
        ).sum(dim=-1)

        # Score XY yaw modulo each peg's rotational symmetry.
        pegs_to_holes_yaw = torch.atan2(
            tf_rotmat_pegs_to_holes[:, :, 0, 1],
            tf_rotmat_pegs_to_holes[:, :, 0, 0],
        )
        symmetry_step = 2 * torch.pi / torch.clamp_min(self._symmetry_pegs, 1)
        yaw_symmetric_dir = pegs_to_holes_yaw % symmetry_step
        yaw_symmetric_norm = torch.where(
            self._symmetry_pegs <= 0,
            0.0,
            torch.min(yaw_symmetric_dir, symmetry_step - yaw_symmetric_dir)
            / (symmetry_step / 2.0),
        )
        align_secondary_shaped = (
            1.0
            - torch.tanh(
                yaw_symmetric_norm.pow(primary_axis_similarity)
                / rew.alignment_secondary_tanh_std
            )
        ).sum(dim=-1)

        # Use the nearer peg end for entrance and bottom distance shaping.
        entrance_gradual_shaped = (
            1.0
            - torch.tanh(
                torch.min(torch.norm(tf_pos_ends_to_entrance, dim=-1), dim=2)[0]
                / rew.distance_entrance_gradual_tanh_std
            )
        ).sum(dim=-1)

        entrance_shaped = (
            1.0
            - torch.tanh(
                torch.min(torch.norm(tf_pos_ends_to_entrance, dim=-1), dim=2)[0]
                / rew.distance_entrance_tanh_std
            )
        ).sum(dim=-1)

        bottom_shaped = (
            1.0
            - torch.tanh(
                torch.min(torch.norm(tf_pos_ends_to_bottom, dim=-1), dim=2)[0]
                / rew.distance_bottom_tanh_std
            )
        ).sum(dim=-1)

        # Panel lift is measured from its reset height.
        panel_height_diff = (
            self._panel.data.root_pos_w[:, 2] - self._tf_pos_panel_initial[:, 2]
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
            "reward_distance_ee_to_objs": rew.reward_distance_ee_to_objs(
                ee_to_objs_shaped
            ),
            "reward_grasp": rew.reward_grasp(grasp_metric),
            "reward_lift": rew.reward_lift(lift_shaped),
            "reward_align_pegs_to_holes_primary": rew.reward_align_pegs_to_holes_primary(
                align_primary_shaped
            ),
            "reward_align_pegs_to_holes_secondary": rew.reward_align_pegs_to_holes_secondary(
                align_secondary_shaped
            ),
            "reward_distance_pegs_to_holes_entrance_gradual": rew.reward_distance_pegs_to_holes_entrance_gradual(
                entrance_gradual_shaped
            ),
            "reward_distance_pegs_to_holes_entrance": rew.reward_distance_pegs_to_holes_entrance(
                entrance_shaped
            ),
            "reward_distance_pegs_to_holes_bottom": rew.reward_distance_pegs_to_holes_bottom(
                bottom_shaped
            ),
            "reward_distance_ee_to_panel": rew.reward_distance_ee_to_panel(
                torch.norm(pos_ee_to_panel, dim=-1)
            ),
            "reward_panel_lift": rew.reward_panel_lift(panel_height_diff),
            "reward_distance_panel_to_target_gradual": rew.reward_distance_panel_to_target_gradual(
                torch.norm(pos_panel_to_target, dim=-1)
            ),
            "reward_distance_panel_to_target": rew.reward_distance_panel_to_target(
                torch.norm(pos_panel_to_target, dim=-1)
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
                    "contact_forces_mean_robot": contact_forces_mean_robot,
                    "contact_forces_mean_end_effector": contact_forces_mean_ee,
                    "tf_pos_end_effector_to_objs": tf_pos_ee_to_objs.flatten(1),
                    "tf_rot6d_end_effector_to_objs": tf_rot6d_ee_to_objs.flatten(1),
                    "tf_pos_pegs_end0_to_holes_entrance": tf_pos_end0_to_entrance.flatten(
                        1
                    ),
                    "tf_pos_pegs_end1_to_holes_entrance": tf_pos_end1_to_entrance.flatten(
                        1
                    ),
                    "tf_pos_pegs_end0_to_holes_bottom": tf_pos_end0_to_bottom.flatten(
                        1
                    ),
                    "tf_pos_pegs_end1_to_holes_bottom": tf_pos_end1_to_bottom.flatten(
                        1
                    ),
                    "tf_rot6d_pegs_to_holes": tf_rot6d_pegs_to_holes.flatten(1),
                    "tf_pos_end_effector_to_panel": pos_ee_to_panel,
                    "tf_rot6d_end_effector_to_panel": rot6d_ee_to_panel,
                    "tf_pos_panel_to_panel_target": pos_panel_to_target,
                    "tf_rot6d_panel_to_panel_target": rot6d_panel_to_target,
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
