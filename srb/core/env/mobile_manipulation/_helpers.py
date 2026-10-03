from __future__ import annotations

from typing import Any, Callable

from srb.core._physx_fixups import wrap_spawn_to_fix_end_effector_mass
from srb.core.action import (
    InverseKinematicsActionGroup,
    OperationalSpaceControlActionGroup,
)
from srb.core.action.term.manipulation._frame_jacobian import (
    end_effector_frame_offset,
)
from srb.core.asset import ArticulationCfg, RigidObjectCfg
from srb.core.asset.robot.manipulation import Manipulator
from srb.core.asset.robot.mobile_manipulation.combined import CombinedMobileManipulator
from srb.core.asset.robot.robot import _first_joint_mode
from srb.core.manager import SceneEntityCfg
from srb.core.sensor import ContactSensorCfg
from srb.utils.math import combine_frame_transforms_tuple


def downgrade_free_floating_manipulator_jacobian_actions(robot: object) -> None:
    """Downgrade unsupported Jacobian manipulator modes on mobile manipulators."""
    if not isinstance(robot, CombinedMobileManipulator) or not isinstance(
        robot.manipulator, Manipulator
    ):
        return

    if not isinstance(
        robot.manipulator.action_group,
        (InverseKinematicsActionGroup, OperationalSpaceControlActionGroup),
    ):
        return

    fallback = _first_joint_mode(type(robot.manipulator).action_modes)
    if fallback is None:
        raise ValueError(
            f"{type(robot.manipulator).__name__} has no joint-space fallback "
            f"mode declared; a free-floating base cannot use Jacobian control. "
            f"Declare at least one 'joint_*' entry in action_modes."
        )
    robot.manipulator.action_mode = fallback


def configure_mobile_manipulator_scene(
    *,
    scene: Any,
    events: Any,
    robot: CombinedMobileManipulator,
) -> None:
    """Redirect manipulator sensors and joint reset events to the child asset."""
    assert scene.manipulator is not None

    # Same PhysX articulation-validation fixup ManipulationEnvCfg applies to
    # `scene.robot` in the fixed-base case: massless wrist frames (e.g.
    # Kinova's `end_effector_link`) need a positive mass/inertia before PhysX
    # validates the articulation, or spawning is rejected/unstable.
    wrap_spawn_to_fix_end_effector_mass(scene.manipulator)

    manipulator = robot.manipulator

    tf_end_effector = getattr(scene, "tf_end_effector", None)
    if tf_end_effector is not None:
        tf_end_effector.prim_path = (
            f"{scene.manipulator.prim_path}/{manipulator.frame_base.prim_relpath}"
        )
        tf_end_effector.target_frames[
            0
        ].prim_path = (
            f"{scene.manipulator.prim_path}/{manipulator.frame_flange.prim_relpath}"
        )
        # Tool centre point (flange ∘ TCP, or the bare flange): the
        # composition the task-space action terms bind to.
        ee_frame = end_effector_frame_offset(
            manipulator, combine=combine_frame_transforms_tuple
        )
        assert ee_frame is not None  # frame_flange is read just above
        (
            tf_end_effector.target_frames[0].offset.pos,
            tf_end_effector.target_frames[0].offset.rot,
        ) = ee_frame

    contacts_robot = getattr(scene, "contacts_robot", None)
    if contacts_robot is not None:
        contacts_robot.prim_path = f"{scene.manipulator.prim_path}/.*"

    ## Only attach an end-effector contact sensor when the end-effector is
    ## spawned as a contact-capable rigid/articulated body. The rigid
    ## end-effector spawn path in ``_add_robot`` converts a non-assembled tool
    ## (e.g. a single-body Scoop) to a base asset via ``as_asset_base_cfg``
    ## (rigid props removed), which has no contact-reporter body; a
    ## ContactSensor over it fails at init ("could not find any bodies with
    ## contact reporter API"). Mirror the fixed-base manipulation env, which
    ## drops the sensor for non-rigid end-effectors.
    scene.contacts_end_effector = (
        ContactSensorCfg(
            prim_path=f"{manipulator.end_effector.asset_cfg.prim_path}/.*",
        )
        if manipulator.end_effector is not None
        and isinstance(
            manipulator.end_effector.asset_cfg, (RigidObjectCfg, ArticulationCfg)
        )
        else None
    )

    events.randomize_robot_joints.params["asset_cfg"] = SceneEntityCfg("manipulator")


def finalize_mobile_manipulation_cfg(
    cfg: Any, run_super_post_init: Callable[[], None]
) -> None:
    """Run the 3-step mobile-manipulator env cfg ceremony.

    Ordering is load-bearing: ``downgrade`` mutates ``cfg.robot`` before the
    parent post-init binds ``cfg._robot``; ``configure_scene`` then reads it.
    Callers pass ``super().__post_init__`` so Python's MRO picks the parent.
    """
    downgrade_free_floating_manipulator_jacobian_actions(cfg.robot)
    run_super_post_init()
    configure_mobile_manipulator_scene(
        scene=cfg.scene,
        events=cfg.events,
        robot=cfg._robot,
    )
