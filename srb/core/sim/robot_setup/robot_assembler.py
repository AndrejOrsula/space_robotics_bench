from collections.abc import Callable

import numpy as np
from isaaclab.sim.utils import standardize_xform_ops
from isaacsim.robot_setup.assembler import RobotAssembler as __RobotAssembler
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

# Mount frames are authored through isaaclab's prim utilities rather than
# isaacsim.core.prims.SingleXFormPrim: that class calls
# SimulationManager._get_backend_utils(), which does not exist on
# isaaclab_physx's PhysxManager replacement, so every assembly raised
# AttributeError during scene setup. The remaining query helpers are SRB's
# ports of the ``isaacsim.core.utils`` functions Isaac Sim 6.1 removed.
from srb.core.sim.prim_utils import (
    create_prim,
    find_unique_string_name,
    get_articulation_root_api_prim_path,
    get_current_stage,
    get_prim_at_path,
    is_prim_path_valid,
    move_articulation_root,
)

from .assembled_bodies import AssembledBodies
from .cfg import RobotAssemblerCfg


def prepare_assembled_scene_physics(stage: Usd.Stage) -> Callable[[], None]:
    """Parse materialized assemblies without PhysX's joint replication path.

    Call after USD cloning and joint authoring, before the first physics reset.
    Replicating connected articulations can stall inside PhysX. Ordinary stage
    parsing retains the separately authored joints and existing USD collision
    filters. Keep the clone backend's prototype namespace excluded so unused
    asset variants do not become live physics bodies.
    """
    from omni.physx import get_physx_replicator_interface

    stage_id = UsdUtils.StageCache.Get().Insert(stage).ToLongInt()
    replicator = get_physx_replicator_interface()
    replicator.unregister_replicator(stage_id)

    def exclude_prototypes(_stage_id: int) -> list[str]:
        return ["/World/template"]

    def finish_parsing(_stage_id: int) -> None:
        # GPU startup attaches, then forces a second USD load. Keep prototype
        # exclusion for both passes and any later stage reattachment.
        pass

    def unchanged_path(path: str, _index: int) -> str:
        # Required by the native API; finish_parsing never invokes replication.
        return path

    replicator.register_replicator(
        stage_id, exclude_prototypes, finish_parsing, unchanged_path
    )
    released = False

    def release() -> None:
        nonlocal released
        if not released:
            replicator.unregister_replicator(stage_id)
            released = True

    return release


class RobotAssembler(__RobotAssembler):
    def assemble_rigid_bodies(self, cfg: RobotAssemblerCfg) -> AssembledBodies:
        fixed_joint_offset = np.array(cfg.fixed_joint_offset)
        fixed_joint_orient = np.array(cfg.fixed_joint_orient)

        # Make mount_frames if they are not specified
        if cfg.base_mount_frame:
            base_mount_path = (
                f"{cfg.base_path}/{cfg.base_mount_frame.removeprefix('/')}"
            )
        else:
            base_mount_path = (
                f"{cfg.base_path}/{cfg.attach_path.split('/')[-1]}_mount_frame"
            )
            base_mount_path = find_unique_string_name(
                base_mount_path, lambda x: not is_prim_path_valid(x)
            )
            create_prim(base_mount_path, "Xform", translation=(0.0, 0.0, 0.0))

        if cfg.attach_mount_frame:
            attach_mount_path = (
                f"{cfg.attach_path}/{cfg.attach_mount_frame.removeprefix('/')}"
            )
        else:
            attach_mount_path = (
                f"{cfg.attach_path}/{cfg.base_path.split('/')[-1]}_mount_frame"
            )
            attach_mount_path = find_unique_string_name(
                attach_mount_path, lambda x: not is_prim_path_valid(x)
            )
            create_prim(attach_mount_path, "Xform", translation=(0.0, 0.0, 0.0))

        # Get the prim and articulation root of the attached asset
        attach_prim = get_prim_at_path(cfg.attach_path)
        articulation_root = get_prim_at_path(
            get_articulation_root_api_prim_path(cfg.attach_path)
        )

        # Move the Articulation root to the attach path to avoid edge cases with physics parsing.
        if articulation_root.HasAPI(UsdPhysics.ArticulationRootAPI):  # type: ignore[misc]
            move_articulation_root(articulation_root, attach_prim)

        # Find and Disable Fixed Joints that Tie Object B to the Stage
        root_joints = [p for p in Usd.PrimRange(attach_prim) if self.is_root_joint(p)]

        if cfg.disable_root_joints:
            for root_joint in root_joints:
                root_joint.GetProperty("physics:jointEnabled").Set(False)

        if attach_prim.HasAttribute("physics:kinematicEnabled"):
            attach_prim.GetAttribute("physics:kinematicEnabled").Set(False)  # type: ignore[misc]
        # Spawn at the joint pose; otherwise the first physics step snaps the
        # joint from the spawn pose and can blow up both articulations.
        self.place_at_mount(
            attach_prim,
            base_mount_path,
            attach_mount_path,
            fixed_joint_offset,
            fixed_joint_orient,
        )
        # Create fixed Joint between attach frames
        fixed_joint = self.create_fixed_joint(
            attach_mount_path,
            base_mount_path,
            attach_mount_path,
            fixed_joint_offset,
            fixed_joint_orient,
        )

        # Make sure that Articulation B is not parsed as a part of Articulation A.
        fixed_joint.GetExcludeFromArticulationAttr().Set(True)

        # Mask collisions
        if cfg.mask_all_collisions:
            collision_mask = self.mask_collisions(cfg.base_path, cfg.attach_path)
        elif cfg.mask_attached_collisions:
            collision_mask = self.mask_collisions(base_mount_path, attach_mount_path)
        else:
            collision_mask = None

        return AssembledBodies(
            cfg.base_path,
            cfg.attach_path,
            fixed_joint,
            root_joints,
            articulation_root,
            collision_mask,
        )

    @staticmethod
    def place_at_mount(
        attach_prim: Usd.Prim,
        base_mount_path: str,
        attach_mount_path: str,
        fixed_joint_offset: np.ndarray,
        fixed_joint_orient: np.ndarray,
    ):
        """Move ``attach_prim`` so its mount frame coincides with the joint frame."""
        cache = UsdGeom.XformCache()
        _o = fixed_joint_orient.astype(float)
        joint_local = Gf.Matrix4d().SetRotate(
            Gf.Quatd(float(_o[3]), float(_o[0]), float(_o[1]), float(_o[2]))
        )
        joint_local.SetTranslateOnly(Gf.Vec3d(*fixed_joint_offset.astype(float)))

        # Gf composes row vectors: child * parent. Rigid frames match the
        # reset-time placement and keep the attach root's scale unchanged.
        def world(prim: Usd.Prim) -> Gf.Matrix4d:
            return cache.GetLocalToWorldTransform(prim).GetOrthonormalized()

        joint_world = joint_local * world(get_prim_at_path(base_mount_path))
        root_in_mount = (
            world(attach_prim) * world(get_prim_at_path(attach_mount_path)).GetInverse()
        )
        local = Gf.Transform(
            root_in_mount
            * joint_world
            * cache.GetParentToWorldTransform(attach_prim).GetInverse()
        )
        quat = local.GetRotation().GetQuat()
        standardize_xform_ops(
            attach_prim,
            translation=tuple(local.GetTranslation()),
            orientation=(*quat.GetImaginary(), quat.GetReal()),
        )

    def create_fixed_joint(
        self,
        prim_path: str,
        target0: str,
        target1: str,
        fixed_joint_offset: np.ndarray,
        fixed_joint_orient: np.ndarray,
    ) -> UsdPhysics.FixedJoint:  # type: ignore[misc]
        stage = get_current_stage()

        fixed_joint_path = prim_path + "/AssemblerFixedJoint"
        fixed_joint_path = find_unique_string_name(
            fixed_joint_path, lambda x: not is_prim_path_valid(x)
        )
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)  # type: ignore[misc]
        fixed_joint_prim = fixed_joint.GetPrim()
        fixed_joint_prim.GetRelationship("physics:body0").SetTargets(
            [Sdf.Path(target0)]
        )
        fixed_joint_prim.GetRelationship("physics:body1").SetTargets(
            [Sdf.Path(target1)]
        )

        fixed_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*fixed_joint_offset.astype(float)))
        # Convert XYZW (SRB/Isaac Lab convention) to WXYZ (Gf.Quatf convention)
        _o = fixed_joint_orient.astype(float)
        fixed_joint.GetLocalRot0Attr().Set(
            Gf.Quatf(float(_o[3]), float(_o[0]), float(_o[1]), float(_o[2]))
        )
        fixed_joint.GetLocalPos1Attr().Set(Gf.Vec3f(*np.zeros(3)))
        # Identity quaternion in WXYZ for Gf.Quatf
        fixed_joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

        return fixed_joint
