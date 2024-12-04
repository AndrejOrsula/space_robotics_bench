from omni.isaac.lab_assets import ANYMAL_B_CFG, ANYMAL_C_CFG, ANYMAL_D_CFG

import space_robotics_bench.core.assets as asset_utils
from space_robotics_bench.core.actions import (
    JointPositionActionCfg,
    LocomotionJointSpaceActionCfg,
)


def anymal_b_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    action_scale: float = 0.5,
    use_default_offset: bool = True,
    **kwargs,
) -> asset_utils.LeggedRobotCfg:
    return asset_utils.LeggedRobotCfg(
        ## Model
        asset_cfg=ANYMAL_B_CFG.replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=LocomotionJointSpaceActionCfg(
            joint_pos=JointPositionActionCfg(
                asset_name=asset_name,
                joint_names=[".*"],
                scale=action_scale,
                use_default_offset=use_default_offset,
            )
        ),
        frame_base=asset_utils.FrameCfg(
            prim_relpath="base",
        ),
    )


def anymal_c_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    action_scale: float = 0.5,
    use_default_offset: bool = True,
    **kwargs,
) -> asset_utils.LeggedRobotCfg:
    return asset_utils.LeggedRobotCfg(
        ## Model
        asset_cfg=ANYMAL_C_CFG.replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=LocomotionJointSpaceActionCfg(
            joint_pos=JointPositionActionCfg(
                asset_name=asset_name,
                joint_names=[".*"],
                scale=action_scale,
                use_default_offset=use_default_offset,
            )
        ),
        frame_base=asset_utils.FrameCfg(
            prim_relpath="base",
        ),
    )


def anymal_d_cfg(
    *,
    prim_path: str = "{ENV_REGEX_NS}/robot",
    asset_name: str = "robot",
    action_scale: float = 0.5,
    use_default_offset: bool = True,
    **kwargs,
) -> asset_utils.LeggedRobotCfg:
    return asset_utils.LeggedRobotCfg(
        ## Model
        asset_cfg=ANYMAL_D_CFG.replace(prim_path=prim_path, **kwargs),
        ## Actions
        action_cfg=LocomotionJointSpaceActionCfg(
            joint_pos=JointPositionActionCfg(
                asset_name=asset_name,
                joint_names=[".*"],
                scale=action_scale,
                use_default_offset=use_default_offset,
            )
        ),
        frame_base=asset_utils.FrameCfg(
            prim_relpath="base",
        ),
    )
