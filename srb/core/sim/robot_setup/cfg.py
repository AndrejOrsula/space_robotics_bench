from pydantic import BaseModel


class RobotAssemblerCfg(BaseModel):
    base_path: str
    attach_path: str
    base_mount_frame: str = ""
    attach_mount_frame: str = ""
    fixed_joint_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    fixed_joint_orient: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    mask_all_collisions: bool = False
    mask_attached_collisions: bool = True
    disable_root_joints: bool = True
