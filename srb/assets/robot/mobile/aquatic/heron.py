from typing import ClassVar

from srb.core.action import (
    ActionGroup,
    ThrustActionCfg,
    ThrustActionGroup,
    ThrusterCfg,
)
from srb.core.asset import Frame, RigidObjectCfg, SurfaceVehicle, Transform
from srb.core.sensor import (
    CameraCfg,
    ImuCfg,
    PinholeCameraCfg,
    SensorBaseCfg,
)
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MeshCollisionPropertiesCfg,
    RigidBodyPropertiesCfg,
    UsdFileCfg,
)
from srb.utils.math import rpy_to_quat
from srb.utils.path import SRB_ASSETS_DIR_SRB_ROBOT


class Heron(SurfaceVehicle):
    ## Model
    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/heron",
        spawn=UsdFileCfg(
            usd_path=SRB_ASSETS_DIR_SRB_ROBOT.joinpath("boat")
            .joinpath("heron.usdc")
            .as_posix(),
            activate_contact_sensors=True,
            collision_props=CollisionPropertiesCfg(),
            mesh_collision_props=MeshCollisionPropertiesCfg(
                mesh_approximation="convexDecomposition"
            ),
            rigid_props=RigidBodyPropertiesCfg(
                max_depenetration_velocity=5.0,
                angular_damping=2.0,
                linear_damping=0.5,
            ),
            mass_props=MassPropertiesCfg(density=500.0),
            semantic_tags=[("class", "boat")],
        ),
    )

    ## Actions
    ## Two stern thrusters provide forward/yaw control. `direction` is exhaust
    ## direction and thrust is `-direction * thrust`, clamped to [0, 1], so
    ## exhaust points aft (-x); the thrusters cannot reverse. Offsets in the
    ## `base_link` frame: (-0.5300, +-0.3777, -0.1600) m.
    action_modes: ClassVar[dict[str, ActionGroup]] = {
        "thrust": ThrustActionGroup(
            ThrustActionCfg(
                asset_name="robot",
                thrusters=(
                    ThrusterCfg(
                        offset=(-0.53, 0.3777, -0.16),
                        direction=(-1.0, 0.0, 0.0),
                        power=50.0,
                    ),
                    ThrusterCfg(
                        offset=(-0.53, -0.3777, -0.16),
                        direction=(-1.0, 0.0, 0.0),
                        power=50.0,
                    ),
                ),
                fuel_capacity=5.0,
                fuel_consumption_rate=0.0,
            )
        ),
    }

    ## Frames
    ## The body prim is `base_link` (`/heron/base_link`), not `base`; sensors
    ## and frames must target the existing prim.
    frame_base: Frame = Frame(prim_relpath="base_link")
    frame_payload_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(pos=(0.0, 0.0, 0.1)),
    )
    frame_manipulator_mount: Frame = Frame(
        prim_relpath="base_link",
        offset=Transform(pos=(0.0, 0.0, 0.1)),
    )
    sensor_cfgs: dict[str, SensorBaseCfg] = {
        "imu_robot": ImuCfg(
            prim_path="{ENV_REGEX_NS}/heron/base_link",
        ),
        "cam_onboard": CameraCfg(
            prim_path="{ENV_REGEX_NS}/heron/base_link/camera_onboard",
            offset=CameraCfg.OffsetCfg(
                pos=(0.25, 0.0, 0.15),
                rot=rpy_to_quat(0.0, 0.0, 0.0),
            ),
            spawn=PinholeCameraCfg(),
        ),
    }
