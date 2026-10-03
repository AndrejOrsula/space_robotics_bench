from srb.core.asset import Object, RigidObjectCfg
from srb.core.sim import (
    CollisionPropertiesCfg,
    MassPropertiesCfg,
    MultiShapeSpawnerCfg,
    RigidBodyPropertiesCfg,
)


class Beam(Object):
    """Simple rigid beam payload for multi-robot transport tasks."""

    asset_cfg: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/beam",
        spawn=MultiShapeSpawnerCfg(
            shapes=("cuboid",),
            scale=(1.2, 0.3, 0.06),
            collision_props=CollisionPropertiesCfg(),
            rigid_props=RigidBodyPropertiesCfg(),
            mass_props=MassPropertiesCfg(density=250.0),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.3),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
    )
