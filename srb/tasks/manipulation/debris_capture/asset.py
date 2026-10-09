from typing import TYPE_CHECKING

from srb import assets
from srb.core.asset import AssetVariant, Object, RigidObjectCfg
from srb.core.sim import RigidBodyPropertiesCfg

if TYPE_CHECKING:
    from .task import TaskCfg


def select_debris(
    env_cfg: "TaskCfg",
    *,
    prim_path: str = "{ENV_REGEX_NS}/debris",
    scale: tuple[float, float, float] = (0.05, 0.05, 0.05),
    init_state: RigidObjectCfg.InitialStateCfg | None = None,
    **kwargs,
) -> RigidObjectCfg:
    if init_state is None:
        init_state = RigidObjectCfg.InitialStateCfg()

    if isinstance(env_cfg.debris, Object):
        debris_cfg = env_cfg.debris.asset_cfg.copy()  # type: ignore[attr-defined]
        assert isinstance(debris_cfg, RigidObjectCfg)
    else:
        match env_cfg.debris:
            case AssetVariant.PRIMITIVE:
                debris_cfg = assets.RandomShape(scale=scale).asset_cfg

            case AssetVariant.DATASET:
                debris_cfg = assets.ProfilePeg().asset_cfg

            case AssetVariant.PROCEDURAL:
                debris_cfg = assets.Cubesat(scale=scale).asset_cfg

            case _:
                raise ValueError(
                    "debris_capture requires a debris asset (an Object or "
                    "AssetVariant.PRIMITIVE, DATASET or PROCEDURAL), got "
                    f"{env_cfg.debris!r}"
                )

    debris_cfg.prim_path = prim_path
    debris_cfg.init_state = init_state
    spawn = debris_cfg.spawn.replace(**kwargs)  # type: ignore[union-attr]
    # Free-floating debris coasts in vacuum: no drag of any kind. Unset damping
    # is not zero -- PhysX then applies its own default (angularDamping 0.05,
    # tau = 20 s) -- so both coefficients are authored explicitly, as on every
    # other vacuum asset (B-3, measurements/m2_vacuum_coast.py).
    rigid_props = getattr(spawn, "rigid_props", None)
    spawn.rigid_props = (  # type: ignore[union-attr]
        RigidBodyPropertiesCfg(linear_damping=0.0, angular_damping=0.0)
        if rigid_props is None
        else rigid_props.replace(linear_damping=0.0, angular_damping=0.0)
    )
    debris_cfg.spawn = spawn
    return debris_cfg
