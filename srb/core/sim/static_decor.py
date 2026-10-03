"""Disable joints on assets spawned as static decor.

``Asset.as_asset_base_cfg`` disables articulation and makes bodies kinematic,
but the source USD still declares joints. PhysX reports an error for each
static-to-static joint, so disable those joints before physics parses them.
"""

from __future__ import annotations

from typing import Any, Iterator

__all__ = [
    "disable_joints_on_static_decor",
    "spawn_is_static_decor",
]


def spawn_is_static_decor(spawn: Any) -> bool:
    """Whether *spawn* has the disabled-articulation shape produced by
    ``as_asset_base_cfg``.

    Structural checks keep this usable without Isaac Sim.
    """

    props = getattr(spawn, "articulation_props", None)
    if props is None:
        return False
    return getattr(props, "articulation_enabled", None) is False


def static_decor_prim_paths(scene_cfg: Any, env_regex_ns: str) -> Iterator[str]:
    """Yield the resolved prim-path pattern of every static-decor scene entry."""

    from srb.core.asset import AssetBaseCfg
    from srb.utils.config_introspection import iter_config_values_by_type

    for asset_cfg in iter_config_values_by_type(scene_cfg, AssetBaseCfg):
        prim_path = getattr(asset_cfg, "prim_path", None)
        if not isinstance(prim_path, str):
            continue
        if not spawn_is_static_decor(getattr(asset_cfg, "spawn", None)):
            continue
        yield prim_path.replace("{ENV_REGEX_NS}", env_regex_ns)


def disable_joints_on_static_decor(scene_cfg: Any, env_regex_ns: str) -> int:
    """Disable every joint under a static-decor asset.

    :returns: The number of joints that were turned off.
    """

    import isaaclab.sim as sim_utils
    from pxr import Usd, UsdPhysics

    disabled = 0
    for pattern in static_decor_prim_paths(scene_cfg, env_regex_ns):
        for root in sim_utils.find_matching_prims(pattern):
            for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                if not prim.IsA(UsdPhysics.Joint):
                    continue
                if prim.IsInstanceProxy():
                    # An instance proxy carries no authorable opinion; the
                    # prototype would have to be edited instead. Leave it and
                    # let the PhysX error stand rather than fail the load.
                    continue
                joint = UsdPhysics.Joint(prim)
                attr = joint.GetJointEnabledAttr()
                if attr and attr.Get() is False:
                    continue
                joint.CreateJointEnabledAttr(False)
                disabled += 1
    return disabled
