import typing as _typing

import warp as _warp
from isaaclab.assets import *  # noqa: F403
from isaaclab.utils.warp.proxy_array import ProxyArray as _ProxyArray

from srb.utils.warp_compat import install_proxy_array_launch_compat as _ipalc

# Patch isaaclab.utils.warp.ProxyArray so unknown attribute lookups fall back
# to the cached `.torch` view. ProxyArray (introduced in Isaac Lab v4.6.13) is
# the new return type for asset/sensor data properties. Upstream provides:
#   - __torch_function__ (handles torch.* free functions auto-unwrap)
#   - __getitem__ / __setitem__ (indexing)
#   - arithmetic / comparison operators
# but no method fallback. This means `proxy.mean(dim=1)`, `proxy.cpu()`,
# `proxy.size(0)`, `proxy.numpy()`, `proxy.unsqueeze(0)`, etc. all raise
# AttributeError because the deprecation shim does not cover Tensor methods.
#
# SRB tasks were written against torch.Tensor data properties and call dozens
# of such methods. Adding `__getattr__` here delegates any unknown attribute
# (after Python's normal lookup chain) to the cached torch view, restoring
# backwards-compat without touching every call site.
#
# `__getattr__` is only invoked when normal lookup fails, so this does not
# shadow ProxyArray's own properties (warp, torch, shape, dtype, device, etc.).
if not getattr(_ProxyArray, "_srb_torch_fallback_applied", False):

    def _srb_proxy_torch_fallback(self, name):
        # Avoid recursive lookup if `.torch` itself is unavailable.
        if name in ("torch", "_warp", "_torch_cache"):
            raise AttributeError(name)
        return getattr(self.torch, name)

    _ProxyArray.__getattr__ = _srb_proxy_torch_fallback
    _ProxyArray._srb_torch_fallback_applied = True
_ipalc(_warp, _ProxyArray)
del _ProxyArray, _ipalc, _warp

# SimForge defaults to `${XDG_CACHE_HOME:-~/.cache}/simforge`. On dev machines it
# can happen that this directory (or its children) becomes root-owned (e.g. after
# running an Isaac/Blender tool under sudo), which then breaks asset generation
# in any new process with a PermissionError. Redirect to a writable cache root
# early (before importing simforge) when needed.
try:
    from srb.utils import logging as _logging
    from srb.utils.simforge_cache import (
        redirect_unwritable_simforge_cache as _redirect_simforge_cache,
    )

    _fallback = _redirect_simforge_cache()
    if _fallback is not None:
        _logging.warning(
            "Redirected XDG_CACHE_HOME to a writable location (%s) to keep SimForge cache writeable",
            _fallback,
        )

    del _fallback, _logging, _redirect_simforge_cache
except Exception:
    pass

from srb.core.sim.spawners import *  # noqa: F403, E402

# IsaacLab develop's `spawn_multi_asset` raises when handed a concrete prim
# path (no '.*' in the base name) without planner-assigned `spawn_paths` —
# exactly what simforge's `spawn_simforge_assets` receives whenever a
# SimForge spawner escapes SRB's pre-scene conversion to MultiAssetSpawnerCfg
# (see srb.core.sim.spawners.multi_asset_compat). Rebind the name resolved by
# simforge's impl module to the tolerant variant.
try:
    from simforge.integrations.isaaclab.spawner.simforge_asset import (
        impl as _sf_simforge_asset_impl,
    )

    from srb.core.sim.spawners.multi_asset_compat import (
        spawn_multi_asset_compat as _srb_spawn_multi_asset_compat,
    )

    if not getattr(_sf_simforge_asset_impl, "_srb_multi_asset_compat", False):
        _sf_simforge_asset_impl.spawn_multi_asset = _srb_spawn_multi_asset_compat
        _sf_simforge_asset_impl._srb_multi_asset_compat = True
    del _sf_simforge_asset_impl, _srb_spawn_multi_asset_compat
except Exception:
    from srb.utils import logging as _logging

    _logging.warning(
        "Failed to patch simforge's spawn_multi_asset with the SRB-compatible "
        "variant; SimForge spawners outside the scene cfg may fail to spawn"
    )
    del _logging

from .asset import Asset, AssetRegistry  # noqa: F401, E402
from .asset_type import AssetType  # noqa: F401, E402
from .asset_variant import AssetVariant  # noqa: F401, E402
from .common import *  # noqa: F403, E402
from .object import *  # noqa: F403, E402
from .particle_system import (  # noqa: F401, E402
    ParticleSystem,
    ParticleSystemCfg,
    ParticleSystemData,
)
from .robot import *  # noqa: F403, E402
from .scenery import *  # noqa: F403, E402

# Runtime wildcard exports above already expose these classes. Explicit type
# exports preserve that public contract under strict implicit-reexport checks.
if _typing.TYPE_CHECKING:
    from .object.pedestal import Pedestal as Pedestal
    from .robot.manipulation.manipulator import Manipulator as Manipulator
    from .robot.mobile.aerial_robot import AerialRobot as AerialRobot
    from .robot.mobile.ground_robot import GroundRobot as GroundRobot
    from .robot.mobile.mobile_robot import MobileRobot as MobileRobot
    from .robot.mobile.orbital_robot import OrbitalRobot as OrbitalRobot
    from .robot.mobile_manipulation.combined.aerial_manipulator import (
        AerialManipulator as AerialManipulator,
    )
    from .robot.mobile_manipulation.combined.ground_manipulator import (
        GroundManipulator as GroundManipulator,
    )
    from .robot.mobile_manipulation.combined.orbital_manipulator import (
        OrbitalManipulator as OrbitalManipulator,
    )
