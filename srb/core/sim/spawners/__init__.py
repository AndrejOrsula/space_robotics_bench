from isaaclab.sim.spawners import *  # noqa: F403

from .particles import *  # noqa: F403
from .shapes import *  # noqa: F403
from .terrains import *  # noqa: F403

# isort: split

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

# simforge.integrations.isaaclab uses isaacsim.core.prims and isaacsim.core.utils
# which are Kit extensions not auto-loaded by the headless experience in newer
# Isaac Sim releases. srb.core.asset already enables and adds them to sys.path,
# so this block is a no-op if asset is imported first. If not, try again here.

from simforge.integrations.isaaclab.spawner import (  # noqa: F401
    FileCfg,
    SimforgeAssetCfg,
    UsdFileCfg,
)

from .simforge_compat import install_simforge_isaaclab_compat

install_simforge_isaaclab_compat()
del install_simforge_isaaclab_compat
