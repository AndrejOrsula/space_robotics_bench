# Compat shim: in IsaacLab develop the deformable-body material configs live
# under `isaaclab_physx.sim.spawners.materials`, but simforge's IsaacLab
# integration still imports `DeformableBodyMaterialCfg` from
# `isaaclab.sim.spawners`. Re-export under the legacy path so simforge can
# resolve its imports during cfg.py module load.
try:
    import isaaclab.sim.spawners as _ils

    if not hasattr(_ils, "DeformableBodyMaterialCfg"):
        from isaaclab_physx.sim.spawners.materials import (
            DeformableBodyMaterialCfg as _DBMC,
        )

        _ils.DeformableBodyMaterialCfg = _DBMC
        del _DBMC
    del _ils
except Exception:
    pass

from isaaclab.sim import *  # noqa: F403, E402

# Backward compatibility for task modules that import object/rigid object config from
# the legacy ``srb.core.sim`` path. Isaac Lab exposes these symbols under
# ``isaaclab.assets`` in newer releases.
try:
    from isaaclab.assets import RigidObjectCfg  # noqa: F401
except Exception:
    pass

from .schemas import *  # noqa: F403, E402
from .spawners import *  # noqa: F403, E402
