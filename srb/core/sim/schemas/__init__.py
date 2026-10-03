from isaaclab.sim.schemas import *  # noqa: F403

# Isaac Lab develop (3.0) relocated the PhysX schema property configs to
# ``isaaclab_physx.sim.schemas`` and re-exposes them on ``isaaclab.sim.schemas``
# through a lazy module ``__getattr__`` forwarding shim. Those forwarded names
# are intentionally NOT listed in ``isaaclab.sim.schemas.__all__``, so the
# wildcard ``import *`` above does not bind them. Import them explicitly (an
# explicit ``from ... import`` triggers the module ``__getattr__``) so they
# re-export through ``srb.core.sim`` for the asset/robot/scenery modules that
# construct ``collision_props``/``rigid_props``/etc.
from isaaclab.sim.schemas import (  # noqa: F401
    ArticulationRootPropertiesCfg,
    CollisionPropertiesCfg,
    FixedTendonPropertiesCfg,
    JointDrivePropertiesCfg,
    MassPropertiesCfg,
    RigidBodyPropertiesCfg,
)
from simforge.integrations.isaaclab.schemas import (  # noqa: F401
    MeshCollisionPropertiesCfg,
)
