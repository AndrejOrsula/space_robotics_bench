"""SRB task package; importing it does not register tasks.

Call :func:`register_all` after Isaac Sim/AppLauncher starts, or
:func:`ensure_registered` when registration state is unknown.
"""

from srb.utils.registry import get_srb_tasks

from .registration import register_all

__all__ = ["ensure_registered", "register_all"]


def ensure_registered() -> None:
    """Register SRB tasks when none are registered yet."""
    register_all()
    if not get_srb_tasks():
        raise RuntimeError(
            "No SRB tasks registered after calling srb.tasks.register_all(). "
            "Start Isaac Sim/AppLauncher before registering tasks and check "
            "earlier task import errors."
        )
