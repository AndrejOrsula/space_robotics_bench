from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .main import run as run

__all__ = ["run"]


def __getattr__(name: str):
    if name == "run":
        from srb.integrations.rsl_rl.main import run

        return run
    raise AttributeError(name)
