import shutil
import sys
from functools import cache
from importlib.util import find_spec
from os import environ
from pathlib import Path

from srb.utils.capability_probe import CapabilityProbe, ModuleProbe


def require_isaac_sim() -> None:
    """Require the simulator symbols used by the runtime launcher."""
    CapabilityProbe(
        feature="Isaac Sim runtime",
        component="srb.cli.isaac",
        dependencies=("isaacsim", "isaaclab.app"),
        install_hint="a supported Isaac Sim environment",
        module_probes=(
            # Version details are unavailable through supported imports before
            # the app starts, so probe only the runtime symbols used here.
            ModuleProbe(
                "isaacsim",
                required_symbols=("SimulationApp",),
                callable_symbols=("SimulationApp",),
            ),
            ModuleProbe(
                "isaaclab.app",
                required_symbols=("AppLauncher",),
                callable_symbols=("AppLauncher",),
            ),
        ),
    ).require()


# Cached paths resolve the installation once; moving it mid-process is
# unsupported.
@cache
def get_isaacsim_path() -> str:
    if isaac_sim_path := environ.get("ISAAC_PATH"):
        return isaac_sim_path
    if isaac_sim_path := environ.get("ISAACSIM_PATH"):
        return isaac_sim_path

    standard_path = Path(environ.get("HOME", "/root")).joinpath("isaac-sim")
    if standard_path.exists():
        return standard_path.as_posix()

    return "/root/isaac-sim"


@cache
def get_isaacsim_python() -> str:
    if isaac_sim_python := environ.get("ISAACSIM_PYTHON"):
        return isaac_sim_python

    return Path(get_isaacsim_path()).joinpath("python.sh").as_posix()


def get_kit_python() -> str:
    """Return the current interpreter if it sees Isaac Sim, else ``python.sh``."""
    if find_spec("isaacsim") is not None:
        return sys.executable
    return get_isaacsim_python()


@cache
def get_isaacsim_app() -> str:
    app_path = Path(get_isaacsim_path()).joinpath("isaac-sim.sh")
    if app_path.exists():
        return app_path.as_posix()

    if isaacsim_bin := shutil.which("isaacsim"):
        return isaacsim_bin

    return app_path.as_posix()


def is_isaacsim_initialized() -> bool:
    try:
        return find_spec("isaacsim.core.version") is not None
    except Exception:
        # RuntimeApp imports may fail before startup; bootstrap handles that state.
        return False


def enable_isaacsim_extension(extension_name: str) -> None:
    """Enable an extension through Kit's public helper, imported lazily."""
    if not extension_name:
        raise ValueError("extension_name must be non-empty")

    try:
        from isaacsim.core.experimental.utils.app import enable_extension
    except ImportError as exc:
        raise RuntimeError(
            f"cannot enable Isaac Sim extension {extension_name!r}: "
            "Isaac Sim's public extension helper is unavailable"
        ) from exc

    try:
        enable_extension(extension_name)
    except Exception as exc:  # noqa: BLE001 - Kit raises backend-specific errors
        raise RuntimeError(
            f"could not enable Isaac Sim extension {extension_name!r}; "
            "start Kit before importing this module"
        ) from exc


def hide_isaacsim_ui() -> None:
    import carb.settings

    settings = carb.settings.get_settings()
    settings.set("/app/window/hideUi", True)
    settings.set("/app/window/fullscreen", True)


def update_app_without_physics(app, count: int = 1) -> None:
    """Pump Kit ``count`` times (rendering, Replicator) without stepping physics.

    A bare ``app.update()`` on a playing timeline also advances PhysX, so frame
    capture would speed up the simulation. Isaac Lab's Kit visualizer pauses
    ``playSimulations`` around its own updates the same way.
    """
    import carb.settings

    settings = carb.settings.get_settings()
    play = settings.get("/app/player/playSimulations")
    settings.set("/app/player/playSimulations", False)
    try:
        for _ in range(count):
            app.update()
    finally:
        settings.set("/app/player/playSimulations", play is None or bool(play))
