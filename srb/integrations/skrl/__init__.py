# Expose optional adapter types to static checkers.
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .runner import SrbSkrlRunner as SrbSkrlRunner
    from .runner import runner_for_config as runner_for_config
    from .sac import SrbSAC as SrbSAC

#: Target skrl API major version. The 2.1 API separates observations/states and
#: uses dataclass configuration; the adapter patches compatibility at import.
SUPPORTED_SKRL_MAJOR = 2


def require_supported_skrl() -> str:
    """Return the installed skrl version or raise when the adapter cannot use it."""
    from importlib.metadata import PackageNotFoundError, version

    from srb.utils.failures import OptionalDependencyUnavailableError

    try:
        installed = version("skrl")
    except PackageNotFoundError as exc:
        raise OptionalDependencyUnavailableError(
            "skrl is not installed; install the `skrl` extra to use the skrl_* algorithms"
        ) from exc
    major = installed.partition(".")[0]
    if not major.isdigit() or int(major) != SUPPORTED_SKRL_MAJOR:
        raise OptionalDependencyUnavailableError(
            f"skrl {installed} is installed, but the SRB skrl adapter targets the "
            f"skrl {SUPPORTED_SKRL_MAJOR}.x API. Install the aligned `skrl` extra."
        )
    return installed


def run(*args, **kwargs):
    """Load the Isaac/SKRL runtime only when the integration is executed."""
    require_supported_skrl()
    from srb.integrations.skrl.main import run as _run

    return _run(*args, **kwargs)


def __getattr__(name: str):
    if name in {"SrbSAC", "SrbSkrlRunner", "runner_for_config"}:
        require_supported_skrl()
        from srb.integrations.skrl.runner import SrbSkrlRunner, runner_for_config
        from srb.integrations.skrl.sac import SrbSAC

        return {
            "SrbSAC": SrbSAC,
            "SrbSkrlRunner": SrbSkrlRunner,
            "runner_for_config": runner_for_config,
        }[name]
    raise AttributeError(name)


__all__ = [
    "SUPPORTED_SKRL_MAJOR",
    "SrbSAC",
    "SrbSkrlRunner",
    "require_supported_skrl",
    "run",
    "runner_for_config",
]
