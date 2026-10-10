"""Isaac-free Optuna tuning driver; each trial boots Isaac in its own child.

Owns the study and spawns one ``srb agent train`` subprocess per trial. Do not
import Isaac Sim, isaaclab, the gym registry, or torch at module load time.
See ``docs/src/workflows/hyperparameter_tuning.md`` for usage.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import optuna


def require_optuna() -> "optuna":  # type: ignore[valid-type]
    """Import optional :mod:`optuna`, or raise an installation hint."""
    try:
        import optuna
    except ModuleNotFoundError as e:  # pragma: no cover - exercised only without extra
        if e.name != "optuna":
            raise
        raise ImportError(
            "The 'optuna' package is required for `srb agent tune`. "
            "Install it with: uv sync --extra tune"
        ) from e
    return optuna


def git_short_sha() -> str:
    """Return the repository's short commit SHA, or ``"unknown"``.

    Shared by export and baseline provenance; unlike
    :func:`srb.tuning.baselines.config_blob_sha`, this tracks the repository.
    """
    from srb.utils.process_supervisor import (
        ProcessKind,
        ProcessPlatformError,
        ProcessSpawnError,
        ProcessStatus,
        run_owned,
    )

    try:
        result = run_owned(
            ["git", "rev-parse", "--short", "HEAD"],
            kind=ProcessKind.FINITE,
            timeout=10.0,
            capture_output=True,
        )
        if (
            result.status is ProcessStatus.EXITED
            and result.returncode == 0
            and result.cleanup_complete
        ):
            return (result.stdout or "").strip() or "unknown"
    except (ProcessPlatformError, ProcessSpawnError):
        return "unknown"
    return "unknown"


__all__ = ("require_optuna", "git_short_sha")
