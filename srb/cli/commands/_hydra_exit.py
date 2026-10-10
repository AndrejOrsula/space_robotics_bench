"""Preserve failure exit codes for CLI workflows running under Isaac Sim.

Kit may exit with code 0 from ``SimulationApp.close()``, swallowing Python's
``SystemExit``. Workflows must report failure before handing control to Kit.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable

__all__ = ["exit_nonzero_if_failed", "exit_now", "invoke_hydra_entrypoint"]


def invoke_hydra_entrypoint(entrypoint: Callable[[], None], *, label: str) -> bool:
    """Run Hydra and report whether the workflow failed.

    Log nonzero ``SystemExit`` and other ``BaseException`` failures because
    Kit may swallow the exit code and buffered stderr. Code 0 or ``None`` is
    normal; other exits, including ``KeyboardInterrupt``, indicate failure.
    """
    from srb.utils import logging

    try:
        entrypoint()
    except SystemExit as exc:
        if exc.code in (0, None):
            return False
        logging.error(f"{label} ABORTED before completion: exit code {exc.code}")
        return True
    except BaseException as exc:  # noqa: BLE001 - fail closed on anything
        from hydra.errors import CompactHydraException

        from srb.utils.failures import FailureError

        # Interrupts, Hydra config errors and typed failures are
        # self-explanatory; skip the traceback.
        quiet = isinstance(
            exc, (KeyboardInterrupt, CompactHydraException, FailureError)
        )
        logging.error(
            f"{label} ABORTED before completion: {str(exc) or type(exc).__name__}",
            exc_info=not quiet,
        )
        return True
    return False


def exit_nonzero_if_failed(failed: bool) -> None:
    """Exit with status 1 on failure; Kit may swallow ``SystemExit``."""
    if failed:
        exit_now(1)


def exit_now(code: int) -> None:
    """Flush output and exit with ``code``, bypassing Kit's shutdown."""
    _flush_process_sinks()
    os._exit(code)


def _flush_process_sinks() -> None:
    """Flush logging handlers and standard streams before ``os._exit``."""
    import logging as stdlib_logging

    stdlib_logging.shutdown()
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.flush()
        except (OSError, ValueError):
            # A closed or broken stream must not stop the non-zero exit.
            pass
