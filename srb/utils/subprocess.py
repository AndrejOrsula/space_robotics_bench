import subprocess
from collections.abc import Iterable
from typing import Literal

from srb.utils import logging
from srb.utils.process_supervisor import (
    DEFAULT_TERM_GRACE_S,
    INTERACTIVE_KILL_GRACE_S,
    ProcessCleanupError,
    ProcessOwnershipError,
    terminate_owned_process,
)


def terminate_process(
    cmd: str | Iterable[str],
    process: subprocess.Popen[str] | subprocess.Popen[bytes] | None = None,
    signal: Literal[
        "INT",
        "QUIT",
        "ABRT",
        "KILL",
        "TERM",
        "STOP",
    ] = "TERM",
    *,
    term_grace_s: float = DEFAULT_TERM_GRACE_S,
    kill_grace_s: float = INTERACTIVE_KILL_GRACE_S,
) -> None:
    """Terminate an owned process handle; ``cmd`` remains for compatibility.

    A handle is required to identify the process safely. Keep TERM grace short
    for Kit, then allow the interactive KILL-to-reap window.
    """

    del cmd
    if process is None:
        # Returning would report success and let callers proceed as if it exited.
        raise ValueError(
            "terminate_process requires a process handle; a command string "
            "cannot identify an owned process safely"
        )
    if process.poll() is not None:
        # Avoid replacing the original error if this runs in a caller's finally.
        return

    try:
        result = terminate_owned_process(
            process,
            signal_name=signal,
            term_grace_s=term_grace_s,
            kill_grace_s=kill_grace_s,
        )
        if not result.cleanup_complete:
            raise ProcessCleanupError(result)
    except (ProcessCleanupError, ProcessOwnershipError):
        raise
    except Exception as error:  # noqa: BLE001
        # Teardown must not report success while Kit remains alive. Keep the log
        # because this raise may replace an error already reported by finally.
        logging.error(f"Failed to terminate process: {error}")
        raise
