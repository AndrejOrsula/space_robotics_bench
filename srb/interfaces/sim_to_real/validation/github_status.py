"""Best-effort GitHub commit status posting for validation sessions."""

from __future__ import annotations

import logging
import shutil

from srb.interfaces.sim_to_real.validation.result import SessionResult
from srb.utils.process_supervisor import ProcessKind, ProcessStatus, run_owned

logger = logging.getLogger(__name__)


def github_status_command(
    *,
    sha: str,
    task: str,
    algo: str,
    result: SessionResult,
) -> list[str]:
    """Build the gh CLI command used to publish a validation status."""

    state = "success" if result.pass_overall else "failure"
    return [
        "gh",
        "api",
        "-X",
        "POST",
        f"repos/:owner/:repo/statuses/{sha}",
        "-f",
        f"state={state}",
        "-f",
        f"context=srb/real-eval/{task}",
        "-f",
        f"description={result.exit_code} ({algo})",
    ]


def post_github_status(
    *,
    sha: str,
    task: str,
    algo: str,
    result: SessionResult,
) -> None:
    """Post a validation status through gh CLI without failing the session."""

    if shutil.which("gh") is None:
        logger.info("gh CLI not present; skipping --post-status")
        return
    try:
        outcome = run_owned(
            github_status_command(
                sha=sha,
                task=task,
                algo=algo,
                result=result,
            ),
            kind=ProcessKind.FINITE,
            timeout=30.0,
            capture_output=True,
        )
        if not outcome.cleanup_complete:
            logger.warning("gh status post cleanup failed: %s", outcome.error)
        elif outcome.status is ProcessStatus.TIMED_OUT:
            logger.warning(
                "gh status post timed out: %s",
                outcome.error or outcome.stderr or outcome.stdout or "no diagnostics",
            )
        elif outcome.returncode:
            logger.warning(
                "gh status post exited %s: %s",
                outcome.returncode,
                outcome.stderr or outcome.stdout or "no diagnostics",
            )
    except Exception as exc:  # noqa: BLE001
        logger.warning("gh status post failed: %s", exc)
