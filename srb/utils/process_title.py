"""Optional process retitling for shared fleet hosts."""

from __future__ import annotations

import os

__all__ = ["PROCESS_TITLE_ENV", "apply_process_title"]

#: Title to install; unset or empty leaves the process unchanged.
PROCESS_TITLE_ENV = "SRB_PROCESS_TITLE"


def apply_process_title(
    environ: os._Environ[str] | dict[str, str] | None = None,
) -> str | None:
    """Install the requested title; return ``None`` if unset or unavailable.

    ``setproctitle`` updates the name shown in process monitors and the command
    line.
    """
    source = os.environ if environ is None else environ
    title = (source.get(PROCESS_TITLE_ENV) or "").strip()
    if not title:
        return None
    try:
        import setproctitle
    except ImportError:
        return None
    setproctitle.setproctitle(title)
    return title
