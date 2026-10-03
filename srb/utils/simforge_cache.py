"""Writable-cache routing for SimForge startup.

SimForge resolves its cache from ``XDG_CACHE_HOME`` during import.  SRB calls
this module before importing SimForge so a root-owned development cache cannot
break every later asset import.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

from srb.utils.path import SRB_CACHE_PATH

_WRITE_PROBE_NAME = ".srb_write_probe"


def redirect_unwritable_simforge_cache() -> Path | None:
    """Redirect ``XDG_CACHE_HOME`` when SimForge's current cache is unwritable.

    Returns the fallback XDG root when redirection occurs, otherwise ``None``.
    The fallback is probed as a best effort, matching startup's fail-open
    behavior; callers decide whether a redirect should be logged.
    """
    xdg_cache = Path(os.environ.get("XDG_CACHE_HOME") or (Path.home() / ".cache"))
    if _simforge_cache_writable(xdg_cache / "simforge"):
        return None

    fallback = Path(SRB_CACHE_PATH) / "xdg_cache"
    os.environ["XDG_CACHE_HOME"] = str(fallback)
    _simforge_cache_writable(fallback / "simforge")
    return fallback


def _simforge_cache_writable(simforge_root: Path) -> bool:
    """Return whether SimForge's root and existing model caches are writable."""
    if not _directory_writable(simforge_root):
        return False

    model_dir = simforge_root / "model"
    if model_dir.exists() and not _directory_writable(model_dir):
        return False

    if model_dir.exists():
        try:
            for child in model_dir.iterdir():
                if child.is_dir() and not _directory_writable(child):
                    return False
        except OSError:
            return False

    return True


def _directory_writable(path: Path) -> bool:
    """Probe one directory without leaving a file behind.

    The probe is a fresh exclusively created file, so a pre-existing entry
    (for example a symlink to another file) is never written or removed.
    """
    try:
        path.mkdir(parents=True, exist_ok=True)
        descriptor, probe = tempfile.mkstemp(prefix=_WRITE_PROBE_NAME, dir=path)
        os.close(descriptor)
        os.unlink(probe)
    except OSError:
        return False
    return True


__all__ = ["redirect_unwritable_simforge_cache"]
