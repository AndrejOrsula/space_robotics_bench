"""Startup policy for simulation-agent runtime commands."""

from __future__ import annotations

from srb.tasks import ensure_registered
from srb.utils.cache import warn_if_offline_srb_cache_incomplete
from srb.utils.isaac_log_filters import install_isaac_startup_log_filters


def run_agent_startup_checks() -> None:
    """Run SRB-owned startup checks after Isaac AppLauncher is initialized."""
    install_isaac_startup_log_filters()
    warn_if_offline_srb_cache_incomplete()
    ensure_registered()


__all__ = ["run_agent_startup_checks"]
