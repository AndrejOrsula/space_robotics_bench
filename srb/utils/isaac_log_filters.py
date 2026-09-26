from __future__ import annotations

import os

from srb.utils import logging as _srb_logging

try:
    from carb import logging as _carb_logging
except Exception:  # pragma: no cover - carb is only present inside Isaac/Kit
    _carb_logging = None  # type: ignore[assignment]


_DISABLE_ENV_VAR = "SRB_DISABLE_ISAAC_LOG_FILTERS"
_TRUTHY_ENV_VALUES = frozenset({"1", "true", "yes", "on"})
_debug = _srb_logging.debug


def install_isaac_startup_log_filters() -> None:
    """Raise thresholds for known benign Isaac/Kit startup log sources."""
    if os.environ.get(_DISABLE_ENV_VAR, "").strip().lower() in _TRUTHY_ENV_VALUES:
        _debug(f"Isaac startup log filters disabled via {_DISABLE_ENV_VAR}.")
        return

    if _carb_logging is None:
        _debug(
            "Isaac startup log filters unavailable because carb.logging could not be imported."
        )
        return

    logger = _carb_logging.acquire_logging()
    behavior = _carb_logging.LogSettingBehavior.OVERRIDE
    logger.set_level_threshold_for_source(
        "omni.fabric.plugin",
        behavior,
        _carb_logging.LEVEL_ERROR,
    )
    logger.set_level_threshold_for_source(
        "omni.physx.tensors.plugin",
        behavior,
        _carb_logging.LEVEL_FATAL,
    )
    _debug("Installed Isaac startup log filters for 2 source patterns.")
