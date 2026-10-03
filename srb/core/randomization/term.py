"""Tagged event-term config and typed category/trigger views."""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional


def _missing_isaac_dependency(exc: ModuleNotFoundError) -> bool:
    name = exc.name or ""
    return name == "isaaclab" or name == "isaacsim" or name.startswith("isaacsim.")


def _fallback_event_term_cfg_type() -> type:
    @dataclass
    class EventTermCfg:
        """Fallback event-term cfg for Isaac-free randomization inspection."""

        func: Any | None = None
        params: dict[str, Any] = field(default_factory=dict)
        mode: str = ""
        interval_range_s: tuple[float, float] | None = None
        is_global_time: bool = False
        min_step_count_between_reset: int = 0

    return EventTermCfg


def _loaded_attr(module_name: str, attr_name: str) -> Any | None:
    module = sys.modules.get(module_name)
    if module is None:
        return None
    return getattr(module, attr_name, None)


def _isaaclab_already_loaded() -> bool:
    return any(
        name == "isaaclab" or name.startswith("isaaclab.") for name in sys.modules
    )


def _loaded_event_term_cfg_type() -> type | None:
    for module_name in ("srb.core.manager", "isaaclab.managers"):
        event_term_cfg = _loaded_attr(module_name, "EventTermCfg")
        if isinstance(event_term_cfg, type):
            return event_term_cfg
    if "srb.core.manager" in sys.modules or "isaaclab.managers" in sys.modules:
        return None
    if not _isaaclab_already_loaded():
        return None
    try:
        from isaaclab.managers import EventTermCfg as IsaacEventTermCfg
    except ModuleNotFoundError as exc:
        if _missing_isaac_dependency(exc):
            return None
        raise
    return IsaacEventTermCfg


def _loaded_configclass() -> Any:
    for module_name in ("srb.utils.cfg", "isaaclab.utils"):
        loaded_configclass = _loaded_attr(module_name, "configclass")
        if callable(loaded_configclass):
            return loaded_configclass
    try:
        from srb.utils.cfg import configclass as loaded_configclass
    except ModuleNotFoundError as exc:
        if _missing_isaac_dependency(exc):
            return dataclass
        raise
    return loaded_configclass


_loaded_event_term_cfg = _loaded_event_term_cfg_type()
_event_term_cfg_is_fallback = _loaded_event_term_cfg is None
if _event_term_cfg_is_fallback:
    EventTermCfg = _fallback_event_term_cfg_type()
    configclass = dataclass
else:
    EventTermCfg = _loaded_event_term_cfg
    configclass = _loaded_configclass()


def _upgrade_event_term_cfg_base_if_available() -> None:
    global EventTermCfg, configclass, _event_term_cfg_is_fallback

    if not _event_term_cfg_is_fallback:
        return
    event_term_cfg = _loaded_event_term_cfg_type()
    if event_term_cfg is None or event_term_cfg is EventTermCfg:
        return
    SrbEventTermCfg.__bases__ = (event_term_cfg,)
    EventTermCfg = event_term_cfg
    configclass = _loaded_configclass()
    _event_term_cfg_is_fallback = False


def is_event_term_cfg(term: object) -> bool:
    """Return true for the active fallback type or a loaded Isaac type."""
    _upgrade_event_term_cfg_base_if_available()
    if isinstance(term, EventTermCfg):
        return True
    for module_name in ("srb.core.manager", "isaaclab.managers"):
        event_term_cfg = _loaded_attr(module_name, "EventTermCfg")
        if (
            isinstance(event_term_cfg, type)
            and event_term_cfg is not EventTermCfg
            and isinstance(term, event_term_cfg)
        ):
            return True
    return False


class RandomizationCategory(str, Enum):
    """Top-level randomization axis."""

    VISUAL = "visual"
    PHYSICS = "physics"
    TASK = "task"
    SENSOR = "sensor"
    LIFECYCLE = "lifecycle"
    UNTAGGED = "untagged"


class RandomizationTrigger(str, Enum):
    """Typed view over Isaac Lab's free-form ``EventTermCfg.mode``."""

    RESET = "reset"
    INTERVAL = "interval"
    STARTUP = "startup"

    @classmethod
    def from_mode(cls, mode: str) -> Optional["RandomizationTrigger"]:
        try:
            return cls(mode)
        except ValueError:
            return None


@configclass
class SrbEventTermCfg(EventTermCfg):
    """``EventTermCfg`` with a category tag and trigger derived from ``mode``."""

    category: RandomizationCategory = RandomizationCategory.UNTAGGED
    """Randomization axis; defaults to ``UNTAGGED``."""

    def __new__(cls, *args, **kwargs):
        _upgrade_event_term_cfg_base_if_available()
        return super().__new__(cls)

    @property
    def trigger(self) -> Optional[RandomizationTrigger]:
        """Return the typed mode, or ``None`` when the mode is unknown."""
        return RandomizationTrigger.from_mode(self.mode)
