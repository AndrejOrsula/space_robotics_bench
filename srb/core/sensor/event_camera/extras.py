"""Import gate for optional event-camera dependencies.

Adapters use :func:`require_extra` to turn import failures into actionable
:class:`MissingExtraError` exceptions. Its implementation is shared in
:mod:`srb.utils.extras`; this module re-exports it. Git-pinned providers are
installed separately from PyPI extras, and their hints include that step.
"""

from __future__ import annotations

from srb.utils.extras import (  # noqa: F401  (re-export)
    E2VID_SOURCE_REQUIREMENT,
    SOURCE_REQUIREMENTS,
    V2E_SOURCE_REQUIREMENT,
    MissingExtraError,
    install_command,
    require_extra,
)

__all__ = [
    "E2VID_SOURCE_REQUIREMENT",
    "SOURCE_REQUIREMENTS",
    "V2E_SOURCE_REQUIREMENT",
    "MissingExtraError",
    "install_command",
    "require_extra",
]
