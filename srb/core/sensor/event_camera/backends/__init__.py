"""Concrete event-generator backends.

Importing this package as a side effect registers every shipped backend with
:func:`srb.core.sensor.event_camera.backend.register_backend`. Each backend
keeps its heavy imports lazy so this package can be imported even when none
of the optional extras are installed; ``MissingExtraError`` surfaces at
backend *instantiation* time.
"""

from __future__ import annotations

from . import (
    metavision,  # noqa: F401  (side-effect: register_backend("metavision", MetavisionBackend))
    v2ce,  # noqa: F401  (side-effect: register_backend("v2ce", V2CEBackend))
    v2e,  # noqa: F401  (side-effect: register_backend("v2e", V2eBackend))
)

__all__: list[str] = []
