"""Namespaced logger factory for the terrain pipeline.

Use ``get_logger("fetch")`` for namespace ``srb.terrain.fetch``. The
``stage_timer`` context manager logs elapsed milliseconds at INFO so users
see per-stage timing without needing DEBUG.
"""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import Iterator

_LOGGER_PREFIX = "srb.terrain"


def get_logger(stage: str) -> logging.Logger:
    """Return logger ``srb.terrain.<stage>``."""
    return logging.getLogger(f"{_LOGGER_PREFIX}.{stage}")


@contextmanager
def stage_timer(logger: logging.Logger, label: str) -> Iterator[None]:
    """Log ``label`` elapsed time at INFO level on context exit."""
    start = time.perf_counter()
    try:
        yield
    finally:
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        logger.info("%s done in %.1f ms", label, elapsed_ms)
