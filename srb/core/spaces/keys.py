"""Shared cross-framework wire keys for ``srb.core.spaces``."""

from __future__ import annotations

#: Non-``Dict`` observations use this key for frameworks requiring dict spaces.
#: Wrappers remove it before data reaches checkpoints or datasets.
ROOT_OBSERVATION_KEY = "__observation__"

__all__ = ["ROOT_OBSERVATION_KEY"]
