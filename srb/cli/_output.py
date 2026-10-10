"""Structured output helpers for SRB CLI commands."""

from __future__ import annotations

import json
from typing import Any


def json_payload(value: Any, **to_dict_kwargs: Any) -> Any:
    """Convert an SRB result with ``to_dict`` when available."""
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        return to_dict(**to_dict_kwargs)
    return value


def format_json(value: Any, **to_dict_kwargs: Any) -> str:
    """Serialize an SRB result as sorted, indented JSON."""
    return json.dumps(
        json_payload(value, **to_dict_kwargs),
        indent=2,
        sort_keys=True,
    )


def print_json(value: Any, **to_dict_kwargs: Any) -> None:
    """Print an SRB result as sorted, indented JSON."""
    print(format_json(value, **to_dict_kwargs))
