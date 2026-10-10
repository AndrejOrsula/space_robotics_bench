from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from srb.utils.path import SRB_DIR, SRB_HYPERPARAMS_DIR

from .generator_types import PyExpr


class RegistrationKwargsRewriter:
    """Rewrite env_spec.kwargs with relocatable expressions for SRB paths."""

    def rewrite(
        self, registration_kwargs: dict[str, Any]
    ) -> tuple[dict[str, Any], set[str]]:
        registration_kwargs = dict(registration_kwargs)
        registration_kwargs.pop("task_cfg", None)

        used_srb_paths: set[str] = set()
        for key, value in registration_kwargs.items():
            registration_kwargs[key] = _rewrite_value(value, used_srb_paths)

        return registration_kwargs, used_srb_paths


def _rewrite_value(value: Any, used_srb_paths: set[str]) -> Any:
    maybe_path = _value_as_existing_path(value)
    if maybe_path is not None:
        for variable, root in (
            ("SRB_HYPERPARAMS_DIR", SRB_HYPERPARAMS_DIR),
            ("SRB_DIR", SRB_DIR),
        ):
            try:
                relative = maybe_path.relative_to(root).as_posix()
            except ValueError:
                continue
            used_srb_paths.add(variable)
            return PyExpr(f"{variable}.joinpath({json.dumps(relative)}).as_posix()")
    if isinstance(value, dict):
        return {
            key: _rewrite_value(item, used_srb_paths) for key, item in value.items()
        }
    if isinstance(value, list):
        return [_rewrite_value(item, used_srb_paths) for item in value]
    if isinstance(value, tuple):
        return tuple(_rewrite_value(item, used_srb_paths) for item in value)
    return value


def _value_as_existing_path(value: Any) -> Path | None:
    if isinstance(value, Path):
        p = value
    elif isinstance(value, str):
        p = Path(value)
    else:
        return None

    try:
        p = p.resolve()
    except Exception:
        return None

    return p if p.exists() else None
