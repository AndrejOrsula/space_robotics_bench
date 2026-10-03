"""EnvSpec adapters. Gym space order follows declared spec order.

``to_gym_space`` passes ordered pairs because a plain dict is sorted by Gymnasium.
Flattened layouts and ONNX exports depend on this order; regenerate consumers
after changing it.
"""

from __future__ import annotations

import gymnasium
import numpy as np

from srb.core.spec.models import (
    ActionSpec,
    DType,
    EnvSpec,
    GroupSpec,
    ObservationSpec,
    TermSpec,
)

_DTYPE_TO_NUMPY: dict[DType, np.dtype] = {
    "float32": np.dtype("float32"),
    "float64": np.dtype("float64"),
    "int8": np.dtype("int8"),
    "int16": np.dtype("int16"),
    "int32": np.dtype("int32"),
    "int64": np.dtype("int64"),
    "bool": np.dtype("bool"),
    "uint8": np.dtype("uint8"),
    "uint16": np.dtype("uint16"),
    "uint32": np.dtype("uint32"),
    "uint64": np.dtype("uint64"),
}


def _term_to_box(t: TermSpec) -> gymnasium.spaces.Box:
    np_dtype = _DTYPE_TO_NUMPY[t.dtype]
    if t.min is not None and isinstance(t.min, (int, float)):
        low = float(t.min)
    elif isinstance(t.min, list):
        low = np.array(t.min, dtype=np_dtype).reshape(t.shape)
    else:
        low = -np.inf
    if t.max is not None and isinstance(t.max, (int, float)):
        high = float(t.max)
    elif isinstance(t.max, list):
        high = np.array(t.max, dtype=np_dtype).reshape(t.shape)
    else:
        high = np.inf
    return gymnasium.spaces.Box(low=low, high=high, shape=t.shape, dtype=np_dtype)


def _group_to_dict_space(
    g: GroupSpec,
    *,
    exclude_privileged: bool = False,
) -> gymnasium.spaces.Dict:
    # A sequence of (name, space) pairs preserves the declared term order
    # (gymnasium sorts keys when handed a plain dict).
    return gymnasium.spaces.Dict(
        [
            (name, _term_to_box(t))
            for name, t in g.terms.items()
            if not (exclude_privileged and t.privileged)
        ]
    )


def to_gym_space(
    spec: ObservationSpec | ActionSpec,
    *,
    exclude_privileged: bool = False,
) -> gymnasium.spaces.Dict:
    """Project a spec into nested gymnasium spaces in declared order.

    ``exclude_privileged=True`` drops privileged terms (actor-visible
    projection); a group whose terms are all privileged is dropped entirely.
    """
    groups: list[tuple[str, gymnasium.spaces.Dict]] = []
    for gname, g in spec.groups.items():
        group_space = _group_to_dict_space(g, exclude_privileged=exclude_privileged)
        if exclude_privileged and not group_space.spaces:
            continue
        groups.append((gname, group_space))
    return gymnasium.spaces.Dict(groups)


def onnx_export_mask(spec: ObservationSpec) -> set[str]:
    """Term names that must be filtered from ONNX export.

    Default: privileged=True OR provenance starts with "<fallback>" (low-trust).
    """
    out: set[str] = set()
    for group in spec.groups.values():
        for term in group.terms.values():
            if term.privileged or (term.provenance or "").startswith("<fallback>"):
                out.add(term.name)
    return out


def metric_keys(
    spec: EnvSpec,
    *,
    include_privileged: bool = False,
    trusted_only: bool = False,
) -> list[str]:
    """Flat list of '<group>.<term>' identifiers for use in validation thresholds."""
    out: list[str] = []
    for gname, group in spec.observation.groups.items():
        for term in group.terms.values():
            if not include_privileged and term.privileged:
                continue
            if trusted_only and (term.provenance or "").startswith("<fallback>"):
                continue
            out.append(f"{gname}.{term.name}")
    return out
