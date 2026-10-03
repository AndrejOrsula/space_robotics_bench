from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any

import gymnasium
import numpy as np
import torch
from gymnasium import Space
from gymnasium.vector.utils import batch_space

from srb.core.spaces import is_dict


@dataclass(frozen=True, slots=True)
class EnvIntrospector:
    num_envs: int
    device: torch.device
    single_observation_space: Space[Any]
    single_action_space: Space[Any]
    observation_space: Space[Any]
    action_space: Space[Any]
    is_obs_dict: bool
    is_act_dict: bool

    @classmethod
    def of(cls, env: Any) -> "EnvIntrospector":
        unwrapped = getattr(env, "unwrapped", env)

        num_envs = runtime_num_envs(env, require_positive=True)
        device = getattr(unwrapped, "device")
        if not isinstance(device, torch.device):
            device = torch.device(device)

        single_observation_space, observation_space = _effective_spaces(
            env,
            unwrapped,
            kind="observation",
            num_envs=num_envs,
        )
        single_action_space, action_space = _effective_spaces(
            env,
            unwrapped,
            kind="action",
            num_envs=num_envs,
        )
        # Resolve the final effective pair exactly once above, then detach the
        # immutable view from every env-owned mutable Gym space.  Wrappers keep
        # these snapshots for their lifetime; no space copying occurs in step.
        single_observation_space = _snapshot_space(single_observation_space)
        observation_space = _snapshot_space(observation_space)
        single_action_space = _snapshot_space(single_action_space)
        action_space = _snapshot_space(action_space)
        return cls(
            num_envs=num_envs,
            device=device,
            single_observation_space=single_observation_space,
            single_action_space=single_action_space,
            observation_space=observation_space,
            action_space=action_space,
            is_obs_dict=is_dict(single_observation_space),
            is_act_dict=is_dict(single_action_space),
        )


def _snapshot_space(space: Space[Any]) -> Space[Any]:
    """Return a deep defensive snapshot of a supported Gym space tree."""

    snapshot = deepcopy(space)
    if not isinstance(snapshot, Space):
        raise TypeError(f"Gym space snapshot produced {snapshot!r}")
    return snapshot


def runtime_num_envs(env: Any, *, require_positive: bool = False) -> int:
    """Return the runtime environment count owned by the unwrapped env.

    Generic Gym environments may omit ``num_envs``; wrappers represent that
    as zero.  SRB integrations call this through ``EnvIntrospector`` with
    ``require_positive=True`` because they require a batched runtime.
    """
    unwrapped = getattr(env, "unwrapped", env)
    return _validate_runtime_num_envs(unwrapped, require_positive=require_positive)


def _validate_runtime_num_envs(
    unwrapped: Any, *, require_positive: bool = False
) -> int:
    value = getattr(unwrapped, "num_envs", 0)
    if value is None:
        value = 0
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(
            f"unwrapped.num_envs must be a non-negative int, got {value!r}"
        )
    if require_positive and value == 0:
        raise ValueError(f"unwrapped.num_envs must be a positive int, got {value!r}")
    return value


def _effective_spaces(
    env: Any,
    unwrapped: Any,
    *,
    kind: str,
    num_envs: int,
) -> tuple[Space[Any], Space[Any]]:
    single_name = f"single_{kind}_space"
    batch_name = f"{kind}_space"
    missing = object()

    outer_single = _read_space_attr(env, single_name, missing)
    outer_batch = _read_space_attr(env, batch_name, missing)
    base_single = _read_space_attr(unwrapped, single_name, missing)
    base_batch = _read_space_attr(unwrapped, batch_name, missing)

    single = next(
        (
            value
            for value in (outer_single, base_single, outer_batch, base_batch)
            if value is not missing and value is not None
        ),
        missing,
    )
    # Only a declared batched space may act as one. Falling back to the single
    # space here would hide "no batched space is declared" behind an alias that
    # the validation below then has to reject; the derivation branch takes that
    # case instead.
    batch = next(
        (
            value
            for value in (outer_batch, base_batch)
            if value is not missing and value is not None
        ),
        missing,
    )
    if single is missing:
        raise AttributeError(
            f"environment exposes neither {single_name} nor {batch_name}"
        )
    if batch is missing:
        if not isinstance(single, gymnasium.Space):
            raise TypeError(
                f"cannot derive batched {batch_name} from non-gym space {single}"
            )
        batch = batch_space(single, num_envs)
    if not isinstance(single, Space) or not isinstance(batch, Space):
        raise TypeError(
            f"effective {kind} spaces must be gymnasium spaces, got "
            f"single={single!r}, batched={batch!r}"
        )
    _validate_space_pair(single, batch, num_envs=num_envs, kind=kind)
    return single, batch


def _read_space_attr(env: Any, name: str, missing: object) -> Any:
    """Read a declared space without treating dynamic mock attributes as real.

    ``MagicMock`` manufactures arbitrary attributes on lookup.  Those are not
    effective spaces and must not shadow the explicitly configured unwrapped
    spaces.  Real instance attributes and descriptors remain authoritative;
    invalid declared values still reach the typed validation below.
    """
    value = getattr(env, name, missing)
    if value is missing or value is None or isinstance(value, Space):
        return value
    instance_attrs = getattr(env, "__dict__", {})
    if isinstance(instance_attrs, dict) and name in instance_attrs:
        return value
    class_attr = getattr(type(env), name, missing)
    if hasattr(class_attr, "__get__"):
        return value
    return missing


def _validate_space_pair(
    single: Space[Any],
    batch: Space[Any],
    *,
    num_envs: int,
    kind: str,
) -> None:
    # Some single-environment adapters expose the single space in both fields;
    # that alias is valid only for num_envs == 1. A vector env must declare a
    # batched space that is exactly Gymnasium's batch_space(single, num_envs),
    # never a stale wrapper/base copy or the aliased single space.
    if _space_signature(single) == _space_signature(batch):
        if num_envs == 1:
            return
        raise ValueError(
            f"effective {kind} spaces are aliased for a vector env with "
            f"num_envs={num_envs}: single={single!r}, batched={batch!r}"
        )
    expected = batch_space(single, num_envs)
    if _space_signature(expected) != _space_signature(batch):
        raise ValueError(
            f"effective {kind} spaces are inconsistent: single={single!r}, "
            f"batched={batch!r}, expected={expected!r}"
        )


def _space_signature(space: Space[Any]) -> tuple[Any, ...]:
    if isinstance(space, gymnasium.spaces.Dict):
        return (
            "Dict",
            tuple(
                (key, _space_signature(child)) for key, child in space.spaces.items()
            ),
        )
    if isinstance(space, gymnasium.spaces.Tuple):
        return ("Tuple", tuple(_space_signature(child) for child in space.spaces))
    # Spaces that carry their content in nested spaces or a character set:
    # comparing only type and dtype accepted, for example, a batched
    # `Sequence(Discrete(5))` for a single `Sequence(Discrete(3))`.
    if isinstance(space, gymnasium.spaces.Sequence):
        return ("Sequence", _space_signature(space.feature_space), bool(space.stack))
    if isinstance(space, gymnasium.spaces.Graph):
        edge = space.edge_space
        return (
            "Graph",
            _space_signature(space.node_space),
            None if edge is None else _space_signature(edge),
        )
    if isinstance(space, gymnasium.spaces.OneOf):
        return ("OneOf", tuple(_space_signature(child) for child in space.spaces))
    if isinstance(space, gymnasium.spaces.Text):
        return (
            "Text",
            int(space.min_length),
            int(space.max_length),
            tuple(sorted(space.character_set)),
        )
    # `Space.shape` is `None` -- present but null -- for `Text`, `Sequence` and
    # `Graph`. A `getattr(..., ())` default only covers an *absent* attribute,
    # so those reached `tuple(None)` and raised `TypeError: 'NoneType' object
    # is not iterable`, replacing this function's space-consistency diagnostic
    # with an opaque crash from inside the validator.
    raw_shape = getattr(space, "shape", ())
    shape = () if raw_shape is None else tuple(raw_shape)
    dtype = str(getattr(space, "dtype", None))
    low = getattr(space, "low", None)
    high = getattr(space, "high", None)
    return (
        type(space).__name__,
        shape,
        dtype,
        _array_signature(low),
        _array_signature(high),
        _cardinality_signature(space),
    )


#: Attributes that carry how many values a space admits, rather than how those
#: values are laid out. `Discrete` reports `shape == ()` and dtype `int64`
#: whatever its `n` is, and `batch_space(Discrete(n), num_envs)` is a
#: `MultiDiscrete`, which carries no `low`/`high` at all -- so without these,
#: `Discrete(5)` and `Discrete(7)` (and their batched forms) produce identical
#: signatures and `_validate_space_pair` accepts a batched space whose
#: per-element cardinality contradicts the single space. That mismatch is
#: exactly what it exists to reject.
#:
#: `Sequence`, `Graph`, `OneOf` and `Text` carry theirs in nested sub-spaces or
#: a character set and are compared through those in `_space_signature`.
_CARDINALITY_ATTRS: tuple[str, ...] = ("n", "nvec", "start", "min_length", "max_length")


def _cardinality_signature(space: Space[Any]) -> tuple[Any, ...]:
    return tuple(
        (name, _array_signature(value))
        for name in _CARDINALITY_ATTRS
        if (value := getattr(space, name, None)) is not None
    )


def _array_signature(value: Any) -> tuple[Any, ...] | None:
    if value is None:
        return None
    array = np.asarray(value)
    return (str(array.dtype), tuple(array.shape), array.tobytes())
