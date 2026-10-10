"""Map actor-visible SRB leaves to TD-MPC2's flat ``state`` input.

Landscaping uses contract order; other tasks use observation-space order after
unsupported actor-visible leaves are rejected.
Flattening delegates to
:class:`~srb.core.env.runtime.observation_layout.NumericObservationProjection`.
"""

from __future__ import annotations

import hashlib
import json
from collections import OrderedDict
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Mapping, Sequence

import gymnasium
import numpy as np
import torch

from srb._typing import ObservationGroup

if TYPE_CHECKING:
    from srb.core.env.runtime.observation_layout import NumericObservationProjection

#: The only vector observation key upstream TD-MPC2's encoder accepts.
STATE_KEY = "state"

_PRIVILEGED_PREFIX = f"{ObservationGroup.STATE_PRIVILEGED}__"

#: Dtypes accepted without widening by the shared projection.
_FLOAT_TENSOR_DTYPES = (
    torch.float16,
    torch.bfloat16,
    torch.float32,
    torch.float64,
)

_IMAGE_REJECTION_MESSAGE = (
    "TD-MPC2 observation '{key}' has shape {shape}: SRB's TD-MPC2 integration "
    "supports the upstream `state` encoder only. Upstream's `rgb` encoder "
    "requires a single 64x64 uint8 stream under the exact key 'rgb', which no "
    "SRB observation term provides. Select a state-only observation group for "
    "this task (or add and qualify an image path) instead of forwarding image "
    "leaves as unsupported encoder keys."
)


@dataclass(frozen=True)
class StateLeaf:
    """One assembled observation key routed into the flat ``state`` vector."""

    key: str
    shape: tuple[int, ...]
    output_slice: tuple[int, int]

    @property
    def size(self) -> int:
        return int(np.prod(self.shape, dtype=np.int64))


@dataclass(frozen=True)
class Tdmpc2StateTransform:
    """Project assembled SRB observations into one flat float32 state tensor."""

    leaves: tuple[StateLeaf, ...]
    state_space: gymnasium.spaces.Box
    projection: "NumericObservationProjection"
    contract_id: str | None = None
    io_schema_fingerprint: str | None = None
    #: Generic spaces may widen integer/bool leaves; contract leaves stay strict.
    casts_non_float_leaves: bool = False

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def from_observation_space(
        cls,
        space: gymnasium.Space,
        *,
        contract_projection: "NumericObservationProjection | None" = None,
    ) -> "Tdmpc2StateTransform":
        """Build the transform from a single-env observation space."""
        from srb.core.env.runtime.observation_layout import (
            NumericObservationProjection,
        )

        if contract_projection is not None:
            leaves = tuple(
                StateLeaf(
                    key=leaf.source_key,
                    shape=leaf.shape,
                    output_slice=leaf.output_slice,
                )
                for leaf in contract_projection.leaves
            )
            return cls(
                leaves=leaves,
                state_space=contract_projection.output_space,
                projection=contract_projection,
                contract_id=contract_projection.contract_id,
                io_schema_fingerprint=contract_projection.io_schema_fingerprint,
            )

        # Keep numeric actor leaves; reject critic-only and image-like inputs.
        if isinstance(space, gymnasium.spaces.Dict):
            entries: dict[str, gymnasium.spaces.Box] = {}
            for key, leaf_space in space.spaces.items():
                if str(key).startswith(_PRIVILEGED_PREFIX):
                    # Exclude critic-only signals from actor state.
                    continue
                entries[str(key)] = _validated_box(str(key), leaf_space)
            if not entries:
                raise ValueError(
                    "TD-MPC2 observation space contains no actor-visible leaves"
                )
        elif isinstance(space, gymnasium.spaces.Box):
            entries = {STATE_KEY: _validated_box(STATE_KEY, space)}
        else:
            raise TypeError(
                f"TD-MPC2 requires a Box or Dict observation space, got {space}"
            )

        projection = NumericObservationProjection.from_observation_space(
            # Preserve source order; Gymnasium sorts plain dict spaces.
            gymnasium.spaces.Dict(OrderedDict(entries))
        )
        leaves = tuple(
            StateLeaf(
                key=leaf.source_key,
                shape=leaf.shape,
                output_slice=leaf.output_slice,
            )
            for leaf in projection.leaves
        )
        return cls(
            leaves=leaves,
            state_space=projection.output_space,
            projection=projection,
            casts_non_float_leaves=True,
        )

    # ------------------------------------------------------------------
    # Derived metadata (one source of truth for every consumer)
    # ------------------------------------------------------------------

    @property
    def state_size(self) -> int:
        return int(self.state_space.shape[0])

    @property
    def obs_shape(self) -> dict[str, tuple[int, ...]]:
        """Exactly what upstream ``cfg.obs_shape`` must be for this env."""
        return {STATE_KEY: (self.state_size,)}

    @property
    def source_keys(self) -> tuple[str, ...]:
        seen: dict[str, None] = {}
        for leaf in self.leaves:
            seen.setdefault(leaf.key, None)
        return tuple(seen)

    def layout_fingerprint(self) -> str:
        """Stable hash of the exact key order/slicing behind ``state``."""
        payload = {
            "contract_id": self.contract_id,
            "io_schema_fingerprint": self.io_schema_fingerprint,
            "dtype": "float32",
            "state_size": self.state_size,
            "leaves": [
                {
                    "key": leaf.key,
                    "shape": list(leaf.shape),
                    "output_slice": list(leaf.output_slice),
                }
                for leaf in self.leaves
            ],
        }
        canonical = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        )
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    def record(self) -> dict[str, Any]:
        """Persisted description of the transform (saved with the run config)."""
        return {
            "state_size": self.state_size,
            "layout_fingerprint": self.layout_fingerprint(),
            "contract_id": self.contract_id,
            "io_schema_fingerprint": self.io_schema_fingerprint,
            "keys": list(self.source_keys),
        }

    # ------------------------------------------------------------------
    # Runtime transform
    # ------------------------------------------------------------------

    def transform(
        self,
        observation: Any,
        *,
        device: torch.device | str | None = None,
    ) -> torch.Tensor:
        """Project one assembled observation into ``(batch, state_size)`` f32."""
        if not isinstance(observation, Mapping):
            if len(self.leaves) != 1:
                raise TypeError(
                    "TD-MPC2 observation must be a mapping with keys "
                    f"{list(self.source_keys)}, got a bare "
                    f"{type(observation).__name__}. A single pre-flattened "
                    "array cannot be routed into this transform's "
                    f"{len(self.leaves)} leaves: pass the assembled "
                    "observation dict (the collector must not flatten or "
                    "rename it first)."
                )
            observation = {self.leaves[0].key: observation}

        sources = _as_tensor_mapping(
            observation,
            self.source_keys,
            device=device,
            cast_to_float32=self.casts_non_float_leaves,
        )
        state = self.projection.project_torch(sources)
        return state if device is None else state.to(device)


def _validated_box(key: str, space: gymnasium.Space) -> gymnasium.spaces.Box:
    if not isinstance(space, gymnasium.spaces.Box):
        raise TypeError(
            f"TD-MPC2 observation '{key}' must use a Box space, got {space}. "
            "Upstream's encoder cannot consume it."
        )
    shape = tuple(int(dim) for dim in space.shape)
    if len(shape) >= 3:
        raise ValueError(_IMAGE_REJECTION_MESSAGE.format(key=key, shape=shape))
    dtype = np.dtype(space.dtype)
    if not (
        np.issubdtype(dtype, np.floating)
        or np.issubdtype(dtype, np.integer)
        or np.issubdtype(dtype, np.bool_)
    ):
        raise TypeError(
            f"TD-MPC2 observation '{key}' dtype {dtype} is not numeric and "
            "cannot enter the flat state vector"
        )
    if np.issubdtype(dtype, np.uint8) and len(shape) == 2:
        raise ValueError(_IMAGE_REJECTION_MESSAGE.format(key=key, shape=shape))
    return space


def _as_tensor_mapping(
    observation: Any,
    expected_keys: Sequence[str],
    *,
    device: torch.device | str | None,
    cast_to_float32: bool = False,
) -> dict[str, torch.Tensor]:
    if not isinstance(observation, Mapping):
        raise TypeError(
            "TD-MPC2 observation must be a mapping with keys "
            f"{list(expected_keys)}, got {type(observation).__name__}"
        )
    expected = set(expected_keys)
    missing = sorted(expected - set(observation))
    if missing:
        raise ValueError(f"TD-MPC2 observation is missing keys: {missing}")
    unexpected = sorted(
        key
        for key in set(observation) - expected
        if not str(key).startswith(_PRIVILEGED_PREFIX)
    )
    if unexpected:
        raise ValueError(
            f"TD-MPC2 observation has unexpected actor-visible keys: {unexpected}"
        )
    tensors: dict[str, torch.Tensor] = {}
    for key in expected_keys:
        value = observation[key]
        if not isinstance(value, torch.Tensor):
            value = torch.as_tensor(np.asarray(value))
        if cast_to_float32 and value.dtype not in _FLOAT_TENSOR_DTYPES:
            # TD-MPC2 admits integer/bool leaves into its float32 state vector
            # (see `_validated_box`); the shared projection only accepts float
            # tensors, so widen here rather than loosening the contract path.
            value = value.to(dtype=torch.float32)
        if device is not None:
            value = value.to(device)
        tensors[key] = value
    return tensors


def resolve_landscaping_projection(
    unwrapped: Any,
    assembled_space: Any,
) -> "NumericObservationProjection | None":
    """Resolve landscaping's contract projection when its fingerprint is present.

    Kept local to avoid importing Dreamer's ``embodied``/``elements`` dependencies.
    """
    metadata_fn = getattr(unwrapped, "landscaping_contract_metadata", None)
    declared_fingerprint = getattr(unwrapped, "IO_SCHEMA_FINGERPRINT", None)
    if not callable(metadata_fn) and not declared_fingerprint:
        return None

    from srb.core.env.runtime.observation_layout import (
        NumericObservationProjection,
        contract_detailed_dict_space,
    )
    from srb.terrain.landscaping_contract import LANDSCAPING_IO_CONTRACT

    contract = LANDSCAPING_IO_CONTRACT
    if callable(metadata_fn):
        published = str(metadata_fn().get("io_schema_fingerprint"))
        # Simulator branch: SRB's runtime layout has always advertised
        # dtype-extrema Box bounds, so an all-infinite assembled bound carries
        # no packed-order information there.
        allow_unbounded = True
    else:
        published = str(declared_fingerprint)
        if published != contract.fingerprint():
            return None
        # Generated RealEnv branch: the generated space is *supposed* to carry
        # physical bounds, so a regression to dtype extrema violates the range
        # stop condition and must be rejected here, not tolerated.
        allow_unbounded = False

    projection = NumericObservationProjection.from_contract(
        contract,
        contract_detailed_dict_space(contract),
        assembled_space,
        allow_unbounded_assembled_bounds=allow_unbounded,
    )
    projection.verify_fingerprint(published)
    return projection


__all__ = [
    "STATE_KEY",
    "StateLeaf",
    "Tdmpc2StateTransform",
    "resolve_landscaping_projection",
]
