"""Isaac- and ROS-free observation keys shared by tasks and deployment.

The seven actor leaves come from
:mod:`srb.terrain.landscaping_contract`; this module groups them and defines
the heightmap and target-patch keys used by deploy adapters.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, TypeAlias

import numpy as np

from srb.terrain.landscaping_contract import (
    OBSERVATION_LEAVES as _CONTRACT_OBSERVATION_LEAVES,
)
from srb.terrain.landscaping_heightmap import (
    DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY,
    LandscapingHeightmapGeometry,
)

ObservationGroupName = Literal["proprio", "proprio_dyn", "state_privileged"]
ObservationShapeDim: TypeAlias = int | str

# Import the frozen leaf order, shapes, and bounds rather than restating them.
LANDSCAPING_ACTOR_CONTRACT_KEYS: tuple[str, ...] = tuple(
    leaf.key for leaf in _CONTRACT_OBSERVATION_LEAVES
)


def build_landscaping_actor_observation(
    leaves: dict[str, object],
) -> dict[str, dict[str, object]]:
    """Group the seven contract leaves into grouped task observations.

    ``leaves`` must contain exactly the contract keys
    (:data:`LANDSCAPING_ACTOR_CONTRACT_KEYS`); anything extra (masks, IMU,
    previous action, shovel state, privileged maps, world pose) is a contract
    violation and raises. Group-internal ordering follows the contract's leaf
    order.
    """
    expected = set(LANDSCAPING_ACTOR_CONTRACT_KEYS)
    provided = set(leaves)
    if provided != expected:
        missing = sorted(expected - provided)
        extra = sorted(provided - expected)
        raise ValueError(
            "actor observation must contain exactly the contract leaves; "
            f"missing={missing} extra={extra}"
        )
    grouped: dict[str, dict[str, object]] = {}
    for key in LANDSCAPING_ACTOR_CONTRACT_KEYS:
        group, _, name = key.partition("/")
        grouped.setdefault(group, {})[name] = leaves[key]
    return grouped


_DEFAULT_PATCH_SHAPE = (
    DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.ny,
    DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY.nx,
)


@dataclass(frozen=True, slots=True)
class LandscapingObservationTerm:
    """Named landscaping observation term shared by sim and deploy."""

    group: ObservationGroupName
    name: str
    shape: tuple[ObservationShapeDim, ...]

    @property
    def key(self) -> str:
        return f"{self.group}/{self.name}"

    @property
    def flat_key(self) -> str:
        return f"{self.group}__{self.name}"

    def batched_shape(self, num_envs: int) -> tuple[int, ...]:
        return (num_envs, *self._static_shape())

    def zeros_np(self) -> np.ndarray:
        return np.zeros(self._static_shape(), dtype=np.float32)

    def with_shape(self, shape: tuple[int, ...]) -> "LandscapingObservationTerm":
        """Return this term with a concrete shape for dynamic observation sizes."""
        return LandscapingObservationTerm(
            group=self.group,
            name=self.name,
            shape=shape,
        )

    def _static_shape(self) -> tuple[int, ...]:
        if all(isinstance(dim, int) for dim in self.shape):
            return self.shape  # type: ignore[return-value]
        raise ValueError(f"{self.key} has dynamic shape {self.shape!r}")


LANDSCAPING_HEIGHTMAP_OBSERVATION = LandscapingObservationTerm(
    group="proprio_dyn",
    name="heightmap",
    shape=_DEFAULT_PATCH_SHAPE,
)
LANDSCAPING_HEIGHTMAP_VALID_MASK_OBSERVATION = LandscapingObservationTerm(
    group="proprio_dyn",
    name="heightmap_valid_mask",
    shape=_DEFAULT_PATCH_SHAPE,
)
LANDSCAPING_TARGET_PATCH_OBSERVATION = LandscapingObservationTerm(
    group="proprio_dyn",
    name="target_patch_local",
    shape=_DEFAULT_PATCH_SHAPE,
)


def landscaping_heightmap_observation(
    *,
    heightmap: np.ndarray,
    valid_mask: np.ndarray,
) -> dict[str, np.ndarray]:
    """Return deploy-side heightmap observations under landscaping keys."""
    return {
        LANDSCAPING_HEIGHTMAP_OBSERVATION.key: np.asarray(
            heightmap,
            dtype=np.float32,
        ),
        LANDSCAPING_HEIGHTMAP_VALID_MASK_OBSERVATION.key: np.asarray(
            valid_mask,
            dtype=np.float32,
        ),
    }


def landscaping_heightmap_fill_observation(
    *,
    shape: tuple[int, int],
    fill_value: float,
) -> dict[str, np.ndarray]:
    """Return a filled heightmap plus empty validity mask for deploy startup."""
    return landscaping_heightmap_observation(
        heightmap=np.full(shape, fill_value, dtype=np.float32),
        valid_mask=np.zeros(shape, dtype=np.float32),
    )


def landscaping_target_patch_zero_observation(
    *,
    geometry: LandscapingHeightmapGeometry = DEFAULT_LANDSCAPING_HEIGHTMAP_GEOMETRY,
) -> dict[str, np.ndarray]:
    """Return a deploy-side zero target-patch observation for the grid geometry."""
    return {
        LANDSCAPING_TARGET_PATCH_OBSERVATION.key: np.zeros(
            (geometry.ny, geometry.nx),
            dtype=np.float32,
        )
    }


__all__ = [
    "LANDSCAPING_ACTOR_CONTRACT_KEYS",
    "LANDSCAPING_HEIGHTMAP_OBSERVATION",
    "LANDSCAPING_HEIGHTMAP_VALID_MASK_OBSERVATION",
    "LANDSCAPING_TARGET_PATCH_OBSERVATION",
    "LandscapingObservationTerm",
    "ObservationGroupName",
    "ObservationShapeDim",
    "build_landscaping_actor_observation",
    "landscaping_heightmap_fill_observation",
    "landscaping_heightmap_observation",
    "landscaping_target_patch_zero_observation",
]
