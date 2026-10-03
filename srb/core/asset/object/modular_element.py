from __future__ import annotations

from functools import cached_property
from itertools import chain
from typing import Sequence

from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class ModularElement(Object, object_entrypoint=ObjectType.MODULAR_ELEMENT):
    pegs: Sequence[tuple[float, float, float]]
    holes: Sequence[tuple[float, float, float]]
    holes_entrance_offset: Sequence[tuple[float, float, float]]

    @classmethod
    def object_registry(cls) -> Sequence[type[ModularElement]]:
        return ObjectRegistry.registry.get(ObjectType.MODULAR_ELEMENT, [])  # type: ignore[return-value]

    @cached_property
    def assembly_spacing(self) -> tuple[float, float, float]:
        corners = (
            tuple(c[0] for c in chain(self.pegs, self.holes)),
            tuple(c[1] for c in chain(self.pegs, self.holes)),
            tuple(c[2] for c in chain(self.pegs, self.holes)),
        )
        return (
            max(corners[0]) - min(corners[0]),
            max(corners[1]) - min(corners[1]),
            max(corners[2]) - min(corners[2]),
        )

    @cached_property
    def holes_entrance(self) -> Sequence[tuple[float, float, float]]:
        return tuple(
            (
                hole[0] + offset[0],
                hole[1] + offset[1],
                hole[2] + offset[2],
            )
            for hole, offset in zip(
                self.holes,
                self.holes_entrance_offset,
                strict=True,
            )
        )
