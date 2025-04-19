from __future__ import annotations

from typing import Sequence, Type

from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Pedestal(Object, object_entrypoint=ObjectType.PEDESTAL):
    @classmethod
    def object_registry(cls) -> Sequence[Type[Pedestal]]:
        return ObjectRegistry.registry.get(ObjectType.PEDESTAL, [])  # type: ignore
