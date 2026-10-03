from __future__ import annotations

from typing import Sequence

from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Light(Object, object_entrypoint=ObjectType.LIGHT):
    @classmethod
    def object_registry(cls) -> Sequence[type[Light]]:
        return ObjectRegistry.registry.get(ObjectType.LIGHT, [])  # type: ignore[return-value]
