from __future__ import annotations

from typing import Sequence

from srb.core.asset.common import Frame
from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Pedestal(Object, object_entrypoint=ObjectType.PEDESTAL):
    ## Frames
    frame_manipulator_mount: Frame

    @classmethod
    def object_registry(cls) -> Sequence[type[Pedestal]]:
        return ObjectRegistry.registry.get(ObjectType.PEDESTAL, [])  # type: ignore[return-value]
