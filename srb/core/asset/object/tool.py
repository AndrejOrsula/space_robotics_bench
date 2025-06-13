from __future__ import annotations

from typing import Sequence, Type

from srb.core.action import ActionGroup
from srb.core.asset import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from srb.core.asset.common import Frame
from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Tool(Object, object_entrypoint=ObjectType.TOOL):
    ## Model
    asset_cfg: AssetBaseCfg | RigidObjectCfg | ArticulationCfg

    ## Frames
    frame_mount: Frame
    frame_tool_centre_point: Frame

    @classmethod
    def object_registry(cls) -> Sequence[Type[Tool]]:
        return ObjectRegistry.registry.get(ObjectType.TOOL, [])  # type: ignore


class ActiveTool(Tool, object_metaclass=True):
    ## Model
    asset_cfg: ArticulationCfg

    ## Actions
    actions: ActionGroup
