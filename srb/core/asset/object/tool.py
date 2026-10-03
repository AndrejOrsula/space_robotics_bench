from __future__ import annotations

from typing import ClassVar, Sequence

from srb.core.action import ActionGroup
from srb.core.asset import ArticulationCfg, RigidObjectCfg
from srb.core.asset._action_mode_utils import _make_action_group_property
from srb.core.asset.common import Frame
from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Tool(Object, object_entrypoint=ObjectType.TOOL):
    ## Model
    asset_cfg: RigidObjectCfg | ArticulationCfg

    ## Frames
    frame_mount: Frame
    frame_tool_centre_point: Frame

    @classmethod
    def object_registry(cls) -> Sequence[type[Tool]]:
        return ObjectRegistry.registry.get(ObjectType.TOOL, [])  # type: ignore[return-value]


class ActiveTool(Tool, object_metaclass=True):
    ## Model
    asset_cfg: ArticulationCfg

    ## Action modes — dict of available control groups keyed by lexical
    ## name. Same API as :attr:`Robot.action_modes` — class-level ClassVar
    ## because the dict contains non-serialisable callables; subclasses
    ## declare it. Insertion order matters: first entry is the default.
    action_modes: ClassVar[dict[str, ActionGroup]]

    ## Selected mode key. None → use first entry of action_modes.
    action_mode: str | None = None

    ## Resolved action group (property — never stored).
    action_group = _make_action_group_property()
