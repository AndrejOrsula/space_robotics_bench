from __future__ import annotations

from typing import ClassVar, Sequence

from srb.core.action import ActionGroup
from srb.core.asset import ArticulationCfg, Frame
from srb.core.asset._action_mode_utils import _make_action_group_property
from srb.core.asset.object.object import Object, ObjectRegistry
from srb.core.asset.object.object_type import ObjectType


class Payload(Object, object_entrypoint=ObjectType.PAYLOAD):
    @classmethod
    def object_registry(cls) -> Sequence[type[Payload]]:
        return ObjectRegistry.registry.get(ObjectType.PAYLOAD, [])  # type: ignore[return-value]


class ActivePayload(Payload, object_metaclass=True):
    ## Model
    asset_cfg: ArticulationCfg

    ## Frame on the payload articulation attached to the mobile robot.
    frame_mount: Frame

    ## Action modes — dict of available control groups keyed by lexical
    ## name. Same API as :attr:`Robot.action_modes` — class-level ClassVar
    ## because the dict contains non-serialisable callables; subclasses
    ## declare it. Insertion order matters: first entry is the default.
    action_modes: ClassVar[dict[str, ActionGroup]]

    ## Selected mode key. None → use first entry of action_modes.
    action_mode: str | None = None

    ## Resolved action group (property — never stored).
    action_group = _make_action_group_property()
