from srb.core.action.action_group import ActionGroup
from srb.core.action.term import DummyActionCfg
from srb.utils.cfg import configclass


@configclass
class DummyActionGroup(ActionGroup):
    dummy: DummyActionCfg = DummyActionCfg()
