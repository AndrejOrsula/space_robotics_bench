from srb.core.action.action_group import ActionGroup
from srb.core.action.term import ThrustActionCfg
from srb.utils.cfg import configclass


@configclass
class ThrustActionGroup(ActionGroup):
    thrust: ThrustActionCfg = ThrustActionCfg(asset_name="robot")
