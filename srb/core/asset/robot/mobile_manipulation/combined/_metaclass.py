from typing import Any, Dict

from pydantic import model_validator

from srb.core.asset.robot.manipulation import Manipulator
from srb.core.asset.robot.mobile.mobile_robot import MobileRobot
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
    MobileManipulator,
)


class CombinedMobileManipulator(
    MobileManipulator, MobileRobot, mobile_manipulator_metaclass=True, extra="allow"
):
    ## Model
    mobile_base: MobileRobot | None = None
    manipulator: Manipulator

    @model_validator(mode="before")
    @classmethod
    def override_mobile_base(cls, data: Dict[str, Any]) -> Dict[str, Any]:
        if isinstance(data.get("mobile_base"), MobileRobot):
            for key, value in data["mobile_base"]._model_values.items():
                data[key] = value
            # data["mobile_base"] = None
        return data

    def model_post_init(self, __context):
        if isinstance(self.mobile_base, MobileRobot):
            for key, value in self.mobile_base._model_values.items():
                setattr(self, key, value)
            # self.mobile_base = None
        super().model_post_init(__context)
