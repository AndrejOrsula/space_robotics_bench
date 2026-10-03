import re
from typing import Any

from pydantic import model_validator

from srb.core.asset.robot.manipulation import Manipulator
from srb.core.asset.robot.mobile.mobile_robot import MobileRobot, MobileRobotRegistry
from srb.core.asset.robot.mobile_manipulation.mobile_manipulator import (
    MobileManipulator,
)

_ENV_PREFIX_RE = re.compile(r"^\{ENV_REGEX_NS\}/([^/]+)(.*)")


def _rescope_sensor_prim_paths(
    sensor_cfgs: dict[str, Any], *, target_segment: str
) -> None:
    """Rewrite the first path segment of every ``{ENV_REGEX_NS}``-scoped sensor
    prim path (and FrameTransformer target-frame paths) to ``target_segment``.

    Assets author their sensor prim paths under their own name
    (``{ENV_REGEX_NS}/<asset>/...``). Non-``{ENV_REGEX_NS}`` paths (e.g. a
    ``/Visuals/...`` visualizer) are left untouched. Idempotent: an
    already-rescoped path maps to itself.
    """

    def _rewrite(path: str) -> str:
        m = _ENV_PREFIX_RE.match(path)
        if not m:
            return path
        return f"{{ENV_REGEX_NS}}/{target_segment}{m.group(2)}"

    for cfg in sensor_cfgs.values():
        prim_path = getattr(cfg, "prim_path", None)
        if isinstance(prim_path, str):
            cfg.prim_path = _rewrite(prim_path)
        target_frames = getattr(cfg, "target_frames", None)
        if target_frames:
            for frame in target_frames:
                frame_path = getattr(frame, "prim_path", None)
                if isinstance(frame_path, str):
                    frame.prim_path = _rewrite(frame_path)


class CombinedMobileManipulator(
    MobileManipulator, MobileRobot, mobile_manipulator_metaclass=True, extra="allow"
):
    ## Model
    mobile_base: MobileRobot | None = None
    manipulator: Manipulator
    ## Name of the concrete mobile_base class used during flattening, kept for
    ## Hydra round-trip serialization so the mobile_base can be reconstructed.
    mobile_base_name: str | None = None

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        # Composites list as mobile manipulators only, not as mobile bases.
        for mobile_robots in MobileRobotRegistry.registry.values():
            if cls in mobile_robots:
                mobile_robots.remove(cls)

    @model_validator(mode="before")
    @classmethod
    def override_mobile_base(cls, data: dict[str, Any]) -> dict[str, Any]:
        if isinstance(data.get("mobile_base"), MobileRobot):
            mobile_base = data["mobile_base"]
            for key, value in mobile_base._model_values.items():
                data[key] = value
            # action_modes is a class-level registry on the mobile_base's
            # concrete type, not a Pydantic model field — copy it onto the
            # composite instance so `self.action_modes` resolves to the
            # right dict. (The generic composite classes declare
            # action_modes as a regular field so it can be shadowed here.)
            data["action_modes"] = type(mobile_base).action_modes
            # Remember the concrete mobile_base class name for Hydra serialization
            # so that the composite can be reconstructed without losing the
            # mobile_base identity after flattening sets mobile_base=None.
            data["mobile_base_name"] = type(mobile_base).name()
            # Preserve instance-level action_mode selector if the user set
            # it on the mobile_base before assembly.
            if getattr(mobile_base, "action_mode", None) is not None:
                data["action_mode"] = mobile_base.action_mode
            data["mobile_base"] = None
        return data

    def model_post_init(self, __context):
        super().model_post_init(__context)
        self.combine_mobile_manipulator()

    def combine_mobile_manipulator(self):
        ## Propagate all mobile robot attributes to self (fallback for
        ## model_construct() which bypasses validators)
        if isinstance(self.mobile_base, MobileRobot):
            mobile_base = self.mobile_base
            for key, value in mobile_base._model_values.items():
                setattr(self, key, value)
            self.action_modes = type(mobile_base).action_modes
            self.mobile_base_name = type(mobile_base).name()
            if getattr(mobile_base, "action_mode", None) is not None:
                self.action_mode = mobile_base.action_mode
            self.mobile_base = None

    def configure_sensors(self):
        if isinstance(self.manipulator, Manipulator):
            self.manipulator.configure_sensors()
            # The manipulator authors its sensor prim paths under its own asset
            # name (e.g. "{ENV_REGEX_NS}/franka/..."). In the combined mobile
            # manipulator the manipulator is spawned as a SEPARATE child prim at
            # "{ENV_REGEX_NS}/manipulator" (see BaseEnvCfg._add_robot), NOT under
            # the mobile-base root "{ENV_REGEX_NS}/robot". Re-scope the merged
            # sensor prim paths to the manipulator prim so scene wiring
            # (_add_sensors -> _fix_prim_path, which preserves the "manipulator"
            # sibling segment) resolves them against the right prims. Without
            # this the FrameTransformer/contact sensors point at
            # "{ENV_REGEX_NS}/robot/<manip_link>", which does not exist, and env
            # construction fails at sensor init.
            _rescope_sensor_prim_paths(
                self.manipulator.sensor_cfgs, target_segment="manipulator"
            )
            self.sensor_cfgs.update(self.manipulator.sensor_cfgs)
