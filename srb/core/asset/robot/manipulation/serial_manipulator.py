from __future__ import annotations

from typing import Sequence

from srb.core.asset.common import Frame
from srb.core.asset.robot.manipulation.manipulator import (
    Manipulator,
    ManipulatorRegistry,
)
from srb.core.asset.robot.manipulation.manipulator_type import ManipulatorType


class SerialManipulator(Manipulator, manipulator_entrypoint=ManipulatorType.SERIAL):
    ## Frames
    frame_flange: Frame

    def configure_sensors(self):
        super().configure_sensors()

        # Imported lazily: this is a foundational ``core.asset`` module loaded
        # during package init, so a top-level import of core.sensor/core.marker
        # could invite an import cycle. configure_sensors runs once per instance.
        from srb.core.action.term.manipulation._frame_jacobian import (
            end_effector_frame_offset,
        )
        from srb.core.marker import FRAME_MARKER_SMALL_CFG
        from srb.core.sensor import FrameTransformerCfg, OffsetCfg
        from srb.utils.math import combine_frame_transforms_tuple

        # The tracked ``robot_ee`` frame sits at the tool centre point
        # (flange ∘ TCP) when a tool is mounted, and at the bare flange
        # otherwise; the task-space action terms bind to the same composition
        #
        ee_frame = end_effector_frame_offset(
            self, combine=combine_frame_transforms_tuple
        )
        assert ee_frame is not None  # a serial manipulator always has a flange
        ee_pos, ee_rot = ee_frame

        self.sensor_cfgs["tf_end_effector"] = FrameTransformerCfg(
            prim_path=(
                f"{self.asset_cfg.prim_path}/{self.frame_base.prim_relpath}"
                if self.frame_base.prim_relpath
                else self.asset_cfg.prim_path
            ),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    name="robot_ee",
                    prim_path=f"{self.asset_cfg.prim_path}/{self.frame_flange.prim_relpath}",
                    offset=OffsetCfg(pos=ee_pos, rot=ee_rot),
                ),
            ],
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(
                prim_path="/Visuals/robot_ee"
            ),
        )

    @classmethod
    def manipulator_registry(cls) -> Sequence[type[SerialManipulator]]:
        return ManipulatorRegistry.registry.get(ManipulatorType.SERIAL, [])  # type: ignore[return-value]
