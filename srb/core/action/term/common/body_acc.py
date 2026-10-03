from typing import TYPE_CHECKING, ClassVar

import torch

from srb.core.manager import ActionTerm, ActionTermCfg
from srb.utils.cfg import configclass
from srb.utils.math import quat_apply

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import Articulation, RigidObject


class BodyAccelerationAction(ActionTerm):
    """Commanded root acceleration, integrated into the root velocity.

    The six normalized channels (linear x/y/z, angular x/y/z) are clamped to
    [-1, 1] and multiplied by ``cfg.scale`` once per environment step, giving a
    linear acceleration in m/s^2 and an angular acceleration in rad/s^2.
    ``apply_actions`` runs once per physics substep and adds
    ``acceleration * physics_dt`` to the root velocity the simulator reports,
    so one environment step changes the velocity by ``acceleration * step_dt``
    whatever the decimation, and gravity or contact impulses the simulator
    already integrated are kept rather than counted again.

    With ``cfg.relative`` the command is expressed in the body frame and is
    rotated into the world frame by the current root orientation on every
    substep; otherwise it is a world-frame command.
    """

    ACTION_DIM: ClassVar[int] = 6

    cfg: "BodyAccelerationActionCfg"
    _env: "AnyEnv"
    _asset: "Articulation | RigidObject"

    @property
    def action_dim(self) -> int:
        return self.ACTION_DIM

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions):
        self._raw_actions = actions
        # Clamp the normalized action to [-1, 1] before scaling so an
        # out-of-range policy output cannot command an unbounded acceleration
        # (matches the clamp every other action term applies).
        self._processed_actions = self.raw_actions.clamp(-1.0, 1.0) * self.cfg.scale

    def _acceleration_w(self) -> torch.Tensor:
        acceleration = self.processed_actions
        if not self.cfg.relative:
            return acceleration
        root_quat_w = self._asset._data.root_quat_w
        return torch.cat(
            (
                quat_apply(root_quat_w, acceleration[:, :3]),
                quat_apply(root_quat_w, acceleration[:, 3:]),
            ),
            dim=-1,
        )

    def apply_actions(self):
        velocity_w = (
            self._asset._data.body_vel_w[:, 0]
            + self._acceleration_w() * self._env.physics_dt
        )
        self._asset.write_root_velocity_to_sim(velocity_w)


@configclass
class BodyAccelerationActionCfg(ActionTermCfg):
    class_type: type = BodyAccelerationAction

    relative: bool = False
    """Express the commanded acceleration in the body frame instead of the world frame."""

    scale: float = 1.0
    """Acceleration at full normalized action: m/s^2 (linear) and rad/s^2 (angular)."""
