from typing import Sequence

import gymnasium
import numpy as np

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)


class MoveitGripperInterfaceCfg(HardwareInterfaceCfg):
    gripper_joint_names: Sequence[str] = ()
    open_gripper_joint_positions: Sequence[float] = ()
    closed_gripper_joint_positions: Sequence[float] = ()
    gripper_group_name: str = "gripper"


class MoveitGripperInterface(HardwareInterface):
    cfg: MoveitGripperInterfaceCfg

    def __init__(self, cfg: MoveitGripperInterfaceCfg | None = None):
        if cfg is None:
            cfg = MoveitGripperInterfaceCfg()
        super().__init__(cfg)
        self._gripper_disabled = False
        self.obs: dict[str, np.ndarray] = {
            "proprio_dyn/joint_pos_end_effector_normalized": ...,
        }

    def start(self, **kwargs):
        super().start(**kwargs)
        from pymoveit2 import MoveIt2Gripper

        self.moveit_gripper = MoveIt2Gripper(
            self.ros_node,
            gripper_joint_names=self.cfg.gripper_joint_names,
            open_gripper_joint_positions=self.cfg.open_gripper_joint_positions,
            closed_gripper_joint_positions=self.cfg.closed_gripper_joint_positions,
            gripper_group_name=self.cfg.gripper_group_name,
        )
        self._gripper_disabled = False

    def close(self):
        super().close()

        gripper = getattr(self, "moveit_gripper", None)
        if gripper is not None and not self._gripper_disabled:
            gripper.disable()
            self._gripper_disabled = True

    def _safe_stop(self) -> None:
        """Physical safe state: hold the current gripper position.

        The gripper has no velocity command to zero; a safe stop cancels any
        pending motion goal and latches the pause so no later open/close
        command can be dispatched.
        """
        gripper = getattr(self, "moveit_gripper", None)
        cancel = getattr(gripper, "cancel", None) if gripper is not None else None
        if callable(cancel):
            cancel()
        super()._safe_stop()

    def reset(self):
        super().reset()

    def sync(self):
        super().sync()
        self._update_joint_pos()

    @property
    def supported_action_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "end_effector/joint_pos": gymnasium.spaces.Box(
                    low=-1.0, high=1.0, shape=(1,), dtype=np.float32
                )
            }
        )

    def apply_action(self, action: dict[str, np.ndarray]):
        if "end_effector/joint_pos" not in action:
            raise ValueError(
                f"[{self.name}] Missing action key 'end_effector/joint_pos'"
            )
        if action["end_effector/joint_pos"].shape != (1,):
            raise ValueError(
                f"[{self.name}] Action shape mismatch: expected (1,), "
                f"got {action['end_effector/joint_pos'].shape}"
            )
        if not np.all(np.isfinite(action["end_effector/joint_pos"])):
            raise ValueError(f"[{self.name}] Action contains non-finite values")
        self.validate_dispatch_ready()

        if action["end_effector/joint_pos"] > 0.0:
            self.moveit_gripper.open()
        else:
            self.moveit_gripper.close()

    @property
    def observation(self) -> dict[str, np.ndarray]:
        return self.obs.copy()

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        return gymnasium.spaces.Dict(
            {
                "proprio_dyn/joint_pos_end_effector_normalized": gymnasium.spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(1,),
                    dtype=np.float32,
                )
            }
        )

    def _update_joint_pos(self):
        if not self.cfg.gripper_joint_names:
            raise ValueError("MoveIt gripper must configure at least one joint")

        joint_state = self.moveit_gripper.joint_state
        if joint_state is None:
            raise RuntimeError("MoveIt gripper joint-state feedback is unavailable")

        names = list(joint_state.name)
        missing = [name for name in self.cfg.gripper_joint_names if name not in names]
        if missing:
            raise RuntimeError(
                "MoveIt gripper joint-state feedback is missing configured joint(s): "
                + ", ".join(missing)
            )

        duplicate = [
            name for name in self.cfg.gripper_joint_names if names.count(name) != 1
        ]
        if duplicate:
            raise RuntimeError(
                "MoveIt gripper joint-state feedback has duplicate configured "
                "joint(s): " + ", ".join(duplicate)
            )

        if not (
            len(self.cfg.gripper_joint_names)
            == len(self.cfg.open_gripper_joint_positions)
            == len(self.cfg.closed_gripper_joint_positions)
        ):
            raise ValueError(
                "MoveIt gripper joint names, open positions, and closed positions "
                "must have matching lengths"
            )

        indices = [names.index(name) for name in self.cfg.gripper_joint_names]
        if any(index >= len(joint_state.position) for index in indices):
            raise RuntimeError(
                "MoveIt gripper joint-state feedback has fewer positions than names"
            )

        # Normalize position: -1 is closed, 1 is open.
        open_positions = np.array(
            self.cfg.open_gripper_joint_positions, dtype=np.float32
        )
        closed_positions = np.array(
            self.cfg.closed_gripper_joint_positions, dtype=np.float32
        )
        current_positions = np.array(
            [joint_state.position[i] for i in indices],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(current_positions)):
            raise RuntimeError(
                "MoveIt gripper joint-state feedback contains non-finite positions"
            )

        # Map [closed, open] to [-1, 1], guarding zero-width ranges.
        position_range = open_positions - closed_positions
        position_range = np.where(np.abs(position_range) < 1e-6, 1.0, position_range)
        normalized = 2.0 * (current_positions - closed_positions) / position_range - 1.0
        if not np.all(np.isfinite(normalized)):
            raise RuntimeError(
                "MoveIt gripper feedback produced non-finite normalized positions"
            )

        # Reduce all joints to one observation.
        self.obs["proprio_dyn/joint_pos_end_effector_normalized"] = np.array(
            [np.mean(normalized)], dtype=np.float32
        )
