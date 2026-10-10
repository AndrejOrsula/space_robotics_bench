"""Publish reward, termination, action, and joint-state step outputs."""

from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from typing import Any

import torch

from srb.interfaces._ros_node import (
    RosCleanupScope,
    RosEndpointCleanupError,
    destroy_ros_endpoint,
)
from srb.interfaces.ros_namespace import RosNamespace

ClearActions = Callable[[torch.Tensor | None], None]
SubscriptionsConfigured = Callable[[], bool]


def _ros_step_messages() -> tuple[type[Any], ...]:
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool as BoolMsg
    from std_msgs.msg import (
        Float32,
        Float64MultiArray,
        Header,
        MultiArrayDimension,
        MultiArrayLayout,
    )

    return (
        Time,
        Clock,
        JointState,
        BoolMsg,
        Float32,
        Float64MultiArray,
        Header,
        MultiArrayDimension,
        MultiArrayLayout,
    )


def _scene_articulation_accessors() -> tuple[Any, Any]:
    from srb.core.scene_introspection import (
        get_scene_articulation,
        iter_scene_articulations,
    )

    return get_scene_articulation, iter_scene_articulations


class RosStepOutput:
    def __init__(
        self,
        *,
        env: Any,
        node: Any,
        namespace: RosNamespace,
        qos_profile: Any,
        clear_actions: ClearActions,
        action_subscriptions_configured: SubscriptionsConfigured,
    ) -> None:
        self._env = env
        self._node = node
        self._num_envs = int(env.num_envs)
        self._namespace = namespace
        self._qos_profile = qos_profile
        self._clear_actions = clear_actions
        self._action_subscriptions_configured = action_subscriptions_configured
        self._action_publishers_resolved = False
        self._pub_action: Sequence[Any] | None = None
        self._lifecycle_lock = threading.RLock()
        self._close_requested = False
        self._closed = False
        self._owned_endpoints: list[tuple[Any, str, bool]] = []
        self._clock_pub: Any = None
        self._pub_reward: Sequence[Any] = ()
        self._pub_reward_term: dict[str, Sequence[Any]] = {}
        self._pub_terminated: Sequence[Any] = ()
        self._pub_truncated: Sequence[Any] = ()
        self._pub_joint_states: dict[str, Sequence[Any]] = {}

        (
            self._Time,
            self._Clock,
            self._JointState,
            self._BoolMsg,
            self._Float32,
            self._Float64MultiArray,
            self._Header,
            self._MultiArrayDimension,
            self._MultiArrayLayout,
        ) = _ros_step_messages()
        (
            self._get_scene_articulation,
            self._iter_scene_articulations,
        ) = _scene_articulation_accessors()

        with RosCleanupScope(
            self,
            "ROS step output construction cleanup incomplete",
        ):
            self._clock_pub = self._create_publisher(
                msg_type=self._Clock,
                topic="/clock",
                qos_profile=self._qos_profile,
            )
            self._pub_reward = tuple(
                self._create_publisher(
                    self._Float32,
                    self._namespace.env_topic(i, "reward"),
                    qos_profile=self._qos_profile,
                )
                for i in range(self._num_envs)
            )
            self._pub_terminated = tuple(
                self._create_publisher(
                    self._BoolMsg,
                    self._namespace.env_topic(i, "terminated"),
                    qos_profile=self._qos_profile,
                )
                for i in range(self._num_envs)
            )
            self._pub_truncated = tuple(
                self._create_publisher(
                    self._BoolMsg,
                    self._namespace.env_topic(i, "truncated"),
                    qos_profile=self._qos_profile,
                )
                for i in range(self._num_envs)
            )
            self._pub_joint_states = {
                articulation_name: tuple(
                    self._create_publisher(
                        self._JointState,
                        self._namespace.env_topic(
                            i,
                            f"{articulation_name}/joint_states",
                        ),
                        qos_profile=self._qos_profile,
                    )
                    for i in range(self._num_envs)
                )
                for articulation_name, _ in self._iter_scene_articulations(
                    self._env.scene
                )
            }

    def publish(
        self,
        *,
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: dict[str, Any],
        action: torch.Tensor | None = None,
    ) -> Any:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS step output is closed")
            self._ensure_delayed_publishers(info=info, action=action)
            action_echo = (
                action.clone()
                if action is not None and self._pub_action is not None
                else None
            )

            time_msg = self._time_msg()
            self._clock_pub.publish(self._Clock(clock=time_msg))

            self._publish_step_terms(
                reward=reward,
                terminated=terminated,
                truncated=truncated,
                info=info,
            )
            self._clear_reset_actions(terminated=terminated, truncated=truncated)
            self._publish_action(action_echo)
            self._publish_joint_states(time_msg)
            return time_msg

    def _ensure_delayed_publishers(
        self,
        *,
        info: dict[str, Any],
        action: torch.Tensor | None,
    ) -> None:
        # Each publisher group is created when its data first appears, so a
        # reward term or action absent from the first step is not lost.
        for reward_term in info.get("reward_terms", ()):
            if reward_term in self._pub_reward_term:
                continue
            self._pub_reward_term[reward_term] = tuple(
                self._create_publisher(
                    self._Float32,
                    self._namespace.env_topic(i, f"reward/{reward_term}"),
                    qos_profile=self._qos_profile,
                )
                for i in range(self._num_envs)
            )

        if self._action_publishers_resolved or action is None:
            return
        if not self._action_subscriptions_configured():
            self._pub_action = tuple(
                self._create_publisher(
                    self._Float64MultiArray,
                    self._namespace.env_topic(i, "action"),
                    qos_profile=self._qos_profile,
                )
                for i in range(self._num_envs)
            )
        self._action_publishers_resolved = True

    def _time_msg(self) -> Any:
        time_s = self._env.common_step_counter * self._env.step_dt
        return self._Time(sec=int(time_s), nanosec=int((time_s % 1) * 1e9))

    def _publish_step_terms(
        self,
        *,
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: dict[str, Any],
    ) -> None:
        # Transfer each tensor once, not once per environment.
        reward_np = reward.cpu().numpy()
        terminated_np = terminated.cpu().numpy()
        truncated_np = truncated.cpu().numpy()
        # Publish only the terms present this step; a term may come and go.
        reward_terms = info.get("reward_terms", {})
        reward_terms_np = {
            reward_term: reward_terms[reward_term].cpu().numpy()
            for reward_term in self._pub_reward_term
            if reward_term in reward_terms
        }
        for i in range(self._num_envs):
            self._pub_reward[i].publish(self._Float32(data=float(reward_np[i])))
            for reward_term, values in reward_terms_np.items():
                self._pub_reward_term[reward_term][i].publish(
                    self._Float32(data=float(values[i]))
                )
            self._pub_terminated[i].publish(self._BoolMsg(data=bool(terminated_np[i])))
            self._pub_truncated[i].publish(self._BoolMsg(data=bool(truncated_np[i])))

    def _clear_reset_actions(
        self,
        *,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
    ) -> None:
        reset_mask = terminated | truncated
        if reset_mask.any():
            self._clear_actions(reset_mask)

    def _publish_action(self, action: torch.Tensor | None) -> None:
        if action is None or self._pub_action is None:
            return

        layout = self._MultiArrayLayout(
            dim=[
                # A one-dimensional stride equals the dimension size in elements.
                self._MultiArrayDimension(
                    label="action",
                    size=action.shape[1],
                    stride=action.shape[1],
                )
            ],
            data_offset=0,
        )
        for i in range(self._num_envs):
            self._pub_action[i].publish(
                self._Float64MultiArray(layout=layout, data=action[i].tolist())
            )

    def _publish_joint_states(self, time_msg: Any) -> None:
        for articulation_name, pubs in self._pub_joint_states.items():
            articulation = self._get_scene_articulation(
                self._env.scene, articulation_name
            )
            # Transfer each tensor once, not once per environment.
            joint_pos_np = articulation.data.joint_pos.cpu().numpy()
            joint_vel_np = articulation.data.joint_vel.cpu().numpy()
            joint_eff_np = articulation.data.applied_torque.cpu().numpy()
            for i in range(self._num_envs):
                pubs[i].publish(
                    self._JointState(
                        header=self._Header(
                            stamp=time_msg,
                            frame_id=self._namespace.env_frame(
                                i,
                                articulation_name,
                            ),
                        ),
                        name=articulation.data.joint_names,
                        position=joint_pos_np[i].tolist(),
                        velocity=joint_vel_np[i].tolist(),
                        effort=joint_eff_np[i].tolist(),
                    )
                )

    def _create_publisher(
        self,
        msg_type: type[Any],
        topic: str,
        *,
        qos_profile: Any,
    ) -> Any:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS step output is closing")
            publisher = self._node.create_publisher(
                msg_type,
                topic,
                qos_profile=qos_profile,
            )
            self._owned_endpoints.append((publisher, "publisher", False))
            return publisher

    def close(self) -> None:
        """Close all publishers created by this output module.

        Lazy reward/action publishers are in the same registry as the
        constructor publishers, so a close racing the first update cannot
        leave an endpoint behind. Failed handles remain retryable.
        """
        with self._lifecycle_lock:
            if self._closed:
                return
            self._close_requested = True
            errors: list[BaseException] = []
            for index, (endpoint, kind, complete) in enumerate(self._owned_endpoints):
                if complete:
                    continue
                try:
                    destroy_ros_endpoint(self._node, endpoint, kind=kind)
                except (
                    AttributeError,
                    OSError,
                    RuntimeError,
                    TypeError,
                    ValueError,
                ) as exc:
                    errors.append(exc)
                else:
                    self._owned_endpoints[index] = (endpoint, kind, True)
            if errors:
                raise RosEndpointCleanupError(
                    "ROS step output endpoint cleanup incomplete",
                    errors=tuple(errors),
                )
            self._closed = True


__all__ = ["RosStepOutput"]
