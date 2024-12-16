import json
import threading
from collections.abc import Callable
from queue import Queue
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np
import rclpy
import torch
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
    Wrench,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Float32, Header, String
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import TransformBroadcaster

from space_robotics_bench.core.actions import (
    ManipulatorTaskSpaceActionCfg,
    MultiCopterActionGroupCfg,
    WheeledRoverActionGroupCfg,
)
from space_robotics_bench.core.envs import BaseEnv
from space_robotics_bench.envs import (
    BaseAerialRoboticsEnv,
    BaseManipulationEnv,
    BaseMobileRoboticsEnv,
)
from space_robotics_bench.utils.string import canonicalize_str


class ROS2:
    OBSERVATION_MAPPING = {
        "image_scene_rgb": ("camera_scene/image_raw", Image),
        "image_scene_depth": ("camera_scene/depth/image_raw", Image),
        "image_front_rgb": ("robot/camera_front/image_raw", Image),
        "image_front_depth": ("robot/camera_front/depth/image_raw", Image),
        "image_bottom_rgb": ("robot/camera_bottom/image_raw", Image),
        "image_bottom_depth": ("robot/camera_bottom/depth/image_raw", Image),
        "image_base_rgb": ("robot/camera_base/image_raw", Image),
        "image_base_depth": ("robot/camera_base/depth/image_raw", Image),
        "image_wrist_rgb": ("robot/camera_wrist/image_raw", Image),
        "image_wrist_depth": ("robot/camera_wrist/depth/image_raw", Image),
    }

    def __init__(
        self,
        env: Union[
            BaseEnv,
            BaseAerialRoboticsEnv,
            BaseManipulationEnv,
            BaseMobileRoboticsEnv,
        ],
        node: Optional[object] = None,
        force_multienv: bool = True,
    ):
        self._env = env
        self._is_multi_env = force_multienv or self._env.unwrapped.num_envs > 1

        ## Initialize node
        if node is None:
            rclpy.init(args=None)
            self._node = Node("srb")
        else:
            self._node = node

        ## Execution queue for actions and services that must be executed in the main thread between environment steps via `update()`
        self._exec_queue = Queue()

        ## Clock publisher
        self._clock_pub = self._node.create_publisher(
            Time,
            "clock",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self._time = 0.0

        ## Initialize transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self._node)

        ## Setup interfaces
        self._setup_actions()
        self._setup_observation()
        self._setup_reward()
        self._setup_terminated()
        self._setup_truncated()
        self._setup_misc()

        # Run a thread for listening to device
        if node is None:
            self._thread = threading.Thread(target=rclpy.spin, args=(self._node,))
            self._thread.daemon = True
            self._thread.start()

    def __del__(self):
        if hasattr(self, "_thread"):
            self._thread.join()

    @property
    def actions(self) -> torch.Tensor:
        return torch.tensor(
            self._actions,
            dtype=torch.float32,
            device=self._env.unwrapped.unwrapped.device,
        )

    def publish(
        self,
        observation: Dict[str, torch.Tensor],
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: Dict[str, Any],
    ):
        if not self._pub_observation:
            self._setup_observation(observation)
        stamp = self._node.get_clock().now().to_msg()
        for key, value in observation.items():
            mapping = self.OBSERVATION_MAPPING.get(key)
            if mapping is None:
                continue
            if mapping[0] == "tf":
                for i in range(self._env.unwrapped.num_envs):
                    self._broadcast_tf(
                        value[i],
                        mapping,
                        stamp,
                        prefix=f"{f'env{i}/' if self._is_multi_env else ''}",
                    )
            else:
                pub = self._pub_observation[key]
                for i in range(self._env.unwrapped.num_envs):
                    pub[i].publish(self._wrap_msg(value[i], mapping, stamp=stamp))

        for i in range(self._env.unwrapped.num_envs):
            self._pub_reward[i].publish(Float32(data=float(reward[i])))
            if terminated[i]:
                self._pub_terminated[i].publish(EmptyMsg())
            if truncated[i]:
                self._pub_truncated[i].publish(EmptyMsg())

        if info:
            self._pub_info.publish(String(data=json.dumps(info)))

    def reset(self):
        self._env.reset()
        self._actions = np.zeros(
            (self._env.unwrapped.num_envs, self._env.unwrapped.cfg.num_actions)
        )

    def update(self):
        self._time += self._env.unwrapped.cfg.sim.dt
        self._clock_pub.publish(
            Time(sec=int(self._time), nanosec=int((self._time % 1) * 1e9))
        )

        while not self._exec_queue.empty():
            request, kwargs = self._exec_queue.get()
            request(**kwargs)

    def _setup_actions(self):
        self._actions = np.zeros(
            (self._env.unwrapped.num_envs, self._env.unwrapped.cfg.num_actions)
        )

        robot_name = self._env.unwrapped.cfg.robot_cfg.asset_cfg.prim_path.split("/")[
            -1
        ]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if isinstance(self._env.unwrapped.cfg.actions, ManipulatorTaskSpaceActionCfg):

            def _create_actions_cb_cmd_vel(
                cb_name: str, env_id: Optional[int] = None
            ) -> Callable:
                def cb_single(self, msg: Twist):
                    self._actions[env_id, :6] = np.array(
                        [
                            msg.linear.x,
                            msg.linear.y,
                            msg.linear.z,
                            msg.angular.x,
                            msg.angular.y,
                            msg.angular.z,
                        ]
                    )

                def cb_all(self, msg: Twist):
                    self._actions[:, :6] = np.array(
                        [
                            msg.linear.x,
                            msg.linear.y,
                            msg.linear.z,
                            msg.angular.x,
                            msg.angular.y,
                            msg.angular.z,
                        ]
                    )

                cb = cb_single if env_id else cb_all
                cb_name = f"cb_{canonicalize_str(cb_name)}{env_id or ''}"
                setattr(self, cb_name, cb.__get__(self, self.__class__))
                return getattr(self, cb_name)

            def _create_actions_cb_gripper(
                cb_name: str, env_id: Optional[int] = None
            ) -> Callable:
                def cb_single(self, msg: Bool):
                    self._actions[env_id, 6] = -1.0 if msg.data else 1.0

                def cb_all(self, msg: Bool):
                    self._actions[:, 6] = -1.0 if msg.data else 1.0

                cb = cb_single if env_id else cb_all
                cb_name = f"cb_{canonicalize_str(cb_name)}{env_id or ''}"
                setattr(self, cb_name, cb.__get__(self, self.__class__))
                return getattr(self, cb_name)

            if self._is_multi_env:
                self._sub_actions = (
                    self._node.create_subscription(
                        Twist,
                        f"envs/{robot_name}/cmd_vel",
                        _create_actions_cb_cmd_vel(cb_name="robot_cmd_vel"),
                        qos_profile,
                    ),
                    self._node.create_subscription(
                        Bool,
                        f"envs/{robot_name}/gripper",
                        _create_actions_cb_gripper(cb_name="robot_gripper"),
                        qos_profile,
                    ),
                    *(
                        self._node.create_subscription(
                            Twist,
                            f"env{i}/{robot_name}/cmd_vel",
                            _create_actions_cb_cmd_vel("robot_cmd_vel", i),
                            qos_profile,
                        )
                        for i in range(self._env.unwrapped.num_envs)
                    ),
                    *(
                        self._node.create_subscription(
                            Bool,
                            f"env{i}/{robot_name}/gripper",
                            _create_actions_cb_gripper("robot_gripper", i),
                            qos_profile,
                        )
                        for i in range(self._env.unwrapped.num_envs)
                    ),
                )
            else:
                self._sub_actions = (
                    self._node.create_subscription(
                        Twist,
                        f"{robot_name}/cmd_vel",
                        _create_actions_cb_cmd_vel(cb_name="robot_cmd_vel"),
                        qos_profile,
                    ),
                    self._node.create_subscription(
                        Bool,
                        f"{robot_name}/gripper",
                        _create_actions_cb_gripper(cb_name="robot_gripper"),
                        qos_profile,
                    ),
                )

        elif isinstance(self._env.unwrapped.cfg.actions, MultiCopterActionGroupCfg):

            def _create_actions_cb_cmd_vel(
                cb_name: str, env_id: Optional[int] = None
            ) -> Callable:
                def cb_single(self, msg: Twist):
                    self._actions[env_id] = np.array(
                        [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
                    )

                def cb_all(self, msg: Twist):
                    self._actions[:] = np.array(
                        [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
                    )

                cb = cb_single if env_id else cb_all
                cb_name = f"cb_{canonicalize_str(cb_name)}{env_id or ''}"
                setattr(self, cb_name, cb.__get__(self, self.__class__))
                return getattr(self, cb_name)

            if self._is_multi_env:
                self._sub_actions = (
                    self._node.create_subscription(
                        Twist,
                        f"envs/{robot_name}/cmd_vel",
                        _create_actions_cb_cmd_vel(cb_name="robot_cmd_vel"),
                        qos_profile,
                    ),
                    *(
                        self._node.create_subscription(
                            Twist,
                            f"env{i}/{robot_name}/cmd_vel",
                            _create_actions_cb_cmd_vel("robot_cmd_vel", i),
                            qos_profile,
                        )
                        for i in range(self._env.unwrapped.num_envs)
                    ),
                )
            else:
                self._sub_actions = self._node.create_subscription(
                    Twist,
                    f"{robot_name}/cmd_vel",
                    _create_actions_cb_cmd_vel(cb_name="robot_cmd_vel"),
                    qos_profile,
                )

        elif isinstance(self._env.unwrapped.cfg.actions, WheeledRoverActionGroupCfg):

            def _create_actions_cb_cmd_vel(
                cb_name: str, env_id: Optional[int] = None
            ) -> Callable:
                def cb_single(self, msg: Twist):
                    self._actions[env_id] = np.array([msg.linear.x, msg.angular.z])

                def cb_all(self, msg: Twist):
                    self._actions[:] = np.array([msg.linear.x, msg.angular.z])

                cb = cb_single if env_id else cb_all
                cb_name = f"cb_{canonicalize_str(cb_name)}{env_id or ''}"
                setattr(self, cb_name, cb.__get__(self, self.__class__))
                return getattr(self, cb_name)

            if self._is_multi_env:
                self._sub_actions = (
                    self._node.create_subscription(
                        Twist,
                        f"envs/{robot_name}/cmd_vel",
                        _create_actions_cb_cmd_vel(cb_name="robot_cmd_vel"),
                        qos_profile,
                    ),
                    *(
                        self._node.create_subscription(
                            Twist,
                            f"env{i}/{robot_name}/cmd_vel",
                            _create_actions_cb_cmd_vel("robot_cmd_vel", i),
                            qos_profile,
                        )
                        for i in range(self._env.unwrapped.num_envs)
                    ),
                )
            else:
                self._sub_actions = self._node.create_subscription(
                    Twist,
                    f"{robot_name}/cmd_vel",
                    _create_actions_cb_cmd_vel(cb_name="robot_cmd_vel"),
                    qos_profile,
                )

    def _setup_observation(self, observation: Optional[Dict[str, torch.Tensor]] = None):
        if observation is None:
            self._pub_observation = {}
            return

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for key in observation.keys():
            mapping = self.OBSERVATION_MAPPING.get(key)
            if mapping is None:
                continue
            if mapping[0] == "tf":
                continue
            self._pub_observation[key] = tuple(
                self._node.create_publisher(
                    mapping[1],
                    f"{f'env{i}/' if self._is_multi_env else ''}{mapping[0]}",
                    qos_profile,
                )
                for i in range(self._env.unwrapped.num_envs)
            )

    def _broadcast_tf(
        self,
        tensor: torch.Tensor,
        mapping: Tuple[str, type, str, str],
        stamp: Time,
        prefix: str = "",
    ):
        _, _, parent_frame_id, child_frame_id = mapping
        self._tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(frame_id=parent_frame_id, stamp=stamp),
                child_frame_id=f"{prefix}{child_frame_id}",
                transform=Transform(
                    translation=Vector3(
                        x=tensor[0].item(),
                        y=tensor[1].item(),
                        z=tensor[2].item(),
                    ),
                    rotation=Quaternion(
                        x=tensor[4].item(),
                        y=tensor[5].item(),
                        z=tensor[6].item(),
                        w=tensor[3].item(),
                    ),
                ),
            )
        )

    def _wrap_msg(
        self,
        tensor: torch.Tensor,
        mapping: Tuple[str, type],
        stamp: Time,
    ):
        topic_name, msg_type = mapping

        if msg_type is JointState:
            return JointState(
                header=Header(frame_id=topic_name.rsplit("/", 1)[0], stamp=stamp),
                name=self._env.unwrapped._robot.joint_names,
                position=tensor.cpu().numpy(),
            )
        elif msg_type is Wrench:
            return Wrench(
                force=Vector3(
                    x=tensor[0].item(),
                    y=tensor[1].item(),
                    z=tensor[2].item(),
                ),
                torque=Vector3(
                    x=tensor[3].item(),
                    y=tensor[4].item(),
                    z=tensor[5].item(),
                ),
            )
        elif msg_type is Image:
            return Image(
                header=Header(frame_id=topic_name.rsplit("/", 1)[0], stamp=stamp),
                width=tensor.shape[1],
                height=tensor.shape[0],
                step=tensor.shape[1] * 3,
                data=(255.0 * tensor).cpu().numpy().astype(np.uint8).tobytes(),
                is_bigendian=0,
                encoding="rgb8" if tensor.shape[2] == 3 else "mono8",
            )
        else:
            return msg_type(*tensor.cpu().numpy())

    def _setup_reward(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_reward = tuple(
            self._node.create_publisher(
                Float32, f"env{i if self._is_multi_env else ''}/reward", qos_profile
            )
            for i in range(self._env.unwrapped.num_envs)
        )

    def _setup_terminated(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_terminated = tuple(
            self._node.create_publisher(
                EmptyMsg,
                f"env{i if self._is_multi_env else ''}/terminated",
                qos_profile,
            )
            for i in range(self._env.unwrapped.num_envs)
        )

    def _setup_truncated(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_truncated = tuple(
            self._node.create_publisher(
                EmptyMsg, f"env{i if self._is_multi_env else ''}/truncated", qos_profile
            )
            for i in range(self._env.unwrapped.num_envs)
        )

    def _setup_misc(self):
        self._srv_reset = self._node.create_service(
            EmptySrv, "sim/reset", self._cb_reset
        )

        self._pub_info = self._node.create_publisher(
            String,
            f"env{'s' if self._is_multi_env else ''}/info",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    def _cb_reset(self, request: EmptySrv.Request, response: EmptySrv.Response):
        self._exec_queue.put((self.reset, {}))
        return response
