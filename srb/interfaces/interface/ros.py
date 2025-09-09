try:
    import rclpy
except ImportError:
    from srb.utils.ros import enable_ros2_bridge

    enable_ros2_bridge()

import threading
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, Dict, List, Sequence, Tuple, Type

import numpy
import rclpy
import torch
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Twist, Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.service import Service
from rclpy.subscription import Subscription
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import Imu as ImuMsg
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs.msg import PointField as ROSPointField
from std_msgs.msg import Bool as BoolMsg
from std_msgs.msg import (
    Float32,
    Float32MultiArray,
    Header,
    MultiArrayDimension,
    MultiArrayLayout,
)
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from srb.core.action import (
    ActionGroup,
    ActionTerm,
    ActionTermCfg,
    BinaryJointPositionAction,
    BinaryJointVelocityAction,
    BodyAccelerationAction,
    DifferentialInverseKinematicsAction,
    MulticopterBodyAccelerationAction,
    NonHolonomicAction,
    OperationalSpaceControllerAction,
    WheeledDriveAction,
)
from srb.core.asset import Articulation, RigidObject, RigidObjectCollection
from srb.core.manager import SimulationManager
from srb.core.sensor import Camera, Imu, RayCaster, RayCasterCamera
from srb.utils.camera import create_pointcloud_from_depth, create_pointcloud_from_rgbd
from srb.utils.math import subtract_frame_transforms

from .base import InterfaceBase

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class RosInterface(InterfaceBase):
    __QOS_PROFILE = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    __QOS_PROFILE_TF = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self, env: "AnyEnv", node: Node | None = None, *args, **kwargs):
        self._env: "AnyEnv" = env.unwrapped  # type: ignore
        self._num_envs = self._env.num_envs

        ## Initialize node
        if not node:
            rclpy.init()
            self._node = Node("sim", namespace="srb", start_parameter_services=False)  # type: ignore
        else:
            self._node = node

        ## Keep track of whether a first update has been called
        self._update_called = False

        ## Initialize queue for async executing of callbacks
        self._async_exec_queue: Dict[Callable, Dict[str, Any]] = {}

        ## Initialize action buffer
        assert self._env.action_space.shape is not None
        self._actions = torch.zeros(
            self._env.action_space.shape, dtype=torch.float32, device=self._env.device
        )

        ## Setup ROS interfaces
        # Pub: Clock
        self._clock_pub: Publisher = self._node.create_publisher(
            msg_type=Clock,
            topic="/clock",
            qos_profile=self.__QOS_PROFILE,
        )

        # Pub: Reward
        self._pub_reward: Sequence[Publisher] = tuple(
            self._node.create_publisher(
                Float32,
                f"env{i}/reward",
                qos_profile=self.__QOS_PROFILE,
            )
            for i in range(self._num_envs)
        )
        # Pub: Reward term
        self._pub_reward_term: Dict[str, Sequence[Publisher]] = {}

        # Pub: Terminated
        self._pub_terminated: Sequence[Publisher] = tuple(
            self._node.create_publisher(
                BoolMsg,
                f"env{i}/terminated",
                qos_profile=self.__QOS_PROFILE,
            )
            for i in range(self._num_envs)
        )
        # Pub: Truncated
        self._pub_truncated: Sequence[Publisher] = tuple(
            self._node.create_publisher(
                BoolMsg,
                f"env{i}/truncated",
                qos_profile=self.__QOS_PROFILE,
            )
            for i in range(self._num_envs)
        )

        # Srv: Reset all
        self._srv_reset_all: Service = self._node.create_service(
            EmptySrv, "envs/reset", self._create_cb_reset()
        )
        # Srv: Reset individual
        self._srv_reset: Sequence[Service] = tuple(
            self._node.create_service(
                EmptySrv, f"env{i}/reset", self._create_cb_reset(i)
            )
            for i in range(self._num_envs)
        )

        ## Pub: Transforms (all scene assets)
        self._tf_broadcaster = TransformBroadcaster(
            self._node, qos=self.__QOS_PROFILE_TF
        )
        self._tf_broadcaster_static = StaticTransformBroadcaster(
            self._node, qos=self.__QOS_PROFILE_TF
        )
        self._broadcast_transforms_static()

        ## Pub: Joint states (all scene articulations)
        self._pub_joint_states: Dict[str, Sequence[Publisher]] = {
            articulation_name: tuple(
                self._node.create_publisher(
                    JointState,
                    f"env{i}/{articulation_name}/joint_states",
                    qos_profile=self.__QOS_PROFILE,
                )
                for i in range(self._num_envs)
            )
            for articulation_name in self._env.scene._articulations.keys()
        }

        ## Pub: Sensors (all scene sensors)
        self._setup_sensor_interfaces()

        ## Spin up ROS executor
        if not node:
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin)
            self._thread.daemon = True
            self._thread.start()

    def __del__(self):
        if hasattr(self, "_executor"):
            self._executor.shutdown()
            self._thread.join()
            self._node.destroy_node()

    @property
    def action(self) -> torch.Tensor:
        return self._actions

    def update(
        self,
        observation: Dict[str, torch.Tensor],
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: Dict[str, Any],
        action: torch.Tensor | None = None,
        *args,
        **kwargs,
    ):
        ## Perform delayed setup on first update
        if not self._update_called:
            self._update_called = True
            # Pub: Reward term
            if "reward_terms" in info:
                self._pub_reward_term = {
                    reward_term: tuple(
                        self._node.create_publisher(
                            Float32,
                            f"env{i}/reward/{reward_term}",
                            qos_profile=self.__QOS_PROFILE,
                        )
                        for i in range(self._num_envs)
                    )
                    for reward_term in info["reward_terms"]
                }

            # Pub: Actions (only if action subscription is not set up)
            if not hasattr(self, "_sub_action_term") and action is not None:
                self._pub_action = tuple(
                    self._node.create_publisher(
                        Float32MultiArray,
                        f"env{i}/action",
                        qos_profile=self.__QOS_PROFILE,
                    )
                    for i in range(self._num_envs)
                )

        ## Update clock
        time_s = self._env.common_step_counter * self._env.step_dt
        time_msg = Time(sec=int(time_s), nanosec=int((time_s % 1) * 1e9))
        self._clock_pub.publish(Clock(clock=time_msg))

        ## Publish per-env messages from the input
        for i in range(self._num_envs):
            self._pub_reward[i].publish(Float32(data=reward[i].item()))
            for reward_term, pubs in self._pub_reward_term.items():
                pubs[i].publish(
                    Float32(data=info["reward_terms"][reward_term][i].item())
                )
            self._pub_terminated[i].publish(BoolMsg(data=terminated[i].item()))
            self._pub_truncated[i].publish(BoolMsg(data=truncated[i].item()))

        ## Publish actions
        if action is not None and hasattr(self, "_pub_action"):
            layout = MultiArrayLayout(
                dim=[
                    MultiArrayDimension(
                        label="action",
                        size=action.shape[1],
                        stride=4 * action.shape[1],
                    )
                ],
                data_offset=0,
            )
            for i in range(self._num_envs):
                self._pub_action[i].publish(
                    Float32MultiArray(layout=layout, data=action[i].tolist())
                )

        ## Broadcast transforms (all scene assets)
        self._broadcast_transforms(time_msg)

        ## Publish joint states (all scene articulations)
        for articulation_name, pubs in self._pub_joint_states.items():
            articulation: Articulation = self._env.scene._articulations[
                articulation_name
            ]
            for i in range(self._num_envs):
                pubs[i].publish(
                    JointState(
                        header=Header(
                            stamp=time_msg,
                            frame_id=f"srb/env{i}/{articulation_name}",
                        ),
                        name=articulation.data.joint_names,
                        position=articulation.data.joint_pos[i],
                        velocity=articulation.data.joint_vel[i],
                        effort=articulation.data.applied_torque[i],
                    )
                )

        ## Publish sensor data (all scene sensors)
        # self._publish_sensor_data(time_msg)

        ## Process async requests
        for request, kwargs in self._async_exec_queue.items():
            request(**kwargs)
        self._async_exec_queue.clear()

    ## Action ##

    def setup_action_sub(self):
        ## Action terms
        self._sub_action_term_all: Dict[str, Subscription] = {}
        self._sub_action_term: Dict[str, Sequence[Subscription]] = {}
        action_offset = 0
        for action_key, action_term in self._env.action_manager._terms.items():
            assert isinstance(action_term, ActionTerm)
            assert isinstance(action_term.cfg, ActionTermCfg)
            topic_base = (
                f"{action_term.cfg.asset_name}/{action_key.rsplit('__', 1)[-1]}"
            )
            msg_type, msg_extractor = self._map_action_term(action_term)
            action_dim = action_term.action_dim
            assert (
                msg_type is Float32MultiArray
                or len(msg_extractor(msg_type())) == action_dim
            )

            self._sub_action_term_all[action_key] = self._node.create_subscription(
                msg_type=msg_type,
                topic=f"envs/{topic_base}",
                callback=self._create_cb_action_any(
                    dim=action_dim,
                    key=action_key,
                    extractor=msg_extractor,
                    offset=action_offset,
                ),
                qos_profile=self.__QOS_PROFILE,
            )
            self._sub_action_term[action_key] = tuple(
                self._node.create_subscription(
                    msg_type=msg_type,
                    topic=f"env{i}/{topic_base}",
                    callback=self._create_cb_action_any(
                        key=action_key,
                        extractor=msg_extractor,
                        dim=action_dim,
                        offset=action_offset,
                        env_id=i,
                    ),
                    qos_profile=self.__QOS_PROFILE,
                )
                for i in range(self._num_envs)
            )

            action_offset += action_dim

        ## Standard actions
        if hasattr(self._env.cfg, "actions") and isinstance(
            self._env.cfg.actions, ActionGroup
        ):
            try:
                self._env.cfg.actions.map_cmd_to_action(torch.zeros(6), False)
                env_supports_direct_teleop = True
            except NotImplementedError:
                env_supports_direct_teleop = False
        else:
            env_supports_direct_teleop = False
        if env_supports_direct_teleop:
            self._actions_cmd_vel = torch.zeros(
                (self._num_envs, 6), dtype=torch.float32, device=self._env.device
            )
            self._actions_event = torch.zeros(
                (self._num_envs, 1), dtype=torch.float32, device=self._env.device
            )
            self._sub_action_cmd_vel_all: Subscription = self._node.create_subscription(
                Twist,
                "envs/action/cmd_vel",
                callback=self._create_cb_action_cmd_vel(),
                qos_profile=self.__QOS_PROFILE,
            )
            self._sub_action_cmd_vel: Sequence[Subscription] = tuple(
                self._node.create_subscription(
                    Twist,
                    f"env{i}/action/cmd_vel",
                    callback=self._create_cb_action_cmd_vel(env_id=i),
                    qos_profile=self.__QOS_PROFILE,
                )
                for i in range(self._num_envs)
            )
            self._sub_action_event_all: Subscription = self._node.create_subscription(
                BoolMsg,
                "envs/action/event",
                callback=self._create_cb_action_event(),
                qos_profile=self.__QOS_PROFILE,
            )
            self._sub_action_event: Sequence[Subscription] = tuple(
                self._node.create_subscription(
                    BoolMsg,
                    f"env{i}/action/event",
                    callback=self._create_cb_action_event(env_id=i),
                    qos_profile=self.__QOS_PROFILE,
                )
                for i in range(self._num_envs)
            )

    def _create_cb_action_any(
        self,
        key: str,
        extractor: Callable,
        dim: int,
        offset: int,
        env_id: int | None = None,
    ) -> Callable:
        start_idx = offset
        end_idx = offset + dim

        if env_id is not None:

            def _proto_cb(self, msg: Any):
                self._actions[env_id, start_idx:end_idx] = torch.tensor(
                    extractor(msg), dtype=torch.float32, device=self._env.device
                )

        else:

            def _proto_cb(self, msg: Any):
                self._actions[:, start_idx:end_idx] = torch.tensor(
                    extractor(msg), dtype=torch.float32, device=self._env.device
                )

        cb_name = f"__cb_action_{key}{env_id or ''}"
        setattr(self, cb_name, _proto_cb.__get__(self, self.__class__))
        return getattr(self, cb_name)

    @staticmethod
    def _map_action_term(action_term: ActionTerm) -> Tuple[Type, Callable]:
        if isinstance(
            action_term,
            (
                BodyAccelerationAction,
                DifferentialInverseKinematicsAction,
                OperationalSpaceControllerAction,
            ),
        ):
            return Twist, lambda msg: [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ]

        if isinstance(action_term, (NonHolonomicAction, WheeledDriveAction)):
            return Twist, lambda msg: [msg.linear.x, msg.angular.z]

        if isinstance(action_term, MulticopterBodyAccelerationAction):
            return Twist, lambda msg: [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z,
            ]

        if isinstance(
            action_term, (BinaryJointPositionAction, BinaryJointVelocityAction)
        ):
            return BoolMsg, lambda msg: [-1.0 if msg.data else 1.0]

        return Float32MultiArray, lambda msg: list(msg.data)

    def _create_cb_action_cmd_vel(self, env_id: int | None = None) -> Callable:
        if env_id is not None:

            def _proto_cb(self, msg: Twist):
                cmd_vel = torch.tensor(
                    [
                        msg.linear.x,
                        msg.linear.y,
                        msg.linear.z,
                        msg.angular.x,
                        msg.angular.y,
                        msg.angular.z,
                    ],
                    dtype=torch.float32,
                    device=self._env.device,
                )
                self._actions_cmd_vel[env_id] = cmd_vel
                self._actions[env_id] = self._env.cfg.actions.map_cmd_to_action(
                    cmd_vel, self._actions_event[env_id].bool()
                )

        else:

            def _proto_cb(self, msg: Twist):
                cmd_vel = torch.tensor(
                    [
                        msg.linear.x,
                        msg.linear.y,
                        msg.linear.z,
                        msg.angular.x,
                        msg.angular.y,
                        msg.angular.z,
                    ],
                    dtype=torch.float32,
                    device=self._env.device,
                )
                self._actions_cmd_vel = cmd_vel.unsqueeze(0).repeat(self._num_envs, 1)
                for i in range(self._num_envs):
                    self._actions[i] = self._env.cfg.actions.map_cmd_to_action(
                        cmd_vel, self._actions_event[i].bool()
                    )

        cb_name = f"__cb_action_cmd_vel{env_id or ''}"
        setattr(self, cb_name, _proto_cb.__get__(self, self.__class__))
        return getattr(self, cb_name)

    def _create_cb_action_event(self, env_id: int | None = None) -> Callable:
        if env_id is not None:

            def _proto_cb(self, msg: BoolMsg):
                event = msg.data
                self._actions_event[env_id] = event
                self._actions[env_id] = self._env.cfg.actions.map_cmd_to_action(
                    self._actions_cmd_vel[env_id], event
                )

        else:

            def _proto_cb(self, msg: BoolMsg):
                event = msg.data
                self._actions_event = (
                    torch.tensor([event], dtype=torch.float32, device=self._env.device)
                    .unsqueeze(0)
                    .repeat(self._num_envs, 1)
                )
                for i in range(self._num_envs):
                    self._actions[i] = self._env.cfg.actions.map_cmd_to_action(
                        self._actions_cmd_vel[i], event
                    )

        cb_name = f"__cb_action_event{env_id or ''}"
        setattr(self, cb_name, _proto_cb.__get__(self, self.__class__))
        return getattr(self, cb_name)

    ## Sensors ##

    def _setup_sensor_interfaces(self):
        self._pub_sensors: Dict[str, Dict[str, Sequence[Publisher]]] = {}

        for sensor_name, sensor in self._env.scene._sensors.items():
            self._pub_sensors[sensor_name] = {}

            if isinstance(sensor, (Camera, RayCasterCamera)):
                self._pub_sensors[sensor_name]["camera_info"] = tuple(
                    self._node.create_publisher(
                        CameraInfo,
                        f"env{i}/{sensor_name}/camera_info",
                        qos_profile=self.__QOS_PROFILE,
                    )
                    for i in range(self._num_envs)
                )

                has_depth_streams = False
                for data_type in sensor.cfg.data_types:
                    if data_type in (
                        "depth",
                        "distance_to_camera",
                        "distance_to_image_plane",
                    ):
                        has_depth_streams = True
                    image_name = (
                        "image_depth"
                        if data_type == "distance_to_image_plane"
                        else f"image_{data_type}"
                    )
                    self._pub_sensors[sensor_name][image_name] = tuple(
                        self._node.create_publisher(
                            Image,
                            f"env{i}/{sensor_name}/{image_name}",
                            qos_profile=self.__QOS_PROFILE,
                        )
                        for i in range(self._num_envs)
                    )

                if has_depth_streams:
                    self._pub_sensors[sensor_name]["pointcloud"] = tuple(
                        self._node.create_publisher(
                            PointCloud2,
                            f"env{i}/{sensor_name}/pointcloud",
                            qos_profile=self.__QOS_PROFILE,
                        )
                        for i in range(self._num_envs)
                    )

            elif isinstance(sensor, RayCaster):
                self._pub_sensors[sensor_name]["pointcloud"] = tuple(
                    self._node.create_publisher(
                        PointCloud2,
                        f"env{i}/{sensor_name}/pointcloud",
                        qos_profile=self.__QOS_PROFILE,
                    )
                    for i in range(self._num_envs)
                )

            elif isinstance(sensor, Imu):
                self._pub_sensors[sensor_name]["imu"] = tuple(
                    self._node.create_publisher(
                        ImuMsg,
                        f"env{i}/{sensor_name}",
                        qos_profile=self.__QOS_PROFILE,
                    )
                    for i in range(self._num_envs)
                )

    def _publish_sensor_data(self, time_msg: Time):
        for sensor_name, publishers in self._pub_sensors.items():
            sensor = self._env.scene._sensors[sensor_name]

            ## Cameras
            if isinstance(sensor, (Camera, RayCasterCamera)):
                # Camera info
                for i in range(self._num_envs):
                    camera_info_msg = CameraInfo()
                    camera_info_msg.header = Header(
                        stamp=time_msg, frame_id=f"srb/env{i}/{sensor_name}"
                    )
                    camera_info_msg.height = sensor.image_shape[0]
                    camera_info_msg.width = sensor.image_shape[1]
                    camera_info_msg.distortion_model = "plumb_bob"
                    camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
                    intrinsic_matrix = sensor.data.intrinsic_matrices[i]
                    camera_info_msg.k = [
                        intrinsic_matrix[0, 0].item(),
                        intrinsic_matrix[0, 1].item(),
                        intrinsic_matrix[0, 2].item(),
                        intrinsic_matrix[1, 0].item(),
                        intrinsic_matrix[1, 1].item(),
                        intrinsic_matrix[1, 2].item(),
                        intrinsic_matrix[2, 0].item(),
                        intrinsic_matrix[2, 1].item(),
                        intrinsic_matrix[2, 2].item(),
                    ]
                    publishers["camera_info"][i].publish(camera_info_msg)

                # Images
                for data_type in sensor.data.output.keys():
                    image_name = (
                        "image_depth"
                        if data_type == "distance_to_image_plane"
                        else f"image_{data_type}"
                    )
                    img_data_all = sensor.data.output[data_type].cpu().numpy()
                    for i in range(self._num_envs):
                        img_data = img_data_all[i]

                        # Create image message
                        image_msg = Image()
                        image_msg.header = Header(
                            stamp=time_msg, frame_id=f"srb/env{i}/{sensor_name}"
                        )
                        image_msg.height = img_data.shape[0]
                        image_msg.width = img_data.shape[1]

                        # Handle different image types
                        if data_type == "rgb" or img_data.shape[2] == 3:
                            image_msg.encoding = "rgb8"
                            image_msg.is_bigendian = False
                            image_msg.step = 3 * img_data.shape[1]
                            if img_data.dtype != numpy.uint8:
                                img_data = (255.0 * img_data).astype(numpy.uint8)
                            image_msg.data = img_data.tobytes()

                        elif data_type == "rgba" or img_data.shape[2] == 4:
                            image_msg.encoding = "rgba8"
                            image_msg.is_bigendian = False
                            image_msg.step = 4 * img_data.shape[1]
                            if img_data.dtype != numpy.uint8:
                                img_data = (255.0 * img_data).astype(numpy.uint8)
                            image_msg.data = img_data.tobytes()

                        elif data_type in (
                            "depth",
                            "distance_to_camera",
                            "distance_to_image_plane",
                        ):
                            image_msg.encoding = "32FC1"
                            image_msg.is_bigendian = False
                            image_msg.step = 4 * img_data.shape[1]
                            image_msg.data = img_data.astype(numpy.float32).tobytes()

                        elif img_data.shape[2] == 1:
                            image_msg.encoding = "mono8"
                            image_msg.is_bigendian = False
                            image_msg.step = img_data.shape[1]
                            if img_data.dtype != numpy.uint8:
                                img_data = (255.0 * img_data).astype(numpy.uint8)
                            image_msg.data = img_data.tobytes()

                        else:
                            # Unsupported format
                            break

                        publishers[image_name][i].publish(image_msg)

                # Pointcloud
                depth_type = None
                for depth_option in (
                    "depth",
                    "distance_to_image_plane",
                    "distance_to_camera",
                ):
                    if depth_option in sensor.data.output:
                        depth_type = depth_option
                        break
                if depth_type:
                    depth_data = sensor.data.output[depth_type]

                    if "rgb" in sensor.data.output.keys():
                        rgb_data = sensor.data.output["rgb"].cpu().numpy()
                    elif "rgba" in sensor.data.output.keys():
                        rgb_data = sensor.data.output["rgba"][..., :3].cpu().numpy()
                    else:
                        rgb_data = None

                    for i in range(self._num_envs):
                        if rgb_data is not None:
                            points, colors = create_pointcloud_from_rgbd(
                                intrinsic_matrix=sensor.data.intrinsic_matrices[i],
                                depth=depth_data[i],
                                rgb=rgb_data[i],
                                normalize_rgb=True,
                                device=self._env.device,
                            )
                            colors = colors.cpu().numpy().astype(numpy.float32)  # type: ignore
                        else:
                            points = create_pointcloud_from_depth(
                                intrinsic_matrix=sensor.data.intrinsic_matrices[i],
                                depth=depth_data[i],
                                device=self._env.device,
                            )
                        points = points.cpu().numpy().astype(numpy.float32)  # type: ignore

                        # Create PointCloud2 message
                        pointcloud_msg = PointCloud2()
                        pointcloud_msg.header = Header(
                            stamp=time_msg, frame_id=f"srb/env{i}/{sensor_name}"
                        )
                        pointcloud_msg.height = 1
                        pointcloud_msg.width = points.shape[0]  # type: ignore

                        # Define fields
                        fields = [
                            ROSPointField(
                                name="x",
                                offset=0,
                                datatype=ROSPointField.FLOAT32,
                                count=1,
                            ),
                            ROSPointField(
                                name="y",
                                offset=4,
                                datatype=ROSPointField.FLOAT32,
                                count=1,
                            ),
                            ROSPointField(
                                name="z",
                                offset=8,
                                datatype=ROSPointField.FLOAT32,
                                count=1,
                            ),
                        ]
                        if rgb_data is not None:
                            fields.extend(
                                [
                                    ROSPointField(
                                        name="r",
                                        offset=12,
                                        datatype=ROSPointField.FLOAT32,
                                        count=1,
                                    ),
                                    ROSPointField(
                                        name="g",
                                        offset=16,
                                        datatype=ROSPointField.FLOAT32,
                                        count=1,
                                    ),
                                    ROSPointField(
                                        name="b",
                                        offset=20,
                                        datatype=ROSPointField.FLOAT32,
                                        count=1,
                                    ),
                                ]
                            )
                            pointcloud_data = numpy.hstack([points, colors])
                            point_step = 24
                        else:
                            pointcloud_data = points
                            point_step = 12

                        pointcloud_msg.fields = fields
                        pointcloud_msg.is_bigendian = False
                        pointcloud_msg.point_step = point_step
                        pointcloud_msg.row_step = (
                            pointcloud_msg.point_step * pointcloud_msg.width
                        )
                        pointcloud_msg.is_dense = True
                        pointcloud_msg.data = pointcloud_data.tobytes()

                        publishers["pointcloud"][i].publish(pointcloud_msg)

            ## RayCaster (non-camera)
            elif isinstance(sensor, RayCaster):
                ray_hits_all = (
                    sensor.data.ray_hits_w.cpu().numpy().astype(numpy.float32)
                )
                for i in range(self._num_envs):
                    ray_hits = ray_hits_all[i]

                    pointcloud_msg = PointCloud2()
                    pointcloud_msg.header = Header(
                        stamp=time_msg, frame_id=f"srb/env{i}/{sensor_name}"
                    )
                    pointcloud_msg.height = 1
                    pointcloud_msg.width = ray_hits.shape[0]

                    # Define fields
                    fields = [
                        ROSPointField(
                            name="x", offset=0, datatype=ROSPointField.FLOAT32, count=1
                        ),
                        ROSPointField(
                            name="y", offset=4, datatype=ROSPointField.FLOAT32, count=1
                        ),
                        ROSPointField(
                            name="z", offset=8, datatype=ROSPointField.FLOAT32, count=1
                        ),
                    ]

                    pointcloud_msg.fields = fields
                    pointcloud_msg.is_bigendian = False
                    pointcloud_msg.point_step = 12
                    pointcloud_msg.row_step = (
                        pointcloud_msg.point_step * pointcloud_msg.width
                    )
                    pointcloud_msg.is_dense = True
                    pointcloud_msg.data = ray_hits.tobytes()

                    publishers["pointcloud"][i].publish(pointcloud_msg)

            # IMU
            elif isinstance(sensor, Imu):
                lin_acc = sensor.data.lin_acc_b
                ang_vel = sensor.data.ang_vel_b
                for i in range(self._num_envs):
                    imu_msg = ImuMsg()
                    imu_msg.header = Header(
                        stamp=time_msg, frame_id=f"srb/env{i}/{sensor_name}"
                    )
                    imu_msg.linear_acceleration.x = lin_acc[i, 0].item()
                    imu_msg.linear_acceleration.y = lin_acc[i, 1].item()
                    imu_msg.linear_acceleration.z = lin_acc[i, 2].item()
                    imu_msg.angular_velocity.x = ang_vel[i, 0].item()
                    imu_msg.angular_velocity.y = ang_vel[i, 1].item()
                    imu_msg.angular_velocity.z = ang_vel[i, 2].item()

                    publishers["imu"][i].publish(imu_msg)

    ## Transforms ##

    def _broadcast_transforms(self, time_msg: Time):
        transforms: List[TransformStamped] = []
        for asset_name, asset in (
            self._env.scene._rigid_objects.items()
            | self._env.scene._articulations.items()
        ):
            assert isinstance(asset, (RigidObject, Articulation))
            root_pos = asset.data.root_pos_w - self._env.scene.env_origins
            transforms.extend(
                [
                    TransformStamped(
                        header=Header(
                            stamp=time_msg,
                            frame_id=f"srb/env{i}",
                        ),
                        child_frame_id=f"srb/env{i}/{asset_name}",
                        transform=Transform(
                            translation=Vector3(
                                x=root_pos[i, 0].item(),
                                y=root_pos[i, 1].item(),
                                z=root_pos[i, 2].item(),
                            ),
                            rotation=Quaternion(
                                w=asset.data.root_quat_w[i, 0].item(),
                                x=asset.data.root_quat_w[i, 1].item(),
                                y=asset.data.root_quat_w[i, 2].item(),
                                z=asset.data.root_quat_w[i, 3].item(),
                            ),
                        ),
                    )
                    for i in range(self._num_envs)
                ]
            )
        for asset_name, asset in self._env.scene._sensors.items():
            if isinstance(asset, (Camera, RayCasterCamera)):
                root_pos = asset.data.pos_w - self._env.scene.env_origins
                root_quat = asset.data.quat_w_ros
            elif isinstance(asset, (RayCaster, Imu)):
                root_pos = asset.data.pos_w - self._env.scene.env_origins
                root_quat = asset.data.quat_w
            else:
                continue

            transforms.extend(
                [
                    TransformStamped(
                        header=Header(
                            stamp=time_msg,
                            frame_id=f"srb/env{i}",
                        ),
                        child_frame_id=f"srb/env{i}/{asset_name}",
                        transform=Transform(
                            translation=Vector3(
                                x=root_pos[i, 0].item(),
                                y=root_pos[i, 1].item(),
                                z=root_pos[i, 2].item(),
                            ),
                            rotation=Quaternion(
                                w=root_quat[i, 0].item(),  # type: ignore
                                x=root_quat[i, 1].item(),  # type: ignore
                                y=root_quat[i, 2].item(),  # type: ignore
                                z=root_quat[i, 3].item(),  # type: ignore
                            ),
                        ),
                    )
                    for i in range(self._num_envs)
                ]
            )

        for asset_name, asset in self._env.scene._rigid_object_collections.items():
            assert isinstance(asset, RigidObjectCollection)
            object_pos = (
                asset.data.object_pos_w
                - self._env.scene.env_origins.unsqueeze(1).repeat(
                    1, asset.num_objects, 1
                )
            )
            transforms.extend(
                [
                    TransformStamped(
                        header=Header(
                            stamp=time_msg,
                            frame_id=f"srb/env{i}",
                        ),
                        child_frame_id=f"srb/env{i}/{object_name}",
                        transform=Transform(
                            translation=Vector3(
                                x=object_pos[i, object_id, 0].item(),
                                y=object_pos[i, object_id, 1].item(),
                                z=object_pos[i, object_id, 2].item(),
                            ),
                            rotation=Quaternion(
                                w=asset.data.object_quat_w[i, object_id, 0].item(),
                                x=asset.data.object_quat_w[i, object_id, 1].item(),
                                y=asset.data.object_quat_w[i, object_id, 2].item(),
                                z=asset.data.object_quat_w[i, object_id, 3].item(),
                            ),
                        ),
                    )
                    for i in range(self._num_envs)
                    for object_id, object_name in enumerate(asset.object_names)
                ]
            )
        for asset_name, asset in self._env.scene._articulations.items():
            assert isinstance(asset, Articulation)
            body_pos, body_quat = subtract_frame_transforms(
                asset.data.root_pos_w.unsqueeze(1).repeat(1, asset.num_bodies, 1),
                asset.data.root_quat_w.unsqueeze(1).repeat(1, asset.num_bodies, 1),
                asset.data.body_pos_w,
                asset.data.body_quat_w,
            )
            transforms.extend(
                [
                    TransformStamped(
                        header=Header(
                            stamp=time_msg,
                            frame_id=f"srb/env{i}/{asset_name}",
                        ),
                        child_frame_id=f"srb/env{i}/{asset_name}/{body_name}",
                        transform=Transform(
                            translation=Vector3(
                                x=body_pos[i, body_id, 0].item(),
                                y=body_pos[i, body_id, 1].item(),
                                z=body_pos[i, body_id, 2].item(),
                            ),
                            rotation=Quaternion(
                                w=body_quat[i, body_id, 0].item(),
                                x=body_quat[i, body_id, 1].item(),
                                y=body_quat[i, body_id, 2].item(),
                                z=body_quat[i, body_id, 3].item(),
                            ),
                        ),
                    )
                    for i in range(self._num_envs)
                    for (
                        body_name,
                        body_id,
                    ) in asset.root_physx_view.shared_metatype.link_indices.items()
                ]
            )
        self._tf_broadcaster.sendTransform(transforms)

    def _broadcast_transforms_static(self):
        static_transforms = [
            # World -> Map (alias for simplicity)
            TransformStamped(
                header=Header(
                    stamp=Time(sec=0, nanosec=0),
                    frame_id="world",
                ),
                child_frame_id="map",
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
                ),
            ),
            # World -> SRB World
            TransformStamped(
                header=Header(
                    stamp=Time(sec=0, nanosec=0),
                    frame_id="world",
                ),
                child_frame_id="srb/world",
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
                ),
            ),
            # SRB World -> Envs
            *(
                TransformStamped(
                    header=Header(
                        stamp=Time(sec=0, nanosec=0),
                        frame_id="srb/world",
                    ),
                    child_frame_id=f"srb/env{i}",
                    transform=Transform(
                        translation=Vector3(
                            x=self._env.scene.env_origins[i, 0].item(),
                            y=self._env.scene.env_origins[i, 1].item(),
                            z=self._env.scene.env_origins[i, 2].item(),
                        ),
                        rotation=Quaternion(
                            w=1.0,
                            x=0.0,
                            y=0.0,
                            z=0.0,
                        ),
                    ),
                )
                for i in range(self._num_envs)
            ),
        ]
        self._tf_broadcaster_static.sendTransform(static_transforms)

    ## Reset ##

    def _create_cb_reset(self, env_id: int | None = None) -> Callable:
        if env_id is not None:

            def _proto_cb(self, request: EmptySrv.Request, response: EmptySrv.Response):
                if self._async_reset in self._async_exec_queue:
                    if "env_ids" in self._async_exec_queue[self._async_reset]:
                        self._async_exec_queue[self._async_reset]["env_ids"].add(env_id)
                else:
                    self._async_exec_queue[self._async_reset] = {"env_ids": {env_id}}

                return response

        else:

            def _proto_cb(self, request: EmptySrv.Request, response: EmptySrv.Response):
                self._async_exec_queue[self._async_reset] = {}
                return response

        cb_name = f"__cb_reset{env_id or ''}"
        setattr(self, cb_name, _proto_cb.__get__(self, self.__class__))
        return getattr(self, cb_name)

    def _async_reset(self, env_ids: Sequence[int] | None = None):
        if env_ids:
            ids = torch.tensor(list(env_ids), dtype=torch.int, device=self._env.device)
            self._actions[ids].zero_()
            self._env._reset_idx(ids)  # type: ignore

            self._env.scene.write_data_to_sim()
            self._env.sim.forward()

            if self._env.sim.has_rtx_sensors():
                if self._env.cfg.rerender_on_reset:
                    self._env.sim.render()
                if self._env.cfg.wait_for_textures:
                    while SimulationManager.assets_loading():
                        self._env.sim.render()
        else:
            self._actions.zero_()
            self._env.reset()
