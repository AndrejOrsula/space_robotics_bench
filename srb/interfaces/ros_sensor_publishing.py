"""Publish camera, ray-caster, and IMU sensor data to ROS."""

from __future__ import annotations

import threading
from collections.abc import Sequence
from typing import Any

import numpy as np

from srb.interfaces._ros_node import (
    RosCleanupScope,
    RosEndpointCleanupError,
    destroy_ros_endpoint,
)
from srb.interfaces.ros_namespace import RosNamespace


def _ros_sensor_messages() -> tuple[type[Any], ...]:
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs.msg import Imu as ImuMsg
    from sensor_msgs.msg import PointField as ROSPointField
    from std_msgs.msg import Header

    return CameraInfo, Image, PointCloud2, ImuMsg, ROSPointField, Header


def _scene_sensor_accessors() -> tuple[Any, Any]:
    from srb.core.scene_introspection import get_scene_sensor, iter_scene_sensors

    return get_scene_sensor, iter_scene_sensors


def _sensor_classes() -> tuple[type[Any], type[Any], type[Any], type[Any]]:
    from srb.core.sensor import Camera, Imu, RayCaster, RayCasterCamera

    return Camera, Imu, RayCaster, RayCasterCamera


def _visual_helpers() -> tuple[Any, ...]:
    from srb.core.sensor.visual_codec import ros_image_payload
    from srb.core.sensor.visual_output import (
        has_visual_depth_stream,
        select_visual_depth_data_type,
        select_visual_rgb_output,
        visual_stream_name,
    )

    return (
        ros_image_payload,
        has_visual_depth_stream,
        select_visual_depth_data_type,
        select_visual_rgb_output,
        visual_stream_name,
    )


def _pointcloud_helpers() -> tuple[Any, Any, Any]:
    from srb.utils.camera import (
        create_pointcloud_from_depth,
        create_pointcloud_from_rgbd,
    )
    from srb.utils.math import orthogonalize_perspective_depth

    return (
        create_pointcloud_from_depth,
        create_pointcloud_from_rgbd,
        orthogonalize_perspective_depth,
    )


def build_pointcloud_fields(point_field_cls: type[Any], *, with_rgb: bool) -> list[Any]:
    fields = [
        point_field_cls(
            name="x",
            offset=0,
            datatype=point_field_cls.FLOAT32,
            count=1,
        ),
        point_field_cls(
            name="y",
            offset=4,
            datatype=point_field_cls.FLOAT32,
            count=1,
        ),
        point_field_cls(
            name="z",
            offset=8,
            datatype=point_field_cls.FLOAT32,
            count=1,
        ),
    ]
    if with_rgb:
        fields.extend(
            [
                point_field_cls(
                    name="r",
                    offset=12,
                    datatype=point_field_cls.FLOAT32,
                    count=1,
                ),
                point_field_cls(
                    name="g",
                    offset=16,
                    datatype=point_field_cls.FLOAT32,
                    count=1,
                ),
                point_field_cls(
                    name="b",
                    offset=20,
                    datatype=point_field_cls.FLOAT32,
                    count=1,
                ),
            ]
        )
    return fields


def pointcloud_payload(
    points: np.ndarray,
    colors: np.ndarray | None = None,
) -> tuple[np.ndarray, int]:
    if colors is None:
        return points, 12
    return np.hstack([points, colors]), 24


class RosSensorPublisher:
    def __init__(
        self,
        *,
        env: Any,
        node: Any,
        namespace: RosNamespace,
        qos_profile: Any,
    ) -> None:
        self._env = env
        self._node = node
        self._num_envs = int(env.num_envs)
        self._namespace = namespace
        self._qos_profile = qos_profile

        self._lifecycle_lock = threading.RLock()
        self._close_requested = False
        self._closed = False
        self._owned_endpoints: list[tuple[Any, str, bool]] = []
        self._pub_sensors: dict[str, dict[str, Sequence[Any]]] = {}
        self._camera_publishers: dict[str, dict[str, Sequence[Any]]] = {}
        self._pointcloud_publishers: dict[str, Sequence[Any]] = {}

        with RosCleanupScope(
            self,
            "ROS sensor publisher construction cleanup incomplete",
        ):
            (
                self._CameraInfo,
                self._Image,
                self._PointCloud2,
                self._ImuMsg,
                self._ROSPointField,
                self._Header,
            ) = _ros_sensor_messages()
            self._get_scene_sensor, self._iter_scene_sensors = _scene_sensor_accessors()
            Camera, Imu, RayCaster, RayCasterCamera = _sensor_classes()
            self._camera_sensor_classes = (Camera, RayCasterCamera)
            self._RayCaster = RayCaster
            self._Imu = Imu
            (
                self._ros_image_payload,
                self._has_visual_depth_stream,
                self._select_visual_depth_data_type,
                self._select_visual_rgb_output,
                self._visual_stream_name,
            ) = _visual_helpers()
            (
                self._create_pointcloud_from_depth,
                self._create_pointcloud_from_rgbd,
                self._orthogonalize_perspective_depth,
            ) = _pointcloud_helpers()
            self._setup_publishers()

    def publish(self, time_msg: Any) -> None:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS sensor publisher is closed")
            for sensor_name, publishers in self._pub_sensors.items():
                sensor = self._get_scene_sensor(self._env.scene, sensor_name)

                if isinstance(sensor, self._camera_sensor_classes):
                    self._publish_camera(time_msg, sensor_name, sensor, publishers)
                elif isinstance(sensor, self._RayCaster):
                    self._publish_raycaster(time_msg, sensor_name, sensor)
                elif isinstance(sensor, self._Imu):
                    self._publish_imu(time_msg, sensor_name, sensor, publishers)

    def _setup_publishers(self) -> None:
        for sensor_name, sensor in self._iter_scene_sensors(self._env.scene):
            self._pub_sensors[sensor_name] = {}

            if isinstance(sensor, self._camera_sensor_classes):
                self._pub_sensors[sensor_name]["camera_info"] = (
                    self._create_env_publishers(
                        self._CameraInfo,
                        f"{sensor_name}/camera_info",
                    )
                )

                for data_type in sensor.cfg.data_types:
                    image_name = self._visual_stream_name(data_type)

                    if sensor_name not in self._camera_publishers:
                        self._camera_publishers[sensor_name] = {}
                    # `depth` and `distance_to_image_plane` share one topic;
                    # a second publisher would silently replace the first.
                    if image_name in self._camera_publishers[sensor_name]:
                        raise ValueError(
                            f"Camera {sensor_name!r} data types resolve to the same "
                            f"ROS topic {image_name!r}; configure only one of them"
                        )
                    self._camera_publishers[sensor_name][image_name] = (
                        self._create_env_publishers(
                            self._Image,
                            f"{sensor_name}/{image_name}",
                        )
                    )

                if self._has_visual_depth_stream(sensor.cfg.data_types):
                    self._pointcloud_publishers[sensor_name] = (
                        self._create_env_publishers(
                            self._PointCloud2,
                            f"{sensor_name}/pointcloud",
                        )
                    )

            elif isinstance(sensor, self._RayCaster):
                self._pointcloud_publishers[sensor_name] = self._create_env_publishers(
                    self._PointCloud2,
                    f"{sensor_name}/pointcloud",
                )

            elif isinstance(sensor, self._Imu):
                self._pub_sensors[sensor_name]["imu"] = self._create_env_publishers(
                    self._ImuMsg,
                    sensor_name,
                )

    def _create_env_publishers(
        self,
        msg_type: type[Any],
        topic_suffix: str,
    ) -> tuple[Any, ...]:
        publishers = []
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS sensor publisher is closing")
            for i in range(self._num_envs):
                publisher = self._node.create_publisher(
                    msg_type,
                    self._namespace.env_topic(i, topic_suffix),
                    qos_profile=self._qos_profile,
                )
                self._owned_endpoints.append((publisher, "publisher", False))
                publishers.append(publisher)
        return tuple(publishers)

    def close(self) -> None:
        """Close every sensor publisher created by this module."""
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
                    "ROS sensor endpoint cleanup incomplete",
                    errors=tuple(errors),
                )
            self._closed = True

    def _header(self, time_msg: Any, env_id: int, sensor_name: str) -> Any:
        return self._Header(
            stamp=time_msg,
            frame_id=self._namespace.env_frame(env_id, sensor_name),
        )

    def _publish_camera(
        self,
        time_msg: Any,
        sensor_name: str,
        sensor: Any,
        publishers: dict[str, Sequence[Any]],
    ) -> None:
        self._publish_camera_info(time_msg, sensor_name, sensor, publishers)
        self._publish_camera_images(time_msg, sensor_name, sensor)
        self._publish_camera_pointcloud(time_msg, sensor_name, sensor)

    def _publish_camera_info(
        self,
        time_msg: Any,
        sensor_name: str,
        sensor: Any,
        publishers: dict[str, Sequence[Any]],
    ) -> None:
        intrinsic_np = sensor.data.intrinsic_matrices.cpu().numpy()
        for i in range(self._num_envs):
            camera_info_msg = self._CameraInfo()
            camera_info_msg.header = self._header(time_msg, i, sensor_name)
            camera_info_msg.height = sensor.image_shape[0]
            camera_info_msg.width = sensor.image_shape[1]
            camera_info_msg.distortion_model = "plumb_bob"
            camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            k = intrinsic_np[i]
            camera_info_msg.k = [
                float(k[0, 0]),
                float(k[0, 1]),
                float(k[0, 2]),
                float(k[1, 0]),
                float(k[1, 1]),
                float(k[1, 2]),
                float(k[2, 0]),
                float(k[2, 1]),
                float(k[2, 2]),
            ]
            # Undistorted camera: identity rectification and projection matching K.
            camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info_msg.p = [
                float(k[0, 0]),
                float(k[0, 1]),
                float(k[0, 2]),
                0.0,
                float(k[1, 0]),
                float(k[1, 1]),
                float(k[1, 2]),
                0.0,
                float(k[2, 0]),
                float(k[2, 1]),
                float(k[2, 2]),
                0.0,
            ]
            publishers["camera_info"][i].publish(camera_info_msg)

    def _publish_camera_images(
        self,
        time_msg: Any,
        sensor_name: str,
        sensor: Any,
    ) -> None:
        stream_publishers_by_name = self._camera_publishers.get(sensor_name, {})
        for data_type in sensor.data.output.keys():
            image_name = self._visual_stream_name(data_type)
            stream_publishers = stream_publishers_by_name.get(image_name)
            if stream_publishers is None:
                # Ignore annotator streams without configured publishers (for example,
                # renderer RGB-to-RGBA aliases).
                continue
            img_data_all = sensor.data.output[data_type].cpu().numpy()
            for i in range(self._num_envs):
                img_data = img_data_all[i]
                payload = self._ros_image_payload(data_type, img_data)
                if payload is None:
                    continue

                image_msg = self._Image()
                image_msg.header = self._header(time_msg, i, sensor_name)
                image_msg.height = img_data.shape[0]
                image_msg.width = img_data.shape[1]
                image_msg.encoding = payload.encoding
                image_msg.is_bigendian = False
                image_msg.step = payload.step
                image_msg.data = payload.data

                stream_publishers[i].publish(image_msg)

    def _publish_camera_pointcloud(
        self,
        time_msg: Any,
        sensor_name: str,
        sensor: Any,
    ) -> None:
        depth_type = self._select_visual_depth_data_type(sensor.data.output)
        if not depth_type:
            return

        depth_data = sensor.data.output[depth_type]
        if depth_type == "distance_to_camera":
            # Unprojection expects image-plane depth; this stream is radial.
            depth_data = self._orthogonalize_perspective_depth(
                depth_data, sensor.data.intrinsic_matrices
            )
        rgb_output = self._select_visual_rgb_output(sensor.data.output)
        rgb_data = rgb_output.cpu().numpy() if rgb_output is not None else None
        # The PointField layout is loop-invariant; build it once per publish.
        pointcloud_fields = build_pointcloud_fields(
            self._ROSPointField,
            with_rgb=rgb_data is not None,
        )

        for i in range(self._num_envs):
            colors = None
            if rgb_data is not None:
                points, colors = self._create_pointcloud_from_rgbd(
                    intrinsic_matrix=sensor.data.intrinsic_matrices[i],
                    depth=depth_data[i],
                    rgb=rgb_data[i],
                    normalize_rgb=True,
                    device=self._env.device,
                )
                colors = colors.cpu().numpy().astype(np.float32)  # type: ignore[misc]
            else:
                points = self._create_pointcloud_from_depth(
                    intrinsic_matrix=sensor.data.intrinsic_matrices[i],
                    depth=depth_data[i],
                    device=self._env.device,
                )
            points = points.cpu().numpy().astype(np.float32)  # type: ignore[misc]
            pointcloud_data, point_step = pointcloud_payload(points, colors)

            pointcloud_msg = self._PointCloud2()
            pointcloud_msg.header = self._header(time_msg, i, sensor_name)
            pointcloud_msg.height = 1
            pointcloud_msg.width = points.shape[0]  # type: ignore[misc]
            pointcloud_msg.fields = pointcloud_fields
            pointcloud_msg.is_bigendian = False
            pointcloud_msg.point_step = point_step
            pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
            # Preserve dense holds; depth conversion otherwise strips invalid points.
            pointcloud_msg.is_dense = True
            pointcloud_msg.data = pointcloud_data.tobytes()

            self._pointcloud_publishers[sensor_name][i].publish(pointcloud_msg)

    def _publish_raycaster(
        self,
        time_msg: Any,
        sensor_name: str,
        sensor: Any,
    ) -> None:
        # HeightmapSensor keeps raw ray hits in ``ray_data``; other raycasters use
        # ``data``.
        data = getattr(sensor, "ray_data", None)
        if data is None:
            data = sensor.data
        ray_hits_w = getattr(data, "ray_hits_w", None)
        if ray_hits_w is None:
            return
        # Isaac Sim 6 ProxyArray stores its tensor in ``.torch``; tensors pass through.
        ray_hits_w = getattr(ray_hits_w, "torch", ray_hits_w)
        # Express world-frame hits relative to the env origin and stamp that frame;
        # TF consumers must not apply the sensor pose again.
        ray_hits_all = (
            (ray_hits_w - self._env.scene.env_origins.unsqueeze(1))
            .cpu()
            .numpy()
            .astype(np.float32)
        )
        # The PointField layout is loop-invariant; build it once per publish.
        pointcloud_fields = build_pointcloud_fields(
            self._ROSPointField,
            with_rgb=False,
        )
        for i in range(self._num_envs):
            ray_hits = ray_hits_all[i]

            pointcloud_msg = self._PointCloud2()
            pointcloud_msg.header = self._Header(
                stamp=time_msg,
                frame_id=self._namespace.env_frame(i),
            )
            pointcloud_msg.height = 1
            pointcloud_msg.width = ray_hits.shape[0]
            pointcloud_msg.fields = pointcloud_fields
            pointcloud_msg.is_bigendian = False
            pointcloud_msg.point_step = 12
            pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
            # Missed rays are non-finite; report unknown density per PointCloud2.
            pointcloud_msg.is_dense = False
            pointcloud_msg.data = ray_hits.tobytes()

            self._pointcloud_publishers[sensor_name][i].publish(pointcloud_msg)

    def _publish_imu(
        self,
        time_msg: Any,
        sensor_name: str,
        sensor: Any,
        publishers: dict[str, Sequence[Any]],
    ) -> None:
        lin_acc_np = sensor.data.lin_acc_b.cpu().numpy()
        ang_vel_np = sensor.data.ang_vel_b.cpu().numpy()
        for i in range(self._num_envs):
            imu_msg = self._ImuMsg()
            imu_msg.header = self._header(time_msg, i, sensor_name)
            # No orientation estimate: covariance[0] = -1 marks it unavailable.
            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.linear_acceleration.x = float(lin_acc_np[i, 0])
            imu_msg.linear_acceleration.y = float(lin_acc_np[i, 1])
            imu_msg.linear_acceleration.z = float(lin_acc_np[i, 2])
            imu_msg.angular_velocity.x = float(ang_vel_np[i, 0])
            imu_msg.angular_velocity.y = float(ang_vel_np[i, 1])
            imu_msg.angular_velocity.z = float(ang_vel_np[i, 2])

            publishers["imu"][i].publish(imu_msg)


__all__ = [
    "RosSensorPublisher",
    "build_pointcloud_fields",
    "pointcloud_payload",
]
