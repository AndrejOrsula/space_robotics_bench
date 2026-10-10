"""Publish static and per-step TF transforms for the simulation scene."""

from __future__ import annotations

import threading
from typing import Any

from srb.interfaces._ros_node import (
    RosCleanupScope,
    RosEndpointCleanupError,
    destroy_ros_endpoint,
)
from srb.interfaces.ros_namespace import RosNamespace


def _ros_transform_messages() -> tuple[type[Any], ...]:
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
    from std_msgs.msg import Header

    return Time, Header, Quaternion, Transform, TransformStamped, Vector3


def _tf_broadcaster_classes() -> tuple[type[Any], type[Any]]:
    from tf2_ros import TransformBroadcaster
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    return TransformBroadcaster, StaticTransformBroadcaster


def _scene_transform_accessors() -> tuple[Any, ...]:
    from srb.core.scene_introspection import (
        iter_scene_articulations,
        iter_scene_rigid_object_collections,
        iter_scene_root_assets,
        iter_scene_sensors,
        scene_sensor_pose_w,
    )

    return (
        iter_scene_articulations,
        iter_scene_rigid_object_collections,
        iter_scene_root_assets,
        iter_scene_sensors,
        scene_sensor_pose_w,
    )


def _rigid_object_collection_class() -> type[Any]:
    from srb.core.asset import RigidObjectCollection

    return RigidObjectCollection


def _subtract_frame_transforms() -> Any:
    from srb.utils.math import subtract_frame_transforms

    return subtract_frame_transforms


class RosTransformPublisher:
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

        with RosCleanupScope(
            self,
            "ROS transform publisher construction cleanup incomplete",
        ):
            (
                self._Time,
                self._Header,
                self._Quaternion,
                self._Transform,
                self._TransformStamped,
                self._Vector3,
            ) = _ros_transform_messages()
            TransformBroadcaster, StaticTransformBroadcaster = _tf_broadcaster_classes()
            (
                self._iter_scene_articulations,
                self._iter_scene_rigid_object_collections,
                self._iter_scene_root_assets,
                self._iter_scene_sensors,
                self._scene_sensor_pose_w,
            ) = _scene_transform_accessors()
            self._RigidObjectCollection = _rigid_object_collection_class()
            self._subtract_frame_transforms = _subtract_frame_transforms()

            # Use latched QoS only for /tf_static; /tf keeps tf2's volatile
            # history so late joiners do not receive a stale single transform.
            self._tf_broadcaster = TransformBroadcaster(self._node)
            self._owned_endpoints.append((self._tf_broadcaster, "broadcaster", False))
            self._tf_broadcaster_static = StaticTransformBroadcaster(
                self._node,
                qos=self._qos_profile,
            )
            self._owned_endpoints.append(
                (self._tf_broadcaster_static, "broadcaster", False)
            )
            self.publish_static()

    def publish(self, time_msg: Any) -> None:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS transform publisher is closed")
            transforms: list[Any] = []
            transforms.extend(self._root_asset_transforms(time_msg))
            transforms.extend(self._sensor_transforms(time_msg))
            transforms.extend(self._rigid_object_collection_transforms(time_msg))
            transforms.extend(self._articulation_body_transforms(time_msg))
            self._tf_broadcaster.sendTransform(transforms)

    def publish_static(self) -> None:
        with self._lifecycle_lock:
            if self._close_requested:
                raise RuntimeError("ROS transform publisher is closed")
            static_transforms = [
                # World -> Map (alias for simplicity)
                self._TransformStamped(
                    header=self._Header(
                        stamp=self._Time(sec=0, nanosec=0),
                        frame_id="world",
                    ),
                    child_frame_id="map",
                    transform=self._identity_transform(),
                ),
                # World -> SRB World
                self._TransformStamped(
                    header=self._Header(
                        stamp=self._Time(sec=0, nanosec=0),
                        frame_id="world",
                    ),
                    child_frame_id="srb/world",
                    transform=self._identity_transform(),
                ),
                # SRB World -> Envs
                *(
                    self._TransformStamped(
                        header=self._Header(
                            stamp=self._Time(sec=0, nanosec=0),
                            frame_id="srb/world",
                        ),
                        child_frame_id=self._namespace.env_frame(i),
                        transform=self._Transform(
                            translation=self._Vector3(
                                x=self._env.scene.env_origins[i, 0].item(),
                                y=self._env.scene.env_origins[i, 1].item(),
                                z=self._env.scene.env_origins[i, 2].item(),
                            ),
                            rotation=self._Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                        ),
                    )
                    for i in range(self._num_envs)
                ),
            ]
            self._tf_broadcaster_static.sendTransform(static_transforms)

    def close(self) -> None:
        """Close the dynamic and static broadcasters created by this module."""
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
                    "ROS transform endpoint cleanup incomplete",
                    errors=tuple(errors),
                )
            self._closed = True

    def _root_asset_transforms(self, time_msg: Any) -> list[Any]:
        transforms: list[Any] = []
        for asset_name, asset in self._iter_scene_root_assets(self._env.scene):
            # Transfer each tensor once, not once per element and environment.
            pos_np = (asset.data.root_pos_w - self._env.scene.env_origins).cpu().numpy()
            quat_np = asset.data.root_quat_w.cpu().numpy()
            transforms.extend(
                self._env_transform(
                    time_msg=time_msg,
                    frame_id=self._namespace.env_frame(i),
                    child_frame_id=self._namespace.env_frame(i, asset_name),
                    translation=pos_np[i],
                    rotation=quat_np[i],
                )
                for i in range(self._num_envs)
            )
        return transforms

    def _sensor_transforms(self, time_msg: Any) -> list[Any]:
        transforms: list[Any] = []
        for asset_name, asset in self._iter_scene_sensors(self._env.scene):
            sensor_pose = self._scene_sensor_pose_w(asset)
            if sensor_pose is None:
                continue

            root_pos, root_quat = sensor_pose
            pos_np = (root_pos - self._env.scene.env_origins).cpu().numpy()
            quat_np = root_quat.cpu().numpy()
            transforms.extend(
                self._env_transform(
                    time_msg=time_msg,
                    frame_id=self._namespace.env_frame(i),
                    child_frame_id=self._namespace.env_frame(i, asset_name),
                    translation=pos_np[i],
                    rotation=quat_np[i],
                )
                for i in range(self._num_envs)
            )
        return transforms

    def _rigid_object_collection_transforms(self, time_msg: Any) -> list[Any]:
        transforms: list[Any] = []
        for asset_name, asset in self._iter_scene_rigid_object_collections(
            self._env.scene
        ):
            assert isinstance(asset, self._RigidObjectCollection)
            object_pos = (
                asset.data.object_pos_w
                - self._env.scene.env_origins.unsqueeze(1).repeat(
                    1,
                    asset.num_objects,
                    1,
                )
            )
            obj_pos_np = object_pos.cpu().numpy()
            obj_quat_np = asset.data.object_quat_w.cpu().numpy()
            transforms.extend(
                self._env_transform(
                    time_msg=time_msg,
                    frame_id=self._namespace.env_frame(i),
                    child_frame_id=self._namespace.env_frame(
                        i,
                        f"{asset_name}/{object_name}",
                    ),
                    translation=obj_pos_np[i, object_id],
                    rotation=obj_quat_np[i, object_id],
                )
                for i in range(self._num_envs)
                for object_id, object_name in enumerate(asset.object_names)
            )
        return transforms

    def _articulation_body_transforms(self, time_msg: Any) -> list[Any]:
        transforms: list[Any] = []
        for asset_name, asset in self._iter_scene_articulations(self._env.scene):
            body_pos, body_quat = self._subtract_frame_transforms(
                asset.data.root_pos_w.unsqueeze(1).repeat(1, asset.num_bodies, 1),
                asset.data.root_quat_w.unsqueeze(1).repeat(1, asset.num_bodies, 1),
                asset.data.body_pos_w,
                asset.data.body_quat_w,
            )
            # Transfer all body poses once per tensor.
            body_pos_np = body_pos.cpu().numpy()
            body_quat_np = body_quat.cpu().numpy()
            # Materialize once; otherwise each environment re-walks PhysX metadata.
            link_indices = tuple(
                asset.root_physx_view.shared_metatype.link_indices.items()
            )
            transforms.extend(
                self._env_transform(
                    time_msg=time_msg,
                    frame_id=self._namespace.env_frame(i, asset_name),
                    child_frame_id=self._namespace.env_frame(
                        i,
                        f"{asset_name}/{body_name}",
                    ),
                    translation=body_pos_np[i, body_id],
                    rotation=body_quat_np[i, body_id],
                )
                for i in range(self._num_envs)
                for body_name, body_id in link_indices
            )
        return transforms

    def _env_transform(
        self,
        *,
        time_msg: Any,
        frame_id: str,
        child_frame_id: str,
        translation: Any,
        rotation: Any,
    ) -> Any:
        return self._TransformStamped(
            header=self._Header(stamp=time_msg, frame_id=frame_id),
            child_frame_id=child_frame_id,
            transform=self._Transform(
                translation=self._Vector3(
                    x=float(translation[0]),
                    y=float(translation[1]),
                    z=float(translation[2]),
                ),
                rotation=self._Quaternion(
                    x=float(rotation[0]),
                    y=float(rotation[1]),
                    z=float(rotation[2]),
                    w=float(rotation[3]),
                ),
            ),
        )

    def _identity_transform(self) -> Any:
        return self._Transform(
            translation=self._Vector3(x=0.0, y=0.0, z=0.0),
            rotation=self._Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )


__all__ = ["RosTransformPublisher"]
