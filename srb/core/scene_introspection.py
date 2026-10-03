from __future__ import annotations

from collections.abc import Iterator, Mapping, MutableMapping
from typing import Any

_SCENE_ENTITY_STORAGE_ATTRS = (
    "articulations",
    "_articulations",
    "rigid_objects",
    "_rigid_objects",
    "rigid_object_collections",
    "_rigid_object_collections",
    "sensors",
    "_sensors",
    "extras",
    "_extras",
)


def iter_scene_articulations(scene: object) -> Iterator[tuple[str, Any]]:
    """Iterate articulations registered on an Isaac interactive scene."""
    yield from _scene_items(scene, "articulations", "_articulations")


def get_scene_articulation(scene: object, name: str) -> Any:
    """Return one articulation by scene name."""
    return _scene_item(scene, name, "articulations", "_articulations")


def find_scene_articulation(scene: object, name: str) -> Any | None:
    """Return one articulation by scene name if present."""
    return _find_scene_item(scene, name, "articulations", "_articulations")


def iter_scene_rigid_objects(scene: object) -> Iterator[tuple[str, Any]]:
    """Iterate rigid objects registered on an Isaac interactive scene."""
    yield from _scene_items(scene, "rigid_objects", "_rigid_objects")


def get_scene_rigid_object(scene: object, name: str) -> Any:
    """Return one rigid object by scene name."""
    return _scene_item(scene, name, "rigid_objects", "_rigid_objects")


def find_scene_rigid_object(scene: object, name: str) -> Any | None:
    """Return one rigid object by scene name if present."""
    return _find_scene_item(scene, name, "rigid_objects", "_rigid_objects")


def iter_scene_rigid_object_collections(scene: object) -> Iterator[tuple[str, Any]]:
    """Iterate rigid object collections registered on an Isaac interactive scene."""
    yield from _scene_items(
        scene, "rigid_object_collections", "_rigid_object_collections"
    )


def get_scene_rigid_object_collection(scene: object, name: str) -> Any:
    """Return one rigid object collection by scene name."""
    return _scene_item(
        scene, name, "rigid_object_collections", "_rigid_object_collections"
    )


def iter_scene_sensors(scene: object) -> Iterator[tuple[str, Any]]:
    """Iterate sensors registered on an Isaac interactive scene."""
    yield from _scene_items(scene, "sensors", "_sensors")


def get_scene_sensor(scene: object, name: str) -> Any:
    """Return one sensor by scene name."""
    return _scene_item(scene, name, "sensors", "_sensors")


def find_scene_sensor(scene: object, name: str) -> Any | None:
    """Return one sensor by scene name if present."""
    return _find_scene_item(scene, name, "sensors", "_sensors")


def iter_scene_root_assets(scene: object) -> Iterator[tuple[str, Any]]:
    """Iterate root assets with world poses."""
    yield from _scene_items(
        scene,
        "rigid_objects",
        "_rigid_objects",
        "articulations",
        "_articulations",
    )


def get_scene_root_asset(scene: object, name: str) -> Any:
    """Return a rigid object or articulation by scene name."""
    return _scene_item(
        scene,
        name,
        "rigid_objects",
        "_rigid_objects",
        "articulations",
        "_articulations",
    )


def find_scene_root_asset(scene: object, name: str) -> Any | None:
    """Return a rigid object or articulation by scene name if present."""
    asset = find_scene_rigid_object(scene, name)
    if asset is not None:
        return asset
    return find_scene_articulation(scene, name)


def get_scene_entity(scene: object, name: str) -> Any:
    """Return any registered scene entity by name."""
    missing = object()
    entity = _find_scene_entity(scene, name, missing=missing)
    if entity is missing:
        raise KeyError(name)
    return entity


def find_scene_entity(scene: object, name: str) -> Any | None:
    """Return any registered scene entity by name if present."""
    missing = object()
    entity = _find_scene_entity(scene, name, missing=missing)
    return None if entity is missing else entity


def scene_sensor_pose_w(sensor: object) -> tuple[Any, Any] | None:
    """Return a sensor world pose as ``(position, quaternion)`` when available."""
    data = getattr(sensor, "data", None)
    if data is None:
        return None

    pos = getattr(data, "pos_w", None)
    if pos is not None:
        quat = getattr(data, "quat_w_ros", None)
        if quat is None:
            quat = getattr(data, "quat_w", None)
        if quat is not None:
            return pos, quat

    # Isaac Lab IMU data currently exposes only private pose buffers.
    pos = getattr(data, "_pos_w", None)
    quat = getattr(data, "_quat_w", None)
    if pos is not None and quat is not None:
        return pos, quat
    return None


def _as_xyz(value: Any) -> tuple[float, float, float] | None:
    """Coerce a 3-vector (tuple, NumPy row, torch row) into plain floats."""
    if value is None:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError, IndexError, KeyError):
        return None


def _scene_asset_origin(
    scene: object,
    *,
    asset_name: str | None,
    body_name: str | None,
    env_index: int,
) -> tuple[float, float, float] | None:
    """World position an ``asset_root`` / ``asset_body`` viewer is relative to."""
    if not asset_name:
        return None
    asset = find_scene_root_asset(scene, asset_name)
    data = getattr(asset, "data", None)
    if data is None:
        return None

    if body_name:
        body_names = getattr(asset, "body_names", None) or []
        try:
            body_index = list(body_names).index(body_name)
        except ValueError:
            body_index = None
        body_pos = getattr(data, "body_pos_w", None)
        if body_index is not None and body_pos is not None:
            try:
                return _as_xyz(body_pos[env_index][body_index])
            except (IndexError, KeyError, TypeError):
                return None
        return None

    root_pos = getattr(data, "root_pos_w", None)
    if root_pos is None:
        return None
    try:
        return _as_xyz(root_pos[env_index])
    except (IndexError, KeyError, TypeError):
        return None


def viewer_origin_w(env: object, viewer: object) -> tuple[float, float, float]:
    """World position that a ``ViewerCfg``'s ``eye``/``lookat`` are relative to.

    ``eye`` and ``lookat`` are offsets from the origin selected by
    ``origin_type``; ``"world"`` makes them absolute. Unresolvable origins
    fall back to world origin.
    """
    origin_type = getattr(viewer, "origin_type", None)
    if origin_type in (None, "world"):
        return (0.0, 0.0, 0.0)

    try:
        env_index = int(getattr(viewer, "env_index", 0) or 0)
    except (TypeError, ValueError):
        env_index = 0

    scene = getattr(getattr(env, "unwrapped", env), "scene", None)
    if origin_type == "env":
        env_origins = getattr(scene, "env_origins", None)
        if env_origins is None:
            return (0.0, 0.0, 0.0)
        try:
            return _as_xyz(env_origins[env_index]) or (0.0, 0.0, 0.0)
        except (IndexError, KeyError, TypeError):
            return (0.0, 0.0, 0.0)

    if origin_type in ("asset_root", "asset_body"):
        body_name = getattr(viewer, "body_name", None)
        if origin_type == "asset_body" and not body_name:
            return (0.0, 0.0, 0.0)
        origin = _scene_asset_origin(
            scene,
            asset_name=getattr(viewer, "asset_name", None),
            body_name=body_name if origin_type == "asset_body" else None,
            env_index=env_index,
        )
        if origin is not None:
            return origin

    return (0.0, 0.0, 0.0)


def viewer_pose_w(
    env: object,
    viewer: object,
    *,
    default_eye: tuple[float, float, float] = (7.5, 7.5, 7.5),
    default_lookat: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Resolve a ``ViewerCfg`` into an absolute ``(eye, lookat)`` world pose.

    See :func:`viewer_origin_w` for what ``origin_type`` changes. Missing or
    malformed vectors fall back to the supplied defaults so that a capture
    still produces a framed image rather than raising.
    """
    eye = _as_xyz(getattr(viewer, "eye", None)) or default_eye
    lookat = _as_xyz(getattr(viewer, "lookat", None)) or default_lookat
    origin = viewer_origin_w(env, viewer)
    return (
        (eye[0] + origin[0], eye[1] + origin[1], eye[2] + origin[2]),
        (lookat[0] + origin[0], lookat[1] + origin[1], lookat[2] + origin[2]),
    )


def replace_scene_extra(scene: object, name: str, value: Any) -> bool:
    """Replace an extra scene asset if the scene exposes that mutable storage."""
    for storage in _optional_scene_mappings(scene, "extras", "_extras"):
        if name not in storage:
            continue
        if not isinstance(storage, MutableMapping):
            raise TypeError(
                "Expected scene extras storage to be mutable, got "
                f"{type(storage).__name__}"
            )
        storage[name] = value
        return True
    return False


def _scene_item(scene: object, name: str, *attrs: str) -> Any:
    for storage in _scene_mappings(scene, *attrs):
        if name in storage:
            return storage[name]
    raise KeyError(name)


def _find_scene_item(scene: object, name: str, *attrs: str) -> Any | None:
    for storage in _optional_scene_mappings(scene, *attrs):
        if name in storage:
            return storage[name]
    return None


def _find_scene_entity(scene: object, name: str, *, missing: object) -> Any:
    for storage in _optional_scene_mappings(scene, *_SCENE_ENTITY_STORAGE_ATTRS):
        if name in storage:
            return _as_scene_view(scene, name, storage[name])
    try:
        return _as_scene_view(scene, name, scene[name])  # type: ignore[index]
    except (AttributeError, KeyError, TypeError):
        return missing


def _as_scene_view(scene: object, name: str, entity: Any) -> Any:
    """Wrap an Isaac Lab 3 ``extras`` config in a prim view, once per scene.

    Isaac Lab 3 keeps static assets (lights, skydomes, props) in
    ``InteractiveScene.extras`` as their ``AssetBaseCfg`` (``v3.0.0-EA``) or
    as a view-less ``isaaclab.assets.asset.Asset`` (``release/3.0.0`` since
    ``4d6241e6``); Isaac Lab 2 kept an
    ``XFormPrim`` view there and every SRB event term drives ``prims`` and
    ``set_world_poses`` on it. The view is cached on the scene object so the
    prims are resolved once.
    """
    from srb.core.static_asset_view import XformPrimView, static_asset_prim_path

    prim_path = static_asset_prim_path(entity)
    if prim_path is None:
        return entity
    cache = getattr(scene, "_srb_static_asset_views", None)
    if cache is None:
        cache = {}
        try:
            setattr(scene, "_srb_static_asset_views", cache)
        except (AttributeError, TypeError):
            pass
    view = cache.get(name)
    if view is None or view.prim_path != prim_path:
        view = XformPrimView(
            prim_path,
            device=str(getattr(scene, "device", "cpu")),
            name=name,
        )
        cache[name] = view
    return view


def _scene_items(scene: object, *attrs: str) -> Iterator[tuple[str, Any]]:
    seen: set[str] = set()
    for storage in _scene_mappings(scene, *attrs):
        for name, value in storage.items():
            if name in seen:
                continue
            seen.add(name)
            yield name, value


def _scene_mappings(scene: object, *attrs: str) -> Iterator[Mapping[str, Any]]:
    yielded = False
    for storage in _optional_scene_mappings(scene, *attrs):
        yielded = True
        yield storage
    if not yielded:
        raise AttributeError(
            f"Scene does not expose any of the expected storages: {attrs}"
        )


def _optional_scene_mappings(scene: object, *attrs: str) -> Iterator[Mapping[str, Any]]:
    missing = object()
    yielded = False
    for attr in attrs:
        storage = getattr(scene, attr, missing)
        if storage is missing or storage is None:
            continue
        if not isinstance(storage, Mapping):
            raise TypeError(
                f"Expected scene.{attr} to be a mapping, got {type(storage).__name__}"
            )
        yielded = True
        yield storage
    if not yielded and isinstance(scene, Mapping):
        yield scene
