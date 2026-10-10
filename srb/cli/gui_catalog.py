"""Build the Rust GUI task catalog from caches and docs sidecars."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

import orjson as json

from srb.cli.offline import flatten_cache_names
from srb.utils.path import SRB_DIR, SRB_GRAPHICS_DIR, SRB_GUI_CATALOG_CACHE_PATH
from srb.utils.publication import ArtifactPublication, PublicationError

if TYPE_CHECKING:
    from srb.utils.cache import OfflineCacheSnapshot

_DEFAULT_WORKFLOW = "teleop"
_DEFAULT_RENDERING_MODE = "quality"
_DEFAULT_TELEOP_DEVICES = ("keyboard", "spacemouse")
_DEFAULT_INTERFACES = ("gui",)
_SRB_FALLBACK_IMAGE = SRB_DIR / "docs" / "theme" / "favicon.png"
_RENDERING_MODES = ("performance", "balanced", "quality", "xr")
_DOMAIN_FALLBACK_ASSETS = {
    "ASTEROID": ("asteroid",),
    "EARTH": ("ground_plane",),
    "MARS": ("mars_surface", "mars_rock"),
    "MOON": ("moon_surface", "moon_rock"),
    "ORBIT": ("gateway", "iss"),
}
_OPTIONAL_DEFAULT_KEYS = (
    "scenery",
    "robot",
    "manipulator",
    "mobile_base",
    "end_effector",
    "payload",
)
_DEFAULT_GUI_CATALOG_CACHE_PATH = SRB_GUI_CATALOG_CACHE_PATH
# Leading element the offline enum cache adds to container-typed keys for the
# shell completions (see `srb.cli.offline_cache_build`).
_CONTAINER_KIND_MARKERS = frozenset({"__set__", "__list__", "__dict__"})


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    ArtifactPublication.file(path).publish_bytes(payload)


def _unique_strings(values: Sequence[Any], *, sort: bool = False) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        if not isinstance(value, str) or not value:
            continue
        if value in seen:
            continue
        seen.add(value)
        result.append(value)
    if sort:
        result.sort()
    return result


def _read_json(path: Path, default: Any) -> Any:
    try:
        data = ArtifactPublication.file(path).read(json.loads)
        if data is None:
            return default
        if (
            isinstance(data, dict)
            and "__srb_cache__" in data
            and "payload" in data
            and isinstance(data["__srb_cache__"], dict)
        ):
            return data.get("payload", default)
        return data
    except (OSError, PublicationError, json.JSONDecodeError, TypeError):
        return default


def _display_name(task_id: str) -> str:
    return task_id.strip("_").replace("_", " ").title()


def _task_info_index(
    task_infos_by_category: Mapping[str, Sequence[Any]],
) -> dict[str, tuple[str, Any]]:
    result: dict[str, tuple[str, Any]] = {}
    for category, infos in task_infos_by_category.items():
        for info in infos:
            name = getattr(info, "name", None)
            if isinstance(name, str):
                result[name] = (str(category), info)
    return result


def _task_infos_or_empty(
    introspect_tasks: Callable[[], Mapping[str, Sequence[Any]]],
) -> Mapping[str, Sequence[Any]]:
    try:
        return introspect_tasks()
    except Exception as exc:  # noqa: BLE001 - optional introspection fallback
        from srb.utils import logging

        logging.warning(
            f"Cannot introspect task metadata for Rust GUI catalog; using cache-only metadata: {exc}"
        )
        return {}


def _relative(path: Path) -> str:
    path = Path(path)
    try:
        return path.resolve().relative_to(SRB_DIR.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def _image_path(path: Path) -> str:
    return _relative(path) if path.is_file() else ""


def _first_existing_asset_image(graphics_dir: Path, names: Sequence[str]) -> str:
    for name in names:
        image = _image_path(graphics_dir / "assets" / f"{name}.webp")
        if image:
            return image
    return ""


def _thumbnail_status(
    *,
    graphics_dir: Path,
    task_id: str,
    defaults: Mapping[str, Any],
    robot: str | None,
    scenery: str | None,
) -> dict[str, Any]:
    task_thumbnail = _image_path(graphics_dir / "tasks" / f"{task_id}.webp")
    robot_thumbnail = (
        _image_path(graphics_dir / "assets" / f"{robot}.webp") if robot else ""
    )
    scenery_thumbnail = (
        _image_path(graphics_dir / "assets" / f"{scenery}.webp") if scenery else ""
    )
    domain_thumbnail = _first_existing_asset_image(
        graphics_dir,
        _DOMAIN_FALLBACK_ASSETS.get(_default_domain(defaults), ()),
    )

    fallback_thumbnail = ""
    fallback_source = "placeholder"
    for thumbnail, source in (
        (robot_thumbnail, "robot"),
        (scenery_thumbnail, "scenery"),
        (domain_thumbnail, "domain"),
        (_relative(_SRB_FALLBACK_IMAGE), "placeholder"),
    ):
        if thumbnail:
            fallback_thumbnail = thumbnail
            fallback_source = source
            break

    display_thumbnail = task_thumbnail or fallback_thumbnail
    display_source = "task" if task_thumbnail else fallback_source

    return {
        "task": task_thumbnail,
        "robot": robot_thumbnail,
        "fallback": fallback_thumbnail,
        "display": display_thumbnail,
        "display_source": display_source,
        "missing_thumbnail": not bool(task_thumbnail),
    }


def _default_domain(defaults: Mapping[str, Any]) -> str:
    domain = defaults.get("domain")
    return domain.upper() if isinstance(domain, str) and domain else "MOON"


def _default_stack(defaults: Mapping[str, Any]) -> bool:
    stack = defaults.get("stack")
    return stack if isinstance(stack, bool) else False


def _gui_meta(meta: Any) -> dict[str, Any]:
    gui = getattr(meta, "gui", None)
    return {
        "featured": getattr(gui, "featured", False),
        "rank": getattr(gui, "rank", 1000),
        "theme": getattr(gui, "theme", "dark"),
    }


def _sanitize_optional_defaults(defaults: Mapping[str, Any]) -> dict[str, str | None]:
    return {
        key: value if isinstance(value := defaults.get(key), str) else None
        for key in _OPTIONAL_DEFAULT_KEYS
    }


def _enum_overrides(
    task_id: str,
    enum_values: Mapping[str, Any],
) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for values_by_key in (
        enum_values.get("__common__", {}),
        enum_values.get(task_id, {}),
    ):
        if not isinstance(values_by_key, Mapping):
            continue
        for key, values in values_by_key.items():
            if (
                isinstance(key, str)
                and key.startswith("env.")
                and isinstance(values, list)
            ):
                if values and values[0] in _CONTAINER_KIND_MARKERS:
                    # A set/list/dict-typed key: the GUI launches every enum
                    # override as a scalar `key=<choice>`, so neither the
                    # completion marker nor a bare member is a valid value.
                    # The key stays in `overrides.keys` for free-form entry.
                    result.pop(key, None)
                    continue
                result[key] = [str(value) for value in values]
    return result


def _config_keys(task_id: str, config_keys: Mapping[str, Any]) -> list[str]:
    keys: set[str] = set()
    for values in (
        config_keys.get("__common__", []),
        config_keys.get(task_id, []),
    ):
        if not isinstance(values, list):
            continue
        keys.update(
            key for key in values if isinstance(key, str) and key.startswith("env.")
        )
    return sorted(keys)


def _domain_options(
    task_defaults: Mapping[str, Any],
    enum_values: Mapping[str, Any],
) -> list[str]:
    common_domains = enum_values.get("__common__", {}).get("env.domain", [])
    if not isinstance(common_domains, Sequence) or isinstance(
        common_domains, (str, bytes, bytearray)
    ):
        common_domains = []
    default_domains = [
        defaults.get("domain")
        for defaults in task_defaults.values()
        if isinstance(defaults, Mapping)
    ]
    return _unique_strings([*common_domains, *default_domains])


def _gui_option_sets(
    *,
    env_ids: Sequence[Any],
    task_defaults: Mapping[str, Any],
    enum_values: Mapping[str, Any],
    robot_cache: Any,
    object_cache: Any,
    scenery_cache: Any,
) -> dict[str, list[str]]:
    objects = object_cache if isinstance(object_cache, Mapping) else {}
    tools = objects.get("tool", [])
    payloads = objects.get("payload", [])

    return {
        "tasks": _unique_strings(env_ids),
        "domains": _domain_options(task_defaults, enum_values),
        "robots": _unique_strings(flatten_cache_names(robot_cache)),
        "sceneries": _unique_strings(flatten_cache_names(scenery_cache)),
        "tools": _unique_strings(flatten_cache_names(tools)),
        "payloads": _unique_strings(flatten_cache_names(payloads)),
    }


def _available_defaults(
    defaults: Mapping[str, Any],
    optional_defaults: Mapping[str, str | None],
    teleop_devices: Sequence[str],
    interfaces: Sequence[str],
) -> dict[str, Any]:
    available_teleop_devices = [str(device) for device in teleop_devices]
    default_teleop_devices = [
        device
        for device in _DEFAULT_TELEOP_DEVICES
        if device in available_teleop_devices
    ]
    if not default_teleop_devices:
        default_teleop_devices = [
            available_teleop_devices[0] if available_teleop_devices else "keyboard"
        ]

    available_interfaces = [str(interface) for interface in interfaces]
    default_interfaces = [
        interface
        for interface in _DEFAULT_INTERFACES
        if interface in available_interfaces
    ]
    if not default_interfaces:
        default_interfaces = [
            available_interfaces[0] if available_interfaces else "gui"
        ]

    return {
        "workflow": _DEFAULT_WORKFLOW,
        "domain": _default_domain(defaults),
        "stack": _default_stack(defaults),
        "scenery": optional_defaults["scenery"],
        "robot": optional_defaults["robot"],
        "manipulator": optional_defaults["manipulator"],
        "mobile_base": optional_defaults["mobile_base"],
        "end_effector": optional_defaults["end_effector"],
        "payload": optional_defaults["payload"],
        "teleop_devices": default_teleop_devices,
        "interfaces": default_interfaces,
        "rendering_mode": _DEFAULT_RENDERING_MODE,
    }


def _action_modes(robot: Any, action_modes: Mapping[str, Any]) -> list[str]:
    if not isinstance(robot, str):
        return []
    modes = action_modes.get(robot, [])
    if not isinstance(modes, list):
        return []
    return [str(mode) for mode in modes]


def _load_sidecar_tasks() -> Mapping[str, Any]:
    from srb.cli.commands._doc._toml import load_sidecar

    return load_sidecar().tasks


def build_gui_catalog(
    *,
    cache_dir: Path,
    graphics_dir: Path,
    task_infos_by_category: Mapping[str, Sequence[Any]],
    teleop_devices: Sequence[str],
    interfaces: Sequence[str],
) -> dict[str, Any]:
    env_ids = _read_json(cache_dir / "env.json", [])
    if not isinstance(env_ids, list):
        env_ids = []

    task_defaults = _read_json(cache_dir / "task_defaults.json", {})
    if not isinstance(task_defaults, Mapping):
        task_defaults = {}

    enum_values = _read_json(cache_dir / "enum_values.json", {})
    if not isinstance(enum_values, Mapping):
        enum_values = {}

    config_keys = _read_json(cache_dir / "config_keys.json", {})
    if not isinstance(config_keys, Mapping):
        config_keys = {}

    action_modes = _read_json(cache_dir / "action_modes.json", {})
    if not isinstance(action_modes, Mapping):
        action_modes = {}

    robot_cache = _read_json(cache_dir / "robot.json", {})
    object_cache = _read_json(cache_dir / "object.json", {})
    scenery_cache = _read_json(cache_dir / "scenery.json", {})

    sidecar_tasks = _load_sidecar_tasks()
    task_infos = _task_info_index(task_infos_by_category)
    task_records = []

    for task_id in sorted(task_id for task_id in env_ids if isinstance(task_id, str)):
        defaults = task_defaults.get(task_id, {})
        if not isinstance(defaults, Mapping):
            defaults = {}
        optional_defaults = _sanitize_optional_defaults(defaults)

        meta = sidecar_tasks.get(task_id)
        gui_meta = _gui_meta(meta)
        category, info = task_infos.get(
            task_id,
            (getattr(meta, "category", "unknown"), None),
        )
        robot = optional_defaults["robot"]
        scenery = optional_defaults["scenery"]
        thumbnails = _thumbnail_status(
            graphics_dir=graphics_dir,
            task_id=task_id,
            defaults=defaults,
            robot=robot,
            scenery=scenery,
        )

        task_records.append(
            {
                "id": task_id,
                "display_name": getattr(info, "display_name", None)
                or _display_name(task_id),
                "tagline": getattr(meta, "tagline", "")
                or getattr(info, "tagline", "")
                or "",
                "category": category,
                **gui_meta,
                "defaults": _available_defaults(
                    defaults,
                    optional_defaults,
                    teleop_devices=teleop_devices,
                    interfaces=interfaces,
                ),
                "thumbnails": thumbnails,
                "overrides": {
                    "enum": _enum_overrides(task_id, enum_values),
                    "keys": _config_keys(task_id, config_keys),
                    "action_modes": _action_modes(robot, action_modes),
                },
            }
        )

    task_records.sort(
        key=lambda task: (
            not bool(task["featured"]),
            int(task["rank"]),
            str(task["display_name"]),
        )
    )

    return {
        "schema_version": 1,
        "tasks": task_records,
        "teleop_devices": [str(device) for device in teleop_devices],
        "interfaces": [str(interface) for interface in interfaces],
        "rendering_modes": list(_RENDERING_MODES),
        "options": _gui_option_sets(
            env_ids=env_ids,
            task_defaults=task_defaults,
            enum_values=enum_values,
            robot_cache=robot_cache,
            object_cache=object_cache,
            scenery_cache=scenery_cache,
        ),
    }


def build_gui_catalog_from_repo(*, cache_dir: Path | None = None) -> dict[str, Any]:
    from srb.cli.commands._doc._introspect import introspect_tasks
    from srb.interfaces import InterfaceType, TeleopDeviceType
    from srb.utils.path import SRB_CACHE_PATH

    return build_gui_catalog(
        cache_dir=cache_dir or SRB_CACHE_PATH,
        graphics_dir=SRB_GRAPHICS_DIR,
        task_infos_by_category=_task_infos_or_empty(introspect_tasks),
        teleop_devices=sorted(map(str, TeleopDeviceType)),
        interfaces=sorted(map(str, InterfaceType)),
    )


def update_offline_srb_gui_catalog_cache(*, cache_dir: Path | None = None) -> None:
    """Build the Rust GUI catalog into *cache_dir*, or refresh the published one.

    With *cache_dir* (the full ``srb cache update`` build stage) the catalog is
    derived from the sibling members in that directory. Without it (the
    thumbnail flow), the current published cache generation is republished
    with only its GUI catalog rebuilt, so refreshed thumbnail metadata reaches
    the members the GUI actually reads.
    """
    from srb.utils import logging

    if cache_dir is not None:
        catalog = build_gui_catalog_from_repo(cache_dir=cache_dir)
        if not catalog.get("tasks"):
            logging.warning(
                "Cannot update Rust GUI catalog cache because no tasks were found"
            )
            return
        target = cache_dir / SRB_GUI_CATALOG_CACHE_PATH.name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(json.dumps(catalog))
        return

    if SRB_GUI_CATALOG_CACHE_PATH != _DEFAULT_GUI_CATALOG_CACHE_PATH:
        # A redirected fixed catalog file is what
        # `read_offline_srb_gui_catalog_cache` reads, so it is also the target.
        _update_redirected_gui_catalog_cache()
        return

    _republish_gui_catalog_generation()


def _update_redirected_gui_catalog_cache() -> None:
    from srb.utils import logging

    catalog = build_gui_catalog_from_repo()
    if not catalog.get("tasks"):
        logging.warning(
            "Cannot update Rust GUI catalog cache because no tasks were found"
        )
        return

    current_cache = read_offline_srb_gui_catalog_cache()
    if catalog == current_cache:
        logging.trace("Rust GUI catalog cache is up-to-date")
        return

    try:
        SRB_GUI_CATALOG_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
        _atomic_write_bytes(SRB_GUI_CATALOG_CACHE_PATH, json.dumps(catalog))
    except PermissionError:
        logging.warning(
            "Skipping Rust GUI catalog cache update; cache path not writable: "
            f"{SRB_GUI_CATALOG_CACHE_PATH}"
        )
        return

    logging.debug(f"Updated Rust GUI catalog cache to {SRB_GUI_CATALOG_CACHE_PATH}")


class _GuiCatalogUnchanged(Exception):
    """Abort a republish whose rebuilt catalog equals the published one."""


class _GuiCatalogSuperseded(Exception):
    """Abort a republish whose source generation is no longer current."""


def _republish_gui_catalog_generation() -> None:
    """Publish the current generation again with a rebuilt GUI catalog.

    Every input is a hash-verified member of the current generation, never a
    legacy root cache file, so the refreshed catalog cannot disagree with the
    task list it is published beside. Registry-backed members are carried over
    unchanged; rebuilding them needs Isaac Sim and is `srb cache update`'s job.
    """
    from srb.cli.offline_cache_build import _validate_offline_cache_build
    from srb.utils import cache as offline_cache
    from srb.utils import logging
    from srb.utils.failures import IntegrityFailureError
    from srb.utils.publication import _read_pointer_generation_id

    generation_root = offline_cache.SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH
    catalog_member = SRB_GUI_CATALOG_CACHE_PATH.name
    try:
        snapshot = offline_cache.open_offline_srb_cache_snapshot(
            generation_root=generation_root
        )
    except (OSError, PublicationError, IntegrityFailureError) as exc:
        logging.warning(
            "Cannot refresh the Rust GUI catalog because no valid SRB cache "
            f"generation is published ({exc}); run `srb cache update`."
        )
        return

    def build(staging: Path) -> None:
        # `build` runs under the publication lock. A generation published
        # after `snapshot` was bound (a concurrent `srb cache update`) must not
        # be replaced by copies of the older members, so the refresh yields.
        current_id = _read_pointer_generation_id(generation_root / "current.json")
        if current_id != snapshot.generation_id:
            raise _GuiCatalogSuperseded(current_id)
        for member in offline_cache.OFFLINE_CACHE_REQUIRED_MEMBERS:
            if member != catalog_member:
                (staging / member).write_bytes(snapshot.read_bytes(member))
        catalog = build_gui_catalog_from_repo(cache_dir=staging)
        if not catalog.get("tasks"):
            raise PublicationError("the rebuilt Rust GUI catalog has no tasks")
        try:
            current = snapshot.read("gui_catalog", _parse_catalog_object)
        except (OSError, PublicationError, IntegrityFailureError):
            current = None
        if catalog == current:
            raise _GuiCatalogUnchanged
        (staging / catalog_member).write_bytes(json.dumps(catalog))
        _validate_offline_cache_build(
            offline_cache._offline_cache_artifacts_at(staging)
        )

    with snapshot:
        try:
            ArtifactPublication.generation(generation_root).publish(
                build,
                required_members=offline_cache.OFFLINE_CACHE_REQUIRED_MEMBERS,
            )
        except _GuiCatalogUnchanged:
            logging.trace("Rust GUI catalog cache is up-to-date")
            return
        except _GuiCatalogSuperseded as exc:
            logging.warning(
                "Skipped the Rust GUI catalog refresh: SRB cache generation "
                f"{snapshot.generation_id} was superseded by {exc.args[0]} while "
                "the catalog was being rebuilt; rerun the refresh or "
                "`srb cache update` to pick up the new thumbnails."
            )
            return

    logging.info(
        "Published a new SRB cache generation with a refreshed Rust GUI catalog "
        f"under {generation_root}"
    )


def _parse_catalog_object(data: object) -> dict[str, Any] | None:
    return data if isinstance(data, dict) else None


def read_offline_srb_gui_catalog_cache(
    *, snapshot: OfflineCacheSnapshot | None = None
) -> dict[str, Any] | None:
    def parse(payload: bytes) -> dict[str, Any]:
        data = json.loads(payload)
        if not isinstance(data, dict):
            raise TypeError("GUI catalog cache must be a JSON object")
        return data

    def parse_raw(data: object) -> dict[str, Any] | None:
        if not isinstance(data, dict):
            return None
        return data

    if snapshot is not None:
        try:
            return snapshot.read("gui_catalog", parse_raw)
        except (OSError, PublicationError, TypeError, ValueError):
            return None

    if SRB_GUI_CATALOG_CACHE_PATH != _DEFAULT_GUI_CATALOG_CACHE_PATH:
        try:
            return ArtifactPublication.file(SRB_GUI_CATALOG_CACHE_PATH).read(parse)
        except (OSError, PublicationError, TypeError, json.JSONDecodeError):
            return None

    try:
        from srb.utils.cache import open_offline_srb_cache_snapshot

        bound = open_offline_srb_cache_snapshot()
        return bound.read("gui_catalog", parse_raw)
    except (OSError, PublicationError, TypeError, ValueError):
        return None
