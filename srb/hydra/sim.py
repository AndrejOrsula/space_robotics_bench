import functools
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, Callable, get_args

_SCENERY_REGION_FIELDS = ("body", "lat_deg", "lon_deg", "size_m")


def _scenery_field_accepts_region(field_type: Any) -> bool:
    """Return whether a scenery field supports real-terrain region selection.

    Limit the Hydra mapping shim to the variant-capable scenery seam so other
    scenery fields retain strict struct validation.
    """
    from srb.core.asset import AssetVariant, Scenery
    from srb.hydra._type_utils import _unwrap_union_members

    members = _unwrap_union_members(field_type)
    has_variant = any(member is AssetVariant for member in members)
    has_scenery = any(
        isinstance(member, type)
        and issubclass(member, Scenery)
        and member is not Scenery
        for member in members
    ) or any(member is Scenery for member in members)
    return has_variant and has_scenery


def _prepare_scenery_region_defaults(task_cfg_cls: type, env_cfg_dict: dict) -> None:
    """Expose region keys to Hydra for variant-capable scenery fields.

    Scalar defaults become mappings so struct-mode dotted overrides work;
    resolution converts untouched selectors back to enums. Existing typed
    scenery mappings keep their serialized fields.
    """
    from srb.hydra.type_resolution import get_type_hints_safe

    field_type = get_type_hints_safe(task_cfg_cls).get("scenery")
    if not _scenery_field_accepts_region(field_type):
        return

    value = env_cfg_dict.get("scenery")
    if isinstance(value, dict):
        # Typed scenery contributes its keys; variant-only placeholders can break
        # resolver comparisons (for example, MoonSurface has no ``body`` field).
        return
    elif value is None:
        scenery_cfg = {"name": None}
    else:
        scenery_cfg = {"name": value}
    for key in _SCENERY_REGION_FIELDS:
        scenery_cfg.setdefault(key, None)
    env_cfg_dict["scenery"] = scenery_cfg


def _nullable_mapping_paths(
    task_cfg_cls: type, registered_cfg: Mapping[str, Any]
) -> set[str]:
    """Return registered mapping fields whose task annotations allow ``None``."""
    from srb.hydra.type_resolution import get_type_hints_safe

    env_schema = registered_cfg.get("env")
    if not isinstance(env_schema, Mapping):
        return set()

    return {
        f"env.{name}"
        for name, field_type in get_type_hints_safe(task_cfg_cls).items()
        if isinstance(env_schema.get(name), Mapping)
        and type(None) in get_args(field_type)
    }


def _registered_node(registered_cfg: Mapping[str, Any], dotted: str) -> Any:
    """Return the node at *dotted* in *registered_cfg* (``None`` if absent)."""
    node: Any = registered_cfg
    for part in dotted.split("."):
        if not isinstance(node, Mapping) or part not in node:
            return None
        node = node[part]
    return node


def _composite_robot_overrides(
    prefix: str, key: str, value: str, node: Any
) -> list[str] | None:
    """Split ``base+manipulator[+end_effector]`` for a composite robot node."""
    if not (
        isinstance(node, Mapping)
        and "name" not in node
        and isinstance(node.get("mobile_base"), Mapping)
        and isinstance(node.get("manipulator"), Mapping)
    ):
        return None
    parts = value.split("+")
    if len(parts) not in (2, 3) or not all(parts):
        return None
    # The manipulator selector resolves its own ``+end_effector`` suffix.
    return [
        f"{prefix}{key}.mobile_base.name={parts[0]}",
        f"{prefix}{key}.manipulator.name={'+'.join(parts[1:])}",
    ]


def _rewrite_asset_shortcut_overrides(
    argv: list[str],
    dict_asset_paths: set[str],
    registered_cfg: Mapping[str, Any] | None = None,
) -> list[str]:
    """Rewrite short asset selectors as ``path.name=value`` overrides.

    Preserve Hydra prefixes and null selectors. With ``registered_cfg``, typed
    paths need mappings; named mappings also cover unhinted sub-assets.
    Returns a new list without modifying ``argv``.
    """
    out: list[str] = []
    for arg in argv:
        if "=" not in arg:
            out.append(arg)
            continue
        raw_key, _sep, value = arg.partition("=")
        prefix = (
            "++" if raw_key.startswith("++") else "+" if raw_key.startswith("+") else ""
        )
        key = raw_key[len(prefix) :]
        node = None if registered_cfg is None else _registered_node(registered_cfg, key)
        composite = _composite_robot_overrides(prefix, key, value, node)
        if composite is not None:
            out.extend(composite)
            continue
        # Only scalar values are asset names; mappings and lists stay as given.
        if (
            value not in {"null", "~"}
            and not value.lstrip().startswith(("{", "["))
            and (
                (
                    key in dict_asset_paths
                    and (registered_cfg is None or isinstance(node, Mapping))
                )
                # Nested sub-assets may lack type hints but register as mappings
                # with a `name` selector.
                or (isinstance(node, Mapping) and "name" in node)
            )
        ):
            out.append(f"{prefix}{key}.name={value}")
        else:
            out.append(arg)
    return out


def _explicit_env_override_paths(
    overrides: Iterable[str],
) -> frozenset[tuple[str, ...]]:
    """Return explicit CLI key paths below ``env``, with Hydra prefixes removed."""
    paths: set[tuple[str, ...]] = set()
    for override in overrides:
        key = override.partition("=")[0].strip().lstrip("~+")
        parts = tuple(key.split("."))
        if len(parts) > 1 and parts[0] == "env":
            paths.add(parts[1:])
    return frozenset(paths)


def _added_agent_paths(overrides: Iterable[str]) -> set[str]:
    """Return ``agent.*`` keys added with Hydra ``+``/``++`` prefixes."""
    keys = (
        override.partition("=")[0].strip()
        for override in overrides
        if override.startswith("+")
    )
    return {key.lstrip("+") for key in keys if key.lstrip("+").startswith("agent.")}


def _explicit_env_config_paths(config_path: Path | None) -> frozenset[tuple[str, ...]]:
    """Return ``env`` paths from a user config, excluding Hydra run snapshots.

    Run snapshots replay explicit values from their sibling ``overrides.yaml``.
    """
    if (
        config_path is None
        or config_path.parent.name == ".hydra"
        or not config_path.is_file()
    ):
        return frozenset()

    import yaml

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, Mapping):
        return frozenset()
    env = config.get("env")
    if not isinstance(env, Mapping):
        return frozenset()

    paths: set[tuple[str, ...]] = set()

    def collect(value: Any, path: tuple[str, ...]) -> None:
        if isinstance(value, Mapping):
            for key, child in value.items():
                child_path = (*path, str(key))
                paths.add(child_path)
                collect(child, child_path)

    collect(env, ())
    return frozenset(paths)


def _recorded_run_overrides(config_path: Path | None) -> list[str]:
    """Read a run snapshot's sibling ``overrides.yaml``; empty outside one."""
    if config_path is None or config_path.parent.name != ".hydra":
        return []
    overrides_path = config_path.parent / "overrides.yaml"
    if not overrides_path.is_file():
        return []
    import yaml

    recorded = yaml.safe_load(overrides_path.read_text(encoding="utf-8"))
    if recorded is None:
        return []
    if not isinstance(recorded, list) or not all(
        isinstance(item, str) for item in recorded
    ):
        raise ValueError(
            f"{overrides_path} must be a YAML list of Hydra override strings"
        )
    return recorded


def _recorded_run_env_overrides(config_path: Path | None) -> list[str]:
    """Return replayable ``env`` overrides recorded with a run snapshot.

    Skip deletions; upgrade ``+key=value`` to ``++key=value`` because the key
    already exists in the saved config.
    """
    replayed: list[str] = []
    for override in _recorded_run_overrides(config_path):
        key = override.partition("=")[0].strip()
        if key.startswith("~") or not key.lstrip("+").startswith("env."):
            continue
        if key.startswith("+") and not key.startswith("++"):
            override = f"+{override.strip()}"
        replayed.append(override)
    return replayed


def _insert_before_first_override(argv: list[str], extra: list[str]) -> list[str]:
    """Insert replayed overrides before CLI overrides, keeping them contiguous."""
    if not extra:
        return list(argv)
    for index, arg in enumerate(argv[1:], start=1):
        if "=" in arg and not arg.startswith("-"):
            return [*argv[:index], *extra, *argv[index:]]
    return [*argv, *extra]


def _collect_dict_asset_paths(task_cfg_cls: type) -> set[str]:
    """Return dotted paths for dict-form asset fields in task config hints.

    Returns an empty set when Isaac Sim asset types are unavailable.
    """
    from srb.hydra._type_utils import (
        _unwrap_to_config_type,
        _unwrap_union_members,
    )
    from srb.hydra.type_resolution import get_type_hints_safe

    try:
        from srb.core.asset import Object, Robot
        from srb.core.asset.object.tool import ActiveTool, Tool
        from srb.core.asset.scenery import Scenery
    except ImportError:
        # Without asset types, leave CLI overrides untouched.
        return set()

    asset_bases: tuple[type, ...] = (Robot, Tool, ActiveTool, Object, Scenery)
    paths: set[str] = set()

    def _walk(cls, prefix: str, seen: set[int]) -> None:
        """Collect nested asset paths, avoiding cycles by class identity."""
        cls_id = id(cls)
        if cls_id in seen:
            return
        seen.add(cls_id)

        hints = get_type_hints_safe(cls)
        for field_name, field_type in hints.items():
            if field_name.startswith("_"):
                continue
            dotted = f"{prefix}.{field_name}" if prefix else field_name
            members = _unwrap_union_members(field_type)
            is_asset = any(
                isinstance(m, type) and issubclass(m, asset_bases) for m in members
            )
            if is_asset:
                paths.add(dotted)
                for m in members:
                    if isinstance(m, type) and issubclass(m, asset_bases):
                        _walk(m, dotted, seen)
                continue
            nested_cls = _unwrap_to_config_type(field_type)
            if nested_cls is not None:
                _walk(nested_cls, dotted, seen)

    _walk(task_cfg_cls, "env", set())
    return paths


def register_task_to_hydra(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> tuple[type, Any]:
    task_cfg_cls, agent_cfg, _cfg_dict = _register_task_cfg(
        task_name, agent_cfg_entry_point
    )
    return task_cfg_cls, agent_cfg


def _register_task_cfg(
    task_name: str, agent_cfg_entry_point: str | None = None
) -> tuple[type, Any, dict[str, Any]]:
    """Register *task_name* with Hydra; also return the registered config."""
    from srb.hydra.common import replace_enums_with_strings
    from srb.hydra.extract import extract_defaults_from_class
    from srb.utils.cfg import load_cfg_class_from_registry, load_cfg_from_registry
    from srb.utils.dict import replace_slices_with_strings
    from srb.utils.spaces import replace_dict_spaces_with_strings

    # Get the task config CLASS (no instantiation)
    task_cfg_cls = load_cfg_class_from_registry(task_name, "task_cfg")
    # Extract defaults via pure class introspection
    env_cfg_dict = extract_defaults_from_class(task_cfg_cls)

    # Load agent config (still loaded as instance via registry)
    if agent_cfg_entry_point is None:
        agent_cfg = {}
        agent_cfg_dict = {}
    else:
        agent_cfg = load_cfg_from_registry(task_name, agent_cfg_entry_point)
        if isinstance(agent_cfg, dict):
            agent_cfg_dict = agent_cfg
        else:
            agent_cfg_dict = agent_cfg.to_dict()  # type: ignore[misc]

    cfg_dict = {"env": env_cfg_dict, "agent": agent_cfg_dict}
    # Encode values OmegaConf cannot represent.
    cfg_dict = replace_dict_spaces_with_strings(cfg_dict)
    cfg_dict = replace_slices_with_strings(cfg_dict)
    cfg_dict = replace_enums_with_strings(cfg_dict)
    # OmegaConf rejects str subclasses such as Isaac Lab's ResolvableString.
    # Demote all subclasses because configclass may retain another class object.
    from srb.hydra.common import _recursive_replace

    cfg_dict = _recursive_replace(
        cfg_dict,
        lambda v: (
            str.__new__(str, v) if isinstance(v, str) and type(v) is not str else v
        ),
    )
    # Keep struct validation enabled while making the documented region form
    # addressable through ordinary dotted overrides.
    _prepare_scenery_region_defaults(task_cfg_cls, cfg_dict["env"])
    # Store the configuration to Hydra
    from hydra.core.config_store import ConfigStore

    ConfigStore.instance().store(name=task_name.rsplit("/", 1)[1], node=cfg_dict)
    return task_cfg_cls, agent_cfg, cfg_dict


def hydra_task_config(
    task_name: str,
    agent_cfg_entry_point: str | None = None,
    config_path: Path | str | None = None,
) -> Callable:
    config_path = Path(config_path) if config_path else None

    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            import sys

            import hydra
            from hydra.core.hydra_config import HydraConfig
            from omegaconf import DictConfig, OmegaConf

            from srb.hydra.common import (
                extract_cfg_section,
                replace_strings_with_enums,
                validate_cfg_schema,
            )
            from srb.hydra.create import create_instance
            from srb.hydra.resolve import AssetResolver
            from srb.utils.dict import replace_strings_with_slices
            from srb.utils.spaces import replace_strings_with_env_cfg_spaces

            # Register the task to Hydra
            task_cfg_cls, agent_cfg, registered_cfg = _register_task_cfg(
                task_name, agent_cfg_entry_point
            )

            # Rewrite CLI shortcuts: `env.robot=X` → `env.robot.name=X`
            # for any dict-form asset field declared on the task config
            # whose registered node is a mapping.
            dict_asset_paths = _collect_dict_asset_paths(task_cfg_cls)
            # A replayed run config keeps the run's explicit env overrides.
            sys.argv = _insert_before_first_override(
                sys.argv, _recorded_run_env_overrides(config_path)
            )
            sys.argv = _rewrite_asset_shortcut_overrides(
                sys.argv, dict_asset_paths, registered_cfg
            )

            # Define the new Hydra main function
            @hydra.main(
                config_path=config_path.parent.as_posix() if config_path else None,
                config_name=task_name.rsplit("/", 1)[1]
                if config_path is None
                else config_path.stem,
                version_base=None,
            )
            def hydra_main(
                hydra_cfg: DictConfig,
                task_cfg_cls=task_cfg_cls,
                agent_cfg=agent_cfg,
            ):
                # Convert to a native dictionary
                hydra_cfg = OmegaConf.to_container(  # type: ignore[misc]
                    hydra_cfg,
                    resolve=True,
                )
                validate_cfg_schema(
                    hydra_cfg,
                    registered_cfg,
                    nullable_mapping_paths=_nullable_mapping_paths(
                        task_cfg_cls, registered_cfg
                    ),
                    # A replayed run snapshot keeps its recorded `+agent.*` keys.
                    added_paths=_added_agent_paths(
                        (
                            *_recorded_run_overrides(config_path),
                            *HydraConfig.get().overrides.task,
                        )
                    ),
                )
                # Replace serialized strings with their corresponding python objects
                hydra_cfg = replace_strings_with_slices(hydra_cfg)  # type: ignore[misc]
                hydra_cfg = replace_strings_with_enums(hydra_cfg)  # type: ignore[misc]
                # A user-supplied ``--cfg`` YAML may contain only some of the
                # top-level sections; missing ones fall back to the registered
                # defaults instead of raising ``KeyError``.
                env_overrides = extract_cfg_section(hydra_cfg, "env") or {}
                agent_overrides = extract_cfg_section(hydra_cfg, "agent")
                # Resolve asset strings in the env config dict. The job's own
                # overrides (one sweep point under --multirun) mark which
                # asset fields the user set explicitly, so they survive the
                # reset a changed asset selector applies to the old default.
                explicit = _explicit_env_override_paths(
                    HydraConfig.get().overrides.task
                ) | _explicit_env_config_paths(config_path)
                AssetResolver(explicit_overrides=explicit).resolve(
                    task_cfg_cls, env_overrides
                )
                # Construct the env config instance from class + dict
                env_cfg = create_instance(task_cfg_cls, env_overrides)
                # Restore spaces after composition to avoid OmegaConf reinterpretation.
                replace_strings_with_env_cfg_spaces(env_cfg)
                # Get agent configs
                if isinstance(agent_cfg, dict):
                    agent_cfg = (
                        agent_overrides if agent_overrides is not None else agent_cfg
                    )
                elif agent_overrides is not None:
                    agent_cfg = create_instance(type(agent_cfg), agent_overrides)
                # Call the original function
                func(env_cfg, agent_cfg, *args, **kwargs)

            # Call the new Hydra main function
            hydra_main()

        return wrapper

    return decorator
