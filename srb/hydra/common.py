import enum
import importlib
import json
from typing import Any, Callable, Set


def replace_enums_with_strings(cfg: dict[str, Any]) -> dict[str, Any]:
    def _serialize_enum(obj: enum.Enum) -> str:
        enum_class = obj.__class__
        module_path = enum_class.__module__
        class_name = enum_class.__name__
        member_name = obj.name

        return json.dumps(
            {
                "_type_": "enum",
                "path": f"{module_path}.{class_name}",
                "value": member_name,
            }
        )

    def _replacer(value: Any) -> Any:
        if isinstance(value, enum.Enum):
            return _serialize_enum(value)
        return value

    return _recursive_replace(cfg, _replacer)


def replace_strings_with_enums(cfg: dict[str, Any]) -> dict[str, Any]:
    def _deserialize_enum(s: str) -> enum.Enum:
        data = json.loads(s)
        full_path = data["path"]
        member_name = data["value"]

        module_path, class_name = full_path.rsplit(".", 1)
        module = importlib.import_module(module_path)
        enum_class = getattr(module, class_name)

        return enum_class[member_name]

    def _replacer(value: Any) -> Any:
        if isinstance(value, str) and '"_type_": "enum"' in value:
            try:
                return _deserialize_enum(value)
            except (json.JSONDecodeError, KeyError, AttributeError, ImportError):
                pass
        return value

    return _recursive_replace(cfg, _replacer)


def _recursive_replace(data: Any, replacer: Callable[[Any], Any]) -> Any:
    if isinstance(data, dict):
        return {
            _recursive_replace(k, replacer): _recursive_replace(v, replacer)
            for k, v in data.items()
        }
    elif isinstance(data, list):
        return [_recursive_replace(i, replacer) for i in data]
    elif isinstance(data, tuple):
        return tuple(_recursive_replace(i, replacer) for i in data)
    elif isinstance(data, Set):
        return {replacer(i) for i in data}
    else:
        return replacer(data)


def extract_cfg_section(cfg: dict[str, Any], key: str) -> Any:
    """Return a top-level section of a composed Hydra config, or ``None``.

    A user-supplied ``--cfg`` YAML is free to define only some of the
    top-level sections (e.g. only ``env:``); absent or empty sections
    resolve to ``None`` so callers can fall back to registered defaults.
    """
    value = cfg.get(key)
    return value if isinstance(value, dict) and value else None


def validate_cfg_schema(
    cfg: dict[str, Any],
    schema: dict[str, Any],
    *,
    nullable_mapping_paths: set[str] | None = None,
    added_paths: set[str] | None = None,
) -> None:
    """Reject keys in a user config that are absent from the registered schema.

    Partial sections are valid. Empty mapping nodes are treated as open maps,
    since they provide no keys from which to infer a fixed schema. Keys in
    ``added_paths`` (explicit Hydra ``+key=value`` additions) are accepted.
    """

    def validate_mapping(value: Any, reference: dict[str, Any], path: str) -> None:
        if not isinstance(value, dict):
            raise ValueError(f"Config section {path} must be a mapping")
        for key, child in value.items():
            child_path = f"{path}.{key}" if path else str(key)
            if key not in reference:
                if reference and not any(
                    added == child_path or added.startswith(f"{child_path}.")
                    for added in added_paths or ()
                ):
                    raise ValueError(f"Unknown config field: {child_path}")
                continue
            expected = reference[key]
            if isinstance(expected, dict):
                if child is None and child_path in (nullable_mapping_paths or set()):
                    continue
                validate_mapping(child, expected, child_path)

    validate_mapping(cfg, schema, "")
