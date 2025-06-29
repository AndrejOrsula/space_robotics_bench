import enum
import importlib
import json
from typing import Any, Callable, Dict, List, Set, Tuple


def replace_enums_with_strings(cfg: Dict[str, Any]) -> Dict[str, Any]:
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


def replace_strings_with_enums(cfg: Dict[str, Any]) -> Dict[str, Any]:
    def _deserialize_enum(s: str) -> enum.Enum:
        data = json.loads(s)
        full_path = data["path"]
        member_name = data["value"]

        # Dynamically import the module and get the enum class
        module_path, class_name = full_path.rsplit(".", 1)
        module = importlib.import_module(module_path)
        enum_class = getattr(module, class_name)

        # Get the specific member from the enum class
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
    if isinstance(data, Dict):
        return {
            _recursive_replace(k, replacer): _recursive_replace(v, replacer)
            for k, v in data.items()
        }
    elif isinstance(data, List):
        return [_recursive_replace(i, replacer) for i in data]
    elif isinstance(data, Tuple):
        return tuple(_recursive_replace(i, replacer) for i in data)
    elif isinstance(data, Set):
        return {replacer(i) for i in data}
    else:
        return replacer(data)
