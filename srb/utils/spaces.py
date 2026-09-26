from isaaclab.envs.utils.spaces import *  # noqa: F403
from isaaclab.envs.utils.spaces import serialize_space


def replace_dict_spaces_with_strings(data):
    """Replace gymnasium Space objects with JSON string representations in a dict tree.

    Counterpart to :func:`replace_env_cfg_spaces_with_strings` but operates
    on plain dicts instead of config instances.  Used when serialising the
    output of :func:`extract_defaults_from_class` for Hydra's ConfigStore.
    """
    try:
        from gymnasium import spaces
    except ImportError:
        return data

    if isinstance(data, dict):
        return {k: replace_dict_spaces_with_strings(v) for k, v in data.items()}
    elif isinstance(data, (list, tuple)):
        replaced = [replace_dict_spaces_with_strings(v) for v in data]
        return type(data)(replaced)
    elif isinstance(data, spaces.Space):
        return serialize_space(data)
    return data
