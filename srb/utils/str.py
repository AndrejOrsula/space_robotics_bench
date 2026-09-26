import functools
import re

try:
    from isaaclab.utils.string import *  # noqa: F403
except ModuleNotFoundError as exc:
    if exc.name and exc.name.split(".", maxsplit=1)[0] != "isaaclab":
        raise

_REGEX_SNAKE_CASE_PATTERN = (
    re.compile(r"(.)([A-Z][a-z]+)"),
    re.compile(r"__([A-Z])"),
    re.compile(r"([a-z0-9])([A-Z])"),
)


# Bound caller-supplied string caches so dynamically generated names cannot
# cause unbounded growth. Limits exceed the repository's expected name counts.
@functools.lru_cache(maxsize=4096)
def convert_to_snake_case(input: str) -> str:
    input = _REGEX_SNAKE_CASE_PATTERN[0].sub(r"\1_\2", input)
    input = _REGEX_SNAKE_CASE_PATTERN[1].sub(r"_\1", input)
    return _REGEX_SNAKE_CASE_PATTERN[2].sub(r"\1_\2", input).lower()


_REGEX_SANITIZE_CAM_NAME = re.compile(r"cam_|camera_|sensor_")


@functools.lru_cache(maxsize=1024)
def sanitize_cam_name(name: str) -> str:
    return _REGEX_SANITIZE_CAM_NAME.sub("", name)


@functools.lru_cache(maxsize=1024)
def sanitize_action_term_name(name: str) -> str:
    return convert_to_snake_case(name)


_REGEX_ENV_PRIM_PATH_PATTERN = re.compile(r"({ENV_REGEX_NS}|/World/envs/env_\.\*)")


# This cache also keys on environment index, so its size can scale at runtime.
@functools.lru_cache(maxsize=8192)
def resolve_env_prim_path(name: str, i: int) -> str:
    return _REGEX_ENV_PRIM_PATH_PATTERN.sub(f"/World/envs/env_{i}", name)
