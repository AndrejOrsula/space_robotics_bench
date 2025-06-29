import functools
import re

from isaaclab.utils.string import *  # noqa: F403

_REGEX_SNAKE_CASE_PATTERN = (
    re.compile(r"(.)([A-Z][a-z]+)"),
    re.compile(r"__([A-Z])"),
    re.compile(r"([a-z0-9])([A-Z])"),
)


@functools.cache
def convert_to_snake_case(input: str) -> str:
    input = _REGEX_SNAKE_CASE_PATTERN[0].sub(r"\1_\2", input)
    input = _REGEX_SNAKE_CASE_PATTERN[1].sub(r"_\1", input)
    return _REGEX_SNAKE_CASE_PATTERN[2].sub(r"\1_\2", input).lower()


_REGEX_SANITIZE_CAM_NAME = re.compile(r"cam_|camera_|sensor_")


@functools.cache
def sanitize_cam_name(name: str) -> str:
    return _REGEX_SANITIZE_CAM_NAME.sub("", name)


@functools.cache
def sanitize_action_term_name(name: str) -> str:
    return convert_to_snake_case(name.removesuffix("Action"))


_REGEX_ENV_PRIM_PATH_PATTERN = re.compile(r"({ENV_REGEX_NS}|/World/envs/env_.*)")


@functools.cache
def resolve_env_prim_path(name: str, i: int) -> str:
    return _REGEX_ENV_PRIM_PATH_PATTERN.sub(f"/World/envs/env_{i}", name)
