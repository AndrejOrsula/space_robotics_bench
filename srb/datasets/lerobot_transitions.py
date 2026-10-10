"""Pure path and marker rules for SRB's certified LeRobot transitions."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from typing import Any

_SCHEMA = "srb.lerobot.transitions.v1"


def encode_observation_path(components: Sequence[str]) -> str:
    """Encode a nested observation path as a canonical slash-free suffix."""
    if isinstance(components, (str, bytes)) or not components:
        raise ValueError("observation path must contain non-empty string components")
    encoded: list[str] = []
    for component in components:
        if not isinstance(component, str) or not component:
            raise ValueError("observation path components must be non-empty strings")
        try:
            raw = component.encode("utf-8")
        except UnicodeEncodeError as exc:
            raise ValueError("observation path component is not valid UTF-8") from exc
        token_parts: list[str] = []
        for byte in raw:
            if (
                0x41 <= byte <= 0x5A
                or 0x61 <= byte <= 0x7A
                or 0x30 <= byte <= 0x39
                or byte in (0x2D, 0x5F, 0x7E)
            ):
                token_parts.append(chr(byte))
            else:
                token_parts.append(f"%{byte:02X}")
        encoded.append("".join(token_parts))
    return ".".join(encoded)


def decode_observation_path(suffix: str) -> tuple[str, ...]:
    """Decode only canonical encoded suffixes, rejecting aliases."""
    if not isinstance(suffix, str) or not suffix:
        raise ValueError("encoded observation path must be a non-empty string")
    components: list[str] = []
    for token in suffix.split("."):
        if not token:
            raise ValueError("encoded observation path contains an empty component")
        raw = bytearray()
        index = 0
        while index < len(token):
            char = token[index]
            if char == "%":
                if index + 2 >= len(token):
                    raise ValueError("encoded observation path has a malformed escape")
                digits = token[index + 1 : index + 3]
                if any(digit not in "0123456789ABCDEF" for digit in digits):
                    raise ValueError(
                        "encoded observation path escape is not uppercase hex"
                    )
                raw.append(int(digits, 16))
                index += 3
                continue
            codepoint = ord(char)
            if not (
                0x41 <= codepoint <= 0x5A
                or 0x61 <= codepoint <= 0x7A
                or 0x30 <= codepoint <= 0x39
                or char in "-_~"
            ):
                raise ValueError(
                    "encoded observation path contains an unescaped character"
                )
            raw.append(codepoint)
            index += 1
        try:
            component = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("encoded observation path is not valid UTF-8") from exc
        if not component or encode_observation_path((component,)) != token:
            raise ValueError("encoded observation path is not canonical")
        components.append(component)
    if encode_observation_path(components) != suffix:
        raise ValueError("encoded observation path is not canonical")
    return tuple(components)


def build_transition_extension(
    paths: Iterable[Sequence[str]],
) -> dict[str, Any]:
    """Build the canonical marker mapping for a set of nested paths."""
    observation_paths: dict[str, list[str]] = {}
    seen_paths: set[tuple[str, ...]] = set()
    for path in paths:
        if isinstance(path, (str, bytes)):
            raise ValueError("observation paths must be component sequences")
        components = tuple(path)
        suffix = encode_observation_path(components)
        if components in seen_paths:
            raise ValueError("duplicate observation path")
        if suffix in observation_paths:
            raise ValueError("observation path encodings collide")
        seen_paths.add(components)
        observation_paths[suffix] = list(components)
    if not observation_paths:
        raise ValueError("transition extension requires at least one observation path")
    _reject_leaf_prefix_paths(seen_paths)
    return {"schema": _SCHEMA, "observation_paths": observation_paths}


def validate_transition_extension(value: object) -> dict[str, Any]:
    """Validate a marker and return a detached canonical dictionary."""
    if not isinstance(value, Mapping) or set(value) != {"schema", "observation_paths"}:
        raise ValueError(
            "transition extension must contain schema and observation_paths"
        )
    if value.get("schema") != _SCHEMA:
        raise ValueError(
            f"unsupported transition extension schema: {value.get('schema')!r}"
        )
    raw_paths = value.get("observation_paths")
    if not isinstance(raw_paths, Mapping) or not raw_paths:
        raise ValueError(
            "transition extension observation_paths must be a non-empty object"
        )
    paths: dict[str, list[str]] = {}
    seen: set[tuple[str, ...]] = set()
    for raw_suffix, raw_components in raw_paths.items():
        if not isinstance(raw_suffix, str):
            raise ValueError("transition extension suffix keys must be strings")
        suffix = raw_suffix
        components_value = raw_components
        if not isinstance(components_value, (list, tuple)):
            raise ValueError(f"transition path {suffix!r} must be a component array")
        components = tuple(components_value)
        if any(
            not isinstance(component, str) or not component for component in components
        ):
            raise ValueError(
                f"transition path {suffix!r} components must be non-empty strings"
            )
        decoded = decode_observation_path(suffix)
        if components != decoded:
            raise ValueError(f"transition path {suffix!r} does not match its encoding")
        if components in seen:
            raise ValueError("transition extension contains duplicate paths")
        seen.add(components)
        paths[suffix] = list(components)
    if len(paths) != len(raw_paths):
        raise ValueError("transition extension contains duplicate suffixes")
    _reject_leaf_prefix_paths(seen)
    return {"schema": _SCHEMA, "observation_paths": paths}


def _reject_leaf_prefix_paths(paths: Iterable[tuple[str, ...]]) -> None:
    ordered = sorted(paths)
    for parent, child in zip(ordered, ordered[1:]):
        if len(parent) < len(child) and child[: len(parent)] == parent:
            raise ValueError("transition observation leaf paths cannot be prefixes")
