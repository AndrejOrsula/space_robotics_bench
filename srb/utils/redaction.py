"""Redact secret-bearing command-line arguments before persistence.

The Rust GUI duplicates these rules in
``crates/srb_gui/src/command_contract.rs``; keep both copies synchronized.
"""

from __future__ import annotations

import re
from collections.abc import Iterable
from urllib.parse import unquote_plus

REDACTED = "<redacted>"

#: Credential vocabulary. A key is secret when any of its components is one
#: of these words. Keys are split into segments at every character outside
#: ``[A-Za-z0-9_-]`` and segments into components at ``_``, ``-`` and
#: lower-to-upper camelCase boundaries, then lowercased; so ``HF_TOKEN``,
#: ``authToken`` and ``+experiment.secret_sauce`` are secrets while
#: ``monkey``, ``author`` and ``max_tokens`` are not.
SECRET_KEY_COMPONENTS: frozenset[str] = frozenset(
    {
        "accesskey",
        "apikey",
        "auth",
        "authorization",
        "bearer",
        "cookie",
        "cookies",
        "credential",
        "credentials",
        "dsn",
        "passphrase",
        "passwd",
        "password",
        "passwords",
        "pat",
        "privatekey",
        "secret",
        "secrets",
        "token",
    }
)

#: ``key`` alone is ambiguous (``obs_key``, ``sort_key``). A segment names a
#: credential when it is exactly ``key`` (``wandb.key``, ``--key``) or ends in
#: one of these qualifiers followed by ``key`` (``api_key``, ``sshKey``).
SECRET_KEY_QUALIFIERS: frozenset[str] = frozenset(
    {
        "access",
        "api",
        "auth",
        "client",
        "encryption",
        "private",
        "secret",
        "session",
        "signing",
        "ssh",
    }
)

_SEGMENT_SEPARATOR = re.compile(r"[^0-9A-Za-z_-]+")
_CAMEL_BOUNDARY = re.compile(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])")
_COMPONENT_SEPARATOR = re.compile(r"[_-]+")
# A ``KEY=VALUE`` token keeps its key text only when the key looks like a
# config path or option; otherwise the whole token is treated as a value.
_KEY_SHAPE = re.compile(r"[A-Za-z0-9_.\-/+~@\[\]]+")
# A bare long option in the lowercase spelling every SRB and framework flag
# uses. After a credential flag, only a token of exactly this shape (or a
# single-letter short option, or the ``--`` separator) is treated as the next
# option rather than as the credential's value.
_LONG_OPTION = re.compile(r"--[a-z][a-z0-9]*(?:[-_.][a-z0-9]+)*")
_SHORT_OPTION = re.compile(r"-[A-Za-z]")
# Rust ``u8::is_ascii_whitespace``.
_ASCII_WHITESPACE = frozenset(" \t\n\x0c\r")
_HEADER_NAME = re.compile(r"[A-Za-z0-9_-]+")
# Raw credential formats are caught even when their surrounding key is
# innocuous. Minimum lengths avoid treating short examples and ordinary text
# as credentials.
_RAW_CREDENTIAL = re.compile(
    r"(?<![A-Za-z0-9])(?:"
    r"hf_[A-Za-z0-9]{20,}|ghp_[A-Za-z0-9]{20,}|"
    r"sk-[A-Za-z0-9_-]{20,}|glpat-[A-Za-z0-9_-]{20,}|"
    r"AKIA[A-Z0-9]{16}"
    r")(?![A-Za-z0-9])"
)


def _segment_components(segment: str) -> list[str]:
    split = _CAMEL_BOUNDARY.sub("_", segment)
    return [part.lower() for part in _COMPONENT_SEPARATOR.split(split) if part]


def is_secret_key(key: str) -> bool:
    """True when an argument or config key names a credential-like value."""
    for segment in _SEGMENT_SEPARATOR.split(key):
        components = _segment_components(segment)
        if any(component in SECRET_KEY_COMPONENTS for component in components):
            return True
        if components == ["key"] or (
            len(components) >= 2
            and components[-1] == "key"
            and components[-2] in SECRET_KEY_QUALIFIERS
        ):
            return True
    return False


def _is_flag(arg: str) -> bool:
    return arg.startswith("-") and len(arg) > 1


def _is_negated_flag(arg: str) -> bool:
    """``--no-auth``-style boolean switches take no value."""
    if not arg.startswith("--"):
        return False
    components = _segment_components(_SEGMENT_SEPARATOR.split(arg)[0])
    return bool(components) and components[0] == "no"


def _is_exact_option(arg: str) -> bool:
    return (
        arg == "--"
        or _LONG_OPTION.fullmatch(arg) is not None
        or _SHORT_OPTION.fullmatch(arg) is not None
    )


def _is_ascii_alnum(character: str) -> bool:
    return character.isascii() and character.isalnum()


def _redact_url_userinfo(value: str) -> str:
    """Replace ``scheme://USERINFO@host`` userinfo with :data:`REDACTED`."""
    pieces: list[str] = []
    kept_from = 0
    search = 0
    while (marker := value.find("://", search)) >= 0:
        start = marker + 3
        search = start
        if marker == 0 or not _is_ascii_alnum(value[marker - 1]):
            continue
        end = start
        while (
            end < len(value)
            and value[end] not in "/?#"
            and value[end] not in _ASCII_WHITESPACE
        ):
            end += 1
        at = value.rfind("@", start, end)
        if at > start:
            pieces.append(value[kept_from:start])
            pieces.append(REDACTED)
            kept_from = at
        search = max(end, start)
    pieces.append(value[kept_from:])
    return "".join(pieces)


def _redact_query_parameters(value: str) -> str:
    pieces: list[str] = []
    kept_from = 0
    search = 0
    # Fragments (``#access_token=...``) carry OAuth-style parameters too.
    while (
        marker := min(
            (i for i in (value.find("?", search), value.find("#", search)) if i >= 0),
            default=-1,
        )
    ) >= 0:
        end = marker + 1
        while (
            end < len(value)
            and (value[marker] == "#" or value[end] != "#")
            and value[end] not in _ASCII_WHITESPACE
        ):
            end += 1
        query = value[marker + 1 : end]
        params: list[str] = []
        changed = False
        for parameter in query.split("&"):
            key, sep, _ = parameter.partition("=")
            try:
                decoded_key = unquote_plus(key)
            except (UnicodeDecodeError, ValueError):
                decoded_key = key
            if sep and is_secret_key(decoded_key):
                params.append(f"{key}={REDACTED}")
                changed = True
            else:
                params.append(parameter)
        if changed:
            pieces.append(value[kept_from:marker])
            pieces.append(value[marker] + "&".join(params))
            kept_from = end
        search = max(end, marker + 1)
    pieces.append(value[kept_from:])
    return "".join(pieces)


def _redact_bearer(value: str) -> str:
    """Replace the credential after a ``Bearer`` auth scheme word."""
    pieces: list[str] = []
    kept_from = 0
    index = 0
    while index + 6 <= len(value):
        word = value[index : index + 6]
        if (
            word.isascii()
            and word.lower() == "bearer"
            and (index == 0 or not _is_ascii_alnum(value[index - 1]))
        ):
            gap = index + 6
            while gap < len(value) and value[gap] in _ASCII_WHITESPACE:
                gap += 1
            end = gap
            while end < len(value) and value[end] not in _ASCII_WHITESPACE:
                end += 1
            if gap > index + 6 and end > gap:
                pieces.append(value[kept_from:gap])
                pieces.append(REDACTED)
                kept_from = end
                index = end
                continue
        index += 1
    pieces.append(value[kept_from:])
    return "".join(pieces)


def _redact_raw_credentials(value: str) -> str:
    """Mask recognizable raw credentials regardless of their config key."""
    return _RAW_CREDENTIAL.sub(REDACTED, value)


def _split_mapping_entries(value: str) -> list[str]:
    """Split a brace-map body on commas outside nested maps and strings."""
    entries: list[str] = []
    start = 0
    depth = 0
    quote: str | None = None
    escaped = False
    for index, character in enumerate(value):
        if quote is not None:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote:
                quote = None
        elif character in "'\"":
            quote = character
        elif character == "{":
            depth += 1
        elif character == "}":
            depth = max(0, depth - 1)
        elif character == "," and depth == 0:
            entries.append(value[start:index])
            start = index + 1
    entries.append(value[start:])
    return entries


def _mapping_separator(entry: str) -> int | None:
    """Find a map entry's first colon or equals outside nested values."""
    depth = 0
    quote: str | None = None
    escaped = False
    for index, character in enumerate(entry):
        if quote is not None:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote:
                quote = None
        elif character in "'\"":
            quote = character
        elif character == "{":
            depth += 1
        elif character == "}":
            depth = max(0, depth - 1)
        elif depth == 0 and character in ":=":
            return index
    return None


def _redact_nested_credentials(value: str) -> str:
    """Redact secret-key values in brace-delimited argv maps recursively."""
    opening = value.find("{")
    if opening < 0:
        return value
    depth = 0
    quote: str | None = None
    escaped = False
    closing: int | None = None
    for index in range(opening, len(value)):
        character = value[index]
        if quote is not None:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == quote:
                quote = None
        elif character in "'\"":
            quote = character
        elif character == "{":
            depth += 1
        elif character == "}":
            depth -= 1
            if depth == 0:
                closing = index
                break
    if closing is None:
        return value[: opening + 1] + _redact_nested_credentials(value[opening + 1 :])

    body = value[opening + 1 : closing]
    redacted_entries: list[str] = []
    for entry in _split_mapping_entries(body):
        separator = _mapping_separator(entry)
        if separator is None:
            redacted_entries.append(_redact_nested_credentials(entry))
            continue
        key = entry[:separator].strip()
        if len(key) >= 2 and key[0] == key[-1] and key[0] in "'\"":
            key = key[1:-1]
        raw_value = entry[separator + 1 :]
        if is_secret_key(key):
            leading = raw_value[: len(raw_value) - len(raw_value.lstrip())]
            trailing = raw_value[len(raw_value.rstrip()) :]
            redacted_value = f"{leading}{REDACTED}{trailing}"
        else:
            redacted_value = _redact_nested_credentials(raw_value)
        redacted_entries.append(entry[: separator + 1] + redacted_value)
    mapped = "{" + ",".join(redacted_entries) + "}"
    return value[:opening] + mapped + _redact_nested_credentials(value[closing + 1 :])


def redact_secret_text(value: str) -> str:
    """Redact URL, query, header, bearer, and raw credentials."""
    value = _redact_nested_credentials(value)
    value = _redact_url_userinfo(value)
    value = _redact_query_parameters(value)
    colon = value.find(":")
    if colon > 0:
        name = value[:colon].strip("".join(_ASCII_WHITESPACE))
        rest = value[colon + 1 :].strip("".join(_ASCII_WHITESPACE))
        if rest and _HEADER_NAME.fullmatch(name) and is_secret_key(name):
            return f"{value[: colon + 1]} {REDACTED}"
    value = _redact_bearer(value)
    return _redact_raw_credentials(value)


def redact_secret_arguments(args: Iterable[str]) -> list[str]:
    """Replace secret values while preserving recognized option and config keys.

    Credential ``KEY=VALUE`` pairs retain their key and mask the value; other
    values pass through :func:`redact_secret_text`. Credential flags consume the
    next token unless it is exactly an option (``--long``, ``-x``, ``--``);
    ``--no-*`` takes no value. ``key`` is secret alone or after a qualifier.
    """
    redacted: list[str] = []
    expect_secret_value = False
    for arg in args:
        if expect_secret_value:
            expect_secret_value = False
            if not _is_exact_option(arg):
                redacted.append(REDACTED)
                continue
        key, sep, value = arg.partition("=")
        if sep and _KEY_SHAPE.fullmatch(key):
            if is_secret_key(key):
                redacted.append(f"{key}={REDACTED}")
            else:
                redacted.append(f"{key}={redact_secret_text(value)}")
            continue
        if (
            not sep
            and _is_flag(arg)
            and is_secret_key(arg)
            and not _is_negated_flag(arg)
        ):
            expect_secret_value = True
        redacted.append(redact_secret_text(arg))
    return redacted
