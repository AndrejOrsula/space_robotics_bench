"""Read sidecar TOML files for manually-curated doc metadata."""

from __future__ import annotations

import re
import tomllib
from dataclasses import dataclass, field

from srb.utils.path import SRB_DIR, SRB_TASKS_SIDECAR

_ASSETS_TOML = SRB_DIR / "docs" / "data" / "assets.toml"
_TASKS_TOML = SRB_TASKS_SIDECAR


@dataclass(frozen=True)
class AssetMeta:
    tagline: str = ""
    video_id: str = ""
    source: str = ""
    license: str = ""
    author: str = ""
    notes: str = ""
    """Free-form markdown appended to the generated catalog entry."""


@dataclass(frozen=True)
class TaskGuiMeta:
    featured: bool = False
    rank: int = 1000
    theme: str = "dark"


@dataclass(frozen=True)
class TaskMeta:
    tagline: str = ""
    video_id: str = ""
    category: str = "unknown"
    gui: TaskGuiMeta = field(default_factory=TaskGuiMeta)


@dataclass
class SidecarData:
    assets: dict[str, AssetMeta] = field(default_factory=dict)
    tasks: dict[str, TaskMeta] = field(default_factory=dict)


def _bool_or_default(value: object, default: bool) -> bool:
    return value if isinstance(value, bool) else default


def _int_or_default(value: object, default: int) -> int:
    return value if isinstance(value, int) and not isinstance(value, bool) else default


def _str_or_default(value: object, default: str) -> str:
    return value if isinstance(value, str) else default


def _category_id_from_heading(heading: str) -> str:
    heading = re.sub(r"\([^)]*\)", "", heading).strip().lower()
    heading = re.sub(r"[^a-z0-9]+", "_", heading).strip("_")
    return {
        "templates": "template",
        "mobile_robotics": "mobile",
        "mobile_manipulation": "mobile_manipulation",
        "digital_twin": "digital_twin",
        "manipulation": "manipulation",
    }.get(heading, heading or "unknown")


# Bare-key table header, optionally followed by a TOML comment.
_TOP_LEVEL_TABLE_HEADER = re.compile(r"^\[\s*([A-Za-z0-9_-]+)\s*\]\s*(?:#.*)?$")


def _task_categories_from_toml_text(text: str) -> dict[str, str]:
    categories: dict[str, str] = {}
    current_category = "unknown"
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.startswith("## "):
            current_category = _category_id_from_heading(line[3:])
            continue
        # Top-level tables only; dotted (`[x.gui]`) and array tables do not match.
        match = _TOP_LEVEL_TABLE_HEADER.match(line)
        if match is not None:
            categories[match.group(1)] = current_category
    return categories


_GUI_THEMES = ("dark", "light")
_SUCCESS_SIGNALS = ("episode_info", "return_only")
_SIDECAR_SCHEMA_VERSION = 1


def _check_fields(
    errors: list[str], where: str, table: dict, fields: dict[str, type]
) -> None:
    for key, expected in fields.items():
        if key not in table:
            continue
        value = table[key]
        valid = isinstance(value, expected) and not (
            expected is int and isinstance(value, bool)
        )
        if not valid:
            errors.append(
                f"{where}.{key}: expected {expected.__name__}, "
                f"got {type(value).__name__} {value!r}"
            )


def validate_sidecar() -> list[str]:
    """Return sidecar schema errors; return an empty list when valid.

    ``srb doc docgen --static-only`` checks schema version, known field types,
    table structure, GUI themes, and nonnegative ranks. Unknown keys are
    ignored. Name-to-registry checks require simulator-backed docgen;
    :func:`load_sidecar` instead falls back to defaults for invalid values.
    """
    errors: list[str] = []
    for path, kind in ((_ASSETS_TOML, "assets"), (_TASKS_TOML, "tasks")):
        if not path.is_file():
            errors.append(f"{path.name}: missing")
            continue
        try:
            raw = tomllib.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as exc:
            errors.append(f"{path.name}: unreadable: {exc}")
            continue
        version = raw.get("schema_version")
        if type(version) is not int or version != _SIDECAR_SCHEMA_VERSION:
            errors.append(
                f"{path.name}: schema_version must be {_SIDECAR_SCHEMA_VERSION}, "
                f"got {version!r}"
            )
        for name, entry in raw.items():
            where = f"{path.name}:{name}"
            if not isinstance(entry, dict):
                if name != "schema_version":
                    errors.append(f"{where}: expected a table")
                continue
            if kind == "assets":
                _check_fields(
                    errors,
                    where,
                    entry,
                    {
                        "tagline": str,
                        "video_id": str,
                        "notes": str,
                        "attribution": dict,
                    },
                )
                attribution = entry.get("attribution")
                if isinstance(attribution, dict):
                    _check_fields(
                        errors,
                        f"{where}.attribution",
                        attribution,
                        {"source": str, "license": str, "author": str},
                    )
                continue
            _check_fields(
                errors,
                where,
                entry,
                {"tagline": str, "video_id": str, "success_signal": str, "gui": dict},
            )
            success_signal = entry.get("success_signal")
            if (
                isinstance(success_signal, str)
                and success_signal not in _SUCCESS_SIGNALS
            ):
                errors.append(
                    f"{where}.success_signal: expected one of {_SUCCESS_SIGNALS}, "
                    f"got {success_signal!r}"
                )
            gui = entry.get("gui")
            if isinstance(gui, dict):
                _check_fields(
                    errors,
                    f"{where}.gui",
                    gui,
                    {"featured": bool, "rank": int, "theme": str},
                )
                if (
                    isinstance(gui.get("theme"), str)
                    and gui["theme"] not in _GUI_THEMES
                ):
                    errors.append(
                        f"{where}.gui.theme: expected one of {_GUI_THEMES}, "
                        f"got {gui['theme']!r}"
                    )
                rank = gui.get("rank")
                if isinstance(rank, int) and not isinstance(rank, bool) and rank < 0:
                    errors.append(f"{where}.gui.rank: must be nonnegative, got {rank}")
    return errors


def load_sidecar() -> SidecarData:
    """Load both sidecar TOML files, returning defaults for missing entries."""
    data = SidecarData()

    if _ASSETS_TOML.is_file():
        raw = tomllib.loads(_ASSETS_TOML.read_text(encoding="utf-8"))
        for name, entry in raw.items():
            if not isinstance(entry, dict):
                continue
            attr = entry.get("attribution")
            if not isinstance(attr, dict):
                attr = {}
            data.assets[name] = AssetMeta(
                tagline=_str_or_default(entry.get("tagline"), ""),
                video_id=_str_or_default(entry.get("video_id"), ""),
                source=_str_or_default(attr.get("source"), ""),
                license=_str_or_default(attr.get("license"), ""),
                author=_str_or_default(attr.get("author"), ""),
                notes=_str_or_default(entry.get("notes"), ""),
            )

    if _TASKS_TOML.is_file():
        tasks_toml_text = _TASKS_TOML.read_text(encoding="utf-8")
        categories = _task_categories_from_toml_text(tasks_toml_text)
        raw = tomllib.loads(tasks_toml_text)
        for name, entry in raw.items():
            if not isinstance(entry, dict):
                continue
            gui_entry = entry.get("gui", {})
            if not isinstance(gui_entry, dict):
                gui_entry = {}
            rank = _int_or_default(gui_entry.get("rank"), 1000)
            theme = _str_or_default(gui_entry.get("theme"), "dark")
            data.tasks[name] = TaskMeta(
                tagline=_str_or_default(entry.get("tagline"), ""),
                video_id=_str_or_default(entry.get("video_id"), ""),
                category=categories.get(name, "unknown"),
                gui=TaskGuiMeta(
                    featured=_bool_or_default(gui_entry.get("featured"), False),
                    # Values validate_sidecar rejects fall back to defaults; the
                    # GUI catalog deserialises rank as an unsigned integer.
                    rank=rank if rank >= 0 else 1000,
                    theme=theme if theme in _GUI_THEMES else "dark",
                ),
            )

    return data
