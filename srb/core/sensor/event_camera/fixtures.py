"""Fetch and verify event-camera reference fixtures."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from srb.utils.artifact_fetch import (
    _PLACEHOLDER_SHA256,
    _PLACEHOLDER_URL_PREFIX,
    HashMismatchError,
    PlaceholderError,
    _urllib_download,
    fetch_artifact,
)


@dataclass(frozen=True)
class FixtureArtifact:
    """Metadata for a downloadable event-camera fixture.

    Consumers must match ``schema_version`` to the ``EVENT_HDF5`` reader.
    """

    name: str
    url: str
    sha256: str
    size_bytes: int | None = None
    schema_version: int = 1
    sensor_model: str = "prophesee_gen4_hd"
    num_episodes: int | None = None


GEN4_REFERENCE_DEFAULT = FixtureArtifact(
    name="gen4_reference.h5",
    url=f"{_PLACEHOLDER_URL_PREFIX}gen4_reference.h5",
    sha256=_PLACEHOLDER_SHA256,
    size_bytes=None,
    schema_version=1,
    sensor_model="prophesee_gen4_hd",
    num_episodes=None,
)


# Mapping from CLI ``--artifact`` short-name to its canonical artifact.
FIXTURE_DEFAULTS: dict[str, "FixtureArtifact"] = {
    "gen4": GEN4_REFERENCE_DEFAULT,
}


@dataclass(frozen=True)
class FixtureMirror:
    """HTTP-fetchable fixture mirror with verified hash and license metadata.

    Entries must serve a matching ``EVENT_HDF5`` schema and document capture
    attribution. See the fixture mirror publishing contract for details.
    """

    short_name: str
    artifact: FixtureArtifact
    license_note: str
    notes: str = ""


# HTTP-fetchable mirrors for event-camera fixtures.
KNOWN_FIXTURES: tuple[FixtureMirror, ...] = (
    FixtureMirror(
        short_name="hf-andrejorsula-active-marker-gen4",
        artifact=FixtureArtifact(
            name="active_marker.h5",
            url="https://huggingface.co/datasets/AndrejOrsula/srb-event-camera-fixtures/resolve/main/prophesee/active_marker/v2/active_marker.h5",
            sha256="a07d44b16440347b1b6c3a7cc86b16eee11ad417548cb385c2704dc79cf12c5a",
            size_bytes=57_811_264,
            schema_version=1,
            sensor_model="prophesee_gen4_hd",
            num_episodes=1,
        ),
        license_note=(
            "Prophesee active_marker capture; Prophesee terms govern redistribution."
        ),
        notes=(
            "HF mirror. Converted 2026-10-11 from Prophesee "
            "active_marker RAW EVT3 KDrive ZIP to SRB EVENT_HDF5 schema v1."
        ),
    ),
)


class UnknownFixtureError(KeyError):
    """Raised when ``--mirror NAME`` is not registered."""

    def __init__(self, short_name: str, known: tuple[str, ...]) -> None:
        self.short_name = short_name
        self.known = known
        if known:
            known_csv = ", ".join(known)
            detail = (
                f"Known fixtures: {known_csv}. See "
                "docs/src/sensors/event_camera_fixture_mirrors.md for the "
                "publishing contract."
            )
        else:
            detail = (
                "No fixtures are registered yet. See "
                "docs/src/sensors/event_camera_fixture_mirrors.md for the "
                "publishing contract, or pass --url + --sha256 manually."
            )
        super().__init__(f"Unknown fixture short_name={short_name!r}. {detail}")


def list_known_fixtures() -> tuple[FixtureMirror, ...]:
    """Return registered fixture mirrors in declaration order."""
    return KNOWN_FIXTURES


def get_known_fixture(short_name: str) -> FixtureMirror:
    """Look up a mirror by short name."""
    for fixture in KNOWN_FIXTURES:
        if fixture.short_name == short_name:
            return fixture
    raise UnknownFixtureError(short_name, tuple(f.short_name for f in KNOWN_FIXTURES))


class FixturePlaceholderError(PlaceholderError):
    """Raised when fetching an unpinned placeholder fixture."""

    def __init__(self, artifact: "FixtureArtifact") -> None:
        message = (
            f"FixtureArtifact {artifact.name!r} is an unpinned placeholder "
            f"(url={artifact.url!r}, sha256={artifact.sha256!r}). "
            "Mirror the capture to an HTTP-fetchable host and pass "
            "--url + --sha256 on `srb event-camera fetch-fixture`, or "
            "construct FixtureArtifact(... url=..., sha256=...) directly. "
            "See docs/src/sensors/event_camera_fixture_mirrors.md."
        )
        super().__init__(artifact, message)


class FixtureHashMismatchError(HashMismatchError):
    """Raised when a fixture fails SHA256 verification."""

    def __init__(self, *, expected: str, actual: str, path: Path) -> None:
        message = (
            f"Fixture integrity check failed for {path}: "
            f"expected sha256={expected}, got sha256={actual}"
        )
        super().__init__(expected=expected, actual=actual, path=path, message=message)


def default_fixture_cache_dir() -> Path:
    """Return the fixture cache path, honoring ``$XDG_CACHE_HOME``."""
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    return Path(base) / "srb" / "event_fixtures"


def fetch_fixture(
    artifact: FixtureArtifact,
    *,
    cache_dir: Path | None = None,
    downloader: Callable[[str, Path], None] | None = None,
    force: bool = False,
) -> Path:
    """Fetch a fixture, verify its SHA256, and return its cached path.

    Matching cached files are reused; mismatches remain untouched. The
    injected downloader keeps network access mockable.
    """
    return fetch_artifact(
        artifact,
        cache_dir=(cache_dir or default_fixture_cache_dir()),
        downloader=downloader if downloader is not None else _urllib_download,
        force=force,
        placeholder_exc=FixturePlaceholderError,
        mismatch_exc=FixtureHashMismatchError,
    )


__all__ = [
    "FIXTURE_DEFAULTS",
    "FixtureArtifact",
    "FixtureHashMismatchError",
    "FixtureMirror",
    "FixturePlaceholderError",
    "GEN4_REFERENCE_DEFAULT",
    "KNOWN_FIXTURES",
    "UnknownFixtureError",
    "default_fixture_cache_dir",
    "fetch_fixture",
    "get_known_fixture",
    "list_known_fixtures",
]
