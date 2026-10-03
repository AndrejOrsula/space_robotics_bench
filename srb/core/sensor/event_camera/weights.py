"""Fetch and verify event-camera checkpoints.

The registered E2VID and V2CE mirrors provide checkpoints for the
reconstruction and simulation backends. Defaults are unpinned placeholders;
fetch with a registered mirror or provide ``--url`` and ``--sha256``. Files
are cached under ``~/.cache/srb/event_weights/`` after SHA256 verification.

Download, verification, and cache handling are shared with
:mod:`srb.utils.artifact_fetch`; this module defines the weight-specific
artifacts, mirrors, and errors.
"""

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
    compute_sha256,
    fetch_artifact,
)


@dataclass(frozen=True)
class WeightArtifact:
    """One downloadable weight checkpoint with integrity hash."""

    name: str
    url: str
    sha256: str
    size_bytes: int | None = None


# E2VID placeholder; a mirror can override its canonical upstream filename.
E2VID_DEFAULT = WeightArtifact(
    name="E2VID_lightweight.pth.tar",
    url=f"{_PLACEHOLDER_URL_PREFIX}e2vid_lightweight.pth.tar",
    sha256=_PLACEHOLDER_SHA256,
    size_bytes=None,
)

# V2CE placeholder; a mirror can override the upstream release filename.
V2CE_DEFAULT = WeightArtifact(
    name="v2ce_3d.pt",
    url=f"{_PLACEHOLDER_URL_PREFIX}v2ce_3d.pt",
    sha256=_PLACEHOLDER_SHA256,
    size_bytes=None,
)

# Mapping from CLI ``--artifact`` short-name to its canonical artifact.
ARTIFACT_DEFAULTS: dict[str, "WeightArtifact"] = {
    "e2vid": E2VID_DEFAULT,
    "v2ce": V2CE_DEFAULT,
}


@dataclass(frozen=True)
class WeightMirror:
    """Registered HTTPS mirror; URL fetch needs no consent or interactive auth.

    Private Hugging Face URLs may require ``HF_TOKEN``; the fetch path sends
    it as a bearer token. A registered SHA256 asserts that the mirror was
    downloaded and verified end to end. ``license_note`` records upstream
    licensing and attribution. See
    ``docs/src/sensors/event_camera_weight_mirrors.md`` for the contract.
    """

    short_name: str
    artifact: WeightArtifact
    license_note: str
    notes: str = ""


# Mirrors identify their artifact by ``WeightMirror.artifact.name``.
KNOWN_MIRRORS: tuple[WeightMirror, ...] = (
    WeightMirror(
        short_name="hf-andrejorsula-e2vid",
        artifact=WeightArtifact(
            name="E2VID_lightweight.pth.tar",
            url="https://huggingface.co/AndrejOrsula/srb-event-camera-weights/resolve/main/E2VID_lightweight.pth.tar",
            sha256="4cfeb2c850bf48fc9fa907e969cb8a04e3c51314da2d65bdb81145ac96574128",
            size_bytes=42_878_232,
        ),
        license_note="GPL-3.0; mirrored from uzh-rpg/rpg_e2vid lightweight checkpoint.",
        notes="HF mirror. Mirrored 2026-05-27 from http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar.",
    ),
    WeightMirror(
        short_name="hf-andrejorsula-v2ce",
        artifact=WeightArtifact(
            name="v2ce_3d.pt",
            url="https://huggingface.co/AndrejOrsula/srb-event-camera-weights/resolve/main/v2ce_3d.pt",
            sha256="97920e0c3f3c1df55f12800c166bf1de11466f7546bdb7b40261c5a14cddef26",
            size_bytes=211_737_165,
        ),
        license_note="MIT; mirrored from ucsd-hdsi-dvs/V2CE-Toolbox pretrained model.",
        notes="HF mirror. Mirrored 2026-05-27 from V2CE-Toolbox Google Drive artifact 1-aC6CTGZgAZk3snANZ46FAGNkPzu_Scw.",
    ),
)


class UnknownMirrorError(KeyError):
    """Raised when ``--mirror NAME`` does not resolve in :data:`KNOWN_MIRRORS`."""

    def __init__(self, short_name: str, known: tuple[str, ...]) -> None:
        self.short_name = short_name
        self.known = known
        if known:
            known_csv = ", ".join(known)
            detail = f"Known mirrors: {known_csv}."
        else:
            detail = (
                "No mirrors are registered yet. See "
                "docs/src/sensors/event_camera_weight_mirrors.md for the "
                "publishing contract, or pass --url + --sha256 manually."
            )
        super().__init__(f"Unknown mirror short_name={short_name!r}. {detail}")


def list_known_mirrors() -> tuple[WeightMirror, ...]:
    """Return the registered mirrors in declaration order."""
    return KNOWN_MIRRORS


def get_known_mirror(short_name: str) -> WeightMirror:
    """Look up a mirror by short name. Raises :class:`UnknownMirrorError`."""
    for mirror in KNOWN_MIRRORS:
        if mirror.short_name == short_name:
            return mirror
    raise UnknownMirrorError(short_name, tuple(m.short_name for m in KNOWN_MIRRORS))


class WeightPlaceholderError(PlaceholderError):
    """Raised when fetching an unpinned placeholder artifact.

    :data:`E2VID_DEFAULT` and :data:`V2CE_DEFAULT` are *named* placeholders,
    not fetchable artifacts. SRB does not redistribute these checkpoints, and
    the upstream Google Drive links are not HTTP-fetchable. Override both
    ``url`` and ``sha256`` before fetching.
    """

    def __init__(self, artifact: "WeightArtifact") -> None:
        message = (
            f"WeightArtifact {artifact.name!r} is an unpinned placeholder "
            f"(url={artifact.url!r}, sha256={artifact.sha256!r}). "
            "Mirror the checkpoint to an HTTP-fetchable host and pass "
            "--url + --sha256 on `srb event-camera fetch-weights`, or "
            "construct WeightArtifact(... url=..., sha256=...) directly."
        )
        super().__init__(artifact, message)


class WeightHashMismatchError(HashMismatchError):
    """Raised when a downloaded weight file fails SHA256 verification."""

    def __init__(self, *, expected: str, actual: str, path: Path) -> None:
        message = (
            f"Weight integrity check failed for {path}: "
            f"expected sha256={expected}, got sha256={actual}"
        )
        super().__init__(expected=expected, actual=actual, path=path, message=message)


def default_cache_dir() -> Path:
    """``~/.cache/srb/event_weights/`` (honors ``$XDG_CACHE_HOME``)."""
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    return Path(base) / "srb" / "event_weights"


def fetch_weight_artifact(
    artifact: WeightArtifact,
    *,
    cache_dir: Path | None = None,
    downloader: Callable[[str, Path], None] | None = None,
    force: bool = False,
) -> Path:
    """Fetch an artifact and verify SHA256; return its cached path.

    Matching cached files skip download unless ``force=True``. Mismatches
    remain untouched and raise :class:`WeightHashMismatchError`; remove them
    explicitly to retry.
    The injected ``downloader`` keeps network calls mockable. The default uses
    the module-bound :func:`_urllib_download`, so patches to that name take
    effect on the next call.
    """
    return fetch_artifact(
        artifact,
        cache_dir=(cache_dir or default_cache_dir()),
        downloader=downloader if downloader is not None else _urllib_download,
        force=force,
        placeholder_exc=WeightPlaceholderError,
        mismatch_exc=WeightHashMismatchError,
    )


__all__ = [
    "ARTIFACT_DEFAULTS",
    "E2VID_DEFAULT",
    "KNOWN_MIRRORS",
    "UnknownMirrorError",
    "V2CE_DEFAULT",
    "WeightArtifact",
    "WeightHashMismatchError",
    "WeightMirror",
    "WeightPlaceholderError",
    "compute_sha256",
    "default_cache_dir",
    "fetch_weight_artifact",
    "get_known_mirror",
    "list_known_mirrors",
]
