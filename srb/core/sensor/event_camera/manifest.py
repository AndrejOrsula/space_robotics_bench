"""Typed event-manifest rows and writers, shared by core and dataset tools.

The model depends only on publication and artifact-path helpers in
``srb.utils``; readers, validation, selection, and export remain in
``srb.datasets``.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass
from numbers import Integral
from pathlib import Path
from typing import Any

from srb.utils.artifact_paths import (
    resolve_artifact_member,
    validate_artifact_relative_path,
    validate_artifact_segment,
)
from srb.utils.publication import ArtifactPublication

EVENTS_MANIFEST_FILENAME = "events_manifest.jsonl"
SYNTHETIC_FIXTURE_TASK_ID = "fixture_moving_patch"
SYNTHETIC_FIXTURE_POLICY_ID = "synthetic_fixture"
_EVENT_SHA256_HEX = frozenset("0123456789abcdefABCDEF")


def _coerce_event_sha256(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, (bytes, bytearray)):
        return value.decode("ascii", errors="replace")
    return str(value)


def _require_event_sha256(value: object) -> str:
    digest = _coerce_event_sha256(value)
    if len(digest) != 64 or any(
        character not in _EVENT_SHA256_HEX for character in digest
    ):
        raise ValueError("event_sha256 must be 64 hexadecimal characters")
    return digest.lower()


@dataclass(frozen=True)
class EventManifestRow:
    """One JSONL row; provenance mirrors HDF5 attrs, paths locate episode data."""

    sensor_preset: str = ""
    resolution: tuple[int, int] = (0, 0)
    sub_render_hz: float = 0.0
    backend_name: str = ""
    backend_version: str = ""
    backend_commit_sha: str = ""
    representation: str = ""
    time_bins: int = 0
    episode_id: int = 0
    policy_id: str = ""
    policy_checkpoint_hash: str = ""
    task_id: str = ""
    seed: int = 0
    srb_git_sha: str = ""
    isaaclab_version: str = ""
    rgb_mp4_path: str = ""
    success_flag: int = 0
    reward_summary: str = ""
    sensor_name: str = ""
    env_id: int = 0
    event_hdf5_path: str = ""
    event_count: int = 0
    event_sha256: str = ""
    # Mirror HDF5 /metadata provenance in the self-describing JSONL row.
    hot_pixel_mask: tuple[tuple[int, int], ...] = ()
    hf_repo_ids: tuple[str, ...] = ()
    # ``None`` means unknown (the row predates the field).
    hot_pixel_rate_hz: float | None = None

    @classmethod
    def from_mapping(cls, row: Mapping[str, Any]) -> "EventManifestRow":
        """Coerce a JSON row into the typed manifest shape."""
        return cls(
            sensor_preset=str(row.get("sensor_preset", "")),
            resolution=_coerce_resolution(row.get("resolution", (0, 0))),
            sub_render_hz=float(row.get("sub_render_hz", 0.0) or 0.0),
            backend_name=str(row.get("backend_name", "")),
            backend_version=str(row.get("backend_version", "")),
            backend_commit_sha=str(row.get("backend_commit_sha", "")),
            representation=str(row.get("representation", "")),
            time_bins=int(row.get("time_bins", 0) or 0),
            episode_id=int(row.get("episode_id", 0) or 0),
            policy_id=str(row.get("policy_id", "")),
            policy_checkpoint_hash=str(row.get("policy_checkpoint_hash", "")),
            task_id=str(row.get("task_id", "")),
            seed=int(row.get("seed", 0) or 0),
            srb_git_sha=str(row.get("srb_git_sha", "")),
            isaaclab_version=str(row.get("isaaclab_version", "")),
            rgb_mp4_path=str(row.get("rgb_mp4_path", "")),
            success_flag=int(row.get("success_flag", 0) or 0),
            reward_summary=str(row.get("reward_summary", "")),
            sensor_name=str(row.get("sensor_name", "")),
            env_id=int(row.get("env_id", 0) or 0),
            event_hdf5_path=str(row.get("event_hdf5_path", "")),
            event_count=int(row.get("event_count", 0) or 0),
            event_sha256=_coerce_event_sha256(row.get("event_sha256", "")),
            hot_pixel_mask=_coerce_hot_pixel_mask(row.get("hot_pixel_mask", ())),
            hf_repo_ids=_coerce_hf_repo_ids(row.get("hf_repo_ids", ())),
            hot_pixel_rate_hz=_coerce_hot_pixel_rate_hz(row.get("hot_pixel_rate_hz")),
        )

    @classmethod
    def from_provenance(
        cls,
        provenance: object,
        *,
        sensor_name: str,
        env_id: int,
        event_hdf5_path: str,
        event_count: int,
        event_sha256: str,
    ) -> "EventManifestRow":
        """Build a manifest row from an event HDF5 provenance dataclass."""
        if hasattr(provenance, "__dataclass_fields__"):
            payload = asdict(provenance)
        elif isinstance(provenance, Mapping):
            payload = dict(provenance)
        else:
            payload = {}
        payload.update(
            {
                "sensor_name": sensor_name,
                "env_id": env_id,
                "event_hdf5_path": event_hdf5_path,
                "event_count": event_count,
                "event_sha256": _require_event_sha256(event_sha256),
            }
        )
        return cls.from_mapping(payload)

    def to_json_dict(self) -> dict[str, Any]:
        validate_artifact_segment(self.sensor_name, field="sensor_name")
        event_path = validate_artifact_relative_path(
            self.event_hdf5_path, field="event_hdf5_path"
        )
        if event_path.suffix.lower() not in {".h5", ".hdf5"}:
            raise ValueError("event_hdf5_path must end in .h5 or .hdf5")
        if self.rgb_mp4_path:
            validate_artifact_relative_path(self.rgb_mp4_path, field="rgb_mp4_path")
        if self.event_sha256:
            _require_event_sha256(self.event_sha256)
        if any(
            len(pair) != 2
            or any(
                isinstance(coordinate, bool)
                or not isinstance(coordinate, int)
                or coordinate < 0
                for coordinate in pair
            )
            for pair in self.hot_pixel_mask
        ):
            raise ValueError("hot_pixel_mask must contain exact integer [x, y] pairs")
        payload = asdict(self)
        payload["resolution"] = list(self.resolution)
        # Keep provenance as JSON arrays, matching HDF5 /metadata.
        payload["hot_pixel_mask"] = [list(pair) for pair in self.hot_pixel_mask]
        payload["hf_repo_ids"] = list(self.hf_repo_ids)
        return payload

    def hdf5_path(self, root: Path) -> Path:
        path = validate_artifact_relative_path(
            self.event_hdf5_path, field="event_hdf5_path"
        )
        if path.suffix.lower() not in {".h5", ".hdf5"}:
            raise ValueError("event_hdf5_path must end in .h5 or .hdf5")
        return resolve_artifact_member(
            root,
            self.event_hdf5_path,
            field="event_hdf5_path",
        )

    def reward_summary_payload(self) -> dict[str, Any]:
        """Parse ``reward_summary`` as a strict JSON object."""
        return parse_event_reward_summary(self.reward_summary)


def create_event_manifest(run_dir: Path | str) -> Path:
    """Create an empty manifest skeleton and return its path."""
    manifest_path = Path(run_dir) / EVENTS_MANIFEST_FILENAME
    publication = ArtifactPublication.file(manifest_path)
    publication.append_bytes(b"")
    return publication.path


def append_event_manifest_row(
    manifest_path: Path | str,
    row: EventManifestRow | Mapping[str, Any],
) -> None:
    """Append one typed manifest row to ``events_manifest.jsonl``."""
    publication = ArtifactPublication.file(manifest_path)
    if isinstance(row, EventManifestRow):
        manifest_row = row
    else:
        manifest_row = EventManifestRow.from_mapping(row)
    line = (json.dumps(manifest_row.to_json_dict()) + "\n").encode("utf-8")
    publication.append_bytes(line)


def append_event_manifest_rows(
    manifest_path: str | Path,
    rows: Iterable[EventManifestRow | Mapping[str, Any]],
) -> None:
    """Append one ordered manifest batch with a single publication operation."""

    encoded_rows: list[bytes] = []
    for row in rows:
        manifest_row = (
            row
            if isinstance(row, EventManifestRow)
            else EventManifestRow.from_mapping(row)
        )
        encoded_rows.append(
            (json.dumps(manifest_row.to_json_dict()) + "\n").encode("utf-8")
        )
    if encoded_rows:
        ArtifactPublication.file(Path(manifest_path)).append_bytes(
            b"".join(encoded_rows)
        )


def parse_event_reward_summary(reward_summary: str) -> dict[str, Any]:
    """Parse strict JSON; empty means absent, and NaN/Infinity are rejected."""
    if not reward_summary.strip():
        return {}
    try:
        payload = json.loads(
            reward_summary,
            parse_constant=_reject_json_constant,
        )
    except ValueError as exc:
        if "non-standard JSON constant" in str(exc):
            raise
        raise ValueError("invalid reward_summary JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("reward_summary must be a JSON object")
    return payload


def _coerce_resolution(value: Any) -> tuple[int, int]:
    if isinstance(value, str):
        left, _, right = value.lower().partition("x")
        if left and right:
            return (int(left), int(right))
        return (0, 0)
    try:
        seq = tuple(value)
    except TypeError:
        return (0, 0)
    if len(seq) < 2:
        return (0, 0)
    return (int(seq[0]), int(seq[1]))


def _coerce_hot_pixel_mask(value: Any) -> tuple[tuple[int, int], ...]:
    """Coerce JSON or native coordinate pairs; skip malformed entries."""
    try:
        outer = tuple(value)
    except TypeError:
        return ()
    coerced: list[tuple[int, int]] = []
    for pair in outer:
        try:
            seq = tuple(pair)
        except TypeError:
            continue
        if len(seq) != 2:
            continue
        if any(
            isinstance(coordinate, bool)
            or not isinstance(coordinate, Integral)
            or coordinate < 0
            for coordinate in seq
        ):
            continue
        coerced.append((int(seq[0]), int(seq[1])))
    return tuple(coerced)


def _coerce_hot_pixel_rate_hz(value: Any) -> float | None:
    """Coerce ``hot_pixel_rate_hz``; absent/``None`` stays ``None`` (unknown)."""
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"hot_pixel_rate_hz must be a number; got {value!r}")
    try:
        rate = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"hot_pixel_rate_hz must be a number; got {value!r}") from exc
    if not math.isfinite(rate) or rate < 0:
        raise ValueError(f"hot_pixel_rate_hz must be finite and >= 0; got {value!r}")
    return rate


def _coerce_hf_repo_ids(value: Any) -> tuple[str, ...]:
    """Coerce repo IDs to strings; tuple order does not indicate a primary mirror."""
    if isinstance(value, str):
        # Tolerate legacy scalar input.
        return (value,) if value else ()
    try:
        seq = tuple(value)
    except TypeError:
        return ()
    return tuple(str(item) for item in seq if item)


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON constant {value}")
