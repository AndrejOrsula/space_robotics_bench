"""Export canonical event-camera HDF5 runs to LeRobot, Zenodo, EVT3, ROS bag, or NPZ."""

from __future__ import annotations

import json
import re
import shutil
import tarfile
import warnings
from collections.abc import Callable, Iterable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from srb.core.sensor.event_camera.extras import require_extra
from srb.core.sensor.event_camera.manifest import (
    SYNTHETIC_FIXTURE_POLICY_ID,
    SYNTHETIC_FIXTURE_TASK_ID,
)
from srb.datasets._evt3 import (
    Evt3Decoder,
    Evt3Encoder,
    Evt3Event,
    Evt3ProtocolError,
    pack_word_bytes,
    words_to_bytes,
)
from srb.datasets.event_camera_run import EventCameraRunArtifact
from srb.utils.publication import ArtifactPublication, PublicationSnapshot
from srb.utils.resource_budget import (
    MAX_EXTERNAL_FILE_BYTES,
    MAX_METADATA_ATTRIBUTE_BYTES,
)

_LEROBOT_CODEBASE_VERSION = "v3.0"
_LEROBOT_EVENT_PATH_TEMPLATE = (
    "events/{sensor_key}/chunk-{chunk_index:03d}/episode-{episode_index:06d}.h5"
)
_LEROBOT_DATA_PATH_TEMPLATE = (
    "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
)
_LEROBOT_EVENT_SCHEMA_VERSION = 1


def _publish_file(
    path: Path,
    writer: Callable[[Path], object],
    validator: Callable[[Path], object],
) -> Path:
    """Build, validate, and publish one event artifact through its stage."""

    def build(staging: Path) -> None:
        writer(staging)
        validator(staging)

    published = ArtifactPublication.file(path).publish(build)
    return cast(Path, published)


@contextmanager
def _open_published_file(path: Path) -> Iterator[PublicationSnapshot]:
    """Yield a validated snapshot while holding the publication lock.

    Streaming avoids loading large HDF5 members into memory; the lock keeps
    atomic replacement from changing bytes exposed by the descriptor.
    """

    with ArtifactPublication.file(path).open_snapshot(
        max_bytes=MAX_EXTERNAL_FILE_BYTES
    ) as snapshot:
        if snapshot is None:
            raise FileNotFoundError(path)
        yield snapshot


def _copy_published_file(source: Path, destination: Path) -> None:
    """Stream one published file to ``destination`` in bounded chunks."""

    with _open_published_file(source) as snapshot:
        with destination.open("wb") as sink:
            shutil.copyfileobj(snapshot.file, sink, length=1 << 20)


def _add_published_file(tar: tarfile.TarFile, arcname: str, source: Path) -> None:
    """Append one published file to ``tar`` without buffering it in memory."""

    with _open_published_file(source) as snapshot:
        info = tarfile.TarInfo(name=arcname)
        info.size = snapshot.size_bytes
        tar.addfile(info, snapshot.file)


@dataclass(frozen=True)
class LeRobotBundleResult:
    """Summary of one LeRobot v3 event-camera bundle export."""

    output_dir: Path
    num_episodes: int
    total_event_count: int
    sensors: tuple[str, ...]


def to_lerobot_v3_bundle(
    event_hdf5_dir: Path | str,
    output_dir: Path | str,
    *,
    episodes: Iterable[str] | None = None,
    extra_metadata: dict[str, Any] | None = None,
) -> LeRobotBundleResult:
    """Export a manifest-backed run as LeRobot v3 tables plus event sidecars.

    ``episodes`` selects manifest filenames. Each episode has one table row;
    the ``event_camera`` extension records sidecar paths and counts.
    """
    src = Path(event_hdf5_dir)
    if not src.is_dir():
        raise FileNotFoundError(f"event_hdf5_dir must exist: {src}")
    requested_episodes = _normalize_episode_filter(episodes)
    run = EventCameraRunArtifact.load(src)
    selection = run.select(episodes=requested_episodes, validate_hdf5=True)
    _raise_event_run_validation_errors(
        run,
        episodes=requested_episodes,
    )
    manifest_rows = [row.to_json_dict() for row in selection.rows]
    fixture_rows = [row for row in manifest_rows if _is_synthetic_fixture(row)]
    _warn_fixture_export("LeRobot", fixture_rows)

    _require_pyarrow()

    import pandas as pd

    out = Path(output_dir)

    tasks = sorted({str(row.get("task_id", "")) for row in manifest_rows})
    task_to_index = {task: idx for idx, task in enumerate(tasks)}

    sensors = list(selection.sensors)
    total_event_count = 0
    episode_rows: list[dict[str, Any]] = []
    frame_rows: list[dict[str, Any]] = []
    event_sources: list[tuple[Path, str]] = []

    for episode_index, (row, src_h5) in enumerate(
        zip(manifest_rows, selection.hdf5_paths, strict=True)
    ):
        sensor_name = str(row["sensor_name"])
        rel_event_path = _LEROBOT_EVENT_PATH_TEMPLATE.format(
            sensor_key=sensor_name,
            chunk_index=0,
            episode_index=episode_index,
        )
        event_sources.append((src_h5, rel_event_path))

        event_count = int(row.get("event_count", 0))
        total_event_count += event_count
        task_index = task_to_index[str(row.get("task_id", ""))]
        episode_rows.append(
            _build_episode_row(
                row=row,
                episode_index=episode_index,
                task_index=task_index,
                event_path=rel_event_path,
                data_from=episode_index,
                data_to=episode_index + 1,
            )
        )
        frame_rows.append(
            _build_frame_row(
                row=row,
                episode_index=episode_index,
                task_index=task_index,
                event_path=rel_event_path,
                event_count=event_count,
                sensors=sensors,
                sensor_name=sensor_name,
            )
        )

    info_payload = _lerobot_info_payload(
        manifest_rows=manifest_rows,
        sensors=sensors,
        total_episodes=len(episode_rows),
        total_frames=len(frame_rows),
        extra_metadata=extra_metadata,
        fixture_rows=fixture_rows,
    )

    required_members = [
        "meta/info.json",
        "meta/stats.json",
        "meta/tasks.parquet",
        "meta/episodes/chunk-000/file-000.parquet",
        "data/chunk-000/file-000.parquet",
        *(rel_path for _source, rel_path in event_sources),
    ]

    def build_bundle(staging: Path) -> None:
        meta_dir = staging / "meta"
        episodes_dir = meta_dir / "episodes" / "chunk-000"
        data_dir = staging / "data" / "chunk-000"
        for directory in (meta_dir, episodes_dir, data_dir):
            directory.mkdir(parents=True, exist_ok=True)

        for source, relative_path in event_sources:
            destination = staging / relative_path
            destination.parent.mkdir(parents=True, exist_ok=True)
            _copy_published_file(source, destination)

        pd.DataFrame(frame_rows).to_parquet(
            data_dir / "file-000.parquet",
            index=False,
        )
        pd.DataFrame(episode_rows).to_parquet(
            episodes_dir / "file-000.parquet",
            index=False,
        )
        pd.DataFrame(
            [{"task_index": idx, "task": task} for task, idx in task_to_index.items()]
        ).to_parquet(meta_dir / "tasks.parquet", index=False)
        (meta_dir / "stats.json").write_bytes(b"{}\n")
        (meta_dir / "info.json").write_bytes(
            (json.dumps(info_payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
        )
        _validate_lerobot_bundle_stage(
            staging,
            required_members=required_members,
            manifest_rows=manifest_rows,
            sensors=sensors,
            total_episodes=len(episode_rows),
            total_frames=len(frame_rows),
            task_count=len(task_to_index),
        )

    ArtifactPublication.directory(out).publish(
        build_bundle,
        required_members=required_members,
    )

    return LeRobotBundleResult(
        output_dir=out,
        num_episodes=len(episode_rows),
        total_event_count=total_event_count,
        sensors=tuple(sensors),
    )


def _normalize_episode_filter(
    episodes: Iterable[str] | None,
) -> tuple[str, ...] | None:
    if episodes is None:
        return None
    return tuple(str(episode) for episode in episodes)


def _raise_event_run_validation_errors(
    run: EventCameraRunArtifact,
    *,
    episodes: Iterable[str] | None,
) -> None:
    report = run.validate(episodes=episodes)
    report.raise_for_errors()


def _build_episode_row(
    *,
    row: dict[str, Any],
    episode_index: int,
    task_index: int,
    event_path: str,
    data_from: int,
    data_to: int,
) -> dict[str, Any]:
    sensor_name = str(row["sensor_name"])
    return {
        "episode_index": episode_index,
        "length": 1,
        "tasks": [str(row.get("task_id", ""))],
        "task_index": task_index,
        "data_from": data_from,
        "data_to": data_to,
        "sensor_name": sensor_name,
        "env_id": int(row.get("env_id", 0)),
        "srb_episode_id": int(row.get("episode_id", 0)),
        f"observation.events.{sensor_name}.count": int(row.get("event_count", 0)),
        f"observation.events.{sensor_name}.path": event_path,
        "seed": int(row.get("seed", 0)),
        "success_flag": int(row.get("success_flag", 0)),
        "policy_id": str(row.get("policy_id", "")),
        "policy_checkpoint_hash": str(row.get("policy_checkpoint_hash", "")),
        "srb_git_sha": str(row.get("srb_git_sha", "")),
        "isaaclab_version": str(row.get("isaaclab_version", "")),
        "backend_name": str(row.get("backend_name", "")),
        "backend_version": str(row.get("backend_version", "")),
        "backend_commit_sha": str(row.get("backend_commit_sha", "")),
        "sensor_preset": str(row.get("sensor_preset", "")),
        "resolution": list(row.get("resolution", [0, 0])),
        "representation": str(row.get("representation", "")),
        "time_bins": int(row.get("time_bins", 0)),
        "sub_render_hz": float(row.get("sub_render_hz", 0.0)),
        "reward_summary": str(row.get("reward_summary", "")),
        "rgb_mp4_path": str(row.get("rgb_mp4_path", "")),
    }


def _build_frame_row(
    *,
    row: dict[str, Any],
    episode_index: int,
    task_index: int,
    event_path: str,
    event_count: int,
    sensors: list[str],
    sensor_name: str,
) -> dict[str, Any]:
    """Build a rectangular LeRobot row with per-sensor sidecar pointers.

    Events are not frame-synced, so each episode uses one aggregate row at
    ``frame_index = 0``; absent sensors receive empty/zero values.
    """
    frame: dict[str, Any] = {
        "episode_index": episode_index,
        "frame_index": 0,
        "timestamp": 0.0,
        "task_index": task_index,
    }
    for s in sensors:
        if s == sensor_name:
            frame[f"observation.events.{s}.count"] = event_count
            frame[f"observation.events.{s}.path"] = event_path
        else:
            frame[f"observation.events.{s}.count"] = 0
            frame[f"observation.events.{s}.path"] = ""
    return frame


def _lerobot_info_payload(
    *,
    manifest_rows: list[dict[str, Any]],
    sensors: list[str],
    total_episodes: int,
    total_frames: int,
    extra_metadata: dict[str, Any] | None,
    fixture_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    # Use the maximum source rate because LeRobot's ``fps`` describes dataset
    # resolution, including heterogeneous task bundles.
    fps = 0.0
    for row in manifest_rows:
        fps = max(fps, float(row.get("sub_render_hz", 0.0)))

    features: dict[str, dict[str, Any]] = {
        "episode_index": {"dtype": "int64", "shape": [1]},
        "frame_index": {"dtype": "int64", "shape": [1]},
        "timestamp": {"dtype": "float32", "shape": [1]},
        "task_index": {"dtype": "int64", "shape": [1]},
    }
    for sensor_name in sensors:
        features[f"observation.events.{sensor_name}.count"] = {
            "dtype": "int64",
            "shape": [1],
        }
        features[f"observation.events.{sensor_name}.path"] = {
            "dtype": "string",
            "shape": [1],
        }

    payload: dict[str, Any] = {
        "codebase_version": _LEROBOT_CODEBASE_VERSION,
        "robot_type": "srb",
        "fps": fps,
        "data_path": _LEROBOT_DATA_PATH_TEMPLATE,
        "event_path": _LEROBOT_EVENT_PATH_TEMPLATE,
        "features": features,
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "srb_event_camera": {
            "schema_version": _LEROBOT_EVENT_SCHEMA_VERSION,
            "sensors": sensors,
            "presets": sorted(
                {str(row.get("sensor_preset", "")) for row in manifest_rows}
            ),
            "backends": sorted(
                {str(row.get("backend_name", "")) for row in manifest_rows}
            ),
            "resolutions": sorted(
                {tuple(row.get("resolution", [0, 0])) for row in manifest_rows}
            ),
        },
    }
    if extra_metadata:
        payload.update(extra_metadata)
    if fixture_rows:
        camera_metadata = payload.get("srb_event_camera")
        if not isinstance(camera_metadata, dict):
            camera_metadata = {}
        camera_metadata = dict(camera_metadata)
        camera_metadata.update(
            {
                "schema_version": _LEROBOT_EVENT_SCHEMA_VERSION,
                "sensors": sensors,
                "presets": sorted(
                    {str(row.get("sensor_preset", "")) for row in manifest_rows}
                ),
                "backends": sorted(
                    {str(row.get("backend_name", "")) for row in manifest_rows}
                ),
                "resolutions": sorted(
                    {tuple(row.get("resolution", [0, 0])) for row in manifest_rows}
                ),
                "contains_synthetic_fixture": True,
                "fixture_task_ids": [SYNTHETIC_FIXTURE_TASK_ID],
                "fixture_policy_ids": [SYNTHETIC_FIXTURE_POLICY_ID],
            }
        )
        payload["srb_event_camera"] = camera_metadata
    return payload


def _validate_lerobot_bundle_stage(
    root: Path,
    *,
    required_members: list[str],
    manifest_rows: list[dict[str, Any]],
    sensors: list[str],
    total_episodes: int,
    total_frames: int,
    task_count: int,
) -> None:
    """Validate the complete LeRobot tree before directory promotion."""

    required = set(required_members)
    actual_files: set[str] = set()
    actual_directories: set[str] = set()
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"LeRobot bundle contains symlink: {path}")
        relative = path.relative_to(root).as_posix()
        if path.is_dir():
            actual_directories.add(relative)
        elif path.is_file():
            actual_files.add(relative)
        else:
            raise ValueError(f"LeRobot bundle contains unsupported member: {path}")

    if actual_files != required:
        missing = sorted(required - actual_files)
        unexpected = sorted(actual_files - required)
        raise ValueError(
            "LeRobot bundle file tree mismatch: "
            f"missing={missing}, unexpected={unexpected}"
        )

    expected_directories: set[str] = set()
    for member in required_members:
        parent = Path(member).parent
        while parent != Path("."):
            expected_directories.add(parent.as_posix())
            parent = parent.parent
    if actual_directories != expected_directories:
        missing = sorted(expected_directories - actual_directories)
        unexpected = sorted(actual_directories - expected_directories)
        raise ValueError(
            "LeRobot bundle directory tree mismatch: "
            f"missing={missing}, unexpected={unexpected}"
        )

    info_path = root / "meta" / "info.json"
    try:
        info = json.loads(info_path.read_bytes())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"LeRobot info.json is malformed: {exc}") from exc
    if not isinstance(info, dict):
        raise ValueError("LeRobot info.json must contain a JSON object")

    required_info_keys = {
        "codebase_version",
        "robot_type",
        "fps",
        "data_path",
        "event_path",
        "features",
        "total_episodes",
        "total_frames",
        "srb_event_camera",
    }
    if not required_info_keys.issubset(info):
        raise ValueError("LeRobot info.json is missing required schema fields")
    if info["codebase_version"] != _LEROBOT_CODEBASE_VERSION:
        raise ValueError("LeRobot info.json has an unsupported codebase version")
    if info["robot_type"] != "srb":
        raise ValueError("LeRobot info.json has an unsupported robot_type")
    if info["total_episodes"] != total_episodes or info["total_frames"] != total_frames:
        raise ValueError("LeRobot info.json episode/frame counts do not match tables")

    features = info["features"]
    if not isinstance(features, dict):
        raise ValueError("LeRobot info.json features must be an object")
    expected_features = {
        "episode_index",
        "frame_index",
        "timestamp",
        "task_index",
        *(f"observation.events.{sensor}.count" for sensor in sensors),
        *(f"observation.events.{sensor}.path" for sensor in sensors),
    }
    if set(features) != expected_features:
        raise ValueError("LeRobot info.json features do not match the data schema")
    expected_dtypes = {
        "episode_index": "int64",
        "frame_index": "int64",
        "timestamp": "float32",
        "task_index": "int64",
        **{f"observation.events.{sensor}.count": "int64" for sensor in sensors},
        **{f"observation.events.{sensor}.path": "string" for sensor in sensors},
    }
    for feature_name, descriptor in features.items():
        if not isinstance(descriptor, dict) or set(descriptor) != {"dtype", "shape"}:
            raise ValueError(f"LeRobot feature descriptor is malformed: {feature_name}")
        if descriptor["dtype"] != expected_dtypes[feature_name]:
            raise ValueError(f"LeRobot feature dtype is malformed: {feature_name}")
        if descriptor["shape"] != [1]:
            raise ValueError(f"LeRobot feature has unexpected shape: {feature_name}")

    camera_schema = info["srb_event_camera"]
    if not isinstance(camera_schema, dict):
        raise ValueError("LeRobot srb_event_camera schema must be an object")
    if not {
        "schema_version",
        "sensors",
        "presets",
        "backends",
        "resolutions",
    }.issubset(camera_schema):
        raise ValueError("LeRobot event-camera schema is missing required fields")
    if camera_schema.get("schema_version") != _LEROBOT_EVENT_SCHEMA_VERSION:
        raise ValueError("LeRobot event-camera schema version is unsupported")
    if camera_schema.get("sensors") != sensors:
        raise ValueError("LeRobot event-camera sensor list does not match the bundle")
    if info["data_path"] != _LEROBOT_DATA_PATH_TEMPLATE:
        raise ValueError("LeRobot data_path template is malformed")
    if info["event_path"] != _LEROBOT_EVENT_PATH_TEMPLATE:
        raise ValueError("LeRobot event_path template is malformed")

    try:
        stats = json.loads((root / "meta" / "stats.json").read_bytes())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"LeRobot stats.json is malformed: {exc}") from exc
    if stats != {}:
        raise ValueError("LeRobot stats.json must be the empty placeholder object")

    import pandas as pd

    def parquet_columns(path: Path) -> set[str]:
        try:
            frame = pd.read_parquet(path)
        except Exception as exc:  # noqa: BLE001 - normalize malformed artifacts.
            raise ValueError(f"LeRobot parquet is malformed: {path}: {exc}") from exc
        return {str(column) for column in frame.columns}

    data_columns = {
        "episode_index",
        "frame_index",
        "timestamp",
        "task_index",
        *(f"observation.events.{sensor}.count" for sensor in sensors),
        *(f"observation.events.{sensor}.path" for sensor in sensors),
    }
    if (
        parquet_columns(root / "data" / "chunk-000" / "file-000.parquet")
        != data_columns
    ):
        raise ValueError("LeRobot data parquet schema does not match info.json")

    episode_columns = {
        "episode_index",
        "length",
        "tasks",
        "task_index",
        "data_from",
        "data_to",
        "sensor_name",
        "env_id",
        "srb_episode_id",
        "seed",
        "success_flag",
        "policy_id",
        "policy_checkpoint_hash",
        "srb_git_sha",
        "isaaclab_version",
        "backend_name",
        "backend_version",
        "backend_commit_sha",
        "sensor_preset",
        "resolution",
        "representation",
        "time_bins",
        "sub_render_hz",
        "reward_summary",
        "rgb_mp4_path",
        *(f"observation.events.{sensor}.count" for sensor in sensors),
        *(f"observation.events.{sensor}.path" for sensor in sensors),
    }
    if (
        parquet_columns(root / "meta" / "episodes" / "chunk-000" / "file-000.parquet")
        != episode_columns
    ):
        raise ValueError("LeRobot episode parquet schema is malformed")

    if parquet_columns(root / "meta" / "tasks.parquet") != {
        "task_index",
        "task",
    }:
        raise ValueError("LeRobot tasks parquet schema is malformed")

    if len(manifest_rows) != total_episodes or total_frames != total_episodes:
        raise ValueError("LeRobot table counts are inconsistent with the manifest")
    if task_count != len({str(row.get("task_id", "")) for row in manifest_rows}):
        raise ValueError("LeRobot task table count does not match the manifest")


def _require_pyarrow() -> None:
    """Raise an actionable error when the Parquet dependency is unavailable."""
    import importlib.util

    if importlib.util.find_spec("pyarrow") is None:
        raise RuntimeError(
            "LeRobot v3 event-camera bundle requires pyarrow for Parquet "
            "output. Install pyarrow (already a core SRB dependency)."
        )


_DEFAULT_LICENSE = "CC-BY-4.0"


@dataclass(frozen=True)
class ZenodoBundleResult:
    """Summary of one Zenodo bundle export."""

    output_path: Path
    file_count: int
    license: str
    doi: str | None
    citation_md: Path | None


def to_zenodo_bundle(
    event_hdf5_dir: Path | str,
    output_path: Path | str,
    *,
    citation_md: Path | str | None = None,
    license: str = _DEFAULT_LICENSE,
    doi: str | None = None,
    episodes: Iterable[str] | None = None,
    extra_metadata: dict[str, Any] | None = None,
) -> ZenodoBundleResult:
    """Create a Zenodo ``.tar.gz`` with selected episodes and metadata.

    ``event_hdf5_dir`` may be a manifest run directory or a directory of
    HDF5 files. ``episodes`` selects filenames; ``citation_md`` supplies
    ``README.md``. Manifest rows are archived byte-for-byte so extraction
    preserves provenance validated against EVENT_HDF5 metadata. A non-file
    ``citation_md`` raises ``FileNotFoundError`` before publication.
    """
    src = Path(event_hdf5_dir)
    if not src.is_dir():
        raise FileNotFoundError(f"event_hdf5_dir must exist: {src}")
    citation_path: Path | None = None
    citation_text: str | None = None
    if citation_md is not None:
        citation_path = Path(citation_md)
        if not citation_path.is_file():
            raise FileNotFoundError(
                f"citation_md must be a readable file: {citation_path}"
            )
        citation_text = ArtifactPublication.file(citation_path).read(
            lambda raw: raw.decode("utf-8"),
            max_bytes=MAX_METADATA_ATTRIBUTE_BYTES,
        )
        if citation_text is None:
            raise FileNotFoundError(
                f"citation_md must be a readable file: {citation_path}"
            )

    out = Path(output_path)
    if out.suffix not in (".tar.gz", ".tgz") and not str(out).endswith(".tar.gz"):
        out = out.with_name(out.name + ".tar.gz")

    requested_episodes = _normalize_episode_filter(episodes)
    run = EventCameraRunArtifact.load(src, require_manifest=False)
    manifest_src = run.manifest_path
    selected_manifest_lines: list[str] = []
    manifest_reward_summary: dict[str, Any] = {}
    selected_hf_repo_ids: tuple[str, ...] = ()
    fixture_rows: list[dict[str, Any]] = []
    if run.has_manifest:
        selection = run.select(
            episodes=requested_episodes,
            validate_hdf5=True,
        )
        _raise_event_run_validation_errors(
            run,
            episodes=requested_episodes,
        )
        hdf5_files = list(selection.hdf5_paths)
        archive_hdf5_files = list(
            zip(
                hdf5_files,
                (row.event_hdf5_path for row in selection.rows),
                strict=True,
            )
        )
        first_row = selection.summary().first_row
        # Preserve row-level ids; publish their union in bundle metadata.
        selected_hf_repo_ids = tuple(
            sorted(
                {
                    *selection.hf_repo_ids,
                    *(repo for row in selection.rows for repo in row.hf_repo_ids),
                }
            )
        )
        selected_manifest_lines = _source_manifest_lines(
            manifest_src,
            selection.rows,
        )
        manifest_meta = first_row.to_json_dict() if first_row else {}
        fixture_rows = [
            row.to_json_dict() for row in selection.rows if _is_synthetic_fixture(row)
        ]
        manifest_reward_summary = (
            first_row.reward_summary_payload() if first_row else {}
        )
    else:
        hdf5_files = sorted((*src.glob("*.h5"), *src.glob("*.hdf5")))
        if requested_episodes is not None:
            selected = set(requested_episodes)
            available = {path.name for path in hdf5_files}
            missing = sorted(selected - available)
            if missing:
                raise ValueError(
                    f"requested episodes are missing from {src}: {missing}"
                )
            hdf5_files = [p for p in hdf5_files if p.name in selected]
            if not hdf5_files:
                raise ValueError(
                    f"no HDF5 files selected from {src}; "
                    f"requested episodes={list(requested_episodes)}"
                )
        manifest_meta = {}
        archive_hdf5_files = [(path, path.name) for path in hdf5_files]
        fixture_rows = [
            metadata
            for path in hdf5_files
            if _is_synthetic_fixture(metadata := _read_event_hdf5_metadata(path))
        ]
    _warn_fixture_export("Zenodo", fixture_rows)
    zenodo_meta = {
        "license": license,
        "sensor_preset": manifest_meta.get("sensor_preset", ""),
        "policy_id": manifest_meta.get("policy_id", ""),
        "policy_checkpoint_hash": manifest_meta.get("policy_checkpoint_hash", ""),
        "seed": manifest_meta.get("seed", 0),
        "task_id": manifest_meta.get("task_id", ""),
        "schema_version": 1,
    }
    if doi:
        zenodo_meta["doi"] = doi
    if manifest_reward_summary:
        zenodo_meta["reward_summary"] = manifest_reward_summary
    if selected_hf_repo_ids:
        zenodo_meta["hf_repo_ids"] = list(selected_hf_repo_ids)
    if extra_metadata:
        zenodo_meta.update(extra_metadata)
    if fixture_rows:
        zenodo_meta.update(
            {
                "contains_synthetic_fixture": True,
                "fixture_task_ids": [SYNTHETIC_FIXTURE_TASK_ID],
                "fixture_policy_ids": [SYNTHETIC_FIXTURE_POLICY_ID],
                "task_id": SYNTHETIC_FIXTURE_TASK_ID,
                "policy_id": SYNTHETIC_FIXTURE_POLICY_ID,
            }
        )

    effective_license = str(zenodo_meta.get("license", license))
    effective_doi = (
        str(zenodo_meta["doi"]) if zenodo_meta.get("doi") is not None else None
    )
    readme_text = _render_readme(
        citation_text=citation_text,
        license=effective_license,
        doi=effective_doi,
        zenodo_meta=zenodo_meta,
        episode_count=len(hdf5_files),
    )

    manifest_payload = None
    if run.has_manifest:
        manifest_payload = "".join(
            line + "\n" for line in selected_manifest_lines
        ).encode("utf-8")
    member_names = ["README.md", "zenodo_metadata.json"]
    if manifest_payload is not None:
        member_names.append("events_manifest.jsonl")
    member_names.extend(relative for _path, relative in archive_hdf5_files)
    if len(member_names) != len(set(member_names)):
        duplicates = sorted(
            name for name in set(member_names) if member_names.count(name) > 1
        )
        raise ValueError(f"Zenodo bundle member collision: {duplicates}")

    def build_tar(staging: Path) -> None:
        with tarfile.open(staging, mode="w:gz") as tar:
            _add_bytes(tar, "README.md", readme_text.encode("utf-8"))
            _add_bytes(
                tar,
                "zenodo_metadata.json",
                (json.dumps(zenodo_meta, indent=2, sort_keys=True) + "\n").encode(),
            )
            if manifest_payload is not None:
                _add_bytes(tar, "events_manifest.jsonl", manifest_payload)
            for h5_path, relative_path in archive_hdf5_files:
                _add_published_file(tar, relative_path, h5_path)
        _validate_zenodo_tar(staging, expected_members=member_names)

    _publish_file(out, build_tar, lambda _staging: None)

    file_count = len(member_names)
    return ZenodoBundleResult(
        output_path=out,
        file_count=file_count,
        license=effective_license,
        doi=effective_doi,
        citation_md=citation_path,
    )


def _source_manifest_lines(
    manifest_path: Path,
    rows: Iterable[Any],
) -> list[str]:
    """Return original JSONL lines for selected rows.

    Keeping source bytes preserves provenance exactly as validated against
    EVENT_HDF5 metadata.
    """

    from srb.core.sensor.event_camera.manifest import EventManifestRow

    wanted = list(rows)
    text = ArtifactPublication.file(manifest_path).read(lambda raw: raw.decode("utf-8"))
    if text is None:
        raise FileNotFoundError(f"event manifest does not exist: {manifest_path}")
    lines: list[str] = []
    for raw_line in text.splitlines():
        if len(lines) == len(wanted):
            break
        line = raw_line.strip()
        if not line:
            continue
        if EventManifestRow.from_mapping(json.loads(line)) == wanted[len(lines)]:
            lines.append(line)
    if len(lines) != len(wanted):
        raise ValueError(
            f"event manifest changed while exporting the Zenodo bundle: {manifest_path}"
        )
    return lines


def _validate_zenodo_tar(path: Path, *, expected_members: list[str]) -> None:
    """Validate the exact archive tree before a Zenodo bundle is promoted."""

    try:
        with tarfile.open(path, mode="r:gz") as tar:
            members = tar.getmembers()
            names = [member.name for member in members]
            if sorted(names) != sorted(expected_members) or len(names) != len(
                expected_members
            ):
                raise ValueError(
                    "Zenodo bundle member tree mismatch: "
                    f"expected={sorted(expected_members)}, actual={sorted(names)}"
                )
            if any(not member.isfile() for member in members):
                raise ValueError("Zenodo bundle contains a non-file member")
            metadata_file = tar.extractfile("zenodo_metadata.json")
            if metadata_file is None:
                raise ValueError("Zenodo bundle is missing zenodo_metadata.json")
            metadata = json.loads(metadata_file.read())
            if not isinstance(metadata, dict) or metadata.get("schema_version") != 1:
                raise ValueError("Zenodo metadata schema is malformed")
    except (OSError, tarfile.TarError, json.JSONDecodeError) as exc:
        raise ValueError(f"Zenodo bundle is malformed: {exc}") from exc


def _add_bytes(tar: tarfile.TarFile, arcname: str, data: bytes) -> None:
    import io

    info = tarfile.TarInfo(name=arcname)
    info.size = len(data)
    tar.addfile(info, io.BytesIO(data))


def _read_event_hdf5_metadata(path: Path) -> dict[str, Any]:
    from srb.core.sensor.event_camera import validate_event_hdf5_schema

    report = validate_event_hdf5_schema(path)
    report.raise_for_errors()
    return dict(report.metadata)


def _is_synthetic_fixture(metadata: Any) -> bool:
    """Recognize fixture data only from the reserved exact task marker."""

    if hasattr(metadata, "get"):
        return metadata.get("task_id") == SYNTHETIC_FIXTURE_TASK_ID
    return getattr(metadata, "task_id", None) == SYNTHETIC_FIXTURE_TASK_ID


def _warn_fixture_export(format_name: str, fixture_rows: list[dict[str, Any]]) -> None:
    if fixture_rows:
        warnings.warn(
            f"{format_name} export includes synthetic fixture event data; "
            "it is not real-data evidence",
            UserWarning,
            stacklevel=3,
        )


def _render_readme(
    *,
    citation_text: str | None,
    license: str,
    doi: str | None,
    zenodo_meta: dict[str, Any],
    episode_count: int,
) -> str:
    header = (
        citation_text.rstrip()
        if citation_text is not None
        else "# SRB Event-Camera Dataset Bundle"
    )
    doi_value = zenodo_meta.get("doi", doi)
    doi_line = f"DOI: `{doi_value}`  \n" if doi_value else ""
    return (
        f"{header}\n\n"
        "## Bundle metadata\n\n"
        f"{doi_line}"
        f"License: `{zenodo_meta.get('license', license)}`  \n"
        f"Sensor preset: `{zenodo_meta.get('sensor_preset', '')}`  \n"
        f"Policy: `{zenodo_meta.get('policy_id', '')}`  \n"
        f"Policy checkpoint hash: `{zenodo_meta.get('policy_checkpoint_hash', '')}`  \n"
        f"Task: `{zenodo_meta.get('task_id', '')}`  \n"
        f"Seed: `{zenodo_meta.get('seed', 0)}`  \n"
        f"Episodes: {episode_count}\n\n"
        "Schema: `StorageFormat.EVENT_HDF5` (see "
        "`docs/src/sensors/event_camera.md` in the SRB repository).\n"
    )


_EVT3_HEADER_TEMPLATE = (
    "% format EVT3\n"
    "% camera_integrator_name SpaceRoboticsBench\n"
    "% generation 3.0\n"
    "% geometry {width}x{height}\n"
    "% srb_t0_ns {t0_ns}\n"
    "% end_header\n"
)
_EVT3_T0_HEADER_RE = re.compile(rb"^% srb_t0_ns (\d{1,19})\n", re.MULTILINE)


def to_prophesee_raw_evt3(
    in_h5: Path | str,
    out_raw: Path | str,
) -> Path:
    """Convert canonical EVENT_HDF5 data to Prophesee RAW EVT3.

    Encoding writes the wire format from the HDF5 schema without vendor tooling.
    """
    in_path = Path(in_h5)

    from srb.core.sensor.event_camera import read_event_hdf5

    events = read_event_hdf5(in_path)
    metadata = events.get("metadata") or {}
    if _is_synthetic_fixture(metadata):
        raise ValueError(
            "RAW EVT3 cannot export synthetic fixture events because the format "
            "does not preserve fixture provenance"
        )

    out_path = Path(out_raw)

    _publish_file(
        out_path,
        lambda staging: _write_prophesee_raw_evt3(events, staging),
        lambda staging: _validate_prophesee_raw_evt3(
            staging,
            expected_events=events,
        ),
    )
    return out_path


def _write_prophesee_raw_evt3(events: dict[str, Any], staging_path: Path) -> None:
    """Write canonical ``(t, x, y, p)`` events as Prophesee EVT3 words.

    Times are floored to microseconds and rebased to the first event; the
    origin is stored in the header and restored by the reader. Gaps above
    ``MAX_EVT3_GAP_NS`` raise ``Evt3ProtocolError``.
    """
    width, height = _resolution_from_metadata(events)
    t0_ns = _evt3_time_origin_ns(events)
    header = _EVT3_HEADER_TEMPLATE.format(
        width=width,
        height=height,
        t0_ns=t0_ns,
    ).encode("ascii")
    encoder = Evt3Encoder(width, height)
    with staging_path.open("wb") as stream:
        stream.write(header)
        pending = bytearray()
        for word in encoder.iter_words(
            _iter_evt3_events(events, width=width, height=height, t0_ns=t0_ns)
        ):
            pending.extend(pack_word_bytes(word))
            if len(pending) >= 64 * 1024:
                stream.write(pending)
                pending.clear()
        if pending:
            stream.write(pending)


def _validate_prophesee_raw_evt3(
    path: Path,
    *,
    expected_events: dict[str, Any] | None = None,
) -> None:
    """Strictly decode staged bytes before publication."""

    try:
        with path.open("rb") as stream:
            if stream.readline() != b"% format EVT3\n":
                raise ValueError("missing EVT3 format header")
            header_lines: list[bytes] = []
            while True:
                line = stream.readline()
                if not line:
                    raise ValueError("missing EVT3 end header")
                header_lines.append(line)
                if line == b"% end_header\n":
                    break
            header = b"".join(header_lines)
            geometry = re.search(rb"% geometry (\d+)x(\d+)\n", header)
            if geometry is None:
                raise ValueError("missing EVT3 geometry")
            width = int(geometry.group(1))
            height = int(geometry.group(2))
            t0_match = _EVT3_T0_HEADER_RE.search(header)
            if t0_match is None:
                raise ValueError("missing EVT3 srb_t0_ns header")
            t0_ns = int(t0_match.group(1))

            expected_iter: Iterable[tuple[int, int, int, int]] | None = None
            expected_count: int | None = None
            if expected_events is not None:
                expected_iter = iter(
                    _iter_evt3_events(
                        expected_events,
                        width=width,
                        height=height,
                        t0_ns=t0_ns,
                    )
                )
                expected_count = len(expected_events["t"])
            compared = 0

            def compare(event: Evt3Event) -> None:
                nonlocal compared
                if expected_iter is None:
                    return
                try:
                    expected = next(expected_iter)
                except StopIteration as exc:
                    raise ValueError("decoded more EVT3 events than expected") from exc
                if tuple(event) != expected:
                    raise ValueError(
                        f"EVT3 event mismatch at index {compared}: "
                        f"decoded={tuple(event)!r}, expected={expected!r}"
                    )
                compared += 1

            decoder = Evt3Decoder(width, height, on_event=compare)
            while True:
                chunk = stream.read(64 * 1024)
                if not chunk:
                    break
                decoder.feed_bytes(chunk)
            decoder.finish()
            if decoder.skipped_leading_words:
                raise ValueError(
                    f"{decoder.skipped_leading_words} EVT3 words precede the "
                    "first TIME_HIGH"
                )
            if expected_count is not None and compared != expected_count:
                raise ValueError(
                    f"decoded {compared} EVT3 events, expected {expected_count}"
                )
    except Evt3ProtocolError as exc:
        raise ValueError(f"malformed Prophesee RAW EVT3 output: {path}: {exc}") from exc
    except (OSError, ValueError) as exc:
        raise ValueError(f"malformed Prophesee RAW EVT3 output: {path}: {exc}") from exc


def _resolution_from_metadata(events: dict[str, Any]) -> tuple[int, int]:
    """Read metadata resolution, or infer it from the event bounding box."""
    metadata = events.get("metadata") or {}
    res = metadata.get("resolution")
    if res is not None:
        try:
            width = int(res[0])
            height = int(res[1])
            if width > 0 and height > 0:
                return width, height
        except (TypeError, IndexError, ValueError):
            pass
    import numpy as np

    x = np.asarray(events.get("x", []))
    y = np.asarray(events.get("y", []))
    width = int(x.max()) + 1 if x.size else 1
    height = int(y.max()) + 1 if y.size else 1
    return width, height


def _evt3_time_origin_ns(events: dict[str, Any]) -> int:
    """Return the first event time floored to a microsecond, in ns; empty is 0."""

    import numpy as np

    t = np.asarray(events.get("t", []))
    if not t.size:
        return 0
    first_ns = int(t.min())
    if first_ns < 0:
        raise Evt3ProtocolError("negative_timestamp", "timestamp must be non-negative")
    return (first_ns // 1000) * 1000


def _iter_evt3_events(
    events: dict[str, Any],
    *,
    width: int,
    height: int,
    t0_ns: int,
) -> Iterable[tuple[int, int, int, int]]:
    """Yield events with microsecond timestamps rebased by ``t0_ns``.

    ``read_event_hdf5`` already rejects decreasing timestamps, so file order is
    the stable time order.
    """

    import numpy as np

    t = np.asarray(events.get("t", []))
    x = np.asarray(events.get("x", []))
    y = np.asarray(events.get("y", []))
    p = np.asarray(events.get("p", []))
    if not (len(t) == len(x) == len(y) == len(p)):
        raise Evt3ProtocolError(
            "invalid_event",
            "EVENT_HDF5 event columns have unequal lengths",
        )
    for index in range(len(t)):
        t_us_ns = (int(t[index]) // 1000) * 1000
        yield t_us_ns - t0_ns, int(x[index]), int(y[index]), int(p[index])


def _encode_evt3_bytes(
    events: dict[str, Any],
    *,
    width: int,
    height: int,
    t0_ns: int,
) -> bytes:
    """Encode a packet body, rebased by ``t0_ns``, through the shared encoder."""

    encoder = Evt3Encoder(width, height)
    return words_to_bytes(
        encoder.iter_words(
            _iter_evt3_events(events, width=width, height=height, t0_ns=t0_ns)
        )
    )


_DEFAULT_ROS_MSG_TYPE = "event_camera_msgs"
_ROSBAG_STAGE_CHILD = "bag"


def to_rosbag(
    in_h5: Path | str,
    out_bag: Path | str,
    *,
    msg_type: str = _DEFAULT_ROS_MSG_TYPE,
    topic: str = "/events",
) -> Path:
    """Convert canonical EVENT_HDF5 data to a ROS bag on ``topic``.

    Requires the ``event-ros`` extra; missing packages raise
    ``MissingExtraError``. ``msg_type`` selects ``event_camera_msgs`` or ``dvs_msgs``.
    """
    if msg_type not in ("event_camera_msgs", "dvs_msgs"):
        raise ValueError(
            f"msg_type must be event_camera_msgs or dvs_msgs; got {msg_type!r}"
        )

    in_path = Path(in_h5)
    out_path = Path(out_bag)

    from srb.core.sensor.event_camera import read_event_hdf5

    events = read_event_hdf5(in_path)
    metadata = events.get("metadata") or {}
    if _is_synthetic_fixture(metadata):
        raise ValueError(
            "ROS bag cannot export synthetic fixture events because the format "
            "does not preserve fixture provenance"
        )
    ArtifactPublication.directory(out_path).publish(
        lambda staging: _write_rosbag(
            events,
            staging,
            msg_type=msg_type,
            topic=topic,
        ),
    )
    return out_path


def _write_rosbag(
    events: dict[str, Any],
    out_path: Path,
    *,
    msg_type: str,
    topic: str,
) -> None:
    """Write one episode as a single ROS bag message on ``topic``.

    The writer uses a fresh fixed child because rosbag2 rejects an existing
    URI; files move to the publication stage after close. One message per
    episode preserves episode boundaries.
    """
    width, height = _resolution_from_metadata(events)

    with require_extra("event-ros"):
        import rosbag2_py  # type: ignore[import-not-found]
        from rclpy.serialization import (
            serialize_message,  # type: ignore[import-not-found]
        )

        if msg_type == "dvs_msgs":
            from dvs_msgs.msg import (  # type: ignore[import-not-found]
                Event,
                EventArray,
            )

            ros_type = "dvs_msgs/msg/EventArray"
            msg = _build_dvs_event_array(
                events,
                EventArray=EventArray,
                Event=Event,
                width=width,
                height=height,
            )
        else:
            from event_camera_msgs.msg import (  # type: ignore[import-not-found]
                EventPacket,
            )

            ros_type = "event_camera_msgs/msg/EventPacket"
            msg = _build_event_packet(
                events,
                EventPacket=EventPacket,
                width=width,
                height=height,
            )

    serialized = serialize_message(msg)
    first_t_ns = int(events["t"][0]) if len(events["t"]) else 0

    bag_name = _ROSBAG_STAGE_CHILD
    bag_uri = out_path / bag_name
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    topic_metadata = rosbag2_py.TopicMetadata(
        name=topic,
        type=ros_type,
        serialization_format="cdr",
    )
    writer.create_topic(topic_metadata)
    writer.write(topic, serialized, first_t_ns)
    close = getattr(writer, "close", None)
    if callable(close):
        close()
    else:
        # Bindings without close() finalize metadata when the writer is dropped.
        del writer
    if not bag_uri.is_dir():
        raise RuntimeError(f"rosbag2 did not create the bag directory: {bag_uri}")
    members = sorted(bag_uri.iterdir())
    if any(member.name == bag_name for member in members):
        raise RuntimeError(f"rosbag2 bag member collides with its directory: {bag_uri}")
    for member in members:
        member.rename(out_path / member.name)
    bag_uri.rmdir()


def _build_dvs_event_array(
    events: dict[str, Any],
    *,
    EventArray: Any,
    Event: Any,
    width: int,
    height: int,
) -> Any:
    msg = EventArray()
    msg.width = width
    msg.height = height
    msg.events = []
    t = events["t"]
    x = events["x"]
    y = events["y"]
    p = events["p"]
    for i in range(len(t)):
        e = Event()
        ts_ns = int(t[i])
        e.ts.sec = ts_ns // 1_000_000_000
        e.ts.nanosec = ts_ns % 1_000_000_000
        e.x = int(x[i])
        e.y = int(y[i])
        e.polarity = bool(int(p[i]) > 0)
        msg.events.append(e)
    return msg


def _build_event_packet(
    events: dict[str, Any],
    *,
    EventPacket: Any,
    width: int,
    height: int,
) -> Any:
    """Pack one EVT3 byte stream; ``time_base + decoded_t_ns`` gives event time."""
    t0_ns = _evt3_time_origin_ns(events)
    msg = EventPacket()
    msg.width = int(width)
    msg.height = int(height)
    msg.encoding = "evt3"
    msg.time_base = t0_ns
    msg.seq = 0
    msg.events = list(
        _encode_evt3_bytes(events, width=width, height=height, t0_ns=t0_ns)
    )
    return msg


def to_npz(
    in_h5: Path | str,
    out_npz: Path | str,
    *,
    compress: bool = True,
) -> Path:
    """Convert canonical EVENT_HDF5 columns and JSON metadata to ``.npz``."""
    in_path = Path(in_h5)
    out_path = Path(out_npz)

    from srb.core.sensor.event_camera import read_event_hdf5

    events = read_event_hdf5(in_path)
    metadata = events.get("metadata") or {}
    if _is_synthetic_fixture(metadata):
        _warn_fixture_export("NPZ", [metadata])
    _publish_file(
        out_path,
        lambda staging: _write_npz(events, staging, compress=compress),
        _validate_npz_output,
    )
    return out_path


def _write_npz(events: dict[str, Any], staging_path: Path, *, compress: bool) -> None:
    """Pack canonical columns with their on-disk dtypes and JSON metadata.

    ``compress`` selects compressed or uncompressed NumPy archive writing.
    """
    import numpy as np

    columns = {
        "t": np.asarray(events.get("t", []), dtype=np.int64),
        "x": np.asarray(events.get("x", []), dtype=np.uint16),
        "y": np.asarray(events.get("y", []), dtype=np.uint16),
        "p": np.asarray(events.get("p", []), dtype=np.int8),
    }
    metadata = events.get("metadata") or {}
    columns["metadata"] = np.asarray(json.dumps(_jsonable(metadata)))

    saver: Any = np.savez_compressed if compress else np.savez
    with staging_path.open("wb") as stream:
        saver(stream, **columns)


def _validate_npz_output(path: Path) -> None:
    """Validate the complete NPZ schema before publication."""

    import numpy as np

    try:
        with np.load(path) as archive:
            required = {"t", "x", "y", "p", "metadata"}
            if set(archive.files) != required:
                raise ValueError(
                    f"NPZ keys must be {sorted(required)}, got {sorted(archive.files)}"
                )
            lengths = {int(archive[name].shape[0]) for name in ("t", "x", "y", "p")}
            if len(lengths) != 1:
                raise ValueError("NPZ event columns have unequal lengths")
            metadata = archive["metadata"]
            if metadata.shape != ():
                raise ValueError("NPZ metadata must be a scalar JSON array")
            json.loads(str(metadata.item()))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"malformed event NPZ output: {path}: {exc}") from exc


def _jsonable(value: Any) -> Any:
    """Coerce HDF5 metadata values into JSON-compatible Python values."""
    import numpy as np

    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (bytes, bytearray)):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


_HF_REPO_PREFIX = "srb-events"
_HF_REPO_SLUG_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")


def event_camera_hf_repo_id(
    task_id: str,
    preset_name: str,
    *,
    owner: str | None = None,
) -> str:
    """Build ``srb-events-<task>-<preset_family>`` with optional HF owner.

    Preset family is the segment before the first underscore; task and
    family segments are lower-cased with underscores changed to hyphens.
    """
    task_slug = _slugify_hf_segment(task_id, field="task_id")
    family_slug = _slugify_hf_segment(
        preset_name.split("_", 1)[0],
        field="preset_name",
    )
    name = f"{_HF_REPO_PREFIX}-{task_slug}-{family_slug}"
    if owner is None:
        return name
    owner_slug = _slugify_hf_segment(owner, field="owner", lower=False)
    return f"{owner_slug}/{name}"


def _slugify_hf_segment(value: str, *, field: str, lower: bool = True) -> str:
    """Lower-case + underscore-to-hyphen + HF-charset validation."""
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string, got {value!r}")
    slug = value.strip().replace("_", "-")
    if lower:
        slug = slug.lower()
    if _HF_REPO_SLUG_PATTERN.fullmatch(slug) is None or slug.endswith((".", "-")):
        raise ValueError(
            f"{field}={value!r} does not form a valid HuggingFace repo segment "
            f"(allowed: [A-Za-z0-9._-], no leading hyphen, no trailing '.'/'-')"
        )
    return slug


__all__ = [
    "LeRobotBundleResult",
    "ZenodoBundleResult",
    "event_camera_hf_repo_id",
    "to_lerobot_v3_bundle",
    "to_npz",
    "to_prophesee_raw_evt3",
    "to_rosbag",
    "to_zenodo_bundle",
]
