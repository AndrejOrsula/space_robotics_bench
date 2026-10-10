"""Event-camera subactions for ``srb dataset``."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from srb.cli._output import print_json
from srb.cli._parsing import SrbHelpFormatter
from srb.utils.failures import InvalidInputError


def _invalid_input(message: str) -> InvalidInputError:
    # A FailureError reaches the CLI dispatcher's one-line report (exit 2)
    # instead of a traceback; it is still a ValueError for direct callers.
    return InvalidInputError(
        message=message,
        code="invalid_input.dataset_cli",
        component="cli.dataset",
    )


EVENT_ACTIONS: tuple[str, ...] = (
    "record-events",
    "validate-events",
    "export-lerobot-v3",
    "export-zenodo-bundle",
    "export-raw-evt3",
    "import-raw-evt3",
    "export-rosbag",
)
# Names of the dataset actions handled by this module.
EVENT_BACKEND_CHOICES: tuple[str, ...] = (
    "noop",
    "v2e",
    "v2e_streaming",
    "metavision",
    "v2ce",
)
EVENT_PRESET_CHOICES: tuple[str, ...] = (
    "clean",
    "imx636_low_light",
    "imx636_nominal",
    "imx636_outdoor_sunlit",
)


# ---------- argparse construction ---------------------------------------


def add_subparsers(dataset_subs: argparse._SubParsersAction) -> None:
    """Register event-camera subactions on the ``dataset`` parser."""
    _add_record_events(dataset_subs)
    _add_validate_events(dataset_subs)
    _add_export_lerobot_v3(dataset_subs)
    _add_export_zenodo_bundle(dataset_subs)
    _add_export_raw_evt3(dataset_subs)
    _add_import_raw_evt3(dataset_subs)
    _add_export_rosbag(dataset_subs)


def _add_record_events(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "record-events",
        help=(
            "Record a synthetic event-camera dataset with canonical EVENT_HDF5 "
            "episodes and a provenance manifest."
        ),
        formatter_class=SrbHelpFormatter,
    )
    p.add_argument(
        "--task",
        default="landing",
        help=(
            "Compatibility task input retained in the CLI summary; fixture "
            "provenance is always fixture_moving_patch"
        ),
    )
    p.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Number of positive episodes to record",
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output dataset directory",
    )
    p.add_argument(
        "--num-envs",
        type=int,
        default=1,
        help="Parallel environment instances per episode",
    )
    p.add_argument(
        "--steps-per-episode",
        type=int,
        default=10,
        help="Positive recorded steps per episode before flushing",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional run seed for the environment and event-camera episodes",
    )
    p.add_argument(
        "--preset",
        default="imx636_nominal",
        choices=EVENT_PRESET_CHOICES,
        help="IMX636 preset name",
    )
    p.add_argument(
        "--backend",
        dest="event_backend",
        default="v2e",
        choices=EVENT_BACKEND_CHOICES,
        help=(
            "EventGenerator backend name. Defaults to v2e. "
            "'noop' remains available for the empty-events smoke path."
        ),
    )
    p.add_argument(
        "--policy-id",
        dest="policy_id",
        default="walking_skeleton",
        help="Compatibility flag; fixture provenance uses synthetic_fixture",
    )


def _add_validate_events(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "validate-events",
        help=(
            "On-demand event-camera fidelity validation (statistics, "
            "reconstruction, and downstream checks)."
        ),
        formatter_class=SrbHelpFormatter,
    )
    nested = p.add_subparsers(
        title="validate-events actions",
        dest="validate_events_action",
        required=True,
    )
    stats = nested.add_parser(
        "stats",
        help=(
            "Statistical fidelity scorecard (rate, polarity, ISI/EMD, "
            "hot-pixel, sparsity) for a synth vs. real event HDF5 pair."
        ),
        formatter_class=SrbHelpFormatter,
    )
    stats.add_argument(
        "--synth",
        type=Path,
        required=True,
        help="Path to a synthetic SRB event HDF5 (StorageFormat.EVENT_HDF5)",
    )
    stats.add_argument(
        "--real",
        type=Path,
        required=True,
        help="Path to a real-IMX636 reference HDF5 (same schema)",
    )
    stats.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output JSON scorecard path",
    )
    stats.add_argument(
        "--resolution",
        default=None,
        help=(
            "Resolution as WxH (e.g. 1280x720) for files without "
            "/metadata/resolution; declared resolutions take precedence."
        ),
    )
    stats.add_argument(
        "--sparsity-window-ms",
        dest="sparsity_window_ms",
        type=float,
        default=1.0,
        help="Spatial-sparsity window in milliseconds",
    )
    reconstruction = nested.add_parser(
        "reconstruction",
        help=(
            "Reconstruction-fidelity scorecard: reconstruct events at "
            "the RGB frame timestamps and score SSIM (and optionally LPIPS) "
            "against the ground-truth RGB."
        ),
        formatter_class=SrbHelpFormatter,
    )
    reconstruction.add_argument(
        "--events",
        type=Path,
        required=True,
        help="Path to an SRB event HDF5 (StorageFormat.EVENT_HDF5)",
    )
    reconstruction.add_argument(
        "--rgb-npy",
        dest="rgb_npy",
        type=Path,
        required=True,
        help="Path to a .npy (T, H, W, 3) RGB tensor (uint8 or float)",
    )
    reconstruction.add_argument(
        "--times-npy",
        dest="times_npy",
        type=Path,
        required=True,
        help="Path to a .npy (T,) int64 nanosecond timestamps for each RGB frame",
    )
    reconstruction.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output JSON scorecard path",
    )
    reconstruction.add_argument(
        "--operator",
        choices=("baseline", "e2vid"),
        default="baseline",
        help=(
            "Reconstruction operator. 'baseline' is pure numpy and always "
            "available; 'e2vid' is gated on the event-reconstruction extra."
        ),
    )
    reconstruction.add_argument(
        "--lpips",
        dest="include_lpips",
        action="store_true",
        help=(
            "Additionally compute LPIPS. Requires the event-reconstruction "
            "extra (torch + lpips)."
        ),
    )
    reconstruction.add_argument(
        "--resolution",
        default=None,
        help=(
            "Override resolution as WxH (e.g. 1280x720). Defaults to the "
            "events file's /metadata/resolution, falling back to the RGB shape."
        ),
    )


def _add_export_lerobot_v3(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "export-lerobot-v3",
        help=(
            "Wrap an EVENT_HDF5 run directory into a LeRobot v3 dataset "
            "layout with an SRB event_camera modality."
        ),
        formatter_class=SrbHelpFormatter,
    )
    p.add_argument(
        "path",
        type=Path,
        help=(
            "Source directory containing events_manifest.jsonl and the "
            "nested .h5 files referenced by its rows"
        ),
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output LeRobot v3 dataset directory (created if absent)",
    )


def _add_export_zenodo_bundle(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "export-zenodo-bundle",
        help=(
            "Tar selected EVENT_HDF5 episodes into a Zenodo-ready "
            "bundle (README, manifest, zenodo_metadata.json, and referenced "
            ".h5 files)."
        ),
        formatter_class=SrbHelpFormatter,
    )
    p.add_argument(
        "path",
        type=Path,
        help=(
            "Source directory containing events_manifest.jsonl and the "
            "nested .h5 files referenced by its rows"
        ),
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output bundle path (.tar.gz appended if missing)",
    )
    p.add_argument(
        "--citation-md",
        dest="citation_md",
        type=Path,
        default=None,
        help="Optional README.md for the bundle; defaults to a generated metadata summary",
    )
    p.add_argument(
        "--license",
        dest="license_id",
        default="CC-BY-4.0",
        help="Bundle license identifier",
    )
    p.add_argument(
        "--doi",
        default=None,
        help="DOI recorded in zenodo_metadata.json and the generated README",
    )


def _add_export_raw_evt3(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "export-raw-evt3",
        help="Convert one EVENT_HDF5 file to Prophesee RAW EVT3.",
        formatter_class=SrbHelpFormatter,
    )
    p.add_argument(
        "path",
        type=Path,
        help="Input EVENT_HDF5 file",
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output .raw path",
    )


def _add_import_raw_evt3(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "import-raw-evt3",
        help="Convert one Prophesee RAW EVT3 file to canonical EVENT_HDF5.",
        formatter_class=SrbHelpFormatter,
    )
    p.add_argument(
        "path",
        type=Path,
        help="Input Prophesee RAW EVT3 file",
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output EVENT_HDF5 path",
    )
    p.add_argument(
        "--task",
        default="",
        help="Task/source identifier persisted to /metadata/task_id",
    )
    p.add_argument(
        "--policy-id",
        dest="policy_id",
        default="",
        help="Policy/source identifier persisted to /metadata/policy_id",
    )
    p.add_argument(
        "--preset",
        default="imx636_nominal",
        choices=EVENT_PRESET_CHOICES,
        help="Sensor preset persisted to /metadata/sensor_preset",
    )


def _add_export_rosbag(subs: argparse._SubParsersAction) -> None:
    p = subs.add_parser(
        "export-rosbag",
        help=(
            "Convert one EVENT_HDF5 file to a ROS bag (gated on the event-ros extra)."
        ),
        formatter_class=SrbHelpFormatter,
    )
    p.add_argument(
        "path",
        type=Path,
        help="Input EVENT_HDF5 file",
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output ROS bag path",
    )
    p.add_argument(
        "--msg-type",
        dest="msg_type",
        choices=("event_camera_msgs", "dvs_msgs"),
        default="event_camera_msgs",
        help="ROS message type for the events",
    )
    p.add_argument(
        "--topic",
        default="/events",
        help="ROS topic name",
    )


# ---------- dispatch ----------------------------------------------------


def execute_if_event(
    dataset_action: str,
    *,
    path: Path | None = None,
    output: Path | None = None,
    task: str | None = None,
    episodes: int = 1,
    num_envs: int = 1,
    steps_per_episode: int = 10,
    seed: int | None = None,
    preset: str = "imx636_nominal",
    event_backend: str = "v2e",
    policy_id: str = "walking_skeleton",
    validate_events_action: str | None = None,
    synth: Path | None = None,
    real: Path | None = None,
    resolution: str | None = None,
    sparsity_window_ms: float = 1.0,
    rgb_npy: Path | None = None,
    times_npy: Path | None = None,
    events: Path | None = None,
    operator: str = "baseline",
    include_lpips: bool = False,
    citation_md: Path | None = None,
    license_id: str = "CC-BY-4.0",
    doi: str | None = None,
    msg_type: str = "event_camera_msgs",
    topic: str = "/events",
    **_ignored: Any,
) -> bool:
    """Dispatch event-camera subactions; return True when handled."""
    if dataset_action == "record-events":
        if output is None:
            raise _invalid_input("dataset record-events requires --output")
        execute_record_events(
            task=task or "landing",
            episodes=episodes,
            num_envs=num_envs,
            steps_per_episode=steps_per_episode,
            seed=seed,
            preset=preset,
            event_backend=event_backend,
            policy_id=policy_id,
            output=output,
        )
        return True
    if dataset_action == "validate-events":
        if validate_events_action == "stats":
            if synth is None or real is None or output is None:
                raise _invalid_input(
                    "dataset validate-events stats requires --synth, --real, --output"
                )
            execute_validate_events_stats(
                synth=synth,
                real=real,
                output=output,
                resolution=resolution,
                sparsity_window_ms=sparsity_window_ms,
            )
            return True
        if validate_events_action == "reconstruction":
            if events is None or rgb_npy is None or times_npy is None or output is None:
                raise _invalid_input(
                    "dataset validate-events reconstruction requires "
                    "--events, --rgb-npy, --times-npy, --output"
                )
            execute_validate_events_reconstruction(
                events_path=events,
                rgb_npy=rgb_npy,
                times_npy=times_npy,
                output=output,
                operator=operator,
                include_lpips=include_lpips,
                resolution=resolution,
            )
            return True
        raise ValueError(f"unknown validate-events action: {validate_events_action!r}")
    if dataset_action == "export-lerobot-v3":
        if output is None or path is None:
            raise _invalid_input("dataset export-lerobot-v3 requires path and --output")
        execute_export_lerobot_v3(event_hdf5_dir=path, output=output)
        return True
    if dataset_action == "export-zenodo-bundle":
        if output is None or path is None:
            raise _invalid_input(
                "dataset export-zenodo-bundle requires path and --output"
            )
        execute_export_zenodo_bundle(
            event_hdf5_dir=path,
            output=output,
            citation_md=citation_md,
            license_id=license_id,
            doi=doi,
        )
        return True
    if dataset_action == "export-raw-evt3":
        if output is None or path is None:
            raise _invalid_input("dataset export-raw-evt3 requires path and --output")
        execute_export_raw_evt3(in_h5=path, out_raw=output)
        return True
    if dataset_action == "import-raw-evt3":
        if output is None or path is None:
            raise _invalid_input("dataset import-raw-evt3 requires path and --output")
        execute_import_raw_evt3(
            in_raw=path,
            out_h5=output,
            task_id=task or "",
            policy_id=policy_id,
            sensor_preset=preset,
        )
        return True
    if dataset_action == "export-rosbag":
        if output is None or path is None:
            raise _invalid_input("dataset export-rosbag requires path and --output")
        execute_export_rosbag(
            in_h5=path,
            out_bag=output,
            msg_type=msg_type,
            topic=topic,
        )
        return True
    return False


def execute_validate_event_run_if_present(path: Path, *, strict: bool = False) -> bool:
    """Validate an EVENT_HDF5 run artifact when ``path`` points at one."""
    run_root = _event_run_root(path)
    if run_root is None:
        return False

    from srb.datasets.event_camera_run import EventCameraRunArtifact

    report = EventCameraRunArtifact.validate_path(run_root, strict=strict)
    print_json(report)
    if not report.valid:
        raise SystemExit(1)
    return True


# ---------- handlers ----------------------------------------------------


def _detect_provenance() -> tuple[str, str]:
    """Return manifest provenance from the module's repo root and package metadata.

    Git metadata falls back to an adjacent source manifest; unavailable fields
    are empty strings.
    """
    from srb.utils.source_revision import detect_srb_git_sha

    repo_root = Path(__file__).resolve().parents[3]
    git_sha = detect_srb_git_sha(repo_root)

    isaaclab_version = ""
    try:
        import isaaclab  # type: ignore[import-not-found]

        isaaclab_version = str(getattr(isaaclab, "__version__", "") or "")
    except ImportError:
        try:
            from importlib.metadata import version

            isaaclab_version = version("isaaclab")
        except Exception:  # noqa: BLE001
            isaaclab_version = ""

    return git_sha, isaaclab_version


def execute_record_events(
    *,
    task: str,
    episodes: int,
    num_envs: int,
    steps_per_episode: int,
    seed: int | None = None,
    preset: str,
    event_backend: str,
    policy_id: str,
    output: Path,
) -> None:
    """Record synthetic EVENT_HDF5 episodes without booting Isaac Sim."""
    for name, value in (
        ("episodes", episodes),
        ("num_envs", num_envs),
        ("steps_per_episode", steps_per_episode),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise _invalid_input(f"{name} must be a positive integer")
    from srb.cli.capabilities import require_event_backend

    require_event_backend(event_backend)
    from srb.core.sensor.event_camera import (
        EventCameraCfg,
        EventRecorder,
    )
    from srb.core.sensor.event_camera.manifest import (
        SYNTHETIC_FIXTURE_POLICY_ID,
        SYNTHETIC_FIXTURE_TASK_ID,
    )

    if seed is not None:
        import torch

        torch.manual_seed(seed)

    cfg = EventCameraCfg(preset_name=preset)
    cfg.backend = event_backend
    if event_backend == "v2e_streaming":
        # The bounded streaming profile omits shot noise until ordered merging
        # is available.
        cfg.shot_noise_rate_hz = 0.0

    scene = _DummyEventScene({"event_camera": cfg}, num_envs=num_envs)
    git_sha, _isaaclab_version = _detect_provenance()
    provenance = {
        "policy_id": SYNTHETIC_FIXTURE_POLICY_ID,
        "task_id": SYNTHETIC_FIXTURE_TASK_ID,
        "srb_git_sha": git_sha,
        "isaaclab_version": "",
    }
    recorder = EventRecorder(
        scene,
        output,
        num_envs=num_envs,
        provenance=provenance,
        run_seed=seed,
    )
    if not recorder.has_sensors:
        raise RuntimeError("EventRecorder failed to register the seeded sensor")

    # Sensor time is absolute and survives `save_episodes`; keep timestamps
    # increasing across episodes (1 ms per synthetic step).
    sensor = scene.sensors["event_camera"]
    try:
        for ep_idx in range(episodes):
            ep_offset_ns = ep_idx * steps_per_episode * 1_000_000
            for step in range(steps_per_episode):
                sensor.set_frame(ep_idx * steps_per_episode + step)
                recorder.record_step(sim_time_ns=ep_offset_ns + step * 1_000_000)
            recorder.save_episodes(list(range(num_envs)))
    except BaseException as exc:
        from srb.utils.failures import attempt_cleanup

        for env_id in range(num_envs):
            _result, abort_error = attempt_cleanup(
                lambda env_id=env_id, cause=exc: recorder.abort_episode(env_id, cause)
            )
            if abort_error is not None:
                exc.add_note(
                    f"event recorder abort failed for env {env_id}: {abort_error!r}"
                )
        try:
            recorder.close()
        except BaseException as close_error:
            exc.add_note(f"event recorder cleanup failed: {close_error!r}")
        raise

    print_json(
        {
            "task": task,
            "source": "synthetic_fixture",
            "episodes": episodes,
            "num_envs": num_envs,
            "steps_per_episode": steps_per_episode,
            "backend": event_backend,
            "preset": preset,
            "output_dir": str(recorder.record_dir),
            "manifest": str(recorder.record_dir / "events_manifest.jsonl"),
        }
    )


def execute_export_lerobot_v3(*, event_hdf5_dir: Path, output: Path) -> None:
    from srb.datasets.event_camera_export import to_lerobot_v3_bundle

    result = to_lerobot_v3_bundle(event_hdf5_dir, output)
    print_json(
        {
            "output_dir": str(result.output_dir),
            "num_episodes": result.num_episodes,
            "total_event_count": result.total_event_count,
            "sensors": list(result.sensors),
        }
    )


def execute_export_zenodo_bundle(
    *,
    event_hdf5_dir: Path,
    output: Path,
    citation_md: Path | None,
    license_id: str,
    doi: str | None,
) -> None:
    from srb.datasets.event_camera_export import to_zenodo_bundle

    result = to_zenodo_bundle(
        event_hdf5_dir,
        output,
        citation_md=citation_md,
        license=license_id,
        doi=doi,
    )
    print_json(
        {
            "output_path": str(result.output_path),
            "file_count": result.file_count,
            "license": result.license,
            "doi": result.doi,
            "citation_md": (str(result.citation_md) if result.citation_md else None),
        }
    )


def execute_export_raw_evt3(*, in_h5: Path, out_raw: Path) -> None:
    from srb.datasets.event_camera_export import to_prophesee_raw_evt3

    result = to_prophesee_raw_evt3(in_h5, out_raw)
    print_json({"input": str(in_h5), "output": str(result)})


def execute_import_raw_evt3(
    *,
    in_raw: Path,
    out_h5: Path,
    task_id: str,
    policy_id: str,
    sensor_preset: str,
) -> None:
    from srb.datasets.event_camera_import import from_prophesee_raw_evt3

    result = from_prophesee_raw_evt3(
        in_raw,
        out_h5,
        task_id=task_id,
        policy_id=policy_id,
        sensor_preset=sensor_preset,
    )
    print_json(
        {
            "input": str(result.input_path),
            "output": str(result.output_path),
            "event_count": result.event_count,
            "resolution": list(result.resolution),
            "time_wraps": result.time_wraps,
            "ignored_words": result.ignored_words,
            "invalid_events": result.invalid_events,
            "skipped_leading_words": result.skipped_leading_words,
            "time_offset_ns": result.time_offset_ns,
        }
    )
    if result.skipped_leading_words:
        import sys

        print(
            f"warning: skipped {result.skipped_leading_words} EVT3 words before "
            "the first TIME_HIGH (mid-stream capture start); events in that "
            "span were not imported",
            file=sys.stderr,
        )


def execute_export_rosbag(
    *,
    in_h5: Path,
    out_bag: Path,
    msg_type: str,
    topic: str,
) -> None:
    from srb.datasets.event_camera_export import to_rosbag

    result = to_rosbag(in_h5, out_bag, msg_type=msg_type, topic=topic)
    print_json(
        {
            "input": str(in_h5),
            "output": str(result),
            "msg_type": msg_type,
            "topic": topic,
        }
    )


def execute_validate_events_stats(
    *,
    synth: Path,
    real: Path,
    output: Path,
    resolution: str | None,
    sparsity_window_ms: float,
) -> None:
    window_ns = sparsity_window_ms * 1_000_000
    if (
        not math.isfinite(sparsity_window_ms)
        or not math.isfinite(window_ns)
        or window_ns < 1
    ):
        raise _invalid_input("--sparsity-window-ms must be finite and at least 1 ns")

    from srb.core.sensor.event_camera import read_validated_event_hdf5
    from srb.datasets.event_validation import score_pair
    from srb.datasets.event_validation.statistical import (
        scorecard_json_payload,
        write_scorecard,
    )

    res = _parse_resolution(resolution)
    synth_events = read_validated_event_hdf5(synth)
    real_events = read_validated_event_hdf5(real)
    card = score_pair(
        synth_events,
        real_events,
        resolution=res,
        sparsity_window_ns=int(window_ns),
    )
    write_scorecard(card, output)
    payload = scorecard_json_payload(card)
    print_json(
        {
            "synth": str(synth),
            "real": str(real),
            "scorecard": str(output),
            "distances": payload["distances"],
        }
    )


def execute_validate_events_reconstruction(
    *,
    events_path: Path,
    rgb_npy: Path,
    times_npy: Path,
    output: Path,
    operator: str,
    include_lpips: bool,
    resolution: str | None,
) -> None:
    """Reconstruct events from RGB frames and save SSIM/optional LPIPS scores."""
    from srb.cli.capabilities import require_event_reconstruction

    require_event_reconstruction(
        operator=operator,
        include_lpips=include_lpips,
    )
    import numpy as np

    from srb.core.sensor.event_camera import read_validated_event_hdf5
    from srb.datasets.event_validation.reconstruction import (
        score_reconstruction,
        write_scorecard,
    )
    from srb.utils.publication import ArtifactPublication
    from srb.utils.resource_budget import inspect_npy_array

    def load_npy_snapshot(path: Path, *, resource: str) -> np.ndarray:
        with ArtifactPublication.file(path).open_snapshot() as snapshot:
            if snapshot is None:
                raise FileNotFoundError(path)
            inspect_npy_array(snapshot.file, resource=resource)
            snapshot.file.seek(0)
            loaded = np.load(snapshot.file, allow_pickle=False)
            if not isinstance(loaded, np.ndarray):
                raise ValueError(f"{resource} is not an NPY array")
            return loaded

    try:
        res = _parse_resolution(resolution)
        ev = read_validated_event_hdf5(events_path)
        rgb = load_npy_snapshot(rgb_npy, resource="event reconstruction RGB NPY")
        times = _frame_times_ns(
            load_npy_snapshot(
                times_npy,
                resource="event reconstruction timestamp NPY",
            )
        )
        card = score_reconstruction(
            ev,
            rgb,
            times,
            resolution=res,
            operator=operator,
            include_lpips=include_lpips,
        )
    except (OSError, ValueError) as exc:
        raise _invalid_input(f"dataset validate-events reconstruction: {exc}") from exc
    write_scorecard(card, output)
    print_json(
        {
            "events": str(events_path),
            "rgb_npy": str(rgb_npy),
            "times_npy": str(times_npy),
            "operator": operator,
            "scorecard": str(output),
            "metrics": card.metrics,
        }
    )


def _frame_times_ns(raw: Any) -> Any:
    """Return nonempty, one-dimensional, nondecreasing int64 nanosecond times.

    Reject values outside the int64 nanosecond range or with fractional,
    non-finite, empty, or incorrectly shaped input. Floats are accepted only
    when finite and integral, preventing silent truncation of timestamps.
    """
    import numpy as np

    arr = np.asarray(raw)
    if arr.ndim != 1:
        raise ValueError(f"frame times must be one-dimensional; got shape {arr.shape}")
    if arr.dtype.kind in "iu":
        if (
            arr.dtype.kind == "u"
            and arr.size
            and int(arr.max()) > np.iinfo(np.int64).max
        ):
            raise ValueError("frame times exceed the int64 nanosecond range")
        times = arr.astype(np.int64)
    elif arr.dtype.kind == "f":
        if not np.all(np.isfinite(arr)):
            raise ValueError("frame times must be finite")
        if not np.all(arr == np.floor(arr)):
            raise ValueError(
                "frame times must be integral nanoseconds; got fractional values"
            )
        info = np.iinfo(np.int64)
        if arr.size and (arr.min() < info.min or arr.max() >= float(info.max)):
            raise ValueError("frame times exceed the int64 nanosecond range")
        times = arr.astype(np.int64)
    else:
        raise ValueError(
            f"frame times must be an integer or float array; got dtype {arr.dtype}"
        )
    if times.size == 0:
        raise ValueError("frame times must not be empty")
    # Compare neighbours directly; ``np.diff`` would wrap at the int64 bounds.
    if bool(np.any(times[1:] < times[:-1])):
        raise ValueError("frame times must be nondecreasing")
    return times


def _parse_resolution(value: str | None) -> tuple[int, int] | None:
    if value is None:
        return None
    parts = value.lower().split("x")
    if (
        len(parts) != 2
        or not all(p.strip().isdecimal() for p in parts)
        or 0 in (int(parts[0]), int(parts[1]))
    ):
        raise _invalid_input(
            f"--resolution must be WxH with positive integers (e.g. 1280x720); "
            f"got {value!r}"
        )
    return (int(parts[0]), int(parts[1]))


def _event_run_root(path: Path) -> Path | None:
    from srb.datasets.event_camera_run import EVENTS_MANIFEST_FILENAME

    if path.is_dir() and (path / EVENTS_MANIFEST_FILENAME).exists():
        return path
    if path.is_file() and path.name == EVENTS_MANIFEST_FILENAME:
        return path.parent
    return None


class _DummyEventScene:
    """Minimal scene shape consumed by :func:`iter_event_camera_sensors`.

    The CLI runs outside Isaac Sim; the simulation runtime supplies an
    :class:`isaaclab.scene.InteractiveScene`.
    """

    def __init__(self, sensors: dict[str, Any], *, num_envs: int = 1) -> None:
        self.sensors = {
            name: _DummySensor(cfg, num_envs=num_envs) for name, cfg in sensors.items()
        }


class _DummySensor:
    def __init__(self, cfg: Any, *, num_envs: int = 1) -> None:
        import numpy as np

        self.cfg = cfg
        self.num_instances = num_envs
        width, height = cfg.effective_resolution
        self.data = SimpleNamespace(
            output={"rgb": np.zeros((num_envs, height, width, 3), dtype=np.uint8)}
        )

    def set_frame(self, step: int) -> None:
        """Write a deterministic moving-bright-patch RGB fixture."""
        frame = self.data.output["rgb"]
        frame.fill(0)
        width = frame.shape[2]
        x = (int(step) * 3) % max(1, width - 8)
        frame[:, :8, x : x + 8, :] = 255


__all__ = [
    "EVENT_ACTIONS",
    "add_subparsers",
    "execute_export_lerobot_v3",
    "execute_export_raw_evt3",
    "execute_export_rosbag",
    "execute_export_zenodo_bundle",
    "execute_import_raw_evt3",
    "execute_if_event",
    "execute_record_events",
    "execute_validate_event_run_if_present",
    "execute_validate_events_reconstruction",
    "execute_validate_events_stats",
]
