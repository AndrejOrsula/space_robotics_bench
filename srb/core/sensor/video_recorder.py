from __future__ import annotations

from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Sequence

from srb.core.sensor.event_camera.recorder import EventRecorder
from srb.core.sensor.visual_codec import video_frame_from_visual_image
from srb.core.sensor.visual_output import (
    construct_visual_observation,
    is_visual_data_type,
    iter_visual_sensor_outputs,
)
from srb.utils import logging
from srb.utils.artifact_paths import (
    validate_artifact_relative_path,
    validate_artifact_segment,
)
from srb.utils.failures import attempt_cleanup

try:
    from srb.utils.cfg import DEFAULT_DATETIME_FORMAT
except ImportError:
    DEFAULT_DATETIME_FORMAT = "%Y%m%dT%H%M%S"

if TYPE_CHECKING:
    from isaaclab.scene import InteractiveScene


class VideoRecorder:
    """Records visual + event sensor outputs alongside one another.

    Composes an :class:`~srb.core.sensor.event_camera.recorder.EventRecorder`
    so MP4s and event HDF5 files share the same lifecycle (one timestamped
    record dir, one ``record_step`` / ``save_episodes`` pair). Either side
    may have zero sensors — the recorder no-ops accordingly.

    Instantiate after scene construction with the scene and recording
    parameters. Call record_step() each env step and save_episodes() when
    environments reset.

    One frame is captured per ``record_step``, so MP4s play at the env-step
    rate measured from successive ``sim_time_ns`` values; the camera
    ``update_period`` is only the fallback when no sim time is supplied.
    """

    def __init__(
        self,
        scene: "InteractiveScene",
        record_dir: Path | str,
        num_envs: int,
        *,
        event_provenance: dict[str, str] | None = None,
        record_dir_is_run_dir: bool = False,
        record_visual: bool = True,
        persist_event_artifacts: bool = True,
    ):
        self._cameras = list(iter_visual_sensor_outputs(scene)) if record_visual else []
        self._last_sim_time_ns: int | None = None
        self._capture_period_ns: int | None = None
        # Envs whose newest frame is a prime with no step since.
        self._primed_envs: set[int] = set()
        record_root = Path(record_dir)
        self._record_dir = (
            record_root
            if record_dir_is_run_dir
            else record_root.joinpath(
                f"{datetime.now().strftime(DEFAULT_DATETIME_FORMAT)}"
            )
        )
        self._event_recorder = EventRecorder(
            scene,
            self._record_dir,
            num_envs,
            provenance=event_provenance,
            record_dir_is_run_dir=True,
            persist_artifacts=persist_event_artifacts,
        )

        if not self._cameras and not self._event_recorder.has_sensors:
            logging.warning("VideoRecorder: no VISUAL or EVENT_CAMERA sensors in scene")
            return

        if self._cameras:
            self._record_dir.mkdir(parents=True, exist_ok=True)
            self._ep_frames: list[dict[str, deque]] = [
                defaultdict(lambda: deque()) for _ in range(num_envs)
            ]
            self._fps: dict[str, float] = {}
            for camera in self._cameras:
                validate_artifact_segment(
                    camera.image_basename,
                    field="camera image basename",
                )
                update_period = camera.sensor.cfg.update_period
                if update_period > 0:
                    self._fps[camera.image_basename] = 1.0 / update_period
                else:
                    raise ValueError(
                        f"Cannot record video for camera with non-positive update period: {update_period}"
                    )

    @property
    def record_dir(self) -> Path:
        return self._record_dir

    @property
    def event_manifest_path(self) -> Path | None:
        """Return the composed event recorder manifest, when present."""
        return self._event_recorder.event_manifest_path

    def recording_artifact_reference(self) -> dict[str, object]:
        """Describe the recorder-owned visual/event run tree."""
        reference = self._event_recorder.recording_artifact_reference()
        if self._cameras:
            reference["visual_recording"] = True
        if self._event_recorder.has_sensors and self._event_recorder.persists_artifacts:
            reference["event_recording"] = True
        return reference

    def episode_artifact_reference(self, env_id: int) -> dict[str, object]:
        """Describe one environment episode in the shared run tree."""
        reference = self._event_recorder.episode_artifact_reference(env_id)
        if self._cameras:
            reference["visual_recording"] = True
        if self._event_recorder.has_sensors and self._event_recorder.persists_artifacts:
            reference["event_recording"] = True
        return reference

    @property
    def has_sensors(self) -> bool:
        return bool(self._cameras) or self._event_recorder.has_sensors

    @property
    def persists_artifacts(self) -> bool:
        """Whether this composed runtime publishes visual or event files."""
        return bool(self._cameras) or self._event_recorder.persists_artifacts

    def record_step(self, *, sim_time_ns: int | None = None):
        """Capture current camera frames + event-camera outputs for all environments."""

        self._event_recorder.record_step(sim_time_ns=sim_time_ns)

        capture_visual_frame = True
        if self._cameras and sim_time_ns is not None:
            sim_time_ns = int(sim_time_ns)
            if self._last_sim_time_ns is not None:
                period_ns = sim_time_ns - self._last_sim_time_ns
                if period_ns > 0:
                    if self._capture_period_ns is None:
                        self._capture_period_ns = period_ns
                    self._last_sim_time_ns = sim_time_ns
                else:
                    capture_visual_frame = False
            else:
                self._last_sim_time_ns = sim_time_ns

        if not capture_visual_frame:
            return

        self._primed_envs.clear()
        self._append_frames(None)

    def prime_envs(self, env_ids: Sequence[int], *, sim_time_ns: int) -> None:
        """Open reset envs' new episodes with their post-reset frame.

        A selective reset happens at the terminal step's sim time, so
        :meth:`record_step` would skip this frame. Call after
        :meth:`save_episodes` and the post-reset render; other envs and the
        capture clock are untouched.
        """
        env_ids = [int(env_id) for env_id in env_ids]
        self._event_recorder.prime_envs(env_ids, sim_time_ns=sim_time_ns)
        if not self._cameras:
            return
        for env_id in env_ids:
            if env_id < 0 or env_id >= len(self._ep_frames):
                raise ValueError(f"env_id out of range: {env_id}")
            # A reset before any step replaces the previous reset's frame.
            if env_id in self._primed_envs:
                for frames in self._ep_frames[env_id].values():
                    if frames:
                        frames.pop()
        self._primed_envs.update(env_ids)
        self._append_frames(env_ids)

    def _append_frames(self, env_ids: Sequence[int] | None) -> None:
        """Append current camera frames for ``env_ids`` (``None``: all envs)."""
        for camera in self._cameras:
            unmerged = construct_visual_observation(
                image_basename=camera.image_basename,
                data_types=camera.data_types,
                clipping_range=camera.clipping_range,
                merge_channels=False,
                **camera.sensor.data.output,
            )
            for image_key, image in unmerged.items():
                for i in range(image.shape[0]) if env_ids is None else env_ids:
                    self._ep_frames[i][image_key].append(
                        video_frame_from_visual_image(image[i])
                    )

    def latest_event_observations(self):
        """Pass through to :class:`EventRecorder.latest_event_observations`."""
        return self._event_recorder.latest_event_observations()

    def save_episodes(
        self,
        env_ids: Sequence[int],
        *,
        outcomes: dict[int, dict[str, Any]] | None = None,
    ) -> list[Path]:
        """Save accumulated frames as MP4 videos + event HDF5 for reset environments."""
        self._event_recorder.validate_episode_outcomes(outcomes)
        written_videos = self._save_video_episodes(env_ids)
        event_outcomes = _merge_video_outcomes(
            self._record_dir,
            env_ids,
            outcomes or {},
            written_videos,
        )
        return self._event_recorder.save_episodes(env_ids, outcomes=event_outcomes)

    def _save_video_episodes(
        self,
        env_ids: Sequence[int],
        *,
        continue_on_error: bool = False,
    ) -> dict[int, dict[str, Path]]:
        if not self._cameras:
            return {}

        env_ids = [int(env_id) for env_id in env_ids]
        if any(env_id < 0 or env_id >= len(self._ep_frames) for env_id in env_ids):
            raise ValueError("env_id out of range")

        try:
            import cv2
        except ImportError:
            logging.warning(
                "OpenCV is unavailable; skipping MP4 video save while "
                "preserving event-camera artifacts"
            )
            for env_id in env_ids:
                self._ep_frames[env_id].clear()
            return {}

        written: dict[int, dict[str, Path]] = {}
        first_failure: BaseException | None = None
        for env_id in env_ids:
            if not self._ep_frames[env_id]:
                continue

            try:
                for image_key, frames in self._ep_frames[env_id].items():
                    if len(frames) < 2:
                        continue
                    validate_artifact_segment(image_key, field="video image key")

                    video_fps = self._video_fps(image_key)

                    # Share the event recorder's episode ID; a local counter
                    # drifted when resets had no frames to save.
                    episode_id = self._event_recorder.current_episode_id(env_id)
                    artifact_name = f"{image_key}_env{env_id}_ep{episode_id}.mp4"
                    staging_path, video_path = (
                        self._event_recorder.reserve_episode_companion(
                            env_id,
                            episode_id,
                            artifact_name,
                        )
                    )
                    logging.info(
                        f"Staging video for {video_path} ({len(frames)} frames)"
                    )

                    height, width = _validate_video_frames(frames, image_key=image_key)
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    video_writer = cv2.VideoWriter(
                        str(staging_path), fourcc, video_fps, (width, height)
                    )

                    if not video_writer.isOpened():
                        raise RuntimeError(
                            f"Could not open video writer for {video_path}"
                        )

                    try:
                        for frame in frames:
                            video_writer.write(frame)
                    finally:
                        video_writer.release()
                    if not staging_path.is_file() or staging_path.stat().st_size <= 0:
                        raise RuntimeError(
                            f"Video encoder produced no file for {video_path}"
                        )

                    written.setdefault(env_id, {})[image_key] = video_path
            except BaseException as exc:
                _result, cleanup_error = attempt_cleanup(
                    lambda error=exc: self._event_recorder.abort_episode(env_id, error)
                )
                if cleanup_error is not None:
                    exc.add_note(
                        f"video episode cleanup remains pending: {cleanup_error}"
                    )
                    if not continue_on_error:
                        raise cleanup_error from exc
                if first_failure is None:
                    first_failure = exc
                self._ep_frames[env_id].clear()
                if not continue_on_error:
                    raise
                continue

            self._ep_frames[env_id].clear()
        if first_failure is not None and not continue_on_error:
            raise first_failure
        return written

    def _video_fps(self, image_key: str) -> float:
        # Captures follow env steps; Isaac Lab 3.0 callers pass
        # ``common_step_counter * step_dt`` because current_time is unavailable.
        if self._capture_period_ns is not None:
            return max(1.0, 1.0e9 / self._capture_period_ns)
        # Match the complete camera basename in ``<basename>_<data_type>``.
        for basename, fps in self._fps.items():
            prefix = f"{basename}_"
            if image_key.startswith(prefix) and is_visual_data_type(
                image_key[len(prefix) :]
            ):
                return fps
        raise ValueError(f"Could not find FPS for {image_key!r}")

    def close(self) -> None:
        """Release recorder-owned resources.

        Event writers may still own the active episode when the environment
        closes, so delegate to their idempotent finalizer.
        """

        written_videos = self._save_video_episodes(
            range(len(self._ep_frames)) if self._cameras else (),
            continue_on_error=True,
        )
        outcomes = _merge_video_outcomes(
            self._record_dir,
            range(len(self._ep_frames)) if self._cameras else (),
            {},
            written_videos,
        )
        self._event_recorder.close(outcomes=outcomes)


def _merge_video_outcomes(
    record_dir: Path,
    env_ids: Sequence[int],
    outcomes: dict[int, dict[str, Any]],
    written_videos: dict[int, dict[str, Path]],
) -> dict[int, dict[str, Any]]:
    merged = {int(env_id): dict(outcomes.get(int(env_id), {})) for env_id in env_ids}
    for env_id, paths in written_videos.items():
        # Offer only colour clips by image key; the event recorder pairs each
        # event sensor with its co-located camera, so depth never poses as RGB.
        rgb_paths = {
            image_key: _relative_run_path(record_dir, path)
            for image_key, path in paths.items()
            if image_key.endswith(("_rgb", "_rgba"))
        }
        if rgb_paths:
            merged.setdefault(int(env_id), {})["rgb_mp4_paths"] = rgb_paths
    return merged


def _relative_run_path(record_dir: Path, path: Path) -> str:
    try:
        relative = path.resolve(strict=False).relative_to(record_dir.resolve())
    except ValueError as exc:
        raise ValueError(f"video artifact escapes record directory: {path}") from exc
    return validate_artifact_relative_path(
        relative.as_posix(), field="rgb_mp4_path"
    ).as_posix()


def _validate_video_frames(frames: Sequence[Any], *, image_key: str) -> tuple[int, int]:
    import numpy as np

    first = frames[0]
    if not isinstance(first, np.ndarray) or first.ndim != 3:
        raise ValueError(f"video frames for {image_key!r} must be rank-3 arrays")
    if first.dtype != np.uint8:
        raise ValueError(f"video frames for {image_key!r} must use uint8")
    height, width = first.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError(f"video frames for {image_key!r} are empty")
    expected = first.shape
    for index, frame in enumerate(frames):
        if index == 0:
            continue
        if not isinstance(frame, np.ndarray) or frame.shape != expected:
            raise ValueError(
                f"video frame {index} for {image_key!r} has shape "
                f"{getattr(frame, 'shape', None)}, expected {expected}"
            )
        if frame.dtype != np.uint8:
            raise ValueError(f"video frame {index} for {image_key!r} must use uint8")
    return int(height), int(width)
