from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Sequence

from srb.core.sensor.event_camera.recorder import iter_event_camera_sensors
from srb.core.sensor.visual_output import scene_has_visual_sensors
from srb.utils import logging

if TYPE_CHECKING:
    from isaaclab.scene import InteractiveScene


def scene_has_event_camera_sensors(scene: object) -> bool:
    """Return whether the scene exposes at least one EVENT_CAMERA sensor."""
    return next(iter_event_camera_sensors(scene), None) is not None


def scene_has_recordable_sensors(scene: object) -> bool:
    """Return whether episode recording has any visual or event payload to save."""
    return scene_has_visual_sensors(scene) or scene_has_event_camera_sensors(scene)


class EpisodeRecordingRuntime:
    """Environment adapter for per-episode visual and event artifacts."""

    def __init__(
        self,
        *,
        scene: "InteractiveScene",
        record_dir: Path | str,
        num_envs: int,
        event_provenance: dict[str, str] | None = None,
        record_visual: bool = True,
        persist_event_artifacts: bool = True,
    ) -> None:
        from datetime import datetime

        from srb.core.sensor.video_recorder import VideoRecorder

        try:
            from srb.utils.cfg import DEFAULT_DATETIME_FORMAT
        except ImportError:
            DEFAULT_DATETIME_FORMAT = "%Y%m%dT%H%M%S"

        self._record_dir = Path(record_dir).joinpath(
            datetime.now().strftime(DEFAULT_DATETIME_FORMAT)
        )
        self._recorder = VideoRecorder(
            scene=scene,
            record_dir=self._record_dir,
            num_envs=num_envs,
            event_provenance=event_provenance,
            record_dir_is_run_dir=True,
            record_visual=record_visual,
            persist_event_artifacts=persist_event_artifacts,
        )

    @property
    def record_dir(self) -> Path:
        return self._record_dir

    @property
    def event_manifest_path(self) -> Path | None:
        """Return the composed event manifest, when event sensors exist."""
        return self._recorder.event_manifest_path

    @property
    def persists_artifacts(self) -> bool:
        """Whether this runtime publishes visual or event files."""
        return self._recorder.persists_artifacts

    def recording_artifact_reference(self) -> dict[str, object]:
        """Describe the recorder-owned visual/event run tree."""
        return self._recorder.recording_artifact_reference()

    def episode_artifact_reference(self, env_id: int) -> dict[str, object]:
        """Describe one environment episode in the shared run tree."""
        return self._recorder.episode_artifact_reference(env_id)

    def record_step(self, *, sim_time_ns: int | None = None) -> None:
        self._recorder.record_step(sim_time_ns=sim_time_ns)

    def prime_envs(self, env_ids: Sequence[int], *, sim_time_ns: int) -> None:
        """Start reset envs' new episodes from the post-reset frame."""
        self._recorder.prime_envs(env_ids, sim_time_ns=sim_time_ns)

    def latest_event_observations(self) -> dict[str, Any]:
        """Latest per-step event observations; empty without event sensors."""
        return self._recorder.latest_event_observations()

    def save_episodes(
        self,
        env_ids: Sequence[int],
        *,
        outcomes: dict[int, dict[str, Any]] | None = None,
    ) -> list[Path]:
        return self._recorder.save_episodes(env_ids, outcomes=outcomes)

    def close(self) -> None:
        if hasattr(self._recorder, "close"):
            self._recorder.close()


def make_episode_recording_runtime(
    *,
    scene: "InteractiveScene",
    cfg: object,
    num_envs: int,
    event_provenance: dict[str, str] | None = None,
) -> EpisodeRecordingRuntime | None:
    """Build the configured episode recorder, or return ``None``."""
    event_sensors = scene_has_event_camera_sensors(scene)
    persist_artifacts = bool(
        getattr(cfg, "camera_record", False)
        or getattr(cfg, "event_record_requested", False)
    )

    if getattr(cfg, "event_record_requested", False) and not event_sensors:
        raise RuntimeError(
            "event recording was requested but the task scene has no "
            "EVENT_CAMERA sensor; add `events` to env.active_sensors"
        )

    if not persist_artifacts and not event_sensors:
        return None

    if persist_artifacts and not scene_has_recordable_sensors(scene):
        logging.warning(
            "EpisodeRecordingRuntime: camera_record=True but scene has no "
            "VISUAL or EVENT_CAMERA sensors; recording disabled"
        )
        return None

    return EpisodeRecordingRuntime(
        scene=scene,
        record_dir=getattr(cfg, "camera_record_dir", Path(".")),
        num_envs=num_envs,
        event_provenance=event_provenance,
        record_visual=bool(getattr(cfg, "camera_record", False)),
        persist_event_artifacts=persist_artifacts,
    )


__all__ = [
    "EpisodeRecordingRuntime",
    "make_episode_recording_runtime",
    "scene_has_event_camera_sensors",
    "scene_has_recordable_sensors",
]
