"""Record per-episode event HDF5 files and provide per-step observations.

Each reset publishes event files per sensor and episode. ``NoopBackend``
produces valid empty files; frames are consumed once per sensor tick and their
measured rate is stored in the manifest.
"""

from __future__ import annotations

import os
import stat
from collections.abc import Iterable, Iterator, Mapping, Sequence
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable
from uuid import uuid4

from srb.core.sensor.event_camera.backend import (
    EventGenerator,
    validate_event_chunk,
)
from srb.core.sensor.event_camera.config import EventCameraCfg
from srb.core.sensor.event_camera.dispatch import (
    EventBackendDispatch,
    EventObservationOverflowError,
)
from srb.core.sensor.event_camera.manifest import (
    EventManifestRow,
    append_event_manifest_rows,
    create_event_manifest,
    parse_event_reward_summary,
)
from srb.core.sensor.event_camera.seeding import (
    bake_hot_pixel_mask,
    derive_episode_event_seed_pack,
    distinct_sensor_base_seeds,
)
from srb.core.sensor.event_camera.storage import (
    EventEpisodeMetadata,
    EventEpisodeProvenance,
    EventHDF5Writer,
)
from srb.utils.artifact_paths import (
    validate_artifact_relative_path,
    validate_artifact_segment,
)
from srb.utils.failures import attempt_cleanup
from srb.utils.publication import (
    OwnedTemporaryDirectory,
    PublicationCleanupError,
    PublicationCommitCleanupError,
)

if TYPE_CHECKING:
    import torch


def _coalesce_event_buffers(bufs: "Sequence[torch.Tensor]") -> "torch.Tensor":
    """Concatenate per-step event buffers into one ``(N, 4)`` int64 tensor.

    Returns the sole buffer as-is, or an empty ``(0, 4)`` int64 tensor.
    Imports ``torch`` lazily so this module remains importable without it.
    """
    import torch

    if len(bufs) == 1:
        return bufs[0]
    if bufs:
        return torch.cat(bufs, dim=0)
    return torch.zeros((0, 4), dtype=torch.int64)


def _read_finalized_event_sha256(published_path: Path) -> str:
    """Read the writer's finalized digest without materializing event payloads."""
    try:
        import h5py
    except ImportError as exc:  # pragma: no cover - writer already requires h5py
        raise RuntimeError(
            "h5py is required to publish event manifest digests"
        ) from exc

    with h5py.File(published_path, "r") as h5:
        try:
            raw_digest = h5["/metadata"].attrs["event_sha256"]
        except (KeyError, OSError) as exc:
            raise RuntimeError(
                "finalized event HDF5 publication is missing event_sha256"
            ) from exc

    if isinstance(raw_digest, (bytes, bytearray)):
        digest = raw_digest.decode("ascii", errors="replace")
    else:
        digest = str(raw_digest)
    if len(digest) != 64 or any(
        character not in "0123456789abcdefABCDEF" for character in digest
    ):
        raise RuntimeError(
            "finalized event HDF5 publication has an invalid event_sha256"
        )
    return digest.lower()


def _same_directory_identity(path: Path, identity: tuple[int, int]) -> bool:
    try:
        # Ownership checks must inspect the directory entry itself.  Following
        # a replacement symlink could promote a foreign tree at commit time.
        current = path.lstat()
    except OSError:
        return False
    return (
        stat.S_ISDIR(current.st_mode)
        and (
            current.st_dev,
            current.st_ino,
        )
        == identity
    )


def _matching_committed_path(
    error: PublicationCleanupError,
    expected: Path,
) -> Path | None:
    """Return the expected path only when the error explicitly names it."""
    expected_absolute = os.path.abspath(os.fspath(expected))
    for candidate in error.committed_paths:
        candidate_path = Path(candidate)
        if os.path.abspath(os.fspath(candidate_path)) == expected_absolute:
            return candidate_path
    return None


def _unique_paths(values: Sequence[Path]) -> tuple[Path, ...]:
    return tuple(dict.fromkeys(values))


def _unique_committed_paths(
    errors: Sequence[PublicationCleanupError],
) -> tuple[Path, ...]:
    return _unique_paths([path for error in errors for path in error.committed_paths])


def _paired_rgb_mp4_path(sensor_name: str, outcome: Mapping[str, Any]) -> str:
    """Return the RGB clip co-located with ``event_camera_<name>``, else empty.

    An explicit ``rgb_mp4_path`` applies to every sensor. Otherwise only the
    ``image_<name>_rgb`` clip pairs, never another camera's clip or depth.
    """
    explicit = outcome.get("rgb_mp4_path")
    if explicit:
        return str(explicit)
    camera_name = sensor_name.removeprefix("event_camera_")
    if camera_name == sensor_name or not camera_name:
        return ""
    from srb.core.sensor.visual_output import visual_image_basename

    basename = visual_image_basename(camera_name)
    clips = outcome.get("rgb_mp4_paths", {})
    return str(clips.get(f"{basename}_rgb") or clips.get(f"{basename}_rgba") or "")


def _validate_episode_outcome(outcome: object) -> None:
    """Validate outcome fields before an episode acquires a publication lease."""

    if not isinstance(outcome, Mapping):
        raise TypeError(
            "event-camera episode outcome must be a mapping; "
            f"got {type(outcome).__name__}"
        )

    rgb_mp4_path = outcome.get("rgb_mp4_path", "")
    if rgb_mp4_path is not None and str(rgb_mp4_path):
        validate_artifact_relative_path(
            str(rgb_mp4_path),
            field="rgb_mp4_path",
        )
    rgb_mp4_paths = outcome.get("rgb_mp4_paths", {})
    if not isinstance(rgb_mp4_paths, Mapping):
        raise TypeError("rgb_mp4_paths must map image keys to relative paths")
    for path in rgb_mp4_paths.values():
        validate_artifact_relative_path(str(path), field="rgb_mp4_paths")

    if outcome.get("success_flag", 0) not in (0, 1):
        raise ValueError("success_flag must be 0 or 1")
    reward_summary = outcome.get("reward_summary", "")
    parse_event_reward_summary("" if reward_summary is None else str(reward_summary))


class _EventPublicationCleanupOwner:
    """Retain event publication owners and finish one pending episode once."""

    def __init__(
        self,
        operations: Sequence[Callable[[], object]],
        *,
        post_operations: Sequence[Callable[[], object]] = (),
        on_complete: Callable[[], object] | None = None,
        committed_paths: Sequence[Path] = (),
    ) -> None:
        self._operations = list(operations)
        self._post_operations = list(post_operations)
        self._on_complete = on_complete
        self._committed_paths = tuple(Path(path) for path in committed_paths)
        self._completed = False

    @property
    def committed_paths(self) -> tuple[Path, ...]:
        return self._committed_paths

    def _merge_committed_paths(self, errors: Sequence[BaseException]) -> None:
        self._committed_paths = _unique_paths(
            [
                *self._committed_paths,
                *(
                    path
                    for error in errors
                    if isinstance(error, PublicationCleanupError)
                    for path in error.committed_paths
                ),
            ]
        )

    def retry_cleanup(self) -> None:
        if self._completed:
            return

        pending: list[Callable[[], object]] = []
        errors: list[BaseException] = []
        for operation in tuple(self._operations):
            _result, exc = attempt_cleanup(operation)
            if exc is not None:
                pending.append(operation)
                errors.append(exc)
        self._operations = pending
        self._merge_committed_paths(errors)
        if errors:
            raise PublicationCleanupError(
                "event publication cleanup remains pending",
                owner=self,
                committed_paths=self._committed_paths,
                errors=errors,
            ) from errors[0]

        post_pending: list[Callable[[], object]] = []
        post_errors: list[BaseException] = []
        for operation in tuple(self._post_operations):
            _result, exc = attempt_cleanup(operation)
            if exc is not None:
                post_pending.append(operation)
                post_errors.append(exc)
        self._post_operations = post_pending
        self._merge_committed_paths(post_errors)
        if post_errors:
            raise PublicationCleanupError(
                "event publication cleanup remains pending",
                owner=self,
                committed_paths=self._committed_paths,
                errors=post_errors,
            ) from post_errors[0]

        if self._on_complete is not None:
            _result, exc = attempt_cleanup(self._on_complete)
            if exc is not None:
                if not isinstance(exc, PublicationCleanupError):
                    raise exc
                self._operations = [exc.owner.retry_cleanup]
                self._merge_committed_paths((exc,))
                raise PublicationCleanupError(
                    "event publication cleanup remains pending",
                    owner=self,
                    committed_paths=self._committed_paths,
                    errors=(exc,),
                ) from exc
        self._completed = True


class _EventEpisodeStagingCleanupOwner:
    """Delete one bound private episode tree, retaining failed cleanup."""

    def __init__(self, transaction: "_EpisodeTransaction") -> None:
        self._transaction = transaction
        self._owner: OwnedTemporaryDirectory | None = None
        self._completed = False

    def retry_cleanup(self) -> None:
        if self._completed:
            return
        if not _same_directory_identity(
            self._transaction.staging_dir.parent,
            self._transaction.parent_identity,
        ) or not _same_directory_identity(
            self._transaction.staging_dir,
            self._transaction.staging_identity,
        ):
            raise RuntimeError(
                "event episode staging directory was replaced before cleanup"
            )
        if self._owner is None:
            self._owner = OwnedTemporaryDirectory.bind(self._transaction.staging_dir)
        self._owner.cleanup()
        self._completed = True


@dataclass
class _PendingEpisode:
    transaction: "_EpisodeTransaction"
    rows: list[EventManifestRow]
    published_paths: list[Path]
    manifest_appended: bool = False


@dataclass(frozen=True)
class EventSensorRef:
    """Pair a scene sensor with its config, preserving any bound config."""

    name: str
    sensor: Any  # Isaac Lab SensorBase-like, with ``.cfg`` and ``.data``
    original_cfg: EventCameraCfg | None = None

    @property
    def cfg(self) -> EventCameraCfg:
        if self.original_cfg is not None:
            return self.original_cfg
        return self.sensor.cfg


def iter_event_camera_sensors(scene: object) -> Iterator[EventSensorRef]:
    """Yield bound or legacy event-camera sensors exactly once by scene name."""
    from srb.core.scene_introspection import iter_scene_sensors
    from srb.core.sensor.category import SensorCategory, get_sensor_category
    from srb.core.sensor.event_camera.materialization import (
        get_bound_event_camera_cfgs,
    )

    bound_cfgs = get_bound_event_camera_cfgs(scene)
    seen_names: set[str] = set()
    seen_sensor_ids: set[int] = set()
    for sensor_name, sensor in iter_scene_sensors(scene):
        if sensor_name in seen_names or id(sensor) in seen_sensor_ids:
            continue
        seen_names.add(sensor_name)
        seen_sensor_ids.add(id(sensor))

        if sensor_name in bound_cfgs:
            yield EventSensorRef(
                name=sensor_name,
                sensor=sensor,
                original_cfg=bound_cfgs[sensor_name],
            )
            continue

        if (
            get_sensor_category(getattr(sensor, "cfg", None))
            != SensorCategory.EVENT_CAMERA
        ):
            continue
        yield EventSensorRef(name=sensor_name, sensor=sensor)


class EventRecorder:
    """Per-episode HDF5 recorder for event-camera sensors.

    Each ``(env_id, sensor_name)`` has one backend and active writer. Chunks
    are appended by :meth:`record_step`; only the latest step stays in memory.
    :meth:`reset` reseeds backends with
    ``derive(derive(base, env_id), episode_id)`` and records each effective
    seed. ``base`` is ``cfg.rng_seed`` or ``run_seed`` (default
    ``torch.initial_seed()``).

    ``record_dir`` gets a timestamped child directory unless
    ``record_dir_is_run_dir`` is true.
    """

    def __init__(
        self,
        scene: object,
        record_dir: Path | str,
        num_envs: int,
        *,
        provenance: dict[str, str] | None = None,
        timestamp_format: str | None = None,
        record_dir_is_run_dir: bool = False,
        persist_artifacts: bool = True,
        run_seed: int | None = None,
    ) -> None:
        try:
            from srb.utils.cfg import DEFAULT_DATETIME_FORMAT
        except ImportError:
            DEFAULT_DATETIME_FORMAT = "%Y%m%dT%H%M%S"

        self._num_envs = int(num_envs)
        if self._num_envs < 0:
            raise ValueError(f"num_envs must be >= 0; got {num_envs}")
        self._sensors: list[EventSensorRef] = list(iter_event_camera_sensors(scene))
        for ref in self._sensors:
            validate_artifact_segment(ref.name, field="event sensor name")
        record_root = Path(record_dir)
        self._record_dir = (
            record_root
            if record_dir_is_run_dir
            else record_root.joinpath(
                datetime.now().strftime(timestamp_format or DEFAULT_DATETIME_FORMAT)
            )
        )
        self._provenance = dict(provenance or {})
        self._persist_artifacts = bool(persist_artifacts)

        self._ep_counter = [0] * self._num_envs

        self._writers: dict[tuple[int, str], EventHDF5Writer | None] = {}
        self._transactions: dict[int, _EpisodeTransaction] = {}
        self._pending_episodes: dict[int, _PendingEpisode] = {}
        self._pending_publication_cleanup: dict[int, PublicationCleanupError] = {}
        self._companion_paths: dict[int, list[Path]] = {}
        self._env_failures: dict[int, BaseException] = {}
        self._close_failure: BaseException | None = None
        self._closed = False
        self._closed_paths: tuple[Path, ...] = ()
        self._close_finalized_envs: set[int] = set()
        # Masks persist across episodes and depend on env_id, not rng_seed.
        self._hot_pixel_masks: dict[tuple[int, str], tuple[tuple[int, int], ...]] = {}
        # Keep the latest observation tick separate from episode writer state.
        self._step_events: dict[tuple[int, str], list["torch.Tensor"]] = {}
        self._step_event_counts: dict[tuple[int, str], int] = {}
        # Run-cumulative timestamps support measured refresh-rate provenance.
        self._env_frame_counts: dict[tuple[int, str], int] = {}
        self._env_first_frame_ns: dict[tuple[int, str], int] = {}
        self._env_last_frame_ns: dict[tuple[int, str], int] = {}
        # Envs whose newest frame is a prime with no step since.
        self._primed_envs: set[int] = set()

        if not self._sensors:
            return

        base_seeds = distinct_sensor_base_seeds(
            (ref.name, ref.cfg.base_seed) for ref in self._sensors
        )
        for ref in self._sensors:
            cfg = ref.cfg
            base_seed = base_seeds[ref.name]
            hot_pixel_frac = (
                cfg.hot_pixel_frac if cfg.hot_pixel_frac is not None else 0.0
            )
            eff_w, eff_h = cfg.effective_resolution
            for env_id in range(self._num_envs):
                _, hot_pixel_seed = derive_episode_event_seed_pack(base_seed, env_id)
                self._hot_pixel_masks[(env_id, ref.name)] = bake_hot_pixel_mask(
                    hot_pixel_seed,
                    width=eff_w,
                    height=eff_h,
                    fraction=hot_pixel_frac,
                )
                key = (env_id, ref.name)
                self._writers[key] = None
                self._step_events[key] = []
                self._step_event_counts[key] = 0

        self._dispatch = EventBackendDispatch(
            self._sensors,
            self._num_envs,
            run_seed=run_seed,
            hot_pixel_masks=self._hot_pixel_masks,
        )
        # Do not leave a valid-looking empty manifest when backend setup fails.
        if self._persist_artifacts:
            self._manifest_path = create_event_manifest(self._record_dir)

    @property
    def record_dir(self) -> Path:
        return self._record_dir

    @property
    def event_manifest_path(self) -> Path | None:
        """Return the run manifest when event sensors are configured."""
        return getattr(self, "_manifest_path", None)

    def recording_artifact_reference(self) -> dict[str, object]:
        """Describe the recorder-owned run tree for dataset provenance."""
        reference: dict[str, object] = {
            "schema": "srb.episode_recording.v1",
            "record_dir": str(self._record_dir),
        }
        manifest_path = self.event_manifest_path
        if manifest_path is not None:
            reference["event_manifest"] = str(manifest_path)
        return reference

    def episode_artifact_reference(self, env_id: int) -> dict[str, object]:
        """Describe one environment episode inside the recorder-owned tree."""
        env_id = int(env_id)
        episode_id = self.current_episode_id(env_id)
        reference = self.recording_artifact_reference()
        reference.update(
            {
                "env_id": env_id,
                "episode_id": episode_id,
                "episode_dir": f"events_env{env_id}_ep{episode_id}",
            }
        )
        return reference

    @property
    def has_sensors(self) -> bool:
        return bool(self._sensors)

    @property
    def persists_artifacts(self) -> bool:
        """Whether this dispatcher owns on-disk event artifacts."""
        return self._persist_artifacts

    def current_episode_id(self, env_id: int) -> int:
        """Episode index this recorder is currently accumulating for *env_id*.

        Companion artifact reservations use this ID, keeping filenames aligned
        with the recorder's episode index.
        """
        env_id = int(env_id)
        if env_id < 0 or env_id >= self._num_envs:
            raise ValueError(f"env_id out of range: {env_id}")
        return self._ep_counter[env_id]

    def record_step(self, *, sim_time_ns: int | None = None) -> None:
        """Drive every backend with the latest RGB frame from each env.

        Consumes each sensor frame once when its sub-render period elapses,
        stamped with ``sim_time_ns``. Env backends share a sensor timestamp;
        if omitted, time advances by one canonical interval.
        """
        self._ensure_usable()
        pending_episode = next(iter(self._pending_episodes.values()), None)
        if pending_episode is not None:
            raise RuntimeError(
                "event episode manifest publication is pending; retry reset "
                "before recording another step"
            )
        if not self._sensors:
            return
        pending_cleanup = next(iter(self._pending_publication_cleanup.values()), None)
        if pending_cleanup is not None:
            raise pending_cleanup
        for bufs in self._step_events.values():
            bufs.clear()
        for key in self._step_event_counts:
            self._step_event_counts[key] = 0

        # Rejected chunks fail their env; dispatch closes its stream and keeps peers running.
        sink = self._accept_event_chunk
        cleanup_failures: list[BaseException] = []

        def error_sink(key: tuple[int, str], exc: BaseException) -> None:
            _result, cleanup_exc = attempt_cleanup(lambda: self._fail_env(key[0], exc))
            if cleanup_exc is not None:
                cleanup_failures.append(cleanup_exc)

        self._dispatch.record_step(
            sim_time_ns=sim_time_ns,
            sink=sink,
            error_sink=error_sink,
        )
        self._primed_envs -= self._count_consumed_frames(range(self._num_envs))
        if cleanup_failures:
            raise cleanup_failures[0]

    def prime_envs(self, env_ids: Sequence[int], *, sim_time_ns: int) -> None:
        """Open reset envs' new episodes with their post-reset frame.

        Call after :meth:`save_episodes` and the post-reset render. Each
        backend takes the frame as its baseline at ``sim_time_ns`` without
        emitting events, and the new episode's first observation drops the
        finished episode's terminal-tick events.
        """
        self._ensure_usable()
        if not self._sensors:
            return
        env_ids = [
            int(env_id) for env_id in env_ids if int(env_id) not in self._env_failures
        ]
        cleanup_failures: list[BaseException] = []

        def error_sink(key: tuple[int, str], exc: BaseException) -> None:
            _result, cleanup_exc = attempt_cleanup(lambda: self._fail_env(key[0], exc))
            if cleanup_exc is not None:
                cleanup_failures.append(cleanup_exc)

        # A reset before any step replaces the previous reset's baseline, so
        # backends restart instead of seeing a frame at the same timestamp.
        for env_id in env_ids:
            if env_id in self._primed_envs:
                self._dispatch.reset_env(env_id, episode_id=self._ep_counter[env_id])
        self._primed_envs.update(env_ids)
        self._dispatch.prime_envs(
            env_ids,
            sim_time_ns=sim_time_ns,
            error_sink=error_sink,
        )
        for env_id in env_ids:
            for ref in self._sensors:
                key = (env_id, ref.name)
                self._step_events[key].clear()
                self._step_event_counts[key] = 0
        # A prime at the terminal tick's time adds no frame interval.
        self._count_consumed_frames(env_ids)
        if cleanup_failures:
            raise cleanup_failures[0]

    def _count_consumed_frames(self, env_ids: Iterable[int]) -> set[int]:
        """Count newly consumed frames; return the envs that consumed one."""
        consumed: set[int] = set()
        for env_id in env_ids:
            for ref in self._sensors:
                key = (env_id, ref.name)
                frame_ns = self._dispatch.last_frame_ns(env_id, ref.name)
                if frame_ns is None or frame_ns == self._env_last_frame_ns.get(key):
                    continue
                self._env_first_frame_ns.setdefault(key, frame_ns)
                self._env_last_frame_ns[key] = frame_ns
                self._env_frame_counts[key] = self._env_frame_counts.get(key, 0) + 1
                consumed.add(env_id)
        return consumed

    def _accept_event_chunk(
        self,
        key: tuple[int, str],
        events: "torch.Tensor",
    ) -> None:
        env_id, sensor_name = key
        if env_id in self._env_failures:
            return
        ref = next(ref for ref in self._sensors if ref.name == sensor_name)
        try:
            rows = validate_event_chunk(
                events,
                max_rows=ref.cfg.event_chunk_rows,
                key=key,
            )
            if rows == 0:
                return
            accepted = self._step_event_counts[key]
            limit = ref.cfg.resolved_observation_window_events()
            if accepted + rows > limit:
                raise EventObservationOverflowError(
                    key=key,
                    accepted_rows=accepted,
                    incoming_rows=rows,
                    max_rows=limit,
                )
            # Check limits before either the writer or observation retains data.
            self._append_event_chunk(key, events)
            self._step_events[key].append(events)
            self._step_event_counts[key] = accepted + rows
        except BaseException as exc:
            self._fail_env(env_id, exc)
            raise

    def _ensure_usable(self) -> None:
        if self._closed:
            if self._close_failure is not None:
                raise self._close_failure
            raise RuntimeError("EventRecorder is closed")

    def _require_publication_cleanup_complete(self, env_id: int) -> None:
        pending = self._pending_publication_cleanup.get(env_id)
        if pending is not None:
            raise pending

    def _retain_publication_cleanup(
        self,
        env_id: int,
        error: PublicationCleanupError,
        *,
        operations: Sequence[Callable[[], object]],
        post_operations: Sequence[Callable[[], object]] = (),
        on_complete: Callable[[], object] | None = None,
        committed_paths: Sequence[Path] = (),
    ) -> PublicationCleanupError:
        paths = list(error.committed_paths)
        for path in committed_paths:
            if path not in paths:
                paths.append(path)
        owner = _EventPublicationCleanupOwner(
            operations,
            post_operations=post_operations,
            on_complete=on_complete,
            committed_paths=paths,
        )
        error.owner = owner
        error.cleanup_owner = owner
        error.committed_paths = tuple(paths)
        self._pending_publication_cleanup[env_id] = error
        return error

    def _retain_uncommitted_publication_cleanup(
        self,
        env_id: int,
        errors: Sequence[PublicationCleanupError],
        *,
        message: str,
        cause: BaseException | None = None,
    ) -> PublicationCleanupError:
        first_error = errors[0]
        causes = tuple(cause for error in errors for cause in error.errors)
        if cause is not None:
            causes = (*causes, cause)
        cleanup_error = PublicationCleanupError(
            message,
            owner=first_error.owner,
            committed_paths=_unique_committed_paths(errors),
            committed_result=first_error.committed_result,
            errors=causes,
        )
        operations = [error.owner.retry_cleanup for error in errors]
        post_operations: list[Callable[[], object]] = []
        transaction = self._transactions.get(env_id)
        if transaction is not None and not transaction.committed:
            post_operations.append(
                _EventEpisodeStagingCleanupOwner(transaction).retry_cleanup
            )
        self._retain_publication_cleanup(
            env_id,
            cleanup_error,
            operations=operations,
            post_operations=post_operations,
            on_complete=lambda: self._fail_env(
                env_id,
                cleanup_error,
                staging_already_cleaned=True,
            ),
        )
        return cleanup_error

    def _clear_publication_cleanup(self, env_id: int) -> None:
        self._pending_publication_cleanup.pop(env_id, None)

    def _clear_writer_state(
        self, env_id: int, *, clear_step_events: bool = True
    ) -> None:
        for ref in self._sensors:
            key = (env_id, ref.name)
            self._writers[key] = None
            if clear_step_events:
                self._step_events[key].clear()
                self._step_event_counts[key] = 0

    def _advance_episode_state(
        self,
        env_id: int,
        transaction: "_EpisodeTransaction",
        *,
        retain_transaction: bool,
    ) -> None:
        if not transaction.accounted:
            self._ep_counter[env_id] += 1
            transaction.accounted = True
        # Preserve the latest observation until the next record_step.
        self._clear_writer_state(env_id, clear_step_events=False)
        if retain_transaction:
            return
        if self._transactions.get(env_id) is transaction:
            self._transactions.pop(env_id, None)
        self._companion_paths.pop(env_id, None)

    def _complete_pending_episode(self, env_id: int) -> None:
        pending = self._pending_episodes.get(env_id)
        if pending is None:
            # A nested cleanup retry may finish the manifest before this owner.
            self._clear_publication_cleanup(env_id)
            return
        transaction = pending.transaction
        if not transaction.committed:
            transaction.commit()
        if not pending.manifest_appended:
            try:
                append_event_manifest_rows(self._manifest_path, pending.rows)
            except PublicationCleanupError as exc:
                manifest_cleanup_error = PublicationCommitCleanupError(
                    "event episode committed but manifest cleanup is pending",
                    owner=exc.owner,
                    committed_paths=_unique_paths(
                        [
                            *pending.published_paths,
                            transaction.final_dir,
                            *exc.committed_paths,
                        ]
                    ),
                    committed_result=exc.committed_result,
                    errors=exc.errors,
                )
                if _matching_committed_path(exc, self._manifest_path) is not None:
                    # Both publications are authoritative; only cleanup remains.
                    pending.manifest_appended = True
                    self._pending_episodes.pop(env_id, None)
                    self._advance_episode_state(
                        env_id,
                        transaction,
                        retain_transaction=False,
                    )
                else:
                    if exc.committed_paths:
                        cleanup_error = self._retain_uncommitted_publication_cleanup(
                            env_id,
                            [manifest_cleanup_error],
                            message=(
                                "event manifest publication completed at an "
                                "unexpected path"
                            ),
                        )
                        raise cleanup_error from exc
                    # Keep the committed episode for a retryable manifest append.
                    raise manifest_cleanup_error from exc
                raise manifest_cleanup_error from exc
            pending.manifest_appended = True
        self._pending_episodes.pop(env_id, None)
        self._advance_episode_state(env_id, transaction, retain_transaction=False)
        self._clear_publication_cleanup(env_id)

    def _aggregate_close_publication_cleanup(
        self,
        errors: Sequence[PublicationCleanupError],
        *,
        written: Sequence[Path],
        pending_paths: Sequence[Path],
        terminal_failure: BaseException | None,
    ) -> PublicationCleanupError:
        first_error = errors[0]
        committed_paths = _unique_committed_paths(errors)

        def finish_close() -> None:
            self._closed_paths = _unique_paths([*written, *pending_paths])
            self._close_failure = next(
                iter(self._env_failures.values()), terminal_failure
            )

        owner = _EventPublicationCleanupOwner(
            [error.owner.retry_cleanup for error in errors],
            on_complete=finish_close,
            committed_paths=committed_paths,
        )
        error_type = (
            PublicationCommitCleanupError
            if all(isinstance(error, PublicationCommitCleanupError) for error in errors)
            else PublicationCleanupError
        )
        causes = tuple(cause for error in errors for cause in error.errors)
        if terminal_failure is not None:
            causes = (*causes, terminal_failure)
        return error_type(
            "event recorder publication cleanup is pending",
            owner=owner,
            committed_paths=committed_paths,
            committed_result=first_error.committed_result,
            errors=causes,
        )

    @staticmethod
    def _aggregate_reset_publication_cleanup(
        errors: Sequence[PublicationCleanupError],
        *,
        terminal_failure: BaseException | None,
    ) -> PublicationCleanupError:
        first_error = errors[0]
        error_type = (
            PublicationCommitCleanupError
            if all(isinstance(error, PublicationCommitCleanupError) for error in errors)
            else PublicationCleanupError
        )
        causes = tuple(cause for error in errors for cause in error.errors)
        if terminal_failure is not None:
            causes = (*causes, terminal_failure)
        owner = _EventPublicationCleanupOwner(
            [error.owner.retry_cleanup for error in errors],
            committed_paths=_unique_committed_paths(errors),
        )
        return error_type(
            "event recorder reset cleanup is pending",
            owner=owner,
            committed_paths=_unique_committed_paths(errors),
            committed_result=first_error.committed_result,
            errors=causes,
        )

    def _staged_episode_path(
        self,
        env_id: int,
        sensor_name: str,
        episode_id: int,
    ) -> Path:
        transaction = self._transaction_for(env_id, episode_id)
        return transaction.staging_dir / (
            f"events_{sensor_name}_env{env_id}_ep{episode_id}.h5"
        )

    def _transaction_for(
        self,
        env_id: int,
        episode_id: int,
    ) -> _EpisodeTransaction:
        transaction = self._transactions.get(env_id)
        if transaction is not None:
            if transaction.episode_id != episode_id:
                raise RuntimeError(
                    f"environment {env_id} already has episode "
                    f"{transaction.episode_id} staged"
                )
            return transaction

        final_dir = self._record_dir / f"events_env{env_id}_ep{episode_id}"
        if final_dir.exists() or final_dir.is_symlink():
            raise FileExistsError(f"event episode already exists: {final_dir}")
        staging_dir = self._record_dir / (
            f".events_env{env_id}_ep{episode_id}.{uuid4().hex}.stage"
        )
        staging_dir.mkdir(parents=True, exist_ok=False)
        record_dir_stat = self._record_dir.lstat()
        staging_stat = staging_dir.lstat()
        transaction = _EpisodeTransaction(
            env_id=env_id,
            episode_id=episode_id,
            staging_dir=staging_dir,
            final_dir=final_dir,
            staging_identity=(staging_stat.st_dev, staging_stat.st_ino),
            parent_identity=(record_dir_stat.st_dev, record_dir_stat.st_ino),
        )
        self._transactions[env_id] = transaction
        return transaction

    def reserve_episode_companion(
        self,
        env_id: int,
        episode_id: int,
        artifact_name: str,
    ) -> tuple[Path, Path]:
        """Reserve one safe file inside the episode's private transaction."""

        self._ensure_usable()
        if not self._persist_artifacts:
            raise RuntimeError(
                "episode companions require an artifact-persisting event recorder"
            )
        if env_id < 0 or env_id >= self._num_envs:
            raise ValueError(f"env_id out of range: {env_id}")
        if episode_id != self._ep_counter[env_id]:
            raise ValueError(
                f"episode_id mismatch for env {env_id}: expected "
                f"{self._ep_counter[env_id]}, got {episode_id}"
            )
        self._require_publication_cleanup_complete(env_id)
        name = validate_artifact_segment(
            artifact_name,
            field="episode companion artifact name",
        )
        transaction = self._transaction_for(env_id, episode_id)
        staging = transaction.staging_dir / name
        final = transaction.final_dir / name
        paths = self._companion_paths.setdefault(env_id, [])
        if final in paths or staging.exists():
            raise FileExistsError(f"episode companion already reserved: {final}")
        paths.append(final)
        return staging, final

    def abort_episode(self, env_id: int, cause: BaseException) -> None:
        """Abort a private episode tree after a companion producer fails."""

        self._fail_env(int(env_id), cause)

    @staticmethod
    def _published_episode_path(
        transaction: _EpisodeTransaction,
        sensor_name: str,
    ) -> Path:
        return transaction.final_dir / (
            f"events_{sensor_name}_env{transaction.env_id}_ep"
            f"{transaction.episode_id}.h5"
        )

    def _create_writer(
        self,
        ref: EventSensorRef,
        env_id: int,
        ep: int,
        outcome: dict[str, Any] | None = None,
    ) -> EventHDF5Writer:
        provenance = self._build_provenance(ref, env_id, ep, outcome)
        metadata = EventEpisodeMetadata(**asdict(provenance))
        return EventHDF5Writer(
            self._staged_episode_path(env_id, ref.name, ep),
            metadata,
            # Match the writer limit to dispatch's configured chunk size.
            max_batch_rows=ref.cfg.event_chunk_rows,
            publish_on_close=False,
        )

    def _writer_for(
        self,
        key: tuple[int, str],
        ref: EventSensorRef,
        env_id: int,
        ep: int,
    ) -> EventHDF5Writer:
        failure = self._env_failures.get(env_id)
        if failure is not None:
            raise failure
        writer = self._writers[key]
        if writer is None:
            writer = self._create_writer(ref, env_id, ep)
            self._writers[key] = writer
        return writer

    def _append_event_chunk(self, key: tuple[int, str], events: "torch.Tensor") -> None:
        if not self._persist_artifacts and self._sensors:
            return
        env_id, sensor_name = key
        ref = next(ref for ref in self._sensors if ref.name == sensor_name)
        try:
            self._writer_for(key, ref, env_id, self._ep_counter[env_id]).append(events)
        except BaseException as exc:
            self._fail_env(env_id, exc)
            raise

    @staticmethod
    def _abort_writer(writer: EventHDF5Writer, exc: BaseException) -> None:
        try:
            abort = getattr(writer, "abort", None)
            if callable(abort):
                abort(exc)
            else:
                writer.__exit__(type(exc), exc, exc.__traceback__)
        except BaseException:
            # Preserve the original writer failure if abort also fails.
            pass

    def _finalize_failed_env(self, env_id: int, exc: BaseException) -> None:
        self._env_failures[env_id] = exc
        self._transactions.pop(env_id, None)
        self._pending_episodes.pop(env_id, None)
        self._pending_publication_cleanup.pop(env_id, None)
        self._companion_paths.pop(env_id, None)

    def _fail_env(
        self,
        env_id: int,
        exc: BaseException,
        *,
        staging_already_cleaned: bool = False,
    ) -> None:
        if env_id in self._env_failures:
            return
        transaction = self._transactions.get(env_id)
        self._pending_episodes.pop(env_id, None)
        self._pending_publication_cleanup.pop(env_id, None)
        self._companion_paths.pop(env_id, None)
        for key, writer in tuple(self._writers.items()):
            if key[0] != env_id:
                continue
            if writer is not None:
                self._abort_writer(writer, exc)
        self._clear_writer_state(env_id)
        if transaction is None or transaction.committed or staging_already_cleaned:
            self._finalize_failed_env(env_id, exc)
            return

        if not _same_directory_identity(
            transaction.staging_dir.parent,
            transaction.parent_identity,
        ) or not _same_directory_identity(
            transaction.staging_dir,
            transaction.staging_identity,
        ):
            # Do not clean a replacement directory owned by another process.
            self._finalize_failed_env(env_id, exc)
            return

        staging_owner = _EventEpisodeStagingCleanupOwner(transaction)
        _result, cleanup_failure = attempt_cleanup(staging_owner.retry_cleanup)
        if cleanup_failure is None:
            self._finalize_failed_env(env_id, exc)
            return

        owner = _EventPublicationCleanupOwner(
            [staging_owner.retry_cleanup],
            on_complete=lambda: self._finalize_failed_env(env_id, exc),
        )
        cleanup_error = PublicationCleanupError(
            "event episode staging cleanup is pending",
            owner=owner,
            errors=(cleanup_failure, exc),
        )
        self._pending_publication_cleanup[env_id] = cleanup_error
        raise cleanup_error from cleanup_failure

    def _reset_env(
        self,
        env_id: int,
        outcome: dict[str, Any],
    ) -> list[Path]:
        failure = self._env_failures.get(env_id)
        if failure is not None:
            raise failure
        self._require_publication_cleanup_complete(env_id)
        if env_id in self._pending_episodes:
            self._complete_pending_episode(env_id)
        _validate_episode_outcome(outcome)

        episode_id = self._ep_counter[env_id]
        transaction = self._transaction_for(env_id, episode_id)
        rows: list[EventManifestRow] = []
        published_paths: list[Path] = []
        publication_errors: list[PublicationCleanupError] = []
        rollback_publication_errors: list[PublicationCleanupError] = []
        try:
            pending_publications: list[
                tuple[EventHDF5Writer, EventEpisodeProvenance, str, Path]
            ] = []
            for ref in self._sensors:
                key = (env_id, ref.name)
                writer = self._writers[key]
                if writer is None:
                    writer = self._create_writer(ref, env_id, episode_id, outcome)
                    self._writers[key] = writer
                provenance = self._build_provenance(ref, env_id, episode_id, outcome)
                writer.finalize(EventEpisodeMetadata(**asdict(provenance)))
                published_path = self._published_episode_path(transaction, ref.name)
                published_paths.append(published_path)
                pending_publications.append(
                    (writer, provenance, ref.name, published_path)
                )

            # Close HDF5 handles and workers before the shared lease; publish files singly.
            for writer, provenance, sensor_name, published_path in pending_publications:
                try:
                    published_writer_path = transaction.publish(writer)
                except PublicationCleanupError as exc:
                    committed_path = _matching_committed_path(exc, writer.path)
                    if committed_path is None:
                        rollback_publication_errors.append(exc)
                        break
                    publication_errors.append(exc)
                    published_writer_path = committed_path
                rows.append(
                    EventManifestRow.from_provenance(
                        provenance,
                        sensor_name=sensor_name,
                        env_id=env_id,
                        event_hdf5_path=str(
                            published_path.relative_to(self._record_dir)
                        ),
                        event_count=int(writer.row_count),
                        event_sha256=_read_finalized_event_sha256(
                            published_writer_path
                        ),
                    )
                )

            if rollback_publication_errors:
                cleanup_errors = [*publication_errors, *rollback_publication_errors]
                cleanup_error = self._retain_uncommitted_publication_cleanup(
                    env_id,
                    cleanup_errors,
                    message=(
                        "event episode publication failed before directory commit"
                    ),
                )
                raise cleanup_error

            self._dispatch.reset_env(env_id, episode_id=episode_id + 1)

            if publication_errors:
                self._pending_episodes[env_id] = _PendingEpisode(
                    transaction=transaction,
                    rows=rows,
                    published_paths=published_paths,
                )
                self._advance_episode_state(
                    env_id,
                    transaction,
                    retain_transaction=True,
                )
                first_error = publication_errors[0]
                cleanup_error = PublicationCommitCleanupError(
                    "event episode private files committed but cleanup is pending",
                    owner=first_error.owner,
                    committed_paths=_unique_committed_paths(publication_errors),
                    committed_result=first_error.committed_result,
                    errors=tuple(
                        cause
                        for publication_error in publication_errors
                        for cause in publication_error.errors
                    ),
                )
                self._retain_publication_cleanup(
                    env_id,
                    cleanup_error,
                    operations=[
                        error.owner.retry_cleanup for error in publication_errors
                    ],
                    on_complete=lambda: self._complete_pending_episode(env_id),
                )
                raise cleanup_error

            transaction.commit()
            pending = _PendingEpisode(
                transaction=transaction,
                rows=rows,
                published_paths=published_paths,
                manifest_appended=False,
            )
            self._pending_episodes[env_id] = pending
            # Retain state so a failed manifest append can be retried.
            self._advance_episode_state(
                env_id,
                transaction,
                retain_transaction=True,
            )
            try:
                append_event_manifest_rows(self._manifest_path, rows)
            except PublicationCleanupError as exc:
                manifest_committed_path = _matching_committed_path(
                    exc, self._manifest_path
                )
                if manifest_committed_path is not None:
                    cleanup_error = PublicationCommitCleanupError(
                        "event episode committed but manifest cleanup is pending",
                        owner=exc.owner,
                        committed_paths=_unique_paths(
                            [
                                *published_paths,
                                transaction.final_dir,
                                *exc.committed_paths,
                            ]
                        ),
                        committed_result=exc.committed_result,
                        errors=exc.errors,
                    )
                    pending.manifest_appended = True
                    self._pending_episodes.pop(env_id, None)
                    self._advance_episode_state(
                        env_id,
                        transaction,
                        retain_transaction=False,
                    )
                    self._retain_publication_cleanup(
                        env_id,
                        cleanup_error,
                        operations=[exc.owner.retry_cleanup],
                        on_complete=lambda: self._clear_publication_cleanup(env_id),
                    )
                    raise cleanup_error from exc
                if exc.committed_paths:
                    cleanup_error = self._retain_uncommitted_publication_cleanup(
                        env_id,
                        [exc],
                        message=(
                            "event manifest publication completed at an unexpected path"
                        ),
                    )
                    raise cleanup_error from exc
                self._retain_publication_cleanup(
                    env_id,
                    exc,
                    operations=[exc.owner.retry_cleanup],
                    on_complete=lambda: self._complete_pending_episode(env_id),
                    committed_paths=[*published_paths, transaction.final_dir],
                )
                raise
            except BaseException:
                raise
            pending.manifest_appended = True
            self._pending_episodes.pop(env_id, None)
            self._advance_episode_state(
                env_id,
                transaction,
                retain_transaction=False,
            )
        except BaseException as exc:
            if isinstance(exc, PublicationCleanupError):
                raise
            if env_id in self._pending_episodes:
                raise
            if publication_errors:
                cleanup_error = self._retain_uncommitted_publication_cleanup(
                    env_id,
                    publication_errors,
                    message=(
                        "event episode publication failed with file cleanup pending"
                    ),
                    cause=exc,
                )
                raise cleanup_error from exc
            self._fail_env(env_id, exc)
            raise

        return published_paths

    def _commit_companion_episode(
        self,
        env_id: int,
    ) -> tuple[list[Path], BaseException | None]:
        """Commit one visual-only transaction and retain failed cleanup."""
        self._require_publication_cleanup_complete(env_id)
        transaction = self._transactions.get(env_id)
        if transaction is None:
            return [], None
        _result, commit_error = attempt_cleanup(transaction.commit)
        if commit_error is not None:
            _result, cleanup_error = attempt_cleanup(
                lambda: self._fail_env(env_id, commit_error)
            )
            return [], cleanup_error or commit_error

        companions = list(self._companion_paths.get(env_id, ()))
        self._transactions.pop(env_id, None)
        self._companion_paths.pop(env_id, None)
        return companions, None

    def latest_event_observations(self) -> "dict[str, torch.Tensor]":
        """Build canonical observation entries for the latest event tick.

        Returns an entry for each configured sensor, including empty ticks,
        or an empty mapping when no event sensors are configured.
        """
        if not self._sensors:
            return {}
        from types import SimpleNamespace

        from srb.core.sensor.event_camera.observation import (
            build_event_observation,
        )

        out: dict[str, torch.Tensor] = {}
        for ref in self._sensors:
            events_per_env: list[torch.Tensor] = []
            for env_id in range(self._num_envs):
                bufs = self._step_events[(env_id, ref.name)]
                events_per_env.append(_coalesce_event_buffers(bufs))
            shim = SimpleNamespace(
                cfg=ref.cfg,
                num_instances=self._num_envs,
                events_per_env=events_per_env,
            )
            out.update(build_event_observation(ref.name, shim))
        return out

    def reset(
        self,
        env_ids: Sequence[int],
        *,
        outcomes: dict[int, dict[str, Any]] | None = None,
    ) -> list[Path]:
        """Publish selected episodes and return their HDF5 paths.

        ``outcomes`` supplies optional per-episode success, reward, and RGB
        video metadata; missing values use defaults.
        """
        self._ensure_usable()
        # Normalize tensor IDs for the recorder's int-keyed state.
        env_ids = [int(env_id) for env_id in env_ids]

        if len(set(env_ids)) != len(env_ids):
            raise ValueError("env_ids must be unique")

        for env_id in env_ids:
            if env_id < 0 or env_id >= self._num_envs:
                raise ValueError(f"env_id out of range: {env_id}")
        self._close_finalized_envs.difference_update(env_ids)

        if not self._persist_artifacts:
            for env_id in env_ids:
                failure = self._env_failures.get(env_id)
                if failure is not None:
                    raise failure
                episode_id = self._ep_counter[env_id]
                try:
                    if self._sensors:
                        self._dispatch.reset_env(env_id, episode_id=episode_id + 1)
                except BaseException as exc:
                    self._fail_env(env_id, exc)
                    raise
                self._ep_counter[env_id] += 1
                self._clear_writer_state(env_id)
            return []

        if not self._sensors:
            written: list[Path] = []
            for env_id in env_ids:
                failure = self._env_failures.get(env_id)
                if failure is not None:
                    raise failure
                companions, failure = self._commit_companion_episode(env_id)
                if failure is not None:
                    raise failure
                if not companions:
                    continue
                written.extend(companions)
                self._ep_counter[env_id] += 1
            return written

        written: list[Path] = []
        outcomes = outcomes or {}
        for env_id in env_ids:
            _validate_episode_outcome(outcomes.get(env_id, {}))
        first_failure: BaseException | None = None
        pending_cleanup_errors: list[PublicationCleanupError] = []
        for env_id in env_ids:
            try:
                written.extend(self._reset_env(env_id, outcomes.get(env_id, {})))
                # Saving preserves this tick's observations until record_step.
                self._clear_writer_state(env_id)
            except BaseException as exc:
                if isinstance(exc, PublicationCleanupError):
                    pending_cleanup_errors.append(exc)
                if first_failure is None:
                    first_failure = exc
        if pending_cleanup_errors:
            if len(pending_cleanup_errors) == 1 and isinstance(
                first_failure, PublicationCleanupError
            ):
                raise first_failure
            raise self._aggregate_reset_publication_cleanup(
                pending_cleanup_errors,
                terminal_failure=(
                    None
                    if isinstance(first_failure, PublicationCleanupError)
                    else first_failure
                ),
            )
        if first_failure is not None:
            raise first_failure
        return written

    def save_episodes(
        self,
        env_ids: Sequence[int],
        *,
        outcomes: dict[int, dict[str, Any]] | None = None,
    ) -> list[Path]:
        """Publish selected episodes while retaining the latest step events."""
        step_events = {key: list(bufs) for key, bufs in self._step_events.items()}
        step_event_counts = dict(self._step_event_counts)
        try:
            return self.reset(env_ids, outcomes=outcomes)
        finally:
            # A later environment's failure must not drop the terminal
            # observation of environments reset before it.
            for key, bufs in step_events.items():
                self._step_events[key][:] = bufs
            self._step_event_counts.update(step_event_counts)

    def close(
        self,
        *,
        outcomes: Mapping[int, Mapping[str, Any]] | None = None,
    ) -> list[Path]:
        """Publish the current episode for every environment, idempotently."""
        if self._closed:
            if self._close_failure is not None:
                raise self._close_failure
            return list(self._closed_paths)
        self._ensure_usable()
        self.validate_episode_outcomes(outcomes)
        outcomes = outcomes or {}
        if not self._persist_artifacts:
            self._closed_paths = ()
            self._closed = True
            self._close_failure = next(iter(self._env_failures.values()), None)
            if self._close_failure is not None:
                raise self._close_failure
            return []
        written: list[Path] = []
        first_failure = next(iter(self._env_failures.values()), None)
        pending_cleanup_errors: list[PublicationCleanupError] = []
        pending_cleanup_paths: list[Path] = []
        if self._sensors:
            for env_id in range(self._num_envs):
                if env_id in self._env_failures:
                    continue
                if env_id in self._close_finalized_envs:
                    continue
                pending_episode = self._pending_episodes.get(env_id)
                if (
                    pending_episode is not None
                    and env_id not in self._pending_publication_cleanup
                ):
                    # Finish the pending manifest row instead of starting an
                    # empty episode on retry.
                    pending_paths = tuple(pending_episode.published_paths)
                    try:
                        self._complete_pending_episode(env_id)
                    except BaseException as exc:
                        if isinstance(exc, PublicationCleanupError):
                            pending_cleanup_errors.append(exc)
                            pending_cleanup_paths.extend(pending_paths)
                        if first_failure is None:
                            first_failure = exc
                    else:
                        written.extend(pending_paths)
                        self._close_finalized_envs.add(env_id)
                    continue
                try:
                    written.extend(
                        self._reset_env(env_id, dict(outcomes.get(env_id, {})))
                    )
                    self._close_finalized_envs.add(env_id)
                except BaseException as exc:
                    if isinstance(exc, PublicationCleanupError):
                        pending_cleanup_errors.append(exc)
                        pending = self._pending_episodes.get(env_id)
                        if pending is not None:
                            pending_cleanup_paths.extend(pending.published_paths)
                        else:
                            pending_cleanup_paths.extend(
                                path
                                for path in exc.committed_paths
                                if path.suffix.lower() in {".h5", ".hdf5"}
                                and path.parent.name.startswith("events_env")
                            )
                    if first_failure is None:
                        first_failure = exc
        else:
            # Commit companion-only transactions without advancing episodes.
            for env_id in range(self._num_envs):
                if env_id in self._env_failures:
                    continue
                if env_id in self._close_finalized_envs:
                    continue
                result, failure = attempt_cleanup(
                    lambda: self._commit_companion_episode(env_id)
                )
                if failure is None and result is not None:
                    env_paths, failure = result
                    written.extend(env_paths)
                    if failure is None:
                        self._close_finalized_envs.add(env_id)
                if failure is not None:
                    if isinstance(failure, PublicationCleanupError):
                        pending_cleanup_errors.append(failure)
                    if first_failure is None:
                        first_failure = failure
        recoverable_manifest_pending = any(
            env_id not in self._pending_publication_cleanup
            for env_id in self._pending_episodes
        )
        if first_failure is not None and recoverable_manifest_pending:
            # Keep close retryable while a committed manifest row is pending.
            self._close_failure = first_failure
            raise first_failure
        self._closed_paths = tuple(written)
        self._closed = True
        if pending_cleanup_errors:
            cleanup_error = self._aggregate_close_publication_cleanup(
                pending_cleanup_errors,
                written=written,
                pending_paths=pending_cleanup_paths,
                terminal_failure=(
                    None
                    if isinstance(first_failure, PublicationCleanupError)
                    else first_failure
                ),
            )
            self._close_failure = cleanup_error
            raise cleanup_error
        self._close_failure = first_failure
        if first_failure is not None:
            raise first_failure
        return written

    def hot_pixel_mask(
        self, env_id: int, sensor_name: str
    ) -> tuple[tuple[int, int], ...]:
        """Return the persistent ``(y, x)`` mask in effective-resolution space.

        The mask depends on ``env_id`` only and remains fixed across episodes.
        """
        return self._hot_pixel_masks[(env_id, sensor_name)]

    def _get_backend(self, env_id: int, sensor_name: str) -> EventGenerator:
        """Return the backend for ``(env_id, sensor_name)``."""
        return self._dispatch.get_backend(env_id, sensor_name)

    def _build_provenance(
        self,
        ref: EventSensorRef,
        env_id: int,
        ep: int,
        outcome: dict[str, Any] | None = None,
    ) -> EventEpisodeProvenance:
        """Build the shared provenance used by HDF5 metadata and manifest rows."""
        cfg = ref.cfg
        backend = self._get_backend(env_id, ref.name)
        outcome = outcome or {}
        prov = self._provenance
        return EventEpisodeProvenance(
            sensor_preset=cfg.preset_name,
            resolution=cfg.effective_resolution,
            # Fall back to configured rate until two frames establish a measure.
            sub_render_hz=self._effective_env_sub_render_hz(
                env_id, ref.name, cfg.sub_render_hz
            ),
            backend_name=backend.name,
            backend_version=backend.version,
            backend_commit_sha=backend.commit_sha or "",
            representation=cfg.representation,
            time_bins=cfg.time_bins,
            episode_id=ep,
            policy_id=prov.get("policy_id", ""),
            policy_checkpoint_hash=prov.get("policy_checkpoint_hash", ""),
            task_id=prov.get("task_id", ""),
            # Record the seed actually used by this episode's backend.
            seed=self._dispatch.episode_seed(env_id, ref.name),
            srb_git_sha=prov.get("srb_git_sha", ""),
            isaaclab_version=prov.get("isaaclab_version", ""),
            rgb_mp4_path=_paired_rgb_mp4_path(ref.name, outcome),
            success_flag=int(outcome.get("success_flag", 0)),
            reward_summary=(
                ""
                if outcome.get("reward_summary") is None
                else str(outcome.get("reward_summary", ""))
            ),
            # Include hot-pixel metadata only when dispatch injected events.
            hot_pixel_mask=(
                self._hot_pixel_masks.get((env_id, ref.name), ())
                if self._dispatch.injects_hot_pixels(env_id, ref.name)
                else ()
            ),
            # Report zero when dispatch injected no hot-pixel events.
            hot_pixel_rate_hz=(
                float(cfg.hot_pixel_rate_hz)
                if self._dispatch.injects_hot_pixels(env_id, ref.name)
                else 0.0
            ),
        )

    def _effective_env_sub_render_hz(
        self, env_id: int, sensor_name: str, declared_hz: float
    ) -> float:
        key = (env_id, sensor_name)
        count = self._env_frame_counts.get(key, 0)
        if count < 2:
            return float(declared_hz)
        span_ns = self._env_last_frame_ns[key] - self._env_first_frame_ns[key]
        if span_ns <= 0:
            return float(declared_hz)
        return (count - 1) * 1e9 / span_ns

    def validate_episode_outcomes(
        self,
        outcomes: Mapping[int, Mapping[str, Any]] | None,
    ) -> None:
        """Validate episode outcome metadata before companion files are staged."""

        for outcome in (outcomes or {}).values():
            _validate_episode_outcome(outcome)


__all__ = [
    "EventObservationOverflowError",
    "EventManifestRow",
    "EventRecorder",
    "EventSensorRef",
    "iter_event_camera_sensors",
]


@dataclass
class _EpisodeTransaction:
    env_id: int
    episode_id: int
    staging_dir: Path
    final_dir: Path
    staging_identity: tuple[int, int]
    parent_identity: tuple[int, int]
    committed: bool = False
    accounted: bool = False

    def publish(self, writer: EventHDF5Writer) -> Path:
        """Publish one closed writer into this transaction's private tree."""
        if writer.path.parent != self.staging_dir:
            raise RuntimeError(
                "event writer destination is outside its episode transaction"
            )
        return writer.publish()

    def commit(self) -> None:
        """Atomically make the complete episode tree visible."""
        if self.committed:
            return
        if not _same_directory_identity(self.staging_dir.parent, self.parent_identity):
            raise RuntimeError(
                f"event episode staging parent was replaced: {self.staging_dir.parent}"
            )
        if not _same_directory_identity(self.staging_dir, self.staging_identity):
            raise RuntimeError(
                f"event episode staging directory was replaced: {self.staging_dir}"
            )
        if self.final_dir.exists() or self.final_dir.is_symlink():
            raise FileExistsError(
                f"event episode destination already exists: {self.final_dir}"
            )
        self.staging_dir.replace(self.final_dir)
        self.committed = True
