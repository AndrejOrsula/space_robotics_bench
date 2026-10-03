"""Dispatch event-camera frames and manage backend lifecycle and seeding.

This keeps event-tick scheduling and batched/per-env dispatch out of
:class:`~srb.core.sensor.event_camera.recorder.EventRecorder`.
"""

from __future__ import annotations

import sys
from collections.abc import Callable, Mapping, Sequence
from typing import TYPE_CHECKING, Any, Protocol

from .backend import (
    EventGenerator,
    EventStreamingUnsupportedError,
    EventStreamNoProgressError,
    get_backend_factory,
    make_backend,
    validate_event_chunk,
    validate_streaming_backend_class,
)
from .config import MAX_HOT_PIXEL_EVENTS_PER_TICK, EventCameraCfg
from .seeding import derive_event_camera_env_seed, distinct_sensor_base_seeds

if TYPE_CHECKING:
    import torch


class _EventSensorRef(Protocol):
    name: str
    sensor: Any

    @property
    def cfg(self) -> EventCameraCfg: ...


# Fail closed on clock jumps that require more than this many catch-up ticks.
MAX_CATCH_UP_SUB_TICKS = 1_000_000


class EventDispatchTimeGapError(RuntimeError):
    """The simulation clock advanced further than the catch-up ceiling allows."""


class EventObservationOverflowError(RuntimeError):
    """An in-memory compatibility or observation window exceeded its bound."""

    code = "event_camera.observation_window_overflow"

    def __init__(
        self,
        *,
        key: tuple[int, str],
        accepted_rows: int,
        incoming_rows: int,
        max_rows: int,
    ) -> None:
        self.key = key
        self.env_id, self.sensor_name = key
        self.accepted_rows = accepted_rows
        self.incoming_rows = incoming_rows
        self.max_rows = max_rows
        super().__init__(
            f"observation window for {key!r} would reach "
            f"{accepted_rows + incoming_rows} rows, exceeding "
            f"observation_window_events {max_rows}"
        )


# Keep hot-pixel sampling on a separate deterministic stream from the backend.
_HOT_PIXEL_STREAM_ID = 0x484F54  # "HOT"


def derive_event_camera_episode_seed(
    base_seed: int, env_id: int, episode_id: int
) -> int:
    """Per-reset backend seed from ``(base_seed, env_id, episode_id)``."""

    return derive_event_camera_env_seed(
        derive_event_camera_env_seed(int(base_seed), int(env_id)), int(episode_id)
    )


class _HotPixelInjector:
    """Sample ON events with capped counts and sorted timestamps in ``(t-dt, t]``.

    Counts cap per started ``period_ns`` and by ``dt_ns``. The CPU generator
    resets per episode.
    """

    def __init__(self, coords: Sequence[tuple[int, int]], rate_hz: float) -> None:
        import torch

        # Baked coordinates are (y, x); event rows are (x, y).
        self._xy = torch.tensor([(x, y) for y, x in coords], dtype=torch.int64).reshape(
            -1, 2
        )
        self._rate_hz = float(rate_hz)
        self._generator = torch.Generator(device="cpu")

    def reset(self, episode_seed: int) -> None:
        self._generator.manual_seed(
            derive_event_camera_env_seed(int(episode_seed), _HOT_PIXEL_STREAM_ID)
        )

    def sample(self, t_ns: int, dt_ns: int, period_ns: int) -> "torch.Tensor":
        import torch

        dt_ns = int(dt_ns)
        periods = max(1, -(-dt_ns // max(1, int(period_ns))))
        n_hot = int(self._xy.shape[0])
        mean = torch.full((n_hot,), self._rate_hz * dt_ns * 1e-9, dtype=torch.float64)
        counts = torch.poisson(mean, generator=self._generator).to(torch.int64)
        counts.clamp_(max=min(MAX_HOT_PIXEL_EVENTS_PER_TICK * periods, dt_ns))
        total = int(counts.sum())
        events = torch.empty((total, 4), dtype=torch.int64)
        if total == 0:
            return events
        pixel = torch.repeat_interleave(torch.arange(n_hot), counts)
        first = torch.cumsum(counts, 0) - counts
        j = torch.arange(total) - first[pixel]
        k = counts[pixel]
        # Slice j of k starts at floor(j * dt / k) and is floor(dt / k) >= 1
        # ns wide, so the k offsets are distinct and increasing.
        width = dt_ns // k
        jitter = torch.floor(
            torch.rand(total, generator=self._generator, dtype=torch.float64)
            * width.to(torch.float64)
        ).to(torch.int64)
        offset = (j * dt_ns) // k + jitter.clamp_(max=width - 1)
        events[:, 0] = int(t_ns) - dt_ns + 1 + offset
        events[:, 1:3] = self._xy[pixel]
        events[:, 3] = 1
        order = torch.argsort(events[:, 0], stable=True)
        return events[order]


def _merge_time_ordered(events: "torch.Tensor", extra: "torch.Tensor") -> Any:
    """Insert time-sorted ``extra`` rows into time-sorted ``events`` rows.

    Equal-time extra rows follow existing rows; inputs remain unchanged.
    """
    import torch

    extra = extra.to(events.device)
    n, m = int(events.shape[0]), int(extra.shape[0])
    position = torch.searchsorted(
        events[:, 0].contiguous(), extra[:, 0].contiguous(), right=True
    ) + torch.arange(m, device=events.device)
    keep = torch.ones(n + m, dtype=torch.bool, device=events.device)
    keep[position] = False
    merged = torch.empty((n + m, 4), dtype=events.dtype, device=events.device)
    merged[position] = extra.to(events.dtype)
    merged[keep] = events
    return merged


class EventRGBUnavailableError(RuntimeError):
    """A concrete event backend was invoked without a native RGB frame."""

    code = "event_camera.rgb_unavailable"

    def __init__(self, sensor_name: str) -> None:
        self.sensor_name = sensor_name
        super().__init__(
            f"event-camera sensor {sensor_name!r} has no native RGB frame; "
            "the selected backend requires rendered RGB input"
        )


class EventBatchCardinalityError(ValueError):
    """A batched backend returned a different number of env entries."""

    code = "event_camera.batch_cardinality"

    def __init__(self, *, expected: int, actual: int | None) -> None:
        self.expected = expected
        self.actual = actual
        actual_text = "an unsized value" if actual is None else str(actual)
        super().__init__(
            "event-camera batch chunk must contain exactly "
            f"{expected} env entries; got {actual_text}"
        )


def _validate_batch_cardinality(chunks: Any, *, expected: int) -> None:
    try:
        actual = len(chunks)
    except (AttributeError, OverflowError, TypeError, ValueError) as exc:
        raise EventBatchCardinalityError(expected=expected, actual=None) from exc
    if actual != expected:
        raise EventBatchCardinalityError(expected=expected, actual=actual)


def _close_event_stream(
    stream: Any,
    *,
    on_error: Callable[[BaseException], None],
) -> None:
    """Close a provider iterator and route cleanup failures to dispatch."""

    close = getattr(stream, "close", None)
    if not callable(close):
        return
    try:
        close()
    except BaseException as exc:
        on_error(exc)


class EventBackendDispatch:
    """Own backend instances and drive them once per new rendered frame.

    The per-sensor clock persists across resets; each rendered frame is
    consumed once, at no more than ``sub_render_hz``. Batched backends must
    implement ``reset_env`` for per-env reseeding.
    """

    def __init__(
        self,
        sensors: Sequence[_EventSensorRef],
        num_envs: int,
        *,
        run_seed: int | None = None,
        hot_pixel_masks: Mapping[tuple[int, str], Sequence[tuple[int, int]]]
        | None = None,
    ) -> None:
        """Read ``torch.initial_seed()`` without drawing when ``run_seed`` is None.

        ``hot_pixel_masks`` enables injection for RGB backends with a positive rate.
        """
        if run_seed is None:
            import torch

            run_seed = int(torch.initial_seed())
        self._run_seed = int(run_seed)
        self._num_envs = int(num_envs)
        if self._num_envs < 0:
            raise ValueError(f"num_envs must be >= 0; got {num_envs}")

        self._sensors = list(sensors)
        seen: set[str] = set()
        dupes: set[str] = set()
        for ref in self._sensors:
            if ref.name in seen:
                dupes.add(ref.name)
            seen.add(ref.name)
        if dupes:
            raise ValueError(f"Duplicate event sensor names: {sorted(dupes)!r}")

        # Non-batched backends get one instance per env; batched backends share
        # one instance per sensor.
        self._per_env: dict[tuple[int, str], EventGenerator] = {}
        self._shared: dict[str, EventGenerator] = {}
        self._episode_seeds: dict[tuple[int, str], int] = {}
        self._hot_pixels: dict[tuple[int, str], _HotPixelInjector] = {}

        # Per-sensor clock persists across episode resets.
        self._dt_sub_ns: dict[str, int] = {}
        self._t_sub_last_ns: dict[str, int] = {}
        self._declared_hz: dict[str, float] = {}
        self._frame_tokens: dict[str, Any] = {}
        # An env whose camera did not refresh keeps its previous frame time.
        self._t_env_last_ns: dict[tuple[int, str], int] = {}
        # Rebase hot-pixel sampling at the first consumed frame after reset.
        self._hot_rebase: set[tuple[int, str]] = set()
        self._first_tick_ns: dict[str, int] = {}
        self._tick_count: dict[str, int] = {}

        self._base_seeds = distinct_sensor_base_seeds(
            (ref.name, ref.cfg.resolve_base_seed(self._run_seed))
            for ref in self._sensors
        )
        if not self._sensors:
            return

        for ref in self._sensors:
            cfg = ref.cfg
            base_seed = self._base_seeds[ref.name]

            hz = float(cfg.sub_render_hz)
            if hz <= 0.0:
                raise ValueError(
                    f"sub_render_hz must be > 0 for sensor {ref.name!r}; got {cfg.sub_render_hz}"
                )
            self._dt_sub_ns[ref.name] = max(1, int(round(1e9 / hz)))
            self._declared_hz[ref.name] = hz
            self._t_sub_last_ns[ref.name] = 0

            factory = get_backend_factory(cfg.backend)
            if cfg.strict_streaming and not factory.supports_streaming:
                raise EventStreamingUnsupportedError(cfg.backend, "strict_streaming")
            validate_streaming_backend_class(factory, backend_name=cfg.backend)
            if factory.supports_batching:
                shared = make_backend(cfg)
                if type(shared).reset_env is EventGenerator.reset_env:
                    raise ValueError(
                        "Batched event-camera backends must override reset_env() "
                        f"for per-env reseeding; {type(shared).__name__} does not."
                    )
                shared.reset(seed=base_seed, episode_id=0)
                self._shared[ref.name] = shared
                for env_id in range(self._num_envs):
                    seed = derive_event_camera_episode_seed(base_seed, env_id, 0)
                    shared.reset_env(env_id, seed=seed, episode_id=0)
                    self._episode_seeds[(env_id, ref.name)] = seed
            else:
                for env_id in range(self._num_envs):
                    seed = derive_event_camera_episode_seed(base_seed, env_id, 0)
                    backend = make_backend(cfg)
                    backend.reset(seed=seed, episode_id=0)
                    self._per_env[(env_id, ref.name)] = backend
                    self._episode_seeds[(env_id, ref.name)] = seed
            if (
                hot_pixel_masks is not None
                and factory.requires_rgb
                and cfg.hot_pixel_rate_hz > 0
            ):
                for env_id in range(self._num_envs):
                    key = (env_id, ref.name)
                    coords = hot_pixel_masks.get(key, ())
                    if coords:
                        injector = _HotPixelInjector(coords, cfg.hot_pixel_rate_hz)
                        injector.reset(self._episode_seeds[key])
                        self._hot_pixels[key] = injector

    @property
    def has_sensors(self) -> bool:
        return bool(self._sensors)

    def get_backend(self, env_id: int, sensor_name: str) -> EventGenerator:
        env_id = int(env_id)
        if env_id < 0 or env_id >= self._num_envs:
            raise ValueError(f"env_id must be in [0, {self._num_envs}); got {env_id}")
        shared = self._shared.get(sensor_name)
        if shared is not None:
            return shared
        return self._per_env[(env_id, sensor_name)]

    def episode_seed(self, env_id: int, sensor_name: str) -> int:
        """Effective backend seed of the current episode for one env/sensor."""
        return self._episode_seeds[(int(env_id), sensor_name)]

    def last_frame_ns(self, env_id: int, sensor_name: str) -> int | None:
        """Timestamp of the latest frame consumed for one env/sensor, if any."""
        return self._t_env_last_ns.get((int(env_id), sensor_name))

    def effective_sub_render_hz(self, sensor_name: str) -> float | None:
        """Measured run-cumulative tick rate, or ``None`` before two frames.

        Counts frames consumed by any env; resets do not restart it.
        """
        if sensor_name not in self._declared_hz:
            raise KeyError(sensor_name)
        count = self._tick_count.get(sensor_name, 0)
        if count < 2:
            return None
        span_ns = self._t_sub_last_ns[sensor_name] - self._first_tick_ns[sensor_name]
        if span_ns <= 0:
            return None
        return (count - 1) * 1e9 / span_ns

    def injects_hot_pixels(self, env_id: int, sensor_name: str) -> bool:
        """Whether hot-pixel events are emitted for one env/sensor stream."""
        return (int(env_id), sensor_name) in self._hot_pixels

    def reset_env(self, env_id: int, *, episode_id: int) -> None:
        """Episode-boundary reseed for one env across all sensors."""

        if not self._sensors:
            return

        env_id = int(env_id)
        if env_id < 0 or env_id >= self._num_envs:
            raise ValueError(f"env_id must be in [0, {self._num_envs}); got {env_id}")
        for ref in self._sensors:
            base_seed = self._base_seeds[ref.name]
            env_seed = derive_event_camera_episode_seed(
                base_seed, env_id, int(episode_id)
            )
            shared = self._shared.get(ref.name)
            if shared is not None:
                shared.reset_env(env_id, seed=env_seed, episode_id=int(episode_id))
            else:
                self._per_env[(env_id, ref.name)].reset(
                    seed=env_seed, episode_id=int(episode_id)
                )
            key = (env_id, ref.name)
            self._episode_seeds[key] = env_seed
            injector = self._hot_pixels.get(key)
            if injector is not None:
                injector.reset(env_seed)
                self._hot_rebase.add(key)

    def prime_envs(
        self,
        env_ids: Sequence[int],
        *,
        sim_time_ns: int,
        error_sink: Callable[[tuple[int, str], BaseException], None] | None = None,
    ) -> None:
        """Feed reset envs their post-reset frame as the new episode baseline.

        A selective reset happens at the terminal tick's sim time, so
        :meth:`record_step` would skip this frame. Backends were reset by
        :meth:`reset_env`, so the frame only sets their baseline; any rows are
        discarded so the reset teleport emits no events. The sensor clock and
        tick count are untouched; only the primed envs' frame time moves.
        """

        if not self._sensors:
            return
        env_ids = sorted({int(env_id) for env_id in env_ids})
        for env_id in env_ids:
            if env_id < 0 or env_id >= self._num_envs:
                raise ValueError(
                    f"env_id must be in [0, {self._num_envs}); got {env_id}"
                )
        if not env_ids:
            return
        t_ns = int(sim_time_ns)

        def fail(key: tuple[int, str], exc: Exception) -> None:
            if error_sink is None:
                raise exc
            error_sink(key, exc)

        for ref in self._sensors:
            rgb_batch = _pull_rgb_batch(ref.sensor)
            frame_token = _frame_token(ref.sensor)
            shared = self._shared.get(ref.name)
            primed: list[int] = []
            try:
                if rgb_batch is not None:
                    rgb_batch = _preprocess_rgb_batch(rgb_batch, ref.cfg)
                # Without RGB, the next record_step reports the missing frame.
                if shared is not None and (
                    rgb_batch is not None or not shared.requires_rgb
                ):
                    # None keeps non-reset envs' backend state untouched.
                    frames = [
                        rgb_batch[env_id]
                        if rgb_batch is not None and env_id in env_ids
                        else None
                        for env_id in range(self._num_envs)
                    ]
                    if shared.supports_streaming:
                        for _chunks in shared.iter_batch(frames, t_ns):
                            pass
                    else:
                        shared.process_batch(frames, t_ns)
                    primed = env_ids
            # Backend errors fail only the affected streams; interrupts propagate.
            except Exception as exc:  # noqa: BLE001
                for env_id in env_ids:
                    fail((env_id, ref.name), exc)
                continue
            if shared is None:
                for env_id in env_ids:
                    backend = self._per_env[(env_id, ref.name)]
                    rgb = (
                        rgb_batch[env_id]
                        if rgb_batch is not None and env_id < len(rgb_batch)
                        else None
                    )
                    if rgb is None and backend.requires_rgb:
                        continue
                    try:
                        if backend.supports_streaming:
                            for _chunk in backend.iter_frame(rgb, t_ns):
                                pass
                        else:
                            backend.process_frame(rgb, t_ns)
                    except Exception as exc:  # noqa: BLE001
                        fail((env_id, ref.name), exc)
                        continue
                    primed.append(env_id)

            previous = self._frame_tokens.get(ref.name)
            if (
                primed
                and frame_token is not None
                and int(frame_token.numel()) == self._num_envs
            ):
                # The next tick must see only frames rendered after this one.
                if previous is None or previous.shape != frame_token.shape:
                    # First prime: -1 (no frame counter) keeps unprimed envs fresh.
                    previous = frame_token.new_full(frame_token.shape, -1)
                else:
                    previous = previous.clone()
                previous[primed] = frame_token[primed]
                self._frame_tokens[ref.name] = previous
            for env_id in primed:
                key = (env_id, ref.name)
                self._t_env_last_ns[key] = t_ns
                # Hot pixels resume from the baseline, not one tick later.
                self._hot_rebase.discard(key)

    def record_step(
        self,
        *,
        sim_time_ns: int | None = None,
        sink: Callable[[tuple[int, str], Any], None] | None = None,
        error_sink: Callable[[tuple[int, str], BaseException], None] | None = None,
    ) -> "dict[tuple[int, str], list[torch.Tensor]]":
        """Consume each due camera frame once and dispatch its validated chunks.

        ``sink`` receives chunks synchronously; otherwise rows are retained up
        to ``observation_window_events``, then overflow raises an error.
        """

        if not self._sensors:
            return {}

        out: dict[tuple[int, str], list[torch.Tensor]] = {}
        out_counts: dict[tuple[int, str], int] = {}
        failed_keys: set[tuple[int, str]] = set()
        pending_interrupt: BaseException | None = None

        def fail(key: tuple[int, str], exc: BaseException) -> None:
            nonlocal pending_interrupt
            if not isinstance(exc, Exception):
                # Process interrupts must escape the per-env error sink.
                pending_interrupt = exc
                raise exc
            if pending_interrupt is not None:
                # Preserve the interrupt if stream cleanup also fails.
                pending_interrupt.add_note(
                    f"event stream {key} cleanup also failed: {exc!r}"
                )
                return
            if key in failed_keys:
                return
            failed_keys.add(key)
            if error_sink is None:
                raise exc
            error_sink(key, exc)

        def fail_envs(
            ref: _EventSensorRef,
            env_ids: Sequence[int],
            exc: BaseException,
        ) -> None:
            for env_id in env_ids:
                fail((env_id, ref.name), exc)

        def emit(ref: _EventSensorRef, env_id: int, events: Any) -> int | None:
            key = (env_id, ref.name)
            if key in failed_keys:
                return None
            try:
                rows = validate_event_chunk(
                    events,
                    max_rows=ref.cfg.event_chunk_rows,
                    key=key,
                )
                if rows == 0:
                    return 0
                if sink is not None:
                    sink(key, events)
                else:
                    accepted = out_counts.get(key, 0)
                    limit = ref.cfg.resolved_observation_window_events()
                    if accepted + rows > limit:
                        raise EventObservationOverflowError(
                            key=key,
                            accepted_rows=accepted,
                            incoming_rows=rows,
                            max_rows=limit,
                        )
                    out.setdefault(key, []).append(events)
                    out_counts[key] = accepted + rows
                return rows
            except BaseException as exc:
                fail(key, exc)
                return None

        pending_hot: dict[tuple[int, str], Any] = {}

        def deliver(
            ref: _EventSensorRef,
            env_id: int,
            events: Any,
            *,
            one_shot: bool,
        ) -> int | None:
            """Merge hot-pixel rows and emit chunks within configured bounds.

            Streaming chunks are validated; oversized one-shot output is sliced.
            """
            key = (env_id, ref.name)
            if key in failed_keys:
                return None
            try:
                rows = validate_event_chunk(
                    events,
                    max_rows=sys.maxsize if one_shot else ref.cfg.event_chunk_rows,
                    key=key,
                )
                hot = pending_hot.get(key)
                if rows and hot is not None and int(hot.shape[0]):
                    import torch

                    events = torch.as_tensor(events)
                    due = int(
                        torch.searchsorted(
                            hot[:, 0].contiguous(),
                            events[-1:, 0].to(device=hot.device, dtype=hot.dtype),
                            right=True,
                        )[0]
                    )
                    if due:
                        events = _merge_time_ordered(events, hot[:due])
                        pending_hot[key] = hot[due:]
                        rows = int(events.shape[0])
            except (IndexError, RuntimeError, TypeError, ValueError) as exc:
                fail(key, exc)
                return None
            if rows == 0:
                return emit(ref, env_id, events)
            step = ref.cfg.event_chunk_rows
            delivered = 0
            for start in range(0, rows, step):
                accepted = emit(ref, env_id, events[start : start + step])
                if accepted is None:
                    return None
                delivered += accepted
            return delivered

        for ref in self._sensors:
            dt_sub = self._dt_sub_ns[ref.name]
            t_last = self._t_sub_last_ns[ref.name]

            # Each consumed frame covers the elapsed interval; sub_render_hz
            # caps dispatch frequency.
            if sim_time_ns is None:
                t_frame = t_last + dt_sub
            else:
                t_now = int(sim_time_ns)
                # Clamp clock regressions; slack covers float-to-ns truncation.
                if t_now <= t_last or t_now - t_last < dt_sub - _clock_slack_ns(dt_sub):
                    continue
                gap_ticks = (t_now - t_last) // dt_sub
                if gap_ticks > MAX_CATCH_UP_SUB_TICKS:
                    error = EventDispatchTimeGapError(
                        f"event sensor {ref.name!r} clock advanced {t_now - t_last} "
                        f"ns ({int(gap_ticks)} sub-render periods) in one step; "
                        f"the ceiling is {MAX_CATCH_UP_SUB_TICKS}. A simulation "
                        "clock jump this large is fail-closed rather than "
                        "stamped as one frame interval."
                    )
                    if error_sink is None:
                        raise error
                    fail_envs(ref, range(self._num_envs), error)
                    continue
                t_frame = t_now

            rgb_batch = _pull_rgb_batch(ref.sensor)
            # RGB access lazily refreshes Isaac Lab's camera frame counter.
            frame_token = _frame_token(ref.sensor)
            refreshed = _refreshed_envs(
                self._frame_tokens.get(ref.name), frame_token, self._num_envs
            )
            if not any(refreshed):
                # Avoid duplicate frames; the next frame covers the full interval.
                continue
            if rgb_batch is not None:
                try:
                    rgb_batch = _preprocess_rgb_batch(rgb_batch, ref.cfg)
                except BaseException as exc:
                    if error_sink is None:
                        raise
                    fail_envs(ref, range(self._num_envs), exc)
                    continue
            shared = self._shared.get(ref.name)
            if shared is not None and rgb_batch is None:
                if shared.requires_rgb:
                    fail_envs(
                        ref,
                        [
                            env_id
                            for env_id, is_refreshed in enumerate(refreshed)
                            if is_refreshed
                        ],
                        EventRGBUnavailableError(ref.name),
                    )
                    continue
                rgb_batch = [None] * self._num_envs

            # Dispatch once per newly rendered frame.
            for sub_t_ns in (t_frame,):
                pending_hot.clear()
                # Preserve batch positions with None to avoid stale RGB and keep
                # per-env backend state aligned.
                if shared is not None and not all(refreshed) and rgb_batch is not None:
                    rgb_batch = [
                        rgb_batch[env_id] if refreshed[env_id] else None
                        for env_id in range(self._num_envs)
                    ]
                fed = refreshed
                for env_id in range(self._num_envs):
                    key = (env_id, ref.name)
                    injector = self._hot_pixels.get(key)
                    if injector is None or key in failed_keys or not fed[env_id]:
                        continue
                    if key in self._hot_rebase:
                        self._hot_rebase.discard(key)
                        continue
                    pending_hot[key] = injector.sample(
                        sub_t_ns,
                        sub_t_ns - self._t_env_last_ns.get(key, 0),
                        dt_sub,
                    )
                if shared is not None:
                    active_env_ids = [
                        env_id
                        for env_id in range(self._num_envs)
                        if refreshed[env_id] and (env_id, ref.name) not in failed_keys
                    ]
                    if not active_env_ids:
                        break
                    if shared.supports_streaming:
                        chunk_stream = None
                        try:
                            chunk_stream = iter(shared.iter_batch(rgb_batch, sub_t_ns))
                            while True:
                                active_env_ids = [
                                    env_id
                                    for env_id in active_env_ids
                                    if (env_id, ref.name) not in failed_keys
                                ]
                                if not active_env_ids:
                                    break
                                try:
                                    chunks_per_env = next(chunk_stream)
                                except StopIteration:
                                    break
                                try:
                                    _validate_batch_cardinality(
                                        chunks_per_env,
                                        expected=self._num_envs,
                                    )
                                except BaseException as exc:
                                    fail_envs(ref, active_env_ids, exc)
                                    break
                                batch_failed = False
                                batch_made_progress = False
                                for env_id in active_env_ids:
                                    try:
                                        events = chunks_per_env[env_id]
                                    except BaseException as exc:
                                        fail_envs(ref, active_env_ids, exc)
                                        batch_failed = True
                                        break
                                    rows = deliver(ref, env_id, events, one_shot=False)
                                    batch_made_progress = batch_made_progress or bool(
                                        rows
                                    )
                                if batch_failed:
                                    break
                                if not batch_made_progress:
                                    fail_envs(
                                        ref,
                                        active_env_ids,
                                        EventStreamNoProgressError(
                                            shared.name,
                                            "iter_batch",
                                        ),
                                    )
                                    break
                        except BaseException as exc:
                            if error_sink is None and any(
                                (env_id, ref.name) in failed_keys
                                for env_id in active_env_ids
                            ):
                                raise
                            fail_envs(ref, active_env_ids, exc)
                        finally:
                            if chunk_stream is not None:
                                _close_event_stream(
                                    chunk_stream,
                                    on_error=lambda exc: fail_envs(
                                        ref, active_env_ids, exc
                                    ),
                                )
                    else:
                        try:
                            events_per_env = shared.process_batch(rgb_batch, sub_t_ns)
                            _validate_batch_cardinality(
                                events_per_env,
                                expected=self._num_envs,
                            )
                        except BaseException as exc:
                            fail_envs(ref, active_env_ids, exc)
                            continue
                        for env_id in active_env_ids:
                            try:
                                events = events_per_env[env_id]
                            except BaseException as exc:
                                fail_envs(ref, active_env_ids, exc)
                                break
                            deliver(ref, env_id, events, one_shot=True)
                else:
                    for env_id in range(self._num_envs):
                        key = (env_id, ref.name)
                        if key in failed_keys or not fed[env_id]:
                            continue
                        rgb = (
                            rgb_batch[env_id]
                            if rgb_batch is not None and env_id < len(rgb_batch)
                            else None
                        )
                        backend = self._per_env[(env_id, ref.name)]
                        if rgb is None and backend.requires_rgb:
                            fail(key, EventRGBUnavailableError(ref.name))
                            continue
                        if backend.supports_streaming:
                            chunk_stream = None
                            try:
                                chunk_stream = iter(backend.iter_frame(rgb, sub_t_ns))
                                while key not in failed_keys:
                                    try:
                                        events = next(chunk_stream)
                                    except StopIteration:
                                        break
                                    rows = deliver(ref, env_id, events, one_shot=False)
                                    if rows == 0:
                                        fail(
                                            key,
                                            EventStreamNoProgressError(
                                                backend.name,
                                                "iter_frame",
                                            ),
                                        )
                                        break
                            except BaseException as exc:
                                if error_sink is None and key in failed_keys:
                                    raise
                                fail(key, exc)
                            finally:
                                if chunk_stream is not None:
                                    _close_event_stream(
                                        chunk_stream,
                                        on_error=lambda exc: fail(key, exc),
                                    )
                        else:
                            try:
                                events = backend.process_frame(rgb, sub_t_ns)
                            except BaseException as exc:
                                fail(key, exc)
                                continue
                            deliver(ref, env_id, events, one_shot=True)
                # Hot-pixel rows later than the backend's last row this tick.
                for key in list(pending_hot):
                    hot = pending_hot.pop(key)
                    if int(hot.shape[0]):
                        deliver(ref, key[0], hot, one_shot=True)

            self._t_sub_last_ns[ref.name] = t_frame
            self._frame_tokens[ref.name] = frame_token
            for env_id in range(self._num_envs):
                if fed[env_id]:
                    self._t_env_last_ns[(env_id, ref.name)] = t_frame
            self._first_tick_ns.setdefault(ref.name, t_frame)
            self._tick_count[ref.name] = self._tick_count.get(ref.name, 0) + 1

        return {} if sink is not None else out


def _frame_token(sensor: object) -> "torch.Tensor | None":
    """CPU camera frame counter, or ``None`` when unavailable."""
    frame = getattr(sensor, "frame", None)
    if frame is None:
        return None
    frame = getattr(frame, "torch", frame)
    import torch

    try:
        return torch.as_tensor(frame).detach().to("cpu", copy=True).reshape(-1)
    except (RuntimeError, TypeError, ValueError):
        return None


def _clock_slack_ns(dt_sub_ns: int) -> int:
    """Tolerance for Isaac Lab's 1 us update-period comparison."""
    return min(1_000, int(dt_sub_ns) // 1_000)


def _refreshed_envs(
    previous: "torch.Tensor | None",
    current: "torch.Tensor | None",
    num_envs: int,
) -> list[bool]:
    """Per-env refresh flags; absent or first counters mark all envs fresh."""
    if current is None or previous is None or previous.shape != current.shape:
        return [True] * num_envs
    changed = previous != current
    if int(changed.numel()) != num_envs:
        return [bool(changed.any())] * num_envs
    return [bool(flag) for flag in changed.tolist()]


def _pull_rgb_batch(sensor: object) -> "torch.Tensor | None":
    """Best-effort RGB batch fetch from an Isaac Lab sensor."""

    data = getattr(sensor, "data", None)
    if data is None:
        return None
    output = getattr(data, "output", None)
    if output is None:
        return None
    if not hasattr(output, "__getitem__"):
        return None
    try:
        rgb = output["rgb"]
    except (KeyError, TypeError):
        return None
    torch_view = getattr(rgb, "torch", None)
    return torch_view if torch_view is not None else rgb


def _preprocess_rgb_batch(rgb_batch: Any, cfg: EventCameraCfg) -> Any:
    """Apply the configured ROI and integer downsample before backend input."""

    crop = cfg.roi_crop
    downsample = cfg.downsample
    if crop is None and downsample == 1:
        return rgb_batch

    if isinstance(rgb_batch, (list, tuple)):
        values = type(rgb_batch)(
            None if frame is None else _preprocess_rgb_frame(frame, cfg)
            for frame in rgb_batch
        )
        return values
    if getattr(rgb_batch, "ndim", None) == 4:
        x, y, width, height = _rgb_crop_bounds(cfg)
        effective_width, effective_height = cfg.effective_resolution
        cropped = rgb_batch[:, y : y + height, x : x + width, :]
        return cropped[
            :,
            : effective_height * cfg.downsample : cfg.downsample,
            : effective_width * cfg.downsample : cfg.downsample,
            :,
        ]
    return _preprocess_rgb_frame(rgb_batch, cfg)


def _preprocess_rgb_frame(rgb: Any, cfg: EventCameraCfg) -> Any:
    if getattr(rgb, "ndim", None) != 3:
        raise ValueError(
            "event-camera RGB frame must have shape (H, W, C); "
            f"got {getattr(rgb, 'shape', None)!r}"
        )
    x, y, width, height = _rgb_crop_bounds(cfg)
    effective_width, effective_height = cfg.effective_resolution
    cropped = rgb[y : y + height, x : x + width, :]
    return cropped[
        : effective_height * cfg.downsample : cfg.downsample,
        : effective_width * cfg.downsample : cfg.downsample,
        :,
    ]


def _rgb_crop_bounds(cfg: EventCameraCfg) -> tuple[int, int, int, int]:
    if cfg.roi_crop is None:
        return (0, 0, cfg.resolution[0], cfg.resolution[1])
    return cfg.roi_crop


__all__ = [
    "EventBatchCardinalityError",
    "EventBackendDispatch",
    "derive_event_camera_episode_seed",
    "EventObservationOverflowError",
    "EventRGBUnavailableError",
]
