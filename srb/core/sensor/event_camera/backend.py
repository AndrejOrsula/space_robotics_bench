"""Event-generator backend contract and registry."""

from __future__ import annotations

import abc
import operator
from collections.abc import Iterator, Sequence
from typing import TYPE_CHECKING, Any

from srb.core.sensor.event_camera.extras import install_command
from srb.utils.failures import InvalidInputError

if TYPE_CHECKING:
    import numpy as np
    import torch

    from srb.core.sensor.event_camera.config import EventCameraCfg


class EventStreamingUnsupportedError(RuntimeError):
    """A backend cannot satisfy a requested streaming operation."""

    code = "event_camera.streaming_unsupported"

    def __init__(self, backend: str, operation: str) -> None:
        self.backend = backend
        self.operation = operation
        super().__init__(
            f"event-camera backend {backend!r} does not support {operation}; "
            "the transitional legacy path is not available for this call"
        )


class EventChunkOverflowError(ValueError):
    """A backend result exceeds the event chunk row limit."""

    code = "event_camera.event_chunk_overflow"

    def __init__(
        self,
        *,
        rows: int,
        max_rows: int,
        key: tuple[int, str] | None = None,
    ) -> None:
        self.rows = rows
        self.max_rows = max_rows
        self.key = key
        location = f" for {key!r}" if key is not None else ""
        super().__init__(
            f"event chunk rows {rows} exceed event_chunk_rows {max_rows}{location}"
        )


class EventStreamNoProgressError(RuntimeError):
    """A streaming iterator yielded no events or progress."""

    code = "event_camera.stream_no_progress"

    def __init__(self, backend: str, operation: str) -> None:
        self.backend = backend
        self.operation = operation
        super().__init__(
            f"event-camera backend {backend!r} produced an empty {operation} "
            "yield; represent an empty interval by ending the iterator"
        )


def validate_event_chunk(
    events: Any,
    *,
    max_rows: int,
    key: tuple[int, str] | None = None,
) -> int:
    """Validate an event array and return its row count."""

    shape = getattr(events, "shape", None)
    try:
        ndim = len(shape) if shape is not None else 0
        rows = operator.index(shape[0]) if ndim >= 1 else -1
        columns = operator.index(shape[1]) if ndim >= 2 else -1
    except (IndexError, TypeError, ValueError) as exc:
        raise ValueError(f"event chunk must be shape (N, 4); got {shape!r}") from exc
    if ndim != 2 or columns != 4 or rows < 0:
        raise ValueError(f"event chunk must be shape (N, 4); got {shape!r}")
    if rows > max_rows:
        raise EventChunkOverflowError(rows=rows, max_rows=max_rows, key=key)
    return rows


def validate_streaming_backend_class(
    factory: type["EventGenerator"],
    *,
    backend_name: str,
) -> None:
    """Require the iterator method selected by dispatch."""

    if not factory.supports_streaming:
        return
    operation = "iter_batch" if factory.supports_batching else "iter_frame"
    if getattr(factory, operation) is getattr(EventGenerator, operation):
        raise EventStreamingUnsupportedError(backend_name, operation)


class EventGenerator(abc.ABC):
    """Abstract backend for generating events from RGB frames."""

    supports_batching: bool = False
    # Streaming backends must yield bounded chunks without materializing the
    # complete upstream result.
    supports_streaming: bool = False
    # Concrete simulators require a rendered RGB frame.
    requires_rgb: bool = False
    name: str = "abstract"
    version: str = "0.0.0"
    commit_sha: str | None = None

    @abc.abstractmethod
    def process_frame(
        self,
        rgb: "torch.Tensor | np.ndarray",
        t_ns: int,
    ) -> "torch.Tensor":
        """Consume RGB at ``t_ns`` nanoseconds; return sorted events.

        Columns: ``(t_ns int64, x uint16, y uint16, p int8 in {-1, +1})``.
        Empty output is valid.
        """

    @abc.abstractmethod
    def reset(self, *, seed: int, episode_id: int) -> None:
        """Reseed RNG and clear any per-episode internal state."""

    def process_batch(
        self,
        rgb_batch: "Sequence[torch.Tensor | np.ndarray | None] | torch.Tensor | np.ndarray | None",
        t_ns: int,
    ) -> "list[torch.Tensor]":
        """Process per-env frames at one sub-tick, returning one event tensor each.

        The default loops over :meth:`process_frame`; batched backends may
        override it. ``None`` is treated as one missing frame.
        """
        if rgb_batch is None:
            return [self.process_frame(None, t_ns)]  # type: ignore[arg-type]
        return [self.process_frame(rgb, t_ns) for rgb in rgb_batch]

    def iter_frame(
        self,
        rgb: "torch.Tensor | np.ndarray",
        t_ns: int,
    ) -> Iterator["torch.Tensor"]:
        """Yield bounded chunks for one frame; unsupported by default.

        It does not fall back to :meth:`process_frame`.
        """

        del rgb, t_ns
        raise EventStreamingUnsupportedError(self.name, "iter_frame")

    def iter_batch(
        self,
        rgb_batch: "Sequence[torch.Tensor | np.ndarray | None] | torch.Tensor | np.ndarray | None",
        t_ns: int,
    ) -> Iterator[Sequence["torch.Tensor"]]:
        """Yield bounded per-env event chunks for one batch tick."""

        del rgb_batch, t_ns
        raise EventStreamingUnsupportedError(self.name, "iter_batch")

    def reset_env(self, env_id: int, *, seed: int, episode_id: int) -> None:
        """Reset one env's episode state; the default resets the full backend.

        Batched backends can override this to reset only that env's state.
        """
        del env_id
        self.reset(seed=seed, episode_id=episode_id)


class NoopBackend(EventGenerator):
    """Backend that emits an empty ``(0, 4)`` event tensor."""

    supports_batching: bool = False
    name: str = "noop"
    version: str = "1"
    commit_sha: str | None = None

    def __init__(self, cfg: "EventCameraCfg") -> None:
        self._cfg = cfg
        self._seed: int | None = None
        self._episode_id: int | None = None

    def process_frame(
        self,
        rgb: "torch.Tensor | np.ndarray",
        t_ns: int,
    ) -> "torch.Tensor":
        import torch

        del rgb, t_ns
        return torch.zeros((0, 4), dtype=torch.int64)

    def reset(self, *, seed: int, episode_id: int) -> None:
        self._seed = seed
        self._episode_id = episode_id


_BACKEND_REGISTRY: dict[str, type[EventGenerator]] = {}


def register_backend(name: str, factory: type[EventGenerator]) -> None:
    """Register or replace an :class:`EventGenerator` under ``name``."""
    _BACKEND_REGISTRY[name] = factory


def get_backend_factory(name: str) -> type[EventGenerator]:
    try:
        return _BACKEND_REGISTRY[name]
    except KeyError as e:
        available = sorted(_BACKEND_REGISTRY)
        raise InvalidInputError(
            f"Unknown event-camera backend {name!r}; available: {available}. "
            "Install the corresponding extra and provider (e.g. "
            f"{install_command('event-v2e')!r}) or register a custom backend "
            "via register_backend()."
        ) from e


def list_backends() -> tuple[str, ...]:
    return tuple(sorted(_BACKEND_REGISTRY))


def make_backend(cfg: "EventCameraCfg") -> EventGenerator:
    """Construct the backend selected by ``cfg.backend``."""
    factory = get_backend_factory(cfg.backend)
    return factory(cfg)


register_backend("noop", NoopBackend)


__all__ = [
    "EventChunkOverflowError",
    "EventGenerator",
    "NoopBackend",
    "EventStreamNoProgressError",
    "EventStreamingUnsupportedError",
    "get_backend_factory",
    "list_backends",
    "make_backend",
    "register_backend",
    "validate_event_chunk",
    "validate_streaming_backend_class",
]
