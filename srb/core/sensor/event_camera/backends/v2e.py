"""Adapter for v2e's stateful, single-environment ``EventEmulator`` backend.

The optional provider is imported at instantiation; large instance counts warn.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Iterator

from srb.core.sensor.event_camera.backend import (
    EventGenerator,
    EventStreamingUnsupportedError,
    register_backend,
)
from srb.core.sensor.event_camera.extras import require_extra

if TYPE_CHECKING:
    import numpy as np
    import torch

    from srb.core.sensor.event_camera.config import EventCameraCfg


ENV_COUNT_WARN_THRESHOLD = 8
# Emit one scale warning after this many per-environment instances.
_UINT64_MASK = (1 << 64) - 1


def resolve_backend_device(cfg: "EventCameraCfg") -> str:
    """Return ``cfg.backend_device`` or ``"cuda"`` when available, else ``"cpu"``."""
    if cfg.backend_device is not None:
        return str(cfg.backend_device)
    import torch

    return "cuda" if torch.cuda.is_available() else "cpu"


class ProviderRng:
    """Isolate provider draws from process-global RNG streams.

    :meth:`active` installs private Python, NumPy, and Torch states around each
    call, then restores caller states. Provider draws remain reproducible from
    ``seed`` without advancing or reseeding caller streams.
    """

    def __init__(self, seed: int, device: str) -> None:
        import random

        import numpy as np
        import torch

        seed = int(seed) & _UINT64_MASK
        self._cuda_device: torch.device | None = None
        dev = torch.device(device)
        if dev.type == "cuda":
            index = dev.index if dev.index is not None else torch.cuda.current_device()
            self._cuda_device = torch.device("cuda", index)
        cpu = torch.Generator(device="cpu")
        cpu.manual_seed(seed)
        self._torch_cpu = cpu.get_state()
        self._torch_cuda: torch.Tensor | None = None
        if self._cuda_device is not None:
            cuda = torch.Generator(device=self._cuda_device)
            cuda.manual_seed(seed)
            self._torch_cuda = cuda.get_state()
        self._numpy = np.random.RandomState(seed & 0xFFFFFFFF).get_state()
        self._python = random.Random(seed).getstate()

    @contextmanager
    def active(self) -> Iterator[None]:
        import random

        import numpy as np
        import torch

        saved_cpu = torch.get_rng_state()
        saved_cuda = (
            torch.cuda.get_rng_state(self._cuda_device)
            if self._cuda_device is not None
            else None
        )
        saved_numpy = np.random.get_state()
        saved_python = random.getstate()
        torch.set_rng_state(self._torch_cpu)
        if self._torch_cuda is not None:
            torch.cuda.set_rng_state(self._torch_cuda, self._cuda_device)
        np.random.set_state(self._numpy)
        random.setstate(self._python)
        try:
            yield
        finally:
            self._torch_cpu = torch.get_rng_state()
            if saved_cuda is not None:
                self._torch_cuda = torch.cuda.get_rng_state(self._cuda_device)
                torch.cuda.set_rng_state(saved_cuda, self._cuda_device)
            self._numpy = np.random.get_state()
            self._python = random.getstate()
            torch.set_rng_state(saved_cpu)
            np.random.set_state(saved_numpy)
            random.setstate(saved_python)


class V2eBackend(EventGenerator):
    """One-environment v2e generator; warns once above ``ENV_COUNT_WARN_THRESHOLD``."""

    supports_batching: bool = False
    supports_streaming: bool = False
    requires_rgb: bool = True
    name: str = "v2e"
    commit_sha: str | None = None  # PyPI install ⇒ no source SHA

    _instance_count: int = 0
    _scale_warning_emitted: bool = False

    def __init__(self, cfg: "EventCameraCfg") -> None:
        with require_extra("event-v2e"):
            import v2ecore  # noqa: F401
            from v2ecore.emulator import EventEmulator

        self.version = self._resolve_version()
        self._cfg = cfg
        self._device = resolve_backend_device(cfg)
        self._seed: int | None = None
        self._episode_id: int | None = None
        self._rng = ProviderRng(0, self._device)
        # Episode origin: the first frame after a reset. The provider sees
        # ``(t_ns - t0_ns) * 1e-9`` seconds; canonical events add ``t0_ns``
        # back in int64.
        self._t0_ns: int | None = None

        with self._rng.active():
            self._emulator: Any = EventEmulator(**self._build_emulator_kwargs())

        type(self)._instance_count += 1
        self._maybe_emit_scale_warning()

    @staticmethod
    def _resolve_version() -> str:
        try:
            import v2ecore

            version = getattr(v2ecore, "__version__", None)
            if version:
                return str(version)
        except ImportError:
            pass
        try:
            from importlib.metadata import PackageNotFoundError, version
        except Exception:  # noqa: BLE001
            return "unknown"

        try:
            return version("v2e")
        except PackageNotFoundError:
            return "unknown"

    def _build_emulator_kwargs(self) -> dict[str, Any]:
        cfg = self._cfg
        return {
            "pos_thres": cfg.pos_thres,
            "neg_thres": cfg.neg_thres,
            "sigma_thres": cfg.sigma_thres,
            "cutoff_hz": cfg.cutoff_hz,
            "leak_rate_hz": cfg.leak_rate_hz,
            "shot_noise_rate_hz": cfg.shot_noise_rate_hz,
            "refractory_period_s": cfg.refractory_period_s,
            "output_width": cfg.effective_resolution[0],
            "output_height": cfg.effective_resolution[1],
            "show_dvs_model_state": None,
            "device": self._device,
            # A nonzero seed makes v2e call torch.manual_seed, np.random.seed
            # and random.seed on the process-global generators. SRB seeds the
            # provider through ``ProviderRng`` instead.
            "seed": 0,
        }

    def _relative_seconds(self, t_ns: int) -> float:
        if self._t0_ns is None:
            self._t0_ns = int(t_ns)
        return (int(t_ns) - self._t0_ns) * 1e-9

    def iter_frame(
        self,
        rgb: "torch.Tensor | np.ndarray",
        t_ns: int,
    ) -> "Iterator[torch.Tensor]":
        """Yield chunks from the reviewed ``iter_events`` provider patch.

        The provider gets separate transport and workspace limits. Interruption
        resets its state; normal exhaustion commits the frame timestamp.
        """
        import numpy as np
        import torch

        if not self.supports_streaming:
            raise EventStreamingUnsupportedError(self.name, "iter_frame")
        if rgb is None:
            return
        if isinstance(rgb, torch.Tensor):
            rgb_np = rgb.detach().cpu().numpy()
        else:
            rgb_np = np.asarray(rgb)
        luma = rgb_to_luma(rgb_np, backend="V2eBackend")
        iterator_factory = getattr(self._emulator, "iter_events", None)
        if not callable(iterator_factory):
            raise EventStreamingUnsupportedError(self.name, "iter_frame")
        t_s = self._relative_seconds(t_ns)
        t0_ns = self._t0_ns
        assert t0_ns is not None

        completed = False
        raw_chunks: Any = None
        try:
            with self._rng.active():
                raw_chunks = iterator_factory(
                    luma,
                    t_s,
                    max_events_per_chunk=self._cfg.event_chunk_rows,
                    max_workspace_events=self._cfg.provider_workspace_rows,
                )
            while True:
                # Provider code runs only inside ``next``; the private RNG
                # state is active exactly for that span, never across a yield.
                with self._rng.active():
                    try:
                        raw_chunk = next(raw_chunks)
                    except StopIteration:
                        break
                chunk = _v2e_events_to_canonical(raw_chunk, t_ns, offset_ns=t0_ns)
                if chunk.shape[0] == 0:
                    continue
                yield chunk
            completed = True
        finally:
            if not completed:
                try:
                    close = getattr(raw_chunks, "close", None)
                    if callable(close):
                        with self._rng.active():
                            close()
                finally:
                    self.reset(
                        seed=0 if self._seed is None else self._seed,
                        episode_id=(
                            0 if self._episode_id is None else self._episode_id
                        ),
                    )

    def _maybe_emit_scale_warning(self) -> None:
        cls = type(self)
        if cls._scale_warning_emitted:
            return
        if cls._instance_count > ENV_COUNT_WARN_THRESHOLD:
            from srb.utils import logging

            logging.warning(
                "V2eBackend instantiated %d times (threshold=%d). Each v2e "
                "instance processes one environment; use Metavision or V2CE "
                "for batched rollouts.",
                cls._instance_count,
                ENV_COUNT_WARN_THRESHOLD,
            )
            cls._scale_warning_emitted = True

    def process_frame(
        self,
        rgb: "torch.Tensor | np.ndarray | None",
        t_ns: int,
    ) -> "torch.Tensor":
        import numpy as np
        import torch

        if rgb is None:
            return torch.zeros((0, 4), dtype=torch.int64)

        if isinstance(rgb, torch.Tensor):
            rgb_np = rgb.detach().cpu().numpy()
        else:
            rgb_np = np.asarray(rgb)
        luma = rgb_to_luma(rgb_np, backend="V2eBackend")

        t_s = self._relative_seconds(t_ns)
        with self._rng.active():
            events = self._emulator.generate_events(luma, t_s)
        return _v2e_events_to_canonical(events, t_ns, offset_ns=self._t0_ns or 0)

    def reset(self, *, seed: int, episode_id: int) -> None:
        self._seed = seed
        self._episode_id = episode_id
        self._t0_ns = None
        # v2e's EventEmulator carries per-pixel state from frame to frame; the
        # canonical reset is to rebuild the emulator with the same kwargs and
        # a fresh private RNG stream derived from the episode seed.
        from v2ecore.emulator import EventEmulator

        self._rng = ProviderRng(seed, self._device)
        with self._rng.active():
            self._emulator = EventEmulator(**self._build_emulator_kwargs())


class V2eStreamingBackend(V2eBackend):
    """Explicit v2e iterator backend for the reviewed local provider patch."""

    supports_streaming: bool = True
    name: str = "v2e_streaming"

    def __init__(self, cfg: "EventCameraCfg", **kwargs: Any) -> None:
        if cfg.shot_noise_rate_hz != 0:
            raise EventStreamingUnsupportedError(
                "v2e_streaming",
                "iter_frame (shot noise must be disabled)",
            )
        super().__init__(cfg, **kwargs)
        if not callable(getattr(self._emulator, "iter_events", None)):
            raise EventStreamingUnsupportedError("v2e_streaming", "iter_frame")
        _require_reviewed_provider_source()
        _require_streaming_profile(self._emulator)

    def reset(self, *, seed: int, episode_id: int) -> None:
        # The cfg is mutable and a reset rebuilds the emulator from it.
        super().reset(seed=seed, episode_id=episode_id)
        _require_streaming_profile(self._emulator)


# Provider features the reviewed ``iter_events`` profile excludes besides shot
# noise. The patch also rejects them, but only once the first
# frame arrives.
_STREAMING_PROFILE_DISABLED_FEATURES = (
    "photoreceptor_noise",
    "csdvs_enabled",
    "scidvs",
)


def _require_reviewed_provider_source() -> None:
    """Require the digest-pinned ``v2ecore.emulator`` patch for streaming."""
    from srb.core.sensor.event_camera.backends.v2e_install import (
        V2ePatchError,
        apply_v2e_patch,
    )

    try:
        apply_v2e_patch(check=True)
    except V2ePatchError as exc:
        raise EventStreamingUnsupportedError(
            "v2e_streaming",
            f"iter_frame (the installed provider is not the reviewed patch: {exc})",
        ) from exc


def _require_streaming_profile(emulator: Any) -> None:
    """Reject a provider state outside the signal-only streaming profile."""
    missing = object()
    for feature in ("shot_noise_rate_hz", *_STREAMING_PROFILE_DISABLED_FEATURES):
        value = getattr(emulator, feature, missing)
        if value is missing:
            raise EventStreamingUnsupportedError(
                "v2e_streaming",
                f"iter_frame (provider does not expose {feature})",
            )
        if value:
            raise EventStreamingUnsupportedError(
                "v2e_streaming",
                f"iter_frame (provider {feature} must be disabled)",
            )


def rgb_to_luma(rgb: "np.ndarray", *, backend: str) -> "np.ndarray":
    """Convert an RGB frame to single-channel luma (Rec. 709)."""
    import numpy as np

    if rgb.ndim == 4:
        rgb = rgb[0]  # batch-of-1 frame; recorder loops over batch outside
    if rgb.ndim == 2:
        return rgb.astype(np.float32)
    if rgb.ndim != 3 or rgb.shape[-1] < 3:
        raise ValueError(
            f"{backend} requires HxWxC (C>=3) or HxW luma; got {rgb.shape}"
        )
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    return (0.2126 * r + 0.7152 * g + 0.0722 * b).astype(np.float32)


def _v2e_events_to_canonical(
    events: Any,
    frame_t_ns: int,
    *,
    offset_ns: int = 0,
) -> "torch.Tensor":
    """Convert v2e ``(t_s, x, y, p)`` events to canonical int64 events.

    Restore the ``offset_ns`` origin and cap timestamps at ``frame_t_ns`` to
    correct float32 rounding. Empty or ``None`` input yields an empty tensor.
    """
    import numpy as np
    import torch

    if events is None:
        return torch.zeros((0, 4), dtype=torch.int64)
    if isinstance(events, torch.Tensor):
        arr = events.detach().cpu().numpy()
    else:
        arr = np.asarray(events)
    if arr.size == 0:
        return torch.zeros((0, 4), dtype=torch.int64)
    if arr.ndim != 2 or arr.shape[1] < 4:
        raise ValueError(
            f"V2eBackend: unexpected event array shape from v2e: {arr.shape}"
        )

    t_s = arr[:, 0].astype(np.float64)
    x = arr[:, 1].astype(np.int64)
    y = arr[:, 2].astype(np.int64)
    p = arr[:, 3].astype(np.int64)
    t_ns = np.int64(offset_ns) + np.rint(t_s * 1e9).astype(np.int64)
    t_ns = np.minimum(t_ns, np.int64(frame_t_ns))
    # Polarity is canonically ±1; v2e uses ±1 directly, but some forks emit 0/1.
    p = np.where(p > 0, 1, -1).astype(np.int64)

    canonical = np.stack([t_ns, x, y, p], axis=1)
    canonical = canonical[np.argsort(canonical[:, 0], kind="stable")]
    return torch.from_numpy(canonical)


register_backend("v2e", V2eBackend)
register_backend("v2e_streaming", V2eStreamingBackend)


__all__ = [
    "ENV_COUNT_WARN_THRESHOLD",
    "ProviderRng",
    "V2eBackend",
    "V2eStreamingBackend",
    "resolve_backend_device",
    "rgb_to_luma",
]
