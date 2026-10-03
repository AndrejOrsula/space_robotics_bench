"""Adapter for openEB's CUDA-batched ``GPUEventSimulator`` backend.

The opt-in dependency loads at instantiation. Batched dispatch creates one
simulator per environment to keep simulator state isolated.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

from srb.core.sensor.event_camera.backend import EventGenerator, register_backend
from srb.core.sensor.event_camera.backends.v2e import (
    ProviderRng,
    resolve_backend_device,
    rgb_to_luma,
)
from srb.core.sensor.event_camera.extras import require_extra

if TYPE_CHECKING:
    import numpy as np
    import torch

    from srb.core.sensor.event_camera.config import EventCameraCfg


class MetavisionBackend(EventGenerator):
    """openEB ``GPUEventSimulator``-backed event generator.

    The IMX636 contrast thresholds and refractory period are passed through. openEB
    exposes no constructor options for the preset's noise-model fields, so this backend
    does not apply them. Each simulator uses a per-environment ``ProviderRng``; reset
    seeds it. Provider draws are reproducible and do not affect process-global RNGs.
    """

    supports_batching: bool = True
    supports_streaming: bool = False
    requires_rgb: bool = True
    name: str = "metavision"
    commit_sha: str | None = None  # openEB install ⇒ no source SHA

    def __init__(self, cfg: "EventCameraCfg") -> None:
        with require_extra("event-metavision"):
            # Import path follows current openEB docs; untested on a live install.
            import metavision_core_ml  # noqa: F401
            from metavision_core_ml.event_simulator import (
                GPUEventSimulator as _GPUEventSimulator,
            )

            del _GPUEventSimulator

        self.version = self._resolve_version()
        self._cfg = cfg
        self._device = resolve_backend_device(cfg)
        self._rng_by_env: dict[int, ProviderRng] = {}
        self._simulators: list[Any] = [self._new_simulator(0)]
        # Backwards-compatible single-frame handle used by ``process_frame``.
        self._simulator: Any = self._simulators[0]
        # Per-env episode origin (first frame after a reset). The simulator
        # sees ``(t_ns - t0) // 1000`` µs; canonical events add ``t0`` back in
        # int64.
        self._t0_ns_by_env: dict[int, int] = {}

    @staticmethod
    def _resolve_version() -> str:
        try:
            import metavision_core_ml

            version = getattr(metavision_core_ml, "__version__", None)
            if version:
                return str(version)
        except ImportError:
            pass
        try:
            from importlib.metadata import PackageNotFoundError, version
        except Exception:  # noqa: BLE001
            return "unknown"

        try:
            return version("metavision_core_ml")
        except PackageNotFoundError:
            return "unknown"

    def _build_simulator_kwargs(self) -> dict[str, Any]:
        cfg = self._cfg
        w, h = cfg.effective_resolution
        # ``refractory_period_s`` is None-safe: ``EventCameraCfg.__post_init__``
        # populates every threshold/noise field from the active preset before
        # any backend ever sees the cfg.
        assert cfg.refractory_period_s is not None
        return {
            "width": w,
            "height": h,
            "cp": cfg.pos_thres,
            "cn": cfg.neg_thres,
            "refractory_period_us": int(cfg.refractory_period_s * 1e6),
        }

    def _rng_for(self, env_id: int) -> ProviderRng:
        rng = self._rng_by_env.get(env_id)
        if rng is None:
            rng = ProviderRng(0, self._device)
            self._rng_by_env[env_id] = rng
        return rng

    def _new_simulator(self, env_id: int) -> Any:
        from metavision_core_ml.event_simulator import GPUEventSimulator

        with self._rng_for(env_id).active():
            return GPUEventSimulator(**self._build_simulator_kwargs())

    def _ensure_env_capacity(self, num_envs: int) -> None:
        while len(self._simulators) < num_envs:
            self._simulators.append(self._new_simulator(len(self._simulators)))
        self._simulator = self._simulators[0]

    def _invoke_simulator(
        self,
        luma: "np.ndarray",
        t_us: int,
        *,
        simulator: Any | None = None,
    ) -> Any:
        """Call the openEB simulator through one API seam."""
        active_simulator = self._simulator if simulator is None else simulator
        return active_simulator(luma, t_us)

    def _process_frame_with_simulator(
        self,
        rgb: "torch.Tensor | np.ndarray | None",
        t_ns: int,
        simulator: Any,
        env_id: int = 0,
    ) -> "torch.Tensor":
        import numpy as np
        import torch

        if rgb is None:
            return torch.zeros((0, 4), dtype=torch.int64)

        if isinstance(rgb, torch.Tensor):
            rgb_np = rgb.detach().cpu().numpy()
        else:
            rgb_np = np.asarray(rgb)
        luma = rgb_to_luma(rgb_np, backend="MetavisionBackend")

        # openEB works in microseconds; the canonical SRB clock is nanoseconds.
        # The simulator sees episode-relative time only.
        t0_ns = self._t0_ns_by_env.setdefault(env_id, int(t_ns))
        t_us = (int(t_ns) - t0_ns) // 1000
        with self._rng_for(env_id).active():
            events = self._invoke_simulator(luma, t_us, simulator=simulator)
        return _metavision_events_to_canonical(events, offset_ns=t0_ns)

    def process_frame(
        self,
        rgb: "torch.Tensor | np.ndarray | None",
        t_ns: int,
    ) -> "torch.Tensor":
        events = self._process_frame_with_simulator(rgb, t_ns, self._simulator)
        return events

    def process_batch(
        self,
        rgb_batch: "Sequence[torch.Tensor | np.ndarray | None] | torch.Tensor | np.ndarray | None",
        t_ns: int,
    ) -> "list[torch.Tensor]":
        if rgb_batch is None:
            rgb_batch = [None]
        self._ensure_env_capacity(len(rgb_batch))

        events_per_env = []
        for env_id, rgb in enumerate(rgb_batch):
            events = self._process_frame_with_simulator(
                rgb, t_ns, self._simulators[env_id], env_id
            )
            events_per_env.append(events)
        return events_per_env

    def reset(self, *, seed: int, episode_id: int) -> None:
        self._t0_ns_by_env = {}
        self._rng_by_env = {
            env_id: ProviderRng(seed, self._device)
            for env_id in range(len(self._simulators))
        }
        # GPUEventSimulator carries per-pixel state across frames; canonical
        # reset is to rebuild it with the same kwargs (same shape as v2e).
        self._simulators = [
            self._new_simulator(env_id) for env_id in range(len(self._simulators))
        ]
        self._simulator = self._simulators[0]

    def reset_env(self, env_id: int, *, seed: int, episode_id: int) -> None:
        self._ensure_env_capacity(env_id + 1)
        self._rng_by_env[env_id] = ProviderRng(seed, self._device)
        self._simulators[env_id] = self._new_simulator(env_id)
        if env_id == 0:
            self._simulator = self._simulators[0]
        self._t0_ns_by_env.pop(env_id, None)


def _metavision_events_to_canonical(
    events: Any,
    *,
    offset_ns: int = 0,
) -> "torch.Tensor":
    """Cast openEB's event output to the canonical ``(t_ns, x, y, p)`` tensor.

    Accepts structured ``t``/``x``/``y``/``p`` records or 2D columns
    ``(t_us, x, y, p)``. Times are relative microseconds; ``offset_ns`` restores
    absolute time. Empty input returns an empty tensor; polarity becomes ±1.
    """
    import numpy as np
    import torch

    if events is None:
        return torch.zeros((0, 4), dtype=torch.int64)
    arr = np.asarray(events)
    if arr.size == 0:
        return torch.zeros((0, 4), dtype=torch.int64)

    # Structured-array path: openEB ships ``EventCD`` records with named fields.
    if arr.dtype.names is not None and {"t", "x", "y", "p"}.issubset(arr.dtype.names):
        t_us = arr["t"].astype(np.int64)
        x = arr["x"].astype(np.int64)
        y = arr["y"].astype(np.int64)
        p = arr["p"].astype(np.int64)
    else:
        if arr.ndim != 2 or arr.shape[1] < 4:
            raise ValueError(
                "MetavisionBackend: unexpected event array shape from "
                f"GPUEventSimulator: {arr.shape}"
            )
        t_us = arr[:, 0].astype(np.int64)
        x = arr[:, 1].astype(np.int64)
        y = arr[:, 2].astype(np.int64)
        p = arr[:, 3].astype(np.int64)

    t_ns = np.int64(offset_ns) + (t_us * 1_000).astype(np.int64)
    p = np.where(p > 0, 1, -1).astype(np.int64)

    canonical = np.stack([t_ns, x, y, p], axis=1)
    canonical = canonical[np.argsort(canonical[:, 0], kind="stable")]
    return torch.from_numpy(canonical)


register_backend("metavision", MetavisionBackend)


__all__ = ["MetavisionBackend"]
