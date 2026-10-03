"""V2CE event-simulator adapter for package and Toolbox providers."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from pathlib import Path
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


class V2CEBackend(EventGenerator):
    """Batched V2CE generator with isolated model and RNG state per environment.

    Requires an upstream checkpoint via ``EventCameraCfg.v2ce_model_path`` or
    ``model_path``. Models receive episode-relative microseconds.
    """

    supports_batching: bool = True
    supports_streaming: bool = False
    requires_rgb: bool = True
    name: str = "v2ce"
    commit_sha: str | None = None  # checkpoint-driven; no source SHA

    def __init__(
        self,
        cfg: "EventCameraCfg",
        *,
        model_path: Path | str | None = None,
        device: str | None = None,
    ) -> None:
        with require_extra("event-v2ce"):
            self._runtime = _resolve_v2ce_runtime()

        resolved_path = model_path if model_path is not None else cfg.v2ce_model_path
        if resolved_path is None:
            raise ValueError(
                "V2CEBackend requires a model checkpoint: set "
                "EventCameraCfg.v2ce_model_path (or pass model_path=). Fetch the "
                "SRB mirror with `srb event-camera fetch-weights --mirror "
                "hf-andrejorsula-v2ce` or pass `--url <mirror-url> --sha256 <hex>` "
                "for another verified mirror."
            )

        self.version = self._resolve_version()
        self._cfg = cfg
        self._device = device if device is not None else resolve_backend_device(cfg)
        self._model_path = Path(resolved_path)
        self._rng_by_env: dict[int, ProviderRng] = {}
        # Per-env episode origin (first frame after a reset); the model sees
        # ``(t_ns - t0) // 1000`` µs and canonical events add ``t0`` back.
        self._t0_ns_by_env: dict[int, int] = {}
        self._models: list[Any] = [self._new_model(0)]
        # Backwards-compatible single-frame handle used by ``process_frame``.
        self._model: Any = self._models[0]

    @staticmethod
    def _resolve_version() -> str:
        try:
            import v2ce

            version = getattr(v2ce, "__version__", None)
            if version:
                return str(version)
        except ImportError:
            pass
        try:
            from importlib.metadata import PackageNotFoundError, version
        except Exception:  # noqa: BLE001
            return "unknown"

        try:
            return version("v2ce")
        except PackageNotFoundError:
            return "unknown"

    def _build_model_kwargs(self) -> dict[str, Any]:
        cfg = self._cfg
        w, h = cfg.effective_resolution
        return {
            "model_path": str(self._model_path),
            "width": w,
            "height": h,
            "device": self._device,
        }

    def _rng_for(self, env_id: int) -> ProviderRng:
        rng = self._rng_by_env.get(env_id)
        if rng is None:
            rng = ProviderRng(0, self._device)
            self._rng_by_env[env_id] = rng
        return rng

    def _new_model(self, env_id: int) -> Any:
        # Module construction initialises weights from the global torch RNG
        # before the checkpoint is loaded; keep those draws private.
        with self._rng_for(env_id).active():
            return self._runtime.create_model(**self._build_model_kwargs())

    def _ensure_env_capacity(self, num_envs: int) -> None:
        while len(self._models) < num_envs:
            self._models.append(self._new_model(len(self._models)))
        self._model = self._models[0]

    def _invoke_model(
        self,
        luma: "np.ndarray",
        t_us: int,
        *,
        model: Any,
    ) -> Any:
        """Call the V2CE provider through one adapter seam."""
        return model(luma, t_us)

    def _process_frame_with_model(
        self,
        rgb: "torch.Tensor | np.ndarray | None",
        t_ns: int,
        model: Any,
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
        luma = rgb_to_luma(rgb_np, backend="V2CEBackend")

        # V2CE works in microseconds in its public docs; canonical SRB clock
        # is nanoseconds. The model sees episode-relative time only.
        t0_ns = self._t0_ns_by_env.setdefault(env_id, int(t_ns))
        t_us = (int(t_ns) - t0_ns) // 1000
        with self._rng_for(env_id).active():
            events = self._invoke_model(luma, t_us, model=model)
        return _v2ce_events_to_canonical(events, offset_ns=t0_ns)

    def process_frame(
        self,
        rgb: "torch.Tensor | np.ndarray | None",
        t_ns: int,
    ) -> "torch.Tensor":
        events = self._process_frame_with_model(rgb, t_ns, self._model)
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
            events = self._process_frame_with_model(
                rgb, t_ns, self._models[env_id], env_id
            )
            events_per_env.append(events)
        return events_per_env

    def reset(self, *, seed: int, episode_id: int) -> None:
        self._t0_ns_by_env = {}
        self._rng_by_env = {
            env_id: ProviderRng(seed, self._device)
            for env_id in range(len(self._models))
        }
        self._models = [self._new_model(env_id) for env_id in range(len(self._models))]
        self._model = self._models[0]

    def reset_env(self, env_id: int, *, seed: int, episode_id: int) -> None:
        self._ensure_env_capacity(env_id + 1)
        self._rng_by_env[env_id] = ProviderRng(seed, self._device)
        self._t0_ns_by_env.pop(env_id, None)
        self._models[env_id] = self._new_model(env_id)
        if env_id == 0:
            self._model = self._models[0]


class _PackageV2CERuntime:
    def __init__(self, model_cls: type[Any]) -> None:
        self._model_cls = model_cls

    def create_model(self, **kwargs: Any) -> Any:
        return self._model_cls(**kwargs)


class _ToolboxV2CERuntime:
    def __init__(
        self,
        model_cls: type[Any],
        sampler: Callable[..., list[Any]],
    ) -> None:
        self._model_cls = model_cls
        self._sampler = sampler

    def create_model(self, **kwargs: Any) -> Any:
        return _V2CEToolboxModel(
            model_cls=self._model_cls,
            sampler=self._sampler,
            **kwargs,
        )


def _resolve_v2ce_runtime() -> Any:
    """Resolve either a package-style V2CE provider or V2CE-Toolbox source tree."""
    package_error: ImportError | None = None
    try:
        from v2ce.inference import V2CE
    except ImportError as exc:
        package_error = exc
    else:
        return _PackageV2CERuntime(V2CE)

    try:
        from scripts.LDATI import sample_voxel_statistical
        from scripts.v2ce_3d import V2ce3d
    except ImportError as exc:
        message = (
            "Could not import V2CE provider. Install a package exposing "
            "`v2ce.inference.V2CE`, or clone "
            "https://github.com/ucsd-hdsi-dvs/V2CE-Toolbox and add that "
            "repository root to PYTHONPATH so `scripts.v2ce_3d` and "
            "`scripts.LDATI` are importable."
        )
        raise ImportError(message) from (package_error or exc)
    return _ToolboxV2CERuntime(V2ce3d, sample_voxel_statistical)


class _V2CEToolboxModel:
    """Adapter around the script-style ucsd-hdsi-dvs/V2CE-Toolbox modules."""

    def __init__(
        self,
        *,
        model_cls: type[Any],
        sampler: Callable[..., list[Any]],
        model_path: str,
        width: int,
        height: int,
        device: str,
    ) -> None:
        import torch

        self._device = torch.device(device)
        self._width = int(width)
        self._height = int(height)
        self._sampler = sampler
        self._last_luma: np.ndarray | None = None
        self._last_t_us: int | None = None

        # Security: weights_only=True restricts unpickling to tensors / plain
        # containers, so a tampered checkpoint (the weights are fetched + only
        # sha256-gated) cannot execute arbitrary code via a pickle payload. The
        # checkpoint is consumed as a state_dict just below, which is
        # weights_only-compatible. PyTorch >= 2.6 already defaults to this; pinned
        # explicitly so older runtimes are protected too.
        checkpoint = torch.load(
            str(model_path), map_location=self._device, weights_only=True
        )
        self._model = model_cls()
        self._model.load_state_dict(_v2ce_state_dict(checkpoint))
        self._model.to(self._device)
        self._model.eval()

    def __call__(self, luma: "np.ndarray", t_us: int) -> Any:
        import torch

        current = _resize_v2ce_luma(luma, width=self._width, height=self._height)
        if self._last_luma is None or self._last_t_us is None:
            self._last_luma = current
            self._last_t_us = int(t_us)
            return None

        previous = self._last_luma
        previous_t_us = self._last_t_us

        dt_us = max(1, int(t_us) - int(previous_t_us))
        model_input = _v2ce_image_pair_to_tensor(
            previous,
            current,
            device=self._device,
        )
        with torch.no_grad():
            voxel = self._model(model_input)
        stage2_input = _v2ce_voxel_to_stage2_input(voxel)
        event_batches = self._sampler(
            stage2_input,
            t0=float(previous_t_us) / 1_000_000.0,
            fps=1_000_000.0 / float(dt_us),
            bidirectional=False,
            additional_events_strategy="slope",
        )
        self._last_luma = current
        self._last_t_us = int(t_us)
        if not event_batches:
            return None
        return event_batches[0]


def _v2ce_state_dict(checkpoint: Any) -> Any:
    if isinstance(checkpoint, dict):
        for key in ("state_dict", "model", "model_state_dict"):
            value = checkpoint.get(key)
            if isinstance(value, dict):
                return value
    return checkpoint


def _resize_v2ce_luma(
    luma: "np.ndarray",
    *,
    width: int,
    height: int,
) -> "np.ndarray":
    import numpy as np

    frame = np.asarray(luma, dtype=np.float32)
    if frame.shape != (height, width):
        import cv2

        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    return np.asarray(frame, dtype=np.float32)


def _v2ce_image_pair_to_tensor(
    previous: "np.ndarray",
    current: "np.ndarray",
    *,
    device: Any,
) -> Any:
    import numpy as np
    import torch

    pair = np.stack([previous, current], axis=0).astype(np.float32, copy=False)
    if pair.size and float(pair.max()) > 1.0:
        pair = pair / 255.0
    tensor = torch.from_numpy(pair).to(device=device, dtype=torch.float32)
    mean = torch.tensor([0.153, 0.153], device=device, dtype=torch.float32).view(
        2, 1, 1
    )
    std = torch.tensor([0.165, 0.165], device=device, dtype=torch.float32).view(2, 1, 1)
    tensor = (tensor - mean) / std
    return tensor.unsqueeze(0).unsqueeze(0)


def _v2ce_voxel_to_stage2_input(voxel: Any) -> Any:
    if voxel.ndim != 5 or int(voxel.shape[2]) != 20:
        raise ValueError(
            "V2CEBackend: unexpected V2CE-Toolbox voxel shape "
            f"{tuple(voxel.shape)}; expected (B, L, 20, H, W)"
        )
    _, _, _, h, w = voxel.shape
    return voxel.reshape(-1, 2, 10, int(h), int(w))


def _v2ce_events_to_canonical(
    events: Any,
    *,
    offset_ns: int = 0,
) -> "torch.Tensor":
    """Convert V2CE events to canonical ``(t_ns, x, y, p)`` int64 rows.

    Accepts ``(N, 4)`` tensors/arrays or structured arrays with ``t, x, y, p``
    fields. Input times are microseconds relative to ``offset_ns``; output times
    are absolute nanoseconds. Empty or ``None`` input yields an empty tensor;
    polarity is normalized to ±1.
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

    if arr.dtype.names is not None:
        names = set(arr.dtype.names)
        if {"t", "x", "y", "p"}.issubset(names):
            t_us = arr["t"].astype(np.int64)
            x = arr["x"].astype(np.int64)
            y = arr["y"].astype(np.int64)
            p = arr["p"].astype(np.int64)
        elif {"timestamp", "x", "y", "polarity"}.issubset(names):
            t_us = arr["timestamp"].astype(np.int64)
            x = arr["x"].astype(np.int64)
            y = arr["y"].astype(np.int64)
            p = arr["polarity"].astype(np.int64)
        else:
            raise ValueError(
                "V2CEBackend: unexpected structured event fields from V2CE: "
                f"{arr.dtype.names}"
            )
    else:
        if arr.ndim != 2 or arr.shape[1] < 4:
            raise ValueError(
                f"V2CEBackend: unexpected event array shape from V2CE: {arr.shape}"
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


register_backend("v2ce", V2CEBackend)


__all__ = ["V2CEBackend"]
