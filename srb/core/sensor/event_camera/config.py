"""Isaac-free config; scene construction materializes its internal camera."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

from srb.core.sensor.category import SensorCategory
from srb.core.sensor.event_camera.preset import IMX636Preset, get_preset
from srb.utils.resource_budget import MAX_EVENT_BATCH_ROWS

# ``event_chunk_rows`` bounds transport chunks; provider workspaces and retained
# per-step rows have separate bounds. A missing observation bound resolves to
# four events per effective-resolution pixel.
EVENT_ROW_BYTES = 32
DEFAULT_EVENT_CHUNK_ROWS = 65_536
DEFAULT_OBSERVATION_WINDOW_EVENTS_PER_PIXEL = 4
# Per-hot-pixel cap per started ``1 / sub_render_hz`` period.
MAX_HOT_PIXEL_EVENTS_PER_TICK = 64

_EVENT_REPRESENTATIONS: tuple[str, ...] = (
    "voxel_grid",
    "histogram",
    "time_surface",
    "stacked",
    "raw",
)


@dataclass
class EventCameraCfg:
    """IMX636-style camera settings; presets fill unset model fields."""

    # Scene wiring resolves the attach point relative to robot or world.
    prim_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    resolution: tuple[int, int] = (1280, 720)
    focal_length_mm: float = 5.0
    horizontal_aperture_mm: float = 7.04  # IMX636 4.86mm horizontal sensor *...
    clipping_range: tuple[float, float] = (0.05, 1.0e4)

    # Applied before backend; affects observations and recordings.
    roi_crop: tuple[int, int, int, int] | None = None  # (x, y, w, h)
    downsample: int = 1

    # Unset sensor-model fields inherit from ``preset_name``.
    preset_name: str = "imx636_nominal"
    pos_thres: float | None = None
    neg_thres: float | None = None
    sigma_thres: float | None = None
    cutoff_hz: float | None = None
    leak_rate_hz: float | None = None
    shot_noise_rate_hz: float | None = None
    refractory_period_s: float | None = None
    hot_pixel_frac: float | None = None
    # Each tick samples Poisson(rate * elapsed) ON events at ordered times,
    # capped per started tick period. Default is 10x the highest built-in
    # shot-noise rate; zero disables injection and leaves the recorded mask empty.
    hot_pixel_rate_hz: float = 200.0

    # Frames closer than ``1 / sub_render_hz`` to the last consumed frame skip.
    sub_render_hz: float = 250.0

    representation: str = "voxel_grid"
    time_bins: int = 5
    normalize: bool = True  # per-step divide by max abs (voxel) / max (others)
    time_surface_tau_s: float = 0.03  # exp-decay time constant for time_surface
    raw_max_events: int = 4096  # padded length for the `raw` representation

    # Transport chunks must fit the provider workspace. A missing observation
    # bound resolves from current geometry at use, so geometry overrides cannot
    # leave it stale; explicit values are retained.
    event_chunk_rows: int = DEFAULT_EVENT_CHUNK_ROWS
    observation_window_events: int | None = None
    # Separate ceiling: native streaming may allocate timestamp, event, and
    # permutation workspaces larger than each yielded transport chunk.
    provider_workspace_rows: int = 1_048_576
    # When true, reject backends before invocation unless they declare
    # ``supports_streaming=True``.
    strict_streaming: bool = False

    # Registered backend name; ``noop`` supports explicit zero-event operation.
    backend: str = "v2e"

    # Torch providers use CUDA when available, otherwise CPU, if unset.
    backend_device: str | None = None
    # Required by ``v2ce``; fetch with ``srb event-camera fetch-weights``.
    v2ce_model_path: str | None = None

    # ``None`` uses the run seed; reset seeds also mix in env index and episode.
    rng_seed: int | None = None

    # ClassVar keeps the category out of user-facing dataclass fields.
    category: ClassVar[SensorCategory] = SensorCategory.EVENT_CAMERA

    def __post_init__(self) -> None:
        if self.representation not in _EVENT_REPRESENTATIONS:
            raise ValueError(
                f"representation must be one of {_EVENT_REPRESENTATIONS}; "
                f"got {self.representation!r}"
            )
        for field_name in ("time_bins", "downsample"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(
                    f"{field_name} must be a positive integer; got {value!r}"
                )
            if value < 1:
                raise ValueError(f"{field_name} must be >= 1; got {value}")
        if isinstance(self.raw_max_events, bool) or not isinstance(
            self.raw_max_events, int
        ):
            raise ValueError(
                "raw_max_events must be a non-negative integer; "
                f"got {self.raw_max_events!r}"
            )
        if not isinstance(self.strict_streaming, bool):
            raise ValueError(
                f"strict_streaming must be bool; got {self.strict_streaming!r}"
            )
        if isinstance(self.hot_pixel_rate_hz, bool) or not isinstance(
            self.hot_pixel_rate_hz, (int, float)
        ):
            raise ValueError(
                f"hot_pixel_rate_hz must be a number; got {self.hot_pixel_rate_hz!r}"
            )
        if not math.isfinite(self.hot_pixel_rate_hz) or self.hot_pixel_rate_hz < 0:
            raise ValueError(
                "hot_pixel_rate_hz must be finite and >= 0; "
                f"got {self.hot_pixel_rate_hz!r}"
            )
        if not math.isfinite(self.sub_render_hz) or self.sub_render_hz <= 0.0:
            raise ValueError(
                f"sub_render_hz must be finite and > 0; got {self.sub_render_hz}"
            )
        if not math.isfinite(self.time_surface_tau_s) or self.time_surface_tau_s <= 0.0:
            raise ValueError(
                "time_surface_tau_s must be finite and > 0; "
                f"got {self.time_surface_tau_s}"
            )
        if self.raw_max_events < 0:
            raise ValueError(f"raw_max_events must be >= 0; got {self.raw_max_events}")
        w, h = self.resolution
        # Pixel dimensions feed the native camera width/height verbatim.
        _require_positive_int("resolution width", w)
        _require_positive_int("resolution height", h)
        if self.roi_crop is not None:
            rx, ry, rw, rh = self.roi_crop
            for field_name, value in zip(
                ("roi_crop x", "roi_crop y", "roi_crop width", "roi_crop height"),
                self.roi_crop,
                strict=True,
            ):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError(f"{field_name} must be an integer; got {value!r}")
            if rx < 0 or ry < 0 or rw <= 0 or rh <= 0:
                raise ValueError(
                    f"roi_crop needs x/y >= 0 and width/height > 0; got {self.roi_crop}"
                )
            if rx + rw > w or ry + rh > h:
                raise ValueError(
                    f"roi_crop {self.roi_crop} exceeds resolution {self.resolution}"
                )

        # Prevent a zero-sized effective sensor after integer downsampling.
        _eff_w, _eff_h = (
            (self.roi_crop[2], self.roi_crop[3])
            if self.roi_crop is not None
            else (w, h)
        )
        if self.downsample > min(_eff_w, _eff_h):
            raise ValueError(
                f"downsample {self.downsample} exceeds the min effective dimension "
                f"{min(_eff_w, _eff_h)} (resolution {self.resolution}, "
                f"roi_crop {self.roi_crop}); effective_resolution would floor to 0"
            )

        for field_name in ("event_chunk_rows", "provider_workspace_rows"):
            _require_positive_int(field_name, getattr(self, field_name))
        if self.event_chunk_rows > MAX_EVENT_BATCH_ROWS:
            raise ValueError(
                f"event_chunk_rows {self.event_chunk_rows} exceeds "
                f"MAX_EVENT_BATCH_ROWS {MAX_EVENT_BATCH_ROWS}; the recorder "
                "writes each transport chunk as one HDF5 batch"
            )
        if self.event_chunk_rows > self.provider_workspace_rows:
            raise ValueError(
                f"event_chunk_rows {self.event_chunk_rows} exceeds "
                f"provider_workspace_rows {self.provider_workspace_rows}; the "
                "transport chunk must fit the provider workspace"
            )
        if self.observation_window_events is not None:
            _require_positive_int(
                "observation_window_events", self.observation_window_events
            )

        self._apply_preset_defaults()
        for field_name in (
            "pos_thres",
            "neg_thres",
            "sigma_thres",
            "cutoff_hz",
            "leak_rate_hz",
            "shot_noise_rate_hz",
            "refractory_period_s",
            "hot_pixel_frac",
        ):
            value = getattr(self, field_name)
            if (
                isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0
                or (field_name == "hot_pixel_frac" and value > 1)
            ):
                raise ValueError(
                    f"{field_name} must be finite and non-negative; got {value!r}"
                )

    def _apply_preset_defaults(self) -> None:
        preset = get_preset(self.preset_name)
        for fname in (
            "pos_thres",
            "neg_thres",
            "sigma_thres",
            "cutoff_hz",
            "leak_rate_hz",
            "shot_noise_rate_hz",
            "refractory_period_s",
            "hot_pixel_frac",
        ):
            if getattr(self, fname) is None:
                setattr(self, fname, getattr(preset, fname))

    @property
    def preset(self) -> IMX636Preset:
        return get_preset(self.preset_name)

    @property
    def effective_resolution(self) -> tuple[int, int]:
        """Resolution after ROI crop and integer downsampling."""
        if self.roi_crop is None:
            w, h = self.resolution
        else:
            _, _, w, h = self.roi_crop
        return (w // self.downsample, h // self.downsample)

    def resolved_observation_window_events(self) -> int:
        """Return the current-geometry default or validated explicit row bound."""
        if self.observation_window_events is None:
            eff_w, eff_h = self.effective_resolution
            return DEFAULT_OBSERVATION_WINDOW_EVENTS_PER_PIXEL * eff_w * eff_h
        return _require_positive_int(
            "observation_window_events", self.observation_window_events
        )

    @property
    def base_seed(self) -> int:
        """Config view: configured seed or zero; runtime seeding uses run seed."""
        return self.rng_seed if self.rng_seed is not None else 0

    def resolve_base_seed(self, run_seed: int) -> int:
        """Resolve the configured seed against the run seed."""
        return int(self.rng_seed) if self.rng_seed is not None else int(run_seed)


def _require_positive_int(field_name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field_name} must be a positive integer; got {value!r}")
    if value < 1:
        raise ValueError(f"{field_name} must be >= 1; got {value}")
    return value


# Rotation from Isaac Lab's "world" camera convention (+X forward, +Z up) to
# its "ros" convention (+Z forward, -Y up), scalar-last.
_WORLD_TO_ROS_CAMERA_XYZW = (-0.5, 0.5, -0.5, 0.5)


def ros_rot_from_world_convention(
    rot_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Compose a world-convention frame-camera rotation with the ROS basis."""
    ax, ay, az, aw = rot_xyzw
    bx, by, bz, bw = _WORLD_TO_ROS_CAMERA_XYZW
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def build_event_camera_cfg(
    prim_path: str,
    pos: tuple[float, float, float],
    *,
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    preset_name: str = "imx636_nominal",
    backend: str = "v2e",
    sub_render_hz: float = 250.0,
    resolution: tuple[int, int] = (1280, 720),
) -> EventCameraCfg:
    """Build the shared task event-camera config from its attach point."""
    return EventCameraCfg(
        prim_path=prim_path,
        pos=pos,
        rot_xyzw=rot_xyzw,
        preset_name=preset_name,
        backend=backend,
        sub_render_hz=sub_render_hz,
        resolution=resolution,
    )


__all__ = [
    "DEFAULT_EVENT_CHUNK_ROWS",
    "DEFAULT_OBSERVATION_WINDOW_EVENTS_PER_PIXEL",
    "EVENT_ROW_BYTES",
    "EventCameraCfg",
    "MAX_HOT_PIXEL_EVENTS_PER_TICK",
    "build_event_camera_cfg",
    "ros_rot_from_world_convention",
]
