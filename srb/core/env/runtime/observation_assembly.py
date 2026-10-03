"""Assemble flat observations and Gym spaces from proprioceptive, visual, and event data."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import gymnasium
import torch
from gymnasium.vector.utils import batch_space

from srb._typing.step_return import StepReturn
from srb.core.env.runtime.observation_layout import (
    ObservationLayout,
    box_for_dtype_shape,
)
from srb.core.sensor.event_camera.observation import representation_shape
from srb.core.sensor.event_camera.recorder import iter_event_camera_sensors
from srb.core.sensor.visual_output import (
    VisualChannel,
    construct_visual_observation,
    has_visual_depth_stream,
    iter_visual_sensor_outputs,
    visual_channel_layout,
    visual_depth_observation_range,
)

if TYPE_CHECKING:
    from isaaclab.scene import InteractiveScene


@dataclass(frozen=True)
class VisualTerm:
    """Bound capture of a VISUAL-category sensor at assembly init time."""

    output_key: str
    sensor_name: str
    sensor: Any
    data_types: tuple[str, ...]
    clipping_range: tuple[float, float]
    num_envs: int
    shape: tuple[int, ...]
    dtype: torch.dtype
    # Dataset metadata; not used on the observation hot path.
    channels: tuple[VisualChannel, ...] = ()
    depth_range: tuple[float, float] | None = None
    segmentation_labels: Mapping[str, Any] | None = field(default=None, hash=False)
    segmentation_labels_truncated: bool = False

    def collect(self) -> dict[str, torch.Tensor]:
        captured = construct_visual_observation(
            image_basename=self.output_key,
            data_types=self.data_types,
            clipping_range=self.clipping_range,
            merge_channels=True,
            **self.sensor.data.output,
        )
        tensor = captured.get(self.output_key)
        if tensor is None:
            raise RuntimeError(
                f"VISUAL sensor output '{self.output_key}' disappeared during "
                "observation assembly"
            )
        _validate_term_tensor(
            term_kind="Visual",
            output_key=self.output_key,
            tensor=tensor,
            num_envs=self.num_envs,
            shape=self.shape,
            dtype=self.dtype,
        )
        return captured


@dataclass(frozen=True)
class EventTerm:
    """Event observation key and shape declared from its camera configuration."""

    output_key: str
    sensor_name: str
    shape: tuple[int, ...]
    dtype: torch.dtype


@dataclass(frozen=True)
class ObservationAssembly:
    """Frozen observation schema for proprioception, visual sensors, and events."""

    layout: ObservationLayout
    visual_terms: tuple[VisualTerm, ...]
    event_terms: tuple[EventTerm, ...]
    num_envs: int

    @classmethod
    def from_first_step(
        cls,
        *,
        scene: "InteractiveScene",
        step_return: StepReturn,
        num_envs: int,
    ) -> ObservationAssembly:
        layout = ObservationLayout.from_step_return(step_return, num_envs=num_envs)
        visual_terms = tuple(_discover_visual_terms(scene, num_envs=num_envs))
        event_terms = tuple(_discover_event_terms(scene))
        return cls(
            layout=layout,
            visual_terms=visual_terms,
            event_terms=event_terms,
            num_envs=num_envs,
        )

    def assemble(
        self,
        step_return: StepReturn,
        *,
        event_observations: dict[str, torch.Tensor] | None = None,
    ) -> dict[str, torch.Tensor]:
        flat = self.layout.flatten(step_return.observation)
        for term in self.visual_terms:
            captured = term.collect()
            overlap = flat.keys() & captured.keys()
            if overlap:
                raise ValueError(
                    f"Visual term '{term.output_key}' collides with proprio key "
                    f"{next(iter(overlap))!r}"
                )
            flat.update(captured)
        # Declared event terms are part of the Gym space, so a missing mapping
        # must fail key validation instead of silently dropping them.
        if event_observations is not None or self.event_terms:
            self._merge_event_observations(flat, event_observations or {})
        return flat

    def _merge_event_observations(
        self,
        flat: dict[str, torch.Tensor],
        events: dict[str, torch.Tensor],
    ) -> None:
        # Declared keys must match exactly: callers (EventRecorder) emit
        # every declared sensor every tick, including zero-event ones, so a
        # mismatch here means the obs-space contract has drifted.
        declared_terms = {term.output_key: term for term in self.event_terms}
        declared = set(declared_terms)
        provided = set(events.keys())
        missing = declared - provided
        if missing:
            raise ValueError(
                f"Event observations missing declared keys: {sorted(missing)}"
            )
        undeclared = provided - declared
        if undeclared:
            raise ValueError(
                f"Event observations contain undeclared keys: {sorted(undeclared)}"
            )
        overlap = flat.keys() & provided
        if overlap:
            raise ValueError(
                f"Event observation '{next(iter(overlap))}' collides with "
                "existing obs key"
            )
        for key, tensor in events.items():
            term = declared_terms[key]
            _validate_term_tensor(
                term_kind="Event",
                output_key=key,
                tensor=tensor,
                num_envs=self.num_envs,
                shape=term.shape,
                dtype=term.dtype,
            )
        flat.update(events)

    def build_spaces(self) -> tuple[gymnasium.spaces.Dict, gymnasium.Space]:
        proprio_single, _ = self.layout.build_spaces()
        spaces: dict[str, gymnasium.spaces.Box] = dict(proprio_single.spaces)
        for term in self.visual_terms:
            if term.output_key in spaces:
                raise ValueError(
                    f"Visual term '{term.output_key}' collides with proprio space"
                )
            spaces[term.output_key] = box_for_dtype_shape(
                key=term.output_key,
                dtype=term.dtype,
                shape=term.shape,
            )
        for term in self.event_terms:
            if term.output_key in spaces:
                raise ValueError(
                    f"Event term '{term.output_key}' collides with existing space"
                )
            spaces[term.output_key] = box_for_dtype_shape(
                key=term.output_key,
                dtype=term.dtype,
                shape=term.shape,
            )
        single = gymnasium.spaces.Dict(spaces)
        return single, batch_space(single, n=self.num_envs)


def _discover_visual_terms(
    scene: "InteractiveScene",
    *,
    num_envs: int,
) -> Iterator[VisualTerm]:
    for visual in iter_visual_sensor_outputs(scene):
        images = construct_visual_observation(
            image_basename=visual.image_basename,
            data_types=visual.data_types,
            clipping_range=visual.clipping_range,
            merge_channels=False,
            **visual.sensor.data.output,
        )
        if not images:
            raise RuntimeError(
                f"VISUAL sensor '{visual.sensor_name}' produced no output for key "
                f"'{visual.image_basename}' on first sample"
            )
        # Same concatenation as ``construct_visual_observation(merge_channels=
        # True)``; the per-type tensors also give the dataset channel layout.
        tensor = torch.cat(list(images.values()), dim=-1)
        channels = visual_channel_layout(
            images,
            image_basename=visual.image_basename,
            data_types=visual.data_types,
        )
        segmentation_labels, labels_truncated = _segmentation_labels(
            visual.sensor, channels
        )
        shape = tuple(tensor.shape[1:])
        dtype = tensor.dtype
        _validate_term_tensor(
            term_kind="Visual",
            output_key=visual.image_basename,
            tensor=tensor,
            num_envs=num_envs,
            shape=shape,
            dtype=dtype,
        )
        yield VisualTerm(
            output_key=visual.image_basename,
            sensor_name=visual.sensor_name,
            sensor=visual.sensor,
            data_types=visual.data_types,
            clipping_range=visual.clipping_range,
            num_envs=num_envs,
            shape=shape,
            dtype=dtype,
            channels=channels,
            depth_range=(
                visual_depth_observation_range(visual.clipping_range)
                if has_visual_depth_stream(visual.data_types)
                else None
            ),
            segmentation_labels=segmentation_labels,
            segmentation_labels_truncated=labels_truncated,
        )


# Bound on recorded segmentation label entries per data type.
_MAX_SEGMENTATION_LABELS = 1024


def _segmentation_labels(
    sensor: Any, channels: tuple[VisualChannel, ...]
) -> tuple[dict[str, dict[str, str]] | None, bool]:
    """Snapshot the sensor's ``idToLabels`` map per segmentation data type."""
    segmentation_types = [
        channel.data_type
        for channel in channels
        if channel.encoding.startswith("segmentation_")
    ]
    if not segmentation_types:
        return None, False
    info = getattr(getattr(sensor, "data", None), "info", None)
    # Isaac Lab 3.0 keys ``info`` by data type; older releases keep one such
    # mapping per environment.
    if isinstance(info, (list, tuple)):
        info = info[0] if info else None
    labels: dict[str, dict[str, str]] = {}
    truncated = False
    for data_type in segmentation_types:
        entry = info.get(data_type) if isinstance(info, Mapping) else None
        id_to_labels = entry.get("idToLabels") if isinstance(entry, Mapping) else None
        if not isinstance(id_to_labels, Mapping):
            continue
        items = sorted(id_to_labels.items(), key=lambda item: str(item[0]))
        truncated |= len(items) > _MAX_SEGMENTATION_LABELS
        labels[data_type] = {
            str(key): _label_text(value)
            for key, value in items[:_MAX_SEGMENTATION_LABELS]
        }
    return labels, truncated


def _label_text(value: Any) -> str:
    if isinstance(value, Mapping):
        return ",".join(f"{key}:{value[key]}" for key in sorted(value, key=str))
    return str(value)


def _validate_term_tensor(
    *,
    term_kind: str,
    output_key: str,
    tensor: torch.Tensor,
    num_envs: int,
    shape: tuple[int, ...],
    dtype: torch.dtype,
) -> None:
    expected_shape = (num_envs, *shape)
    if tuple(tensor.shape) != expected_shape:
        raise ValueError(
            f"{term_kind} term '{output_key}' shape must be {expected_shape}, "
            f"got {tuple(tensor.shape)}"
        )
    if tensor.dtype != dtype:
        raise TypeError(
            f"{term_kind} term '{output_key}' dtype must be {dtype}, got {tensor.dtype}"
        )


def _discover_event_terms(scene: "InteractiveScene") -> Iterator[EventTerm]:
    for ref in iter_event_camera_sensors(scene):
        cfg = ref.cfg
        shape = representation_shape(cfg)
        is_raw = cfg.representation == "raw"
        main_dtype = torch.int64 if is_raw else torch.float32
        yield EventTerm(
            output_key=f"event_{ref.name}",
            sensor_name=ref.name,
            shape=shape,
            dtype=main_dtype,
        )
        if is_raw:
            yield EventTerm(
                output_key=f"event_{ref.name}_count",
                sensor_name=ref.name,
                shape=(),
                dtype=torch.int64,
            )


def discover_event_terms(scene: "InteractiveScene") -> tuple[EventTerm, ...]:
    """Return the declared event observation terms for a scene.

    Legacy DirectEnv tasks build spaces by probing observations before the
    live event dispatcher is initialized. They use this pure declaration seam
    to keep their space contract aligned with the owned ObservationAssembly.
    """
    return tuple(_discover_event_terms(scene))


__all__ = [
    "EventTerm",
    "ObservationAssembly",
    "VisualTerm",
    "discover_event_terms",
]
