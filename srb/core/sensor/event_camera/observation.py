"""Build event observations and infer their shapes from the representation table.

Observation keys are ``event_<name>`` and, for raw events,
``event_<name>_count`` for the per-env count.
"""

from __future__ import annotations

from typing import Any

import torch

from srb.core.sensor.event_camera.config import EventCameraCfg
from srb.core.sensor.event_camera.dispatch import EventObservationOverflowError
from srb.core.sensor.event_camera.representations import get_representation


def representation_shape(cfg: EventCameraCfg) -> tuple[int, ...]:
    """Per-env shape of the representation tensor (excluding the env axis)."""
    return get_representation(cfg.representation).shape(cfg)


def _get_events_per_env(
    sensor: Any,
    num_envs: int,
    *,
    max_events: int,
    sensor_name: str,
) -> list[torch.Tensor]:
    """Pull per-env event tensors off the sensor; pad to num_envs with empties."""
    raw = getattr(sensor, "events_per_env", None)
    available = raw if raw is not None else ()
    device = next(
        (events.device for events in available if events is not None),
        getattr(sensor, "device", torch.device("cpu")),
    )
    empty = torch.zeros((0, 4), dtype=torch.int64, device=device)
    if raw is None:
        return [empty for _ in range(num_envs)]
    out: list[torch.Tensor] = []
    for i in range(num_envs):
        if i < len(raw) and raw[i] is not None:
            events = raw[i]
            rows = int(events.shape[0])
            if rows > max_events:
                raise EventObservationOverflowError(
                    key=(i, sensor_name),
                    accepted_rows=0,
                    incoming_rows=rows,
                    max_rows=max_events,
                )
            out.append(events)
        else:
            out.append(empty)
    return out


def build_event_observation(name: str, sensor: "object") -> dict[str, torch.Tensor]:
    """Build an obs-dict entry using the configured representation.

    Requires ``cfg`` and ``num_instances``; ``events_per_env`` is optional
    and defaults to correctly shaped zero tensors. Results are stacked along
    the env axis and keyed as ``event_<name>`` (plus raw event counts).
    """
    cfg = getattr(sensor, "cfg")
    if not isinstance(cfg, EventCameraCfg):
        raise TypeError(
            f"build_event_observation: sensor.cfg must be EventCameraCfg, "
            f"got {type(cfg).__name__}"
        )
    num_envs = int(getattr(sensor, "num_instances", 1))
    rep = get_representation(cfg.representation)
    events_per_env = _get_events_per_env(
        sensor,
        num_envs,
        max_events=cfg.resolved_observation_window_events(),
        sensor_name=name,
    )
    per_env_dicts = [rep.convert_one_env(ev, cfg) for ev in events_per_env]

    key = f"event_{name}"
    out: dict[str, torch.Tensor] = {}
    # Every row populates "main"; some (raw) additionally populate "count".
    out[key] = torch.stack([d["main"] for d in per_env_dicts], dim=0)
    if "count" in per_env_dicts[0]:
        out[f"{key}_count"] = torch.stack([d["count"] for d in per_env_dicts], dim=0)
    return out


__all__ = [
    "build_event_observation",
    "representation_shape",
]
