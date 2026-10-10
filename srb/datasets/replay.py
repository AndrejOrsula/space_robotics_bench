"""Replay and visualization helpers for canonical SRB datasets."""

from __future__ import annotations

import importlib
import importlib.util
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from srb.datasets.formats import get_dataset_format_adapter
from srb.datasets.jsonl import flat_json_value_len
from srb.datasets.manifest import (
    DatasetManifest,
    DatasetSnapshot,
    open_dataset_snapshot,
)
from srb.datasets.schema import StorageFormat
from srb.datasets.visual_contract import (
    VisualImageView,
    depth_metres,
    view_pixels,
    visual_image_views,
)
from srb.utils.json_types import thaw_json_value
from srb.utils.resource_budget import (
    MAX_METADATA_AGGREGATE_BYTES,
    ByteBudget,
)

_REPLAY_METADATA_ITEM_OVERHEAD = 64


class _ReplayBookkeeping:
    """Charge summary state before retaining it in Python containers."""

    def __init__(self) -> None:
        self._budget = ByteBudget(
            "replay metadata bookkeeping",
            MAX_METADATA_AGGREGATE_BYTES,
        )

    def retain_integer(
        self, values: dict[int, bool], value: int, success: bool
    ) -> bool:
        """Retain one episode flag and return whether it was newly added."""

        if value in values:
            return False
        self._consume_integer(value)
        values[value] = success
        return True

    def retain_string(self, values: set[str], value: str) -> None:
        if value in values:
            return
        self._consume_text(value)
        values.add(value)

    def copy_action_schema(self, schema: object) -> list[dict[str, Any]]:
        if not isinstance(schema, (list, tuple)):
            return []
        components: list[dict[str, Any]] = []
        for component in schema:
            if not isinstance(component, Mapping):
                continue
            self._charge_json(component)
            copied = self._copy_json(component)
            assert isinstance(copied, dict)
            components.append(copied)
        return components

    def _consume_text(self, value: str) -> None:
        overhead = _REPLAY_METADATA_ITEM_OVERHEAD
        remaining = max(0, self._budget.limit - self._budget.consumed)
        encoded_length = _utf8_length(
            value,
            limit=max(0, remaining - overhead),
        )
        self._budget.consume(encoded_length + overhead)

    def _consume_integer(self, value: int) -> None:
        self._budget.consume(
            _integer_payload_cost(value) + _REPLAY_METADATA_ITEM_OVERHEAD
        )

    def _charge_json(self, value: object) -> None:
        self._budget.consume(_REPLAY_METADATA_ITEM_OVERHEAD)
        if isinstance(value, str):
            remaining = max(0, self._budget.limit - self._budget.consumed)
            self._budget.consume(_utf8_length(value, limit=remaining))
        elif isinstance(value, bool) or value is None:
            return
        elif isinstance(value, int):
            self._budget.consume(_integer_payload_cost(value))
        elif isinstance(value, float):
            self._budget.consume(8)
        elif isinstance(value, Mapping):
            for key, item in value.items():
                if isinstance(key, str):
                    self._consume_text(key)
                else:
                    self._budget.consume(_REPLAY_METADATA_ITEM_OVERHEAD)
                self._charge_json(item)
        elif isinstance(value, (list, tuple)):
            for item in value:
                self._charge_json(item)

    @classmethod
    def _copy_json(cls, value: object) -> object:
        if isinstance(value, Mapping):
            return {str(key): cls._copy_json(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._copy_json(item) for item in value]
        return value


def _integer_payload_cost(value: int) -> int:
    """Charge an integer by magnitude before retaining its Python object."""

    return max(1, value.bit_length())


def _utf8_length(value: str, *, limit: int | None = None) -> int:
    """Count UTF-8 bytes without an encoded copy; stop once ``limit`` is exceeded.

    Match the readers' ``surrogatepass`` policy: lone surrogates count as
    three bytes.
    """

    if value.isascii():
        return len(value)
    encoded_length = 0
    for character in value:
        codepoint = ord(character)
        if codepoint <= 0x7F:
            encoded_length += 1
        elif codepoint <= 0x7FF:
            encoded_length += 2
        elif codepoint <= 0xFFFF:
            encoded_length += 3
        else:
            encoded_length += 4
        if limit is not None and encoded_length > limit:
            return encoded_length
    return encoded_length


@dataclass(frozen=True)
class DatasetReplaySummary:
    """Summary produced while replaying or previewing a dataset."""

    manifest_path: Path
    storage_format: StorageFormat
    num_episodes: int
    total_steps: int
    obs_keys: list[str]
    action_dim: int | None
    action_schema: list[dict[str, Any]] | None
    reward_sum: float
    success_count: int
    first_episode_index: int | None = None
    last_episode_index: int | None = None
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "manifest_path": str(self.manifest_path),
            "storage_format": self.storage_format.value,
            "num_episodes": self.num_episodes,
            "total_steps": self.total_steps,
            "obs_keys": self.obs_keys,
            "action_dim": self.action_dim,
            "action_schema": self.action_schema,
            "reward_sum": self.reward_sum,
            "success_count": self.success_count,
            "first_episode_index": self.first_episode_index,
            "last_episode_index": self.last_episode_index,
            "metadata": self.metadata,
        }


def summarize_dataset_replay(
    path: str | Path,
    *,
    max_episodes: int | None = None,
    max_steps: int | None = None,
) -> DatasetReplaySummary:
    """Read a canonical SRB dataset and return a replay summary."""
    with open_dataset_snapshot(path) as snapshot:
        # Pass the verified snapshot through the adapter seam below.
        manifest_path = snapshot.manifest_path
        source_manifest_path = snapshot.source_manifest_path
        manifest = snapshot.manifest
        if manifest_path is None:
            raise ValueError("dataset replay requires a manifest path")
        if source_manifest_path is None:
            raise ValueError("dataset replay requires a source manifest path")
        return _summarize_snapshot(
            snapshot,
            manifest=manifest,
            manifest_path=manifest_path,
            source_manifest_path=source_manifest_path,
            max_episodes=max_episodes,
            max_steps=max_steps,
        )


def _summarize_snapshot(
    snapshot: DatasetSnapshot,
    *,
    manifest: DatasetManifest,
    manifest_path: Path,
    source_manifest_path: Path,
    max_episodes: int | None,
    max_steps: int | None,
) -> DatasetReplaySummary:
    """Summarize rows while the verified source snapshot remains owned."""

    bookkeeping = _ReplayBookkeeping()
    obs_keys: set[str] = set()
    episode_success: dict[int, bool] = {}
    action_dim: int | None = None
    reward_sum = 0.0
    total_steps = 0
    first_episode_index: int | None = None
    last_episode_index: int | None = None
    success_count = 0

    for row in _iter_manifest_steps(
        manifest_path=manifest_path,
        max_episodes=max_episodes,
        max_steps=max_steps,
        snapshot=snapshot,
    ):
        episode_index = int(row.get("episode_index", 0))
        row_success = bool(row.get("success", False))
        is_new_episode = bookkeeping.retain_integer(
            episode_success,
            episode_index,
            row_success,
        )
        if is_new_episode:
            if row_success:
                success_count += 1
            if first_episode_index is None or episode_index < first_episode_index:
                first_episode_index = episode_index
            if last_episode_index is None or episode_index > last_episode_index:
                last_episode_index = episode_index
        elif row_success and not episode_success[episode_index]:
            episode_success[episode_index] = True
            success_count += 1
        obs = row.get("obs") or {}
        for key in obs:
            bookkeeping.retain_string(obs_keys, str(key))
        row_action_dim = flat_json_value_len(row.get("action"))
        if action_dim is None:
            action_dim = row_action_dim
        elif row_action_dim != action_dim:
            raise ValueError(
                f"action dim {row_action_dim} does not match first action dim "
                f"{action_dim}"
            )
        reward_sum += float(row.get("reward", 0.0) or 0.0)
        total_steps += 1

    return DatasetReplaySummary(
        manifest_path=source_manifest_path,
        storage_format=manifest.storage_format,
        num_episodes=len(episode_success),
        total_steps=total_steps,
        obs_keys=sorted(obs_keys),
        action_dim=action_dim,
        action_schema=_manifest_action_schema(manifest, bookkeeping=bookkeeping),
        reward_sum=reward_sum,
        success_count=success_count,
        first_episode_index=first_episode_index,
        last_episode_index=last_episode_index,
        metadata={
            "task": manifest.identity.task,
            "embodiment": manifest.identity.embodiment,
            "action_mode": manifest.identity.action_mode,
            "split": manifest.split.value,
            "controller": manifest.controller,
        },
    )


def replay_dataset(
    path: str | Path,
    *,
    backend: str = "summary",
    max_episodes: int | None = None,
    max_steps: int | None = None,
    save_path: str | Path | None = None,
    spawn: bool = False,
) -> DatasetReplaySummary:
    """Replay a canonical SRB dataset with the requested backend."""
    if backend == "summary":
        return summarize_dataset_replay(
            path,
            max_episodes=max_episodes,
            max_steps=max_steps,
        )
    if backend == "rerun":
        return _replay_dataset_to_rerun(
            path,
            max_episodes=max_episodes,
            max_steps=max_steps,
            save_path=save_path,
            spawn=spawn,
        )
    raise ValueError(f"unsupported replay backend: {backend!r}")


def iter_dataset_steps(
    path: str | Path,
    *,
    max_episodes: int | None = None,
    max_steps: int | None = None,
) -> Iterator[dict[str, Any]]:
    """Yield canonical replay rows through the dataset format adapter."""
    with open_dataset_snapshot(path) as snapshot:
        manifest_path = snapshot.manifest_path
        if manifest_path is None:
            raise ValueError("dataset replay requires a manifest path")
        yield from _iter_manifest_steps(
            manifest_path=manifest_path,
            max_episodes=max_episodes,
            max_steps=max_steps,
            snapshot=snapshot,
        )


def _iter_manifest_steps(
    *,
    manifest_path: Path,
    max_episodes: int | None = None,
    max_steps: int | None = None,
    snapshot: DatasetSnapshot | None = None,
) -> Iterator[dict[str, Any]]:
    # Select the format adapter for the verified snapshot before reading rows.
    if snapshot is not None:
        yield from _iter_snapshot_steps(
            snapshot,
            max_episodes=max_episodes,
            max_steps=max_steps,
        )
        return

    with open_dataset_snapshot(manifest_path) as owned_snapshot:
        yield from _iter_snapshot_steps(
            owned_snapshot,
            max_episodes=max_episodes,
            max_steps=max_steps,
        )


def _iter_snapshot_steps(
    snapshot: DatasetSnapshot,
    *,
    max_episodes: int | None,
    max_steps: int | None,
) -> Iterator[dict[str, Any]]:
    """Yield rows from one verified snapshot without reopening its source path."""

    manifest = snapshot.manifest
    manifest_path = snapshot.manifest_path
    if manifest_path is None:
        raise ValueError("dataset replay requires a manifest path")
    data_path = snapshot.dataset_path
    adapter = get_dataset_format_adapter(manifest.storage_format)
    yield from adapter.iter_steps(
        manifest=manifest,
        manifest_path=manifest_path,
        dataset_path=data_path,
        max_episodes=max_episodes,
        max_steps=max_steps,
    )


def _replay_dataset_to_rerun(
    path: str | Path,
    *,
    max_episodes: int | None,
    max_steps: int | None,
    save_path: str | Path | None,
    spawn: bool,
) -> DatasetReplaySummary:
    if importlib.util.find_spec("rerun") is None:
        raise RuntimeError(
            "Rerun replay requires rerun-sdk. Install rerun-sdk or use "
            "--backend summary."
        )

    rr = importlib.import_module("rerun")
    _init_rerun(rr, save_path=save_path, spawn=spawn)
    with open_dataset_snapshot(path) as snapshot:
        manifest_path = snapshot.manifest_path
        if manifest_path is None:
            raise ValueError("dataset replay requires a manifest path")
        source_manifest_path = snapshot.source_manifest_path
        if source_manifest_path is None:
            raise ValueError("dataset replay requires a source manifest path")
        manifest = snapshot.manifest
        action_schema = _manifest_action_schema(manifest)
        image_views = visual_image_views(
            thaw_json_value(manifest.metadata.get("demo_io_contract"))
        )

        for row in _iter_manifest_steps(
            manifest_path=manifest_path,
            max_episodes=max_episodes,
            max_steps=max_steps,
            snapshot=snapshot,
        ):
            _set_rerun_time(rr, row)
            _log_rerun_row(
                rr, row, action_schema=action_schema, image_views=image_views
            )

        return _summarize_snapshot(
            snapshot,
            manifest=manifest,
            manifest_path=manifest_path,
            source_manifest_path=source_manifest_path,
            max_episodes=max_episodes,
            max_steps=max_steps,
        )


def _manifest_action_schema(
    manifest: DatasetManifest,
    *,
    bookkeeping: _ReplayBookkeeping | None = None,
) -> list[dict[str, Any]] | None:
    # Replay is an external presentation seam, so copy the manifest's frozen
    # metadata rather than leaking recursively immutable value objects.
    schema = manifest.metadata.get("action_schema")
    if not isinstance(schema, (list, tuple)):
        return None
    owner = _ReplayBookkeeping() if bookkeeping is None else bookkeeping
    return owner.copy_action_schema(schema)


def _init_rerun(rr: Any, *, save_path: str | Path | None, spawn: bool) -> None:
    rr.init("srb_dataset_replay", spawn=spawn)
    if save_path is not None and hasattr(rr, "save"):
        rr.save(str(save_path))


def _set_rerun_time(rr: Any, row: dict[str, Any]) -> None:
    episode_index = int(row.get("episode_index", 0))
    step_index = int(row.get("step_index", 0))
    if hasattr(rr, "set_time"):
        rr.set_time("episode", sequence=episode_index)
        rr.set_time("step", sequence=step_index)
    elif hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("episode", episode_index)
        rr.set_time_sequence("step", step_index)


def _log_rerun_row(
    rr: Any,
    row: dict[str, Any],
    *,
    action_schema: list[dict[str, Any]] | None = None,
    image_views: Sequence[VisualImageView] = (),
) -> None:
    _log_rerun_scalar(rr, "metrics/reward", row.get("reward", 0.0))
    action = row.get("action")
    _log_rerun_values(
        rr,
        "actions",
        {"values": action, "done": bool(row.get("done", False))},
    )
    for name, value in _action_component_values(action, action_schema):
        _log_rerun_values(rr, f"actions/{name}", {"values": value})
    observations = row.get("obs") or {}
    covered = _log_rerun_image_views(rr, observations, image_views)
    _log_rerun_observations(rr, observations, path=(), covered=covered)


def _log_rerun_image_views(
    rr: Any,
    observations: Mapping[str, Any],
    image_views: Sequence[VisualImageView],
) -> set[tuple[str, ...]]:
    """Log camera views and return fully covered observation leaf paths."""
    if not image_views or not hasattr(rr, "Image") or not hasattr(rr, "log"):
        return set()
    covered_channels: dict[tuple[str, ...], set[int]] = {}
    widths: dict[tuple[str, ...], int] = {}
    for view in image_views:
        value = _observation_at_path(observations, view.source_path)
        if value is None:
            continue
        source_path = view.source_path
        resource = "observations/" + "/".join(source_path)
        pixels = view_pixels(value, view, resource=resource)
        path = f"{resource}/{view.data_type}"
        if view.depth_range_m is not None and hasattr(rr, "DepthImage"):
            rr.log(
                path,
                rr.DepthImage(
                    depth_metres(pixels[:, :, 0], view.depth_range_m), meter=1.0
                ),
            )
        else:
            rr.log(path, rr.Image(pixels))
        covered_channels.setdefault(source_path, set()).update(
            range(view.start, view.end)
        )
        widths[source_path] = int(np.shape(value)[-1])
    return {
        path
        for path, channels in covered_channels.items()
        if channels == set(range(widths[path]))
    }


def _observation_at_path(
    observations: Mapping[str, Any], path: tuple[str, ...]
) -> Any | None:
    value: Any = observations
    for component in path:
        if not isinstance(value, Mapping) or component not in value:
            return None
        value = value[component]
    return value


def _log_rerun_observations(
    rr: Any,
    observations: Mapping[str, Any],
    *,
    path: tuple[str, ...],
    covered: set[tuple[str, ...]],
) -> None:
    for key, value in observations.items():
        current_path = (*path, str(key))
        if current_path in covered:
            continue
        if isinstance(value, Mapping):
            _log_rerun_observations(rr, value, path=current_path, covered=covered)
            continue
        entity_path = "observations/" + "/".join(current_path)
        if _looks_like_tensor(value):
            _log_rerun_tensor(rr, entity_path, value)
        else:
            _log_rerun_values(rr, entity_path, {"values": value})


def _action_component_values(
    action: Any,
    action_schema: list[dict[str, Any]] | None,
) -> list[tuple[str, Any]]:
    if not action_schema:
        return []
    flat = _flatten_values(action)
    if flat is None:
        return []
    components: list[tuple[str, Any]] = []
    for index, component in enumerate(action_schema):
        try:
            start = int(component["start"])
            end = int(component["end"])
        except (KeyError, TypeError, ValueError):
            continue
        if start < 0 or end <= start or end > len(flat):
            continue
        name = _action_component_name(component, index)
        values = flat[start:end]
        components.append((name, values[0] if len(values) == 1 else values))
    return components


def _flatten_values(value: Any) -> list[Any] | None:
    if isinstance(value, (int, float, bool)):
        return [value]
    if isinstance(value, list):
        flat: list[Any] = []
        for item in value:
            item_flat = _flatten_values(item)
            if item_flat is None:
                return None
            flat.extend(item_flat)
        return flat
    return None


def _action_component_name(component: dict[str, Any], index: int) -> str:
    name = component.get("name")
    if isinstance(name, str) and name:
        return name
    path = component.get("path")
    if isinstance(path, list) and path:
        return "/".join(str(part) for part in path)
    return f"component_{index}"


def _log_rerun_scalar(rr: Any, path: str, value: Any) -> None:
    scalar = float(value or 0.0)
    if hasattr(rr, "Scalars") and hasattr(rr, "log"):
        rr.log(path, rr.Scalars(scalar))
    elif hasattr(rr, "log_scalar"):
        rr.log_scalar(path, scalar)


def _log_rerun_tensor(rr: Any, path: str, value: Any) -> None:
    if hasattr(rr, "Tensor") and hasattr(rr, "log"):
        rr.log(path, rr.Tensor(value))
    else:
        _log_rerun_values(rr, path, {"values": value})


def _log_rerun_values(rr: Any, path: str, values: dict[str, Any]) -> None:
    if hasattr(rr, "AnyValues") and hasattr(rr, "log"):
        rr.log(path, rr.AnyValues(**values))


def _looks_like_tensor(value: Any) -> bool:
    return isinstance(value, list) and bool(value) and isinstance(value[0], list)
