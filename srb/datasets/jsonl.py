"""Canonical SRB JSONL row interface.

Missing ``format_version`` marks legacy rows; readers reject unknown versions.
"""

from __future__ import annotations

import heapq
import io
import json
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass
from json import JSONDecodeError
from numbers import Integral
from pathlib import Path
from typing import IO, Any, NotRequired, TypedDict, cast

import numpy as np

from srb.datasets.metadata_validation import is_int_value as _is_int_value
from srb.datasets.metadata_validation import is_number_value as _is_number_value
from srb.utils.json_types import (
    JsonDecodeBudget,
    JSONObject,
    JSONValue,
    validate_json_object,
    validate_json_value,
)
from srb.utils.resource_budget import (
    MAX_DECODED_AGGREGATE_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    MAX_JSON_DEPTH,
    MAX_JSON_NODES,
    MAX_JSONL_ROW_NODES,
    MAX_METADATA_AGGREGATE_BYTES,
    ByteBudget,
    ResourceLimitError,
    require_bounded_int,
)

#: Current row layout version. Bump when fields or meanings change; readers
#: must reject unknown versions.
SRB_JSONL_VERSION = "0.1"
MAX_JSONL_LINE_BYTES = 16 * (1 << 20)
MAX_JSONL_NESTING = MAX_JSON_DEPTH
MAX_JSONL_NODES = MAX_JSONL_ROW_NODES
_JSONL_METADATA_ITEM_OVERHEAD = 64
_JSONL_METADATA_FAILURE_RESERVE = 256
_JSONL_MISMATCH_EXAMPLES = 8
_JSONL_DIAGNOSTIC_VALUE_CHARS = 256


class SrbJsonlRow(TypedDict):
    """One validated, JSON-ready canonical SRB dataset row."""

    episode_index: int
    step_index: int
    obs: JSONObject
    action: JSONValue
    reward: float
    done: bool
    success: bool
    terminated: NotRequired[bool]
    truncated: NotRequired[bool]
    episode_key: NotRequired[str]
    next_obs: NotRequired[JSONObject]
    info: NotRequired[JSONObject]
    format_version: NotRequired[str]


SrbJsonlEpisodes = list[list[SrbJsonlRow]]


@dataclass(frozen=True, slots=True)
class SrbJsonlRowLocation:
    """One validated row and its byte location in an owned JSONL source."""

    path: Path
    offset: int
    length: int
    line_number: int
    row: SrbJsonlRow

    @property
    def line(self) -> int:
        """Return the one-based physical source line number."""

        return self.line_number


class _JsonlBookkeepingExhausted(Exception):
    """Internal control flow for a bounded validation report."""

    def __init__(self, error: ResourceLimitError) -> None:
        super().__init__(str(error))
        self.error = error


class _JsonlBookkeeping:
    """Charge retained validation state before adding it to a container."""

    def __init__(self) -> None:
        self._limit = MAX_METADATA_AGGREGATE_BYTES
        self._budget = ByteBudget("JSONL metadata bookkeeping", self._limit)
        self._retention_limit = max(
            0,
            self._limit - _JSONL_METADATA_FAILURE_RESERVE,
        )

    def retain_integer(self, values: set[int], value: int) -> None:
        if value in values:
            return
        self._consume(_integer_cost(value))
        values.add(value)

    def retain_string(self, values: set[str], value: str) -> None:
        if value in values:
            return
        self._consume(_text_cost(value))
        values.add(value)

    def retain_mapping_string(
        self,
        values: dict[int, str],
        key: int,
        value: str,
    ) -> None:
        if key in values:
            return
        self._consume(_integer_cost(key) + _text_cost(value))
        values[key] = value

    def retain_mapping_integer(
        self,
        values: dict[int, int],
        key: int,
        value: int,
    ) -> None:
        if key in values:
            previous_cost = _integer_cost(values[key])
            next_cost = _integer_cost(value)
            if next_cost > previous_cost:
                self._consume(next_cost - previous_cost)
            values[key] = value
            return
        self._consume(_integer_cost(key) + _integer_cost(value))
        values[key] = value

    def append_diagnostic(self, values: list[str], message: str) -> None:
        self._consume(_text_cost(message))
        values.append(message)

    def append_exhaustion(
        self,
        values: list[str],
        error: ResourceLimitError,
    ) -> None:
        message = str(error)
        cost = _text_cost(message)
        if self._budget.consumed + cost <= self._limit:
            self._budget.consume(cost)
        values.append(message)

    def _consume(self, amount: int) -> None:
        observed = self._budget.consumed + amount
        if observed > self._retention_limit:
            raise _JsonlBookkeepingExhausted(
                ResourceLimitError(
                    "JSONL metadata bookkeeping",
                    limit=self._limit,
                    observed=observed,
                )
            )
        self._budget.consume(amount)


class _JsonlEpisodeSelector:
    """Track logical episode identity while retaining only bounded state."""

    def __init__(
        self,
        *,
        max_episodes: int | None,
        bookkeeping: _JsonlBookkeeping,
    ) -> None:
        self._max_episodes = max_episodes
        self._bookkeeping = bookkeeping
        self._selected_ids: set[int] = set()
        self._episode_keys: dict[int, str] = {}

    @property
    def episode_count(self) -> int:
        """Return the number of retained logical episode identifiers."""

        return len(self._selected_ids)

    @property
    def selected_ids(self) -> set[int]:
        """Return retained identifiers for bounded diagnostics."""

        return self._selected_ids

    def observe(self, row: SrbJsonlRow, *, context: str) -> bool:
        """Record one row's identity and whether it belongs to the selection."""

        episode_index = row["episode_index"]
        if episode_index not in self._selected_ids and (
            self._max_episodes is None or len(self._selected_ids) < self._max_episodes
        ):
            self._bookkeeping.retain_integer(self._selected_ids, episode_index)

        if "episode_key" in row:
            episode_key = row["episode_key"]
            previous_key = self._episode_keys.get(episode_index)
            if previous_key is None:
                self._bookkeeping.retain_mapping_string(
                    self._episode_keys,
                    episode_index,
                    episode_key,
                )
            elif previous_key != episode_key:
                raise ValueError(
                    f"{context} episode_index {_bounded_repr(episode_index)} "
                    "has conflicting "
                    "episode_key values "
                    f"{_bounded_repr(previous_key)} and {_bounded_repr(episode_key)}"
                )

        return episode_index in self._selected_ids


REQUIRED_SRB_JSONL_KEYS: tuple[str, ...] = (
    "episode_index",
    "step_index",
    "obs",
    "action",
    "reward",
    "done",
    "success",
)
_OPTIONAL_SRB_JSONL_KEYS = frozenset(
    {"episode_key", "next_obs", "info", "terminated", "truncated", "format_version"}
)
_ALLOWED_SRB_JSONL_KEYS = frozenset(
    {*REQUIRED_SRB_JSONL_KEYS, *_OPTIONAL_SRB_JSONL_KEYS}
)


def build_srb_jsonl_row(
    *,
    episode_index: int,
    step_index: int,
    obs: Mapping[str, object],
    action: object,
    reward: object,
    done: bool,
    success: bool,
    episode_key: str | None = None,
    next_obs: Mapping[str, object] | None = None,
    info: Mapping[str, object] | None = None,
    terminated: bool | None = None,
    truncated: bool | None = None,
) -> SrbJsonlRow:
    """Build and validate one JSON-ready canonical SRB dataset row."""

    if not isinstance(episode_index, Integral) or isinstance(episode_index, bool):
        raise TypeError("episode_index must be an integer")
    if not isinstance(step_index, Integral) or isinstance(step_index, bool):
        raise TypeError("step_index must be an integer")
    if not isinstance(done, bool):
        raise TypeError("done must be a boolean")
    if not isinstance(success, bool):
        raise TypeError("success must be a boolean")
    for name, flag in (("terminated", terminated), ("truncated", truncated)):
        if flag is not None and not isinstance(flag, bool):
            raise TypeError(f"{name} must be a boolean or None")
    if (terminated is None) != (truncated is None):
        raise ValueError("terminated and truncated must be supplied together")
    if terminated is not None and truncated is not None:
        if done != (terminated or truncated):
            raise ValueError("done must equal (terminated or truncated)")
    if episode_key is not None and not isinstance(episode_key, str):
        raise TypeError("episode_key must be a string or null")

    row: SrbJsonlRow = {
        "episode_index": int(episode_index),
        "step_index": int(step_index),
        "obs": _require_json_object(to_jsonable(dict(obs)), field_name="obs"),
        "action": to_jsonable(action),
        "reward": _require_reward(reward),
        "done": done,
        "success": success,
        "format_version": SRB_JSONL_VERSION,
    }
    if terminated is not None and truncated is not None:
        # Preserve both flags because legacy ``done`` combines them.
        row["terminated"] = terminated
        row["truncated"] = truncated
    if episode_key is not None:
        row["episode_key"] = episode_key
    if next_obs is not None:
        row["next_obs"] = _require_json_object(
            to_jsonable(dict(next_obs)), field_name="next_obs"
        )
    if info is not None:
        row["info"] = _require_json_object(to_jsonable(dict(info)), field_name="info")
    return validate_srb_jsonl_row(row)


def write_srb_jsonl_row(fh: IO[str], row: Mapping[str, object]) -> None:
    """Append one validated canonical SRB JSONL row to an open text handle."""

    _validate_json_string_lengths(row, context="SRB JSONL row")
    canonical = validate_srb_jsonl_row(row)
    _validate_json_string_lengths(canonical, context="SRB JSONL row")
    encoder = json.JSONEncoder(
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    )
    encoded = io.StringIO()
    encoded_bytes = 0
    for chunk in encoder.iterencode(canonical):
        chunk_bytes = len(chunk.encode("ascii"))
        observed = encoded_bytes + chunk_bytes
        if observed + 1 > MAX_JSONL_LINE_BYTES:
            raise ResourceLimitError(
                "JSONL line",
                limit=MAX_JSONL_LINE_BYTES,
                observed=observed + 1,
            )
        encoded.write(chunk)
        encoded_bytes = observed
    if encoded_bytes + 1 > MAX_JSONL_LINE_BYTES:
        raise ResourceLimitError(
            "JSONL line",
            limit=MAX_JSONL_LINE_BYTES,
            observed=encoded_bytes + 1,
        )
    encoded.write("\n")
    fh.write(encoded.getvalue())


def iter_srb_jsonl_rows(
    dataset_path: str | Path,
    *,
    max_episodes: int | None = None,
    max_steps: int | None = None,
    max_line_bytes: int = MAX_JSONL_LINE_BYTES,
    max_total_bytes: int = MAX_DECODED_AGGREGATE_BYTES,
    json_budget: JsonDecodeBudget | None = None,
) -> Iterator[SrbJsonlRow]:
    """Yield validated rows, optionally charging a retained-row JSON budget."""

    for location in iter_srb_jsonl_row_locations(
        dataset_path,
        max_episodes=max_episodes,
        max_steps=max_steps,
        max_line_bytes=max_line_bytes,
        max_total_bytes=max_total_bytes,
        json_budget=json_budget,
    ):
        yield location.row


def iter_srb_jsonl_row_locations(
    dataset_path: str | Path,
    *,
    max_episodes: int | None = None,
    max_steps: int | None = None,
    max_line_bytes: int = MAX_JSONL_LINE_BYTES,
    max_total_bytes: int = MAX_DECODED_AGGREGATE_BYTES,
    json_budget: JsonDecodeBudget | None = None,
) -> Iterator[SrbJsonlRowLocation]:
    """Yield validated rows with their source path, offset, length and line."""

    if json_budget is not None and not isinstance(json_budget, JsonDecodeBudget):
        raise TypeError("json_budget must be a JsonDecodeBudget or None")
    line_limit, total_budget = _jsonl_budgets(
        max_line_bytes=max_line_bytes,
        max_total_bytes=max_total_bytes,
    )
    if max_steps is not None and max_steps <= 0:
        return
    if max_episodes is not None and max_episodes <= 0:
        return
    bookkeeping = _JsonlBookkeeping()
    selector = _JsonlEpisodeSelector(
        max_episodes=max_episodes,
        bookkeeping=bookkeeping,
    )
    try:
        yield from _iter_srb_jsonl_row_locations(
            dataset_path=Path(dataset_path),
            line_limit=line_limit,
            total_budget=total_budget,
            json_budget=json_budget,
            max_steps=max_steps,
            selector=selector,
        )
    except _JsonlBookkeepingExhausted as exc:
        raise exc.error from None


def _iter_srb_jsonl_row_locations(
    *,
    dataset_path: Path,
    line_limit: int,
    total_budget: ByteBudget,
    json_budget: JsonDecodeBudget | None,
    max_steps: int | None,
    selector: _JsonlEpisodeSelector,
) -> Iterator[SrbJsonlRowLocation]:
    """Yield locations through one caller-owned logical-episode selector."""

    yielded_steps = 0
    for episode_path in srb_jsonl_episode_paths(dataset_path):
        with episode_path.open("rb") as f:
            line_stream = _bounded_jsonl_lines(f, line_limit, total_budget)
            line_number = 0
            while True:
                if max_steps is not None and yielded_steps >= max_steps:
                    return
                offset = f.tell()
                try:
                    raw_line = next(line_stream)
                except StopIteration:
                    break
                line_number += 1
                row = _parse_srb_jsonl_line(
                    raw_line,
                    episode_path=episode_path,
                    line_number=line_number,
                    json_budget=json_budget,
                )
                if row is None:
                    continue
                if not selector.observe(
                    row,
                    context=f"{episode_path}:{line_number}",
                ):
                    continue
                yielded_steps += 1
                yield SrbJsonlRowLocation(
                    path=episode_path,
                    offset=offset,
                    length=len(raw_line),
                    line_number=line_number,
                    row=row,
                )


def _scan_srb_jsonl_counts(
    dataset_path: Path,
) -> tuple[int, int, int | None, str | None]:
    """Scan logical counts without retaining canonical rows."""

    line_limit, total_budget = _jsonl_budgets(
        max_line_bytes=MAX_JSONL_LINE_BYTES,
        max_total_bytes=MAX_DECODED_AGGREGATE_BYTES,
    )
    bookkeeping = _JsonlBookkeeping()
    selector = _JsonlEpisodeSelector(
        max_episodes=None,
        bookkeeping=bookkeeping,
    )
    total_samples = 0
    action_dim: int | None = None
    action_dim_error: str | None = None
    try:
        for location in _iter_srb_jsonl_row_locations(
            dataset_path=dataset_path,
            line_limit=line_limit,
            total_budget=total_budget,
            json_budget=None,
            max_steps=None,
            selector=selector,
        ):
            total_samples += 1
            row_action_dim = flat_json_value_len(location.row["action"])
            if row_action_dim is None:
                continue
            if action_dim is None:
                action_dim = row_action_dim
            elif row_action_dim != action_dim and action_dim_error is None:
                action_dim_error = (
                    f"{location.path}:{location.line_number} action dim "
                    f"{row_action_dim} does not match first action dim {action_dim}"
                )
    except _JsonlBookkeepingExhausted as exc:
        raise exc.error from None
    return selector.episode_count, total_samples, action_dim, action_dim_error


def _parse_srb_jsonl_line(
    raw_line: bytes,
    *,
    episode_path: Path,
    line_number: int,
    json_budget: JsonDecodeBudget | None = None,
) -> SrbJsonlRow | None:
    """Decode and validate one bounded raw JSONL line."""

    try:
        line = raw_line.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{episode_path}:{line_number} is not valid UTF-8") from exc
    stripped = line.strip()
    if not stripped:
        return None
    context = f"{episode_path}:{line_number}"
    try:
        decoder = (
            JsonDecodeBudget(max_nodes=MAX_JSONL_NODES)
            if json_budget is None
            else json_budget
        )
        raw_row = decoder.loads(stripped, context=context)
    except RecursionError as exc:
        raise ValueError(f"{context} exceeds JSON nesting limit") from exc
    try:
        _validate_json_structure(
            raw_row,
            context=context,
            max_nodes=MAX_JSONL_NODES,
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(str(exc)) from exc
    return validate_srb_jsonl_row(raw_row, context=context)


def validate_srb_jsonl_row(
    value: object,
    *,
    context: str = "SRB JSONL row",
) -> SrbJsonlRow:
    """Validate one raw row before any NumPy coercion takes place."""

    row, errors = _coerce_srb_jsonl_row(value, context=context)
    if errors:
        raise ValueError("; ".join(errors))
    assert row is not None
    return row


def read_srb_jsonl_episodes(
    dataset_path: str | Path,
    *,
    max_line_bytes: int = MAX_JSONL_LINE_BYTES,
    max_total_bytes: int = MAX_DECODED_AGGREGATE_BYTES,
    json_budget: JsonDecodeBudget | None = None,
) -> SrbJsonlEpisodes:
    """Group rows by episode index under one retained-node operation budget."""

    episodes_by_index: dict[int, list[SrbJsonlRow]] = {}
    decoder = (
        JsonDecodeBudget(max_nodes=MAX_JSONL_NODES)
        if json_budget is None
        else json_budget
    )
    for row in iter_srb_jsonl_rows(
        dataset_path,
        max_line_bytes=max_line_bytes,
        max_total_bytes=max_total_bytes,
        json_budget=decoder,
    ):
        episodes_by_index.setdefault(row["episode_index"], []).append(row)
    if not episodes_by_index:
        raise ValueError("canonical dataset contains no steps")
    return [
        episodes_by_index[episode_index] for episode_index in sorted(episodes_by_index)
    ]


def srb_jsonl_episode_paths(dataset_path: Path) -> list[Path]:
    """Return canonical SRB JSONL episode files for a data path."""

    if dataset_path.is_dir():
        return sorted(dataset_path.glob("*.jsonl"))
    if dataset_path.suffix == ".jsonl":
        return [dataset_path]
    return []


def resolve_srb_jsonl_obs_keys(
    metadata: Mapping[str, JSONValue],
    episodes: SrbJsonlEpisodes,
) -> list[str]:
    """Resolve observation keys from manifest metadata or row contents."""

    metadata_obs_keys = metadata.get("obs_keys")
    if metadata_obs_keys:
        if not isinstance(metadata_obs_keys, list):
            raise ValueError("metadata obs_keys must be a list of strings")
        obs_keys: list[str] = []
        for obs_key in metadata_obs_keys:
            if not isinstance(obs_key, str):
                raise ValueError("metadata obs_keys must be a list of strings")
            obs_keys.append(obs_key)
        return obs_keys

    obs_keys = []
    seen: set[str] = set()
    for rows in episodes:
        for row in rows:
            for obs_key in row["obs"]:
                if obs_key in seen:
                    continue
                seen.add(obs_key)
                obs_keys.append(obs_key)
    if not obs_keys:
        raise ValueError("canonical dataset contains no observation keys")
    return obs_keys


def srb_jsonl_row_obs_array(
    row: SrbJsonlRow, obs_key: str
) -> np.ndarray[Any, np.dtype[Any]]:
    """Return one observation value as an array, or fail with row context."""

    obs = row["obs"]
    if obs_key not in obs:
        raise ValueError(f"missing observation {obs_key!r} in canonical row")
    return _as_array(obs[obs_key])


def srb_jsonl_row_next_obs_array(
    rows: list[SrbJsonlRow],
    step_index: int,
    obs_key: str,
) -> np.ndarray[Any, np.dtype[Any]]:
    """Return next observation, falling back to the following row if omitted."""

    row = rows[step_index]
    next_obs = row.get("next_obs")
    if next_obs is not None and obs_key in next_obs:
        return _as_array(next_obs[obs_key])

    if step_index + 1 < len(rows):
        return srb_jsonl_row_obs_array(rows[step_index + 1], obs_key)
    return srb_jsonl_row_obs_array(row, obs_key)


def srb_jsonl_row_action_array(row: SrbJsonlRow) -> np.ndarray[Any, np.dtype[Any]]:
    """Return one validated action as a flat array."""

    action = _as_array(row["action"])
    if action.ndim == 0:
        return action.reshape(1)
    if action.ndim > 1:
        return action.reshape(-1)
    return action


def infer_srb_jsonl_action_dim(episodes: SrbJsonlEpisodes) -> int:
    """Infer the flat action dimension from the first canonical row."""

    for rows in episodes:
        for row in rows:
            return int(srb_jsonl_row_action_array(row).shape[0])
    return 0


def flat_json_value_len(value: JSONValue) -> int | None:
    """Return flat scalar/list length for JSON numeric values."""

    measured = _flat_json_value_measure(value)
    return None if measured is None else measured[1]


def _flat_json_value_measure(value: JSONValue) -> tuple[tuple[int, ...], int] | None:
    """Return regular numeric list shape and size, rejecting ragged arrays."""

    if _is_number_value(value) or isinstance(value, bool):
        return (), 1
    if not isinstance(value, list):
        return None
    if not value:
        return (0,), 0
    first = _flat_json_value_measure(value[0])
    if first is None:
        return None
    child_shape, total = first
    for item in value[1:]:
        measured = _flat_json_value_measure(item)
        if measured is None or measured[0] != child_shape:
            return None
        total += measured[1]
    return (len(value), *child_shape), total


def to_jsonable(value: object) -> JSONValue:
    """Convert common NumPy/array containers to validated JSON values."""

    if isinstance(value, np.ndarray):
        return to_jsonable(value.tolist())
    if isinstance(value, np.generic):
        return to_jsonable(value.item())
    if isinstance(value, bytes):
        return value.decode("utf-8")
    if isinstance(value, Mapping):
        result: JSONObject = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(
                    f"JSON object keys must be strings, got {type(key).__name__}"
                )
            result[key] = to_jsonable(item)
        return result
    if isinstance(value, tuple):
        return [to_jsonable(item) for item in value]
    return validate_json_value(value, context="JSON value")


def validate_srb_jsonl_contents(
    *,
    metadata: Mapping[str, JSONValue],
    dataset_path: Path,
    errors: list[str],
    warnings: list[str],
    max_line_bytes: int = MAX_JSONL_LINE_BYTES,
    max_total_bytes: int = MAX_DECODED_AGGREGATE_BYTES,
) -> None:
    """Validate canonical SRB JSONL data against manifest metadata."""

    episode_paths = srb_jsonl_episode_paths(dataset_path)
    if not episode_paths:
        errors.append(f"canonical dataset has no JSONL episode files: {dataset_path}")
        return

    total_samples = 0
    obs_keys: set[str] = set()
    action_dim: int | None = None

    try:
        line_limit, total_budget = _jsonl_budgets(
            max_line_bytes=max_line_bytes,
            max_total_bytes=max_total_bytes,
        )
    except (TypeError, ValueError, ResourceLimitError) as exc:
        errors.append(f"canonical JSONL budget rejected: {exc}")
        return

    bookkeeping = _JsonlBookkeeping()
    selector = _JsonlEpisodeSelector(
        max_episodes=None,
        bookkeeping=bookkeeping,
    )
    declared_leaves = _declared_jsonl_leaves(metadata)
    last_step_indices: dict[int, int] = {}
    try:
        for episode_path in episode_paths:
            line_count = 0
            file_had_done = False
            try:
                line_stream = episode_path.open("rb")
            except OSError as exc:
                bookkeeping.append_diagnostic(
                    errors,
                    f"could not read canonical JSONL file {episode_path}: {exc}",
                )
                continue
            try:
                with line_stream as f:
                    for line_number, raw_line in enumerate(
                        _bounded_jsonl_lines(f, line_limit, total_budget),
                        start=1,
                    ):
                        try:
                            line = raw_line.decode("utf-8")
                        except UnicodeDecodeError:
                            bookkeeping.append_diagnostic(
                                errors,
                                f"{episode_path.name}:{line_number} is not valid UTF-8",
                            )
                            continue
                        stripped = line.strip()
                        if not stripped:
                            continue
                        line_count += 1
                        total_samples += 1
                        context = f"{episode_path.name}:{line_number}"
                        try:
                            raw_row = JsonDecodeBudget(max_nodes=MAX_JSONL_NODES).loads(
                                stripped, context=context
                            )
                        except (JSONDecodeError, RecursionError) as exc:
                            message = (
                                exc.msg
                                if isinstance(exc, JSONDecodeError)
                                else str(exc)
                            )
                            bookkeeping.append_diagnostic(
                                errors,
                                f"{context} invalid JSON: {message}",
                            )
                            continue

                        row, row_errors = _coerce_srb_jsonl_row(
                            raw_row,
                            context=context,
                        )
                        if row_errors:
                            for error in row_errors:
                                bookkeeping.append_diagnostic(
                                    errors,
                                    f"{context} {error}",
                                )
                            continue
                        assert row is not None
                        for error in _declared_leaf_errors(row, declared_leaves):
                            bookkeeping.append_diagnostic(errors, f"{context} {error}")
                        try:
                            selector.observe(
                                row,
                                context=context,
                            )
                        except ValueError as exc:
                            bookkeeping.append_diagnostic(errors, str(exc))
                        episode_index = row["episode_index"]
                        last_step_index = last_step_indices.get(episode_index)
                        if (
                            last_step_index is not None
                            and row["step_index"] <= last_step_index
                        ):
                            bookkeeping.append_diagnostic(
                                errors,
                                f"{context} step_index must be strictly increasing",
                            )
                        bookkeeping.retain_mapping_integer(
                            last_step_indices,
                            episode_index,
                            row["step_index"],
                        )
                        for obs_key in row["obs"]:
                            bookkeeping.retain_string(obs_keys, obs_key)

                        row_action_dim = flat_json_value_len(row["action"])
                        assert row_action_dim is not None and row_action_dim > 0
                        if action_dim is None:
                            action_dim = row_action_dim
                        elif row_action_dim != action_dim:
                            bookkeeping.append_diagnostic(
                                errors,
                                f"{context} action dim {row_action_dim} does not "
                                f"match first action dim {action_dim}",
                            )
                        file_had_done = row["done"]
            except ResourceLimitError as exc:
                bookkeeping.append_diagnostic(
                    errors,
                    f"{episode_path.name} exceeds JSONL resource limit: {exc}",
                )
                return
            except OSError as exc:
                bookkeeping.append_diagnostic(
                    errors,
                    f"could not read canonical JSONL file {episode_path}: {exc}",
                )
                continue

            if line_count == 0:
                bookkeeping.append_diagnostic(
                    errors,
                    f"{episode_path.name} contains no JSONL rows",
                )
            elif not file_had_done:
                bookkeeping.append_diagnostic(
                    warnings,
                    f"{episode_path.name} has no terminal done=True row",
                )

        _append_metadata_count(
            metadata=metadata,
            key="num_episodes",
            actual=selector.episode_count,
            errors=errors,
            bookkeeping=bookkeeping,
        )
        _append_metadata_count(
            metadata=metadata,
            key="total_samples",
            actual=total_samples,
            errors=errors,
            bookkeeping=bookkeeping,
        )
        if action_dim is not None:
            _append_metadata_count(
                metadata=metadata,
                key="action_dim",
                actual=action_dim,
                errors=errors,
                bookkeeping=bookkeeping,
            )

        _validate_metadata_obs_keys_bounded(
            metadata=metadata,
            actual=obs_keys,
            errors=errors,
            bookkeeping=bookkeeping,
        )

        if selector.selected_ids and len(selector.selected_ids) != len(episode_paths):
            bookkeeping.append_diagnostic(
                warnings,
                "episode_index values do not match the number of episode files: "
                f"indices={_bounded_values_text(selector.selected_ids)} "
                f"files={len(episode_paths)}",
            )
    except _JsonlBookkeepingExhausted as exc:
        bookkeeping.append_exhaustion(errors, exc.error)


def _append_metadata_count(
    *,
    metadata: Mapping[str, JSONValue],
    key: str,
    actual: int,
    errors: list[str],
    bookkeeping: _JsonlBookkeeping,
) -> None:
    if key not in metadata:
        return
    expected = metadata[key]
    if not _is_int_value(expected):
        bookkeeping.append_diagnostic(
            errors,
            f"metadata {key} must be an integer",
        )
        return
    expected_int = int(expected)
    if expected_int != actual:
        bookkeeping.append_diagnostic(
            errors,
            f"metadata {key}={_bounded_repr(expected_int)} "
            f"does not match actual {actual}",
        )


_MISSING_LEAF = object()


def _declared_jsonl_leaves(
    metadata: Mapping[str, JSONValue],
) -> list[tuple[tuple[tuple[str, ...], ...], Any, tuple[int, ...] | None]]:
    """Observation dtypes and shapes declared by ``demo_io_contract``."""
    from srb.datasets.visual_contract import declared_observation_terms

    contract = metadata.get("demo_io_contract")
    if not isinstance(contract, Mapping):
        return []
    terms: list[tuple[tuple[tuple[str, ...], ...], Any, tuple[int, ...] | None]] = (
        declared_observation_terms(contract)
    )
    return terms


def _declared_leaf_errors(
    row: Mapping[str, Any],
    declared: list[tuple[tuple[tuple[str, ...], ...], Any, tuple[int, ...] | None]],
) -> list[str]:
    """Report declared leaves whose JSON values break their dtype or shape."""
    if not declared:
        return []
    import numpy as np

    from srb.datasets.visual_contract import cast_declared, contract_shape_error

    errors: list[str] = []
    for tree_name in ("obs", "next_obs"):
        tree = row.get(tree_name)
        if not isinstance(tree, Mapping):
            continue
        for paths, dtype, shape in declared:
            present: list[tuple[tuple[str, ...], Any]] = []
            for path in paths:
                value: Any = tree
                for part in path:
                    if not isinstance(value, Mapping) or part not in value:
                        value = _MISSING_LEAF
                        break
                    value = value[part]
                if value is not _MISSING_LEAF:
                    present.append((path, value))
            if not present:
                aliases = " or ".join("/".join(path) for path in paths)
                errors.append(
                    f"{tree_name} is missing declared observation term; expected "
                    f"one of {aliases}"
                )
                continue
            for path, value in present:
                label = f"{tree_name}/{'/'.join(path)}"
                if value is None:
                    errors.append(f"{label} is null; declared {dtype}")
                    continue
                try:
                    array = (
                        np.asarray(value)
                        if dtype is None
                        else cast_declared(value, dtype, resource=label)
                    )
                except (TypeError, ValueError) as exc:
                    errors.append(str(exc))
                    continue
                if shape is not None:
                    error = contract_shape_error(label, array.shape, shape)
                    if error is not None:
                        errors.append(error)
    return errors


def _validate_metadata_obs_keys_bounded(
    *,
    metadata: Mapping[str, JSONValue],
    actual: set[str],
    errors: list[str],
    bookkeeping: _JsonlBookkeeping,
) -> None:
    raw_keys = metadata.get("obs_keys")
    if raw_keys is None:
        return
    if isinstance(raw_keys, (str, bytes)) or not isinstance(raw_keys, Iterable):
        bookkeeping.append_diagnostic(
            errors,
            "metadata obs_keys must be an iterable of keys",
        )
        return

    declared: set[str] = set()
    for raw_key in raw_keys:
        bookkeeping.retain_string(declared, str(raw_key))
    if declared == actual:
        return
    bookkeeping.append_diagnostic(
        errors,
        "metadata obs_keys does not match dataset observations: "
        f"declared={_bounded_values_text(declared)} "
        f"actual={_bounded_values_text(actual)}",
    )


def _bounded_values_text(values: set[Any]) -> str:
    examples = heapq.nsmallest(_JSONL_MISMATCH_EXAMPLES, values)
    if len(values) <= _JSONL_MISMATCH_EXAMPLES and all(
        _value_is_short(value) for value in examples
    ):
        return str(sorted(values))
    rendered = "[" + ", ".join(_bounded_repr(value) for value in examples) + "]"
    remaining = len(values) - len(examples)
    if remaining:
        return f"{rendered} (remaining={remaining})"
    return rendered


def _value_is_short(value: Any) -> bool:
    if isinstance(value, str):
        return len(value) <= _JSONL_DIAGNOSTIC_VALUE_CHARS
    if isinstance(value, int):
        return value.bit_length() <= _JSONL_DIAGNOSTIC_VALUE_CHARS * 3
    return len(repr(value)) <= _JSONL_DIAGNOSTIC_VALUE_CHARS


def _bounded_repr(value: Any) -> str:
    if isinstance(value, str):
        if len(value) > _JSONL_DIAGNOSTIC_VALUE_CHARS:
            value = value[:_JSONL_DIAGNOSTIC_VALUE_CHARS] + "...<truncated>"
        return repr(value)
    if isinstance(value, int):
        if not _value_is_short(value):
            return f"<integer with {value.bit_length()} bits>"
        return str(value)
    rendered = repr(value)
    if len(rendered) > _JSONL_DIAGNOSTIC_VALUE_CHARS:
        rendered = rendered[:_JSONL_DIAGNOSTIC_VALUE_CHARS] + "...<truncated>"
    return rendered


def _text_cost(value: str) -> int:
    return _utf8_length(value) + _JSONL_METADATA_ITEM_OVERHEAD


def _integer_cost(value: int) -> int:
    """Charge integer magnitude before retaining it in bookkeeping state."""

    return max(1, value.bit_length()) + _JSONL_METADATA_ITEM_OVERHEAD


def _utf8_length(value: str) -> int:
    """Measure UTF-8 bytes without allocating an encoded copy."""

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
    return encoded_length


def _coerce_srb_jsonl_row(
    value: object,
    *,
    context: str,
) -> tuple[SrbJsonlRow | None, list[str]]:
    try:
        _validate_json_structure(
            value,
            context=context,
            max_nodes=MAX_JSONL_NODES,
        )
    except (TypeError, ValueError) as exc:
        return None, [str(exc)]
    try:
        payload = validate_json_object(value, context=context)
    except (TypeError, ValueError) as exc:
        return None, [str(exc)]

    errors: list[str] = []
    unknown_keys = set(payload) - _ALLOWED_SRB_JSONL_KEYS
    errors.extend(f"unknown key {key!r}" for key in sorted(unknown_keys))
    for key in REQUIRED_SRB_JSONL_KEYS:
        if key not in payload:
            errors.append(f"missing required key {key!r}")

    episode_index = 0
    raw_episode_index = payload.get("episode_index")
    if _is_int_value(raw_episode_index) and isinstance(raw_episode_index, int):
        episode_index = raw_episode_index
        if episode_index < 0:
            errors.append("episode_index must be non-negative")
    elif "episode_index" in payload:
        errors.append("episode_index must be an integer")

    step_index = 0
    raw_step_index = payload.get("step_index")
    if _is_int_value(raw_step_index) and isinstance(raw_step_index, int):
        step_index = raw_step_index
        if step_index < 0:
            errors.append("step_index must be non-negative")
    elif "step_index" in payload:
        errors.append("step_index must be an integer")

    obs: JSONObject = {}
    raw_obs = payload.get("obs")
    if isinstance(raw_obs, dict) and raw_obs:
        obs = raw_obs
    elif "obs" in payload:
        errors.append("obs must be a non-empty object")

    action: JSONValue = None
    raw_action = payload.get("action")
    action_length = flat_json_value_len(raw_action)
    if action_length is None or action_length <= 0:
        if "action" in payload:
            errors.append("action must be a non-empty scalar/list")
    else:
        action = raw_action

    reward = 0.0
    raw_reward = payload.get("reward")
    if _is_number_value(raw_reward) and isinstance(raw_reward, (int, float)):
        try:
            reward = float(raw_reward)
        except OverflowError:
            # JSON integers are unbounded; a huge one has no float value.
            errors.append("reward is out of float range")
    elif "reward" in payload:
        errors.append("reward must be numeric")

    done = False
    raw_done = payload.get("done")
    if isinstance(raw_done, bool):
        done = raw_done
    elif "done" in payload:
        errors.append("done must be boolean")

    success = False
    raw_success = payload.get("success")
    if isinstance(raw_success, bool):
        success = raw_success
    elif "success" in payload:
        errors.append("success must be boolean")

    terminated: bool | None = None
    truncated: bool | None = None
    if ("terminated" in payload) != ("truncated" in payload):
        errors.append("terminated and truncated must be supplied together")
    elif "terminated" in payload:
        raw_terminated = payload.get("terminated")
        raw_truncated = payload.get("truncated")
        if not isinstance(raw_terminated, bool) or not isinstance(raw_truncated, bool):
            errors.append("terminated and truncated must be boolean")
        else:
            terminated, truncated = raw_terminated, raw_truncated
            if done != (terminated or truncated):
                errors.append("done must equal (terminated or truncated)")

    episode_key: str | None = None
    if "episode_key" in payload:
        raw_episode_key = payload["episode_key"]
        if isinstance(raw_episode_key, str):
            episode_key = raw_episode_key
        else:
            errors.append("episode_key must be a string")

    next_obs: JSONObject | None = None
    if "next_obs" in payload:
        raw_next_obs = payload["next_obs"]
        if isinstance(raw_next_obs, dict):
            next_obs = raw_next_obs
        else:
            errors.append("next_obs must be an object")

    info: JSONObject | None = None
    if "info" in payload:
        raw_info = payload["info"]
        if isinstance(raw_info, dict):
            info = raw_info
        else:
            errors.append("info must be an object")

    # Absent means a pre-versioning row, which by definition is the layout this
    # module has always read. A version that is PRESENT and unknown is a row
    # written by a newer producer, and guessing at it is how a format silently
    # forks.
    format_version: str | None = None
    if "format_version" in payload:
        raw_format_version = payload["format_version"]
        if not isinstance(raw_format_version, str):
            errors.append("format_version must be a string")
        elif raw_format_version != SRB_JSONL_VERSION:
            errors.append(
                f"unsupported format_version {raw_format_version!r} "
                f"(this reader supports {SRB_JSONL_VERSION!r})"
            )
        else:
            format_version = raw_format_version

    if errors:
        return None, errors

    row: SrbJsonlRow = {
        "episode_index": episode_index,
        "step_index": step_index,
        "obs": obs,
        "action": action,
        "reward": reward,
        "done": done,
        "success": success,
    }
    if terminated is not None and truncated is not None:
        row["terminated"] = terminated
        row["truncated"] = truncated
    if episode_key is not None:
        row["episode_key"] = episode_key
    if next_obs is not None:
        row["next_obs"] = next_obs
    if info is not None:
        row["info"] = info
    if format_version is not None:
        row["format_version"] = format_version
    return row, []


def _jsonl_budgets(
    *,
    max_line_bytes: int,
    max_total_bytes: int,
) -> tuple[int, ByteBudget]:
    line_limit = require_bounded_int(
        max_line_bytes,
        resource="JSONL line bytes",
        minimum=1,
        maximum=MAX_IN_MEMORY_FILE_BYTES,
    )
    total_limit = require_bounded_int(
        max_total_bytes,
        resource="JSONL aggregate bytes",
        minimum=1,
        maximum=MAX_DECODED_AGGREGATE_BYTES,
    )
    return line_limit, ByteBudget("JSONL aggregate bytes", total_limit)


def _bounded_jsonl_lines(
    stream: Any,
    line_limit: int,
    total_budget: ByteBudget,
) -> Iterator[bytes]:
    """Read at most one bounded raw line before any JSON decoding."""

    while True:
        raw_line = stream.readline(line_limit + 1)
        if not raw_line:
            return
        if len(raw_line) > line_limit:
            raise ResourceLimitError(
                "JSONL line",
                limit=line_limit,
                observed=len(raw_line),
            )
        total_budget.consume(len(raw_line))
        yield raw_line


def _validate_json_string_lengths(value: object, *, context: str) -> None:
    """Reject raw strings that cannot fit in one bounded JSONL line."""

    stack: list[tuple[object, int]] = [(value, 0)]
    nodes = 0
    while stack:
        current, depth = stack.pop()
        nodes += 1
        if depth > MAX_JSONL_NESTING:
            raise ResourceLimitError(
                f"{context} JSON nesting",
                limit=MAX_JSONL_NESTING,
                observed=depth,
            )
        if nodes > MAX_JSONL_NODES:
            raise ResourceLimitError(
                f"{context} JSON nodes",
                limit=MAX_JSONL_NODES,
                observed=nodes,
            )
        if isinstance(current, str):
            encoded_length = _utf8_length(current)
            if encoded_length > MAX_JSONL_LINE_BYTES:
                raise ResourceLimitError(
                    "JSONL line",
                    limit=MAX_JSONL_LINE_BYTES,
                    observed=encoded_length,
                )
            continue
        if isinstance(current, Mapping):
            for key, item in current.items():
                if isinstance(key, str):
                    key_length = _utf8_length(key)
                    if key_length > MAX_JSONL_LINE_BYTES:
                        raise ResourceLimitError(
                            "JSONL line",
                            limit=MAX_JSONL_LINE_BYTES,
                            observed=key_length,
                        )
                stack.append((item, depth + 1))
        elif isinstance(current, (list, tuple)):
            stack.extend((item, depth + 1) for item in current)


def _validate_json_structure(
    value: object,
    *,
    context: str,
    max_depth: int = MAX_JSONL_NESTING,
    max_nodes: int = MAX_JSON_NODES,
) -> None:
    """Check decoded JSON shape without recursive traversal or silent truncation."""

    stack: list[tuple[object, int]] = [(value, 0)]
    nodes = 0
    while stack:
        current, depth = stack.pop()
        nodes += 1
        if depth > max_depth:
            raise ResourceLimitError(
                f"{context} JSON nesting",
                limit=max_depth,
                observed=depth,
            )
        if nodes > max_nodes:
            raise ResourceLimitError(
                f"{context} JSON nodes",
                limit=max_nodes,
                observed=nodes,
            )
        if isinstance(current, Mapping):
            for key, item in current.items():
                stack.append((key, depth + 1))
                stack.append((item, depth + 1))
        elif isinstance(current, list):
            stack.extend((item, depth + 1) for item in current)


def _require_json_object(value: JSONValue, *, field_name: str) -> JSONObject:
    if not isinstance(value, dict):
        raise TypeError(f"{field_name} must be an object")
    return value


def _require_reward(value: object) -> float:
    converted = to_jsonable(0.0 if value is None else value)
    if not _is_number_value(converted) or not isinstance(converted, (int, float)):
        raise TypeError("reward must be numeric")
    try:
        return float(converted)
    except OverflowError as error:
        raise ValueError("reward is out of float range") from error


def _as_array(value: JSONValue) -> np.ndarray[Any, np.dtype[Any]]:
    """Cross the validated JSON-to-NumPy adapter boundary."""

    return cast(np.ndarray[Any, np.dtype[Any]], np.asarray(value))


__all__ = [
    "REQUIRED_SRB_JSONL_KEYS",
    "SrbJsonlEpisodes",
    "SrbJsonlRowLocation",
    "SrbJsonlRow",
    "build_srb_jsonl_row",
    "flat_json_value_len",
    "infer_srb_jsonl_action_dim",
    "iter_srb_jsonl_rows",
    "iter_srb_jsonl_row_locations",
    "read_srb_jsonl_episodes",
    "resolve_srb_jsonl_obs_keys",
    "srb_jsonl_episode_paths",
    "srb_jsonl_row_action_array",
    "srb_jsonl_row_next_obs_array",
    "srb_jsonl_row_obs_array",
    "to_jsonable",
    "validate_srb_jsonl_contents",
    "validate_srb_jsonl_row",
    "write_srb_jsonl_row",
]
