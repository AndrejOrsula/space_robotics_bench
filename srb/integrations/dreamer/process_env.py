"""Dependency-light Dreamer environment adapter for process transports."""

from __future__ import annotations

import copy
from collections import OrderedDict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Protocol, runtime_checkable

import gymnasium
import numpy as np

from .process_protocol import validate_landscaping_contract

__all__ = [
    "DreamerProcessEnv",
    "DreamerProcessEnvError",
    "DreamerProcessRowEpisodeState",
    "DreamerProcessTransportProtocol",
]


class DreamerProcessEnvError(RuntimeError):
    """The process-backed Dreamer environment contract was violated."""


@dataclass(frozen=True, slots=True)
class DreamerProcessRowEpisodeState:
    """Immutable callback-only identity and frame flags for one worker row."""

    env_id: int
    episode_id: int
    step_index: int
    is_first: bool
    is_last: bool


DreamerRowEpisodeState = DreamerProcessRowEpisodeState


@runtime_checkable
class DreamerProcessTransportProtocol(Protocol):
    """Public transport seam required by :class:`DreamerProcessEnv`."""

    num_envs: int

    def handshake(self) -> Any:
        """Negotiate and return the immutable process schema."""

    def reset(self) -> Any:
        """Reset every worker row atomically."""

    def reset_and_step(self, action: Any, reset_mask: np.ndarray) -> Any:
        """Reset selected rows and step the remaining rows once."""

    @property
    def row_episode_states(self) -> Sequence[Any]:
        """Return transport-owned row identity snapshots."""

    def is_running(self) -> bool:
        """Return whether all worker processes and endpoints are live."""

    def close(self) -> None:
        """Clean up and reap all worker processes."""


_OBS_RESERVED_KEYS = ("reward", "is_first", "is_last", "is_terminal")
_ACT_RESERVED_KEYS = ("reset",)


def _strict_int(value: Any, *, name: str) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise DreamerProcessEnvError(f"{name} must be an integer")
    return int(value)


def _projection_to_leaves(
    projection: Any,
    *,
    name: str,
) -> tuple[Any, tuple[tuple[str, tuple[str | int, ...], Any], ...]]:
    try:
        metadata = projection.metadata
    except Exception as exc:
        raise DreamerProcessEnvError(
            f"{name} projection metadata is unavailable"
        ) from exc
    if not isinstance(metadata, Mapping) or metadata.get("version") != 2:
        raise DreamerProcessEnvError(f"{name} projection must use metadata version 2")

    to_space = getattr(projection, "to_space", None)
    if not callable(to_space):
        raise DreamerProcessEnvError(f"{name} projection does not expose to_space()")
    try:
        root = to_space()
    except Exception as exc:
        raise DreamerProcessEnvError(
            f"{name} projection cannot reconstruct its space"
        ) from exc

    try:
        wire_names = projection.wire_names
        paths = projection.paths
    except Exception as exc:
        raise DreamerProcessEnvError(
            f"{name} projection wire layout is unavailable"
        ) from exc
    if type(wire_names) is not tuple or type(paths) is not tuple:
        raise DreamerProcessEnvError(
            f"{name} projection wire layout must use immutable tuples"
        )
    if not wire_names or len(wire_names) != len(paths):
        raise DreamerProcessEnvError(
            f"{name} projection wire names and paths must be non-empty and aligned"
        )
    if any(type(wire_name) is not str or not wire_name for wire_name in wire_names):
        raise DreamerProcessEnvError(
            f"{name} projection wire names must be non-empty strings"
        )
    if len(set(wire_names)) != len(wire_names):
        raise DreamerProcessEnvError(f"{name} projection wire names must be unique")

    flat_top_level = isinstance(root, gymnasium.spaces.Dict) and all(
        isinstance(space, (gymnasium.spaces.Box, gymnasium.spaces.Discrete))
        for space in root.spaces.values()
    )

    leaves: list[tuple[str, tuple[str | int, ...], Any]] = []
    learner_keys: set[str] = set()
    for wire_name, path in zip(wire_names, paths, strict=True):
        if type(path) is not tuple:
            raise DreamerProcessEnvError(
                f"{name} projection path for {wire_name!r} must be a tuple"
            )
        space = root
        for depth, component in enumerate(path):
            if isinstance(space, gymnasium.spaces.Dict):
                if type(component) is not str or component not in space.spaces:
                    raise DreamerProcessEnvError(
                        f"{name} projection path for {wire_name!r} has an invalid "
                        f"Dict component at depth {depth}"
                    )
                space = space.spaces[component]
                continue
            if isinstance(space, gymnasium.spaces.Tuple):
                if (
                    type(component) is not int
                    or component < 0
                    or component >= len(space.spaces)
                ):
                    raise DreamerProcessEnvError(
                        f"{name} projection path for {wire_name!r} has an invalid "
                        f"Tuple component at depth {depth}"
                    )
                space = space.spaces[component]
                continue
            raise DreamerProcessEnvError(
                f"{name} projection path for {wire_name!r} descends through a leaf"
            )
        if not isinstance(space, (gymnasium.spaces.Box, gymnasium.spaces.Discrete)):
            raise DreamerProcessEnvError(
                f"{name} projection path for {wire_name!r} does not end at a leaf"
            )
        if flat_top_level:
            if len(path) != 1 or type(path[0]) is not str:
                raise DreamerProcessEnvError(
                    f"{name} flat projection has a non-canonical leaf path"
                )
            learner_key = path[0]
        else:
            learner_key = wire_name
        if learner_key in learner_keys:
            raise DreamerProcessEnvError(
                f"{name} learner projection keys must be unique"
            )
        learner_keys.add(learner_key)
        _validate_gym_leaf(space, name=f"{name} learner leaf {learner_key!r}")
        leaves.append((learner_key, path, space))
    return root, tuple(leaves)


def _collect_batched_leaves(
    value: Any,
    space: Any,
    *,
    name: str,
) -> dict[tuple[str | int, ...], Any]:
    leaves: dict[tuple[str | int, ...], Any] = {}

    def visit(current: Any, current_space: Any, path: tuple[str | int, ...]) -> None:
        if isinstance(current_space, gymnasium.spaces.Dict):
            expected = tuple(current_space.spaces)
            mapping = _validate_mapping_order(current, expected, name=f"{name}{path!r}")
            for key, child_space in current_space.spaces.items():
                visit(mapping[key], child_space, (*path, key))
            return
        if isinstance(current_space, gymnasium.spaces.Tuple):
            if isinstance(current, (str, bytes, bytearray)) or not isinstance(
                current, Sequence
            ):
                raise DreamerProcessEnvError(f"{name}{path!r} must be a sequence")
            if len(current) != len(current_space.spaces):
                raise DreamerProcessEnvError(
                    f"{name}{path!r} must have {len(current_space.spaces)} items"
                )
            for index, child_space in enumerate(current_space.spaces):
                visit(current[index], child_space, (*path, index))
            return
        leaves[path] = current

    visit(value, space, ())
    return leaves


def _rebuild_batched_tree(
    space: Any,
    leaves: Mapping[tuple[str | int, ...], np.ndarray],
    path: tuple[str | int, ...] = (),
) -> Any:
    if isinstance(space, gymnasium.spaces.Dict):
        return OrderedDict(
            (
                key,
                _rebuild_batched_tree(child_space, leaves, (*path, key)),
            )
            for key, child_space in space.spaces.items()
        )
    if isinstance(space, gymnasium.spaces.Tuple):
        return tuple(
            _rebuild_batched_tree(child_space, leaves, (*path, index))
            for index, child_space in enumerate(space.spaces)
        )
    try:
        return leaves[path]
    except KeyError as exc:
        raise DreamerProcessEnvError(
            f"action projection is missing leaf path {path!r}"
        ) from exc


def _validate_gym_leaf(space: Any, *, name: str) -> None:
    if isinstance(space, gymnasium.spaces.Box):
        try:
            dtype = np.dtype(space.dtype)
            shape = tuple(int(dim) for dim in space.shape)
            low = np.asarray(space.low)
            high = np.asarray(space.high)
        except Exception as exc:
            raise DreamerProcessEnvError(f"{name} has malformed Box semantics") from exc
        if low.shape != shape or high.shape != shape:
            raise DreamerProcessEnvError(f"{name} Box bounds do not match its shape")
        if np.dtype(low.dtype) != dtype or np.dtype(high.dtype) != dtype:
            raise DreamerProcessEnvError(f"{name} Box bounds do not match its dtype")
        try:
            ordered = np.all(low <= high)
        except Exception as exc:
            raise DreamerProcessEnvError(
                f"{name} Box bounds are not comparable"
            ) from exc
        if not bool(ordered):
            raise DreamerProcessEnvError(f"{name} Box low bound exceeds high bound")
        return

    if isinstance(space, gymnasium.spaces.Discrete):
        n = _strict_int(space.n, name=f"{name}.n")
        start = _strict_int(getattr(space, "start", 0), name=f"{name}.start")
        if n < 1 or start != 0:
            raise DreamerProcessEnvError(
                f"{name} Discrete semantics must have n>=1 and start=0"
            )
        if tuple(space.shape) != () or np.dtype(space.dtype) != np.dtype(np.int64):
            raise DreamerProcessEnvError(
                f"{name} Discrete semantics must be scalar canonical int64"
            )
        return

    raise DreamerProcessEnvError(f"{name} is not a supported Gymnasium leaf")


def _load_elements_space() -> Any:
    try:
        import elements
    except ImportError as exc:
        raise DreamerProcessEnvError(
            "elements is required when accessing Dreamer learner spaces"
        ) from exc
    space_type = getattr(elements, "Space", None)
    if not callable(space_type):
        raise DreamerProcessEnvError("elements.Space is unavailable")
    return space_type


def _to_elements_space(space: Any, space_type: Any) -> Any:
    if isinstance(space, gymnasium.spaces.Discrete):
        # This mirrors the established Dreamer wrapper contract. The process
        # projection remains canonical int64 on the wire; actions are widened
        # back to that dtype immediately before the transport call.
        return space_type(np.int32, (), 0, int(space.n))
    return space_type(
        space.dtype,
        tuple(space.shape),
        np.array(space.low, copy=True),
        np.array(space.high, copy=True),
    )


def _copy_space_mapping(value: Mapping[str, Any]) -> OrderedDict[str, Any]:
    return OrderedDict((key, copy.deepcopy(space)) for key, space in value.items())


def _validate_mapping_order(
    value: Any, expected: tuple[str, ...], *, name: str
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise DreamerProcessEnvError(f"{name} must be a mapping")
    actual = tuple(value.keys())
    if any(type(key) is not str for key in actual):
        raise DreamerProcessEnvError(f"{name} keys must be strings")
    if actual != expected:
        raise DreamerProcessEnvError(
            f"{name} keys/order must be {expected!r}, got {actual!r}"
        )
    return value


def _validate_array(
    value: Any,
    *,
    name: str,
    shape: tuple[int, ...],
    dtype: np.dtype[Any],
) -> np.ndarray:
    if not isinstance(value, np.ndarray):
        raise DreamerProcessEnvError(f"{name} must be a NumPy array")
    if tuple(value.shape) != shape:
        raise DreamerProcessEnvError(
            f"{name} shape must be {shape}, got {tuple(value.shape)}"
        )
    if np.dtype(value.dtype) != dtype:
        raise DreamerProcessEnvError(f"{name} dtype must be {dtype}, got {value.dtype}")
    return np.array(value, copy=True)


def _validate_vector(value: Any, *, name: str, num_envs: int, dtype: Any) -> np.ndarray:
    return _validate_array(
        value,
        name=name,
        shape=(num_envs,),
        dtype=np.dtype(dtype),
    )


def _copy_info(value: Any, *, num_envs: int) -> tuple[Mapping[str, Any] | None, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise DreamerProcessEnvError("transport info must be a row-aligned sequence")
    if len(value) != num_envs:
        raise DreamerProcessEnvError(
            f"transport info must have length {num_envs}, got {len(value)}"
        )
    result: list[Mapping[str, Any] | None] = []
    for index, item in enumerate(value):
        if item is None:
            result.append(None)
            continue
        if not isinstance(item, Mapping):
            raise DreamerProcessEnvError(
                f"transport info row {index} must be a mapping or None"
            )
        if any(type(key) is not str for key in item):
            raise DreamerProcessEnvError(
                f"transport info row {index} keys must be strings"
            )
        try:
            result.append(copy.deepcopy(dict(item)))
        except Exception as exc:
            raise DreamerProcessEnvError(
                f"transport info row {index} could not be copied"
            ) from exc
    return tuple(result)


class DreamerProcessEnv:
    """Adapt a negotiated process transport to Dreamer's Env seam.

    The transport owns simulator lifecycle and selective resets. This adapter
    validates the flat learner contract, copies host arrays, and adds Dreamer
    control fields.
    """

    def __init__(self, transport: DreamerProcessTransportProtocol) -> None:
        self._transport = transport
        self._closed = False
        self._elements_obs_space: OrderedDict[str, Any] | None = None
        self._elements_act_space: OrderedDict[str, Any] | None = None
        self._demo_projection: Any | None = None
        self._landscaping_contract: Mapping[str, Any] | None = None
        self._info: tuple[Mapping[str, Any] | None, ...] = ()
        self._row_is_first = np.zeros(0, dtype=np.bool_)
        self._row_is_last = np.zeros(0, dtype=np.bool_)

        try:
            schema = self._negotiate()
            self._num_envs = self._read_num_envs()
            self._observation_root, self._observation_leaves = _projection_to_leaves(
                schema.observation_projection,
                name="observation",
            )
            self._action_root, self._action_leaves = _projection_to_leaves(
                schema.action_projection,
                name="action",
            )
            observation_keys = tuple(
                key for key, _path, _space in self._observation_leaves
            )
            action_keys = tuple(key for key, _path, _space in self._action_leaves)
            if set(observation_keys) & set(_OBS_RESERVED_KEYS):
                raise DreamerProcessEnvError(
                    "observation projection uses a reserved Dreamer control key"
                )
            if set(action_keys) & set(_ACT_RESERVED_KEYS):
                raise DreamerProcessEnvError(
                    "action projection uses the reserved reset key"
                )
            self._observation_keys = observation_keys
            self._action_keys = action_keys
            descriptor = getattr(schema, "demo_projection", None)
            if descriptor is not None:
                try:
                    from .demo_projection import DreamerDemoProjection

                    self._demo_projection = DreamerDemoProjection.from_metadata(
                        descriptor
                    )
                except (TypeError, ValueError) as exc:
                    raise DreamerProcessEnvError(
                        "negotiated demo projection metadata is invalid"
                    ) from exc
            landscaping_contract = getattr(schema, "landscaping_contract", None)
            if landscaping_contract is not None:
                try:
                    self._landscaping_contract = MappingProxyType(
                        validate_landscaping_contract(landscaping_contract)
                    )
                except (TypeError, ValueError) as exc:
                    raise DreamerProcessEnvError(
                        "negotiated landscaping contract metadata is invalid"
                    ) from exc
            self._row_is_first = np.zeros(self._num_envs, dtype=np.bool_)
            self._row_is_last = np.zeros(self._num_envs, dtype=np.bool_)
        except DreamerProcessEnvError:
            self._quiet_close_transport()
            raise
        except Exception as exc:
            self._quiet_close_transport()
            raise DreamerProcessEnvError(
                f"invalid negotiated Dreamer process environment: {exc}"
            ) from exc

    def _negotiate(self) -> Any:
        handshake = getattr(self._transport, "handshake", None)
        if callable(handshake):
            schema = handshake()
        else:
            schema = getattr(self._transport, "schema", None)
        if schema is None:
            raise DreamerProcessEnvError(
                "transport negotiation is required before space access"
            )
        if not hasattr(schema, "observation_projection") or not hasattr(
            schema, "action_projection"
        ):
            raise DreamerProcessEnvError("negotiated schema has no learner projections")
        return schema

    def _read_num_envs(self) -> int:
        value = getattr(self._transport, "num_envs", None)
        if value is None:
            try:
                value = len(self._transport)  # type: ignore[arg-type]
            except Exception as exc:
                raise DreamerProcessEnvError(
                    "transport does not expose a worker count"
                ) from exc
        num_envs = _strict_int(value, name="transport.num_envs")
        if num_envs < 1:
            raise DreamerProcessEnvError("transport.num_envs must be positive")
        return num_envs

    def _quiet_close_transport(self) -> None:
        close = getattr(self._transport, "close", None)
        if callable(close):
            try:
                close()
            except Exception:
                pass

    def _ensure_open(self) -> None:
        if self._closed:
            raise DreamerProcessEnvError("Dreamer process environment is closed")

    @property
    def demo_projection(self) -> Any | None:
        """Return the optional canonical-to-Dreamer demonstration adapter."""

        return self._demo_projection

    @property
    def demo_io_contract(self) -> Mapping[str, Any] | None:
        """Return the live canonical contract when workers advertise one."""

        if self._demo_projection is None:
            return None
        return self._demo_projection.contract

    @property
    def landscaping_contract(self) -> Mapping[str, Any] | None:
        """Return a detached read-only copy of the live worker contract."""

        if self._landscaping_contract is None:
            return None
        return MappingProxyType(dict(self._landscaping_contract))

    def __len__(self) -> int:
        return self._num_envs

    @property
    def obs_space(self) -> OrderedDict[str, Any]:
        if self._elements_obs_space is None:
            space_type = _load_elements_space()
            spaces = OrderedDict(
                (key, _to_elements_space(space, space_type))
                for key, _path, space in self._observation_leaves
            )
            spaces.update(
                (
                    ("reward", space_type(np.float32)),
                    ("is_first", space_type(bool)),
                    ("is_last", space_type(bool)),
                    ("is_terminal", space_type(bool)),
                )
            )
            self._elements_obs_space = spaces
        return _copy_space_mapping(self._elements_obs_space)

    @property
    def act_space(self) -> OrderedDict[str, Any]:
        if self._elements_act_space is None:
            space_type = _load_elements_space()
            spaces = OrderedDict(
                (key, _to_elements_space(space, space_type))
                for key, _path, space in self._action_leaves
            )
            spaces["reset"] = space_type(bool)
            self._elements_act_space = spaces
        return _copy_space_mapping(self._elements_act_space)

    @property
    def info(self) -> tuple[Mapping[str, Any] | None, ...]:
        return tuple(
            None if item is None else copy.deepcopy(dict(item)) for item in self._info
        )

    def _validate_observation(self, value: Any) -> OrderedDict[str, np.ndarray]:
        leaves = _collect_batched_leaves(
            value,
            name="transport observation",
            space=self._observation_root,
        )
        result: OrderedDict[str, np.ndarray] = OrderedDict()
        for key, path, space in self._observation_leaves:
            result[key] = _validate_array(
                leaves[path],
                name=f"transport observation {key!r}",
                shape=(self._num_envs, *tuple(space.shape)),
                dtype=np.dtype(space.dtype),
            )
        return result

    def _validate_action(self, value: Any) -> tuple[Any, np.ndarray]:
        expected = (*self._action_keys, "reset")
        mapping = _validate_mapping_order(value, expected, name="Dreamer action")
        leaves: dict[tuple[str | int, ...], np.ndarray] = {}
        for key, path, space in self._action_leaves:
            learner_dtype = (
                np.dtype(np.int32)
                if isinstance(space, gymnasium.spaces.Discrete)
                else np.dtype(space.dtype)
            )
            array = _validate_array(
                mapping[key],
                name=f"Dreamer action {key!r}",
                shape=(self._num_envs, *tuple(space.shape)),
                dtype=learner_dtype,
            )
            if isinstance(space, gymnasium.spaces.Discrete):
                if np.any(array < 0) or np.any(array >= int(space.n)):
                    raise DreamerProcessEnvError(
                        f"Dreamer action {key!r} contains an invalid Discrete value"
                    )
                leaves[path] = np.asarray(array, dtype=np.int64).copy()
            else:
                try:
                    in_bounds = np.logical_and(
                        array >= np.asarray(space.low),
                        array <= np.asarray(space.high),
                    )
                except Exception as exc:
                    raise DreamerProcessEnvError(
                        f"Dreamer action {key!r} cannot be compared with its bounds"
                    ) from exc
                if not bool(np.all(in_bounds)):
                    raise DreamerProcessEnvError(
                        f"Dreamer action {key!r} is outside its negotiated Box bounds"
                    )
                leaves[path] = array

        reset_mask = _validate_vector(
            mapping["reset"],
            name="Dreamer action 'reset'",
            num_envs=self._num_envs,
            dtype=np.bool_,
        )
        return _rebuild_batched_tree(self._action_root, leaves), reset_mask

    def _consume_result(
        self,
        result: Any,
        *,
        reset_mask: np.ndarray,
    ) -> OrderedDict[str, np.ndarray]:
        try:
            observation_value = result.observation
            reward_value = result.reward
            terminated_value = result.terminated
            truncated_value = result.truncated
            first_value = result.is_first
            episode_ids_value = result.episode_ids
            step_delta_value = result.step_delta
            info_value = result.info
        except Exception as exc:
            raise DreamerProcessEnvError(
                "transport result is missing a required field"
            ) from exc

        observations = self._validate_observation(observation_value)
        reward = _validate_vector(
            reward_value,
            name="transport reward",
            num_envs=self._num_envs,
            dtype=np.float32,
        )
        if not bool(np.all(np.isfinite(reward))):
            raise DreamerProcessEnvError("transport reward contains non-finite values")
        terminated = _validate_vector(
            terminated_value,
            name="transport terminated",
            num_envs=self._num_envs,
            dtype=np.bool_,
        )
        truncated = _validate_vector(
            truncated_value,
            name="transport truncated",
            num_envs=self._num_envs,
            dtype=np.bool_,
        )
        is_first = _validate_vector(
            first_value,
            name="transport is_first",
            num_envs=self._num_envs,
            dtype=np.bool_,
        )
        _validate_vector(
            episode_ids_value,
            name="transport episode_ids",
            num_envs=self._num_envs,
            dtype=np.int64,
        )
        step_delta = _validate_vector(
            step_delta_value,
            name="transport step_delta",
            num_envs=self._num_envs,
            dtype=np.int64,
        )
        if np.any(step_delta < 0):
            raise DreamerProcessEnvError(
                "transport step_delta contains a negative value"
            )
        info = _copy_info(info_value, num_envs=self._num_envs)

        if np.any(reset_mask & ~is_first):
            raise DreamerProcessEnvError("reset rows must be marked is_first")
        if np.any(~reset_mask & is_first):
            raise DreamerProcessEnvError("continuing rows must not be marked is_first")
        if np.any(reset_mask & (reward != np.float32(0.0))):
            raise DreamerProcessEnvError("reset rows must have zero reward")
        if np.any(reset_mask & (terminated | truncated)):
            raise DreamerProcessEnvError("reset rows must not be terminal")

        is_last = np.logical_or(terminated, truncated)
        is_terminal = np.array(terminated, copy=True)
        self._info = info
        self._row_is_first = np.array(is_first, copy=True)
        self._row_is_last = np.array(is_last, copy=True)

        output: OrderedDict[str, np.ndarray] = OrderedDict(
            (key, np.array(array, copy=True)) for key, array in observations.items()
        )
        output["reward"] = np.array(reward, copy=True)
        output["is_first"] = np.array(is_first, copy=True)
        output["is_last"] = np.array(is_last, copy=True)
        output["is_terminal"] = np.array(is_terminal, copy=True)
        return output

    def reset(self) -> OrderedDict[str, np.ndarray]:
        """Return one unstepped all-row reset frame from the transport."""

        self._ensure_open()
        reset_mask = np.ones(self._num_envs, dtype=np.bool_)
        try:
            result = self._transport.reset()
        except Exception as exc:
            raise DreamerProcessEnvError("transport reset failed") from exc
        return self._consume_result(result, reset_mask=reset_mask)

    def step(self, action: Mapping[str, np.ndarray]) -> OrderedDict[str, np.ndarray]:
        """Reset true rows, step false rows once, and return one barrier frame."""

        self._ensure_open()
        transport_action, reset_mask = self._validate_action(action)
        try:
            result = self._transport.reset_and_step(transport_action, reset_mask)
        except Exception as exc:
            raise DreamerProcessEnvError("transport reset_and_step failed") from exc
        return self._consume_result(result, reset_mask=reset_mask)

    @property
    def row_episode_states(self) -> tuple[DreamerProcessRowEpisodeState, ...]:
        """Return detached callback identity snapshots, never learner fields."""

        try:
            raw_states = getattr(self._transport, "row_episode_states")
            if callable(raw_states):
                raw_states = raw_states()
        except Exception as exc:
            raise DreamerProcessEnvError(
                "transport row states are unavailable"
            ) from exc
        if isinstance(raw_states, (str, bytes, bytearray)) or not isinstance(
            raw_states, Sequence
        ):
            raise DreamerProcessEnvError("transport row states must be a sequence")
        if len(raw_states) != self._num_envs:
            raise DreamerProcessEnvError(
                f"transport row states must have length {self._num_envs}"
            )
        result: list[DreamerProcessRowEpisodeState] = []
        for index, state in enumerate(raw_states):
            try:
                env_id = _strict_int(state.env_id, name=f"row {index}.env_id")
                episode_id = _strict_int(
                    state.episode_id,
                    name=f"row {index}.episode_id",
                )
                step_index = _strict_int(
                    state.step_index,
                    name=f"row {index}.step_index",
                )
            except AttributeError as exc:
                raise DreamerProcessEnvError(
                    f"transport row {index} has malformed identity"
                ) from exc
            if step_index < 0:
                raise DreamerProcessEnvError(
                    f"transport row {index} has a negative step index"
                )
            result.append(
                DreamerProcessRowEpisodeState(
                    env_id=env_id,
                    episode_id=episode_id,
                    step_index=step_index,
                    is_first=bool(self._row_is_first[index]),
                    is_last=bool(self._row_is_last[index]),
                )
            )
        return tuple(result)

    def is_running(self) -> bool:
        """Delegate the public worker/process liveness probe."""

        try:
            value = getattr(self._transport, "is_running")
            if callable(value):
                value = value()
        except Exception as exc:
            raise DreamerProcessEnvError("transport liveness probe failed") from exc
        if not isinstance(value, (bool, np.bool_)):
            raise DreamerProcessEnvError("transport liveness probe did not return bool")
        return bool(value)

    def close(self) -> None:
        """Idempotently delegate worker cleanup and process reaping."""

        if self._closed:
            return
        self._closed = True
        close = getattr(self._transport, "close", None)
        if not callable(close):
            raise DreamerProcessEnvError("transport does not expose close()")
        try:
            close()
        except Exception as exc:
            raise DreamerProcessEnvError("transport cleanup failed") from exc

    def __enter__(self) -> "DreamerProcessEnv":
        self._ensure_open()
        return self

    def __exit__(self, exc_type: Any, exc_value: Any, traceback: Any) -> bool:
        self.close()
        return False
