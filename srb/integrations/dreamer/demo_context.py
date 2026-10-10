"""Stage validated native demo rows with Dreamer recurrent context.

Copies rows before provider calls and derives context from demonstrated
actions. This module has no replay dependency.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from typing import Any

import numpy as np

from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    ResourceLimitError,
    checked_shape_nbytes,
)

__all__ = ["stage_demo_replay_rows"]


_POLICY_ONLY_FIELDS = frozenset({"finite"})
_PROVIDER_REPLAY_FIELDS = frozenset({"consec", "stepid"})
_DRIVER_FIELDS = frozenset({"reset"})


class _ArrayBudget:
    """Track detached arrays under the existing memory ceilings."""

    __slots__ = ("total",)

    def __init__(self) -> None:
        self.total = 0

    def add(self, nbytes: int, *, name: str) -> None:
        if nbytes > MAX_ARRAY_BYTES:
            raise ResourceLimitError(
                f"{name} array bytes", limit=MAX_ARRAY_BYTES, observed=nbytes
            )
        observed = self.total + nbytes
        if observed > MAX_IN_MEMORY_FILE_BYTES:
            raise ResourceLimitError(
                "Dreamer staged array bytes",
                limit=MAX_IN_MEMORY_FILE_BYTES,
                observed=observed,
            )
        self.total = observed


def _space_mapping(value: Any, *, name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping) or not value:
        raise TypeError(f"{name} must be a non-empty mapping of native spaces")
    result: dict[str, Any] = {}
    for key, space in value.items():
        if not isinstance(key, str) or not key:
            raise TypeError(f"{name} keys must be non-empty strings")
        result[key] = space
    return result


def _numeric_dtype(dtype: np.dtype[Any], *, name: str) -> None:
    if dtype.kind == "O" or dtype.kind in "SU":
        raise TypeError(f"{name} must use numeric or boolean dtype")
    if dtype.kind == "c":
        raise TypeError(f"{name} must use real numeric dtype")
    if dtype.kind not in "biuf":
        raise TypeError(f"{name} has unsupported dtype {dtype}")


def _space_schema(space: Any, *, name: str) -> tuple[tuple[int, ...], np.dtype[Any]]:
    try:
        raw_shape = tuple(space.shape)
        shape = tuple(int(dimension) for dimension in raw_shape)
        dtype = np.dtype(space.dtype)
    except (AttributeError, TypeError, ValueError) as exc:
        raise TypeError(f"{name} does not declare a valid shape and dtype") from exc
    if shape != raw_shape or any(dimension < 0 for dimension in shape):
        raise ValueError(f"{name} declares an invalid shape {raw_shape!r}")
    _numeric_dtype(dtype, name=name)
    try:
        checked_shape_nbytes(
            shape,
            dtype,
            resource=name,
            max_bytes=MAX_ARRAY_BYTES,
        )
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} declares an invalid array: {exc}") from exc
    return shape, dtype


def _metadata_shape_dtype(
    value: Any, *, name: str
) -> tuple[tuple[int, ...], np.dtype[Any]]:
    try:
        shape = tuple(int(dimension) for dimension in tuple(value.shape))
        dtype = np.dtype(value.dtype)
    except (AttributeError, TypeError, ValueError):
        try:
            array = np.asarray(value)
        except (TypeError, ValueError) as exc:
            raise TypeError(f"{name} is not array-like") from exc
        shape = tuple(int(dimension) for dimension in array.shape)
        dtype = np.dtype(array.dtype)
    if any(dimension < 0 for dimension in shape):
        raise ValueError(f"{name} has an invalid shape {shape!r}")
    _numeric_dtype(dtype, name=name)
    try:
        checked_shape_nbytes(
            shape,
            dtype,
            resource=name,
            max_bytes=MAX_ARRAY_BYTES,
        )
    except ResourceLimitError:
        raise
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} has an invalid array: {exc}") from exc
    return shape, dtype


def _as_host_array(value: Any, *, name: str) -> np.ndarray[Any, Any]:
    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} is not host-readable array data") from exc
    actual_shape, actual_dtype = _metadata_shape_dtype(array, name=name)
    if tuple(array.shape) != actual_shape or np.dtype(array.dtype) != actual_dtype:
        raise TypeError(f"{name} changed schema while being read")
    return array


def _check_declared_bounds(
    array: np.ndarray[Any, Any],
    *,
    space: Any,
    expected_shape: tuple[int, ...],
    expected_dtype: np.dtype[Any],
    name: str,
) -> None:
    for bound_name in ("low", "high"):
        bound = getattr(space, bound_name, None)
        if bound is None:
            continue
        try:
            bound_array = np.asarray(bound, dtype=expected_dtype)
            bound_array = np.broadcast_to(bound_array, expected_shape)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{name} declared {bound_name} bound has wrong shape"
            ) from exc
        comparison = array < bound_array if bound_name == "low" else array > bound_array
        if np.any(comparison):
            raise ValueError(f"{name} is outside its declared {bound_name} bound")


def _copy_array(
    value: Any,
    *,
    name: str,
    space: Any,
    budget: _ArrayBudget,
    check_bounds: bool,
) -> np.ndarray[Any, Any]:
    expected_shape, expected_dtype = _space_schema(space, name=f"space for {name}")
    actual_shape, actual_dtype = _metadata_shape_dtype(value, name=name)
    if actual_shape != expected_shape:
        raise ValueError(f"{name} has shape {actual_shape}, expected {expected_shape}")
    if actual_dtype != expected_dtype:
        raise TypeError(f"{name} has dtype {actual_dtype}, expected {expected_dtype}")
    array = _as_host_array(value, name=name)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains non-finite values")
    if check_bounds:
        _check_declared_bounds(
            array,
            space=space,
            expected_shape=expected_shape,
            expected_dtype=expected_dtype,
            name=name,
        )
    budget.add(int(array.nbytes), name=name)
    return np.array(array, copy=True, order="C")


def _config_replay_context(agent: Any) -> Any:
    config = getattr(agent, "config", None)
    if config is None:
        return None
    if isinstance(config, Mapping):
        return config.get("replay_context")
    return getattr(config, "replay_context", None)


def _validate_ext_space(
    agent: Any,
) -> dict[str, tuple[Any, tuple[int, ...], np.dtype[Any]]]:
    try:
        raw_ext_space = agent.ext_space
    except AttributeError:
        # The pinned embodied JAX wrapper exposes the model's ``ext_space``
        # through ``agent.model``, not the outer execution wrapper.
        model = getattr(agent, "model", None)
        try:
            raw_ext_space = model.ext_space
        except AttributeError as exc:
            raise TypeError(
                "Dreamer agent must expose ext_space for context staging"
            ) from exc
    ext_space = _space_mapping(raw_ext_space, name="agent.ext_space")
    schemas = {
        key: (space, *_space_schema(space, name=f"agent.ext_space[{key!r}]"))
        for key, space in ext_space.items()
    }
    context_fields = tuple(key for key in schemas if key not in _PROVIDER_REPLAY_FIELDS)
    if not context_fields:
        raise ValueError("agent.ext_space declares no recurrent context fields")
    replay_context = _config_replay_context(agent)
    if replay_context is not None and not bool(replay_context):
        raise ValueError("Dreamer replay context is disabled in the agent config")
    return schemas


def _validate_rows(
    rows: Iterable[Mapping[str, Any]],
    *,
    obs_space: Mapping[str, Any],
    act_space: Mapping[str, Any],
    is_running: Callable[[], bool] | None,
) -> tuple[
    list[dict[str, np.ndarray[Any, Any]]],
    list[dict[str, np.ndarray[Any, Any]]],
    list[dict[str, np.ndarray[Any, Any]]],
    _ArrayBudget,
]:
    collisions = set(obs_space) & set(act_space)
    if collisions:
        raise ValueError(
            f"observation and action spaces share native fields: {sorted(collisions)}"
        )
    expected_keys = set(obs_space) | set(act_space)
    budget = _ArrayBudget()
    detached_rows: list[dict[str, np.ndarray[Any, Any]]] = []
    observations: list[dict[str, np.ndarray[Any, Any]]] = []
    actions: list[dict[str, np.ndarray[Any, Any]]] = []
    previous_last = False

    for index, source_row in enumerate(rows):
        if is_running is not None and not bool(is_running()):
            raise RuntimeError("Dreamer demonstration context staging cancelled")
        if not isinstance(source_row, Mapping):
            raise TypeError(f"Dreamer demo row {index} must be a mapping")
        if not source_row:
            raise ValueError(f"Dreamer demo row {index} must not be empty")
        row_keys = set(source_row)
        missing = expected_keys - row_keys
        if missing:
            raise ValueError(
                f"row {index} is missing native fields: {sorted(missing, key=repr)}"
            )
        extras = row_keys - expected_keys
        if extras:
            raise ValueError(
                f"row {index} contains unexpected native fields: "
                f"{sorted(extras, key=repr)}"
            )

        row_observation: dict[str, np.ndarray[Any, Any]] = {}
        for key, space in obs_space.items():
            row_observation[key] = _copy_array(
                source_row[key],
                name=f"row {index}.{key}",
                space=space,
                budget=budget,
                check_bounds=False,
            )
        row_actions: dict[str, np.ndarray[Any, Any]] = {}
        for key, space in act_space.items():
            row_actions[key] = _copy_array(
                source_row[key],
                name=f"row {index}.{key}",
                space=space,
                budget=budget,
                check_bounds=True,
            )

        current_first = (
            bool(row_observation["is_first"].item())
            if "is_first" in row_observation
            else False
        )
        current_last = (
            bool(row_observation["is_last"].item())
            if "is_last" in row_observation
            else False
        )
        if index == 0 and "is_first" in row_observation and not current_first:
            raise ValueError("row 0 must be an is_first reset row")
        if index > 0:
            if previous_last and not current_first:
                raise ValueError(
                    f"row {index} continues after an is_last row without reset"
                )
            if current_first and not previous_last:
                raise ValueError(
                    f"row {index} starts an episode before the previous row ended"
                )
        if current_first and "reward" in row_observation:
            if float(row_observation["reward"].item()) != 0.0:
                raise ValueError(f"row {index} reset reward must be zero")
        if (
            "is_terminal" in row_observation
            and bool(row_observation["is_terminal"].item())
            and "is_last" in row_observation
            and not current_last
        ):
            raise ValueError(f"row {index} terminal flag requires is_last")

        detached_rows.append({**row_observation, **row_actions})
        observations.append(row_observation)
        actions.append(row_actions)
        previous_last = current_last

    if not detached_rows:
        raise ValueError("Dreamer demo rows must not be empty")
    return detached_rows, observations, actions, budget


def _admit_context_budget(
    budget: _ArrayBudget,
    *,
    row_count: int,
    ext_space: Mapping[str, tuple[Any, tuple[int, ...], np.dtype[Any]]],
) -> None:
    projected = budget.total
    for key, (_space, shape, dtype) in ext_space.items():
        if key in _PROVIDER_REPLAY_FIELDS:
            continue
        per_row = checked_shape_nbytes(
            shape,
            dtype,
            resource=f"agent.ext_space[{key!r}]",
            max_bytes=MAX_ARRAY_BYTES,
        )
        projected += row_count * per_row
    if projected > MAX_IN_MEMORY_FILE_BYTES:
        raise ResourceLimitError(
            "Dreamer staged array bytes",
            limit=MAX_IN_MEMORY_FILE_BYTES,
            observed=projected,
        )


def _jax_device_put(value: np.ndarray[Any, Any], template: Any) -> Any:
    try:
        import jax
    except (ImportError, ModuleNotFoundError) as exc:
        raise RuntimeError(
            "Dreamer demonstration context requires the configured JAX provider"
        ) from exc
    sharding = getattr(template, "sharding", None)
    if sharding is None:
        return jax.device_put(value)
    return jax.device_put(value, sharding)


def _carry_action_replacement(
    action: np.ndarray[Any, Any],
    *,
    template: Any,
    name: str,
) -> Any:
    template_shape, template_dtype = _metadata_shape_dtype(template, name=name)
    expected_shape = tuple(action.shape)
    if template_dtype != action.dtype:
        raise TypeError(f"{name} has dtype {template_dtype}, expected {action.dtype}")
    if template_shape == expected_shape:
        value = action
    elif template_shape == (1, *expected_shape):
        value = action[None]
    else:
        raise ValueError(
            f"{name} has shape {template_shape}, expected "
            f"{expected_shape} or {(1, *expected_shape)}"
        )
    return _jax_device_put(value, template)


def _replace_carry_actions(
    carry: Any,
    *,
    actions: Mapping[str, np.ndarray[Any, Any]],
) -> Any:
    if not isinstance(carry, (tuple, list)) or len(carry) != 4:
        raise TypeError("Dreamer policy carry must be a four-part tuple/list")
    previous = carry[3]
    if not isinstance(previous, Mapping):
        raise TypeError("Dreamer policy carry previous actions must be a mapping")
    if set(previous) != set(actions):
        raise ValueError(
            "Dreamer policy carry action keys differ from native action keys: "
            f"carry={sorted(previous, key=repr)}, "
            f"native={sorted(actions, key=repr)}"
        )

    replaced: dict[str, Any] = {}
    for key, action in actions.items():
        value = previous[key]
        if isinstance(value, (list, tuple)):
            if len(value) != 1:
                raise ValueError(
                    f"Dreamer carry action {key!r} must contain one local row"
                )
            replacement = _carry_action_replacement(
                action,
                template=value[0],
                name=f"Dreamer carry action {key!r}",
            )
            replaced[key] = [replacement] if isinstance(value, list) else (replacement,)
        else:
            replaced[key] = _carry_action_replacement(
                action,
                template=value,
                name=f"Dreamer carry action {key!r}",
            )

    if isinstance(carry, tuple):
        return (*carry[:3], replaced)
    return [*carry[:3], replaced]


def _policy_observation(
    observation: Mapping[str, np.ndarray[Any, Any]],
) -> dict[str, np.ndarray[Any, Any]]:
    return {
        key: np.ascontiguousarray(value[None], dtype=value.dtype)
        for key, value in observation.items()
    }


def _stage_policy_context(
    policy_result: Any,
    *,
    row_index: int,
    ext_space: Mapping[str, tuple[Any, tuple[int, ...], np.dtype[Any]]],
    budget: _ArrayBudget,
) -> dict[str, np.ndarray[Any, Any]]:
    if not isinstance(policy_result, tuple) or len(policy_result) != 3:
        raise TypeError(f"Dreamer policy result for row {row_index} is invalid")
    _carry, _sampled_action, outputs = policy_result
    if not isinstance(outputs, Mapping):
        raise TypeError(f"Dreamer policy outputs for row {row_index} must be a mapping")

    context_fields = tuple(
        key for key in ext_space if key not in _PROVIDER_REPLAY_FIELDS
    )
    unknown = set(outputs) - set(ext_space) - _POLICY_ONLY_FIELDS
    if unknown:
        raise ValueError(
            f"Dreamer policy returned undeclared context for row {row_index}: "
            f"{sorted(unknown, key=repr)}"
        )
    missing = set(context_fields) - set(outputs)
    if missing:
        raise ValueError(
            f"Dreamer policy did not return all declared context for row {row_index}: "
            f"{sorted(missing, key=repr)}"
        )

    staged: dict[str, np.ndarray[Any, Any]] = {}
    for key in context_fields:
        space, shape, dtype = ext_space[key]
        value = outputs[key]
        actual_shape, actual_dtype = _metadata_shape_dtype(
            value,
            name=f"policy output row {row_index}.{key}",
        )
        if actual_shape == (1, *shape):
            try:
                value = np.asarray(value)[0]
            except (TypeError, ValueError) as exc:
                raise TypeError(
                    f"policy output row {row_index}.{key} is not host-readable"
                ) from exc
        elif actual_shape != shape:
            raise ValueError(
                f"policy output row {row_index}.{key} has shape {actual_shape}, "
                f"expected {shape} or {(1, *shape)}"
            )
        if actual_dtype != dtype:
            raise TypeError(
                f"policy output row {row_index}.{key} has dtype {actual_dtype}, "
                f"expected {dtype}"
            )
        staged[key] = _copy_array(
            value,
            name=f"policy output row {row_index}.{key}",
            space=space,
            budget=budget,
            check_bounds=False,
        )
    return staged


def stage_demo_replay_rows(
    agent: Any,
    rows: Iterable[Mapping[str, Any]],
    *,
    obs_space: Mapping[str, Any],
    act_space: Mapping[str, Any],
    is_running: Callable[[], bool] | None = None,
) -> tuple[dict[str, np.ndarray[Any, Any]], ...]:
    """Return detached rows with recurrent context derived from demo actions.

    Rows contain unbatched observation/action values; the first observation
    must reset the episode. Provider calls start with zero prior actions, then
    carry each demonstrated action forward. Sampled actions are discarded.
    Rows are copied and validated, and context storage is budgeted, before
    ``init_policy``. Inputs are not mutated.
    """

    if not callable(getattr(agent, "init_policy", None)):
        raise TypeError("Dreamer agent must expose init_policy for context staging")
    if not callable(getattr(agent, "policy", None)):
        raise TypeError("Dreamer agent must expose policy for context staging")
    if is_running is not None and not callable(is_running):
        raise TypeError("is_running must be callable when provided")
    observations_space = _space_mapping(obs_space, name="obs_space")
    actions_space = _space_mapping(act_space, name="act_space")
    input_space_keys = set(observations_space) | set(actions_space)
    driver_fields = _DRIVER_FIELDS & input_space_keys
    if driver_fields:
        raise ValueError(
            "native observation/action spaces must omit driver fields: "
            f"{sorted(driver_fields, key=repr)}"
        )
    provider_fields = _PROVIDER_REPLAY_FIELDS & input_space_keys
    if provider_fields:
        raise ValueError(
            "native observation/action spaces must omit provider replay fields: "
            f"{sorted(provider_fields, key=repr)}"
        )
    for key, space in observations_space.items():
        _space_schema(space, name=f"obs_space[{key!r}]")
    for key, space in actions_space.items():
        _space_schema(space, name=f"act_space[{key!r}]")
    ext_space = _validate_ext_space(agent)
    context_input_fields = set(ext_space) & input_space_keys
    if context_input_fields:
        raise ValueError(
            "native observation/action spaces must omit declared context fields: "
            f"{sorted(context_input_fields, key=repr)}"
        )

    def check_running() -> None:
        if is_running is not None and not bool(is_running()):
            raise RuntimeError("Dreamer demonstration context staging cancelled")

    check_running()
    detached_rows, observations, actions, budget = _validate_rows(
        rows,
        obs_space=observations_space,
        act_space=actions_space,
        is_running=is_running,
    )
    check_running()
    _admit_context_budget(
        budget,
        row_count=len(detached_rows),
        ext_space=ext_space,
    )
    check_running()

    carry = agent.init_policy(1)
    zero_actions = {key: np.zeros_like(value) for key, value in actions[0].items()}
    carry = _replace_carry_actions(carry, actions=zero_actions)

    for row_index, (row, observation, row_actions) in enumerate(
        zip(detached_rows, observations, actions, strict=True)
    ):
        check_running()
        result = agent.policy(carry, _policy_observation(observation), mode="train")
        check_running()
        context = _stage_policy_context(
            result,
            row_index=row_index,
            ext_space=ext_space,
            budget=budget,
        )
        row.update(context)
        carry = _replace_carry_actions(result[0], actions=row_actions)
    check_running()
    return tuple(detached_rows)
