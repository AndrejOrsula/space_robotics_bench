from copy import deepcopy
from typing import TYPE_CHECKING, Any, cast

import gymnasium
import numpy as np
import torch
from gymnasium.core import ObservationWrapper, ObsType, WrapperObsType
from gymnasium.vector.utils import batch_space
from gymnasium.wrappers import TransformObservation

from srb.core.env.introspect import runtime_num_envs
from srb.core.env.runtime.observation_layout import ObservationLayout
from srb.wrappers._utils import get_wrapper_attr as _get_wrapper_attr

if TYPE_CHECKING:
    from srb._typing import AnyEnv


_MISSING = object()


def _get_local_attr(env: Any, name: str) -> Any:
    """Read an attribute declared by this wrapper, without traversing ``env``."""

    instance_attrs = getattr(env, "__dict__", {})
    if isinstance(instance_attrs, dict) and name in instance_attrs:
        return instance_attrs[name]
    class_attr = getattr(type(env), name, _MISSING)
    if class_attr is not _MISSING:
        return getattr(env, name)
    return _MISSING


def _has_local_attr(env: Any, name: str) -> bool:
    instance_attrs = getattr(env, "__dict__", {})
    if isinstance(instance_attrs, dict) and name in instance_attrs:
        return True
    return getattr(type(env), name, _MISSING) is not _MISSING


def _spaces_equal(left: gymnasium.Space[Any], right: gymnasium.Space[Any]) -> bool:
    try:
        equal = left == right
    except (TypeError, ValueError):
        return False
    return isinstance(equal, (bool, np.bool_)) and bool(equal)


def _is_valid_batch(
    single: gymnasium.Space[Any],
    batched: gymnasium.Space[Any],
    num_envs: int,
) -> bool:
    try:
        expected = batch_space(single, num_envs)
    except (TypeError, ValueError):
        return False
    return _spaces_equal(expected, batched)


def _unbatch_space(space: gymnasium.Space[Any], num_envs: int) -> gymnasium.Space[Any]:
    """Recover a single space only when batching it reproduces the input."""

    if isinstance(space, gymnasium.spaces.Box):
        if not space.shape or space.shape[0] != num_envs:
            raise ValueError(
                f"Box shape {space.shape} does not start with num_envs={num_envs}"
            )
        dtype = space.dtype
        if dtype is None:  # pragma: no cover - Gym Box always has a dtype
            raise TypeError("batched Box space has no dtype")
        return gymnasium.spaces.Box(
            low=np.array(space.low[0], copy=True),
            high=np.array(space.high[0], copy=True),
            dtype=cast(type[np.floating[Any]] | type[np.integer[Any]], dtype.type),
        )
    if isinstance(space, gymnasium.spaces.Dict):
        return gymnasium.spaces.Dict(
            {
                key: _unbatch_space(child, num_envs)
                for key, child in space.spaces.items()
            }
        )
    if isinstance(space, gymnasium.spaces.Tuple):
        return gymnasium.spaces.Tuple(
            tuple(_unbatch_space(child, num_envs) for child in space.spaces)
        )
    if isinstance(space, gymnasium.spaces.MultiDiscrete):
        if space.nvec.ndim != 1 or space.nvec.shape[0] != num_envs:
            raise ValueError(
                f"MultiDiscrete shape {space.nvec.shape} does not start with "
                f"num_envs={num_envs}"
            )
        if not np.all(space.nvec == space.nvec[0]):
            raise ValueError("MultiDiscrete batch has differing per-env bounds")
        return gymnasium.spaces.Discrete(int(space.nvec[0]))
    raise TypeError(f"cannot derive a single space from {space!r}")


def _effective_single_from_batch(
    batched: gymnasium.Space[Any],
    num_envs: int,
    fallback: Any,
    *,
    kind: str,
) -> gymnasium.Space[Any]:
    if (
        fallback is not _MISSING
        and isinstance(fallback, gymnasium.Space)
        and _is_valid_batch(fallback, batched, num_envs)
    ):
        return deepcopy(fallback)

    try:
        candidate = _unbatch_space(batched, num_envs)
    except (TypeError, ValueError) as exc:
        detail = str(exc)
    else:
        if _is_valid_batch(candidate, batched, num_envs):
            return candidate
        detail = "derived single space does not reproduce the effective batch"

    raise ValueError(
        f"effective {kind} space {batched!r} cannot be derived as a "
        f"num_envs={num_envs} batch ({detail})"
    )


def _effective_observation_single(
    env: Any,
    observation_space: Any,
    num_envs: int,
) -> gymnasium.spaces.Dict:
    local_single = _get_local_attr(env, "single_observation_space")
    fallback = (
        local_single
        if local_single is not _MISSING
        else _get_wrapper_attr(env, "single_observation_space", _MISSING)
    )

    if observation_space is not _MISSING and observation_space is not None:
        if not isinstance(observation_space, gymnasium.spaces.Dict):
            raise TypeError(
                "MergeDictObsWrapper requires an effective observation space "
                f"with Dict structure, got {observation_space}"
            )
        if num_envs > 0:
            single = _effective_single_from_batch(
                observation_space,
                num_envs,
                fallback,
                kind="observation",
            )
        else:
            single = deepcopy(observation_space)
    elif fallback is not _MISSING:
        single = deepcopy(fallback)
    else:
        raise AttributeError("wrapped environment does not expose observation_space")

    if not isinstance(single, gymnasium.spaces.Dict):
        raise TypeError(
            "MergeDictObsWrapper requires an effective observation space "
            f"with Dict structure, got {single}"
        )
    return single


def _effective_action_spaces(
    env: Any,
    action_space: Any,
    num_envs: int,
) -> tuple[gymnasium.Space[Any], gymnasium.Space[Any]]:
    local_single = _get_local_attr(env, "single_action_space")
    fallback = (
        local_single
        if local_single is not _MISSING
        else _get_wrapper_attr(env, "single_action_space", _MISSING)
    )

    if action_space is not _MISSING and action_space is not None:
        if not isinstance(action_space, gymnasium.Space):
            raise TypeError(
                f"effective action space must be a Gym space, got {action_space}"
            )
        if num_envs > 0:
            single = _effective_single_from_batch(
                action_space,
                num_envs,
                fallback,
                kind="action",
            )
            batched = deepcopy(action_space)
        else:
            single = deepcopy(action_space)
            batched = deepcopy(action_space)
    elif fallback is not _MISSING and isinstance(fallback, gymnasium.Space):
        single = deepcopy(fallback)
        batched = batch_space(single, num_envs) if num_envs > 0 else deepcopy(single)
    else:
        raise AttributeError("wrapped environment does not expose an action space")

    if not isinstance(single, gymnasium.Space) or not isinstance(
        batched, gymnasium.Space
    ):
        raise TypeError(
            f"effective action spaces must be Gym spaces, got single={single!r}, "
            f"batched={batched!r}"
        )
    if num_envs > 0 and not _is_valid_batch(single, batched, num_envs):
        raise ValueError(
            "effective action spaces are inconsistent: "
            f"single={single!r}, batched={batched!r}, num_envs={num_envs}"
        )
    return single, batched


def _observation_space_changed(wrapper: gymnasium.Wrapper[Any, Any, Any, Any]) -> bool:
    inner = getattr(wrapper, "env", _MISSING)
    outer_space = getattr(wrapper, "observation_space", _MISSING)
    inner_space = getattr(inner, "observation_space", _MISSING)
    return (
        isinstance(outer_space, gymnasium.Space)
        and isinstance(inner_space, gymnasium.Space)
        and not _spaces_equal(outer_space, inner_space)
    )


def _terminal_projection(
    env: Any,
) -> tuple[Any, tuple[gymnasium.ObservationWrapper[Any, Any, Any], ...]]:
    """Find the terminal snapshot owner and safe transforms above it."""

    wrappers: list[gymnasium.Wrapper[Any, Any, Any, Any]] = []
    current = env
    owner: Any = _MISSING
    while isinstance(current, gymnasium.Wrapper):
        if _has_local_attr(current, "last_final_observation"):
            owner = current
            break
        wrappers.append(current)
        current = current.env
    else:
        if _has_local_attr(current, "last_final_observation"):
            owner = current

    projection: list[gymnasium.ObservationWrapper[Any, Any, Any]] = []
    if owner is not _MISSING:
        for wrapper in reversed(wrappers):
            if isinstance(wrapper, TransformObservation):
                projection.append(wrapper)
            elif isinstance(wrapper, gymnasium.ObservationWrapper):
                raise RuntimeError(
                    "MergeDictObsWrapper cannot project terminal observations "
                    f"through stateful {type(wrapper).__name__}; it must expose "
                    "its own last_final_observation"
                )
            elif _observation_space_changed(wrapper):
                raise RuntimeError(
                    "MergeDictObsWrapper cannot project terminal observations "
                    f"through observation-transforming {type(wrapper).__name__}"
                )
    return owner, tuple(projection)


class MergeDictObsWrapper(ObservationWrapper):
    def __init__(self, env: "AnyEnv"):
        super().__init__(env)

        (
            self._terminal_owner,
            self._terminal_projection_wrappers,
        ) = _terminal_projection(env)

        num_envs = runtime_num_envs(env)
        single_observation_space = _effective_observation_single(
            env,
            getattr(env, "observation_space", _MISSING),
            num_envs,
        )

        self._observation_layout = ObservationLayout.from_spaces(
            single_observation_space
        )
        self.dict_keys: tuple[str, ...] = tuple(single_observation_space.spaces.keys())
        self.single_observation_space = self._observation_layout.merged_space()

        self._is_vector_env = num_envs > 0
        if self._is_vector_env:
            self.num_envs = num_envs
        self.observation_space = (
            batch_space(self.single_observation_space, num_envs)
            if self._is_vector_env
            else deepcopy(self.single_observation_space)
        )
        self._observation_space = self.observation_space

        action_space = getattr(env, "action_space", _MISSING)
        self.single_action_space, self.action_space = _effective_action_spaces(
            env,
            action_space,
            num_envs,
        )
        self._action_space = self.action_space

    def observation(self, observation: ObsType) -> WrapperObsType:  # type: ignore[misc]
        if not isinstance(observation, dict):
            raise TypeError(
                f"MergeDictObsWrapper expected a mapping observation, got {type(observation).__name__}"
            )
        result = self._observation_layout.flatten_torch(
            observation,
            batched=self._is_vector_env if self._is_vector_env else None,
        )
        if not isinstance(result, torch.Tensor):  # pragma: no cover - layout contract
            raise TypeError(
                f"observation layout returned {type(result).__name__}, expected Tensor"
            )
        return result  # type: ignore[return-value]

    @property
    def last_final_observation(self):
        if self._terminal_owner is _MISSING:
            terminal = _get_wrapper_attr(self.env, "last_final_observation")
        else:
            terminal = _get_local_attr(self._terminal_owner, "last_final_observation")
            if terminal is _MISSING:
                terminal = _get_wrapper_attr(self.env, "last_final_observation")
        if terminal is None:
            return None
        for wrapper in self._terminal_projection_wrappers:
            terminal = wrapper.observation(terminal)
        return self._observation_layout.flatten_torch(
            terminal,
            batched=self._is_vector_env if self._is_vector_env else None,
        )

    @property
    def last_final_observation_ids(self):
        ids = _get_wrapper_attr(self.env, "last_final_observation_ids")
        if ids is None:
            return None
        return ids.clone() if isinstance(ids, torch.Tensor) else ids

    @property
    def last_final_observation_mask(self):
        mask = _get_wrapper_attr(self.env, "last_final_observation_mask")
        if mask is None:
            return None
        return mask.clone() if isinstance(mask, torch.Tensor) else mask
