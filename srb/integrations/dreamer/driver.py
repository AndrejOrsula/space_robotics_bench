from dataclasses import dataclass
from typing import TYPE_CHECKING

import elements
import numpy as np
from embodied import Driver

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg


# Callback-only flat scalar keys. The reserved `srb_` prefix avoids learner
# observation collisions, and omitting `log/` prevents Dreamer's log filtering
# from discarding replay attribution metadata.
CALLBACK_ENV_ID_KEY = "srb_env_id"
CALLBACK_EPISODE_ID_KEY = "srb_episode_id"
CALLBACK_STEP_INDEX_KEY = "srb_step_index"
CALLBACK_ROW_METADATA_KEYS = frozenset(
    (
        CALLBACK_ENV_ID_KEY,
        CALLBACK_EPISODE_ID_KEY,
        CALLBACK_STEP_INDEX_KEY,
    )
)


@dataclass(frozen=True, slots=True)
class _CallbackRowState:
    env_id: int
    episode_id: int
    step_index: int
    is_first: bool
    is_last: bool


def _coerce_callback_row_state(value: object) -> _CallbackRowState:
    try:
        fields = {
            "env_id": getattr(value, "env_id"),
            "episode_id": getattr(value, "episode_id"),
            "step_index": getattr(value, "step_index"),
            "is_first": getattr(value, "is_first"),
            "is_last": getattr(value, "is_last"),
        }
    except (AttributeError, TypeError) as exc:
        raise TypeError("Dreamer callback row state has an invalid type") from exc
    for name in ("env_id", "episode_id", "step_index"):
        if type(fields[name]) is not int:
            raise TypeError("Dreamer callback row state has an invalid type")
    for name in ("is_first", "is_last"):
        if type(fields[name]) is not bool:
            raise TypeError("Dreamer callback row state has an invalid type")
    return _CallbackRowState(**fields)


class DriverParallelEnv(Driver):
    def __init__(self, env: "AnyEnvCfg", num_envs: int, **kwargs):
        self.kwargs = kwargs
        self.length = num_envs
        self.env = env
        self.act_space = self.env.act_space  # type: ignore[attr-defined]
        self.callbacks = []
        self.acts = None
        self.carry = None
        self._callback_row_states: tuple[_CallbackRowState, ...] | None = None
        self.reset()

    def reset(self, init_policy=None):
        self._callback_row_states = None
        return super().reset(init_policy)

    def close(self):
        self.env.close()

    def _step(self, policy, step, episode):
        acts = self.acts
        obs = self.env.step(acts)  # type: ignore[misc]
        logs = {k: v for k, v in obs.items() if k.startswith("log/")}  # type: ignore[misc]
        obs = {k: v for k, v in obs.items() if not k.startswith("log/")}  # type: ignore[misc]
        row_states = self._validated_row_episode_states(obs)
        self.carry, acts, outs = policy(self.carry, obs, **self.kwargs)
        if obs["is_last"].any():
            mask = ~obs["is_last"]
            acts = {k: self._mask(v, mask) for k, v in acts.items()}
        self.acts = {**acts, "reset": obs["is_last"].copy()}
        trans = {**obs, **acts, **outs, **logs}
        collisions = CALLBACK_ROW_METADATA_KEYS & trans.keys()
        if collisions:
            raise ValueError(
                f"transition uses reserved Dreamer callback keys: {sorted(collisions)}"
            )
        for i in range(self.length):
            trn = elements.tree.map(lambda x, i=i: x[i], trans)
            state = row_states[i]
            trn = {
                **trn,
                CALLBACK_ENV_ID_KEY: np.int64(state.env_id),
                CALLBACK_EPISODE_ID_KEY: np.int64(state.episode_id),
                CALLBACK_STEP_INDEX_KEY: np.int64(state.step_index),
            }
            [fn(trn, i, **self.kwargs) for fn in self.callbacks]
        step += len(obs["is_first"])
        episode += obs["is_last"].sum()
        return step, episode

    def _validated_row_episode_states(self, obs):
        states = self.env.row_episode_states  # type: ignore[attr-defined]
        if not isinstance(states, tuple) or len(states) != self.length:
            raise ValueError(
                "Dreamer callback row state count does not match driver length"
            )

        flags = {}
        for name in ("is_first", "is_last"):
            value = obs[name]
            if tuple(getattr(value, "shape", ())) != (self.length,):
                raise ValueError(
                    f"Dreamer callback {name} must have shape {(self.length,)}"
                )
            flags[name] = tuple(bool(value[index]) for index in range(self.length))

        previous = self._callback_row_states
        validated_states = tuple(_coerce_callback_row_state(state) for state in states)
        for env_id, state in enumerate(validated_states):
            if state.env_id != env_id:
                raise ValueError("Dreamer callback row env_id ordering changed")
            if state.is_first != flags["is_first"][env_id]:
                raise ValueError("Dreamer callback is_first disagrees with observation")
            if state.is_last != flags["is_last"][env_id]:
                raise ValueError("Dreamer callback is_last disagrees with observation")
            if state.step_index < 0:
                raise ValueError("Dreamer callback step index cannot be negative")

            prior = previous[env_id] if previous is not None else None
            if prior is None:
                if not state.is_first or state.step_index != 0:
                    raise ValueError(
                        "first Dreamer callback row must be an initial reset"
                    )
                continue
            if prior.env_id != state.env_id:
                raise ValueError("Dreamer callback env_id changed")
            if prior.is_last and not state.is_first:
                raise ValueError("terminal Dreamer callback row did not reset next")
            if state.is_first:
                if state.episode_id == prior.episode_id or state.step_index != 0:
                    raise ValueError(
                        "reset Dreamer callback row must change ID at step 0"
                    )
            elif (
                state.episode_id != prior.episode_id
                or state.step_index != prior.step_index + 1
            ):
                raise ValueError(
                    "continuing Dreamer callback row must retain ID and advance once"
                )

        self._callback_row_states = validated_states
        return validated_states
