import functools
import time
from collections.abc import Mapping as ABCMapping
from copy import deepcopy
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Mapping, Protocol, runtime_checkable

import embodied
import numpy as np
import torch
from elements import Space

from srb.core.env.introspect import EnvIntrospector
from srb.core.spaces import coerce_gym_step_outputs
from srb.integrations.dreamer.learner_projection import DreamerLearnerProjection

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.env.runtime.observation_layout import NumericObservationProjection

#: Contract-projected flat state key exposed to Dreamer for landscaping envs.
STATE_KEY = "state"

DreamerArray = np.ndarray | torch.Tensor
DreamerTree = DreamerArray | Mapping[str, Any]


@dataclass(frozen=True, slots=True)
class DreamerSelectiveResetResult:
    """Atomic row-aligned result for Dreamer's selective reset transport."""

    observation: DreamerTree
    reward: DreamerArray
    terminated: DreamerArray
    truncated: DreamerArray
    info: tuple[Mapping[str, Any] | None, ...]
    is_first: DreamerArray
    episode_ids: DreamerArray
    step_delta: DreamerArray


@dataclass(frozen=True, slots=True)
class DreamerRowEpisodeState:
    """Callback-side identity for one row; never part of learner observations."""

    env_id: int
    episode_id: int
    step_index: int
    is_first: bool
    is_last: bool


@runtime_checkable
class DreamerSelectiveResetTransport(Protocol):
    """Explicit transport required for batched Dreamer reset-and-step."""

    def reset(self) -> DreamerSelectiveResetResult:
        """Return unstepped reset rows for the initial batch reset."""

    def reset_and_step(
        self, action: DreamerTree, reset_mask: np.ndarray
    ) -> DreamerSelectiveResetResult:
        """Reset true rows and step false rows exactly once, atomically."""


def _transport_host_array(value: DreamerArray, *, name: str) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if isinstance(value, torch.Tensor):
        try:
            array = value.detach().cpu().numpy()
        except (RuntimeError, TypeError) as exc:
            raise TypeError(f"transport {name} tensor is not host-readable") from exc
        if not isinstance(array, np.ndarray):
            raise TypeError(f"transport {name} did not produce an ndarray")
        return array
    raise TypeError(
        f"transport {name} must be an ndarray or tensor, got {type(value).__name__}"
    )


def _validate_transport_leaf(array: np.ndarray, *, name: str) -> None:
    if array.dtype.kind == "b":
        return
    if array.dtype.kind not in "iuf":
        raise TypeError(f"transport {name} has non-numeric dtype {array.dtype}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"transport {name} contains non-finite values")


def _validate_transport_observation(
    value: DreamerTree, *, name: str, num_envs: int
) -> None:
    if isinstance(value, ABCMapping):
        if not value:
            raise ValueError("transport observation mapping must not be empty")
        for key, child in value.items():
            _validate_transport_observation(
                child, name=f"{name}.{key}", num_envs=num_envs
            )
        return
    array = _transport_host_array(value, name=name)
    if array.ndim == 0 or array.shape[0] != num_envs:
        raise ValueError(
            f"transport {name} must be row-aligned with shape[0]={num_envs}, "
            f"got {array.shape}"
        )
    _validate_transport_leaf(array, name=name)


def _validate_transport_vector(
    value: DreamerArray, *, name: str, num_envs: int
) -> np.ndarray:
    array = _transport_host_array(value, name=name)
    if array.shape != (num_envs,):
        raise ValueError(
            f"transport {name} must have shape {(num_envs,)}, got {array.shape}"
        )
    return array


def _strict_reset_mask(value: DreamerArray, *, num_envs: int) -> np.ndarray:
    array = _transport_host_array(value, name="reset mask")
    if array.dtype != np.dtype(np.bool_):
        raise TypeError(f"Dreamer reset mask must have bool dtype, got {array.dtype}")
    if array.shape != (num_envs,):
        raise ValueError(
            f"Dreamer reset mask must have shape {(num_envs,)}, got {array.shape}"
        )
    return array.copy()


def _validate_selective_reset_result(
    result: DreamerSelectiveResetResult,
    *,
    reset_mask: np.ndarray,
    previous_episode_ids: np.ndarray | None,
    initial_reset: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    num_envs = reset_mask.shape[0]
    if not isinstance(result, DreamerSelectiveResetResult):
        raise TypeError(
            "selective reset transport must return DreamerSelectiveResetResult"
        )

    _validate_transport_observation(
        result.observation, name="observation", num_envs=num_envs
    )
    reward = _validate_transport_vector(result.reward, name="reward", num_envs=num_envs)
    terminated = _validate_transport_vector(
        result.terminated, name="terminated", num_envs=num_envs
    )
    truncated = _validate_transport_vector(
        result.truncated, name="truncated", num_envs=num_envs
    )
    is_first = _validate_transport_vector(
        result.is_first, name="is_first", num_envs=num_envs
    )
    episode_ids = _validate_transport_vector(
        result.episode_ids, name="episode_ids", num_envs=num_envs
    )
    step_delta = _validate_transport_vector(
        result.step_delta, name="step_delta", num_envs=num_envs
    )

    for name, array in (
        ("terminated", terminated),
        ("truncated", truncated),
        ("is_first", is_first),
    ):
        if array.dtype != np.dtype(np.bool_):
            raise TypeError(f"transport {name} must have bool dtype, got {array.dtype}")
    if reward.dtype.kind not in "iuf" or not np.all(np.isfinite(reward)):
        raise TypeError("transport reward must be finite real numeric data")
    if episode_ids.dtype.kind not in "iu" or episode_ids.dtype.kind == "b":
        raise TypeError("transport episode_ids must be integer data")
    if step_delta.dtype.kind not in "iu" or step_delta.dtype.kind == "b":
        raise TypeError("transport step_delta must be integer data")
    if np.any(step_delta < 0):
        raise ValueError("transport step_delta cannot be negative")

    if not isinstance(result.info, tuple) or len(result.info) != num_envs:
        raise TypeError("transport info must be a row-aligned tuple")
    if any(
        item is not None and not isinstance(item, ABCMapping) for item in result.info
    ):
        raise TypeError("transport info rows must be mappings or None")

    expected_steps = (~reset_mask).astype(step_delta.dtype, copy=False)
    if not np.array_equal(step_delta, expected_steps):
        raise ValueError(
            "selective reset transport must report step_delta=0 for reset rows "
            "and step_delta=1 for continuing rows"
        )
    if not np.array_equal(is_first, reset_mask):
        raise ValueError("transport is_first must equal the requested reset mask")
    if np.any(reset_mask & (terminated | truncated)):
        raise ValueError("transport reset rows cannot also be terminal")

    if initial_reset:
        if not np.all(reset_mask):
            raise ValueError("initial transport reset must reset every row")
        if np.any(reward != 0) or np.any(terminated) or np.any(truncated):
            raise ValueError("initial transport reset must be an unstepped reset")
    elif np.any(reset_mask & (reward != 0)):
        raise ValueError("transport reset rows must have zero reward")
    if previous_episode_ids is not None:
        if previous_episode_ids.shape != (num_envs,):
            raise ValueError("previous transport episode IDs are not row-aligned")
        changed = episode_ids != previous_episode_ids
        if np.any(reset_mask & ~changed):
            raise ValueError("reset rows must change episode ID")
        if np.any(~reset_mask & changed):
            raise ValueError("continuing rows must retain episode ID")

    return (
        episode_ids.copy(),
        step_delta.copy(),
        is_first.copy(),
        terminated.copy(),
        truncated.copy(),
    )


_BATCHED_REJECTION_MESSAGE = (
    "Dreamer integration rejects num_envs={num_envs} (> 1): the current "
    "DirectEnv wrapper has no selective reset/first-frame transport. In a "
    "mixed batch (one row ending while another continues) the underlying "
    "DirectEnv auto-resets the ending row internally, and the wrapper would "
    "advance that freshly reset row one hidden step under a masked action "
    "before labelling the resulting frame is_first=True — which is not the "
    "unstepped reset observation Dreamer's reset action expects. Globally "
    "resetting on any row (.any()) would instead wipe the continuing rows. "
    "Until a true selective reset-frame transport exists, run Dreamer with "
    "env.num_envs=1."
)


class EmbodiedEnvWrapper(embodied.Env):
    def __init__(
        self,
        env: "AnyEnv",
        obs_key: str = "image",
        act_key: str = "action",
        *,
        selective_reset_transport: DreamerSelectiveResetTransport | None = None,
        _allow_experimental_batched: bool = False,
    ):
        self.env = env
        self._obs_key = obs_key
        self._act_key = act_key
        self._selective_reset_transport = selective_reset_transport

        view = EnvIntrospector.of(env)
        self._env_view = view
        # Extract spaces
        self._obs_space = deepcopy(view.single_observation_space)
        self._action_space = deepcopy(view.single_action_space)

        # Extract useful information
        self._num_envs = view.num_envs
        self._device = view.device

        if selective_reset_transport is not None and not isinstance(
            selective_reset_transport, DreamerSelectiveResetTransport
        ):
            raise TypeError(
                "selective_reset_transport must implement "
                "DreamerSelectiveResetTransport"
            )

        # Fail closed on batched Dreamer without a selective reset transport.
        # `_allow_experimental_batched` is an opt-in escape hatch for
        # diagnosing the underlying mixed-reset defect that motivates this
        # rejection. It is not a production batched path; supported batched
        # use requires the explicit transport above.
        if (
            self._num_envs > 1
            and selective_reset_transport is None
            and not _allow_experimental_batched
        ):
            raise RuntimeError(
                _BATCHED_REJECTION_MESSAGE.format(num_envs=self._num_envs)
            )

        self._learner_projection = DreamerLearnerProjection.from_spaces(
            self._obs_space,
            self._action_space,
            num_envs=self._num_envs,
            device=self._device,
            obs_key=self._obs_key,
            act_key=self._act_key,
            mode="visual" if self._obs_key == "image" else "state",
            unwrapped=self.unwrapped,
        )
        reserved_observation_keys = {
            "reward",
            "is_first",
            "is_last",
            "is_terminal",
        }
        collisions = reserved_observation_keys.intersection(
            self._learner_projection.observation_spaces
        )
        if collisions:
            raise ValueError(
                "Dreamer observation space uses reserved wrapper keys: "
                + ", ".join(sorted(collisions))
            )
        if "reset" in self._learner_projection.action_spaces:
            raise ValueError("Dreamer action space uses reserved wrapper key: reset")
        self._demo_projection = None
        try:
            from .demo_projection import DreamerDemoProjection

            self._demo_projection = DreamerDemoProjection.from_live(
                env,
                learner_projection=self._learner_projection,
            )
        except (AttributeError, TypeError, ValueError, RuntimeError, ImportError):
            # Demo metadata is optional for ordinary runs; replay callers validate it.
            self._demo_projection = None

        # Bounded simulator-to-learner latency samples (seconds), used by the
        # throughput report:
        #   source   - the simulator's own reset()/step() call;
        #   readback - Torch device->host transfer of the assembled leaves;
        #   wrapper  - contract projection/packing into the flat state vector.
        self._latency_samples_s: dict[str, list[float]] = {
            "source": [],
            "readback": [],
            "wrapper": [],
        }
        self._source_latency_s: float | None = None

        # Init buffers
        self._done = np.ones(self._num_envs, dtype=bool)
        self._info = [None for _ in range(self._num_envs)]
        self._episode_ids: np.ndarray | None = None
        self._row_episode_states: tuple[DreamerRowEpisodeState, ...] = ()
        self._single_episode_id = -1
        self._pending_post_reset_observation: Any | None = None

    def __len__(self) -> int:
        return self._num_envs

    def __str__(self) -> str:
        return f"<{type(self).__name__}{self.env}>"

    def __repr__(self) -> str:
        return str(self)

    @classmethod
    def class_name(cls) -> str:
        return cls.__name__

    @property
    def unwrapped(self) -> "AnyEnv":
        return self.env.unwrapped  # type: ignore[return-value]

    @property
    def info(self) -> Mapping[str, Any]:
        return self._info  # type: ignore[return-value]

    @property
    def simulator_device(self) -> Any:
        """Device the underlying simulator env produces observations on."""
        return self._device

    @property
    def observation_projection(self) -> "NumericObservationProjection | None":
        """Contract projection shared by train/eval/collect/policy paths."""
        return self._learner_projection.observation_projection

    @property
    def demo_projection(self) -> Any | None:
        """Live canonical-to-Dreamer demonstration adapter, when declared."""

        return self._demo_projection

    @property
    def demo_io_contract(self) -> Mapping[str, Any] | None:
        """Canonical producer contract used by the optional demo adapter."""

        if self._demo_projection is None:
            return None
        return self._demo_projection.contract

    @property
    def row_episode_states(self) -> tuple[DreamerRowEpisodeState, ...]:
        """Return a detached immutable snapshot for callback attribution."""
        return tuple(
            DreamerRowEpisodeState(
                env_id=state.env_id,
                episode_id=state.episode_id,
                step_index=state.step_index,
                is_first=state.is_first,
                is_last=state.is_last,
            )
            for state in self._row_episode_states
        )

    @functools.cached_property
    def obs_space(self):
        spaces = {
            key: self._convert(space)
            for key, space in self._learner_projection.observation_spaces.items()
        }
        return {
            **spaces,
            "reward": Space(np.float32),
            "is_first": Space(bool),
            "is_last": Space(bool),
            "is_terminal": Space(bool),
        }

    @functools.cached_property
    def act_space(self):
        spaces = {
            key: self._convert(space)
            for key, space in self._learner_projection.action_spaces.items()
        }
        spaces["reset"] = Space(bool)
        return spaces

    def seed(self, seed: int | None = None) -> list[int | None]:
        # SrbEnv uses a negative seed to request a newly drawn seed.
        seed_value = -1 if seed is None else seed
        return [self.unwrapped.seed(seed_value)] * self.unwrapped.num_envs  # type: ignore[return-value]

    def reset(
        self,
    ) -> Mapping[str, np.ndarray]:
        if self._num_envs == 1 and self._pending_post_reset_observation is not None:
            obs = self._pending_post_reset_observation
            self._pending_post_reset_observation = None
            self._source_latency_s = 0.0
            self._done = np.zeros(self._num_envs, dtype=bool)
            self._single_episode_id += 1
            self._row_episode_states = (
                DreamerRowEpisodeState(
                    env_id=0,
                    episode_id=self._single_episode_id,
                    step_index=0,
                    is_first=True,
                    is_last=False,
                ),
            )
            self._info = [{}]
            return self._obs(
                obs=obs,
                reward=np.zeros(self._num_envs, dtype=np.float32),
                is_first=np.ones(self._num_envs, dtype=bool),
                is_last=np.zeros(self._num_envs, dtype=bool),
                is_terminal=np.zeros(self._num_envs, dtype=bool),
            )

        if self._num_envs > 1 and self._selective_reset_transport is not None:
            reset_mask = np.ones(self._num_envs, dtype=bool)
            previous_episode_ids = (
                None if self._episode_ids is None else self._episode_ids.copy()
            )
            source_start_s = time.perf_counter()
            result = self._selective_reset_transport.reset()
            self._source_latency_s = time.perf_counter() - source_start_s
            (
                episode_ids,
                step_delta,
                is_first,
                terminated,
                truncated,
            ) = _validate_selective_reset_result(
                result,
                reset_mask=reset_mask,
                previous_episode_ids=previous_episode_ids,
                initial_reset=True,
            )
            self._episode_ids = episode_ids
            self._info = list(result.info)
            self._done = terminated | truncated
            self._set_transport_row_episode_states(
                episode_ids=episode_ids,
                step_delta=step_delta,
                is_first=is_first,
                is_last=self._done,
            )
            return self._obs(
                obs=result.observation,
                reward=result.reward,
                is_first=result.is_first,
                is_last=self._done,
                is_terminal=terminated,
            )

        source_start_s = time.perf_counter()
        obs, self._info = self.env.reset()
        self._source_latency_s = time.perf_counter() - source_start_s
        self._done = np.zeros(self._num_envs, dtype=bool)
        if self._num_envs == 1:
            self._pending_post_reset_observation = None
            self._single_episode_id += 1
            self._row_episode_states = (
                DreamerRowEpisodeState(
                    env_id=0,
                    episode_id=self._single_episode_id,
                    step_index=0,
                    is_first=True,
                    is_last=False,
                ),
            )
        return self._obs(
            obs=obs,  # type: ignore[misc]
            reward=np.zeros(self._num_envs, dtype=np.float32),
            is_first=np.ones(self._num_envs, dtype=bool),
            is_last=np.zeros(self._num_envs, dtype=bool),
            is_terminal=np.zeros(self._num_envs, dtype=bool),
        )

    def step(
        self, action: Mapping[str, np.ndarray | torch.Tensor]
    ) -> Mapping[str, np.ndarray]:
        if self._num_envs > 1 and self._selective_reset_transport is not None:
            return self._step_with_selective_reset_transport(action)

        if action["reset"].all() or self._done.all():
            return self.reset()

        # In the diagnostic mixed-batch path, DirectEnv has already reset rows
        # marked done on the prior call; retain that identity before overwrite.
        was_done = np.asarray(self._done, dtype=bool).reshape(self._num_envs).copy()

        act = self._learner_projection.coerce_action_batch(
            {key: value for key, value in action.items() if key != "reset"},
            device=self._device,
        )

        source_start_s = time.perf_counter()
        obs, reward, terminated, truncated, self._info = self.env.step(act)  # type: ignore[attr-defined]
        self._source_latency_s = time.perf_counter() - source_start_s
        reward, terminated, truncated = coerce_gym_step_outputs(
            reward, terminated, truncated, output="cpu_tensor"
        )

        self._done = terminated | truncated
        if self._num_envs == 1:
            if bool(self._done[0]):
                seam_owner = self.unwrapped
                try:
                    has_srb_contract = (
                        getattr(seam_owner, "_srb_env_finalized", False) is True
                        or getattr(seam_owner, "_srb_runtime_mode", None) is not None
                        or any(
                            hasattr(seam_owner, name)
                            for name in (
                                "last_final_observation",
                                "last_final_observation_ids",
                                "last_final_observation_mask",
                            )
                        )
                    )
                    if has_srb_contract:
                        final_observation = getattr(
                            seam_owner, "last_final_observation"
                        )
                        final_ids = getattr(seam_owner, "last_final_observation_ids")
                        final_mask = getattr(seam_owner, "last_final_observation_mask")
                except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
                    raise RuntimeError(
                        "SRB final observation seam is unavailable"
                    ) from exc
                if has_srb_contract:
                    if final_observation is None or (
                        isinstance(final_observation, ABCMapping)
                        and not final_observation
                    ):
                        raise RuntimeError(
                            "SRB final observation is missing or malformed"
                        )
                    try:
                        if isinstance(final_ids, torch.Tensor):
                            final_ids = final_ids.detach().cpu().numpy()
                        else:
                            final_ids = np.asarray(final_ids)
                        if isinstance(final_mask, torch.Tensor):
                            final_mask = final_mask.detach().cpu().numpy()
                        else:
                            final_mask = np.asarray(final_mask)
                    except (RuntimeError, TypeError, ValueError) as exc:
                        raise RuntimeError(
                            "SRB final observation IDs or mask are not CPU-readable"
                        ) from exc
                    if (
                        final_ids.shape != (1,)
                        or final_ids.dtype.kind not in "iu"
                        or final_ids.dtype.kind == "b"
                        or int(final_ids[0]) != 0
                    ):
                        raise RuntimeError(
                            "SRB final observation IDs are not the scalar env row"
                        )
                    if final_mask.dtype != np.dtype(np.bool_) or final_mask.shape != (
                        1,
                    ):
                        raise RuntimeError(
                            "SRB final observation mask must be bool with shape (1,)"
                        )
                    expected_mask = np.zeros((1,), dtype=np.bool_)
                    expected_mask[final_ids] = True
                    if not np.array_equal(final_mask, expected_mask):
                        raise RuntimeError(
                            "SRB final observation mask disagrees with its terminal IDs"
                        )
                    self._pending_post_reset_observation = deepcopy(obs)
                    obs = final_observation

            self._advance_single_row_episode_state(
                is_first=bool(was_done[0]),
                is_last=bool(self._done[0]),
            )

        # DirectEnv returns its post-reset splice from step() and exposes the
        # compact terminal rows separately. Keep the post-reset scalar row for
        # the next first frame, while returning the cached terminal row now.
        return self._obs(
            obs=obs,  # type: ignore[misc]
            reward=reward,
            is_first=was_done,
            is_last=self._done,
            is_terminal=terminated,
        )

    def _step_with_selective_reset_transport(
        self, action: Mapping[str, np.ndarray | torch.Tensor]
    ) -> Mapping[str, np.ndarray]:
        reset_mask = _strict_reset_mask(action["reset"], num_envs=self._num_envs)
        if self._episode_ids is None:
            if np.all(reset_mask):
                return self.reset()
            raise RuntimeError(
                "batched Dreamer transport must initialize with an all-row reset"
            )
        if self._done.all() and not np.any(reset_mask):
            return self.reset()
        if len(self._row_episode_states) != self._num_envs:
            raise RuntimeError("batched Dreamer row identity is not initialized")
        for state in self._row_episode_states:
            if state.env_id < 0 or state.env_id >= self._num_envs:
                raise RuntimeError("batched Dreamer row identity has invalid env_id")
            if state.is_last and not reset_mask[state.env_id]:
                raise ValueError("a terminal Dreamer row must reset before stepping")

        act = self._learner_projection.coerce_action_batch(
            {key: value for key, value in action.items() if key != "reset"},
            device=self._device,
        )
        previous_episode_ids = (
            None if self._episode_ids is None else self._episode_ids.copy()
        )

        source_start_s = time.perf_counter()
        result = self._selective_reset_transport.reset_and_step(act, reset_mask)
        self._source_latency_s = time.perf_counter() - source_start_s
        (
            episode_ids,
            step_delta,
            is_first,
            terminated,
            truncated,
        ) = _validate_selective_reset_result(
            result,
            reset_mask=reset_mask,
            previous_episode_ids=previous_episode_ids,
            initial_reset=False,
        )
        self._episode_ids = episode_ids
        self._info = list(result.info)
        self._done = terminated | truncated
        self._set_transport_row_episode_states(
            episode_ids=episode_ids,
            step_delta=step_delta,
            is_first=is_first,
            is_last=self._done,
        )
        return self._obs(
            obs=result.observation,
            reward=result.reward,
            is_first=result.is_first,
            is_last=self._done,
            is_terminal=terminated,
        )

    def _set_transport_row_episode_states(
        self,
        *,
        episode_ids: np.ndarray,
        step_delta: np.ndarray,
        is_first: np.ndarray,
        is_last: np.ndarray,
    ) -> None:
        previous = self._row_episode_states
        if previous and len(previous) != self._num_envs:
            raise RuntimeError("Dreamer row state count changed")

        next_states: list[DreamerRowEpisodeState] = []
        for env_id in range(self._num_envs):
            episode_id = int(episode_ids[env_id])
            delta = int(step_delta[env_id])
            first = bool(is_first[env_id])
            last = bool(is_last[env_id])
            prior = previous[env_id] if previous else None

            if prior is not None and prior.env_id != env_id:
                raise ValueError("Dreamer row env_id ordering changed")
            if first:
                if delta != 0 or last:
                    raise ValueError("Dreamer reset row must be fresh at step_index 0")
                if prior is not None and episode_id == prior.episode_id:
                    raise ValueError("Dreamer reset row must change episode ID")
                step_index = 0
            else:
                if prior is None:
                    raise ValueError("Dreamer continuing row has no prior identity")
                if prior.is_last:
                    raise ValueError(
                        "Dreamer terminal row cannot continue without reset"
                    )
                if episode_id != prior.episode_id or delta != 1:
                    raise ValueError(
                        "Dreamer continuing row must retain ID and advance once"
                    )
                step_index = prior.step_index + 1

            next_states.append(
                DreamerRowEpisodeState(
                    env_id=env_id,
                    episode_id=episode_id,
                    step_index=step_index,
                    is_first=first,
                    is_last=last,
                )
            )

        self._row_episode_states = tuple(next_states)

    def _advance_single_row_episode_state(
        self, *, is_first: bool, is_last: bool
    ) -> None:
        if len(self._row_episode_states) != 1:
            raise RuntimeError("single-environment Dreamer identity is not initialized")
        previous = self._row_episode_states[0]
        if is_first or previous.is_last:
            raise ValueError("single-environment episode must reset before continuing")
        self._row_episode_states = (
            DreamerRowEpisodeState(
                env_id=0,
                episode_id=previous.episode_id,
                step_index=previous.step_index + 1,
                is_first=False,
                is_last=is_last,
            ),
        )

    def render(self) -> np.ndarray:
        image = self.env.render()
        if image is None:
            raise RuntimeError("wrapped environment returned no render image")
        return image

    def close(self):
        self.env.close()

    def get_attr(self, attr_name, indices=None) -> Any:
        if indices is None:
            indices = slice(None)
            num_indices = self._num_envs
        else:
            num_indices = len(indices)
        attr_val = getattr(self.env, attr_name)
        if not isinstance(attr_val, torch.Tensor):
            return [attr_val] * num_indices
        return attr_val[indices].detach().cpu()

    def set_attr(self, attr_name, value, indices=None):
        raise NotImplementedError("Setting attributes is not supported.")

    def env_method(
        self, method_name: str, *method_args, indices=None, **method_kwargs
    ) -> Any:
        if method_name == "render":
            return self.env.render()
        # One vectorized env serves every index; `indices` is not forwarded.
        env_method = getattr(self.env, method_name)
        return env_method(*method_args, **method_kwargs)

    def conversion_latency_stats(self) -> dict[str, Any]:
        """p50/p95 source/readback/wrapper latency (seconds) since construction.

        Measures the actual simulator->learner observation boundary stage by
        stage: the simulator step itself (``source``), the Torch device->host
        readback (``readback``), and the contract projection/packing
        (``wrapper``). ``total`` is the per-frame sum of the three. This path
        is a real transfer; it is never zero-copy.
        """
        stats: dict[str, Any] = {}
        for stage, samples_list in self._latency_samples_s.items():
            stats[stage] = _percentile_stats(samples_list)
        paired = min(len(values) for values in self._latency_samples_s.values())
        totals = [
            sum(
                self._latency_samples_s[stage][index]
                for stage in ("readback", "wrapper")
            )
            + self._latency_samples_s["source"][index]
            for index in range(paired)
        ]
        stats["total"] = _percentile_stats(totals)
        # Backwards-compatible flat view of the observation-boundary conversion
        # (readback + wrapper), i.e. everything after the simulator returned.
        conversion = [
            self._latency_samples_s["readback"][index]
            + self._latency_samples_s["wrapper"][index]
            for index in range(paired)
        ]
        stats.update(_percentile_stats(conversion))
        return stats

    def _obs(
        self,
        obs: np.ndarray | torch.Tensor | dict[str, np.ndarray | torch.Tensor],
        reward: np.ndarray | torch.Tensor,
        is_first: np.ndarray | torch.Tensor,
        is_last: np.ndarray | torch.Tensor,
        is_terminal: np.ndarray | torch.Tensor,
    ) -> Mapping[str, np.ndarray | torch.Tensor]:
        obs, readback_s, wrapper_s = (
            self._learner_projection.project_observation_batch_with_timings(obs)
        )

        self._record_latency(
            source_s=self._source_latency_s,
            readback_s=readback_s,
            wrapper_s=wrapper_s,
        )

        obs.update(
            reward=reward,
            is_first=is_first,
            is_last=is_last,
            is_terminal=is_terminal,
        )
        return obs

    def _record_latency(
        self, *, source_s: float | None, readback_s: float, wrapper_s: float
    ) -> None:
        if len(self._latency_samples_s["wrapper"]) >= 200_000:
            return
        self._latency_samples_s["source"].append(float(source_s or 0.0))
        self._latency_samples_s["readback"].append(float(readback_s))
        self._latency_samples_s["wrapper"].append(float(wrapper_s))
        self._source_latency_s = None

    def _convert(self, space) -> Space:
        if hasattr(space, "n"):
            return Space(np.int32, (), 0, space.n)
        return Space(space.dtype, space.shape, space.low, space.high)


def _percentile_stats(values: "list[float]") -> dict[str, float]:
    samples = np.asarray(values, dtype=np.float64)
    if samples.size == 0:
        return {"samples": 0}
    return {
        "samples": int(samples.size),
        "p50_s": float(np.percentile(samples, 50)),
        "p95_s": float(np.percentile(samples, 95)),
        "mean_s": float(samples.mean()),
        "max_s": float(samples.max()),
    }
