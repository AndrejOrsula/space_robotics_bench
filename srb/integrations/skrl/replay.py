"""Bounded demonstration replay and checkpoint support for SKRL adapters."""

from __future__ import annotations

import copy
import io
import json
import math
import pickle
import random
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import torch

from srb.utils.resource_budget import (
    MAX_ARRAY_BYTES,
    MAX_EXTERNAL_FILE_BYTES,
    MAX_IN_MEMORY_FILE_BYTES,
    checked_shape_nbytes,
    require_bounded_int,
)

if TYPE_CHECKING:
    from srb.integrations.skrl.wrapper import SkrlEnvWrapper


# SKRL checkpoints include this module only when demo replay is requested.
REPLAY_CHECKPOINT_MODULE = "srb_demo_replay"
REPLAY_STATE_VERSION = 1

# Leave checkpoint space for policy, critic and optimizer state.
MAX_NATIVE_REPLAY_BYTES = min(int(MAX_IN_MEMORY_FILE_BYTES), 192 * (1 << 20))
MAX_NATIVE_CHECKPOINT_BYTES = int(MAX_IN_MEMORY_FILE_BYTES)
MAX_NATIVE_REPLAY_TENSORS = 64
MAX_NATIVE_REPLAY_METADATA_BYTES = 1 * (1 << 20)
MAX_NATIVE_REPLAY_TRANSITIONS = 5_000_000

_REPLAY_TENSOR_NAMES = (
    "states",
    "actions",
    "rewards",
    "next_states",
    "terminated",
    "truncated",
)
_NATIVE_REPLAY_TENSOR_NAMES = (
    "observations",
    "states",
    "actions",
    "rewards",
    "next_observations",
    "next_states",
    "terminated",
    "truncated",
)


class DemoReplayError(ValueError):
    """Raised when canonical demonstrations cannot bind to native SKRL replay."""


def _memory_tensor(memory: Any, name: str) -> torch.Tensor:
    """Get a memory tensor across the SKRL 1.x/2.x accessor signatures."""

    try:
        return memory.get_tensor_by_name(name)
    except TypeError:
        return memory.get_tensor_by_name(name, keepdim=False)


def _memory_flat_tensor(memory: Any, name: str) -> torch.Tensor:
    """Return a row-major view regardless of SKRL memory accessor shape."""

    views = getattr(memory, "tensors_view", {})
    if name in views:
        return views[name]
    tensor = _memory_tensor(memory, name)
    if tensor.ndim >= 3:
        return tensor.view((-1, *tensor.shape[2:]))
    return tensor


def _memory_tensor_names(memory: Any) -> tuple[str, ...]:
    """Return canonical tensors present in a native memory instance."""

    available = getattr(memory, "tensors", None)
    if not isinstance(available, Mapping):
        available = set(memory.get_tensor_names())
    return tuple(name for name in _NATIVE_REPLAY_TENSOR_NAMES if name in available)


def _native_payload_tensors(
    memory: Any, tensors: Mapping[str, torch.Tensor]
) -> dict[str, torch.Tensor]:
    """Project the legacy six-field payload into SKRL 2.x native names."""

    projected: dict[str, torch.Tensor] = {}
    for name in _memory_tensor_names(memory):
        if name == "observations":
            projected[name] = (
                tensors["observations"]
                if "observations" in tensors
                else tensors["states"]
            )
        elif name == "next_observations":
            projected[name] = (
                tensors["next_observations"]
                if "next_observations" in tensors
                else tensors["next_states"]
            )
        elif name in tensors:
            projected[name] = tensors[name]
    return projected


class _BoundedBytesIO(io.BytesIO):
    """Bytes buffer that rejects a serialized checkpoint before growth."""

    def __init__(self, limit: int) -> None:
        super().__init__()
        self._limit = limit

    def write(self, data: bytes | bytearray) -> int:
        current = self.tell()
        existing = len(self.getbuffer())
        if max(current + len(data), existing) > self._limit:
            raise DemoReplayError(
                "SKRL checkpoint exceeds native size limit during serialization"
            )
        return super().write(data)


@dataclass(frozen=True, slots=True)
class DemoReplayPayload:
    """Fully staged replay rows, safe to commit to a native memory."""

    states: torch.Tensor
    actions: torch.Tensor
    rewards: torch.Tensor
    next_states: torch.Tensor
    terminated: torch.Tensor
    truncated: torch.Tensor
    binding: Mapping[str, Any]
    dataset_fingerprint: str
    dataset_path: str
    transition_count: int
    # Optional SKRL 2.x observation tensors preserve six-field construction.
    observations: torch.Tensor | None = None
    next_observations: torch.Tensor | None = None

    @property
    def tensors(self) -> dict[str, torch.Tensor]:
        """Return the six SKRL memory tensors in its canonical name order."""
        return {
            "states": self.states,
            "actions": self.actions,
            "rewards": self.rewards,
            "next_states": self.next_states,
            "terminated": self.terminated,
            "truncated": self.truncated,
        }


def build_demo_replay_payload(
    dataset_path: str | Path,
    wrapper: "SkrlEnvWrapper",
    max_transitions: int,
) -> DemoReplayPayload:
    """Stage a bounded source without touching native memory.

    Import the dataset reader lazily; ``srb.datasets`` validates source
    snapshots, and optional providers are not needed to import this module.
    """

    limit = require_bounded_int(
        max_transitions,
        resource="SKRL demo transitions",
        minimum=1,
        maximum=MAX_NATIVE_REPLAY_TRANSITIONS,
    )
    try:
        from srb.datasets.metadata_validation import validate_demo_io_contract
        from srb.datasets.transitions import (
            DemoTransition,
            ValidatedDemoSource,
            open_demo_transition_source,
        )
    except (ImportError, ModuleNotFoundError) as exc:
        raise DemoReplayError(
            "SKRL demonstration prefill requires the canonical "
            "srb.datasets.transitions reader"
        ) from exc

    source_path = Path(dataset_path)
    with open_demo_transition_source(
        source_path,
        max_transitions=limit,
    ) as source:
        if not isinstance(source, ValidatedDemoSource):
            raise DemoReplayError(
                "canonical demo reader returned an invalid validated source"
            )
        if any(
            not isinstance(transition, DemoTransition)
            for transition in source.transitions
        ):
            raise DemoReplayError(
                "canonical demo reader returned an invalid transition type"
            )
        if not source.transitions or len(source.transitions) > limit:
            raise DemoReplayError(
                "canonical demo source transition count exceeds the requested bound"
            )
        try:
            contract_value = validate_demo_io_contract(
                source.demo_io_contract,
                required=True,
            )
        except (TypeError, ValueError) as exc:
            raise DemoReplayError(
                "canonical demo source requires a valid demo_io_contract"
            ) from exc
        if contract_value is None:
            raise DemoReplayError("canonical demo source requires a demo_io_contract")
        binding = _source_binding(source, contract=contract_value)
        _validate_observation_binding(binding, wrapper)
        _validate_action_binding(binding, wrapper, index=0)
        _validate_projected_capacity(wrapper, len(source.transitions))
        rows: list[tuple[torch.Tensor, ...]] = []
        staged_bytes = 0

        for index, transition in enumerate(source.transitions):
            projected = _project_transition(transition, wrapper, index=index)
            projected_bytes = sum(
                int(value.numel() * value.element_size()) for value in projected
            )
            staged_bytes += projected_bytes
            if staged_bytes > MAX_NATIVE_REPLAY_BYTES:
                raise DemoReplayError(
                    "staged SKRL demo replay exceeds native memory budget"
                )
            rows.append(projected)

        if not rows:
            raise DemoReplayError("canonical demo source contains no transitions")
        dataset_fingerprint = source.content_fingerprint
        binding = dict(binding)
        binding["dataset_fingerprint"] = dataset_fingerprint
        binding["transition_count"] = len(rows)

    tensors = _stack_projected_rows(rows)
    _validate_staged_payload(
        {
            "states": tensors[1],
            "actions": tensors[2],
            "rewards": tensors[3],
            "next_states": tensors[5],
            "terminated": tensors[6],
            "truncated": tensors[7],
        },
        binding=binding,
    )
    return DemoReplayPayload(
        states=tensors[1],
        actions=tensors[2],
        rewards=tensors[3],
        next_states=tensors[5],
        terminated=tensors[6],
        truncated=tensors[7],
        binding=binding,
        dataset_fingerprint=dataset_fingerprint,
        dataset_path=str(source_path.resolve()),
        transition_count=len(rows),
        observations=tensors[0],
        next_observations=tensors[4],
    )


def prefill_demo_replay(
    agent: Any,
    payload: DemoReplayPayload,
    *,
    replace: bool = False,
) -> None:
    """Validate rows, then commit them with rollback on insertion failure."""

    memory = getattr(agent, "memory", None)
    if memory is None:
        raise DemoReplayError("SKRL demo prefill requires an initialized memory")
    if not isinstance(payload, DemoReplayPayload):
        raise DemoReplayError("SKRL demo prefill payload has an invalid type")
    tensors = payload.tensors
    _validate_staged_payload(tensors, binding=payload.binding)
    if payload.transition_count != int(tensors["states"].shape[0]):
        raise DemoReplayError("SKRL demo prefill payload count does not match tensors")
    if payload.dataset_fingerprint != payload.binding.get("dataset_fingerprint"):
        raise DemoReplayError("SKRL demo prefill payload fingerprint is inconsistent")
    _validate_memory_contract(
        memory,
        tensors,
        observations=payload.observations,
        next_observations=payload.next_observations,
    )
    _validate_optional_payload_tensor(
        memory,
        payload.observations,
        native_name="observations",
        count=payload.transition_count,
    )
    _validate_optional_payload_tensor(
        memory,
        payload.next_observations,
        native_name="next_observations",
        count=payload.transition_count,
    )
    reward_scale = _demo_reward_scale(agent)
    if reward_scale != 1.0:
        tensors["rewards"] = tensors["rewards"] * reward_scale
        if not torch.isfinite(tensors["rewards"]).all():
            raise DemoReplayError("SKRL demo scaled rewards are non-finite")
    native_tensors = _native_payload_tensors(memory, tensors)
    native_names = _memory_tensor_names(memory)
    if payload.observations is not None and "observations" in native_names:
        native_tensors["observations"] = payload.observations
    if payload.next_observations is not None and "next_observations" in native_names:
        native_tensors["next_observations"] = payload.next_observations

    capacity = _memory_capacity(memory)
    current = int(len(memory))
    # Replacement validates the staged payload before resetting memory.
    required = (
        payload.transition_count if replace else current + payload.transition_count
    )
    if required > capacity:
        raise DemoReplayError(
            "SKRL demo replay exceeds native memory capacity: "
            f"required {required}, capacity {capacity}"
        )
    if payload.transition_count % int(memory.num_envs) != 0:
        raise DemoReplayError(
            "SKRL demo replay transition count must be divisible by native num_envs"
        )
    if int(memory.env_index) != 0:
        raise DemoReplayError(
            "SKRL demo replay requires an environment-aligned native memory cursor"
        )

    overwritten = (
        None if replace else _insertion_slot_indices(memory, payload.transition_count)
    )
    before = _capture_memory_state(memory, indices=overwritten)
    completed = False
    try:
        if replace:
            memory.reset()
        for offset in range(payload.transition_count):
            memory.add_samples(
                **{
                    name: value[offset].to(device=memory.device)
                    for name, value in native_tensors.items()
                }
            )
        completed = True
    except (
        RuntimeError,
        ValueError,
        TypeError,
        LookupError,
        OSError,
        MemoryError,
    ) as exc:
        raise DemoReplayError("SKRL demo replay insertion failed") from exc
    finally:
        if not completed:
            try:
                _restore_memory_state(memory, before)
            except (
                RuntimeError,
                ValueError,
                TypeError,
                LookupError,
                OSError,
                MemoryError,
            ) as rollback_exc:
                raise DemoReplayError(
                    "SKRL demo replay insertion failed and memory rollback failed"
                ) from rollback_exc


def validate_demo_replay_configuration(
    agent_cfg: Mapping[str, Any],
    wrapper: Any,
) -> None:
    """Admit full replay checkpoint capacity before SKRL allocates memory."""

    memory_cfg = agent_cfg.get("memory")
    if not isinstance(memory_cfg, Mapping) or "memory_size" not in memory_cfg:
        raise DemoReplayError(
            "SKRL demo replay requires memory.memory_size for bounded checkpoint admission"
        )
    try:
        memory_size = require_bounded_int(
            memory_cfg["memory_size"],
            resource="SKRL replay memory_size",
            minimum=1,
            maximum=MAX_NATIVE_REPLAY_TRANSITIONS,
        )
        num_envs = require_bounded_int(
            getattr(getattr(wrapper, "_env_view", None), "num_envs", 1),
            resource="SKRL replay num_envs",
            minimum=1,
            maximum=MAX_NATIVE_REPLAY_TRANSITIONS,
        )
        observation_dim = _space_flat_dim(
            wrapper.single_observation_space,
            resource="SKRL observation space",
        )
        state_dim = _space_flat_dim(
            getattr(wrapper, "state_space", None) or wrapper.single_observation_space,
            resource="SKRL state space",
        )
        action_dim = _space_flat_dim(
            wrapper.single_action_space,
            resource="SKRL action space",
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise DemoReplayError(
            "cannot establish the bounded SKRL demo replay configuration"
        ) from exc
    row_bytes = (2 * observation_dim + 2 * state_dim + action_dim + 1) * np.dtype(
        np.float32
    ).itemsize + 2
    capacity = memory_size * num_envs
    replay_bytes = capacity * row_bytes
    if replay_bytes > MAX_NATIVE_REPLAY_BYTES:
        max_memory_size = MAX_NATIVE_REPLAY_BYTES // (num_envs * row_bytes)
        raise DemoReplayError(
            "configured SKRL demo replay capacity exceeds the checkpoint policy: "
            f"memory_size={memory_size}, num_envs={num_envs}, "
            f"maximum memory_size={max_memory_size} for this observation/action layout"
        )


def install_demo_replay_checkpoint(
    agent: Any,
    binding: Mapping[str, Any],
) -> "DemoReplayCheckpointModule":
    """Attach the opt-in checkpoint module and return it."""

    normalized_binding = _normalise_binding(binding)
    memory = getattr(agent, "memory", None)
    if memory is None:
        raise DemoReplayError("cannot enable demo replay without native memory")
    _validate_checkpoint_memory_capacity(memory)
    _demo_reward_scale(agent)
    if getattr(agent, "demo_algorithm", None) in {"skrl_ddpg", "skrl_td3"}:
        _off_policy_noise_contract(agent)
        _off_policy_update_configuration(agent)
    module = DemoReplayCheckpointModule(agent, binding=normalized_binding)
    agent._srb_demo_replay_checkpoint = module
    agent._srb_demo_replay_binding = dict(normalized_binding)
    agent._srb_demo_replay_restored = False
    agent.checkpoint_modules[REPLAY_CHECKPOINT_MODULE] = module
    return module


class DemoReplayCheckpointModule:
    """Adapter object consumed by SKRL's ``checkpoint_modules`` seam."""

    def __init__(self, agent: Any, *, binding: Mapping[str, Any]) -> None:
        self.agent = agent
        self.binding = _normalise_binding(binding)

    def state_dict(self) -> dict[str, Any]:
        return export_demo_replay_state(self.agent, binding=self.binding)

    def load_state_dict(self, state: Mapping[str, Any]) -> None:
        restore_demo_replay_state(
            self.agent,
            state,
            expected_binding=self.binding,
        )
        self.agent._srb_demo_replay_restored = True


def export_demo_replay_state(
    agent: Any,
    *,
    binding: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return compact active-slot state omitted by SKRL's native checkpoint."""

    memory = getattr(agent, "memory", None)
    if memory is None:
        raise DemoReplayError("cannot checkpoint demo replay without native memory")
    binding_value = dict(binding or getattr(agent, "_srb_demo_replay_binding", {}))
    active_count = int(len(memory))
    if active_count % int(memory.num_envs) != 0 or int(memory.env_index) != 0:
        raise DemoReplayError(
            "SKRL demo replay checkpoint cursor is not environment-aligned"
        )
    replay_bytes = active_count * _memory_row_bytes(memory)
    if replay_bytes > MAX_NATIVE_REPLAY_BYTES:
        raise DemoReplayError(
            "SKRL demo replay checkpoint exceeds native payload limit: "
            f"{replay_bytes} > {MAX_NATIVE_REPLAY_BYTES}"
        )
    tensors: dict[str, torch.Tensor] = {}
    for name in _memory_tensor_names(memory):
        if name not in memory.tensors:
            raise DemoReplayError(f"native memory is missing replay tensor {name!r}")
        flat = _memory_flat_tensor(memory, name)
        active = flat[:active_count].detach().cpu().clone()
        tensors[name] = active

    return {
        "version": REPLAY_STATE_VERSION,
        "binding": copy.deepcopy(binding_value),
        "memory": {
            "memory_size": int(memory.memory_size),
            "num_envs": int(memory.num_envs),
            "active_count": active_count,
            "memory_index": int(memory.memory_index),
            "env_index": int(memory.env_index),
            "filled": bool(memory.filled),
            "tensors": tensors,
        },
        "training": _training_state(agent),
        "rng": _rng_state(),
    }


def restore_demo_replay_state(
    agent: Any,
    state: Mapping[str, Any],
    *,
    expected_binding: Mapping[str, Any] | None = None,
) -> None:
    """Validate a complete replay state, then restore it transactionally."""

    validated = _validate_replay_state(
        state,
        agent=agent,
        expected_binding=expected_binding,
    )
    memory = getattr(agent, "memory", None)
    if memory is None:
        raise DemoReplayError("cannot restore demo replay without native memory")

    before_memory = _capture_memory_state(memory)
    before_training = _training_state(agent)
    before_rng = _rng_state()
    completed = False
    try:
        _restore_memory_state(memory, validated["memory"])
        _restore_training_state(agent, validated["training"])
        _restore_rng_state(validated["rng"])
        completed = True
    except (
        RuntimeError,
        ValueError,
        TypeError,
        LookupError,
        OSError,
        MemoryError,
    ) as exc:
        raise DemoReplayError("SKRL demo replay restore failed") from exc
    finally:
        if not completed:
            try:
                _restore_memory_state(memory, before_memory)
                _restore_training_state(agent, before_training)
                _restore_rng_state(before_rng)
            except (
                RuntimeError,
                ValueError,
                TypeError,
                LookupError,
                OSError,
                MemoryError,
            ) as rollback_exc:
                raise DemoReplayError(
                    "SKRL demo replay restore failed and rollback failed"
                ) from rollback_exc


def validate_replay_checkpoint_state(
    state: object,
    *,
    agent: Any,
    expected_binding: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Public preflight used by the agent checkpoint loader."""

    if not isinstance(state, Mapping):
        raise DemoReplayError("SKRL demo replay checkpoint state must be a mapping")
    return _validate_replay_state(
        state,
        agent=agent,
        expected_binding=expected_binding,
    )


def save_checkpoint_modules(agent: Any, path: str | Path) -> None:
    """Save SKRL checkpoint modules with a bounded serialized size."""

    modules = {
        name: agent._get_internal_value(module)
        for name, module in agent.checkpoint_modules.items()
    }
    save_checkpoint_values(modules, path)


def save_checkpoint_values(values: Mapping[str, Any], path: str | Path) -> None:
    """Save already captured checkpoint values with a bounded serialization."""

    stream = _BoundedBytesIO(MAX_NATIVE_CHECKPOINT_BYTES)
    try:
        torch.save(dict(values), stream)
    except DemoReplayError:
        raise
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        raise DemoReplayError("cannot serialize SKRL checkpoint") from exc
    payload = stream.getvalue()
    if len(payload) > MAX_NATIVE_CHECKPOINT_BYTES:
        raise DemoReplayError(
            "SKRL checkpoint exceeds native size limit: "
            f"{len(payload)} > {MAX_NATIVE_CHECKPOINT_BYTES}"
        )
    try:
        from srb.utils.publication import ArtifactPublication, PublicationError

        ArtifactPublication.file(path).publish_bytes(
            payload,
            max_bytes=MAX_NATIVE_CHECKPOINT_BYTES,
        )
    except (OSError, PublicationError, ValueError) as exc:
        raise DemoReplayError(f"cannot publish SKRL checkpoint: {path}") from exc


def load_checkpoint_modules(
    agent: Any,
    path: str | Path,
    *,
    allow_replay: bool = False,
) -> None:
    """Load SKRL modules atomically, with opt-in replay checkpoint handling."""

    source = Path(path)
    replay_module = getattr(agent, "_srb_demo_replay_checkpoint", None)
    try:
        from srb.utils.publication import ArtifactPublication, PublicationError

        publication = ArtifactPublication.file(source)
        if replay_module is None:
            # Keep ordinary no-demo loads on SKRL's streaming path.  The
            # descriptor-backed snapshot avoids a path replacement race while
            # retaining compatibility with large upstream checkpoints.
            with publication.open_snapshot(
                max_bytes=MAX_EXTERNAL_FILE_BYTES
            ) as snapshot:
                if snapshot is None:
                    raise DemoReplayError(f"SKRL checkpoint is missing: {source}")
                modules = torch.load(
                    snapshot.file,
                    map_location=getattr(agent, "device", "cpu"),
                    weights_only=False,
                )
        else:
            checkpoint_bytes = publication.read(
                lambda payload: payload,
                max_bytes=MAX_NATIVE_CHECKPOINT_BYTES,
            )
            if checkpoint_bytes is None:
                raise DemoReplayError(f"SKRL checkpoint is missing: {source}")
            modules = torch.load(
                io.BytesIO(checkpoint_bytes),
                map_location=getattr(agent, "device", "cpu"),
                weights_only=False,
            )
    except (
        OSError,
        PublicationError,
        RuntimeError,
        TypeError,
        ValueError,
        EOFError,
        AttributeError,
        IndexError,
        KeyError,
        pickle.UnpicklingError,
    ) as exc:
        raise DemoReplayError(f"cannot load SKRL checkpoint: {source}") from exc
    if type(modules) is not dict:
        raise DemoReplayError("SKRL checkpoint root must be a dictionary")

    replay_present = REPLAY_CHECKPOINT_MODULE in modules
    replay_state = modules.get(REPLAY_CHECKPOINT_MODULE)
    if replay_present and allow_replay and replay_module is None:
        raise DemoReplayError(
            "demo replay checkpoint requires --demo-dataset and "
            "--demo-max-transitions for training resume"
        )
    if allow_replay and replay_module is not None and not replay_present:
        raise DemoReplayError(
            "demo replay checkpoint state is missing from the requested training resume"
        )
    use_replay = replay_present and allow_replay and replay_module is not None
    if use_replay:
        if set(modules) != set(agent.checkpoint_modules):
            raise DemoReplayError("SKRL demo checkpoint native modules differ")
        validate_replay_checkpoint_state(
            replay_state,
            agent=agent,
            expected_binding=getattr(agent, "_srb_demo_replay_binding", None),
        )

    snapshots: dict[str, Any] = {}
    previous_replay_restored = bool(getattr(agent, "_srb_demo_replay_restored", False))
    for name, module in agent.checkpoint_modules.items():
        if name == REPLAY_CHECKPOINT_MODULE and not use_replay:
            continue
        if name not in modules:
            continue
        if not hasattr(module, "state_dict"):
            raise DemoReplayError(
                f"SKRL checkpoint module {name!r} cannot be snapshotted"
            )
        snapshots[name] = _clone_value(module.state_dict())

    completed = False
    try:
        for name, data in modules.items():
            if name == REPLAY_CHECKPOINT_MODULE:
                continue
            module = agent.checkpoint_modules.get(name)
            if module is None:
                continue
            if not hasattr(module, "load_state_dict"):
                raise DemoReplayError(
                    f"SKRL checkpoint module {name!r} cannot be restored"
                )
            module.load_state_dict(data)
            if hasattr(module, "eval"):
                module.eval()
        if use_replay:
            replay_module.load_state_dict(replay_state)
        # A weights-only load must not inherit an earlier replay restoration.
        agent._srb_demo_replay_restored = use_replay
        completed = True
    except (
        RuntimeError,
        ValueError,
        TypeError,
        LookupError,
        OSError,
        MemoryError,
    ) as exc:
        raise DemoReplayError("SKRL checkpoint restore failed") from exc
    finally:
        if not completed:
            try:
                for name, data in snapshots.items():
                    module = agent.checkpoint_modules.get(name)
                    if module is not None and hasattr(module, "load_state_dict"):
                        module.load_state_dict(data)
            except (
                RuntimeError,
                ValueError,
                TypeError,
                LookupError,
                OSError,
                MemoryError,
            ) as rollback_exc:
                raise DemoReplayError(
                    "SKRL checkpoint restore failed and rollback failed"
                ) from rollback_exc
            finally:
                agent._srb_demo_replay_restored = previous_replay_restored


def _source_binding(source: Any, *, contract: Mapping[str, Any]) -> dict[str, Any]:
    """Build the immutable binding for one validated source generation."""

    try:
        from srb.datasets.transitions import build_demo_source_binding

        return build_demo_source_binding(source, contract=contract)
    except (AttributeError, KeyError, OverflowError, TypeError, ValueError) as exc:
        raise DemoReplayError(str(exc)) from exc


def _normalise_binding(value: Any) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise DemoReplayError("canonical demo binding must be a mapping")
    try:
        payload = json.loads(json.dumps(value, sort_keys=True, default=_json_default))
    except (TypeError, ValueError, OverflowError) as exc:
        raise DemoReplayError("canonical demo binding is not JSON-compatible") from exc
    if not isinstance(payload, dict):
        raise DemoReplayError("canonical demo binding must be a JSON object")
    encoded = _canonical_bytes(payload)
    if len(encoded) > MAX_NATIVE_REPLAY_METADATA_BYTES:
        raise DemoReplayError("canonical demo binding exceeds metadata budget")
    return payload


def _project_transition(
    transition: Any,
    wrapper: "SkrlEnvWrapper",
    *,
    index: int,
) -> tuple[torch.Tensor, ...]:
    try:
        observations = wrapper.observation_from_value(transition.obs)
        next_observations = wrapper.observation_from_value(transition.next_obs)
        state_from_value = getattr(wrapper, "state_from_value", None)
        if not callable(state_from_value):
            raise DemoReplayError(
                "canonical demo transition cannot project a historical state "
                "without the explicit state_from_value seam"
            )
        states = state_from_value(transition.obs)
        next_states = state_from_value(transition.next_obs)
        if states is None:
            states = observations
        if next_states is None:
            next_states = next_observations
    except (TypeError, ValueError, KeyError, RuntimeError) as exc:
        raise DemoReplayError(
            f"canonical demo transition {index} observation layout mismatch: {exc}"
        ) from exc
    if not all(
        isinstance(value, torch.Tensor)
        for value in (observations, states, next_observations, next_states)
    ):
        raise DemoReplayError(
            f"canonical demo transition {index} observations are not tensors"
        )
    if (
        observations.ndim != 2
        or next_observations.ndim != 2
        or states.ndim != 2
        or next_states.ndim != 2
        or observations.shape[0] != 1
        or next_observations.shape[0] != 1
        or states.shape[0] != 1
        or next_states.shape[0] != 1
    ):
        raise DemoReplayError(
            f"canonical demo transition {index} observations must project to (1, dim)"
        )
    if observations.shape != next_observations.shape:
        raise DemoReplayError(
            f"canonical demo transition {index} observation and next observation shapes differ"
        )
    if states.shape != next_states.shape:
        raise DemoReplayError(
            f"canonical demo transition {index} state and next state shapes differ"
        )

    action = _coerce_action(transition.action, wrapper, index=index)
    if action.ndim != 2 or action.shape[0] != 1:
        raise DemoReplayError(
            f"canonical demo transition {index} action must project to (1, dim)"
        )
    return (
        observations.detach().to(dtype=torch.float32, device="cpu").clone(),
        states.detach().to(dtype=torch.float32, device="cpu").clone(),
        action.detach().to(dtype=torch.float32, device="cpu").clone(),
        torch.tensor([[float(transition.reward)]], dtype=torch.float32),
        next_observations.detach().to(dtype=torch.float32, device="cpu").clone(),
        next_states.detach().to(dtype=torch.float32, device="cpu").clone(),
        torch.tensor([[transition.terminated]], dtype=torch.bool),
        torch.tensor([[transition.truncated]], dtype=torch.bool),
    )


def _coerce_action(
    raw_action: Any,
    wrapper: "SkrlEnvWrapper",
    *,
    index: int,
) -> torch.Tensor:
    try:
        array = np.asarray(raw_action)
    except (TypeError, ValueError) as exc:
        raise DemoReplayError(
            f"canonical demo transition {index} action is not an array"
        ) from exc
    if array.dtype == np.dtype("O") or not np.issubdtype(array.dtype, np.number):
        raise DemoReplayError(
            f"canonical demo transition {index} action dtype is not numeric"
        )
    if np.issubdtype(array.dtype, np.complexfloating):
        raise DemoReplayError(f"canonical demo transition {index} action must be real")
    if not np.isfinite(array).all():
        raise DemoReplayError(f"canonical demo transition {index} action is not finite")
    array = array.reshape(-1)
    space = wrapper.single_action_space
    expected_shape = tuple(int(dim) for dim in space.shape)
    if tuple(array.shape) != expected_shape:
        raise DemoReplayError(
            f"canonical demo transition {index} action shape {tuple(array.shape)} "
            f"does not match live shape {expected_shape}"
        )
    if not np.issubdtype(np.dtype(space.dtype), np.floating):
        raise DemoReplayError("SKRL replay requires a floating live action space")

    low = np.asarray(space.low, dtype=np.float64).reshape(-1)
    high = np.asarray(space.high, dtype=np.float64).reshape(-1)
    values = np.asarray(array, dtype=np.float64)
    # Canonical transitions already carry SRB's normalized policy coordinates.
    # There is no affine physical-action conversion at this seam: a declared
    # source domain must equal the live normalized domain exactly.
    if np.any(values < low) or np.any(values > high):
        raise DemoReplayError(
            f"canonical demo transition {index} action is outside live action bounds"
        )
    if not array.flags.writeable:
        array = np.array(array, copy=True)
    return torch.as_tensor(array, dtype=torch.float32).reshape(1, -1)


def _validate_action_binding(
    binding: Mapping[str, Any],
    wrapper: "SkrlEnvWrapper",
    *,
    index: int,
) -> None:
    contract = _demo_io_contract(binding)
    action = contract.get("action")
    if not isinstance(action, Mapping):
        raise DemoReplayError("canonical demo action contract is missing")
    live = _live_action_binding(wrapper)
    if action.get("source") != live["source"]:
        raise DemoReplayError(
            "canonical demo action source differs from the live action source"
        )
    declared_dim = action.get("action_dim")
    if declared_dim != live["dim"]:
        raise DemoReplayError(
            f"canonical demo action_dim {declared_dim} does not match live {live['dim']}"
        )
    schema = action.get("schema")
    if not isinstance(schema, list) or len(schema) != len(live["schema"]):
        raise DemoReplayError("canonical demo action schema differs from live layout")
    required_term_keys = ("name", "path", "shape", "dim", "start", "end")
    for component, expected in zip(schema, live["schema"], strict=True):
        if not isinstance(component, Mapping):
            raise DemoReplayError("canonical demo action schema term is not an object")
        for key in required_term_keys:
            if component.get(key) != expected[key]:
                raise DemoReplayError(
                    f"canonical demo action schema {key!r} differs at transition {index}"
                )
        for key in ("dtype", "low", "high"):
            if key not in component or component[key] != expected.get(key):
                raise DemoReplayError(
                    f"canonical demo action schema {key!r} differs at transition {index}"
                )
        if "scale" in expected:
            if component.get("scale") != expected["scale"]:
                raise DemoReplayError(
                    f"canonical demo action schema 'scale' differs at transition {index}"
                )
        elif "scale" in component:
            raise DemoReplayError(
                f"canonical demo action schema 'scale' differs at transition {index}"
            )

    normalized = action.get("normalized")
    if normalized is not True:
        raise DemoReplayError(
            "SKRL demo actions are required in normalized live coordinates"
        )
    if action.get("low") != live["low"] or action.get("high") != live["high"]:
        raise DemoReplayError(
            "canonical demo action bounds do not match normalized live bounds"
        )
    if live["scales"]:
        if action.get("scales") != live["scales"]:
            raise DemoReplayError(
                "canonical demo action scales do not match the live action layout"
            )
    elif "scales" in action and action["scales"] != {}:
        raise DemoReplayError("canonical demo action scales differ from live layout")


def _validate_observation_binding(
    binding: Mapping[str, Any],
    wrapper: "SkrlEnvWrapper",
) -> None:
    """Compare declared observation projection metadata with the live layout."""

    method = getattr(wrapper, "demo_observation_binding", None)
    if not callable(method):
        raise DemoReplayError(
            "SKRL demo prefill requires the live observation projection binding"
        )
    live = method()
    contract = _demo_io_contract(binding)
    declared = contract.get("observation")
    if not isinstance(declared, Mapping):
        raise DemoReplayError("canonical demo observation contract is missing")
    required = (
        "wrapper",
        "obs_key",
        "projection",
        "projection_path",
        "selection",
        "selection_present",
        "root_observation_key",
        "root",
        "keys",
        "terms",
    )
    for key in required:
        if key not in declared or declared[key] != live.get(key):
            raise DemoReplayError(
                f"canonical demo observation projection field {key!r} differs"
            )
    terms = declared["terms"]
    live_terms = live.get("terms")
    if not isinstance(terms, list) or not isinstance(live_terms, list):
        raise DemoReplayError("canonical demo observation terms are invalid")
    if len(terms) != len(live_terms):
        raise DemoReplayError("canonical demo observation term count differs")
    for component, expected in zip(terms, live_terms, strict=True):
        if not isinstance(component, Mapping) or component != expected:
            raise DemoReplayError("canonical demo observation term differs")


def _stack_projected_rows(
    rows: Sequence[tuple[torch.Tensor, ...]],
) -> tuple[torch.Tensor, ...]:
    try:
        tensors = tuple(
            torch.cat([row[column] for row in rows], dim=0) for column in range(8)
        )
    except (IndexError, RuntimeError) as exc:
        raise DemoReplayError(
            "canonical demo transitions have inconsistent projected shapes"
        ) from exc
    return cast(tuple[torch.Tensor, ...], tensors)


def _validate_projected_capacity(wrapper: "SkrlEnvWrapper", count: int) -> None:
    """Admit the projected source size before allocating staged row tensors."""

    try:
        observation_dim = _space_flat_dim(
            wrapper.single_observation_space,
            resource="SKRL projected observation space",
        )
        state_space = getattr(wrapper, "state_space", None)
        state_dim = _space_flat_dim(
            state_space or wrapper.single_observation_space,
            resource="SKRL projected state space",
        )
        action_dim = _space_flat_dim(
            wrapper.single_action_space,
            resource="SKRL projected action space",
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise DemoReplayError(
            "cannot establish the projected SKRL demo replay shape"
        ) from exc
    if observation_dim <= 0 or state_dim <= 0 or action_dim <= 0:
        raise DemoReplayError("projected SKRL demo replay shapes must be non-empty")
    shapes = (
        (count, observation_dim),
        (count, state_dim),
        (count, action_dim),
        (count, 1),
        (count, observation_dim),
        (count, state_dim),
        (count, 1),
        (count, 1),
    )
    dtypes = (
        np.dtype(np.float32),
        np.dtype(np.float32),
        np.dtype(np.float32),
        np.dtype(np.float32),
        np.dtype(np.float32),
        np.dtype(np.float32),
        np.dtype(np.bool_),
        np.dtype(np.bool_),
    )
    total_bytes = 0
    try:
        for index, (shape, dtype) in enumerate(zip(shapes, dtypes, strict=True)):
            total_bytes += checked_shape_nbytes(
                shape,
                dtype,
                resource=f"SKRL projected demo replay tensor {index}",
                max_bytes=MAX_NATIVE_REPLAY_BYTES,
            )
    except (TypeError, ValueError, OverflowError) as exc:
        raise DemoReplayError(
            "projected SKRL demo replay exceeds the native memory budget"
        ) from exc
    if total_bytes > MAX_NATIVE_REPLAY_BYTES:
        raise DemoReplayError(
            "projected SKRL demo replay exceeds the native memory budget"
        )


def _space_flat_dim(space: Any, *, resource: str) -> int:
    """Return a checked float32 flat dimension for one Gym space."""

    try:
        shape = tuple(space.shape)
        byte_count = checked_shape_nbytes(
            shape,
            np.dtype(np.float32),
            resource=resource,
            max_bytes=MAX_ARRAY_BYTES,
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise DemoReplayError(f"{resource} has an invalid bounded shape") from exc
    return byte_count // np.dtype(np.float32).itemsize


def _validate_staged_payload(
    tensors: Mapping[str, torch.Tensor] | tuple[torch.Tensor, ...],
    *,
    binding: Mapping[str, Any],
) -> None:
    if isinstance(tensors, Mapping):
        values = tensors
    else:
        values = dict(zip(_REPLAY_TENSOR_NAMES, tensors, strict=True))
    if tuple(values) != _REPLAY_TENSOR_NAMES:
        raise DemoReplayError(
            "staged demo replay tensor names are incomplete or reordered"
        )
    count: int | None = None
    total_bytes = 0
    expected_dtypes = {
        "states": torch.float32,
        "actions": torch.float32,
        "rewards": torch.float32,
        "next_states": torch.float32,
        "terminated": torch.bool,
        "truncated": torch.bool,
    }
    for name in _REPLAY_TENSOR_NAMES:
        tensor = values[name]
        if not isinstance(tensor, torch.Tensor):
            raise DemoReplayError(f"staged demo replay {name} is not a tensor")
        if tensor.ndim != 2:
            raise DemoReplayError(f"staged demo replay {name} must be rank 2")
        if count is None:
            count = int(tensor.shape[0])
        elif int(tensor.shape[0]) != count:
            raise DemoReplayError("staged demo replay tensor row counts differ")
        if tensor.dtype != expected_dtypes[name]:
            raise DemoReplayError(
                f"staged demo replay {name} dtype {tensor.dtype} != {expected_dtypes[name]}"
            )
        try:
            total_bytes += checked_shape_nbytes(
                tensor.shape,
                np.dtype(str(tensor.detach().cpu().numpy().dtype)),
                resource=f"SKRL demo replay {name}",
                max_bytes=MAX_ARRAY_BYTES,
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise DemoReplayError(
                f"staged demo replay {name} exceeds array budget"
            ) from exc
        if tensor.dtype.is_floating_point and not torch.isfinite(tensor).all():
            raise DemoReplayError(
                f"staged demo replay {name} contains non-finite values"
            )
    if not count:
        raise DemoReplayError("staged demo replay contains no transitions")
    if total_bytes > MAX_NATIVE_REPLAY_BYTES:
        raise DemoReplayError("staged demo replay exceeds native memory budget")
    declared_count = binding.get("transition_count")
    if (
        isinstance(declared_count, bool)
        or not isinstance(declared_count, int)
        or declared_count != count
    ):
        raise DemoReplayError(
            "canonical demo binding transition count differs from payload"
        )


def _demo_io_contract(binding: Mapping[str, Any]) -> Mapping[str, Any]:
    contract = binding.get("demo_io_contract")
    if not isinstance(contract, Mapping):
        raise DemoReplayError("canonical demo source requires a demo_io_contract")
    if contract.get("version") != "srb.demo_io.v1":
        raise DemoReplayError("canonical demo I/O contract version is unsupported")
    return contract


def _validate_memory_contract(
    memory: Any,
    tensors: Mapping[str, torch.Tensor],
    *,
    observations: torch.Tensor | None = None,
    next_observations: torch.Tensor | None = None,
) -> None:
    names = tuple(memory.get_tensor_names())
    required = ("actions", "rewards", "terminated")
    if "states" in names or "next_states" in names:
        required += ("states", "next_states")
    elif "observations" not in names or "next_observations" not in names:
        raise DemoReplayError(
            "native SKRL memory is missing both state and observation transitions"
        )
    if any(name not in names for name in required):
        raise DemoReplayError(
            "native SKRL memory is not initialized for off-policy transitions"
        )
    for name in required:
        native = _memory_flat_tensor(memory, name)
        staged = tensors[name]
        if tuple(native.shape[1:]) != tuple(staged.shape[1:]):
            raise DemoReplayError(
                f"native replay {name} shape {tuple(native.shape[1:])} does not match "
                f"staged {tuple(staged.shape[1:])}"
            )
        if native.dtype != staged.dtype:
            raise DemoReplayError(
                f"native replay {name} dtype does not match staged payload"
            )
    for native_name, staged_name in (
        ("observations", "states"),
        ("next_observations", "next_states"),
    ):
        if native_name not in names:
            continue
        native = _memory_flat_tensor(memory, native_name)
        staged = (
            observations
            if native_name == "observations" and observations is not None
            else next_observations
            if native_name == "next_observations" and next_observations is not None
            else tensors.get(staged_name)
        )
        if staged is None or tuple(native.shape[1:]) != tuple(staged.shape[1:]):
            raise DemoReplayError(
                f"native replay {native_name} shape does not match staged payload"
            )
        if native.dtype != staged.dtype:
            raise DemoReplayError(
                f"native replay {native_name} dtype does not match staged payload"
            )


def _validate_optional_payload_tensor(
    memory: Any,
    value: torch.Tensor | None,
    *,
    native_name: str,
    count: int,
) -> None:
    if value is None or native_name not in _memory_tensor_names(memory):
        return
    native = _memory_flat_tensor(memory, native_name)
    if tuple(value.shape) != (count, *tuple(native.shape[1:])):
        raise DemoReplayError(
            f"staged demo replay {native_name} shape does not match native memory"
        )
    if value.dtype != native.dtype:
        raise DemoReplayError(
            f"staged demo replay {native_name} dtype does not match native memory"
        )
    if value.dtype.is_floating_point and not torch.isfinite(value).all():
        raise DemoReplayError(f"staged demo replay {native_name} is non-finite")


def _memory_capacity(memory: Any) -> int:
    return int(memory.memory_size) * int(memory.num_envs)


def _memory_row_bytes(memory: Any) -> int:
    total = 0
    for name in _memory_tensor_names(memory):
        tensor = _memory_flat_tensor(memory, name)
        total += int(tensor[0].numel() * tensor.element_size())
    return total


def _validate_checkpoint_memory_capacity(memory: Any) -> None:
    capacity = _memory_capacity(memory)
    row_bytes = _memory_row_bytes(memory)
    replay_bytes = capacity * row_bytes
    if replay_bytes <= MAX_NATIVE_REPLAY_BYTES:
        return
    num_envs = int(memory.num_envs)
    max_memory_size = MAX_NATIVE_REPLAY_BYTES // (num_envs * row_bytes)
    raise DemoReplayError(
        "configured SKRL demo replay capacity exceeds the checkpoint policy: "
        f"memory_size={int(memory.memory_size)}, num_envs={num_envs}, "
        f"maximum memory_size={max_memory_size} for this replay layout"
    )


def _insertion_slot_indices(memory: Any, count: int) -> list[int]:
    start = int(memory.memory_index) * int(memory.num_envs) + int(memory.env_index)
    capacity = _memory_capacity(memory)
    if start < 0 or start + count > capacity:
        raise DemoReplayError("native replay insertion slots exceed capacity")
    return list(range(start, start + count))


def _capture_memory_state(
    memory: Any,
    *,
    indices: Sequence[int] | None = None,
) -> dict[str, Any]:
    active_count = int(len(memory))
    selected = (
        list(range(active_count))
        if indices is None
        else [int(index) for index in indices]
    )
    capacity = _memory_capacity(memory)
    if any(index < 0 or index >= capacity for index in selected):
        raise DemoReplayError("native replay snapshot slot is invalid")
    if len(selected) * _memory_row_bytes(memory) > MAX_NATIVE_REPLAY_BYTES:
        raise DemoReplayError("native replay rollback snapshot exceeds size limit")
    tensors: dict[str, torch.Tensor] = {}
    for name in _memory_tensor_names(memory):
        tensor = _memory_flat_tensor(memory, name)
        tensors[name] = tensor[selected].detach().clone()
    state: dict[str, Any] = {
        "memory_size": int(memory.memory_size),
        "num_envs": int(memory.num_envs),
        "active_count": active_count,
        "memory_index": int(memory.memory_index),
        "env_index": int(memory.env_index),
        "filled": bool(memory.filled),
        "tensors": tensors,
    }
    if indices is not None:
        state["indices"] = selected
    return state


def _restore_memory_state(memory: Any, state: Mapping[str, Any]) -> None:
    if int(state["memory_size"]) != int(memory.memory_size) or int(
        state["num_envs"]
    ) != int(memory.num_envs):
        raise DemoReplayError("native replay memory dimensions differ from checkpoint")
    tensors = state["tensors"]
    if not isinstance(tensors, Mapping):
        raise DemoReplayError("native replay checkpoint tensors must be a mapping")
    if any(name not in tensors for name in _memory_tensor_names(memory)):
        raise DemoReplayError("native replay checkpoint tensor set is incomplete")
    active_count = int(state.get("active_count", 0))
    if active_count < 0 or active_count > _memory_capacity(memory):
        raise DemoReplayError("native replay checkpoint active count is invalid")
    indices = state.get("indices")
    if indices is not None:
        if not isinstance(indices, (list, tuple)):
            raise DemoReplayError("native replay rollback slots must be a sequence")
        checked_indices = [int(index) for index in indices]
        if any(
            index < 0 or index >= _memory_capacity(memory) for index in checked_indices
        ):
            raise DemoReplayError("native replay rollback slot is invalid")
    else:
        checked_indices = None
    for name in _memory_tensor_names(memory):
        source = tensors.get(name)
        if source is None and name == "observations":
            source = tensors.get("states")
        elif source is None and name == "next_observations":
            source = tensors.get("next_states")
        if source is None:
            raise DemoReplayError(f"native replay checkpoint is missing {name!r}")
        target = _memory_flat_tensor(memory, name)
        if checked_indices is not None:
            expected_shape = (len(checked_indices), *tuple(target.shape[1:]))
            if tuple(source.shape) != expected_shape:
                raise DemoReplayError(f"native replay rollback {name} shape differs")
        else:
            compact_shape = (active_count, *tuple(target.shape[1:]))
            if tuple(source.shape) not in (tuple(target.shape), compact_shape):
                raise DemoReplayError(f"native replay checkpoint {name} shape differs")
        if source.dtype != target.dtype:
            raise DemoReplayError(f"native replay checkpoint {name} dtype differs")
    with torch.no_grad():
        for name in _memory_tensor_names(memory):
            target = _memory_flat_tensor(memory, name)
            source = tensors.get(name)
            if source is None and name == "observations":
                source = tensors["states"]
            elif source is None and name == "next_observations":
                source = tensors["next_states"]
            if source is None:
                raise DemoReplayError(f"native replay checkpoint is missing {name!r}")
            source = source.to(memory.device)
            if checked_indices is not None:
                for offset, index in enumerate(checked_indices):
                    target[index].copy_(source[offset])
            elif source.shape == target.shape:
                target.copy_(source)
            else:
                target[:active_count].copy_(source)
    memory.memory_index = int(state["memory_index"])
    memory.env_index = int(state["env_index"])
    memory.filled = bool(state["filled"])


def _validate_replay_state(
    state: Mapping[str, Any],
    *,
    agent: Any,
    expected_binding: Mapping[str, Any] | None,
) -> dict[str, Any]:
    if state.get("version") != REPLAY_STATE_VERSION:
        raise DemoReplayError("unsupported SKRL demo replay checkpoint version")
    binding = state.get("binding")
    if not isinstance(binding, Mapping):
        raise DemoReplayError("SKRL demo replay checkpoint binding is missing")
    normalized_binding = _normalise_binding(binding)
    if expected_binding is not None:
        expected_fingerprint = expected_binding.get("dataset_fingerprint")
        recorded_fingerprint = normalized_binding.get("dataset_fingerprint")
        if expected_fingerprint != recorded_fingerprint:
            raise DemoReplayError(
                "SKRL demo replay checkpoint dataset fingerprint does not match request"
            )
        if _binding_without_path(expected_binding) != _binding_without_path(
            normalized_binding
        ):
            raise DemoReplayError("SKRL demo replay checkpoint layout binding differs")

    memory = state.get("memory")
    if not isinstance(memory, Mapping):
        raise DemoReplayError("SKRL demo replay checkpoint memory is missing")
    native_memory = getattr(agent, "memory", None)
    if native_memory is None:
        raise DemoReplayError("cannot validate replay checkpoint without native memory")
    try:
        memory_size = memory.get("memory_size")
        num_envs = memory.get("num_envs")
        if (
            isinstance(memory_size, bool)
            or not isinstance(memory_size, int)
            or isinstance(num_envs, bool)
            or not isinstance(num_envs, int)
        ):
            raise DemoReplayError(
                "SKRL demo replay checkpoint memory dimensions are invalid"
            )
        if memory_size != int(native_memory.memory_size) or num_envs != int(
            native_memory.num_envs
        ):
            raise DemoReplayError(
                "SKRL demo replay checkpoint memory dimensions differ"
            )
        active_count = require_bounded_int(
            memory.get("active_count"),
            resource="SKRL replay active count",
            minimum=0,
            maximum=_memory_capacity(native_memory),
        )
        memory_index = require_bounded_int(
            memory.get("memory_index"),
            resource="SKRL replay memory index",
            minimum=0,
            maximum=int(native_memory.memory_size) - 1,
        )
        env_index = require_bounded_int(
            memory.get("env_index"),
            resource="SKRL replay environment index",
            minimum=0,
            maximum=int(native_memory.num_envs) - 1,
        )
    except (TypeError, ValueError) as exc:
        if isinstance(exc, DemoReplayError):
            raise
        raise DemoReplayError("SKRL demo replay checkpoint cursor is invalid") from exc
    filled = memory.get("filled")
    if type(filled) is not bool:
        raise DemoReplayError("SKRL demo replay checkpoint filled flag is invalid")
    expected_active = (
        _memory_capacity(native_memory)
        if filled
        else memory_index * int(native_memory.num_envs) + env_index
    )
    if active_count != expected_active:
        raise DemoReplayError(
            "SKRL demo replay checkpoint cursor and active count disagree"
        )
    if active_count % int(native_memory.num_envs) != 0 or env_index != 0:
        raise DemoReplayError(
            "SKRL demo replay checkpoint cursor is not environment-aligned"
        )
    tensors = memory.get("tensors")
    native_names = _memory_tensor_names(native_memory)
    if not isinstance(tensors, Mapping):
        raise DemoReplayError("SKRL demo replay checkpoint tensor set is invalid")
    allowed_names = set(_REPLAY_TENSOR_NAMES) | set(
        ("observations", "next_observations")
    )
    if any(name not in allowed_names for name in tensors):
        raise DemoReplayError("SKRL demo replay checkpoint tensor set is invalid")
    fallback_names = {"observations": "states", "next_observations": "next_states"}
    required_names = {name for name in native_names if name != "truncated"}
    if any(
        name not in tensors and fallback_names.get(name) not in tensors
        for name in required_names
    ):
        raise DemoReplayError("SKRL demo replay checkpoint tensor set is incomplete")
    total_bytes = 0
    checked: dict[str, torch.Tensor] = {}
    for name in native_names:
        native = _memory_flat_tensor(native_memory, name)
        value = tensors.get(name)
        if value is None and name == "observations":
            value = tensors.get("states")
        elif value is None and name == "next_observations":
            value = tensors.get("next_states")
        elif value is None and name == "truncated":
            value = torch.zeros(
                (active_count, *tuple(native.shape[1:])),
                dtype=native.dtype,
            )
        if value is None:
            raise DemoReplayError(
                f"SKRL demo replay checkpoint tensor {name!r} is missing"
            )
        if not isinstance(value, torch.Tensor):
            raise DemoReplayError(f"SKRL demo replay checkpoint {name} is not a tensor")
        expected_shape = (active_count, *tuple(native.shape[1:]))
        if tuple(value.shape) != expected_shape:
            raise DemoReplayError(
                f"SKRL demo replay checkpoint {name} shape {tuple(value.shape)} "
                f"!= {expected_shape}"
            )
        if value.dtype != native.dtype:
            raise DemoReplayError(f"SKRL demo replay checkpoint {name} dtype differs")
        total_bytes += int(value.numel() * value.element_size())
        if value.dtype.is_floating_point and not torch.isfinite(value).all():
            raise DemoReplayError(f"SKRL demo replay checkpoint {name} is non-finite")
        checked[name] = value.detach().to(device=native.device).clone()
    if total_bytes > MAX_NATIVE_REPLAY_BYTES:
        raise DemoReplayError("SKRL demo replay checkpoint payload exceeds size limit")

    training = _validate_training_state(state.get("training"), agent)
    rng = _validate_rng_state(state.get("rng"))
    return {
        "binding": normalized_binding,
        "memory": {
            "memory_size": int(memory["memory_size"]),
            "num_envs": int(memory["num_envs"]),
            "active_count": active_count,
            "memory_index": memory_index,
            "env_index": env_index,
            "filled": filled,
            "tensors": checked,
        },
        "training": training,
        "rng": rng,
    }


def _training_state(agent: Any) -> dict[str, Any]:
    state: dict[str, Any] = {
        "cursor": int(getattr(agent, "_srb_training_cursor", 0)),
        "reward_scale": _demo_reward_scale(agent),
        "entropy_coefficient": _clone_value(
            getattr(agent, "_entropy_coefficient", None)
        ),
        "target_entropy": _clone_value(getattr(agent, "_target_entropy", None)),
        "log_entropy_coefficient": _clone_value(
            getattr(agent, "log_entropy_coefficient", None)
        ),
        "scaler": _clone_value(
            agent.scaler.state_dict()
            if hasattr(getattr(agent, "scaler", None), "state_dict")
            else {}
        ),
    }
    for name in ("policy_scheduler", "critic_scheduler"):
        scheduler = getattr(agent, name, None)
        state[name] = _clone_value(
            scheduler.state_dict() if hasattr(scheduler, "state_dict") else None
        )
    algorithm = getattr(agent, "demo_algorithm", None)
    if algorithm in {"skrl_ddpg", "skrl_td3"}:
        state["off_policy"] = {
            "algorithm": algorithm,
            "noise_contract": _off_policy_noise_contract(agent),
            "update_configuration": _off_policy_update_configuration(agent),
            "critic_update_counter": getattr(
                agent,
                "_update_counter",
                getattr(agent, "_critic_update_counter", None),
            ),
            "exploration_timesteps": getattr(agent, "_exploration_timesteps", None),
            "noise_states": {
                name: _clone_value(noise.state)
                for name in ("_exploration_noise", "_smooth_regularization_noise")
                if (noise := getattr(agent, name, None)) is not None
                and hasattr(noise, "state")
            },
        }
    return state


def _demo_reward_scale(agent: Any) -> float:
    """Admit only the stateless reward transformation used by online replay."""

    from srb.integrations.skrl.rewards import NativeRewardScale

    shaper = getattr(getattr(agent, "cfg", None), "rewards_shaper", None)
    if shaper is None:
        shaper = getattr(agent, "_rewards_shaper", None)
    if shaper is None:
        return 1.0
    if type(shaper) is not NativeRewardScale:
        raise DemoReplayError(
            "SKRL demo replay requires a declared constant reward scale; "
            "custom reward-shaping callbacks are unsupported"
        )
    scale = shaper.scale
    if (
        not isinstance(scale, (int, float, np.integer, np.floating))
        or isinstance(scale, (bool, np.bool_))
        or not math.isfinite(float(scale))
    ):
        raise DemoReplayError("SKRL demo reward scale must be finite")
    return float(scale)


def _validate_training_state(value: Any, agent: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise DemoReplayError("SKRL demo replay checkpoint training state is missing")
    cursor = require_bounded_int(
        value.get("cursor", 0),
        resource="SKRL training cursor",
        minimum=0,
        maximum=2**63 - 1,
    )
    result = dict(value)
    result["cursor"] = cursor
    # Older SAC version-1 states contain unscaled demonstration rewards.
    legacy_scale = 1.0 if getattr(agent, "demo_algorithm", None) == "skrl_sac" else None
    reward_scale = value.get("reward_scale", legacy_scale)
    if (
        not isinstance(reward_scale, (int, float))
        or isinstance(reward_scale, bool)
        or not math.isfinite(reward_scale)
        or reward_scale != _demo_reward_scale(agent)
    ):
        raise DemoReplayError("SKRL demo checkpoint reward scale differs")
    for name in ("entropy_coefficient", "target_entropy"):
        candidate = value.get(name)
        if candidate is None:
            continue
        if isinstance(candidate, torch.Tensor):
            if candidate.numel() != 1 or not candidate.dtype.is_floating_point:
                raise DemoReplayError(f"SKRL {name} state is invalid")
            if not torch.isfinite(candidate).all():
                raise DemoReplayError(f"SKRL {name} state is non-finite")
        elif isinstance(candidate, (int, float, np.number)) and not isinstance(
            candidate, (bool, np.bool_)
        ):
            if not math.isfinite(float(candidate)):
                raise DemoReplayError(f"SKRL {name} state is non-finite")
        else:
            raise DemoReplayError(f"SKRL {name} state is invalid")
    if "log_entropy_coefficient" in value:
        log_entropy = value["log_entropy_coefficient"]
        if log_entropy is not None and not isinstance(log_entropy, torch.Tensor):
            raise DemoReplayError("SKRL entropy state is not a tensor")
        current = getattr(agent, "log_entropy_coefficient", None)
        if (
            isinstance(log_entropy, torch.Tensor)
            and isinstance(current, torch.Tensor)
            and tuple(log_entropy.shape) != tuple(current.shape)
        ):
            raise DemoReplayError("SKRL entropy state shape differs")
        if isinstance(log_entropy, torch.Tensor) and (
            not log_entropy.dtype.is_floating_point
            or not torch.isfinite(log_entropy).all()
        ):
            raise DemoReplayError("SKRL entropy state is invalid")
    scaler = value.get("scaler")
    if scaler is not None and not isinstance(scaler, Mapping):
        raise DemoReplayError("SKRL scaler state is invalid")
    for name in ("policy_scheduler", "critic_scheduler"):
        scheduler = value.get(name)
        if scheduler is not None and not isinstance(scheduler, Mapping):
            raise DemoReplayError(f"SKRL {name} state is invalid")
        native = getattr(agent, name, None)
        if (scheduler is None) != (native is None) or (
            scheduler is not None and set(scheduler) != set(native.state_dict())
        ):
            raise DemoReplayError(f"SKRL {name} state/configuration fields differ")
    algorithm = getattr(agent, "demo_algorithm", None)
    off_policy = value.get("off_policy")
    if algorithm in {"skrl_ddpg", "skrl_td3"} or off_policy is not None:
        if not isinstance(off_policy, Mapping):
            raise DemoReplayError("SKRL off-policy algorithm state is missing")
        if (
            algorithm not in {"skrl_ddpg", "skrl_td3"}
            or off_policy.get("algorithm") != algorithm
        ):
            raise DemoReplayError("SKRL demo checkpoint algorithm differs")
        if off_policy.get("noise_contract") != _off_policy_noise_contract(agent):
            raise DemoReplayError("SKRL demo checkpoint noise configuration differs")
        _validate_off_policy_state(off_policy, agent)
    return result


def _validate_off_policy_state(state: Mapping[str, Any], agent: Any) -> None:
    """Reject malformed native counters and OU state before restoring modules."""

    if set(state) != {
        "algorithm",
        "noise_contract",
        "update_configuration",
        "critic_update_counter",
        "exploration_timesteps",
        "noise_states",
    }:
        raise DemoReplayError("SKRL off-policy checkpoint fields differ")
    if state["update_configuration"] != _off_policy_update_configuration(agent):
        raise DemoReplayError("SKRL off-policy update configuration differs")
    counter = state["critic_update_counter"]
    horizon = state["exploration_timesteps"]
    if agent.demo_algorithm == "skrl_td3":
        if type(counter) is not int or not 0 <= counter <= 2**63 - 1:
            raise DemoReplayError("SKRL off-policy critic update counter is invalid")
    elif counter is not None:
        raise DemoReplayError("SKRL off-policy critic update counter is unexpected")
    if horizon is not None and (
        type(horizon) is not int or not 1 <= horizon <= 2**63 - 1
    ):
        raise DemoReplayError("SKRL off-policy exploration horizon is invalid")
    expected_noises = {
        name
        for name, contract in _off_policy_noise_contract(agent).items()
        if contract is not None and contract["kind"] == "ou"
    }
    noise_states = state["noise_states"]
    if not isinstance(noise_states, Mapping) or set(noise_states) != expected_noises:
        raise DemoReplayError("SKRL off-policy noise state fields differ")
    action_shape = tuple(_memory_flat_tensor(agent.memory, "actions").shape[1:])
    for name, value in noise_states.items():
        if type(value) is int and value == 0:
            # The pinned OU provider uses scalar zero until its first sample.
            continue
        batch = (
            int(agent.memory.num_envs)
            if name == "_exploration_noise"
            else int(agent.cfg.batch_size)
        )
        minimum_batch = batch
        if name == "_smooth_regularization_noise" and not getattr(
            agent.memory, "_replacement", True
        ):
            # Sampling without replacement may have used fewer than batch_size
            # rows. Retain that prior OU state; the provider resets it itself
            # if a later sample grows to a different shape.
            minimum_batch = 1
            batch = min(batch, _memory_capacity(agent.memory))
        if (
            not isinstance(value, torch.Tensor)
            or value.ndim != len(action_shape) + 1
            or tuple(value.shape[1:]) != action_shape
            or not minimum_batch <= value.shape[0] <= batch
            or value.dtype != getattr(agent, name).distribution.loc.dtype
            or value.layout != torch.strided
            or value.numel() * value.element_size() > MAX_NATIVE_REPLAY_BYTES
            or not torch.isfinite(value).all()
        ):
            raise DemoReplayError("SKRL off-policy noise tensor is invalid")


def _off_policy_update_configuration(agent: Any) -> dict[str, Any]:
    """Bind native update, sampling and scheduler behavior used after resume."""

    cfg = agent.cfg
    exploration_scheduler = getattr(cfg, "exploration_scheduler", None)
    if exploration_scheduler is None:
        exploration_scheduler_contract = None
    else:
        if not callable(exploration_scheduler):
            raise DemoReplayError(
                "SKRL exploration scheduler must be callable for replay checkpoints"
            )
        try:
            samples = tuple(
                float(exploration_scheduler(timestep, 100)) for timestep in (0, 1, 100)
            )
        except (TypeError, ValueError, OverflowError) as exc:
            raise DemoReplayError(
                "SKRL exploration scheduler cannot be evaluated for replay checkpoints"
            ) from exc
        if not all(math.isfinite(value) for value in samples):
            raise DemoReplayError(
                "SKRL exploration scheduler must return finite scales"
            )
        exploration_scheduler_contract = {
            "callable": (
                f"{type(exploration_scheduler).__module__}."
                f"{type(exploration_scheduler).__qualname__}"
            ),
            "samples": samples,
        }
    result = {
        "policy_delay": getattr(cfg, "policy_delay", None),
        "exploration_scheduler": exploration_scheduler_contract,
        "memory_class": f"{type(agent.memory).__module__}.{type(agent.memory).__qualname__}",
        "sampling_with_replacement": getattr(agent.memory, "_replacement", None),
        "scheduler_kwargs": _clone_value(cfg.learning_rate_scheduler_kwargs),
    }
    for field in (
        "gradient_steps",
        "batch_size",
        "discount_factor",
        "polyak",
        "random_timesteps",
        "learning_starts",
        "grad_norm_clip",
        "smooth_regularization_clip",
        "mixed_precision",
    ):
        result[field] = getattr(cfg, field, None)
    for name in ("policy_scheduler", "critic_scheduler"):
        scheduler = getattr(agent, name, None)
        result[name] = (
            None
            if scheduler is None
            else f"{type(scheduler).__module__}.{type(scheduler).__qualname__}"
        )
    return _normalise_binding(result)


def _off_policy_noise_contract(agent: Any) -> dict[str, Any]:
    """Bind the pinned native noise families without serializing their objects."""

    from skrl.resources.noises.torch import GaussianNoise, OrnsteinUhlenbeckNoise

    result: dict[str, Any] = {}
    for name in ("_exploration_noise", "_smooth_regularization_noise"):
        noise = getattr(agent, name, None)
        if noise is None:
            result[name] = None
            continue
        if type(noise) not in {GaussianNoise, OrnsteinUhlenbeckNoise}:
            raise DemoReplayError(
                "SKRL demonstration checkpoints require native Gaussian or "
                f"Ornstein-Uhlenbeck noise; unsupported {name}"
            )
        values = {
            "mean": float(noise.distribution.loc.detach().cpu().item()),
            "std": float(noise.distribution.scale.detach().cpu().item()),
        }
        if type(noise) is OrnsteinUhlenbeckNoise:
            values.update(
                theta=float(noise.theta),
                sigma=float(noise.sigma),
                base_scale=float(noise.base_scale),
            )
        if any(not math.isfinite(value) for value in values.values()):
            raise DemoReplayError("SKRL demo noise parameters must be finite")
        if values["std"] <= 0:
            raise DemoReplayError("SKRL demo noise standard deviation must be positive")
        result[name] = {
            "kind": "ou" if type(noise) is OrnsteinUhlenbeckNoise else "gaussian",
            "parameters": values,
        }
    return result


def _restore_training_state(agent: Any, state: Mapping[str, Any]) -> None:
    agent._srb_training_cursor = int(state.get("cursor", 0))
    if "entropy_coefficient" in state and state["entropy_coefficient"] is not None:
        entropy_coefficient = state["entropy_coefficient"]
        agent._entropy_coefficient = (
            entropy_coefficient.to(agent.device)
            if isinstance(entropy_coefficient, torch.Tensor)
            else _clone_value(entropy_coefficient)
        )
    if "target_entropy" in state and state["target_entropy"] is not None:
        target_entropy = state["target_entropy"]
        agent._target_entropy = (
            target_entropy.to(agent.device)
            if isinstance(target_entropy, torch.Tensor)
            else _clone_value(target_entropy)
        )
    log_entropy = state.get("log_entropy_coefficient")
    current_log_entropy = getattr(agent, "log_entropy_coefficient", None)
    if isinstance(log_entropy, torch.Tensor) and isinstance(
        current_log_entropy, torch.Tensor
    ):
        with torch.no_grad():
            current_log_entropy.copy_(log_entropy.to(current_log_entropy.device))
    scaler_state = state.get("scaler")
    scaler = getattr(agent, "scaler", None)
    if isinstance(scaler_state, Mapping) and hasattr(scaler, "load_state_dict"):
        scaler.load_state_dict(scaler_state)
    for name in ("policy_scheduler", "critic_scheduler"):
        scheduler_state = state.get(name)
        scheduler = getattr(agent, name, None)
        if isinstance(scheduler_state, Mapping) and hasattr(
            scheduler, "load_state_dict"
        ):
            scheduler.load_state_dict(scheduler_state)
    off_policy = state.get("off_policy")
    if isinstance(off_policy, Mapping):
        counter = off_policy.get("critic_update_counter")
        if counter is not None:
            if hasattr(agent, "_update_counter"):
                agent._update_counter = int(counter)
            else:
                agent._critic_update_counter = int(counter)
        if hasattr(agent, "_exploration_timesteps"):
            agent._exploration_timesteps = off_policy["exploration_timesteps"]
        for name, value in off_policy["noise_states"].items():
            noise = getattr(agent, name)
            noise.state = (
                value.detach().to(noise.device).clone()
                if isinstance(value, torch.Tensor)
                else _clone_value(value)
            )


def _rng_state() -> dict[str, Any]:
    state: dict[str, Any] = {
        "python": random.getstate(),
        "numpy": np.random.get_state(),
        "torch": torch.get_rng_state().clone(),
    }
    if torch.cuda.is_available():
        state["cuda"] = [value.clone() for value in torch.cuda.get_rng_state_all()]
    return state


def _validate_rng_state(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise DemoReplayError("SKRL demo replay checkpoint RNG state is missing")
    cpu = value.get("torch")
    if (
        not isinstance(cpu, torch.Tensor)
        or cpu.dtype != torch.uint8
        or cpu.ndim != 1
        or cpu.numel() > MAX_NATIVE_REPLAY_BYTES
    ):
        raise DemoReplayError("SKRL demo replay checkpoint CPU RNG state is invalid")
    python_state = value.get("python")
    numpy_state = value.get("numpy")
    if not isinstance(python_state, tuple) or not isinstance(numpy_state, tuple):
        raise DemoReplayError("SKRL demo replay checkpoint host RNG state is invalid")
    try:
        random.Random().setstate(python_state)
        np.random.RandomState().set_state(numpy_state)
    except (TypeError, ValueError, OverflowError) as exc:
        raise DemoReplayError(
            "SKRL demo replay checkpoint host RNG state is invalid"
        ) from exc
    cuda = value.get("cuda", [])
    if (
        not isinstance(cuda, list)
        or len(cuda) > 256
        or not all(
            isinstance(item, torch.Tensor)
            and item.dtype == torch.uint8
            and item.ndim == 1
            and item.numel() <= MAX_NATIVE_REPLAY_BYTES
            for item in cuda
        )
    ):
        raise DemoReplayError("SKRL demo replay checkpoint CUDA RNG state is invalid")
    return {
        "python": python_state,
        "numpy": numpy_state,
        "torch": cpu.clone(),
        "cuda": [item.clone() for item in cuda],
    }


def _restore_rng_state(value: Mapping[str, Any]) -> None:
    random.setstate(value["python"])
    np.random.set_state(value["numpy"])
    torch.set_rng_state(value["torch"].cpu())
    if torch.cuda.is_available() and value.get("cuda"):
        torch.cuda.set_rng_state_all(value["cuda"])


def _live_action_binding(wrapper: "SkrlEnvWrapper") -> dict[str, Any]:
    space = wrapper.single_action_space
    low = np.asarray(space.low, dtype=np.float64).reshape(-1)
    high = np.asarray(space.high, dtype=np.float64).reshape(-1)
    schema: list[dict[str, Any]] = []
    layout = getattr(wrapper, "_action_layout", None)
    terms = getattr(layout, "terms", ())
    source_layout = getattr(
        getattr(wrapper, "_unwrapped", None), "_action_layout", None
    )
    source_terms = getattr(source_layout, "terms", ())
    source = (
        "runtime_action_layout" if source_terms else "effective_single_action_space"
    )
    scales = dict(getattr(layout, "scale", {})) if source_terms else {}
    if terms:
        for term in terms:
            start = term.action_slice.start
            end = term.action_slice.stop
            if start is None or end is None:
                raise DemoReplayError("live action layout has an incomplete slice")
            term_space = term.space
            term_low = np.asarray(term_space.low).reshape(-1).tolist()
            term_high = np.asarray(term_space.high).reshape(-1).tolist()
            term_schema: dict[str, Any] = {
                "name": str(term.key),
                "path": [str(term.key)],
                "shape": list(term.shape),
                "dim": int(end - start),
                "start": int(start),
                "end": int(end),
                "dtype": str(np.dtype(term.dtype)),
                "low": term_low,
                "high": term_high,
            }
            if str(term.key) in scales:
                scales_value = scales[str(term.key)]
                term_schema["scale"] = float(scales_value)
            schema.append(term_schema)
    else:
        dim = int(np.prod(space.shape, dtype=np.int64))
        schema.append(
            {
                "name": "action",
                "path": [],
                "shape": [dim],
                "dim": dim,
                "start": 0,
                "end": dim,
                "dtype": str(np.dtype(space.dtype)),
                "low": low.tolist(),
                "high": high.tolist(),
            }
        )
    result: dict[str, Any] = {
        "source": source,
        "dim": int(low.size),
        "low": low.tolist(),
        "high": high.tolist(),
        "schema": schema,
    }
    if scales:
        result["scales"] = {str(key): float(value) for key, value in scales.items()}
    else:
        result["scales"] = {}
    return result


def _binding_without_path(value: Mapping[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(dict(value))
    result.pop("dataset_path", None)
    result.pop("path", None)
    return result


def _clone_value(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        return value.detach().clone()
    return copy.deepcopy(value)


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
        default=_json_default,
    ).encode("utf-8")


def _json_default(value: Any) -> Any:
    if isinstance(value, Mapping):
        return dict(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"value {type(value).__name__} is not JSON serializable")


__all__ = [
    "DemoReplayCheckpointModule",
    "DemoReplayError",
    "DemoReplayPayload",
    "MAX_NATIVE_CHECKPOINT_BYTES",
    "MAX_NATIVE_REPLAY_BYTES",
    "REPLAY_CHECKPOINT_MODULE",
    "REPLAY_STATE_VERSION",
    "build_demo_replay_payload",
    "export_demo_replay_state",
    "install_demo_replay_checkpoint",
    "load_checkpoint_modules",
    "prefill_demo_replay",
    "restore_demo_replay_state",
    "save_checkpoint_modules",
    "save_checkpoint_values",
    "validate_demo_replay_configuration",
    "validate_replay_checkpoint_state",
]
