"""Policy loading helpers for DreamerV3 checkpoints."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np

from srb.utils.model_artifacts import (
    find_saved_policy_config,
    try_model_artifact_reference,
)
from srb.utils.policy_contract import LoadedPolicy

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def load_policy(
    model_path: Path,
    env: "AnyEnv",
    *,
    artifact_reference: Any | None = None,
) -> LoadedPolicy:
    """Load a DreamerV3 checkpoint as a collection/evaluation policy."""
    from srb.integrations.dreamer import configure_xla_flags

    configure_xla_flags()

    import elements
    from dreamerv3 import agent as dreamer_agent

    from srb.integrations.dreamer.checkpoint_transaction import checkpoint_read_lock
    from srb.integrations.dreamer.wrapper import EmbodiedEnvWrapper

    model_path = _normalize_checkpoint_path(model_path)
    wrapped_env = EmbodiedEnvWrapper(env)
    with checkpoint_read_lock(model_path) as load_path:
        # Landscaping-contract preflight: reject drifted checkpoints BEFORE agent
        # construction/inference (task ID, fingerprint, target hash, mapper hash,
        # normalization policy).
        validate_landscaping_checkpoint_contract(
            load_path,
            env=env,
            wrapped_env=wrapped_env,
            artifact_reference=artifact_reference,
        )
        projection = wrapped_env.observation_projection
        obs_space = {
            key: value
            for key, value in wrapped_env.obs_space.items()
            if not key.startswith("log/")
        }
        act_space = {
            key: value for key, value in wrapped_env.act_space.items() if key != "reset"
        }
        config = _load_config(load_path)
        config = config.update({"run.from_checkpoint": Path(load_path)})
        agent = dreamer_agent.Agent(
            obs_space,
            act_space,
            elements.Config(
                **config.agent,
                logdir=config.logdir,
                seed=config.seed,
                jax=config.jax,
                batch_size=config.batch_size,
                batch_length=config.batch_length,
                replay_context=config.replay_context,
                report_length=config.report_length,
                replica=config.replica,
                replicas=config.replicas,
            ),
        )
        checkpoint = elements.Checkpoint()
        checkpoint.agent = agent
        checkpoint.load(load_path, keys=["agent"])

    num_envs = int(getattr(env.unwrapped, "num_envs", 1))
    carry = None
    is_first = np.ones(num_envs, dtype=bool)

    def start_episode_fn(done_mask: Any | None = None) -> None:
        nonlocal carry, is_first
        # Full (re)start when there is no carry yet, or no per-env mask is given.
        if carry is None or done_mask is None:
            carry = agent.init_policy(num_envs)
            is_first = np.ones(num_envs, dtype=bool)
            return
        # Per-env: flag ONLY the envs that just finished. DreamerV3's policy()
        # resets the RSSM state for exactly those envs via the is_first mask
        # (rssm.py nn.mask), so the still-running envs keep their latent -- no
        # full-carry rebuild that would wipe every env at num_envs>1.
        is_first = np.logical_or(is_first, _as_env_mask(done_mask, num_envs))

    def policy_fn(obs: Mapping[str, Any]) -> np.ndarray:
        nonlocal carry, is_first
        dreamer_obs = _make_dreamer_observation(
            obs=obs,
            obs_space=obs_space,
            num_envs=num_envs,
            is_first=is_first,
            projection=projection,
        )
        carry, acts, _outs = agent.policy(carry, dreamer_obs, mode="eval")
        is_first = np.zeros(num_envs, dtype=bool)
        return _select_action(acts, act_space)

    start_episode_fn()
    # The pinned Dreamer policy samples both its action and posterior even when
    # called with ``mode="eval"``.  Keep bounded evaluation fail-closed rather
    # than treating the mode label as a deterministic-action guarantee.
    return policy_fn, start_episode_fn, False


#: Contract fields compared before a landscaping checkpoint reaches inference.
#: ``projection_layout_fingerprint`` is the "order" half of the
#: "shape/order" rejection criterion: the IO-schema fingerprint pins the leaf
#: set and shapes, the layout fingerprint pins the packed vector order.
LANDSCAPING_CONTRACT_PREFLIGHT_FIELDS = (
    "task_id",
    "io_schema_fingerprint",
    "projection_layout_fingerprint",
    "target_manifest_sha256",
    "mapper_profile_sha256",
    "normalization_policy",
    "target_identity_kind",
    "target_generator_version",
    "target_profile_sha256",
    "target_reset_layout_sha256",
)
_GENERAL_MISSION_IDENTITY = "general_mission_family_v1"
_GENERAL_MISSION_FAMILY_FIELDS = frozenset(
    {
        "target_identity_kind",
        "target_generator_version",
        "target_profile_sha256",
        "target_reset_layout_sha256",
    }
)

#: Operator escape hatch for the fail-closed preflight (see below). Set to a
#: truthy value to downgrade an unverifiable landscaping checkpoint to a
#: warning; the default is a hard rejection.
ALLOW_UNVERIFIED_ENV_VAR = "SRB_ALLOW_UNVERIFIED_LANDSCAPING_CHECKPOINT"


class LandscapingContractPreflightError(ValueError):
    """A landscaping checkpoint failed its pre-inference contract preflight."""


def validate_landscaping_checkpoint_contract(
    model_path: Path | str,
    *,
    env: Any,
    wrapped_env: Any | None = None,
    artifact_reference: Any | None = None,
    allow_unverified: bool | None = None,
) -> None:
    """Check landscaping checkpoint metadata against *env* before inference.

    Compares task ID, IO-schema and projection-layout/packed-order
    fingerprints, target-manifest and mapper-profile hashes, and normalization
    policy. Non-landscaping envs pass through. Missing manifest, contract, or
    declared field is rejected; ``allow_unverified`` or a truthy
    ``SRB_ALLOW_UNVERIFIED_LANDSCAPING_CHECKPOINT`` downgrades these cases to
    warnings.
    """
    expected = _env_landscaping_expectations(env, wrapped_env)
    if expected is None:
        return

    if allow_unverified is None:
        import os

        allow_unverified = bool(os.environ.get(ALLOW_UNVERIFIED_ENV_VAR))

    manifest = getattr(artifact_reference, "manifest", None)
    if manifest is None:
        try:
            reference = try_model_artifact_reference(Path(model_path))
        except (FileNotFoundError, ValueError):
            reference = None
        manifest = getattr(reference, "manifest", None)
    if manifest is None:
        _unverifiable(
            f"Dreamer checkpoint {model_path} has no SRB model manifest, so its "
            "landscaping contract cannot be verified before inference",
            allow_unverified=allow_unverified,
        )
        return

    recorded = manifest.metadata.get("landscaping_contract")
    if not isinstance(recorded, Mapping):
        _unverifiable(
            f"Model artifact for {model_path} records no landscaping_contract "
            "metadata, so contract drift cannot be verified before inference",
            allow_unverified=allow_unverified,
        )
        return

    # Per-episode target hashes are diagnostic for the general mission stream;
    # static crater and mound tasks continue to bind their checked-in manifest.
    if expected.get("target_identity_kind") == _GENERAL_MISSION_IDENTITY:
        expected.pop("target_manifest_sha256", None)

    mismatches: list[str] = []
    unverifiable: list[str] = []
    for field in LANDSCAPING_CONTRACT_PREFLIGHT_FIELDS:
        expected_value = expected.get(field)
        recorded_value = recorded.get(field)
        if expected_value is None:
            # The environment does not declare this field; nothing to compare.
            continue
        if recorded_value is None:
            unverifiable.append(field)
            continue
        if str(expected_value) != str(recorded_value):
            mismatches.append(
                f"{field}: checkpoint={recorded_value!r} env={expected_value!r}"
            )
    if mismatches:
        raise LandscapingContractPreflightError(
            "Refusing Dreamer checkpoint load: landscaping contract mismatch "
            "between checkpoint and environment — " + "; ".join(mismatches)
        )
    if unverifiable:
        _unverifiable(
            f"Model artifact for {model_path} records no value for landscaping "
            f"contract field(s) {sorted(unverifiable)} that this environment "
            "declares",
            allow_unverified=allow_unverified,
        )


def _unverifiable(message: str, *, allow_unverified: bool) -> None:
    if not allow_unverified:
        raise LandscapingContractPreflightError(
            f"Refusing Dreamer checkpoint load: {message}. Set "
            f"{ALLOW_UNVERIFIED_ENV_VAR}=1 (or pass allow_unverified=True) to "
            "load it anyway."
        )
    from srb.utils import logging

    logging.warning("%s; loading anyway (%s).", message, ALLOW_UNVERIFIED_ENV_VAR)


def _env_landscaping_expectations(
    env: Any, wrapped_env: Any | None
) -> dict[str, Any] | None:
    unwrapped = getattr(env, "unwrapped", env)
    metadata_fn = getattr(unwrapped, "landscaping_contract_metadata", None)
    projection = getattr(wrapped_env, "observation_projection", None)
    env_id = getattr(getattr(env, "spec", None), "id", None) or getattr(
        unwrapped, "ENV_ID", None
    )
    task_id = str(env_id).rsplit("/", 1)[-1] if env_id else None
    is_general_task = task_id == "terrain_landscaping"

    expected: dict[str, Any] = {}
    if callable(metadata_fn):
        task_meta = metadata_fn()
        expected["io_schema_fingerprint"] = task_meta.get("io_schema_fingerprint")
        expected["target_manifest_sha256"] = task_meta.get("target_manifest_sha256")
        expected["mapper_profile_sha256"] = task_meta.get("mapper_profile_sha256")
        if is_general_task:
            missing_family = sorted(
                field
                for field in _GENERAL_MISSION_FAMILY_FIELDS
                if task_meta.get(field) is None
            )
            if missing_family:
                raise LandscapingContractPreflightError(
                    "Live general landscaping environment is missing mission "
                    f"family field(s) {missing_family}"
                )
            for field in _GENERAL_MISSION_FAMILY_FIELDS:
                expected[field] = task_meta[field]
    elif projection is not None:
        if is_general_task:
            raise LandscapingContractPreflightError(
                "Live general landscaping environment does not publish mission "
                "family metadata"
            )
        expected["io_schema_fingerprint"] = projection.io_schema_fingerprint
        target_sha = getattr(unwrapped, "TARGET_MANIFEST_SHA256", None)
        if target_sha:
            expected["target_manifest_sha256"] = target_sha
    else:
        if is_general_task:
            raise LandscapingContractPreflightError(
                "Live general landscaping environment does not publish mission "
                "family metadata"
            )
        return None

    # Packed-vector order ("order" half of the shape/order criterion). Only
    # available once a projection exists for this env.
    if projection is not None:
        expected["projection_layout_fingerprint"] = projection.layout_fingerprint

    if task_id:
        expected["task_id"] = task_id
    expected["normalization_policy"] = (
        "metric observation units; upstream Dreamer symlog transform; "
        "no running normalizer"
    )
    return expected


def _normalize_checkpoint_path(model_path: Path) -> Path:
    """Return the Dreamer checkpoint directory accepted by elements.Checkpoint."""

    model_path = Path(model_path)
    if model_path.is_file() and model_path.name == "agent.pkl":
        return model_path.parent
    return model_path


def _load_config(model_path: Path) -> Any:
    from srb.integrations.dreamer import configure_xla_flags

    configure_xla_flags()

    import elements
    from dreamerv3 import agent as dreamer_agent
    from ruamel import yaml

    from srb.integrations.dreamer.config import _safe_update_config

    yaml_parser = yaml.YAML(typ="safe")
    upstream_config_path = Path(dreamer_agent.__file__).parent / "configs.yaml"
    upstream_configs = yaml_parser.load(
        upstream_config_path.read_text(encoding="utf-8")
    )
    config = elements.Config(upstream_configs["defaults"])

    saved_config_path = _find_saved_config(model_path)
    if saved_config_path is not None:
        saved_config = yaml_parser.load(saved_config_path.read_text(encoding="utf-8"))
        if saved_config:
            config, _ = _safe_update_config(config, saved_config)
    return config


def _find_saved_config(model_path: Path) -> Path | None:
    return find_saved_policy_config(_normalize_checkpoint_path(model_path))


def _as_env_mask(done_mask: Any, num_envs: int) -> np.ndarray:
    """Coerce a per-env done signal to ``(num_envs,)`` bools.

    A malformed shape resets every env to avoid mis-indexing.
    """
    if done_mask is None:
        return np.ones(num_envs, dtype=bool)
    if hasattr(done_mask, "detach"):
        done_mask = done_mask.detach().cpu().numpy()
    m = np.asarray(done_mask, dtype=bool).reshape(-1)
    return m if m.shape[0] == num_envs else np.ones(num_envs, dtype=bool)


def _make_dreamer_observation(
    *,
    obs: Mapping[str, Any],
    obs_space: Mapping[str, Any],
    num_envs: int,
    is_first: "bool | np.ndarray",
    projection: Any | None = None,
) -> dict[str, np.ndarray]:
    mapped_obs: dict[str, np.ndarray] = {}
    if projection is not None:
        # Identical contract transform as train/eval/collect: one flat metric
        # `state` leaf projected from the assembled observation.
        assembled = {
            key: _as_batched_array(value, num_envs) for key, value in obs.items()
        }
        mapped_obs["state"] = projection.project_numpy(assembled)
    else:
        for key, value in obs.items():
            dreamer_key = _resolve_obs_key(key, obs_space)
            mapped_obs[dreamer_key] = _as_batched_array(value, num_envs)

    mapped_obs["reward"] = np.zeros(num_envs, dtype=np.float32)
    # is_first may be a scalar (initial) or a per-env array; broadcast either.
    mapped_obs["is_first"] = np.broadcast_to(
        np.asarray(is_first, dtype=bool), (num_envs,)
    ).copy()
    mapped_obs["is_last"] = np.zeros(num_envs, dtype=bool)
    mapped_obs["is_terminal"] = np.zeros(num_envs, dtype=bool)
    return mapped_obs


def _resolve_obs_key(key: str, obs_space: Mapping[str, Any]) -> str:
    if key in obs_space:
        return key
    policy_key = f"policy/{key}"
    if policy_key in obs_space:
        return policy_key
    return key


def _as_batched_array(value: Any, num_envs: int) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    array = np.asarray(value)
    if array.shape[:1] == (num_envs,):
        return array
    if array.shape == ():
        return np.full((num_envs,), array.item(), dtype=array.dtype)
    return array.reshape((num_envs, *array.shape))


def _select_action(
    acts: Mapping[str, Any],
    act_space: Mapping[str, Any] | None = None,
) -> np.ndarray:
    selected_key: str | None = None
    selected_keys: list[str] | None = None
    if "action" in acts:
        selected_key = "action"
        action = acts["action"]
    elif len(acts) == 1:
        selected_key, action = next(iter(acts.items()))
    else:
        selected_key = None
        selected_keys = list(acts)
        action = np.concatenate(
            [
                np.asarray(acts[key]).reshape((np.asarray(acts[key]).shape[0], -1))
                for key in selected_keys
            ],
            axis=-1,
        )
    if hasattr(action, "detach"):
        action = action.detach().cpu().numpy()
    action_array = np.asarray(action)
    action_array = _clip_action(action_array, act_space, selected_key, selected_keys)
    return action_array[0] if action_array.shape[:1] == (1,) else action_array


def _clip_action(
    action: np.ndarray,
    act_space: Mapping[str, Any] | None,
    selected_key: str | None,
    selected_keys: list[str] | None,
) -> np.ndarray:
    if act_space is None:
        return action
    if selected_key is not None:
        space = act_space.get(selected_key)
        if space is None:
            return action
        return _clip_to_space(action, space)
    lows: list[np.ndarray] = []
    highs: list[np.ndarray] = []
    for key in selected_keys or list(act_space):
        if key not in act_space:
            continue
        space = act_space[key]
        if not hasattr(space, "low") or not hasattr(space, "high"):
            return action
        lows.append(np.asarray(space.low).reshape(-1))
        highs.append(np.asarray(space.high).reshape(-1))
    if not lows:
        return action
    low = np.concatenate(lows, axis=0)
    high = np.concatenate(highs, axis=0)
    return _clip_to_bounds(action, low, high)


def _clip_to_space(action: np.ndarray, space: Any) -> np.ndarray:
    if not hasattr(space, "low") or not hasattr(space, "high"):
        return action
    return _clip_to_bounds(
        action,
        np.asarray(space.low),
        np.asarray(space.high),
    )


def _clip_to_bounds(
    action: np.ndarray, low: np.ndarray, high: np.ndarray
) -> np.ndarray:
    try:
        return np.asarray(np.clip(action, low, high))
    except ValueError as exc:
        raise ValueError(
            f"action shape {np.shape(action)} does not match action bounds "
            f"{np.shape(low)}"
        ) from exc
