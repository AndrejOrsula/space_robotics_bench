"""Load SB3/SBX policies with SRB layouts, saved normalization, and recurrent state."""

from __future__ import annotations

import pickle
from collections.abc import Mapping
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from srb.utils.policy_contract import LoadedPolicy

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.integrations.algorithms import AlgorithmAdapter
    from srb.utils.model_artifacts import ModelArtifactReference


_VECNORMALIZE_NAME = "vecnormalize.pkl"
_NORMALIZATION_METADATA_KEY = "rl_zoo_normalization"


def load_policy(
    adapter: "AlgorithmAdapter",
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    """Load a policy without resetting or stepping the live environment."""

    import yaml

    try:
        saved_cfg = _raw_checkpoint_config(model_path, None)
    except (OSError, yaml.YAMLError):
        saved_cfg = {}  # The saved config is optional for loading.
    if any(
        isinstance(cfg, Mapping) and cfg.get("frame_stack") is not None
        for cfg in (agent_cfg, saved_cfg, saved_cfg.get("hyperparams"))
    ):
        raise ValueError(
            "SB3/SBX policy loading does not stack frames; evaluate frame_stack "
            "runs from their run directory instead of a model artifact"
        )

    from srb.utils.torch_runtime import warm_torch_native_stack

    warm_torch_native_stack()

    from rl_zoo3 import ALGOS
    from stable_baselines3.common.vec_env import VecNormalize

    from srb.integrations.sb3.wrapper import Sb3EnvWrapper

    adapter_framework = str(getattr(adapter, "framework", "sb3"))
    observation_from_value: Any
    algo_name = getattr(adapter, "local_algo", None)
    if not isinstance(algo_name, str) or not algo_name:
        raise ValueError("SB3/SBX policy loading requires a local algorithm name")
    algorithm = ALGOS.get(algo_name)
    if algorithm is None:
        raise ValueError(f"RL-Zoo has no registered algorithm {algo_name!r}")

    if (
        adapter_framework == "sbx" or not _supports_dict_observations(algo_name)
    ) and _is_dict_observation_space(env):
        # Mirror training's merge for algorithms without MultiInputPolicy.
        from srb.wrappers.merge_dict_obs import MergeDictObsWrapper

        merged_env = cast(Any, MergeDictObsWrapper(env))
        wrapped_env = Sb3EnvWrapper(merged_env)

        def observation_from_value(observations: Any) -> Any:
            observations = _maybe_batch_sbx_observation(observations, merged_env)
            return wrapped_env.observation_from_value(
                merged_env.observation(observations)
            )

    else:
        if callable(getattr(env, "observation_from_value", None)):
            wrapped_env = env
            observation_from_value = wrapped_env.observation_from_value
        elif _has_wrapper_spaces(env):
            wrapped_env = Sb3EnvWrapper(env)
            observation_from_value = wrapped_env.observation_from_value
        else:
            # Keep bare Box checkpoints compatible; normalization restore requires a VecEnv.
            wrapped_env = env
            legacy_num_envs = _declared_num_envs(env)

            def observation_from_value(observations: Any) -> Any:
                return _legacy_observation_from_value(
                    observations,
                    num_envs=legacy_num_envs,
                )

    normalizer = _load_observation_normalizer(
        model_path,
        wrapped_env,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
        artifact_reference=artifact_reference,
        vecnormalize_loader=VecNormalize,
    )

    device = getattr(getattr(env, "unwrapped", env), "device", "cpu")
    rl_policy = algorithm.load(str(model_path), device=device)
    num_envs = _num_envs(wrapped_env, env)
    recurrent = algo_name.endswith(("_lstm", "_rnn")) or _is_recurrent_policy(rl_policy)
    recurrent_state: Any = None
    episode_start = np.ones(num_envs, dtype=bool)

    def policy_fn(observations: Any) -> Any:
        nonlocal recurrent_state, episode_start
        converted = observation_from_value(observations)
        if normalizer is not None:
            converted = normalizer.normalize_obs(converted)
        predict_kwargs: dict[str, Any] = {"deterministic": True}
        if recurrent:
            predict_kwargs.update(
                state=recurrent_state,
                episode_start=episode_start.copy(),
            )
        action, next_state = rl_policy.predict(converted, **predict_kwargs)
        recurrent_state = next_state if recurrent else None
        # SB3 consumes episode_start for this prediction.  A later selective
        # reset changes only the completed environment slots.
        episode_start = np.zeros(num_envs, dtype=bool)
        return action

    def start_episode_fn(done_mask: Any | None = None) -> None:
        nonlocal episode_start
        if done_mask is None:
            episode_start = np.ones(num_envs, dtype=bool)
            return
        episode_start = _coerce_done_mask(done_mask, num_envs)

    return policy_fn, start_episode_fn, True


def _supports_dict_observations(algo: str) -> bool:
    from rl_zoo3 import ALGOS

    algorithm = ALGOS.get(algo)
    aliases = getattr(algorithm, "policy_aliases", {})
    return any(name.startswith("MultiInput") for name in aliases)


def _is_dict_observation_space(env: Any) -> bool:
    import gymnasium

    effective_spaces = [
        space
        for name in ("single_observation_space", "observation_space")
        if (space := getattr(env, name, None)) is not None
    ]
    if effective_spaces:
        return all(
            isinstance(space, gymnasium.spaces.Dict) for space in effective_spaces
        )

    # Some transparent wrappers expose no effective space of their own.  Only
    # then may the unwrapped environment supply the observation structure.
    unwrapped = getattr(env, "unwrapped", env)
    if unwrapped is env:
        return False
    fallback_spaces = [
        space
        for name in ("single_observation_space", "observation_space")
        if (space := getattr(unwrapped, name, None)) is not None
    ]
    return bool(fallback_spaces) and all(
        isinstance(space, gymnasium.spaces.Dict) for space in fallback_spaces
    )


def _has_wrapper_spaces(env: Any) -> bool:
    unwrapped = getattr(env, "unwrapped", env)
    observation_space = any(
        getattr(owner, name, None) is not None
        for owner in (env, unwrapped)
        for name in ("single_observation_space", "observation_space")
    )
    action_space = any(
        getattr(owner, name, None) is not None
        for owner in (env, unwrapped)
        for name in ("single_action_space", "action_space")
    )
    return observation_space and action_space


def _legacy_observation_from_value(value: Any, *, num_envs: int | None) -> Any:
    if isinstance(value, Mapping):
        arrays = [
            _legacy_observation_from_value(child, num_envs=num_envs)
            for child in value.values()
        ]
        if not arrays:
            raise ValueError("RL-Zoo policy received an empty observation mapping")
        converted = [np.asarray(array) for array in arrays]
        if num_envs is None:
            return np.concatenate(
                [array.reshape(-1) for array in converted],
                axis=0,
            )
        flattened = []
        for array in converted:
            if array.ndim == 0 or array.shape[0] != num_envs:
                raise ValueError(
                    "RL-Zoo batched observation leaf must start with "
                    f"num_envs={num_envs}, got {array.shape}"
                )
            flattened.append(array.reshape(num_envs, -1))
        return np.concatenate(flattened, axis=1)
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return value if isinstance(value, np.ndarray) else np.asarray(value)


def _declared_num_envs(env: Any) -> int | None:
    raw = getattr(getattr(env, "unwrapped", env), "num_envs", None)
    if isinstance(raw, int) and not isinstance(raw, bool) and raw > 0:
        return raw
    return None


def _is_recurrent_policy(policy: Any) -> bool:
    policy_model = getattr(policy, "policy", None)
    return bool(
        policy_model is not None
        and any(
            hasattr(policy_model, attribute)
            for attribute in ("lstm_actor", "lstm_hidden_state_shape")
        )
    )


def _maybe_batch_sbx_observation(observations: Any, merged_env: Any) -> Any:
    """Match MergeDictObsWrapper's vector layout for a single raw env value."""
    if getattr(merged_env, "num_envs", None) != 1:
        return observations
    layout = getattr(merged_env, "_observation_layout", None)
    terms = getattr(layout, "terms", ())
    if not terms:
        return observations
    term = terms[0]
    if term.category:
        group = observations.get(term.category)
        value = group.get(term.key) if isinstance(group, Mapping) else None
    else:
        value = (
            observations.get(term.key) if isinstance(observations, Mapping) else None
        )
    shape = tuple(getattr(value, "shape", ()))
    if shape == tuple(term.shape):
        return _add_observation_batch_axis(observations)
    return observations


def _add_observation_batch_axis(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _add_observation_batch_axis(child) for key, child in value.items()}
    if isinstance(value, np.ndarray):
        return value.reshape((1, *value.shape))
    if hasattr(value, "unsqueeze"):
        return value.unsqueeze(0)
    return value


def _num_envs(wrapped_env: Any, env: Any) -> int:
    raw = getattr(wrapped_env, "num_envs", None)
    if raw is None:
        raw = getattr(getattr(env, "unwrapped", env), "num_envs", None)
    if raw is None:
        return 1
    if isinstance(raw, bool) or not isinstance(raw, int) or raw <= 0:
        raise ValueError(
            f"SB3/SBX policy loading requires positive num_envs, got {raw!r}"
        )
    return cast(int, raw)


def _coerce_done_mask(done_mask: Any, num_envs: int) -> np.ndarray:
    if hasattr(done_mask, "detach"):
        done_mask = done_mask.detach().cpu().numpy()
    mask = np.asarray(done_mask, dtype=bool)
    if mask.ndim == 0:
        if num_envs != 1:
            raise ValueError(
                "recurrent episode reset mask must contain one value per environment"
            )
        mask = mask.reshape(1)
    else:
        mask = mask.reshape(-1)
    if mask.shape != (num_envs,):
        raise ValueError(
            "recurrent episode reset mask must contain one value per environment: "
            f"expected {(num_envs,)}, got {tuple(mask.shape)}"
        )
    return mask.copy()


def _load_observation_normalizer(
    model_path: Path,
    wrapped_env: Any,
    env: Any,
    *,
    agent_cfg: Mapping[str, Any] | None,
    env_cfg: Any,
    logdir: Path | None,
    artifact_reference: Any | None,
    vecnormalize_loader: Any,
) -> Any | None:
    if artifact_reference is not None:
        stats_path = _artifact_normalization_path(artifact_reference)
        if stats_path is None:
            return None
        return _load_vecnormalize(stats_path, wrapped_env, vecnormalize_loader)

    config = _raw_checkpoint_config(model_path, agent_cfg)
    stats_path = _find_raw_vecnormalize(
        model_path,
        env,
        env_cfg=env_cfg,
        config=config,
        logdir=logdir,
    )
    if stats_path is None:
        if _normalization_requested(config):
            raise ValueError(
                "SB3/SBX checkpoint configuration requests observation "
                "normalization, but no saved vecnormalize.pkl was found; "
                "refusing to create fresh normalization statistics"
            )
        return None
    return _load_vecnormalize(stats_path, wrapped_env, vecnormalize_loader)


def _artifact_normalization_path(reference: Any) -> Path | None:
    manifest = getattr(reference, "manifest", None)
    metadata = getattr(manifest, "metadata", None)
    if not isinstance(metadata, Mapping):
        raise ValueError(_artifact_normalization_diagnostic())
    spec = metadata.get(_NORMALIZATION_METADATA_KEY)
    if not isinstance(spec, Mapping):
        raise ValueError(_artifact_normalization_diagnostic())
    kind = spec.get("kind")
    if kind == "none":
        if dict(spec) != {"kind": "none"}:
            raise ValueError(
                "model artifact rl_zoo_normalization is not canonical; "
                "repackage it with {'kind':'none'} or the verified "
                "normalization/vecnormalize.pkl member"
            )
        return None
    if kind != "vecnormalize":
        raise ValueError(
            f"model artifact {_NORMALIZATION_METADATA_KEY!r} has unsupported "
            f"kind {kind!r}; repackage the artifact with kind 'none' or "
            "'vecnormalize'"
        )
    relative_path = spec.get("path")
    if not isinstance(relative_path, str) or not relative_path.strip():
        raise ValueError(
            "model artifact VecNormalize metadata requires a relative 'path'; "
            "repackage the artifact with metadata.rl_zoo_normalization"
        )
    if dict(spec) != {
        "kind": "vecnormalize",
        "path": "normalization/vecnormalize.pkl",
    }:
        raise ValueError(
            "model artifact rl_zoo_normalization is not canonical; repackage "
            "it with the verified normalization/vecnormalize.pkl member"
        )
    relative = Path(relative_path)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(
            "model artifact VecNormalize path must stay inside the artifact; "
            "repackage the artifact with a relative normalization path"
        )
    snapshot_member = getattr(reference, "snapshot_member", None)
    if not callable(snapshot_member):
        raise ValueError(
            "model artifact cannot provide a verified VecNormalize snapshot; "
            "repackage the artifact with normalization/vecnormalize.pkl"
        )
    try:
        snapshot = snapshot_member(relative.as_posix())
    except (OSError, RuntimeError, ValueError) as exc:
        raise ValueError(
            "model artifact VecNormalize statistics are unavailable from the "
            "verified snapshot; repackage the artifact"
        ) from exc
    if snapshot is None:
        raise ValueError(
            f"model artifact VecNormalize statistics are missing: {relative}; "
            "repackage the artifact"
        )
    if isinstance(snapshot, (str, Path)):
        path = Path(snapshot)
    else:
        path_value = getattr(snapshot, "path", None)
        if not isinstance(path_value, (str, Path)):
            raise ValueError(
                "model artifact VecNormalize snapshot did not return a path; "
                "repackage the artifact"
            )
        path = Path(path_value)
    if not path.is_file():
        raise ValueError(
            f"model artifact VecNormalize statistics are missing: {relative}; "
            "repackage the artifact"
        )
    return path


def _artifact_normalization_diagnostic() -> str:
    return (
        "model artifact does not declare rl_zoo_normalization; normalization "
        "context is unknown, so inference is refused. Repackage the artifact "
        "with metadata.rl_zoo_normalization={'kind':'none'} or with the "
        "verified normalization/vecnormalize.pkl member"
    )


def _load_vecnormalize(path: Path, wrapped_env: Any, loader: Any) -> Any:
    try:
        normalizer = loader.load(str(path), wrapped_env)
    except (
        AttributeError,
        EOFError,
        ImportError,
        IndexError,
        KeyError,
        OSError,
        pickle.UnpicklingError,
        RuntimeError,
        TypeError,
        ValueError,
    ) as exc:
        raise ValueError(
            f"could not load saved SB3 VecNormalize statistics from {path}"
        ) from exc
    normalizer.training = False
    normalizer.norm_reward = False
    return normalizer


def _raw_checkpoint_config(
    model_path: Path, agent_cfg: Mapping[str, Any] | None
) -> Mapping[str, Any]:
    if agent_cfg is not None:
        return agent_cfg
    try:
        from srb.utils.model_artifacts import find_saved_policy_config

        config_path = find_saved_policy_config(model_path)
    except FileNotFoundError:
        config_path = None
    if config_path is None:
        return {}
    import yaml

    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(loaded, Mapping):
        raise ValueError(f"SB3 config must be a mapping: {config_path}")
    return loaded


def _normalization_requested(config: Mapping[str, Any]) -> bool:
    value: Any = config.get("normalize")
    if value is None:
        hyperparams = config.get("hyperparams")
        if isinstance(hyperparams, Mapping):
            value = hyperparams.get("normalize")
    if value is None:
        return False
    # RL-Zoo's exp manager treats every string (including an empty one) and
    # mapping (including an empty one) as VecNormalize kwargs.  Those values
    # therefore require saved statistics; never silently create fresh state.
    if isinstance(value, (str, Mapping)):
        return True
    return bool(value)


def _find_raw_vecnormalize(
    model_path: Path,
    env: Any,
    *,
    env_cfg: Any,
    config: Mapping[str, Any],
    logdir: Path | None,
) -> Path | None:
    names: list[str] = []
    for value in (
        config.get("env"),
        getattr(getattr(env, "spec", None), "id", None),
        getattr(getattr(env, "unwrapped", env), "spec", None),
        getattr(env_cfg, "task_name", None),
        env_cfg.get("task_name") if isinstance(env_cfg, Mapping) else None,
    ):
        value_id = getattr(value, "id", None)
        if isinstance(value_id, str):
            value = value_id
        if isinstance(value, str) and value and value not in names:
            names.append(value)

    # Eval/resume's lookup first, so a periodic checkpoint gets its own
    # statistics and every caller agrees on the rest.
    from srb.integrations.run_layouts import (
        SB3_PERIODIC_CHECKPOINT,
        find_sb3_vecnormalize,
        sb3_checkpoint_paths,
        sb3_checkpoint_stem,
    )

    lookup = find_sb3_vecnormalize(model_path, env_names=names)
    if lookup.missing_own is not None:
        raise ValueError(
            f"{model_path} belongs to a run that saves VecNormalize statistics "
            f"per checkpoint, but its own {lookup.missing_own} is missing; the "
            "run's other statistics were written at a different training step "
            "and are not substituted. Load the run's final model or a periodic "
            "checkpoint whose statistics exist"
        )
    stats_path = lookup.path
    if stats_path is None:
        # Earlier raw-loader locations: the model's grandparent and ``logdir``.
        roots: list[Path] = []
        for root in (
            model_path.parent,
            model_path.parent.parent,
            Path(logdir) if logdir is not None else None,
        ):
            if root is not None and root not in roots:
                roots.append(root)
        candidates: list[Path] = []
        for root in roots:
            candidates.append(root / _VECNORMALIZE_NAME)
            candidates.extend(root / name / _VECNORMALIZE_NAME for name in names)
        stats_path = next(
            (candidate for candidate in candidates if candidate.is_file()), None
        )
    # Run-level statistics for a periodic checkpoint predate it.
    stale = lookup.stale or (
        lookup.path is None
        and stats_path is not None
        and any(
            SB3_PERIODIC_CHECKPOINT.fullmatch(sb3_checkpoint_stem(model)) is not None
            for model in sb3_checkpoint_paths(model_path)
        )
    )
    if stale:
        from srb.utils import logging

        logging.warning(
            f"{model_path} has no VecNormalize statistics of its own (saved "
            f"before per-checkpoint statistics); using {stats_path}, which was "
            "written at a different training step"
        )
    return stats_path


__all__ = ["load_policy"]
