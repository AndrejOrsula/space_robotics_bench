"""Policy checkpoint resolution and loading helpers for SRB integrations."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, Callable, Literal, cast

from srb.integrations.algorithms import (
    AlgorithmAdapter,
    resolve_algorithm_adapter,
)
from srb.integrations.algorithms import (
    supports_policy_collection as _supports_algorithm_policy_collection,
)
from srb.integrations.policy_contract import PolicyLoader
from srb.integrations.run_layouts import (
    discover_run_checkpoint,
    require_unsanitized_config,
)
from srb.utils.artifact_identity import slugify_underscore
from srb.utils.model_artifacts import (
    ModelArtifactReference,
    find_saved_policy_config,
    try_model_artifact_reference,
)
from srb.utils.policy_contract import LoadedPolicy, policy_action_to_numpy

if TYPE_CHECKING:
    from srb._typing import AnyEnv


@dataclass(frozen=True)
class CheckpointInference:
    """Algorithm inference result and confidence."""

    algorithm: str | None
    confidence: Literal["manifest", "high", "ambiguous", "unknown"]
    reason: str


def infer_model_algorithm(model: Any) -> CheckpointInference:
    """Infer an algorithm from artifact metadata or an unambiguous legacy name.

    Ambiguous suffixes require explicit ``--algo`` selection.
    """
    model_path = Path(model) if isinstance(model, (str, Path)) else Path(str(model))
    artifact_reference = try_model_artifact_reference(model_path)
    if artifact_reference is not None:
        return CheckpointInference(
            artifact_reference.algo,
            "manifest",
            "algorithm declared by the SRB model artifact manifest",
        )

    model_str = str(model_path).lower()
    filename = model_path.name.lower()
    suffix = model_path.suffix.lower()
    path_tokens = {part.lower() for part in model_path.parts}

    if suffix == ".pt":
        pt_candidates: list[tuple[str, str]] = []
        if "tdmpc2" in model_str or filename.removesuffix(".pt").isdigit():
            pt_candidates.append(("tdmpc2", "TD-MPC2 legacy checkpoint naming"))
        if "rsl_rl" in model_str or filename.startswith("model_"):
            pt_candidates.append(("rsl_rl_ppo", "RSL-RL legacy model_<step>.pt naming"))
        return _resolve_legacy_candidates(model_path, suffix, pt_candidates)

    if suffix == ".pth":
        pth_candidates: list[tuple[str, str]] = []
        # Run layouts name the algorithm directory (``<env>/<algo>/<run>``).
        run_algo = next(
            (
                part.lower()
                for part in reversed(model_path.parts)
                if part.lower().startswith("robomimic_")
            ),
            None,
        )
        if run_algo is not None:
            pth_candidates.append((run_algo, "Robomimic run directory naming"))
        elif (
            "robomimic" in model_str
            or "model_epoch_" in filename
            or filename.startswith(("bc", "iql"))
            or bool({"bc", "iql"} & path_tokens)
        ):
            pth_candidates.append(
                ("robomimic_bc", "Robomimic legacy checkpoint naming")
            )
        if "skrl" in model_str:
            pth_candidates.append(("skrl_ppo", "SKRL legacy checkpoint naming"))
        return _resolve_legacy_candidates(
            model_path,
            suffix,
            pth_candidates,
            fallback_candidates=("robomimic_bc", "skrl_ppo"),
        )

    if suffix == ".zip":
        candidates = []
        if "sbx" in model_str:
            candidates.append(("sbx_ppo", "SBX legacy checkpoint naming"))
        if "sb3" in model_str or filename.startswith("rl_model_"):
            candidates.append(("sb3_ppo", "SB3 legacy checkpoint naming"))
        return _resolve_legacy_candidates(
            model_path,
            suffix,
            candidates,
            fallback_candidates=("sb3_ppo", "sbx_ppo"),
        )

    if suffix == ".ckpt":
        return CheckpointInference(
            "dreamer",
            "high",
            "Dreamer legacy .ckpt checkpoint format",
        )

    return CheckpointInference(
        None,
        "unknown",
        f"cannot infer algorithm from checkpoint {model_path}: unknown "
        f"checkpoint suffix {suffix or '<none>'!r}; pass --algo explicitly",
    )


def infer_algo_from_model(model: Any) -> str:
    """Infer an SRB algorithm or fail closed with an actionable reason."""
    inference = infer_model_algorithm(model)
    if inference.algorithm is None:
        raise ValueError(inference.reason)
    return inference.algorithm


def _resolve_legacy_candidates(
    model_path: Path,
    suffix: str,
    candidates: list[tuple[str, str]],
    *,
    fallback_candidates: tuple[str, ...] = ("rsl_rl_ppo", "tdmpc2"),
) -> CheckpointInference:
    if len(candidates) == 1:
        algorithm, reason = candidates[0]
        return CheckpointInference(algorithm, "high", reason)
    if len(candidates) > 1:
        return _ambiguous_legacy_inference(
            model_path,
            suffix,
            tuple(candidate[0] for candidate in candidates),
        )
    return _ambiguous_legacy_inference(
        model_path,
        suffix,
        fallback_candidates,
    )


def _ambiguous_legacy_inference(
    model_path: Path,
    suffix: str,
    candidates: tuple[str, ...],
) -> CheckpointInference:
    return CheckpointInference(
        None,
        "ambiguous",
        f"cannot infer algorithm from checkpoint {model_path}: suffix "
        f"{suffix!r} is ambiguous between {', '.join(candidates)}; pass "
        "--algo explicitly",
    )


def supports_policy_collection(algo: str) -> bool:
    """Return whether *algo* can be rolled out into a dataset collector."""
    return _supports_algorithm_policy_collection(algo)


def resolve_agent_workflow_algo(
    *,
    agent_subcommand: str,
    algo: str | None,
    model: Path | str | None,
) -> str | None:
    """Infer policy algorithm for workflows that accept checkpoint-only input."""
    if algo is not None or model is None:
        return algo
    if agent_subcommand in ("collect", "eval", "teleop"):
        return infer_algo_from_model(model)
    return algo


def resolve_model_artifact_policy_inputs(
    *,
    algo: str | None,
    model: Path | str | None,
) -> tuple[str | None, Path | str | None]:
    """Resolve model artifact directories to concrete checkpoint paths."""
    artifact_reference = resolve_model_artifact_reference(algo=algo, model=model)
    if artifact_reference is not None:
        return artifact_reference.algo, artifact_reference.checkpoint_path
    return algo, model


def resolve_model_artifact_reference(
    *,
    algo: str | None,
    model: Path | str | None,
) -> "ModelArtifactReference | None":
    """Return a validated SRB model artifact reference if *model* is an artifact."""
    if model is None:
        return None

    artifact_reference = try_model_artifact_reference(model)
    if artifact_reference is None:
        return None

    if algo is not None and slugify_underscore(algo) != artifact_reference.algo:
        raise ValueError(
            f"algo {algo!r} does not match model artifact algo "
            f"{artifact_reference.algo!r}"
        )
    return artifact_reference


def resolve_model_path(
    algo: str,
    model: Path | str | None,
    logdir: Path,
) -> Path:
    """Resolve the checkpoint path for *algo* from an explicit model or run dir."""
    if model:
        _resolved_algo, resolved_model = resolve_model_artifact_policy_inputs(
            algo=algo,
            model=model,
        )
        model_path = Path(cast(Path | str, resolved_model))
        if not model_path.exists():
            raise ValueError(f"Model path does not exist: {model_path}")
        return model_path

    discovered_model = discover_run_checkpoint(
        logdir,
        algo,
        workflow="train",
        continue_training=True,
        required=False,
    )

    if not discovered_model or not Path(discovered_model).exists():
        raise ValueError(
            f"Could not find checkpoint for algorithm '{algo}' in {logdir}. "
            "Provide a model path via --model."
        )

    return Path(discovered_model)


def load_policy(
    algo: str,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    """Load a trained policy for simulation, collection, or validation.

    ``artifact_reference`` lets loaders verify the checkpoint's artifact.
    """
    adapter = resolve_algorithm_adapter(algo)
    loader = _POLICY_LOADERS.get(adapter.framework)
    if loader is None:
        raise ValueError(f"Unsupported algorithm for policy loading: {algo}")
    # Older CLI adapters drop the reference; rediscover it to load the verified
    # snapshot instead of reopening a mutable checkpoint member.
    effective_reference = artifact_reference or try_model_artifact_reference(model_path)
    load_path = (
        effective_reference.load_path if effective_reference is not None else model_path
    )
    return loader(
        adapter,
        load_path,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
        artifact_reference=effective_reference,
    )


def _load_sb3_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    return _load_rl_zoo_policy(
        adapter,
        model_path,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
        artifact_reference=artifact_reference,
    )


def _load_sbx_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    # RL-Zoo's registry defaults to SB3 classes; install SBX's JAX classes only
    # for this load.
    from srb.integrations.sbx.main import sbx_algorithms

    with sbx_algorithms():
        return _load_rl_zoo_policy(
            adapter,
            model_path,
            env,
            agent_cfg=agent_cfg,
            env_cfg=env_cfg,
            logdir=logdir,
            artifact_reference=artifact_reference,
        )


def _load_rl_zoo_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    from srb.integrations.sb3.policy import load_policy

    return load_policy(
        adapter,
        model_path,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
        artifact_reference=artifact_reference,
    )


def _flatten_rl_zoo_observation(
    obs: Any,
    *,
    num_envs: int | None = None,
) -> Any:
    import numpy as np

    if isinstance(obs, dict):
        arrays = [
            value if isinstance(value, np.ndarray) else value.detach().cpu().numpy()
            for value in obs.values()
        ]
        if not arrays:
            raise ValueError("RL-Zoo policy received an empty observation mapping")
        if num_envs is None:
            return np.concatenate([value.reshape(-1) for value in arrays], axis=0)
        flattened = []
        for value in arrays:
            if value.ndim == 0 or value.shape[0] != num_envs:
                raise ValueError(
                    "RL-Zoo batched observation leaf must start with "
                    f"num_envs={num_envs}, got {value.shape}"
                )
            flattened.append(value.reshape(num_envs, -1))
        return np.concatenate(flattened, axis=1)
    return obs if isinstance(obs, np.ndarray) else obs.detach().cpu().numpy()


def _load_skrl_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    import copy

    import torch

    from srb.integrations.skrl.runner import runner_for_config
    from srb.integrations.skrl.wrapper import SkrlEnvWrapper

    train_cfg = copy.deepcopy(agent_cfg)
    if train_cfg is None:
        train_cfg = _recover_training_agent_cfg(model_path, framework="skrl")
    if not train_cfg:
        raise _missing_training_agent_cfg("SKRL", model_path)

    agent_section = train_cfg.get("agent")
    if not isinstance(agent_section, dict):
        raise ValueError("SKRL configuration is missing the 'agent' mapping")
    experiment = dict(agent_section.get("experiment") or {})
    inference_logdir = Path(logdir or model_path.parent)
    experiment["directory"] = str(inference_logdir.parent)
    experiment["experiment_name"] = str(inference_logdir.name)
    agent_section["experiment"] = experiment

    wrapped_env = SkrlEnvWrapper(env)
    runner = runner_for_config(wrapped_env, train_cfg)
    agent = runner.agent
    agent.load(str(model_path))
    if hasattr(agent, "enable_training_mode"):
        agent.enable_training_mode(False)
    else:
        # Older adapters expose this toggle; current SKRL uses the branch above.
        agent.set_running_mode("eval")

    policy_mixin_names = {
        base.__name__ for base in type(getattr(agent, "policy", object)).__mro__
    }
    categorical_policy = bool(
        {"CategoricalMixin", "MultiCategoricalMixin"} & policy_mixin_names
    )
    deterministic_policy = (
        bool(
            {
                "DeterministicMixin",
                "GaussianMixin",
                "MultivariateGaussianMixin",
            }
            & policy_mixin_names
        )
        and not categorical_policy
    )
    deterministic_off_policy = getattr(agent, "demo_algorithm", None) in {
        "skrl_ddpg",
        "skrl_td3",
    }

    trainer_cfg = train_cfg.get("trainer")
    configured_timesteps = (
        int(trainer_cfg.get("timesteps", 1)) if isinstance(trainer_cfg, dict) else 1
    )
    inference_step = 0

    def policy_fn(obs: dict[str, Any]) -> Any:
        nonlocal inference_step
        obs_tensor = wrapped_env.observation_from_value(obs)
        state_tensor = None
        state_space = getattr(wrapped_env, "state_space", None)
        state_from_value = getattr(wrapped_env, "state_from_value", None)
        if callable(state_from_value):
            state_tensor = state_from_value(obs)
        elif callable(getattr(wrapped_env, "_flatten_states", None)):
            # Use the private projection when critic state differs from policy
            # observations.
            state_tensor = wrapped_env._flatten_states(obs)
        elif state_space is not None:
            if not hasattr(state_space, "shape") or tuple(state_space.shape) != tuple(
                wrapped_env.observation_space.shape
            ):
                raise ValueError(
                    "SKRL policy loading requires a state projection for a state "
                    "space distinct from the policy observation space"
                )
            state_tensor = obs_tensor
        random_timesteps = max(
            0,
            int(
                getattr(
                    getattr(agent, "cfg", None),
                    "random_timesteps",
                    getattr(agent, "_random_timesteps", 0),
                )
            ),
        )
        timestep = max(random_timesteps, inference_step)

        with torch.inference_mode():
            if deterministic_off_policy:
                # DDPG/TD3 ``Agent.act`` adds exploration noise in eval mode;
                # call the policy directly for stable checkpoint inference.
                inputs = {
                    "observations": agent._observation_preprocessor(obs_tensor),
                    "states": agent._state_preprocessor(state_tensor),
                }
                with torch.autocast(
                    device_type=agent._device_type,
                    enabled=agent.cfg.mixed_precision,
                ):
                    result = agent.policy.act(inputs, role="policy")
            else:
                result = agent.act(
                    obs_tensor,
                    state_tensor,
                    timestep=timestep,
                    timesteps=max(configured_timesteps, timestep + 1),
                )
        action = result
        if isinstance(result, tuple):
            action = result[0]
            outputs = result[-1]
            if isinstance(outputs, dict):
                action = outputs.get("mean_actions", action)
        if getattr(agent, "_rnn", False):
            initial = getattr(agent, "_rnn_initial_states", None)
            final = getattr(agent, "_rnn_final_states", None)
            if isinstance(initial, dict) and isinstance(final, dict):
                final_policy = final.get("policy")
                if isinstance(final_policy, list) and final_policy:
                    initial["policy"] = final_policy
        inference_step += 1
        # Training steps through SkrlEnvWrapper, which clamps to the action bounds.
        if isinstance(action, torch.Tensor):
            action = torch.clamp(
                action,
                min=wrapped_env._clip_actions_min.to(action.device),
                max=wrapped_env._clip_actions_max.to(action.device),
            )
        return policy_action_to_numpy(action)

    def start_episode_fn(done_mask: Any | None = None) -> None:
        reset_hidden_state = getattr(agent, "reset_hidden_state", None)
        if callable(reset_hidden_state):
            with torch.inference_mode():
                if done_mask is None:
                    reset_hidden_state()
                else:
                    reset_hidden_state(
                        torch.as_tensor(
                            done_mask,
                            dtype=torch.bool,
                            device=getattr(agent, "device", None),
                        )
                    )
            return
        states_by_role = getattr(agent, "_rnn_initial_states", None)
        if not isinstance(states_by_role, dict):
            return
        for states in states_by_role.values():
            if not isinstance(states, list):
                continue
            for state in states:
                if not isinstance(state, torch.Tensor):
                    continue
                if done_mask is None:
                    state.zero_()
                    continue
                mask = torch.as_tensor(
                    done_mask, dtype=torch.bool, device=state.device
                ).reshape(-1)
                if state.ndim < 2 or state.shape[1] != mask.shape[0]:
                    raise ValueError(
                        "SKRL recurrent reset mask does not match policy batch: "
                        f"mask={tuple(mask.shape)}, state={tuple(state.shape)}"
                    )
                state[:, mask] = 0

    return policy_fn, start_episode_fn, deterministic_policy


def _recover_training_agent_cfg(
    model_path: Path,
    *,
    framework: str,
) -> dict[str, Any] | None:
    """Recover the saved training config for checkpoint-only eval.

    Use run or artifact metadata first; RSL-RL also stores it in sibling
    ``.hydra/config.yaml`` files.
    """
    import yaml

    config_path = find_saved_policy_config(model_path)
    if config_path is None:
        sibling_hydra = Path(model_path).parent / ".hydra" / "config.yaml"
        if sibling_hydra.is_file():
            config_path = sibling_hydra
    if config_path is None:
        return None
    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"{framework} config must be a mapping: {config_path}")
    agent_cfg = _project_training_agent_cfg(
        loaded, config_path=config_path, framework=framework
    )
    # An artifact packaged from the sanitized ``metadata.json`` snapshot (or a
    # metadata-shaped file) must not feed ``<redacted>``/``'inf'`` to a runner.
    require_unsanitized_config(agent_cfg, source=str(config_path))
    return agent_cfg


def _project_training_agent_cfg(
    loaded: dict[str, Any],
    *,
    config_path: Path,
    framework: str,
) -> dict[str, Any]:
    """Select agent config from run metadata, Hydra composites, or framework config.

    Only project SKRL's ``agent`` section for Hydra composites; native SKRL
    configs use the same key.
    """
    import copy

    run_cfg = loaded.get("run")
    if isinstance(run_cfg, dict) and isinstance(run_cfg.get("agent_cfg"), dict):
        return copy.deepcopy(run_cfg["agent_cfg"])
    if isinstance(loaded.get("agent_cfg"), dict):
        return copy.deepcopy(loaded["agent_cfg"])
    agent = loaded.get("agent")
    is_hydra_composite = config_path.parent.name == ".hydra"
    if isinstance(agent, dict) and (framework != "skrl" or is_hydra_composite):
        return copy.deepcopy(agent)
    return copy.deepcopy(loaded)


def _missing_training_agent_cfg(framework_label: str, model_path: Path) -> ValueError:
    return ValueError(
        f"{framework_label} policy loading requires the training agent "
        f"configuration; none was supplied or found beside {model_path}. Load "
        "the checkpoint from its training run directory (which keeps "
        "`.hydra/config.yaml`) or supply the agent configuration explicitly."
    )


def _load_dreamer_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    from srb.integrations.dreamer.policy import load_policy as load_dreamer_policy

    return load_dreamer_policy(model_path, env, artifact_reference=artifact_reference)


def _load_tdmpc2_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    from srb.integrations.tdmpc2.policy import load_policy as load_tdmpc2_policy

    return load_tdmpc2_policy(
        model_path,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
    )


def _load_rsl_rl_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    from srb.integrations.rsl_rl.policy import load_policy as load_rsl_rl_policy

    if agent_cfg is None:
        agent_cfg = _recover_training_agent_cfg(model_path, framework="rsl_rl")
        if not agent_cfg:
            raise _missing_training_agent_cfg("RSL-RL", model_path)
    return load_rsl_rl_policy(
        model_path,
        env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
    )


def _load_robomimic_policy(
    adapter: AlgorithmAdapter,
    model_path: Path,
    env: "AnyEnv",
    *,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
    artifact_reference: "ModelArtifactReference | None" = None,
) -> LoadedPolicy:
    """Load a robomimic checkpoint written by ``srb.integrations.robomimic.train``.

    The policy sees observations under the keys it was trained on: the
    collector's projection (``obs["policy"]`` when present) flattened into
    slash-joined leaf paths such as ``state/joint_pos``. Its actions are mapped
    back from the ``--normalize_actions`` range saved in the checkpoint's
    ``env_metadata``; robomimic's ``RolloutPolicy`` already undoes the
    ``action_config`` statistics.
    """
    import robomimic.utils.file_utils as FileUtils

    device = env.unwrapped.device
    policy, ckpt_dict = FileUtils.policy_from_checkpoint(
        ckpt_path=str(model_path),
        device=device,
        verbose=True,
    )
    action_range = _robomimic_action_range(ckpt_dict, model_path)
    shape_meta = ckpt_dict.get("shape_metadata") or {}
    all_shapes = shape_meta.get("all_shapes") or {}
    obs_keys = list(shape_meta.get("all_obs_keys") or all_shapes)
    unshaped = sorted(str(key) for key in obs_keys if key not in all_shapes)
    if unshaped:
        raise ValueError(
            f"robomimic checkpoint {model_path} shape_metadata lists observation "
            f"keys {unshaped} without a shape in all_shapes"
        )
    obs_ranks = {str(key): len(all_shapes[key]) for key in obs_keys}
    if not obs_ranks:
        raise ValueError(
            f"robomimic checkpoint {model_path} names no observation keys in "
            "shape_metadata"
        )

    frame_stack = _robomimic_frame_stack(ckpt_dict)
    history = _RobomimicFrameHistory(frame_stack)

    def policy_fn(obs: dict[str, Any]) -> Any:
        obs_dict, batched = _robomimic_policy_observation(obs, obs_ranks)
        if frame_stack > 1:
            obs_dict = history.push(obs_dict, batched=batched)
        action = policy(obs_dict, batched_ob=batched)
        if action_range is not None:
            norm_min, norm_max = action_range
            action = (action + 1) / 2 * (norm_max - norm_min) + norm_min

        return action

    def start_episode_fn(done_mask: Any = None) -> None:
        # Robomimic resets the whole environment; accept the shared eval argument.
        policy.start_episode()
        history.reset(done_mask)

    return policy_fn, start_episode_fn, _robomimic_policy_is_deterministic(ckpt_dict)


def _robomimic_config(ckpt_dict: Mapping[str, Any]) -> Mapping[str, Any] | None:
    """Return the checkpoint's training config, parsing its JSON form."""
    import json

    config = ckpt_dict.get("config")
    if isinstance(config, str):
        try:
            config = json.loads(config)
        except ValueError:
            return None
    return config if isinstance(config, Mapping) else None


def _robomimic_frame_stack(ckpt_dict: Mapping[str, Any]) -> int:
    """Return the observation history length the policy was trained on."""
    config = _robomimic_config(ckpt_dict) or {}
    train = config.get("train")
    frame_stack = train.get("frame_stack", 1) if isinstance(train, Mapping) else 1
    return max(int(frame_stack or 1), 1)


class _RobomimicFrameHistory:
    """Stack recent observations like robomimic's ``FrameStackWrapper``.

    A new episode repeats its first observation to fill the history, matching
    ``pad_frame_stack`` in training.
    """

    def __init__(self, frame_stack: int) -> None:
        self._frame_stack = frame_stack
        self._frames: dict[str, Any] = {}
        self._fresh: Any = True

    def reset(self, done_mask: Any = None) -> None:
        import numpy as np

        if done_mask is None or self._fresh is True:
            self._fresh = True
            return
        mask = np.asarray(done_mask, dtype=bool).reshape(-1)
        self._fresh = mask if self._fresh is False else self._fresh | mask

    def push(self, obs_dict: dict[str, Any], *, batched: bool) -> dict[str, Any]:
        import numpy as np

        axis = 1 if batched else 0
        for key, value in obs_dict.items():
            current = np.expand_dims(value, axis)
            filled = np.repeat(current, self._frame_stack, axis=axis)
            frames = self._frames.get(key)
            if frames is None or self._fresh is True or frames.shape != filled.shape:
                frames = filled
            else:
                frames = np.concatenate(
                    (np.delete(frames, 0, axis=axis), current), axis
                )
                if batched and self._fresh is not False:
                    frames[self._fresh] = filled[self._fresh]
            self._frames[key] = frames
            obs_dict[key] = frames
        self._fresh = False
        return obs_dict


def _robomimic_policy_is_deterministic(ckpt_dict: Mapping[str, Any]) -> bool:
    """Return whether plain BC has explicitly disabled GMM, Gaussian, and VAE heads."""
    if ckpt_dict.get("algo_name") != "bc":
        return False
    config = _robomimic_config(ckpt_dict)
    if config is None:
        return False
    algo = config.get("algo")
    if not isinstance(algo, Mapping):
        return False
    for head in ("gmm", "gaussian", "vae"):
        head_cfg = algo.get(head)
        if not isinstance(head_cfg, Mapping) or head_cfg.get("enabled") is not False:
            return False
    return True


_ROBOMIMIC_ACTION_RANGE_KEYS = ("norm_factor_min", "norm_factor_max")
# Marker written when dataset actions were normalized.
_ROBOMIMIC_NORMALIZATION_SIDECAR = "normalization_params.txt"


def _robomimic_action_range(
    ckpt_dict: dict[str, Any], model_path: Path
) -> tuple[float, float] | None:
    """Return the saved action range; fail if a legacy run normalized actions."""
    import math

    env_meta = ckpt_dict.get("env_metadata") or {}
    present = [key for key in _ROBOMIMIC_ACTION_RANGE_KEYS if key in env_meta]
    if not present:
        sidecar = model_path.parent.parent / "logs" / _ROBOMIMIC_NORMALIZATION_SIDECAR
        if sidecar.is_file():
            raise ValueError(
                f"robomimic checkpoint {model_path} saves no action range "
                "(env_metadata norm_factor_min/norm_factor_max), but its run "
                f"normalized actions ({sidecar}); it predates checkpoints that "
                "carry the range, so its actions cannot be mapped back to the "
                "environment. Re-train with the current SRB version."
            )
        return None
    if len(present) != len(_ROBOMIMIC_ACTION_RANGE_KEYS):
        missing = sorted(set(_ROBOMIMIC_ACTION_RANGE_KEYS) - set(present))
        raise ValueError(
            f"robomimic checkpoint {model_path} env_metadata has {present} but "
            f"lacks {missing}"
        )
    values: list[float] = []
    for key in _ROBOMIMIC_ACTION_RANGE_KEYS:
        value = env_meta[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(
                f"robomimic checkpoint {model_path} env_metadata {key} must be "
                f"a number, got {value!r}"
            )
        values.append(float(value))
    norm_min, norm_max = values
    if not (math.isfinite(norm_min) and math.isfinite(norm_max)) or not (
        norm_max > norm_min
    ):
        raise ValueError(
            f"robomimic checkpoint {model_path} action range must be finite with "
            f"norm_factor_max > norm_factor_min, got [{norm_min}, {norm_max}]"
        )
    return norm_min, norm_max


def _robomimic_policy_observation(
    obs: Any, obs_ranks: dict[str, int]
) -> tuple[dict[str, Any], bool]:
    """Project observations to trained keys; return the mapping and batch flag.

    Try ``obs["policy"]`` before the full mapping. Leaf rank determines whether
    values are batched; mappings use robomimic's slash-joined observation keys.
    """
    from collections.abc import Mapping

    import numpy as np

    selections: list[Any] = []
    if isinstance(obs, Mapping) and "policy" in obs:
        selections.append(obs["policy"])
    selections.append(obs)
    available: list[str] = []
    for selection in selections:
        leaves = (
            _slash_joined_leaves(selection)
            if isinstance(selection, Mapping)
            else {"obs": selection}
        )
        if all(key in leaves for key in obs_ranks):
            break
        available = sorted(leaves)
    else:
        missing = sorted(key for key in obs_ranks if key not in available)
        raise ValueError(
            f"robomimic policy observation lacks trained keys {missing}; "
            f"observation provides {available}"
        )

    obs_dict: dict[str, Any] = {}
    batched_keys: dict[str, bool] = {}
    for key, rank in obs_ranks.items():
        value = leaves[key]
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        value = np.asarray(value)
        if value.ndim not in (rank, rank + 1):
            raise ValueError(
                f"robomimic policy observation {key!r} has shape {value.shape}; "
                f"the policy was trained on rank-{rank} samples"
            )
        obs_dict[key] = value
        batched_keys[key] = value.ndim == rank + 1
    if len(set(batched_keys.values())) != 1:
        raise ValueError(
            "robomimic policy observation mixes batched and unbatched leaves: "
            f"{batched_keys}"
        )
    return obs_dict, next(iter(batched_keys.values()))


def _slash_joined_leaves(tree: Any, prefix: str = "") -> dict[str, Any]:
    from collections.abc import Mapping

    leaves: dict[str, Any] = {}
    for key, value in tree.items():
        path = f"{prefix}{key}"
        if isinstance(value, Mapping):
            leaves.update(_slash_joined_leaves(value, f"{path}/"))
        else:
            leaves[path] = value
    return leaves


def _require_local_algo(adapter: AlgorithmAdapter) -> str:
    if adapter.local_algo is None:
        raise ValueError(f"{adapter.framework} policy loading requires local algo")
    return adapter.local_algo


_POLICY_LOADERS: dict[str, PolicyLoader] = {
    "dreamer": _load_dreamer_policy,
    "rsl_rl": _load_rsl_rl_policy,
    "sb3": _load_sb3_policy,
    "sbx": _load_sbx_policy,
    "skrl": _load_skrl_policy,
    "tdmpc2": _load_tdmpc2_policy,
    "robomimic": _load_robomimic_policy,
}


def load_policy_from_checkpoint(
    algo: str,
    checkpoint: Path | str,
    *,
    device: str = "cpu",
    env: "AnyEnv | None" = None,
    agent_cfg: dict[str, Any] | None = None,
    env_cfg: Any = None,
    logdir: Path | None = None,
) -> Callable[[Any], Any]:
    """Load a checkpoint policy for non-interactive validation workflows."""
    artifact_reference = resolve_model_artifact_reference(algo=algo, model=checkpoint)
    if artifact_reference is not None:
        resolved_algo: str | None = artifact_reference.algo
        resolved_checkpoint: Path | str | None = artifact_reference.checkpoint_path
    else:
        resolved_algo, resolved_checkpoint = algo, checkpoint
    if resolved_algo is None or resolved_checkpoint is None:
        raise ValueError("policy checkpoint loading requires an algorithm and model")

    policy_env = (
        env
        if env is not None
        else cast("AnyEnv", SimpleNamespace(unwrapped=SimpleNamespace(device=device)))
    )
    policy_fn, start_episode_fn, _deterministic = load_policy(
        resolved_algo,
        Path(resolved_checkpoint),
        policy_env,
        agent_cfg=agent_cfg,
        env_cfg=env_cfg,
        logdir=logdir,
        artifact_reference=artifact_reference,
    )
    setattr(policy_fn, "start_episode", start_episode_fn)
    return policy_fn
