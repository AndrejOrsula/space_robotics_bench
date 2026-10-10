import contextlib
import gc
import glob
import inspect
import json
import math
import os
import pickle
import signal
import threading
import zipfile
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

import gymnasium
import numpy as np
import torch
from huggingface_sb3 import EnvironmentName
from rl_zoo3 import ALGOS
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, tqdm
from stable_baselines3.common.off_policy_algorithm import OffPolicyAlgorithm
from stable_baselines3.common.vec_env import VecEnv, VecNormalize

from srb.integrations.eval_quota import (
    EpisodeQuota,
    effective_eval_seed,
    eval_episode_budget,
    eval_max_steps,
    write_eval_result,
)
from srb.integrations.run_layouts import (
    SB3_PERIODIC_CHECKPOINT as _PERIODIC_CHECKPOINT,
)
from srb.integrations.run_layouts import (
    SB3_VECNORMALIZE_NAME as _VECNORMALIZE_NAME,
)
from srb.integrations.run_layouts import Sb3StatsLookup as _StatsLookup
from srb.integrations.run_layouts import find_sb3_vecnormalize
from srb.integrations.run_layouts import sb3_checkpoint_paths as _checkpoint_paths
from srb.integrations.run_layouts import sb3_checkpoint_stem as _checkpoint_stem
from srb.integrations.run_logging import run_logging
from srb.integrations.sb3.exp_manager import (
    ExperimentManager as _SrbExperimentManager,
)
from srb.integrations.sb3.exp_manager import experiment_args
from srb.integrations.sb3.policy import (
    _is_dict_observation_space,
    _normalization_requested,
    _supports_dict_observations,
)
from srb.integrations.sb3.wrapper import Sb3EnvWrapper
from srb.utils import logging
from srb.utils.checkpoint import find_sb3_checkpoint
from srb.utils.path import new_eval_logdir
from srb.wrappers import MergeDictObsWrapper

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv, AnyEnvCfg

OFF_POLICY_ALGOS: Sequence[str] = (
    "qrdqn",
    "dqn",
    "ddpg",
    "sac",
    "her",
    "td3",
    "tqc",
    "crossq",
)
_REPLAY_BUFFER_NAME = "replay_buffer.pkl"
# Set on SIGINT; a KeyboardInterrupt raised inside a Kit/USD callback can be swallowed.
_INTERRUPTED = threading.Event()
_NORMALIZATION_WAY_OUT = (
    "Resume or evaluate the run's final model (<run>/ckpt/<env>.zip, whose "
    "statistics are <run>/vecnormalize.pkl) or a periodic checkpoint whose "
    "rl_model_vecnormalize_<N>_steps.pkl exists, or set agent.normalize=false "
    "to run this policy without normalization."
)


def _checkpoint_save_frequency(env_step_frequency: int, num_envs: int) -> int:
    """Convert the configured environment-step cadence to VecEnv calls."""
    if num_envs < 1:
        raise ValueError(f"num_envs must be positive, got {num_envs}")
    if env_step_frequency <= 0:
        return env_step_frequency
    return max(env_step_frequency // num_envs, 1)


class ExperimentManager(_SrbExperimentManager):
    """Adapt rl_zoo3 checkpoint, normalization, and resume paths for SRB runs.

    ``resume_requested`` marks CLI ``--continue``; a finished run is a no-op,
    while fine-tuning an explicit model at its timestep limit is an error.
    """

    def __init__(self, *args: Any, resume_requested: bool = False, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self._resume_requested = bool(resume_requested)
        self._trained = True

    def create_callbacks(self) -> None:
        super().create_callbacks()
        final_model = Path(self.save_path) / str(self.env_name)
        if not final_model.suffix:
            final_model = final_model.with_name(f"{final_model.name}.zip")
        self.callbacks = [
            _CompleteCheckpointCallback.replacing(
                callback,
                save_replay_buffer=bool(self.save_replay_buffer),
                final_partials=(
                    final_model.with_name(f".{final_model.name}.partial"),
                    Path(self.save_path) / ".replay_buffer.pkl.partial",
                    Path(self.params_path) / ".vecnormalize.pkl.partial",
                ),
            )
            if type(callback) is CheckpointCallback
            else callback
            for callback in self.callbacks
        ] + [_StopOnInterruptCallback()]

    def save_trained_model(self, model: BaseAlgorithm) -> None:
        # Publish stats, model, then buffer; skip saves when nothing was trained.
        if not self._trained:
            logging.info(
                f"Nothing was trained; keeping the saved models in {self.save_path}"
            )
            return
        if self.normalize:
            vec_normalize = model.get_vec_normalize_env()
            assert vec_normalize is not None
            _publish(
                Path(self.params_path) / _VECNORMALIZE_NAME,
                lambda path: vec_normalize.save(str(path)),
            )
        model_path = Path(self.save_path) / str(self.env_name)
        if not model_path.suffix:
            model_path = model_path.with_name(f"{model_path.name}.zip")
        logging.info(f"Saving the final model to {model_path}")
        _publish(model_path, model.save)
        if self.save_replay_buffer and isinstance(model, OffPolicyAlgorithm):
            _publish(
                Path(self.save_path) / _REPLAY_BUFFER_NAME, model.save_replay_buffer
            )

    def _maybe_normalize(self, env: VecEnv, eval_env: bool) -> VecEnv:
        # Evaluation bypasses setup_experiment, so read its normalization config.
        requested = bool(self.normalize) or _normalization_requested(
            self.custom_hyperparams or {}
        )
        # Evaluation loads trained_agent even though rl_zoo3 only does so on resume.
        loads_checkpoint = bool(self.trained_agent) and (
            self.continue_training or eval_env
        )
        if not loads_checkpoint:
            return self._fresh_vecnormalize(env, eval_env) if requested else env

        lookup = _find_vecnormalize(self.trained_agent, env_name=str(self.env_name))
        if not requested:
            if lookup.path is not None:
                logging.warning(
                    "Normalization is disabled in the agent config; ignoring "
                    f"the saved VecNormalize statistics {lookup.path}"
                )
            return env
        if lookup.path is None:
            if lookup.missing_own is not None:
                raise FileNotFoundError(
                    f"{self.trained_agent} belongs to a run that saves VecNormalize "
                    f"statistics per checkpoint, but its own {lookup.missing_own} "
                    "is missing. The run's other statistics were written at a "
                    "different training step and are not substituted. "
                    + _NORMALIZATION_WAY_OUT
                )
            searched = ", ".join(str(path) for path in lookup.searched)
            raise FileNotFoundError(
                f"{self.trained_agent} is configured with VecNormalize, but no "
                f"saved statistics were found (searched: {searched}); refusing "
                "to create fresh statistics for a trained policy. "
                + _NORMALIZATION_WAY_OUT
            )
        if lookup.stale:
            logging.warning(
                f"{self.trained_agent} has no statistics of its own (saved before "
                f"per-checkpoint statistics); using {lookup.path}, which was "
                "written at a different training step"
            )
        logging.info(f"Loading VecNormalize statistics from {lookup.path}")
        normalized = VecNormalize.load(str(lookup.path), env)
        if eval_env:
            normalized.training = False
            normalized.norm_reward = False
        return normalized

    def _fresh_vecnormalize(self, env: VecEnv, eval_env: bool) -> VecNormalize:
        # Avoid rl_zoo3's relative trained_agent lookup when creating fresh stats.
        kwargs = dict(self.normalize_kwargs)
        if eval_env:
            kwargs["norm_reward"] = False
            kwargs["training"] = False
        return VecNormalize(env, **kwargs)

    def _load_pretrained_agent(
        self, hyperparams: dict[str, Any], env: VecEnv
    ) -> BaseAlgorithm:
        # Restore this checkpoint's buffer; rl_zoo3 otherwise loads the run's final buffer.
        logging.info(f"Loading pretrained agent from {self.trained_agent}")
        del hyperparams["policy"]
        hyperparams.pop("policy_kwargs", None)
        model = ALGOS[self.algo].load(
            self.trained_agent,
            env=env,
            seed=self.seed,
            tensorboard_log=self.tensorboard_log,
            verbose=self.verbose,
            device=self.device,
            **hyperparams,
        )
        if isinstance(model, OffPolicyAlgorithm):
            self._restore_replay_buffer(model, hyperparams)
        return model

    def _restore_replay_buffer(
        self, model: OffPolicyAlgorithm, hyperparams: dict[str, Any]
    ) -> None:
        if self.continue_training and (
            int(self.n_timesteps) - int(model.num_timesteps) <= 0
        ):
            return
        buffer_path = _find_replay_buffer(
            self.trained_agent, env_name=str(self.env_name)
        )
        if buffer_path is not None and self._load_replay_buffer(model, buffer_path):
            buffer = model.replay_buffer
            assert buffer is not None
            # Resuming keeps the step counter, so SB3 does not close this trajectory.
            if getattr(model, "optimize_memory_usage", False):
                buffer.dones[(buffer.pos - 1) % buffer.buffer_size] = True
            return
        # A resumed counter skips warm-up; re-arm from config to avoid training on an empty buffer.
        warmup = hyperparams.get("learning_starts")
        if warmup is None:
            param = inspect.signature(type(model).__init__).parameters.get(
                "learning_starts"
            )
            warmup = (
                param.default
                if param is not None and isinstance(param.default, int)
                else model.learning_starts
            )
        model.learning_starts = int(model.num_timesteps) + int(warmup)
        logging.warning(
            f"No replay buffer saved with {self.trained_agent}; resuming with an "
            f"empty buffer and re-arming warm-up until step {model.learning_starts} "
            "(enable save_replay_buffer to resume with the buffer)"
        )

    def _load_replay_buffer(self, model: OffPolicyAlgorithm, path: Path) -> bool:
        """Load ``path`` into ``model``; ``False`` if it holds no transitions."""
        logging.info(f"Loading replay buffer from {path}")
        try:
            model.load_replay_buffer(
                path, truncate_last_traj=self.truncate_last_trajectory
            )
        except (EOFError, pickle.UnpicklingError) as exc:
            # The model may be complete even if interruption left the buffer partial.
            logging.warning(f"Replay buffer {path} could not be read ({exc})")
            return False
        buffer = model.replay_buffer
        if buffer is not None and buffer.n_envs != model.n_envs:
            raise ValueError(
                "Replay buffer environment count does not match model: "
                f"buffer n_envs={buffer.n_envs}, model n_envs={model.n_envs}"
            )
        if buffer is None or (buffer.pos == 0 and not buffer.full):
            logging.warning(f"Replay buffer {path} holds no transitions")
            return False
        return True

    def learn(self, model: BaseAlgorithm) -> None:
        self._trained = True
        if not self.continue_training:
            super().learn(model)
            return
        # Treat n_timesteps as the total budget and preserve the resumed step counter.
        remaining = int(self.n_timesteps) - int(model.num_timesteps)
        if remaining <= 0:
            self._trained = False
            if model.env is not None:
                model.env.close()
            if not self._resume_requested:
                raise ValueError(
                    f"{self.trained_agent} is at step {model.num_timesteps}, which "
                    f"already meets n_timesteps={self.n_timesteps}; there is "
                    "nothing to fine-tune. Set n_timesteps to the total step "
                    f"count to reach (above {model.num_timesteps}), e.g. "
                    "agent.n_timesteps=<total>."
                )
            logging.warning(
                f"Checkpoint is at step {model.num_timesteps}, which already meets "
                f"n_timesteps={self.n_timesteps}; nothing to train. Raise "
                "n_timesteps to extend the run."
            )
            return
        # Only a resume that will train can overwrite newer saved state.
        _refuse_later_checkpoint_overwrite(
            Path(self.save_path),
            Path(self.trained_agent),
            int(model.num_timesteps),
            env_name=str(self.env_name),
            normalized=bool(self.normalize),
        )
        kwargs: dict[str, Any] = {"reset_num_timesteps": False}
        if self.log_interval > -1:
            kwargs["log_interval"] = self.log_interval
        elif self.log_interval < -1:
            kwargs["log_interval"] = None
        if len(self.callbacks) > 0:
            kwargs["callback"] = self.callbacks
        if self.algo == "ars" and self.n_envs > 1:
            from sb3_contrib.common.vec_env import AsyncEval

            kwargs["async_eval"] = AsyncEval(
                [
                    lambda: self.create_envs(n_envs=1, no_log=True)
                    for _ in range(self.n_envs)
                ],
                model.policy,
            )
        try:
            model.learn(remaining, **kwargs)
        except KeyboardInterrupt:
            # Keep the model so the final save can publish it after interruption.
            pass
        finally:
            if len(self.callbacks) > 0:
                self.callbacks[0].on_training_end()
            # ARS closes its workers only when learn() returns normally.
            if "async_eval" in kwargs:
                kwargs["async_eval"].close()
            try:
                if model.env is not None:
                    model.env.close()
            except EOFError:
                pass


def _publish(path: Path, save: Callable[[Path], Any]) -> None:
    """Publish a complete file via a hidden partial, syncing file and directory."""
    partial = path.with_name(f".{path.name}.partial")
    published = False
    try:
        save(partial)
        with partial.open("rb") as partial_file:
            os.fsync(partial_file.fileno())
        os.replace(partial, path)
        published = True
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if not published:
            partial.unlink(missing_ok=True)


class _StopOnInterruptCallback(BaseCallback):
    """Stop training at the next step once SIGINT was received."""

    def _on_step(self) -> bool:
        # Raise instead of returning False: ARS ignores callback return values.
        if _INTERRUPTED.is_set():
            raise KeyboardInterrupt
        return True


class _CompleteCheckpointCallback(CheckpointCallback):
    """Publish statistics and buffer before the model; keep only the latest buffer."""

    @classmethod
    def replacing(
        cls,
        callback: CheckpointCallback,
        *,
        save_replay_buffer: bool,
        final_partials: Sequence[Path] = (),
    ) -> "_CompleteCheckpointCallback":
        replacement = cls(
            save_freq=callback.save_freq,
            save_path=callback.save_path,
            name_prefix=callback.name_prefix,
            save_replay_buffer=save_replay_buffer,
            save_vecnormalize=True,
            verbose=callback.verbose,
        )
        replacement._final_partials = tuple(final_partials)
        return replacement

    def _init_callback(self) -> None:
        super()._init_callback()
        # Remove partials left by interrupted checkpoint writes.
        for partial in Path(self.save_path).glob(
            f".{glob.escape(self.name_prefix)}_*.partial"
        ):
            partial.unlink(missing_ok=True)
        for partial in getattr(self, "_final_partials", ()):
            partial.unlink(missing_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.save_freq != 0:
            return True
        vec_normalize = self.model.get_vec_normalize_env()
        if self.save_vecnormalize and vec_normalize is not None:
            _publish(
                Path(self._checkpoint_path("vecnormalize_", extension="pkl")),
                lambda path: vec_normalize.save(str(path)),
            )
        buffer_path = None
        if (
            self.save_replay_buffer
            and isinstance(self.model, OffPolicyAlgorithm)
            and self.model.replay_buffer is not None
        ):
            buffer_path = Path(self._checkpoint_path("replay_buffer_", extension="pkl"))
            _publish(buffer_path, self.model.save_replay_buffer)
        model_path = Path(self._checkpoint_path(extension="zip"))
        _publish(model_path, self.model.save)
        if self.verbose >= 2:
            print(f"Saving model checkpoint to {model_path}")
        if buffer_path is not None:
            self._remove_earlier_replay_buffers()
        return True

    def _remove_earlier_replay_buffers(self) -> None:
        for path in Path(self.save_path).glob(
            f"{glob.escape(self.name_prefix)}_replay_buffer_*_steps.pkl"
        ):
            match = _PERIODIC_CHECKPOINT.fullmatch(path.name.removesuffix(".pkl"))
            if match is not None and int(match["steps"]) < self.num_timesteps:
                path.unlink(missing_ok=True)


def _resolve_model_argument(model: Path | str) -> str:
    """Resolve an explicit ``--model`` to an existing SB3/SBX ``.zip``."""
    path = _checkpoint_paths(model)[0]
    if path.suffix != ".zip" or not path.is_file():
        raise FileNotFoundError(
            f"--model {model} is not an SB3/SBX checkpoint: expected an existing "
            "`.zip` model file (the `.zip` suffix may be omitted)"
        )
    return str(path)


def _is_complete_checkpoint(
    checkpoint: Path, *, env_name: str, normalized: bool
) -> bool:
    """Check archive and stats; a missing buffer only re-arms warm-up."""
    if not zipfile.is_zipfile(checkpoint):
        return False
    return (
        not normalized
        or _find_vecnormalize(checkpoint, env_name=env_name).path is not None
    )


def _complete_checkpoint(checkpoint: Path, *, env_name: str, normalized: bool) -> Path:
    """Use ``checkpoint`` or its newest complete sibling; retain it if none."""
    if _is_complete_checkpoint(checkpoint, env_name=env_name, normalized=normalized):
        return checkpoint
    candidates = sorted(
        (
            path
            for path in checkpoint.parent.glob("*.zip")
            if path != checkpoint and path.is_file() and not path.is_symlink()
        ),
        key=lambda path: path.stat().st_mtime_ns,
        reverse=True,
    )
    for candidate in candidates:
        if _is_complete_checkpoint(candidate, env_name=env_name, normalized=normalized):
            logging.warning(
                f"The newest checkpoint {checkpoint} is incomplete (the run was "
                "probably killed while writing it); using the newest complete "
                f"checkpoint {candidate}"
            )
            return candidate
    return checkpoint


def _saved_num_timesteps(checkpoint: Path) -> int | None:
    """Read the scalar timestep metadata from an SB3 archive without loading it."""
    try:
        with zipfile.ZipFile(checkpoint) as archive:
            data = json.loads(archive.read("data"))
    except (OSError, KeyError, TypeError, ValueError, zipfile.BadZipFile):
        return None
    value = data.get("num_timesteps") if isinstance(data, Mapping) else None
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        return None
    return value


def _refuse_later_checkpoint_overwrite(
    checkpoint_dir: Path,
    loaded_checkpoint: Path,
    resumed_steps: int,
    *,
    env_name: str,
    normalized: bool,
) -> None:
    """Stop same-run resumes before learning can replace newer saved state."""
    try:
        loaded_identity = loaded_checkpoint.resolve()
    except OSError:
        loaded_identity = loaded_checkpoint.absolute()

    later: list[tuple[int, Path]] = []
    for candidate in checkpoint_dir.glob("*.zip"):
        try:
            if candidate.resolve() == loaded_identity:
                continue
        except OSError:
            pass
        if not _is_complete_checkpoint(
            candidate, env_name=env_name, normalized=normalized
        ):
            continue
        periodic = _PERIODIC_CHECKPOINT.fullmatch(candidate.stem)
        if periodic is not None:
            candidate_steps = int(periodic["steps"])
        elif candidate.name == f"{env_name}.zip":
            candidate_steps = _saved_num_timesteps(candidate)
            if candidate_steps is None:
                raise ValueError(
                    f"Cannot verify existing final checkpoint {candidate} before "
                    "resuming; use the newest checkpoint or start a separate run."
                )
        else:
            continue
        if candidate_steps > resumed_steps:
            later.append((candidate_steps, candidate))

    if later:
        _steps, candidate = min(later, key=lambda item: (item[0], item[1].name))
        raise ValueError(
            f"Resuming from {loaded_checkpoint} at step {resumed_steps} would "
            f"overwrite later checkpoint {candidate}; resume from the newest "
            "checkpoint or start a separate run."
        )


def _find_vecnormalize(checkpoint: str | Path, *, env_name: str) -> _StatsLookup:
    """Find checkpoint statistics using the shared run-layout lookup."""
    return find_sb3_vecnormalize(checkpoint, env_names=(env_name,))


def _find_replay_buffer(checkpoint: str | Path, *, env_name: str) -> Path | None:
    """Find the matching periodic or final-model replay buffer."""
    if not checkpoint:
        return None
    models = _checkpoint_paths(checkpoint)
    periodic = False
    for model in models:
        match = _PERIODIC_CHECKPOINT.fullmatch(_checkpoint_stem(model))
        if match is None:
            continue
        periodic = True
        own = model.with_name(
            f"{match['prefix']}_replay_buffer_{match['steps']}_steps.pkl"
        )
        if own.is_file():
            return own
    for model in models:
        end_of_run = model.parent / _REPLAY_BUFFER_NAME
        if not end_of_run.is_file():
            continue
        if (
            not periodic
            and _checkpoint_stem(model) == env_name
            and model.is_file()
            and end_of_run.stat().st_mtime_ns >= model.stat().st_mtime_ns
        ):
            return end_of_run
        logging.info(
            f"Not loading {end_of_run}: it was saved with the final model at a "
            f"different training step than {checkpoint}"
        )
    return None


def run(
    workflow: Literal["train", "eval"],
    algo: str,
    env: "AnyEnv | gymnasium.Env",
    sim_app: "SimulationApp",
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: dict,
    logdir: Path,
    model: Path | str | None,
    continue_training: bool | None = None,
    **kwargs,
) -> dict[str, Any] | None:
    # The SIGINT handler that closes rich tqdm bars is scoped to this call and
    # restored afterwards; importing this module must not mutate process state.
    with _sigint_closes_tqdm():
        return _run(
            workflow,
            algo,
            env,
            sim_app,
            env_id,
            env_cfg,
            agent_cfg,
            logdir,
            model,
            continue_training=continue_training,
            **kwargs,
        )


def _run(
    workflow: Literal["train", "eval"],
    algo: str,
    env: "AnyEnv | gymnasium.Env",
    sim_app: "SimulationApp",
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: dict,
    logdir: Path,
    model: Path | str | None,
    continue_training: bool | None = None,
    **kwargs,
) -> dict[str, Any] | None:
    run_metadata = kwargs.pop("run_metadata", None)
    evaluation_logdir = kwargs.pop("evaluation_logdir", None)

    device = agent_cfg.pop("device", "auto")
    log_interval = agent_cfg.pop("log_interval", -1)
    verbose = agent_cfg.pop("verbose", True)
    track = agent_cfg.pop("track", False)
    init_tensorboard = agent_cfg.pop("tensorboard", True)
    init_wandb = agent_cfg.pop("wandb", False)
    # Configured save_freq is in environment steps; convert it to VecEnv calls.
    save_freq = agent_cfg.pop("save_freq", 50_000)
    save_replay_buffer = agent_cfg.pop("save_replay_buffer", False)
    truncate_last_trajectory = agent_cfg.pop("truncate_last_trajectory", True)

    training_logdir = Path(logdir)
    if model:
        model = _resolve_model_argument(model)
    from_checkpoint = find_sb3_checkpoint(
        training_logdir, model, workflow, continue_training
    )
    requested_checkpoint = from_checkpoint
    if from_checkpoint and not model:
        from_checkpoint = _complete_checkpoint(
            Path(from_checkpoint),
            env_name=str(EnvironmentName(env_id)),
            normalized=_normalization_requested(agent_cfg),
        )
    checkpoint_path = (
        _checkpoint_loader_path(from_checkpoint) if from_checkpoint else ""
    )
    loaded_checkpoint = from_checkpoint
    checkpoint_fallback = bool(
        requested_checkpoint
        and loaded_checkpoint
        and not _same_checkpoint(requested_checkpoint, loaded_checkpoint)
    )
    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")
    elif continue_training:
        logging.info(
            "continue_training requested but no checkpoint found — starting fresh"
        )

    if workflow == "eval":
        logdir = (
            Path(evaluation_logdir)
            if evaluation_logdir is not None
            else new_eval_logdir(training_logdir)
        )
    else:
        logdir = training_logdir

    tensorboard_log = (
        logdir.joinpath("tensorboard") if track or init_tensorboard else None
    )
    with run_logging(
        enabled=init_wandb,
        name=f"{env_id}_{algo}",
        run_metadata=run_metadata,
    ):
        if tensorboard_log and run_metadata:
            try:
                from torch.utils.tensorboard import SummaryWriter

                from srb.utils.metadata import metadata_json_text

                with SummaryWriter(tensorboard_log.as_posix()) as _tb_writer:
                    _tb_writer.add_text(
                        "run_metadata",
                        "```json\n" + metadata_json_text(run_metadata) + "\n```",
                    )
            except Exception as exc:  # noqa: BLE001 - metadata logging is best-effort
                logging.warning(f"Could not write TensorBoard run metadata: {exc}")

        # Algorithms without MultiInputPolicy (ARS, CrossQ) need one Box observation.
        if not _supports_dict_observations(algo) and _is_dict_observation_space(env):
            env = MergeDictObsWrapper(env)  # type: ignore[misc]
        # Preserve task success on completed rows for bounded evaluation.
        env = Sb3EnvWrapper(env)  # type: ignore[misc]
        exp_manager = ExperimentManager(
            # rl_zoo3 records args.yml from args.__dict__; provide the SRB invocation.
            args=experiment_args(
                srb_entrypoint=f"srb agent {workflow}",
                algo=algo,
                env=env_id,
                seed=env_cfg.seed if env_cfg else 0,
                log_folder=logdir.as_posix(),
            ),
            algo=algo,
            env_id=env_id,
            log_folder=logdir.as_posix(),
            tensorboard_log=tensorboard_log.as_posix() if tensorboard_log else "",
            n_timesteps=0,
            eval_freq=0,
            n_eval_episodes=0,
            save_freq=_checkpoint_save_frequency(save_freq, env.num_envs),
            hyperparams=agent_cfg,
            trained_agent=checkpoint_path,
            optimize_hyperparameters=False,
            truncate_last_trajectory=truncate_last_trajectory,
            seed=env_cfg.seed if env_cfg else 0,
            log_interval=log_interval,
            save_replay_buffer=save_replay_buffer,
            verbose=verbose,
            n_eval_envs=0,
            no_optim_plots=False,
            device=device,
            config=None,
            show_progress=True,
            env=env,
            resume_requested=bool(continue_training),
        )

        match workflow:
            case "train":
                agent_model, _saved_hyperparams = exp_manager.setup_experiment()  # type: ignore[misc]
                exp_manager.learn(agent_model)
                exp_manager.save_trained_model(agent_model)
            case "eval":
                env = exp_manager.create_envs(0, eval_env=True)  # type: ignore[misc]
                eval_load_kwargs: dict[str, Any] = {
                    "env": env,
                    "device": env.unwrapped.device,  # type: ignore[misc]
                }
                if algo in OFF_POLICY_ALGOS:
                    eval_load_kwargs["buffer_size"] = 1
                    eval_load_kwargs["optimize_memory_usage"] = False
                    replay_buffer_kwargs = agent_cfg.get("replay_buffer_kwargs")
                    if isinstance(replay_buffer_kwargs, dict):
                        eval_load_kwargs["replay_buffer_kwargs"] = replay_buffer_kwargs
                    replay_buffer_class = agent_cfg.get("replay_buffer_class")
                    if isinstance(replay_buffer_class, type):
                        eval_load_kwargs["replay_buffer_class"] = replay_buffer_class

                agent = ALGOS[algo].load(
                    checkpoint_path,  # type: ignore[misc]
                    **eval_load_kwargs,
                )

                n_eval_episodes = eval_episode_budget()
                if n_eval_episodes is None:
                    # Keep open-ended visualization when no quantitative budget is set.
                    episode_start = np.ones(
                        (env.unwrapped.num_envs,),  # type: ignore[misc]
                        dtype=bool,
                    )
                    lstm_states = None

                    obs = env.reset()
                    for _ in tqdm(range(agent_cfg["n_timesteps"])):
                        if not sim_app.is_running():
                            break
                        action, lstm_states = agent.predict(
                            obs,  # type: ignore[misc]
                            state=lstm_states,
                            episode_start=episode_start,  # type: ignore[misc]
                            deterministic=True,
                        )
                        obs, _reward, episode_start, _infos = env.step(action)  # type: ignore[misc]
                else:
                    result = _run_bounded_eval(
                        agent,
                        env,
                        sim_app,
                        target_episodes=n_eval_episodes,
                        reset_seed=effective_eval_seed(env_cfg),
                        requested_checkpoint=(
                            str(requested_checkpoint) if requested_checkpoint else None
                        ),
                        loaded_checkpoint=(
                            str(loaded_checkpoint) if loaded_checkpoint else None
                        ),
                        checkpoint_fallback=checkpoint_fallback,
                    )
                    write_eval_result(result, evaluation_logdir=logdir)
                    return result


def _same_checkpoint(first: Path | str, second: Path | str) -> bool:
    try:
        return Path(first).resolve() == Path(second).resolve()
    except OSError:
        return Path(first).absolute() == Path(second).absolute()


def _checkpoint_loader_path(checkpoint: Path | str) -> str:
    """Normalize CLI and discovered checkpoints for SB3 loader APIs."""
    return str(checkpoint)


def _explicit_success(info: Any) -> bool | None:
    """Extract a scalar task-provided success flag from terminal info."""
    if not isinstance(info, Mapping) or "success" not in info:
        return None
    value = info["success"]
    if value is None:
        return None
    if isinstance(value, torch.Tensor):
        if value.numel() != 1:
            return None
        value = value.detach().cpu().item()
    elif isinstance(value, np.ndarray):
        if value.size != 1:
            return None
        value = value.item()
    elif isinstance(value, (list, tuple)):
        if len(value) != 1:
            return None
        value = value[0]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return bool(value) if value in (0, 1) else None
    if isinstance(value, (float, np.floating)):
        numeric = float(value)
        return bool(value) if math.isfinite(numeric) and numeric in (0.0, 1.0) else None
    return None


def _run_bounded_eval(
    agent: Any,
    env: Any,
    sim_app: Any,
    *,
    target_episodes: int,
    reset_seed: Any | None = None,
    requested_checkpoint: str | None = None,
    loaded_checkpoint: str | None = None,
    checkpoint_fallback: bool = False,
) -> dict[str, Any]:
    """Evaluate deterministic SB3/SBX actions over exactly N episodes.

    A per-env quota of episodes started after reset avoids over-counting short episodes.
    Sum vector rewards before resets for undiscounted returns, independent of monitor
    wrappers; :class:`Sb3EnvWrapper` owns terminal observations and resets.
    """
    if target_episodes <= 0:
        raise ValueError(f"target_episodes must be positive, got {target_episodes!r}")
    num_envs = int(env.unwrapped.num_envs)
    max_steps = eval_max_steps(env, target_episodes)
    quota = EpisodeQuota(target_episodes, num_envs)
    episode_start = np.ones((num_envs,), dtype=bool)
    episode_returns = np.zeros((num_envs,), dtype=np.float64)
    lstm_states = None
    if reset_seed is not None:
        # VecNormalize.reset() has no seed argument; seed the wrapper chain before reset.
        env.seed(reset_seed)
    obs = env.reset()
    completed_returns: list[float] = []
    success_values: list[bool | None] = []
    step_count = 0
    stop_reason: str | None = None

    while not quota.complete and step_count < max_steps and sim_app.is_running():
        action, lstm_states = agent.predict(
            obs,
            state=lstm_states,
            episode_start=episode_start,
            deterministic=True,
        )
        obs, reward, dones, infos = env.step(action)
        step_count += 1
        reward_array = np.asarray(reward, dtype=np.float64).reshape(-1)
        if reward_array.size != num_envs:
            raise ValueError(
                "SB3 evaluation expected one reward per environment, "
                f"got {reward_array.size} for {num_envs} environments"
            )
        if not np.isfinite(reward_array).all():
            raise ValueError("SB3 evaluation received a non-finite reward")
        # Reject accumulator overflow before returning a non-finite result.
        with np.errstate(over="ignore", invalid="ignore"):
            episode_returns += reward_array
        if not np.isfinite(episode_returns).all():
            raise ValueError("SB3 evaluation episode return became non-finite")
        done_array = np.asarray(dones, dtype=bool).reshape(-1)
        if done_array.size != num_envs:
            raise ValueError(
                "SB3 evaluation expected one done flag per environment, "
                f"got {done_array.size} for {num_envs} environments"
            )
        done_ids = [int(env_id) for env_id in np.flatnonzero(done_array)]
        # Attribute only completions within this row's quota; the wrapper still resets every row.
        for env_id in quota.accept(done_ids):
            completed_returns.append(float(episode_returns[env_id]))
            info = infos[env_id] if len(infos) > env_id else None
            success_values.append(_explicit_success(info))
        episode_returns[done_array] = 0.0
        episode_start = done_array

    if quota.complete:
        stop_reason = None
    elif step_count >= max_steps:
        stop_reason = "step_budget"
    elif not sim_app.is_running():
        stop_reason = "sim_stopped"
    else:  # pragma: no cover - defensive guard for a future loop change
        stop_reason = "incomplete"

    complete = quota.complete
    if not complete:
        logging.warning(
            "SB3 evaluation incomplete: %s/%s episodes after %s/%s steps (%s)",
            len(completed_returns),
            target_episodes,
            step_count,
            max_steps,
            stop_reason,
        )
    has_success = bool(success_values) and all(
        value is not None for value in success_values
    )
    successes = sum(bool(value) for value in success_values) if has_success else None
    result: dict[str, Any] = {
        "success_rate": (
            successes / len(completed_returns)
            if successes is not None and completed_returns
            else None
        ),
        "successes": successes,
        "failures": (
            len(completed_returns) - successes if successes is not None else None
        ),
        "mean_return": (
            math.fsum(value / len(completed_returns) for value in completed_returns)
            if completed_returns
            else None
        ),
        "n_episodes": len(completed_returns),
        "episodes_requested": target_episodes,
        "episodes_completed": len(completed_returns),
        "complete": complete,
        "incomplete": not complete,
        "steps": step_count,
        "max_steps": max_steps,
        "stop_reason": stop_reason,
        "requested_checkpoint": requested_checkpoint,
        "loaded_checkpoint": loaded_checkpoint,
        "checkpoint_fallback": checkpoint_fallback,
        **quota.result_fields(),
    }
    return result


def _close_rich_tqdm() -> None:
    tqdm_objects = [obj for obj in gc.get_objects() if "tqdm" in type(obj).__name__]
    for tqdm_object in tqdm_objects:
        if "tqdm_rich" in type(tqdm_object).__name__:
            tqdm_object.close()


@contextlib.contextmanager
def _sigint_closes_tqdm():
    """Close rich tqdm bars on SIGINT for the duration of a run.

    The previous handler (SRB's, which force-closes Kit on a second Ctrl-C)
    then runs as usual. Only the main thread may install signal handlers;
    elsewhere the handler is left untouched. The previous handler is always
    restored.
    """
    if threading.current_thread() is not threading.main_thread():
        yield
        return

    def on_interrupt(signum, frame):
        _INTERRUPTED.set()
        _close_rich_tqdm()
        if callable(previous):
            previous(signum, frame)
        raise KeyboardInterrupt

    _INTERRUPTED.clear()
    previous = signal.signal(signal.SIGINT, on_interrupt)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, previous)
