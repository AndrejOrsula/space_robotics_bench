import gc
import signal
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Sequence

import gymnasium
import numpy
from isaacsim.simulation_app import SimulationApp
from rl_zoo3 import ALGOS
from stable_baselines3.common.callbacks import tqdm

from srb.integrations.sb3.exp_manager import ExperimentManager
from srb.integrations.sb3.wrapper import Sb3EnvWrapper
from srb.utils import logging
from srb.utils.cfg import last_file, stamp_dir
from srb.wrappers import maybe_wrap_action_smoothing

if TYPE_CHECKING:
    from srb._typing import AnyEnv, AnyEnvCfg

FRAMEWORK_NAME = "sb3"
OFF_POLICY_ALGOS: Sequence[str] = ("qrdqn", "dqn", "ddpg", "sac", "her", "td3", "tqc")


def run(
    workflow: Literal["train", "eval"],
    algo: str,
    env: "AnyEnv | gymnasium.Env",
    sim_app: SimulationApp,
    env_id: str,
    env_cfg: "AnyEnvCfg | None",
    agent_cfg: dict,
    logdir: Path,
    model: Path,
    continue_training: bool | None = None,
    **kwargs,
):
    ## Extract params from agent_cfg
    # All
    log_interval = agent_cfg.pop("log_interval", -1)
    verbose = agent_cfg.pop("verbose", True)
    track = agent_cfg.pop("track", False)
    init_tensorboard = agent_cfg.pop("tensorboard", True)
    init_wandb = agent_cfg.pop("wandb", False)
    # Train
    save_freq = agent_cfg.pop("save_freq", -1)
    save_replay_buffer = agent_cfg.pop("save_replay_buffer", False)
    # Optimize
    n_trials = agent_cfg.pop("n_trials", 500)
    sampler = agent_cfg.pop("sampler", "tpe")
    pruner = agent_cfg.pop("pruner", "median")
    n_startup_trials = agent_cfg.pop("n_startup_trials", 10)
    n_evaluations = agent_cfg.pop("n_evaluations", 1)
    # HER
    truncate_last_trajectory = agent_cfg.pop("truncate_last_trajectory", True)

    # Pop the entire smoothing config dictionary to be handled separately.
    smoothing_cfg = agent_cfg.pop("smoothing", {})

    # Determine checkpoint path
    if model:
        from_checkpoint = model
    elif workflow == "eval" or continue_training:
        from_checkpoint = last_file(logdir.joinpath("ckpt"), modification_time=True)
    else:
        from_checkpoint = ""
    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")

    # Special handling for eval workflow
    if workflow == "eval":
        logdir = stamp_dir(logdir.joinpath("eval"))

    tensorboard_log = (
        logdir.joinpath("tensorboard") if track or init_tensorboard else None
    )
    if init_wandb:
        import wandb

        _run = wandb.init(
            name=f"{env_id}_{algo}",
            sync_tensorboard=True,
            monitor_gym=True,
        )

    # Enable action smoothing if enabled
    env = maybe_wrap_action_smoothing(
        env,  # type: ignore
        smoothing_cfg,
    )

    # Wrap the environment
    env = Sb3EnvWrapper(env)  # type: ignore

    exp_manager = ExperimentManager(
        args={},  # type: ignore
        algo=algo,
        env_id=env_id,
        log_folder=logdir.as_posix(),
        tensorboard_log=tensorboard_log.as_posix() if tensorboard_log else "",
        n_timesteps=0,
        eval_freq=0,
        n_eval_episodes=0,
        save_freq=save_freq,
        hyperparams=agent_cfg,
        trained_agent=from_checkpoint.as_posix()
        if isinstance(from_checkpoint, Path)
        else from_checkpoint,
        optimize_hyperparameters=workflow == "optimize",
        n_trials=n_trials,
        sampler=sampler,
        pruner=pruner,
        # optimization_log_path=args.optimization_log_path,
        n_startup_trials=n_startup_trials,
        n_evaluations=n_evaluations,
        truncate_last_trajectory=truncate_last_trajectory,
        seed=env_cfg.seed if env_cfg else 0,
        log_interval=log_interval,
        save_replay_buffer=save_replay_buffer,
        verbose=verbose,
        n_eval_envs=0,
        no_optim_plots=False,
        device=env.unwrapped.device,  # type: ignore
        config=None,
        show_progress=True,
        env=env,
    )

    # Run the workflow
    match workflow:
        case "train":
            agent_model, _saved_hyperparams = exp_manager.setup_experiment()  # type: ignore
            exp_manager.learn(agent_model)
            exp_manager.save_trained_model(agent_model)
        case "optimize":
            exp_manager.setup_experiment()
            exp_manager.hyperparameters_optimization()
        case "eval":
            env = exp_manager.create_envs(0, eval_env=True)  # type: ignore

            # Update agent config
            if algo in OFF_POLICY_ALGOS:
                agent_cfg.update(dict(buffer_size=1))
                if "optimize_memory_usage" in agent_cfg:
                    agent_cfg.update(optimize_memory_usage=False)
            if "HerReplayBuffer" in agent_cfg.get("replay_buffer_class", ""):
                agent_cfg["env"] = env

            # Load the agent
            agent = ALGOS[algo].load(
                from_checkpoint.as_posix(),  # type: ignore
                device=env.unwrapped.device,  # type: ignore
            )

            # Initialize the runner
            episode_start = numpy.ones(
                (env.unwrapped.num_envs,),  # type: ignore
                dtype=bool,
            )
            lstm_states = None

            obs = env.reset()
            for _ in tqdm(range(agent_cfg["n_timesteps"])):
                if not sim_app.is_running():
                    break
                action, lstm_states = agent.predict(
                    obs,  # type: ignore
                    state=lstm_states,
                    episode_start=episode_start,  # type: ignore
                    deterministic=True,
                )
                obs, _reward, episode_start, _infos = env.step(action)  # type: ignore


def gc_tqdm(*args):
    tqdm_objects = [obj for obj in gc.get_objects() if "tqdm" in type(obj).__name__]
    for tqdm_object in tqdm_objects:
        if "tqdm_rich" in type(tqdm_object).__name__:
            tqdm_object.close()
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, gc_tqdm)
