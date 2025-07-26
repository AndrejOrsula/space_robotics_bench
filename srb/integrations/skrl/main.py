from pathlib import Path
from typing import TYPE_CHECKING, Literal

import gymnasium
from isaacsim.simulation_app import SimulationApp
from skrl.utils.runner.torch import Runner

from srb.integrations.skrl.wrapper import SkrlEnvWrapper
from srb.utils import logging
from srb.utils.cfg import last_file, stamp_dir
from srb.wrappers import maybe_wrap_action_smoothing

if TYPE_CHECKING:
    from srb._typing import AnyEnv, AnyEnvCfg

FRAMEWORK_NAME = "skrl"


def run(
    workflow: Literal["train", "eval"],
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
    # Pop the entire smoothing config dictionary to be handled separately.
    smoothing_cfg = agent_cfg.pop("smoothing", {})

    # Determine checkpoint path
    if model:
        from_checkpoint = model
    elif workflow == "eval" or continue_training:
        from_checkpoint = last_file(
            logdir.joinpath("checkpoints"), modification_time=True
        )
    else:
        from_checkpoint = ""
    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")

    # Special handling for eval workflow
    if workflow == "eval":
        logdir = stamp_dir(logdir.joinpath("eval"))

    # Update agent config
    agent_cfg["seed"] = env_cfg.seed if env_cfg else 0
    agent_cfg["agent"]["experiment"]["directory"] = logdir.parent
    agent_cfg["agent"]["experiment"]["experiment_name"] = logdir

    # Enable action smoothing if enabled
    env = maybe_wrap_action_smoothing(
        env,  # type: ignore
        smoothing_cfg,
    )

    # Wrap the environment
    env = SkrlEnvWrapper(env)  # type: ignore

    # Create the runner
    runner = Runner(
        env,  # type: ignore
        agent_cfg,
    )

    # Load checkpoint if needed
    if from_checkpoint:
        runner.agent.load(
            from_checkpoint,  # type: ignore
        )

    # Run the workflow
    runner.run(mode=workflow)
