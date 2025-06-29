from functools import partial as bind
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Literal

import elements
import embodied
import gymnasium
import numpy
import portal
from dreamerv3 import agent as dreamer_agent
from dreamerv3 import main as dreamer_main
from isaacsim.simulation_app import SimulationApp
from ruamel import yaml

from srb.integrations.dreamer.eval import eval_only
from srb.integrations.dreamer.train import train
from srb.integrations.dreamer.wrapper import EmbodiedEnvWrapper
from srb.utils import logging
from srb.utils.cfg import stamp_dir

if TYPE_CHECKING:
    from srb._typing import AnyEnv, AnyEnvCfg

ALGO_NAME = "dreamer"
UPSTREAM_CONFIG_PATH = Path(dreamer_agent.__file__).parent.joinpath("configs.yaml")


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
    save_replay = agent_cfg.get("replay", {}).pop("save", False)

    # Determine checkpoint path
    if model:
        from_checkpoint = model
    elif workflow == "eval" or continue_training:
        from_checkpoint = logdir.joinpath("ckpt").joinpath(
            logdir.joinpath("ckpt").joinpath("latest").read_text().strip()
        )
    else:
        from_checkpoint = ""
    if from_checkpoint:
        logging.info(f"Loading model from {from_checkpoint}")

    # Special handling for eval workflow
    if workflow == "eval":
        logdir = stamp_dir(logdir.joinpath("eval"))

    # Setup logdir
    logdir = elements.Path(logdir)  # type: ignore
    logdir.mkdir()
    print("Agent logdir:", logdir)

    # Load the config
    configs: Dict[str, Any] = yaml.YAML(typ="safe").load(
        UPSTREAM_CONFIG_PATH.read_text()
    )
    config = elements.Config(configs["defaults"])
    config = config.update(
        {
            **agent_cfg,
            "seed": env_cfg.seed if env_cfg else 0,
            "task": env_id.replace("/", "_"),
            "logdir": logdir,
            "run.from_checkpoint": from_checkpoint,
            "run.envs": env_cfg.scene.num_envs if env_cfg else 1,  # type: ignore
            "run.eval_envs": env_cfg.scene.num_envs if env_cfg else 0,  # type: ignore
        }
    )

    # Save the config
    config.save(logdir / "config.yaml")

    # Wrap the environment
    env = EmbodiedEnvWrapper(env)  # type: ignore
    for name, space in env.act_space.items():  # type: ignore
        if not space.discrete:
            env = embodied.wrappers.NormalizeAction(env, name)  # type: ignore
    env = embodied.wrappers.UnifyDtypes(env)  # type: ignore
    for name, space in env.act_space.items():  # type: ignore
        if not space.discrete:
            env = embodied.wrappers.ClipAction(env, name)  # type: ignore

    # Setup the workflow
    def init():
        elements.timer.global_timer.enabled = config.logger.timer  # type: ignore

    portal.setup(
        errfile=config.errfile and logdir / "error",
        clientkw=dict(logging_color="cyan"),
        serverkw=dict(logging_color="cyan"),
        initfns=[init],
        ipv6=config.ipv6,
    )

    args = elements.Config(
        **config.run,
        replica=config.replica,
        replicas=config.replicas,
        logdir=config.logdir,
        batch_size=config.batch_size,
        batch_length=config.batch_length,
        report_length=config.report_length,
        consec_train=config.consec_train,
        consec_report=config.consec_report,
        replay_context=config.replay_context,
    )

    def make_env():
        return env

    def make_agent(config):
        obs_space = {k: v for k, v in env.obs_space.items() if not k.startswith("log/")}  # type: ignore
        act_space = {k: v for k, v in env.act_space.items() if k != "reset"}  # type: ignore
        if config.random_agent:
            return embodied.RandomAgent(obs_space, act_space)
        cpdir = elements.Path(config.logdir)
        cpdir = cpdir.parent if config.replicas > 1 else cpdir
        return dreamer_agent.Agent(
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

    # Run the workflow
    match workflow:
        case "train":
            train(
                bind(make_agent, config),
                bind(make_replay, config, "replay" if save_replay else None),
                make_env,
                bind(dreamer_main.make_stream, config),
                bind(dreamer_main.make_logger, config),
                args,
                sim_app=sim_app,
            )
        case "eval":
            eval_only(
                bind(make_agent, config),
                make_env,
                bind(dreamer_main.make_logger, config),
                args,
                sim_app=sim_app,
            )


def make_replay(config, folder: str | Path | None, mode: str = "train"):
    batlen = config.batch_length if mode == "train" else config.report_length
    consec = config.consec_train if mode == "train" else config.consec_report
    capacity = config.replay.size if mode == "train" else config.replay.size / 10
    length = consec * batlen + config.replay_context
    assert config.batch_size * length <= capacity

    if folder:
        directory = elements.Path(config.logdir) / folder
        if config.replicas > 1:
            directory /= f"{config.replica:05}"
    else:
        directory = None
    replay_kwargs = {
        "length": length,
        "capacity": int(capacity),
        "online": config.replay.online,
        "chunksize": config.replay.chunksize,
        "directory": directory,
    }

    if mode == "train":
        assert (
            config.replay.fracs.uniform
            + config.replay.fracs.priority
            + config.replay.fracs.recency
            == 1.0
        ), "Replay fractions must sum to 1."

        if config.replay.fracs.priority > 0.0:
            assert config.jax.compute_dtype in ("bfloat16", "float32"), (
                "Gradient scaling for low-precision training can produce invalid loss "
                "outputs that are incompatible with prioritized replay."
            )
        if config.replay.fracs.recency > 0.0:
            recency = 1.0 / numpy.arange(1, capacity + 1) ** config.replay.recexp

        if config.replay.fracs.uniform == 1.0:
            replay_kwargs["selector"] = embodied.replay.selectors.Uniform(
                seed=config.seed
            )
        elif config.replay.fracs.priority == 1.0:
            replay_kwargs["selector"] = embodied.replay.selectors.Prioritized(
                seed=config.seed, **config.replay.prio
            )
        elif config.replay.fracs.recency == 1.0:
            replay_kwargs["selector"] = embodied.replay.selectors.Recency(
                recency, seed=config.seed
            )
        else:
            from srb.integrations.dreamer.selector import Mixture

            replay_kwargs["selector"] = Mixture(
                selectors={
                    "uniform": embodied.replay.selectors.Uniform(seed=config.seed),
                    "priority": embodied.replay.selectors.Prioritized(
                        seed=config.seed, **config.replay.prio
                    ),
                    "recency": embodied.replay.selectors.Recency(
                        recency, seed=config.seed
                    ),
                },
                fractions=config.replay.fracs,
                seed=config.seed,
            )

    return embodied.replay.Replay(**replay_kwargs)
