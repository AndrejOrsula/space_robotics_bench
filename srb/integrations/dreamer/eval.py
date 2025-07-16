from collections import defaultdict

import elements
import numpy
from isaacsim.simulation_app import SimulationApp

from srb.integrations.dreamer.driver import DriverParallelEnv


def eval_only(
    make_agent,
    make_env,
    make_logger,
    args,
    sim_app: SimulationApp,
):
    assert args.from_checkpoint

    agent = make_agent()
    logger = make_logger()

    logdir = elements.Path(args.logdir)
    logdir.mkdir()
    print("Logdir", logdir)
    step = logger.step
    usage = elements.Usage(**args.usage)
    agg = elements.Agg()
    epstats = elements.Agg()
    episodes = defaultdict(elements.Agg)
    should_log = elements.when.Clock(args.log_every)
    policy_fps = elements.FPS()

    @elements.timer.section("logfn")
    def logfn(tran, worker):
        episode = episodes[worker]
        tran["is_first"] and episode.reset()  # type: ignore
        episode.add("score", tran["reward"], agg="sum")
        episode.add("length", 1, agg="sum")
        episode.add("rewards", tran["reward"], agg="stack")
        for key, value in tran.items():
            isimage = (value.dtype == numpy.uint8) and (value.ndim == 3)
            if isimage and worker == 0:
                episode.add(f"policy_{key}", value, agg="stack")
            elif key.startswith("log/"):
                assert value.ndim == 0, (key, value.shape, value.dtype)
                episode.add(key, value, agg=("avg", "max", "sum"))  # type: ignore
        if tran["is_last"]:
            result = episode.result()
            logger.add(
                {
                    "score": result.pop("score"),
                    "length": result.pop("length"),
                },
                prefix="episode",
            )
            rew = result.pop("rewards")
            if len(rew) > 1:
                result["reward_rate"] = (numpy.abs(rew[1:] - rew[:-1]) >= 0.01).mean()
            epstats.add(result)

    ########### ONLY THIS IS CHANGED ############
    # fns = [bind(make_env, i) for i in range(args.envs)]
    # driver = embodied.Driver(fns, parallel=(not args.debug))
    driver = DriverParallelEnv(env=make_env(), num_envs=args.envs)
    ########### ONLY THIS IS CHANGED ############
    driver.on_step(lambda tran, _: step.increment())
    driver.on_step(lambda tran, _: policy_fps.step())
    driver.on_step(logfn)

    cp = elements.Checkpoint()
    cp.agent = agent
    cp.load(args.from_checkpoint, keys=["agent"])

    print("Start evaluation")
    policy = lambda *args: agent.policy(*args, mode="eval")  # noqa: E731
    driver.reset(agent.init_policy)
    while step < args.steps:
        ########### Sim extra (optional) ############
        if not sim_app.is_running():
            break
        ########### Sim extra (optional) ############
        driver(policy, steps=10)
        if should_log(step):
            logger.add(agg.result())
            logger.add(epstats.result(), prefix="epstats")
            logger.add(usage.stats(), prefix="usage")
            logger.add({"fps/policy": policy_fps.result()})
            timer_stats = elements.timer.stats()
            if "summary" in timer_stats:
                logger.add({"timer": timer_stats["summary"]})
            logger.write()

    logger.close()
