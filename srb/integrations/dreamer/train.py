import collections
from functools import partial as bind

import elements
import embodied
import numpy
from isaacsim.simulation_app import SimulationApp

from srb.integrations.dreamer.driver import DriverParallelEnv


def train(
    make_agent,
    make_replay,
    make_env,
    make_stream,
    make_logger,
    args,
    sim_app: SimulationApp,
):
    agent = make_agent()
    replay = make_replay()
    logger = make_logger()

    logdir = elements.Path(args.logdir)
    step = logger.step
    usage = elements.Usage(**args.usage)
    train_agg = elements.Agg()
    epstats = elements.Agg()
    episodes = collections.defaultdict(elements.Agg)
    policy_fps = elements.FPS()
    train_fps = elements.FPS()

    batch_steps = args.batch_size * args.batch_length
    should_train = elements.when.Ratio(args.train_ratio / batch_steps)
    should_log = embodied.LocalClock(args.log_every)
    should_report = embodied.LocalClock(args.report_every)
    should_save = embodied.LocalClock(args.save_every)

    @elements.timer.section("logfn")
    def logfn(tran, worker):
        episode = episodes[worker]
        tran["is_first"] and episode.reset()  # type: ignore
        episode.add("score", tran["reward"], agg="sum")
        episode.add("length", 1, agg="sum")
        episode.add("rewards", tran["reward"], agg="stack")
        for key, value in tran.items():
            if value.dtype == numpy.uint8 and value.ndim == 3:
                if worker == 0:
                    if value.shape[-1] >= 3:
                        episode.add(f"policy_{key}_ch0-2", value[..., :3], agg="stack")
                        for i in range(3, value.shape[-1]):
                            episode.add(
                                f"policy_{key}_ch{i}",
                                value[..., i : i + 1],
                                agg="stack",
                            )
                    else:
                        for i in range(value.shape[-1]):
                            episode.add(
                                f"policy_{key}_ch{i}",
                                value[..., i : i + 1],
                                agg="stack",
                            )
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

    ########### ONLY THIS IS CHANGED (+1 fix below) ############
    # fns = [bind(make_env, i) for i in range(args.envs)]
    # driver = embodied.Driver(fns, parallel=not args.debug)
    driver = DriverParallelEnv(env=make_env(), num_envs=args.envs)
    ########### ONLY THIS IS CHANGED (+1 fix below) ############
    driver.on_step(lambda tran, _: step.increment())
    driver.on_step(lambda tran, _: policy_fps.step())
    driver.on_step(replay.add)
    driver.on_step(logfn)

    stream_train = iter(agent.stream(make_stream(replay, "train")))
    stream_report = iter(agent.stream(make_stream(replay, "report")))

    carry_train = [agent.init_train(args.batch_size)]
    carry_report = agent.init_report(args.batch_size)

    def trainfn(tran, worker):
        if len(replay) < args.batch_size * args.batch_length:
            return
        for _ in range(should_train(step)):
            with elements.timer.section("stream_next"):
                batch = next(stream_train)
            carry_train[0], outs, mets = agent.train(carry_train[0], batch)
            train_fps.step(batch_steps)
            if "replay" in outs:
                replay.update(outs["replay"])
            train_agg.add(mets, prefix="train")

    driver.on_step(trainfn)

    cp = elements.Checkpoint(logdir / "ckpt")
    cp.step = step
    cp.agent = agent
    cp.replay = replay
    if args.from_checkpoint:
        ########### Fix below ############
        # elements.checkpoint.load(
        #     args.from_checkpoint,
        #     dict(agent=bind(agent.load, regex=args.from_checkpoint_regex)),
        # )
        elements.checkpoint.load(args.from_checkpoint, dict(agent=bind(agent.load)))
        ########### Fix below ############
    cp.load_or_save()

    print("Start training loop")
    policy = lambda *args: agent.policy(*args, mode="train")  # noqa: E731
    driver.reset(agent.init_policy)
    while step < args.steps:
        ########### Sim extra (optional) ############
        if not sim_app.is_running():
            break
        ########### Sim extra (optional) ############
        driver(policy, steps=10)

        if should_report(step) and len(replay):
            agg = elements.Agg()
            for _ in range(args.consec_report * args.report_batches):
                carry_report, mets = agent.report(carry_report, next(stream_report))
                agg.add(mets)
            logger.add(agg.result(), prefix="report")

        if should_log(step):
            logger.add(train_agg.result())
            logger.add(epstats.result(), prefix="epstats")
            logger.add(replay.stats(), prefix="replay")
            logger.add(usage.stats(), prefix="usage")
            logger.add({"fps/policy": policy_fps.result()})
            logger.add({"fps/train": train_fps.result()})
            timer_stats = elements.timer.stats()
            if "summary" in timer_stats:
                logger.add({"timer": timer_stats["summary"]})
            logger.write()

        if should_save(step):
            cp.save()

    logger.close()
