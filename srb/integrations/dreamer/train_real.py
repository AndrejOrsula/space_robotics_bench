import collections
import hashlib
import os
import shutil
import sys
import tempfile
import threading
from functools import partial as bind
from pathlib import Path

import elements
import embodied
from isaacsim.simulation_app import SimulationApp

from srb.integrations.dreamer.checkpoint_transaction import checkpoint_read_lock
from srb.integrations.dreamer.real_driver import RealTimeDriver
from srb.integrations.dreamer.train import make_episode_logfn
from srb.utils import logging

# Complete `--model`/`--continue` source checkpoints, including replay, live in
# `<logdir>/source_checkpoint_backup/<source name>-<source path key>/`, outside
# `ckpt/` retention.
# This depth lets `srb real_agent train --model <backup>` fork a fresh
# `real_transfer/` run instead of training in place.
SOURCE_CHECKPOINT_BACKUP_DIR = "source_checkpoint_backup"
# Where runs written before this layout kept their backup.
LEGACY_CHECKPOINT_BACKUP_DIR = "backup"
# Keep the last two checkpoint generations for resume.
CHECKPOINT_GENERATIONS_KEPT = 2


def _free_child(root: Path, name: str) -> Path:
    candidate = root / name
    index = 1
    while candidate.exists() or candidate.is_symlink():
        candidate = root / f"{name}-{index}"
        index += 1
    return candidate


def _is_complete_checkpoint(path: Path) -> bool:
    return path.is_dir() and (path / "done").is_file()


def _backup_source_checkpoint(from_checkpoint, logdir: Path) -> Path:
    """Atomically back up a complete source checkpoint.

    Stage in a hidden sibling, remove stale staging folders, and reuse an
    existing complete backup to avoid publishing partial copies.
    """
    source = Path(str(from_checkpoint))
    if not (source / "done").is_file():
        raise FileNotFoundError(
            f"Dreamer source checkpoint {source} is missing or incomplete: it has "
            "no 'done' marker, so it cannot be loaded or backed up."
        )
    backup_root = logdir / SOURCE_CHECKPOINT_BACKUP_DIR
    backup_root.mkdir(parents=True, exist_ok=True)
    source_key = hashlib.sha256(str(source.resolve()).encode()).hexdigest()[:16]
    backup_name = f"{source.name}-{source_key}"
    staging_prefix = f".{backup_name}.partial-"
    for stale in backup_root.iterdir():
        if stale.name.startswith(staging_prefix):
            logging.warning(f"Removing interrupted source checkpoint copy {stale}")
            if stale.is_dir() and not stale.is_symlink():
                shutil.rmtree(stale)
            else:
                stale.unlink()
    existing = backup_root / backup_name
    if _is_complete_checkpoint(existing):
        logging.info(
            f"Reusing the existing complete backup {existing} of the source "
            f"checkpoint {source}"
        )
        return existing
    destination = _free_child(backup_root, backup_name)
    staging = Path(tempfile.mkdtemp(prefix=staging_prefix, dir=backup_root))
    published = False
    try:
        shutil.copytree(source, staging, dirs_exist_ok=True)
        os.replace(staging, destination)
        published = True
    finally:
        if not published:
            shutil.rmtree(staging, ignore_errors=True)
    return destination


def _repair_legacy_checkpoint_dir(logdir: Path, *, has_source: bool) -> None:
    """Undo what the old `ckpt/backup` layout left in a run folder.

    `ckpt/backup` would hold one of the `keep` retention slots forever, so it
    moves to the backup folder. Generation folders without a `done` marker
    (interrupted saves) would hold slots too, so they are removed. The old
    retention also deleted the generation `ckpt/latest` named;
    `elements.Checkpoint.load_or_save` would then fail on that marker, so it is
    pointed at the newest complete generation. When none remains and no
    `--model` source is given, the run cannot be continued: this raises rather
    than letting the robot train from random weights.
    """
    ckpt_dir = logdir / "ckpt"
    legacy_backup = ckpt_dir / LEGACY_CHECKPOINT_BACKUP_DIR
    if legacy_backup.exists() or legacy_backup.is_symlink():
        backup_root = logdir / SOURCE_CHECKPOINT_BACKUP_DIR
        backup_root.mkdir(parents=True, exist_ok=True)
        destination = _free_child(backup_root, LEGACY_CHECKPOINT_BACKUP_DIR)
        os.replace(legacy_backup, destination)
        logging.warning(
            f"Moved legacy source checkpoint backup {legacy_backup} to "
            f"{destination} so it no longer takes a checkpoint retention slot"
        )

    if not ckpt_dir.is_dir():
        return
    for entry in sorted(ckpt_dir.iterdir()):
        if entry.name == "latest" or _is_complete_checkpoint(entry):
            continue
        if entry.is_dir() and not entry.is_symlink():
            logging.warning(
                f"Removing incomplete checkpoint {entry} (no 'done' marker) so it "
                "does not take a checkpoint retention slot"
            )
            shutil.rmtree(entry)

    latest = ckpt_dir / "latest"
    if not latest.is_file():
        return
    name = latest.read_text().strip("\n")
    if name and _is_complete_checkpoint(ckpt_dir / name):
        return
    complete = sorted(
        entry.name
        for entry in ckpt_dir.iterdir()
        if entry.name != "latest" and _is_complete_checkpoint(entry)
    )
    if complete:
        latest.write_text(complete[-1])
        logging.warning(
            f"{latest} named missing checkpoint {name!r}; repointed it at the "
            f"newest complete checkpoint {complete[-1]!r}"
        )
        return
    if has_source:
        # The `--model` source provides the weights; the marker is just stale.
        latest.unlink()
        logging.warning(
            f"{latest} named missing checkpoint {name!r} and no complete "
            "checkpoint remains; removed it, training continues from the --model "
            "source checkpoint"
        )
        return
    backup_root = logdir / SOURCE_CHECKPOINT_BACKUP_DIR
    backups = (
        sorted(
            str(entry)
            for entry in backup_root.iterdir()
            if _is_complete_checkpoint(entry)
        )
        if backup_root.is_dir()
        else []
    )
    if backups:
        remedy = "Resume from the source weights with `--model " + (
            "` or `--model ".join(backups) + "`, or start a new run."
        )
    else:
        remedy = "No source checkpoint backup exists either; start a new run."
    raise RuntimeError(
        f"Cannot continue Dreamer real-robot training in {logdir}: {latest} names "
        f"missing checkpoint {name!r} and ckpt/ holds no complete checkpoint. "
        f"Refusing to drive the hardware from untrained weights. {remedy}"
    )


def train_real(
    make_agent,
    make_replay,
    make_env,
    make_stream,
    make_logger,
    args,
    sim_app: SimulationApp = None,
):
    # Runs written before the backup moved out of `ckpt/` are repaired first,
    # before anything is built and long before the hardware env exists.
    _repair_legacy_checkpoint_dir(
        Path(str(args.logdir)), has_source=bool(args.from_checkpoint)
    )

    source_backup_path = None
    if args.from_checkpoint:
        with checkpoint_read_lock(args.from_checkpoint) as source_path:
            source_backup_path = _backup_source_checkpoint(
                source_path, Path(str(args.logdir))
            )
        logging.info(
            f"Created backup of the source checkpoint at: {source_backup_path}"
        )

    # 1. Instantiate TWO separate agents to avoid JAX threading race conditions
    print("[TrainReal] Instantiating Learner Agent...")
    agent_train = make_agent()
    print("[TrainReal] Instantiating Actor Agent...")
    agent_actor = make_agent()

    replay = make_replay()
    logger = make_logger()

    try:
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

        logfn = make_episode_logfn(episodes, logger, epstats)

        # 2. Checkpoint the learner. Retention sorts entries except `latest`, so a
        # `ckpt/backup` sibling could evict the newest generation. Keep backups
        # outside `ckpt/` and retain the last two generations.
        cp = elements.Checkpoint(logdir / "ckpt", keep=CHECKPOINT_GENERATIONS_KEPT)
        cp.step = step
        cp.agent = agent_train
        cp.replay = replay

        # 3. Load Checkpoint (Syncs Learner)
        if args.from_checkpoint:
            assert source_backup_path is not None
            elements.checkpoint.load(
                source_backup_path, dict(agent=bind(agent_train.load))
            )

        # Resume (or write the first generation) before the actor is synced, so the
        # policy that drives the hardware starts from the resumed weights.
        cp.load_or_save()

        # 4. Sync Actor to Learner initially
        print("[TrainReal] Syncing Actor weights from Learner...")
        agent_actor.load(agent_train.save())

        # 5. Setup Driver with ACTOR Agent
        driver = RealTimeDriver(
            env=make_env(),
            agent=agent_actor,  # <--- Driver uses separate instance
        )
        training_stop = threading.Event()
        training_failure: list[BaseException] = []
        training_thread: threading.Thread | None = None
    except BaseException as error:
        try:
            logger.close()
        except BaseException as cleanup_error:
            error.add_note(
                f"cleanup failure: {type(cleanup_error).__name__}: {cleanup_error}"
            )
        raise
    # Guard the hardware environment from the moment the driver owns it.
    try:
        driver.on_step(lambda tran, _: step.increment())
        driver.on_step(lambda tran, _: policy_fps.step())
        driver.on_step(replay.add)
        driver.on_step(logfn)

        # 6. Setup Training Loop with LEARNER Agent
        stream_train = iter(agent_train.stream(make_stream(replay, "train")))
        stream_report = iter(agent_train.stream(make_stream(replay, "report")))

        carry_train = [agent_train.init_train(args.batch_size)]
        carry_report = agent_train.init_report(args.batch_size)

        def train_worker():
            """Running in separate thread to avoid blocking actor."""
            try:
                print("[TrainReal] Training thread started.")
                while not training_stop.is_set():
                    if len(replay) < args.batch_size * args.batch_length:
                        training_stop.wait(0.1)
                        continue

                    train_steps = should_train(step)

                    if train_steps > 0:
                        for _ in range(train_steps):
                            if training_stop.is_set():
                                return
                            with elements.timer.section("stream_next"):
                                try:
                                    batch = next(stream_train)
                                except StopIteration:
                                    continue
                            # Train the LEARNER agent
                            carry_train[0], outs, mets = agent_train.train(
                                carry_train[0], batch
                            )

                            train_fps.step(batch_steps)
                            if "replay" in outs:
                                replay.update(outs["replay"])
                            train_agg.add(mets, prefix="train")

                        # Sync new weights to the ACTOR agent (via Driver)
                        # agent_train.save() returns the dict of variables
                        driver.update_variables(agent_train.save())
                    else:
                        training_stop.wait(0.01)
            except BaseException as error:
                training_failure.append(error)
                driver.record_external_failure(error)

        training_thread = threading.Thread(
            target=train_worker, daemon=False, name="TrainingThread"
        )
        training_thread.start()

        print("Start training loop")

        # Policy for Driver uses ACTOR agent
        def policy(*args):
            return agent_actor.policy(*args, mode="train")

        # Reset driver state
        driver.reset(agent_actor.init_policy)

        while step < args.steps:
            if sim_app is not None and not sim_app.is_running():
                break

            # Driver runs Actor loop in background threads
            driver(policy, steps=10)

            if should_report(step) and len(replay):
                agg = elements.Agg()
                for _ in range(args.consec_report * args.report_batches):
                    carry_report, mets = agent_train.report(
                        carry_report, next(stream_report)
                    )
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
    finally:
        from srb.integrations.dreamer.cleanup import (
            join_thread_bounded,
            run_cleanup_steps,
        )

        primary_error = sys.exc_info()[1]
        training_stop.set()
        cleanup_errors: list[BaseException] = [
            error
            for _name, error in run_cleanup_steps([("driver.close", driver.close)])
            if error is not primary_error
        ]
        join_error = join_thread_bounded(
            "TrainingThread", training_thread, timeout_s=5.0
        )
        if join_error is not None:
            cleanup_errors.append(join_error)
        cleanup_errors.extend(
            error
            for _name, error in run_cleanup_steps([("logger.close", logger.close)])
        )

        if primary_error is not None:
            for cleanup_error in cleanup_errors:
                primary_error.add_note(
                    f"cleanup failure: {type(cleanup_error).__name__}: {cleanup_error}"
                )
        elif training_failure:
            raise training_failure[0]
        elif cleanup_errors:
            raise cleanup_errors[0]
