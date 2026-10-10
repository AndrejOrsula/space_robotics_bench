from __future__ import annotations

from typing import TYPE_CHECKING

from common.buffer import Buffer
from common.logger import Logger

from srb.integrations.tdmpc2.batched import BatchedOnlineTrainer, BatchedTDMPC2

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb.integrations.tdmpc2.wrapper import SrbTdmpc2Wrapper


def train(
    cfg,
    env: "SrbTdmpc2Wrapper",
    agent: BatchedTDMPC2,
    sim_app: "SimulationApp",
    start_step: int = 0,
):
    """Run batched TD-MPC2 training from ``start_step``."""
    logger = Logger(cfg)
    logger._save_agent = True
    buffer = Buffer(cfg)

    trainer = BatchedOnlineTrainer(
        cfg=cfg,
        env=env,
        agent=agent,
        buffer=buffer,
        logger=logger,
        sim_app=sim_app,
    )
    trainer._step = start_step

    print("Start training loop")
    try:
        trainer.train()
    except KeyboardInterrupt:
        # Updates run outside Kit, so Ctrl-C propagates; keep a trained model.
        if getattr(trainer, "_pretrained", False):
            logger.finish(agent)
        raise
