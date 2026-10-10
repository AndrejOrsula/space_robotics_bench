"""CLI adapter for baseline simulation agents."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaacsim.simulation_app import SimulationApp

    from srb._typing import AnyEnv


def random_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    **kwargs,
) -> None:
    import torch

    from srb.utils import logging

    with torch.inference_mode():
        while sim_app.is_running():
            action = torch.from_numpy(env.action_space.sample()).to(
                device=env.unwrapped.device  # type: ignore[misc]
            )
            (
                observation,
                reward,
                terminated,
                truncated,
                info,
            ) = env.step(action)  # type: ignore[misc]
            if logging.logger.isEnabledFor(logging.TRACE):
                logging.trace(
                    f"action: {action}\n"
                    f"observation: {observation}\n"
                    f"reward: {reward}\n"
                    f"terminated: {terminated}\n"
                    f"truncated: {truncated}\n"
                    f"info: {info}\n"
                )


def zero_agent(
    env: "AnyEnv",
    sim_app: "SimulationApp",
    **kwargs,
) -> None:
    import torch

    from srb.utils import logging

    action = torch.zeros(
        env.action_space.shape,
        device=env.unwrapped.device,  # type: ignore[misc]
    )
    with torch.inference_mode():
        while sim_app.is_running():
            observation, reward, terminated, truncated, info = env.step(action)
            if logging.logger.isEnabledFor(logging.TRACE):
                logging.trace(
                    f"action: {action}\n"
                    f"observation: {observation}\n"
                    f"reward: {reward}\n"
                    f"terminated: {terminated}\n"
                    f"truncated: {truncated}\n"
                    f"info: {info}\n"
                )


__all__ = (
    "random_agent",
    "zero_agent",
)
