from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from types import MappingProxyType
from typing import TYPE_CHECKING, Any

import rl_zoo3
from sbx import DDPG, DQN, PPO, SAC, TD3, TQC, CrossQ

from srb.integrations.sb3 import main as sb3
from srb.integrations.sb3.policy import _is_dict_observation_space
from srb.wrappers import MergeDictObsWrapper

if TYPE_CHECKING:
    from srb._typing import AnyEnv


#: The JAX implementations this integration substitutes for SB3's PyTorch ones.
SBX_ALGOS: Mapping[str, Any] = MappingProxyType(
    {
        "crossq": CrossQ,
        "ddpg": DDPG,
        "dqn": DQN,
        "ppo": PPO,
        "sac": SAC,
        "td3": TD3,
        "tqc": TQC,
    }
)

_ABSENT = object()


@contextmanager
def sbx_algorithms() -> Iterator[None]:
    """Temporarily install SBX algorithms; concurrent SB3/SBX runs are unsupported."""

    registry = rl_zoo3.ALGOS
    previous = {name: registry.get(name, _ABSENT) for name in SBX_ALGOS}
    registry.update(SBX_ALGOS)
    try:
        yield
    finally:
        for name, algo_cls in previous.items():
            if algo_cls is _ABSENT:
                registry.pop(name, None)
            else:
                registry[name] = algo_cls


def run(env: "AnyEnv", **kwargs):
    run_metadata = kwargs.pop("run_metadata", None)
    # SBX lacks MultiInputPolicy; merge Dict observations into one Box.
    # Box observations already fit, matching SBX policy loading.
    if _is_dict_observation_space(env):
        env = MergeDictObsWrapper(env)  # type: ignore[misc]
    with sbx_algorithms():
        return sb3.run(env=env, run_metadata=run_metadata, **kwargs)
