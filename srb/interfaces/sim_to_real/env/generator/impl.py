from pathlib import Path
from typing import TYPE_CHECKING, Dict, Iterable, Sequence, Type

import gymnasium
import gymnasium.core
from pydantic import BaseModel

from srb.interfaces.sim_to_real import RealEnv

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class EnvInfo(BaseModel):
    action_space: Dict[str, gymnasium.Space]
    observation_space: Dict[str, gymnasium.Space]

    robot: Sequence[str] | str | None
    action_rate: float


class RealEnvGenerator:
    def generate_offline(
        self,
        env: "AnyEnv" | Iterable["AnyEnv"],
        output_dir: Path | str = Path(__file__).parent.parent.resolve(),
    ):
        output_dir = Path(output_dir).resolve()
        if not isinstance(env, Iterable):
            env = (env,)

        prelude = "import gymnasium; import numpy; from srb.interfaces.sim_to_real import RealEnv"
        env_classes = []
        for e in env:
            pass

        content = f"{prelude}"

    def generate_online(self, env: "AnyEnv") -> Type[RealEnv]:
        class GeneratedRealEnv(RealEnv):
            ACTION_SPACE: Dict[str, gymnasium.Space] = ...
            OBSERVATION_SPACE: Dict[str, gymnasium.Space] = ...

            ROBOT: Sequence[str] | str | None = ...
            ACTION_RATE: float = ...

        return GeneratedRealEnv

    def _extract_env_info(self, env: "AnyEnv") -> EnvInfo:
        return EnvInfo()
