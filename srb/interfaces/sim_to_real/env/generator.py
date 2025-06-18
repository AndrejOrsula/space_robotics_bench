import shutil
import subprocess
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Iterable, List, Sequence, Tuple, Type

import gymnasium
import numpy
import torch
from pydantic import BaseModel

from srb.interfaces.sim_to_real.env.base import RealEnv

if TYPE_CHECKING:
    from srb._typing import AnyEnv, AnyEnvCfg
    from srb.core.action import ActionTermCfg


class EnvInfo(BaseModel, arbitrary_types_allowed=True):
    action_space: gymnasium.spaces.Dict
    observation_space: gymnasium.spaces.Dict

    action_rate: float
    action_scale: dict[str, float] = {}
    robot: str | None


class RealEnvGenerator:
    def generate_online(self, env: "AnyEnv") -> Type[RealEnv]:
        # Extract environment information
        env_info = self._extract_env_info(env)

        # Generate the environment class
        class Env(RealEnv):
            ACTION_SPACE: gymnasium.spaces.Dict = env_info.action_space
            OBSERVATION_SPACE: gymnasium.spaces.Dict = env_info.observation_space

            ACTION_RATE: float = env_info.action_rate
            ACTION_SCALE: Dict[str, float] = env_info.action_scale
            ROBOT: str | None = env_info.robot

        return Env

    def generate_offline(
        self,
        env: "AnyEnv",
        output: Path | str,
    ):
        output = Path(output).resolve()

        # Extract environment information
        env_info = self._extract_env_info(env)

        # Generate the content of the file
        content = f"""\
from typing import Dict

import gymnasium
import numpy

from srb.interfaces.sim_to_real import RealEnv


class Env(RealEnv):
    ACTION_SPACE: gymnasium.spaces.Dict = {self._format_space(env_info.action_space)}
    OBSERVATION_SPACE: gymnasium.spaces.Dict = {self._format_space(env_info.observation_space)}

    ACTION_RATE: float = {env_info.action_rate}
    ACTION_SCALE: Dict[str, float] = {env_info.action_scale}
    ROBOT: str | None = {repr(env_info.robot)}
"""

        # Write to file
        output.parent.mkdir(parents=True, exist_ok=True)
        with open(output, "w") as file:
            file.write(content)

        # Format the generated file
        self._format_python_file(output)

    def _extract_env_info(self, env: "AnyEnv") -> EnvInfo:
        env_cfg: "AnyEnvCfg" = env.cfg

        # Extract actions from action manager
        action_space_dict = {}
        action_scale = {}
        for action_key, action_term in env.action_manager._terms.items():
            # Create action space
            action_space_dict[action_key] = gymnasium.spaces.Box(
                low=-1.0, high=1.0, shape=(action_term.action_dim,), dtype=numpy.float32
            )

            # Extract scale from different action term types
            action_term_cfg: "ActionTermCfg" = action_term.cfg  # type: ignore
            if hasattr(action_term_cfg, "scale"):
                action_scale[action_key] = action_term_cfg.scale  # type: ignore
            if hasattr(action_term_cfg, "scale_linear"):
                action_scale[f"{action_key}_linear"] = action_term_cfg.scale_linear  # type: ignore
                action_scale[f"{action_key}_angular"] = action_term_cfg.scale_angular  # type: ignore
            if hasattr(action_term_cfg, "position_scale"):
                action_scale[f"{action_key}_position"] = action_term_cfg.position_scale  # type: ignore
                action_scale[f"{action_key}_orientation"] = (
                    action_term_cfg.orientation_scale  # type: ignore
                )
        action_space = gymnasium.spaces.Dict(action_space_dict)

        # Extract observation space from extract_step_return (if available)
        observation_space = None
        if (
            hasattr(env.unwrapped, "extract_step_return")
            and hasattr(env.unwrapped.__class__, "extract_step_return")
            and env.unwrapped.__class__.extract_step_return  # type: ignore
            is not getattr(
                env.unwrapped.__class__.__bases__[0], "extract_step_return", None
            )
        ):
            step_return = env.unwrapped.extract_step_return()  # type: ignore
            observation_space_dict = {}
            for obs_cat, obs_group in step_return.observation.items():
                cat_spaces = {}
                for obs_key, obs_value in obs_group.items():
                    cat_spaces[obs_key] = self._space_from_value(obs_value)
                observation_space_dict[obs_cat] = gymnasium.spaces.Dict(cat_spaces)
            observation_space = gymnasium.spaces.Dict(observation_space_dict)
        else:
            observation_space = env.single_observation_space

        return EnvInfo(
            action_space=action_space,
            observation_space=observation_space,
            action_rate=env_cfg.agent_rate,
            action_scale=action_scale,
            robot=env_cfg._robot.name(),
        )

    def _space_from_value(self, value: numpy.ndarray | torch.Tensor) -> gymnasium.Space:
        if isinstance(value, torch.Tensor):
            dtype = self.__torch_dtype_to_numpy_dtype(value.dtype)
        else:
            dtype = value.dtype
        low, high = self.__numpy_dtype_limits(dtype)
        return gymnasium.spaces.Box(
            low=low,
            high=high,
            shape=value.shape[1:],
            dtype=dtype,  # type: ignore
        )

    def _format_space(self, space: gymnasium.Space) -> str:
        if isinstance(space, gymnasium.spaces.Box):
            low_limit, high_limit = self.__numpy_dtype_limits(space.dtype)
            low_limit_str, high_limit_str = self.__numpy_dtype_limits_str(space.dtype)

            def __flatten(input: Iterable) -> List:
                output = []
                for item in input:
                    if isinstance(item, Iterable):
                        output.extend(__flatten(item))
                    else:
                        output.append(item)
                return output

            def __format(value: bool | int | float) -> str:
                if value == low_limit or value == -numpy.inf:
                    return low_limit_str
                elif value == high_limit or value == numpy.inf:
                    return high_limit_str
                else:
                    return str(value)

            space_low_flattened = __flatten(space.low.tolist())
            space_high_flattened = __flatten(space.high.tolist())
            if (
                len(set(space_low_flattened)) == 1
                and len(set(space_high_flattened)) == 1
            ):
                low_str = __format(space_low_flattened[0])
                high_str = __format(space_high_flattened[0])
            elif not isinstance(space.low[0], Iterable):
                low_str = f"numpy.array([{', '.join(__format(value) for value in space.low)}])"
                high_str = f"numpy.array([{', '.join(__format(value) for value in space.high)}])"
            else:
                low_str = f"numpy.array({repr(space.low)})"
                high_str = f"numpy.array({repr(space.high)})"
            return f"gymnasium.spaces.Box(low={low_str}, high={high_str}, shape={space.shape}, dtype=numpy.{space.dtype})"
        elif isinstance(space, gymnasium.spaces.Discrete):
            return f"gymnasium.spaces.Discrete({space.n})"
        elif isinstance(space, gymnasium.spaces.MultiDiscrete):
            return f"gymnasium.spaces.MultiDiscrete({space.nvec})"
        elif isinstance(space, gymnasium.spaces.MultiBinary):
            return f"gymnasium.spaces.MultiBinary({space.n})"
        elif isinstance(space, gymnasium.spaces.Dict):
            items = []
            for key, subspace in space.spaces.items():
                items.append(f'"{key}": {self._format_space(subspace)}')
            return "gymnasium.spaces.Dict({" + ", ".join(items) + "})"
        elif isinstance(space, gymnasium.spaces.Tuple):
            entries = []
            for subspace in space.spaces:
                entries.append(self._format_space(subspace))
            return f"gymnasium.spaces.Tuple(({', '.join(entries)}))"
        else:
            raise TypeError(f"Unsupported space type: {type(space).__name__}")

    __FORMAT_CMDS: Sequence[Sequence[str]] = (
        ("ruff", "format"),
        ("black",),
    )

    def _format_python_file(self, file_path: Path) -> bool:
        for cmd in self.__FORMAT_CMDS:
            if shutil.which(cmd[0]):
                try:
                    result = subprocess.run(
                        (*cmd, file_path.as_posix()),
                        capture_output=True,
                        text=True,
                        check=True,
                    )
                    if result.returncode == 0:
                        return True
                except Exception:
                    pass
        return False

    def __torch_dtype_to_numpy_dtype(self, dtype: torch.dtype) -> Any:
        match dtype:
            case torch.float32:
                return numpy.float32
            case torch.float64:
                return numpy.float64
            case torch.int8:
                return numpy.int8
            case torch.uint8:
                return numpy.uint8
            case torch.int16:
                return numpy.int16
            case torch.uint16:
                return numpy.uint16
            case torch.int32:
                return numpy.int32
            case torch.uint32:
                return numpy.uint32
            case torch.int64:
                return numpy.int64
            case torch.uint64:
                return numpy.uint64
            case torch.bool:
                return numpy.bool_
            case _:
                raise TypeError(f"Unsupported torch dtype '{dtype}'")

    def __numpy_dtype_limits(self, dtype: numpy.dtype) -> Tuple[float, float]:
        if numpy.issubdtype(dtype, numpy.floating):
            return numpy.finfo(dtype).min, numpy.finfo(dtype).max  # type: ignore
        elif numpy.issubdtype(dtype, numpy.integer):
            return numpy.iinfo(dtype).min, numpy.iinfo(dtype).max
        elif dtype == numpy.bool_:
            return 0.0, 1.0
        else:
            return -numpy.inf, numpy.inf

    def __numpy_dtype_limits_str(self, dtype: numpy.dtype) -> Tuple[str, str]:
        if numpy.issubdtype(dtype, numpy.floating):
            return f"numpy.finfo(numpy.{dtype}).min", f"numpy.finfo(numpy.{dtype}).max"
        elif numpy.issubdtype(dtype, numpy.integer):
            return f"numpy.iinfo(numpy.{dtype}).min", f"numpy.iinfo(numpy.{dtype}).max"
        elif dtype == numpy.bool_:
            return "0.0", "1.0"
        else:
            return "-numpy.inf", "numpy.inf"
