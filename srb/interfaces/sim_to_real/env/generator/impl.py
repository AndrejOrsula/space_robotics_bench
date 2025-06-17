from pathlib import Path
from typing import TYPE_CHECKING, Sequence, Type

import gymnasium
import numpy
import torch
from pydantic import BaseModel

from srb.interfaces.sim_to_real import RealEnv

if TYPE_CHECKING:
    from srb._typing import AnyEnv


class EnvInfo(BaseModel, arbitrary_types_allowed=True):
    action_space: gymnasium.spaces.Dict
    observation_space: gymnasium.spaces.Dict

    robot: str | None
    action_rate: float


# TODO: Extract action from robot's action config to get the name, scale and such
# TODO: Incorporate action scaling to the hardware somehow


class RealEnvGenerator:
    def generate_offline(
        self,
        env: "AnyEnv",
        output_dir: Path | str = Path(__file__).parent.parent.resolve(),
    ):
        output_dir = Path(output_dir).resolve()

        imports = (
            "import gymnasium",
            "import numpy",
            "from srb.interfaces.sim_to_real import RealEnv",
        )

        env_info = self._extract_env_info(env)
        class_def = f"""
class Env(RealEnv):
    ACTION_SPACE: gymnasium.spaces.Dict = {self._format_space(env_info.action_space)}
    OBSERVATION_SPACE: gymnasium.spaces.Dict = {self._format_space(env_info.observation_space)}
    ACTION_RATE: float = {env_info.action_rate}
    ACTION_SCALE: Dict[str, float] = {{}}
    ROBOT: str | None = {repr(env_info.robot)}
"""

        content = "\n".join(imports) + class_def

        # Write to file
        output_file = output_dir / "generated_real_envs.py"
        output_dir.mkdir(parents=True, exist_ok=True)
        with open(output_file, "w") as f:
            f.write(content)

        return output_file

    def generate_online(self, env: "AnyEnv") -> Type[RealEnv]:
        env_info = self._extract_env_info(env)

        class GeneratedRealEnv(RealEnv):
            ACTION_SPACE: gymnasium.Space = env_info.action_space
            OBSERVATION_SPACE: gymnasium.Space = env_info.observation_space

            ROBOT: Sequence[str] | str | None = env_info.robot
            ACTION_RATE: float = env_info.action_rate

        return GeneratedRealEnv

    def _extract_env_info(self, env: "AnyEnv") -> EnvInfo:
        # Extract action space
        action_space = None
        if hasattr(env, "single_action_space"):
            action_space = env.single_action_space

        # Extract observation space - prioritize extract_step_return if available
        observation_space = None

        # Check if environment uses step return workflow (similar to DirectEnv)
        uses_step_return = (
            hasattr(env.unwrapped, "extract_step_return")
            and hasattr(env.unwrapped.__class__, "extract_step_return")
            and env.unwrapped.__class__.extract_step_return  # type: ignore
            is not getattr(
                env.unwrapped.__class__.__bases__[0], "extract_step_return", None
            )
        )

        if uses_step_return:
            # Extract observations from step return to get the structure and actual tensor shapes
            try:
                step_return = env.unwrapped.extract_step_return()  # type: ignore

                # Build hierarchical observation spaces using actual tensor shapes
                obs_spaces = {}
                for obs_cat, obs_group in step_return.observation.items():
                    cat_spaces = {}
                    for obs_key, obs_tensor in obs_group.items():
                        # Create space directly from tensor properties with correct shape
                        cat_spaces[obs_key] = self._create_space_from_tensor(obs_tensor)

                    obs_spaces[obs_cat] = gymnasium.spaces.Dict(cat_spaces)

                observation_space = gymnasium.spaces.Dict(obs_spaces)
            except Exception:
                # Fallback to standard method if step return extraction fails
                uses_step_return = False

        if not uses_step_return:
            # Extract observation space from standard gymnasium interface
            if hasattr(env, "single_observation_space"):
                observation_space = env.single_observation_space

        # Extract robot information from environment config
        robot_cfg = None
        if hasattr(env, "cfg") and hasattr(env.cfg, "_robot"):
            robot_cfg = env.cfg._robot

        # Extract action rate from environment config
        action_rate = 1.0 / 50.0  # Default rate
        if hasattr(env, "cfg") and hasattr(env.cfg, "agent_rate"):
            action_rate = env.cfg.agent_rate

        return EnvInfo(
            action_space=action_space,  # type: ignore
            observation_space=observation_space,  # type: ignore
            robot=robot_cfg.name() if robot_cfg else None,
            action_rate=action_rate,
        )

    def _create_space_from_tensor(self, obs_tensor: torch.Tensor) -> gymnasium.Space:
        """Create a gymnasium space from a tensor."""
        if hasattr(obs_tensor, "shape") and hasattr(obs_tensor, "dtype"):
            if obs_tensor.dtype == torch.float32:
                dtype = numpy.float32
                low = -numpy.inf
                high = numpy.inf
            elif obs_tensor.dtype == torch.float64:
                dtype = numpy.float64
                low = -numpy.inf
                high = numpy.inf
            elif obs_tensor.dtype == torch.int32:
                dtype = numpy.int32
                low = numpy.iinfo(numpy.int32).min
                high = numpy.iinfo(numpy.int32).max
            elif obs_tensor.dtype == torch.int64:
                dtype = numpy.int64
                low = numpy.iinfo(numpy.int64).min
                high = numpy.iinfo(numpy.int64).max
            elif obs_tensor.dtype == torch.bool:
                dtype = numpy.bool_
                low = 0
                high = 1
            else:
                # Default fallback
                dtype = numpy.float32
                low = -numpy.inf
                high = numpy.inf

            return gymnasium.spaces.Box(
                low=low,
                high=high,
                shape=obs_tensor.shape[1:],  # Remove batch dimension
                dtype=dtype,  # type: ignore
            )
        else:
            # Fallback for unknown tensor types
            return gymnasium.spaces.Box(
                low=-numpy.inf, high=numpy.inf, shape=(1,), dtype=numpy.float32
            )

    def _format_space(self, space: gymnasium.Space) -> str:
        """Format a gymnasium space as a string for code generation."""
        if isinstance(space, gymnasium.spaces.Box):
            low_list = space.low.tolist()
            high_list = space.high.tolist()

            # Flatten lists in case of nested structures
            def flatten_list(lst):
                flattened = []
                for item in lst:
                    if isinstance(item, list):
                        flattened.extend(flatten_list(item))
                    else:
                        flattened.append(item)
                return flattened

            flat_low = flatten_list(low_list)
            flat_high = flatten_list(high_list)

            # Determine if we can use scalars instead of arrays
            if len(set(flat_low)) == 1 and len(set(flat_high)) == 1:
                # All values are the same, use scalars
                low_val = flat_low[0]
                high_val = flat_high[0]

                # Handle infinite values
                if low_val == float("-inf"):
                    low_str = "-numpy.inf"
                elif low_val == float("inf"):
                    low_str = "numpy.inf"
                else:
                    low_str = str(low_val)

                if high_val == float("-inf"):
                    high_str = "-numpy.inf"
                elif high_val == float("inf"):
                    high_str = "numpy.inf"
                else:
                    high_str = str(high_val)

                return (
                    f"gymnasium.spaces.Box("
                    f"low={low_str}, high={high_str}, "
                    f"shape={space.shape}, dtype=numpy.{space.dtype})"
                )
            else:
                # Values differ, use arrays
                # Handle infinite values in arrays
                def format_array_value(val):
                    if val == float("-inf"):
                        return "-numpy.inf"
                    elif val == float("inf"):
                        return "numpy.inf"
                    else:
                        return str(val)

                if isinstance(low_list[0], list):
                    # Multi-dimensional array
                    low_str = (
                        "numpy.array("
                        + str(low_list)
                        .replace("inf", "numpy.inf")
                        .replace("-numpy.inf", "-numpy.inf")
                        + ")"
                    )
                    high_str = (
                        "numpy.array("
                        + str(high_list)
                        .replace("inf", "numpy.inf")
                        .replace("-numpy.inf", "-numpy.inf")
                        + ")"
                    )
                else:
                    # 1D array
                    low_formatted = [format_array_value(val) for val in low_list]
                    high_formatted = [format_array_value(val) for val in high_list]
                    low_str = f"numpy.array([{', '.join(low_formatted)}])"
                    high_str = f"numpy.array([{', '.join(high_formatted)}])"

                return (
                    f"gymnasium.spaces.Box("
                    f"low={low_str}, high={high_str}, "
                    f"dtype=numpy.{space.dtype})"
                )
        elif isinstance(space, gymnasium.spaces.Discrete):
            return f"gymnasium.spaces.Discrete({space.n})"
        elif isinstance(space, gymnasium.spaces.MultiDiscrete):
            return f"gymnasium.spaces.MultiDiscrete({space.nvec.tolist()})"
        elif isinstance(space, gymnasium.spaces.MultiBinary):
            return f"gymnasium.spaces.MultiBinary({space.n})"
        elif isinstance(space, gymnasium.spaces.Dict):
            formatted_items = []
            for key, subspace in space.spaces.items():
                formatted_items.append(f'"{key}": {self._format_space(subspace)}')
            return (
                "gymnasium.spaces.Dict({\n    "
                + ",\n    ".join(formatted_items)
                + "\n})"
            )
        elif isinstance(space, gymnasium.spaces.Tuple):
            formatted_items = []
            for subspace in space.spaces:
                formatted_items.append(self._format_space(subspace))
            return f"gymnasium.spaces.Tuple(({', '.join(formatted_items)}))"
        else:
            # Fallback for unknown space types
            return f"# Unknown space type: {type(space).__name__}"
