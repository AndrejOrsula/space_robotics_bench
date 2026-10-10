import argparse
import os
from collections.abc import Mapping
from typing import TYPE_CHECKING, Any

import gymnasium
from gymnasium import spaces
from rl_zoo3.exp_manager import ExperimentManager as __ExperimentManager
from stable_baselines3.common.preprocessing import (
    is_image_space,
    is_image_space_channels_first,
)
from stable_baselines3.common.vec_env import (
    VecEnv,
    VecFrameStack,
    VecTransposeImage,
    is_vecenv_wrapped,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def experiment_args(**fields: Any) -> argparse.Namespace:
    """Build the Namespace required by rl_zoo3's ``vars(args)`` config dump.

    Values must be YAML-serialisable for the resulting ``args.yml``.
    """
    return argparse.Namespace(**fields)


def as_experiment_args(value: Any) -> Any:
    """Coerce mappings to a Namespace; leave other values unchanged."""
    return experiment_args(**value) if isinstance(value, Mapping) else value


class ExperimentManager(__ExperimentManager):
    def __init__(
        self,
        *positional,
        env: "AnyEnv | gymnasium.Env",
        tensorboard_log: str = "",
        **kwargs,
    ):
        # rl_zoo3's first positional parameter is `args`; accept it either way.
        if positional:
            positional = (as_experiment_args(positional[0]), *positional[1:])
        if "args" in kwargs:
            kwargs["args"] = as_experiment_args(kwargs["args"])
        super().__init__(*positional, tensorboard_log=tensorboard_log, **kwargs)
        self._env = env

        # rl_zoo3 maps "" to None (no TensorBoard). SB3 treats "" as a real
        # directory and writes `<ALGO>_<n>/` into the working directory.
        self.tensorboard_log = tensorboard_log or None
        self.log_path = self.log_folder
        self.save_path = os.path.join(self.log_folder, "ckpt")
        self.params_path = self.log_folder

    def create_envs(
        self, n_envs: int, eval_env: bool = False, no_log: bool = False
    ) -> VecEnv:
        env = self._env  # type: ignore[misc]
        if self.vec_env_wrapper is not None:
            env = self.vec_env_wrapper(env)

        # Normalize the env and load saved statistics when present.
        env = self._maybe_normalize(env, eval_env)

        # Optional Frame-stacking; evaluation bypasses setup_experiment, so
        # read the agent config as well.
        n_stack = self.frame_stack
        if n_stack is None:
            n_stack = (self.custom_hyperparams or {}).get("frame_stack")
        if n_stack is not None:
            env = VecFrameStack(env, n_stack)
            if self.verbose > 0:
                print(f"Stacking {n_stack} frames")

        if not is_vecenv_wrapped(env, VecTransposeImage):
            wrap_with_vectranspose = False
            if isinstance(env.observation_space, spaces.Dict):
                # Transpose if any image key is channel-last. Mixed layouts fail.
                for space in env.observation_space.spaces.values():
                    wrap_with_vectranspose = wrap_with_vectranspose or (
                        is_image_space(space)
                        and not is_image_space_channels_first(space)  # type: ignore[arg-type]
                    )
            else:
                wrap_with_vectranspose = is_image_space(
                    env.observation_space
                ) and not is_image_space_channels_first(
                    env.observation_space  # type: ignore[arg-type]
                )

            if wrap_with_vectranspose:
                if self.verbose >= 1:
                    print("Wrapping the env in a VecTransposeImage.")
                env = VecTransposeImage(env)

        return env
