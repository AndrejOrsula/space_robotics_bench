import os
from collections import OrderedDict
from pprint import pprint
from typing import TYPE_CHECKING, Any, Dict, Tuple

import gymnasium
import yaml
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


class ExperimentManager(__ExperimentManager):
    def __init__(
        self, *args, env: "AnyEnv | gymnasium.Env", tensorboard_log: str = "", **kwargs
    ):
        super().__init__(*args, tensorboard_log=tensorboard_log, **kwargs)
        self._env = env

        self.tensorboard_log = tensorboard_log
        self.log_path = self.log_folder
        self.save_path = os.path.join(self.log_folder, "ckpt")
        self.params_path = self.log_folder

    def read_hyperparameters(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        hyperparams = self.custom_hyperparams
        saved_hyperparams = OrderedDict(
            [(key, hyperparams[key]) for key in sorted(hyperparams.keys())]  # type: ignore
        )
        print(
            "Default hyperparameters for environment (ones being tuned will be overridden):"
        )
        pprint(saved_hyperparams)
        return hyperparams, saved_hyperparams  # type: ignore

    def _save_config(self, saved_hyperparams: Dict[str, Any]) -> None:
        # Save hyperparams
        with open(os.path.join(self.params_path, "config.yml"), "w") as f:
            yaml.dump(saved_hyperparams, f)

        print(f"Log path: {self.save_path}")

    def create_envs(
        self, n_envs: int, eval_env: bool = False, no_log: bool = False
    ) -> VecEnv:
        # Special case for GoalEnvs: log success rate too
        if (
            # env.is_goal_env  # TODO[low]: Add goal env handling for SB3/SBX
            False and len(self.monitor_kwargs) == 0
        ):
            self.monitor_kwargs = dict(info_keywords=("is_success",))

        # Wrap the env into a VecNormalize wrapper if needed
        # and load saved statistics when present
        env = self._maybe_normalize(
            self._env,  # type: ignore
            eval_env,
        )

        # Optional Frame-stacking
        if self.frame_stack is not None:
            n_stack = self.frame_stack
            env = VecFrameStack(env, n_stack)
            if self.verbose > 0:
                print(f"Stacking {n_stack} frames")

        if not is_vecenv_wrapped(env, VecTransposeImage):
            wrap_with_vectranspose = False
            if isinstance(env.observation_space, spaces.Dict):
                # If even one of the keys is an image-space in need of transpose, apply transpose
                # If the image spaces are not consistent (for instance, one is channel first,
                # the other channel last); VecTransposeImage will throw an error
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
