from typing import Any, List, Literal, Mapping, Type

import gymnasium

from srb.utils.cfg import parse_algo_configs
from srb.utils.path import SRB_HYPERPARAMS_DIR


def register_srb_tasks(
    tasks: Mapping[
        str,
        Mapping[
            Literal["entry_point", "task_cfg", "cfg_dir"],
            gymnasium.Env | Any | str,
        ],
    ],
    *,
    default_entry_point: Type[gymnasium.Env] | None = None,
    default_task_cfg: Any | None = None,
    default_cfg_dir: str | None = SRB_HYPERPARAMS_DIR.as_posix(),
    namespace: str = "srb",
):
    for id, cfg in tasks.items():
        entry_point: gymnasium.Env = cfg.get("entry_point", default_entry_point)  # type: ignore
        gymnasium.register(
            id=f"{namespace}/{id}",
            entry_point=f"{entry_point.__module__}:{entry_point.__name__}",  # type: ignore
            kwargs={
                "task_cfg": cfg.get("task_cfg", default_task_cfg),
                **parse_algo_configs(cfg.get("cfg_dir", default_cfg_dir)),  # type: ignore
            },
            disable_env_checker=True,
        )


def get_srb_tasks(namespace: str = "srb") -> List[str]:
    return [
        env_id
        for env_id in gymnasium.registry.keys()
        if env_id.startswith(f"{namespace}/")
    ]
