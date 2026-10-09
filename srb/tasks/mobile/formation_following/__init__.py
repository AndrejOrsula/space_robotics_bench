from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg, TaskCfgSingle
from .task_orbital import Task as OrbitalTask
from .task_orbital import TaskCfg as OrbitalTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_orbital": {
            "entry_point": OrbitalTask,
            "task_cfg": OrbitalTaskCfg,
        },
        f"{BASE_TASK_NAME}_decentralized": {
            "entry_point": Task,
            "task_cfg": TaskCfgSingle,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
