from srb.tasks.mobile._orbital.task import Task as OrbitalTask
from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_orbital import TaskCfg as OrbitalTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"orbital_{BASE_TASK_NAME}": {
            "entry_point": OrbitalTask,
            "task_cfg": OrbitalTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
