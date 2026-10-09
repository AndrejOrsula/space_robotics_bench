from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_multi import MultiTask, MultiTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"multi_{BASE_TASK_NAME}": {
            "entry_point": MultiTask,
            "task_cfg": MultiTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
