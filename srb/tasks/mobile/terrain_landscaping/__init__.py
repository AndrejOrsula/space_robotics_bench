from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_crater import Task as CraterTask
from .task_crater import TaskCfg as CraterTaskCfg
from .task_mound import Task as MoundTask
from .task_mound import TaskCfg as MoundTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_crater": {
            "entry_point": CraterTask,
            "task_cfg": CraterTaskCfg,
        },
        f"{BASE_TASK_NAME}_mound": {
            "entry_point": MoundTask,
            "task_cfg": MoundTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
