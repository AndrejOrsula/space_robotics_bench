from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_demo import DemoTask, DemoTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_demo": {
            "entry_point": DemoTask,
            "task_cfg": DemoTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
