from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_multi import MultiTask, MultiTaskCfg
from .task_visual import VisualMultiTask, VisualMultiTaskCfg, VisualTask, VisualTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_visual": {
            "entry_point": VisualTask,
            "task_cfg": VisualTaskCfg,
        },
        f"multi_{BASE_TASK_NAME}": {
            "entry_point": MultiTask,
            "task_cfg": MultiTaskCfg,
        },
        f"multi_{BASE_TASK_NAME}_visual": {
            "entry_point": VisualMultiTask,
            "task_cfg": VisualMultiTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
