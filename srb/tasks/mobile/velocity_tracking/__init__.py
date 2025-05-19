from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_locomotion import LocomotionTask, LocomotionTaskCfg
from .task_visual import (
    VisualLocomotionTask,
    VisualLocomotionTaskCfg,
    VisualTask,
    VisualTaskCfg,
)

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_visual": {
            "entry_point": VisualTask,
            "task_cfg": VisualTaskCfg,
        },
        f"locomotion_{BASE_TASK_NAME}": {
            "entry_point": LocomotionTask,
            "task_cfg": LocomotionTaskCfg,
        },
        f"locomotion_{BASE_TASK_NAME}_visual": {
            "entry_point": VisualLocomotionTask,
            "task_cfg": VisualLocomotionTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
