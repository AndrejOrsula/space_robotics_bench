from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg
from .task_locomotion import LocomotionTask, LocomotionTaskCfg

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"locomotion_{BASE_TASK_NAME}": {
            "entry_point": LocomotionTask,
            "task_cfg": LocomotionTaskCfg,
        },
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
