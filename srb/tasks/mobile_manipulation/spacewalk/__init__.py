from srb.utils.registry import register_srb_tasks

from .task import Task, TaskCfg, TaskCfgEVA, TaskCfgIVA

BASE_TASK_NAME = __name__.split(".")[-1]
register_srb_tasks(
    {
        BASE_TASK_NAME: {},
        f"{BASE_TASK_NAME}_eva": {"task_cfg": TaskCfgEVA},
        f"{BASE_TASK_NAME}_iva": {"task_cfg": TaskCfgIVA},
    },
    default_entry_point=Task,
    default_task_cfg=TaskCfg,
)
