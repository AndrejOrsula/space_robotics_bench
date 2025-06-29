from srb.utils import logging
from srb.utils.importer import import_recursively
from srb.utils.registry import get_srb_tasks

import_recursively(__name__, modules_only=False)
logging.debug(
    f'Recursively imported Space Robotics Bench module "{__name__}" ({len(get_srb_tasks("srb_real"))} registered sim-to-real tasks)'
)
