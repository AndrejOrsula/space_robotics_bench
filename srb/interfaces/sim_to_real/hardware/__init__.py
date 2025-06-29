from srb.interfaces.sim_to_real.core.hardware import HardwareInterfaceRegistry
from srb.utils import logging
from srb.utils.importer import import_recursively

import_recursively(__name__, ignorelist=("_template",), modules_only=False)
logging.debug(
    f'Recursively imported Space Robotics Bench module "{__name__}" ({len(HardwareInterfaceRegistry.registry)} registered hardware interfaces)'
)
