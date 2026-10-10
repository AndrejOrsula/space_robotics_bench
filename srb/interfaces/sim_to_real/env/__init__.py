"""Generated ``srb_real/*`` environments.

Import is safe: this module does *not* register environments on import,
the same contract ``srb/tasks/__init__.py`` states for the simulated tasks.
Registering at module scope would make a caller that reloads the package --
or clears the gymnasium registry between two environments in one
interpreter, as the sim-to-real validation path does -- hit gymnasium's
duplicate-id error.

Call :func:`register_all` to register them. It is idempotent.
"""

from __future__ import annotations

import importlib
import pkgutil
import sys
from dataclasses import dataclass

import gymnasium

from srb.utils import logging
from srb.utils.importer import RecursiveImportFailure, import_recursively
from srb.utils.registry import get_srb_tasks, set_srb_task_registry_failures

REAL_ENV_NAMESPACE = "srb_real"


@dataclass(frozen=True, slots=True)
class RealEnvRegistrationReport:
    namespace: str
    before: int
    after: int
    failures: tuple[RecursiveImportFailure, ...] = ()


def register_all(*, namespace: str = REAL_ENV_NAMESPACE) -> RealEnvRegistrationReport:
    """Import the generated modules so their ``gymnasium.register`` calls run.

    Idempotent and self-healing: every call reconciles the registry with the
    generated modules on disk. Unimported modules are imported, and an already
    imported module whose id is missing (the registry was cleared, or only some
    modules were imported directly) is re-executed, so no call can leave the
    namespace partial without reporting a failure.
    """

    # A sweep that supersedes the previous one clears its verdict first, or a
    # fixed import could never take effect (and counting would raise).
    set_srb_task_registry_failures(namespace, ())
    before = len(get_srb_tasks(namespace))
    failures = list(import_recursively(__name__, modules_only=False))
    # The recursive importer skips modules already in ``sys.modules``; re-run
    # those whose registration is gone. Generated module names equal env names.
    for info in pkgutil.iter_modules(__path__):
        module_name = f"{__name__}.{info.name}"
        module = sys.modules.get(module_name)
        if module is None or f"{namespace}/{info.name}" in gymnasium.registry:
            continue
        try:
            importlib.reload(module)
        except Exception as e:  # noqa: BLE001 - recorded like importer failures
            failures.append(RecursiveImportFailure(module_name, e))
    after = len(get_srb_tasks(namespace))

    # The same defect the `srb` namespace had: discarding this tuple leaves a
    # registry short by an unknown number of environments and indistinguishable
    # from a complete one. `get_srb_tasks` refuses to enumerate a flagged
    # namespace; the flag is per namespace, so a broken real environment does
    # not poison the simulated registry or vice versa.
    set_srb_task_registry_failures(namespace, tuple(failures))
    if failures:
        logging.critical(
            f"Registration of sim-to-real environments (namespace={namespace!r}) "
            f"is incomplete: {len(failures)} module(s) failed to import; "
            f"{after} environment(s) registered. Failures: "
            + "; ".join(f"{f.module_name}: {f.error}" for f in failures)
        )

    logging.debug(
        f"Registered SRB sim-to-real environments (namespace={namespace!r}, "
        f"before={before}, after={after})"
    )
    return RealEnvRegistrationReport(
        namespace=namespace, before=before, after=after, failures=tuple(failures)
    )


__all__ = ["REAL_ENV_NAMESPACE", "RealEnvRegistrationReport", "register_all"]
