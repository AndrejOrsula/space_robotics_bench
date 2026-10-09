"""Register tasks on demand after checking Isaac Sim readiness."""

from __future__ import annotations

from dataclasses import dataclass

from srb.utils import logging
from srb.utils.importer import RecursiveImportFailure, import_recursively
from srb.utils.isaacsim import is_isaacsim_initialized
from srb.utils.registry import get_srb_tasks, set_srb_task_registry_failures


@dataclass(frozen=True, slots=True)
class TaskRegistrationReport:
    namespace: str
    before: int
    after: int
    failures: tuple[RecursiveImportFailure, ...] = ()


def register_all() -> TaskRegistrationReport:
    """Register SRB tasks; raise if Isaac Sim is not initialized.

    Task packages register into the default ``srb`` namespace only.
    """

    if not is_isaacsim_initialized():
        raise RuntimeError(
            "Cannot register Space Robotics Bench tasks because Isaac Sim is not initialized. "
            "Start the Omniverse simulation app before calling srb.tasks.register_all()."
        )

    namespace = "srb"
    # Clear stale failures before this sweep re-evaluates the registry.
    set_srb_task_registry_failures(namespace, ())

    before = len(get_srb_tasks(namespace))
    failures = import_recursively("srb.tasks")
    after = len(get_srb_tasks(namespace))

    # Preserve import failures so callers cannot mistake a partial registry
    # for a complete one. get_srb_tasks raises only when a caller uses it.
    set_srb_task_registry_failures(namespace, failures)

    if failures:
        logging.critical(
            f"Task registration for namespace {namespace!r} is incomplete: "
            f"{len(failures)} package(s) failed to import; {after} task(s) "
            "registered. Enumerating SRB tasks will now fail rather than "
            "return a silently short list."
        )

    logging.debug(
        f"Registered SRB tasks via recursive import (namespace={namespace!r}, before={before}, after={after})"
    )

    return TaskRegistrationReport(
        namespace=namespace, before=before, after=after, failures=failures
    )
