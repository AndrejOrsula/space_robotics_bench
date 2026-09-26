from typing import Any, Literal, Mapping, Sequence

import gymnasium

from srb.utils import logging
from srb.utils.agent_config_catalog import agent_config_entry_points_for_task
from srb.utils.env_capability import EnvCapability
from srb.utils.importer import RecursiveImportFailure
from srb.utils.path import SRB_HYPERPARAMS_DIR

_SRB_TASK_CAPABILITIES: dict[str, EnvCapability] = {}

# Failed registration sweeps, keyed by namespace. Partial registries otherwise
# look complete and can silently omit tasks, so `get_srb_tasks` refuses to list
# them until a later successful sweep clears the failures.
_SRB_TASK_REGISTRY_FAILURES: dict[str, tuple[RecursiveImportFailure, ...]] = {}


class IncompleteTaskRegistryError(RuntimeError):
    """The namespace's registration sweep failed; task enumeration is unsafe.

    Registered tasks remain available by id; only enumeration is refused.
    """

    def __init__(
        self,
        namespace: str,
        failures: Sequence[RecursiveImportFailure],
        registered: int,
    ) -> None:
        self.namespace = namespace
        self.failures = tuple(failures)
        self.registered = registered
        super().__init__(
            _format_incomplete_registry_message(namespace, self.failures, registered)
        )


def _format_incomplete_registry_message(
    namespace: str,
    failures: Sequence[RecursiveImportFailure],
    registered: int,
    *,
    limit: int = 8,
) -> str:
    shown = failures[:limit]
    # Include the import cause; otherwise this error would hide the failure.
    details = "; ".join(
        f"{failure.module_name}: {type(failure.error).__name__}: {failure.error}"
        for failure in shown
    )
    remaining = len(failures) - len(shown)
    suffix = f"; ... {remaining} more" if remaining else ""
    package_word = "package" if len(failures) == 1 else "packages"
    return (
        f"The task registry for namespace {namespace!r} is incomplete: "
        f"{len(failures)} {package_word} failed to import during registration, "
        f"so the {registered} task(s) that did register are not the full set. "
        f"Failed imports: {details}{suffix}. "
        "Fix the failing import(s) and register again. Tasks that did register "
        "remain usable by id; only enumeration is refused, because an "
        "enumeration of an incomplete registry is silently wrong."
    )


def set_srb_task_registry_failures(
    namespace: str, failures: Sequence[RecursiveImportFailure]
) -> None:
    """Record failures, or clear them when a fresh registration sweep succeeds."""
    if failures:
        _SRB_TASK_REGISTRY_FAILURES[namespace] = tuple(failures)
    else:
        _SRB_TASK_REGISTRY_FAILURES.pop(namespace, None)


def set_srb_task_capability(env_id: str, capability: EnvCapability | str) -> None:
    _SRB_TASK_CAPABILITIES[env_id] = EnvCapability(capability)


def get_declared_srb_task_capability(env_id: str) -> EnvCapability | None:
    return _SRB_TASK_CAPABILITIES.get(env_id)


def register_srb_tasks(
    tasks: Mapping[
        str,
        Mapping[
            Literal["entry_point", "task_cfg", "cfg_dir", "capability"],
            gymnasium.Env | Any | str,
        ],
    ],
    *,
    default_entry_point: type[gymnasium.Env] | None = None,
    default_task_cfg: Any | None = None,
    default_cfg_dir: str | None = SRB_HYPERPARAMS_DIR.as_posix(),
    namespace: str = "srb",
):
    capabilities = {
        id: EnvCapability(capability)
        for id, cfg in tasks.items()
        if (capability := cfg.get("capability")) is not None
    }
    for id, cfg in tasks.items():
        entry_point: gymnasium.Env = cfg.get("entry_point", default_entry_point)  # type: ignore[misc]
        env_id = f"{namespace}/{id}"
        capability = capabilities.get(id)
        gymnasium.register(
            id=env_id,
            entry_point=f"{entry_point.__module__}:{entry_point.__name__}",  # type: ignore[misc]
            kwargs={
                "task_cfg": cfg.get("task_cfg", default_task_cfg),
                **agent_config_entry_points_for_task(
                    cfg.get("cfg_dir", default_cfg_dir),  # type: ignore[misc]
                    id,
                ),
            },
            disable_env_checker=True,
        )
        if capability is not None:
            set_srb_task_capability(env_id, capability)
        else:
            # Re-registration replaces the task definition; an omitted
            # capability clears the prior value and restores the native default.
            dropped = _SRB_TASK_CAPABILITIES.pop(env_id, None)
            if dropped is not None:
                logging.warning(
                    f"Re-registering {env_id!r} without a 'capability' cleared "
                    f"its previously declared capability {dropped.value!r}; it "
                    "now resolves to "
                    f"{EnvCapability.GYMNASIUM_NATIVE.value!r}. Pass "
                    "capability= explicitly to keep it."
                )


def _registered_srb_tasks(namespace: str) -> dict[str, str]:
    """Read the gymnasium registry without consulting the completeness flag."""
    prefix = f"{namespace}/"
    return {
        env_id.removeprefix(prefix): env_id
        for env_id in gymnasium.registry.keys()
        if env_id.startswith(prefix)
    }


def get_srb_tasks(namespace: str = "srb") -> dict[str, str]:
    """Return a mapping of short task name → full gymnasium env ID for the given namespace.

    Example: ``{"spacewalk": "srb/spacewalk", "_aerial": "srb/_aerial", ...}``

    Raises :class:`IncompleteTaskRegistryError` if the sweep that populated
    ``namespace`` could not import every task package. Every caller of this
    function treats the result as the authoritative set of tasks -- it drives
    the `srb ls --live env` table, the offline cache snapshot and the generated
    docs -- so returning a short mapping makes each of them silently wrong.
    Failing here fails the operation that actually depends on the registry, and
    names the module, exception type and exception message that caused it.
    """
    tasks = _registered_srb_tasks(namespace)
    failures = _SRB_TASK_REGISTRY_FAILURES.get(namespace)
    if failures:
        raise IncompleteTaskRegistryError(namespace, failures, len(tasks))
    return tasks
