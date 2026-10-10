"""Import-free command metadata and lazy factories for the public CLI.

Keep root help and completion usable without Isaac Sim or optional command
dependencies; import a command module only when :func:`load_command` selects it.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from types import ModuleType
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from srb.cli._base import BaseCommand


@dataclass(frozen=True, slots=True)
class CommandSpec:
    """Static root-command metadata used before command import."""

    module: str
    class_name: str
    name: str
    aliases: tuple[str, ...]
    help: str

    def names(self) -> tuple[str, ...]:
        """Return the canonical command name followed by its aliases."""
        return (self.name, *self.aliases)


# This order defines the root help and dispatch order.
# Do not replace these strings with class attributes: doing so would import
# command modules during ``import srb.cli``.
COMMAND_SPECS: tuple[CommandSpec, ...] = (
    CommandSpec(
        "srb.cli.commands.agent",
        "AgentCommand",
        "agent",
        (),
        "Run simulated agent workflows",
    ),
    CommandSpec(
        "srb.cli.commands.real_agent",
        "RealAgentCommand",
        "real_agent",
        (),
        "Run sim-to-real agent workflows",
    ),
    CommandSpec(
        "srb.cli.commands.listing",
        "ListCommand",
        "list",
        ("ls",),
        "List registered assets and environments",
    ),
    CommandSpec(
        "srb.cli.commands.dataset",
        "DatasetCommand",
        "dataset",
        (),
        "Manage dataset artifacts",
    ),
    CommandSpec(
        "srb.cli.commands.model",
        "ModelCommand",
        "model",
        (),
        "Manage model artifacts",
    ),
    CommandSpec(
        "srb.cli.commands.asset",
        "AssetCommand",
        "asset",
        ("assets",),
        "Manage assets",
    ),
    CommandSpec(
        "srb.cli.commands.event_camera",
        "EventCameraCommand",
        "event-camera",
        (),
        "Manage event-camera weights, fixtures, and backends",
    ),
    CommandSpec(
        "srb.cli.commands.ephemeris",
        "EphemerisCommand",
        "ephemeris",
        (),
        "Real sun/planet geometry from NAIF SPICE kernels (optional feature)",
    ),
    CommandSpec(
        "srb.cli.commands.cache",
        "CacheCommand",
        "cache",
        (),
        "Manage generated SRB offline caches",
    ),
    CommandSpec("srb.cli.commands.gui", "GuiCommand", "gui", (), "Launch GUI"),
    CommandSpec(
        "srb.cli.commands.isaacsim",
        "IsaacCommand",
        "isaac",
        ("isaacsim",),
        "Isaac Sim utilities",
    ),
    CommandSpec(
        "srb.cli.commands.doc",
        "DocsCommand",
        "doc",
        ("docs",),
        "Build, serve, and generate documentation",
    ),
    CommandSpec("srb.cli.commands.test", "TestCommand", "test", (), "Run tests"),
    CommandSpec(
        "srb.cli.commands.clean",
        "CleanCommand",
        "clean",
        (),
        "Remove cached files from SRB, Isaac Sim, and/or SimForge",
    ),
    CommandSpec(
        "srb.cli.commands.completions",
        "CompletionsCommand",
        "completions",
        (),
        "Manage shell completions (generate cache, install scripts)",
    ),
)

# Public spelling for callers inspecting the registry.
COMMAND_REGISTRY = COMMAND_SPECS

_COMMAND_BY_NAME = {name: spec for spec in COMMAND_SPECS for name in spec.names()}


def get_command_spec(name: str | None) -> CommandSpec | None:
    """Return static metadata for a canonical name or alias."""
    return None if name is None else _COMMAND_BY_NAME.get(name)


def load_command(spec: CommandSpec | str) -> type["BaseCommand"]:
    """Import and return one selected command class.

    This is the sole command-class import seam used by the bootstrap.  The
    registry itself never imports a command module.
    """
    resolved = get_command_spec(spec) if isinstance(spec, str) else spec
    if resolved is None:
        raise KeyError(f"unknown SRB CLI command: {spec!r}")
    module: ModuleType = import_module(resolved.module)
    command = getattr(module, resolved.class_name, None)
    if not isinstance(command, type):
        raise TypeError(
            f"{resolved.module} does not export command class {resolved.class_name}"
        )
    from srb.cli._base import BaseCommand

    if not issubclass(command, BaseCommand):
        raise TypeError(
            f"{resolved.module}.{resolved.class_name} must subclass BaseCommand"
        )

    declared_name = getattr(command, "NAME", None)
    declared_aliases = getattr(command, "ALIASES", None)
    if declared_name != resolved.name:
        raise ValueError(
            f"{resolved.module}.{resolved.class_name} declares NAME={declared_name!r}; "
            f"registry requires {resolved.name!r}"
        )
    if not isinstance(declared_aliases, tuple) or not all(
        isinstance(alias, str) for alias in declared_aliases
    ):
        raise TypeError(
            f"{resolved.module}.{resolved.class_name} declares invalid ALIASES"
        )
    if declared_aliases != resolved.aliases:
        raise ValueError(
            f"{resolved.module}.{resolved.class_name} declares ALIASES={declared_aliases!r}; "
            f"registry requires {resolved.aliases!r}"
        )
    return cast(type["BaseCommand"], command)


__all__ = [
    "COMMAND_REGISTRY",
    "COMMAND_SPECS",
    "CommandSpec",
    "get_command_spec",
    "load_command",
]
