"""Custom argparse components and CLI enums."""

from __future__ import annotations

import argparse
import string
import sys
import warnings
from collections import Counter
from collections.abc import Sequence
from enum import auto

from srb.core.algorithms import StrEnum as _StrEnum
from srb.core.algorithms import SupportedILAlgo as SupportedILAlgo
from srb.core.algorithms import SupportedRLAlgo as SupportedRLAlgo

_HYDRA_VARIADIC_ATTRS = ("interface", "teleop_device", "hardware")
# Canonical names of the commands that forward tokens after `--` to a spawned
# process; aliases are resolved through `srb.cli.registry`.
_FORWARDING_COMMANDS = frozenset({"gui", "doc", "test", "isaac"})


###############################################################################
# Custom argparse classes
###############################################################################


class SrbHelpFormatter(argparse.HelpFormatter):
    """Compact help: short usage, choices listed once, meaningful defaults only.

    ``Alias: <name>`` subcommand rows fold into their target's row.
    """

    _MAX_INLINE_CHOICES = 8
    # Registry-backed choices point to their listing command instead.
    _CHOICE_LISTERS = {"env_id": ("ENV", "srb ls env")}

    def _format_usage(self, usage, actions, groups, prefix):
        if usage is None:
            parts = [self._prog]
            if any(
                a.option_strings and not a.required and a.help is not argparse.SUPPRESS
                for a in actions
            ):
                parts.append("[options]")
            for action in actions:
                if action.help is argparse.SUPPRESS:
                    continue
                if action.option_strings and action.required:
                    parts.append(self._format_option_usage(action))
                elif not action.option_strings:
                    parts.append(
                        self._format_args(
                            action, self._get_default_metavar_for_positional(action)
                        )
                    )
            usage = " ".join(parts)
        return super()._format_usage(usage, actions, groups, prefix)

    def _format_option_usage(self, action) -> str:
        option = action.option_strings[0]
        if action.nargs == 0:
            return option
        metavar = self._format_args(
            action, self._get_default_metavar_for_optional(action)
        )
        return f"{option} {metavar}"

    def _get_default_metavar_for_optional(self, action):
        if action.dest in self._CHOICE_LISTERS:
            return self._CHOICE_LISTERS[action.dest][0]
        return super()._get_default_metavar_for_optional(action)

    def _metavar_formatter(self, action, default_metavar):
        if action.metavar is None:
            if isinstance(action, argparse._SubParsersAction):
                return lambda size: ("<command>",) * size
            if self._long_choices(action):
                return lambda size: (default_metavar,) * size
        return super()._metavar_formatter(action, default_metavar)

    def _format_action_invocation(self, action):
        if not action.option_strings:
            return super()._format_action_invocation(action)
        # `--foo-bar` and `--foo_bar` spell one option; show the first spelling.
        options = []
        seen = set()
        for option in action.option_strings:
            key = option.replace("_", "-")
            if key not in seen:
                seen.add(key)
                options.append(option)
        invocation = ", ".join(options)
        if action.nargs != 0:
            invocation += " " + self._format_args(
                action, self._get_default_metavar_for_optional(action)
            )
        return invocation

    def _get_help_string(self, action):
        help_text = action.help or ""
        if action.dest in self._CHOICE_LISTERS and action.choices:
            help_text += f" [list: {self._CHOICE_LISTERS[action.dest][1]}]"
        elif self._long_choices(action):
            help_text += f" [choices: {', '.join(map(str, action.choices))}]"
        default = action.default
        if (
            "%(default)" not in help_text
            and "default" not in help_text.lower()
            and action.option_strings
            and default is not None
            and not isinstance(default, bool)
            and default not in ("", [], (), argparse.SUPPRESS)
            and action.nargs != 0
        ):
            help_text += " [default: %(default)s]"
        return help_text

    def _long_choices(self, action) -> bool:
        # Multi-value options would repeat an inline choice set; list it in help.
        return (
            action.choices is not None
            and not isinstance(action, argparse._SubParsersAction)
            and (
                len(action.choices) > self._MAX_INLINE_CHOICES
                or action.nargs in ("*", "+")
            )
        )

    def _format_action(self, action):
        if isinstance(action, argparse._SubParsersAction) and not action.help:
            # List the subcommands without a `<command>` header row.
            return "".join(
                self._format_action(sub)
                for sub in self._iter_indented_subactions(action)
            )
        return super()._format_action(action)

    def _iter_indented_subactions(self, action):
        # Materialized eagerly, so rows keep the group indentation.
        subactions = list(super()._iter_indented_subactions(action))
        aliases: dict[str, list[str]] = {}
        for sub in subactions:
            target = (sub.help or "").removeprefix("Alias: ")
            if sub.help and sub.help.startswith("Alias: "):
                aliases.setdefault(target, []).append(sub.dest)
        for sub in subactions:
            if sub.help and sub.help.startswith("Alias: "):
                continue
            if sub.dest in aliases:
                sub = argparse.Action(
                    option_strings=[],
                    dest=sub.dest,
                    help=sub.help,
                    metavar=", ".join([sub.dest, *aliases[sub.dest]]),
                )
            yield sub


class PermissiveChoiceArgumentParser(argparse.ArgumentParser):
    """Warn on unknown dynamic choices while keeping fixed enums strict.

    Enable per action with ``action.permissive_choices = True``.
    """

    def __init__(self, *args, formatter_class=SrbHelpFormatter, **kwargs) -> None:
        super().__init__(*args, formatter_class=formatter_class, **kwargs)

    def _check_value(self, action, value) -> None:
        if action.choices is not None and value not in action.choices:
            if getattr(action, "permissive_choices", False):
                if (
                    action.dest in _HYDRA_VARIADIC_ATTRS
                    and isinstance(value, str)
                    and _is_hydra_override_candidate(value)
                ):
                    return
                if (
                    isinstance(action, AutoNamespaceTaskAction)
                    and isinstance(value, str)
                    and f"{action.NAMESPACE}/{value}" in action.choices
                ):
                    return  # Short task id; the action adds the namespace.
                warnings.warn(
                    f"Choice '{value}' is not (yet) a known option from the registered set {{{', '.join(action.choices)}}}.",
                    category=UserWarning,
                    stacklevel=2,
                )
                return
        return super()._check_value(action, value)


class AutoNamespaceTaskAction(argparse.Action):
    """Automatically prepend the SRB namespace to task names."""

    NAMESPACE: str = "srb"

    def __call__(
        self,
        parser: PermissiveChoiceArgumentParser,
        namespace: argparse.Namespace,
        values: str,
        option_string: str | None = None,
    ) -> None:
        if "/" not in values:
            values = f"{self.NAMESPACE}/{values}"
        setattr(namespace, self.dest, values)


class AutoRealNamespaceTaskAction(AutoNamespaceTaskAction):
    """Automatically prepend the SRB real namespace to task names."""

    NAMESPACE: str = "srb_real"


class ExplicitAction(argparse.Action):
    """Action that sets an additional ``*_explicit`` flag when invoked."""

    def __call__(
        self,
        parser: PermissiveChoiceArgumentParser,
        namespace: argparse.Namespace,
        values: str,
        option_string: str | None = None,
    ) -> None:
        setattr(namespace, self.dest, values)
        setattr(namespace, f"{self.dest}_explicit", True)


###############################################################################
# Enums
###############################################################################


class EntityToList(_StrEnum):
    ALL = auto()
    ACTION = auto()
    ASSET = auto()
    ENV = auto()
    SCENERY = auto()
    OBJECT = auto()
    ROBOT = auto()


class CacheTarget(_StrEnum):
    ALL = auto()
    SRB = auto()
    ISAACSIM = auto()
    SIMFORGE = auto()


class Lang(_StrEnum):
    PYTHON = auto()
    RUST = auto()


###############################################################################
# Argument handling helpers
###############################################################################


def split_forwarded_args(argv: list[str]) -> tuple[list[str], list[str]]:
    """Split *argv* at ``--`` into parser and forwarded arguments.

    Without ``--``, forwarded arguments are empty.
    """
    if "--" in argv:
        idx = argv.index("--")
        return argv[:idx], argv[idx + 1 :]
    return argv, []


def _is_hydra_override_arg(arg: str) -> bool:
    if not _is_hydra_override_candidate(arg):
        return False
    key, value = arg.split("=", maxsplit=1)
    return bool(key and value)


def _is_hydra_override_candidate(arg: str) -> bool:
    """Identify override-shaped tokens, including malformed empty sides."""
    return "=" in arg and not arg.startswith("-")


def normalize_hydra_args_from_variadic_options(
    namespace: argparse.Namespace,
    other_args: list[str],
    argv: list[str],
    variadic_attrs: tuple[str, ...] = _HYDRA_VARIADIC_ATTRS,
) -> list[str]:
    """Move Hydra overrides consumed by greedy argparse options to leftovers.

    For example, ``--teleop_device keyboard env.foo=bar`` must remove the
    override from the device list and forward it. ``variadic_attrs`` names the
    list-valued attributes to inspect.
    """
    consumed_hydra_args: list[str] = []

    for attr in variadic_attrs:
        values = getattr(namespace, attr, None)
        if not isinstance(values, list):
            continue

        kept_values: list[object] = []
        for value in values:
            if isinstance(value, str) and _is_hydra_override_candidate(value):
                consumed_hydra_args.append(value)
            else:
                kept_values.append(value)
        setattr(namespace, attr, kept_values)

    if not consumed_hydra_args:
        return other_args

    hydra_args = [arg for arg in other_args if _is_hydra_override_arg(arg)]
    unsupported_args = [arg for arg in other_args if not _is_hydra_override_arg(arg)]
    valid_consumed = [arg for arg in consumed_hydra_args if _is_hydra_override_arg(arg)]
    invalid_consumed = [
        arg for arg in consumed_hydra_args if not _is_hydra_override_arg(arg)
    ]
    unsupported_args.extend(invalid_consumed)
    remaining_counts = Counter([*hydra_args, *valid_consumed])
    ordered_hydra_args: list[str] = []

    for arg in argv[1:]:
        if remaining_counts[arg] > 0:
            ordered_hydra_args.append(arg)
            remaining_counts[arg] -= 1

    return [*unsupported_args, *ordered_hydra_args]


def detect_unsupported_args(
    other_args: list[str],
    subcommand: str,
    argv: Sequence[str] | None = None,
) -> None:
    """Raise :class:`ValueError` for unsupported arguments.

    ``argv`` supplies the command line for forwarding suggestions and defaults
    to ``sys.argv``. Callers with explicit argv pass it to avoid suggestions
    quoting the host command (for example, pytest's arguments). The caller
    converts this error to a usage error; keeping ``ValueError`` makes the
    helper usable outside a parser.
    """
    unsupported_args = [arg for arg in other_args if not _is_hydra_override_arg(arg)]
    if not unsupported_args:
        return

    source_argv = list(sys.argv if argv is None else argv)

    # Resolve through the registry so the canonical name and every alias of a
    # forwarding command (`doc`/`docs`, `isaac`/`isaacsim`, `asset`/`assets`)
    # get the same hint.
    from srb.cli.registry import get_command_spec

    spec = get_command_spec(subcommand)
    offers_forwarding = spec is not None and spec.name in _FORWARDING_COMMANDS

    raise ValueError(
        f"Unsupported CLI argument{'s' if len(unsupported_args) > 1 else ''}: "
        + ", ".join(
            f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
            for arg in unsupported_args
        )
        + (
            (
                '\nUse "--" to separate arguments meant for spawned processes: '
                + " ".join(
                    f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
                    for arg in source_argv
                    if arg not in unsupported_args and arg != "--"
                )
                + " -- "
                + " ".join(
                    f'"{arg}"' if any(c in string.whitespace for c in arg) else arg
                    for arg in unsupported_args
                )
            )
            if offers_forwarding
            else ""
        )
    )
