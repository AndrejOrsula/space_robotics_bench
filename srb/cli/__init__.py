"""Space Robotics Bench CLI bootstrap.

Root help and completion use the static command registry. A selected command
then gets its historical argparse factory and imports only that command module.
Runtime dependencies and rich traceback setup are intentionally outside this
parse/bootstrap path.
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import sys
from collections.abc import Callable, Sequence
from importlib.util import find_spec
from pathlib import Path
from typing import Any

from srb.cli._parsing import (
    _FORWARDING_COMMANDS,
    PermissiveChoiceArgumentParser,
    SrbHelpFormatter,
    SupportedILAlgo,
    SupportedRLAlgo,
    detect_unsupported_args,
    normalize_hydra_args_from_variadic_options,
    split_forwarded_args,
)
from srb.cli.registry import (
    COMMAND_SPECS,
    CommandSpec,
    get_command_spec,
    load_command,
)
from srb.utils.failures import IntegrityFailureError

_REAL_ENV_ID_PATTERN = re.compile(r'\bid="(srb_real/[A-Za-z0-9_.-]+)"')


def _root_parser() -> PermissiveChoiceArgumentParser:
    """Build the import-free root parser from static command metadata."""
    parser = PermissiveChoiceArgumentParser(
        description="Space Robotics Bench",
        formatter_class=SrbHelpFormatter,
    )
    subparsers = parser.add_subparsers(
        title="Subcommands",
        dest="subcommand",
        required=True,
    )
    for spec in COMMAND_SPECS:
        subparsers.add_parser(spec.name, help=spec.help)
        for alias in spec.aliases:
            subparsers.add_parser(alias, help=f"Alias: {spec.name}")
    return parser


def _root_command_token(argv: Sequence[str]) -> str | None:
    """Return the first root command token before the forwarding separator."""
    for token in argv:
        if token == "--":
            break
        if token.startswith("-"):
            continue
        return token
    return None


def _root_help_requested(argv: Sequence[str]) -> bool:
    """Whether argparse would handle help at the root parser."""
    command = _root_command_token(argv)
    for token in argv:
        if token == "--" or token == command:
            break
        if token in {"-h", "--help"}:
            return True
    return False


def _selected_spec(argv: Sequence[str]) -> CommandSpec | None:
    command = _root_command_token(argv)
    return get_command_spec(command)


def _read_completion_cache(reader: Callable[[], Any]) -> Any:
    """Read optional parser choices without making help depend on cache health."""
    try:
        return reader()
    except IntegrityFailureError:
        # Runtime preflight still reports the typed integrity failure. Parser
        # help/completion must remain usable enough to explain that recovery.
        return None


def _completion_parser_argv(source_argv: Sequence[str]) -> list[str]:
    """Reconstruct shell words so completion selects the nested parser.

    Argcomplete passes the editable line through ``COMP_LINE`` and
    ``COMP_POINT``, while ``sys.argv`` usually contains only ``srb``. Rebuild
    enough for parser selection; argcomplete handles final tokenisation and
    filtering.
    """
    marker = os.environ.get("_ARGCOMPLETE")
    if marker is None:
        return list(source_argv)
    try:
        if int(marker) <= 0:
            return list(source_argv)
    except ValueError:
        return list(source_argv)

    line = os.environ.get("COMP_LINE")
    point_text = os.environ.get("COMP_POINT")
    if line is None or point_text is None:
        return list(source_argv)
    try:
        point = int(point_text)
    except ValueError:
        return list(source_argv)
    point = max(0, min(point, len(line)))
    line = line[:point]
    if not line:
        return list(source_argv)

    try:
        words = shlex.split(line, posix=True)
    except ValueError:
        # An open quote is common while completing.  Parser selection only
        # needs command names, so retain a conservative whitespace fallback.
        words = line.split()
    if not words:
        return list(source_argv)

    # The generated scripts target an executable, but users may register
    # ``python -m srb`` manually.  Strip the command prefix in either form.
    source_name = Path(source_argv[0]).name if source_argv else "srb"
    start = 1
    if len(words) >= 3 and words[1] == "-m" and words[2] == "srb":
        start = 3
    elif words[0] == source_name or words[0] in {"srb", "space_robotics_bench"}:
        start = 1
    elif words[0].endswith("/srb") or words[0].endswith("/space_robotics_bench"):
        start = 1

    executable = source_argv[0] if source_argv else (words[0] if words else "srb")
    return [executable, *words[start:]]


def _command_parser(spec: CommandSpec) -> PermissiveChoiceArgumentParser:
    """Build a root parser containing only the selected command factory."""
    parser = PermissiveChoiceArgumentParser(
        description="Space Robotics Bench",
        formatter_class=SrbHelpFormatter,
    )
    subparsers = parser.add_subparsers(
        title="Subcommands",
        dest="subcommand",
        required=True,
    )
    command = load_command(spec)
    kwargs: dict[str, Any] = {}

    if spec.name in {"agent", "real_agent"}:
        # Load Isaac-free choices only for commands that use them.
        from srb.interfaces.enums import InterfaceType, TeleopDeviceType
        from srb.utils.cache import (
            read_offline_srb_env_cache,
            read_offline_srb_hardware_interface_cache,
        )

        env_choices = _read_completion_cache(read_offline_srb_env_cache)
        interface_choices = sorted(map(str, InterfaceType))
        teleop_device_choices = sorted(map(str, TeleopDeviceType))
        algo_choices_rl = sorted(map(str, SupportedRLAlgo))
        algo_choices_il = sorted(map(str, SupportedILAlgo))
        algo_choices_all = sorted(algo_choices_rl + algo_choices_il)
        kwargs.update(
            env_choices=env_choices,
            interface_choices=interface_choices,
            teleop_device_choices=teleop_device_choices,
            algo_choices_rl=algo_choices_rl,
            algo_choices_il=algo_choices_il,
            algo_choices_all=algo_choices_all,
        )
        if spec.name == "real_agent":
            kwargs["hardware_choices"] = _read_completion_cache(
                read_offline_srb_hardware_interface_cache
            )
            kwargs["real_env_choices"] = _real_env_choices()

    command.register_parser(subparsers, **kwargs)
    _enable_argcomplete(parser)
    return parser


def _real_env_choices() -> list[str]:
    """List generated ``srb_real/*`` ids without importing their modules.

    Each generated module registers its id with a literal ``id="srb_real/..."``;
    importing them would load Isaac Sim, so read the sources instead.
    """
    spec = find_spec("srb.interfaces.sim_to_real.env")
    if spec is None or not spec.submodule_search_locations:
        return []
    env_ids: set[str] = set()
    for location in spec.submodule_search_locations:
        for module in Path(location).glob("*.py"):
            try:
                source = module.read_text(encoding="utf-8")
            except OSError:
                continue
            env_ids.update(_REAL_ENV_ID_PATTERN.findall(source))
    return sorted(env_ids)


def _enable_argcomplete(parser: argparse.ArgumentParser) -> None:
    """Offer completion without installing tracing or importing commands."""
    if find_spec("argcomplete") is None:
        return
    # Ignore stray or incomplete markers so ordinary invocations remain safe.
    marker = os.environ.get("_ARGCOMPLETE")
    if marker is None:
        return
    try:
        if int(marker) <= 0:
            return
    except ValueError:
        return
    if "COMP_LINE" not in os.environ or "COMP_POINT" not in os.environ:
        return
    try:
        int(os.environ["COMP_POINT"])
    except ValueError:
        return
    import argcomplete

    argcomplete.autocomplete(parser)


def _lazy_execute(name: str, /, **kwargs: Any) -> Any:
    # Positional-only: a command option whose dest is ``name`` (for example
    # ``srb event-camera fetch-weights --name``) must reach the command.
    spec = get_command_spec(name)
    if spec is None:
        raise ValueError(f"Unknown subcommand: {name}")
    return load_command(spec).execute(**kwargs)


def _make_dispatch(name: str):
    return lambda **kwargs: _lazy_execute(name, **kwargs)


# Preserve direct handler calls without importing command methods.
_DISPATCH: dict[str, Any] = {
    name: _make_dispatch(name) for spec in COMMAND_SPECS for name in spec.names()
}


def _kit_running() -> bool:
    # A running Kit app turns ``SystemExit`` into status 0 at shutdown, so
    # failures raised after launch must exit directly.
    return "omni.kit.app" in sys.modules


def main() -> None:
    """Parse and dispatch the CLI, installing runtime presentation lazily."""
    from srb.utils.process_title import apply_process_title

    apply_process_title()
    args = parse_cli_args()
    from srb.cli.preflight import preflight_cli_namespace
    from srb.utils.failures import FailureError

    try:
        preflight_cli_namespace(vars(args))
    except FailureError as exc:
        print(
            f"[{exc.info.category.value}] {exc.info.message}",
            file=sys.stderr,
        )
        raise SystemExit(2) from None

    # Install rich tracebacks only after completion has finished.
    from srb.utils.tracing import with_rich

    with_rich()
    kwargs = vars(args)
    subcommand = kwargs.pop("subcommand")
    handler = _DISPATCH.get(subcommand)
    if handler is None:
        raise ValueError(f"Unknown subcommand: {subcommand}")
    try:
        handler(**kwargs)
    except FailureError as exc:
        print(
            f"[{exc.info.category.value}] {exc.info.message}",
            file=sys.stderr,
        )
        if _kit_running():
            from srb.cli.commands._hydra_exit import exit_now

            exit_now(2)
        raise SystemExit(2) from None
    except Exception:
        if _kit_running():
            from srb.cli.commands._hydra_exit import exit_now

            sys.excepthook(*sys.exc_info())
            exit_now(1)
        raise


def parse_cli_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Build and parse the selected command without runtime side effects.

    ``argv`` follows the existing contract and includes the executable name.
    Root help, root completion, and root syntax errors use only static command
    metadata. A selected command imports its own parser factory in stage two.
    """
    source_argv = list(sys.argv if argv is None else argv)
    parser_argv, forwarded_args = split_forwarded_args(source_argv)
    if argv is None:
        parser_argv = _completion_parser_argv(parser_argv)
    command_argv = parser_argv[1:]
    spec = _selected_spec(command_argv)

    if spec is None or _root_help_requested(command_argv):
        parser = _root_parser()
        _enable_argcomplete(parser)
        # Preserve argparse's root invocation errors.
        args, other_args = parser.parse_known_args(command_argv)
    else:
        parser = _command_parser(spec)
        args, other_args = parser.parse_known_args(command_argv)

    other_args = normalize_hydra_args_from_variadic_options(
        args,
        other_args,
        parser_argv,
    )
    args.forwarded_args = forwarded_args
    args.hydra_args = other_args

    try:
        detect_unsupported_args(other_args, args.subcommand, parser_argv)
    except ValueError as exc:
        # Match argparse and preflight failures: print usage and exit 2.
        parser.error(str(exc))
    if (
        getattr(args, "subcommand", None) == "agent"
        and getattr(args, "agent_subcommand", None) == "tune"
        and (other_args or forwarded_args)
    ):
        # No tune action passes overrides or `--` tokens to its trials; reject
        # them before preflight instead of starting a study without them.
        parser.error(
            "srb agent tune does not pass bare key=value overrides or "
            "arguments after -- to trials: "
            + " ".join([*other_args, *forwarded_args])
            + "; fix a setting for every trial with --pin KEY=VALUE"
        )
    if (
        getattr(args, "subcommand", None) == "agent"
        and getattr(args, "agent_subcommand", None) == "learn"
    ):
        unsupported = [arg for arg in other_args if not arg.startswith("agent.")]
        if unsupported or forwarded_args:
            parser.error(
                "srb agent learn accepts only agent.KEY=VALUE overrides and "
                "no arguments after --: " + " ".join([*unsupported, *forwarded_args])
            )
    if (
        forwarded_args
        and spec is not None
        and spec.name not in {*_FORWARDING_COMMANDS, "agent", "real_agent"}
    ):
        parser.error(
            f"srb {spec.name} takes no arguments after --: " + " ".join(forwarded_args)
        )

    if argv is None:
        # Hydra expects the remaining runtime arguments in ``sys.argv``. The
        # explicit argv API remains fully side-effect free.
        sys.argv = [parser_argv[0], *other_args]

    return args


__all__ = [
    "COMMAND_SPECS",
    "CommandSpec",
    "_DISPATCH",
    "main",
    "parse_cli_args",
]
