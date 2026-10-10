from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import ClassVar

from srb.cli._base import BaseCommand

_SETUP_SCRIPT = Path(__file__).parents[1] / "setup_cli.bash"


def hydra_free_flags() -> list[str]:
    """Options of Hydra-enabled commands that leave the next word free.

    Shell scripts embed this list at install time: after a flag that takes no
    value, or a variadic one whose consumed overrides are forwarded to Hydra,
    the next word may start an ``env.``/``agent.`` override.
    """
    from srb.cli import _command_parser
    from srb.cli.registry import get_command_spec

    flags: set[str] = set()

    def visit(parser: argparse.ArgumentParser) -> None:
        for action in parser._actions:
            if isinstance(action, argparse._SubParsersAction):
                for subparser in action.choices.values():
                    visit(subparser)
            elif action.nargs in (0, "*", "+"):
                flags.update(action.option_strings)

    for name in ("agent", "real_agent"):
        spec = get_command_spec(name)
        assert spec is not None
        visit(_command_parser(spec))
    return sorted(flags)


class CompletionsCommand(BaseCommand):
    """Manage shell completions for the SRB CLI."""

    NAME: ClassVar[str] = "completions"
    HELP: ClassVar[str] = "Manage shell completions (generate cache, install scripts)"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
        **kwargs,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
        )
        sub = parser.add_subparsers(dest="completions_subcommand")
        sub.add_parser(
            "generate",
            help="Regenerate completion cache (requires Isaac Sim)",
        )
        sub.add_parser(
            "install",
            help="Install shell completion scripts for Fish/Bash/Zsh",
        )
        return parser

    @classmethod
    def execute(cls, **kwargs) -> None:
        subcommand = kwargs.get("completions_subcommand")
        if subcommand == "generate":
            cls._generate()
        elif subcommand == "install":
            cls._install()
        else:
            print("Usage: srb completions {generate,install}", file=sys.stderr)
            raise SystemExit(2)

    @classmethod
    def _generate(cls) -> None:
        from srb.cli.commands.cache import _update_cache_with_app

        _update_cache_with_app()
        print("Completion cache regenerated.")

    @classmethod
    def _install(cls) -> None:
        import subprocess

        from srb.utils.process_supervisor import (
            ProcessCleanupError,
            ProcessKind,
            ProcessStatus,
            run_owned,
        )

        if not _SETUP_SCRIPT.exists():
            raise FileNotFoundError(
                f"Packaged completion setup script not found: {_SETUP_SCRIPT}"
            )
        # Default to this interpreter, which can import srb and argcomplete.
        env = {"ISAACSIM_PYTHON": sys.executable, **os.environ}
        result = run_owned(
            ["bash", str(_SETUP_SCRIPT), "--completions-only"],
            kind=ProcessKind.FINITE,
            timeout=60.0,
            env=env,
        )
        if not result.cleanup_complete:
            raise ProcessCleanupError(result)
        if result.status is ProcessStatus.TIMED_OUT:
            raise subprocess.TimeoutExpired(result.args, 60.0)
        if result.returncode:
            # The setup script already printed its diagnostics.
            print(
                f"srb completions install: setup failed (exit {result.returncode})",
                file=sys.stderr,
            )
            raise SystemExit(1)
        print("Shell completions installed.")
