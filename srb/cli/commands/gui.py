"""``srb gui`` -- Launch the native GUI."""

from __future__ import annotations

import argparse
import os
from typing import Sequence

from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter
from srb.utils.path import SRB_DIR, SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH
from srb.utils.process_supervisor import ProcessKind

_HELP_ARGS = {"--help", "-h"}

# Share the generation root published by ``srb cache update`` with the GUI;
# its cache reader uses this variable and fails closed when it is absent.
GUI_OFFLINE_CACHE_ROOT_ENV = "SRB_GUI_OFFLINE_CACHE_ROOT"

_USAGE = """\
Space Robotics Bench GUI

Usage:
  srb gui
  srb gui -- --help

Options:
  -h, --help    Print this help message and exit.
"""


class GuiCommand(BaseCommand):
    """Launch the SRB graphical user interface."""

    NAME = "gui"
    HELP = "Launch GUI"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
    ) -> argparse.ArgumentParser:
        return subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )

    @classmethod
    def execute(
        cls,
        forwarded_args: Sequence[str] = (),
        **_kwargs,
    ) -> None:
        if any(arg in _HELP_ARGS for arg in forwarded_args):
            print(_USAGE, end="")
            return

        from srb.cli._utils import run_subprocess
        from srb.cli.capabilities import require_gui_toolchain, resolve_gui_prebuilt

        os.environ[GUI_OFFLINE_CACHE_ROOT_ENV] = (
            SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH.as_posix()
        )

        prebuilt = resolve_gui_prebuilt()
        if prebuilt is not None:
            run_subprocess(
                (prebuilt.as_posix(), *forwarded_args),
                description="Launching GUI with the following command",
                kind=ProcessKind.INTERACTIVE,
                timeout=None,
            )
            return

        require_gui_toolchain()

        run_subprocess(
            (
                "cargo",
                "run",
                "--manifest-path",
                SRB_DIR.joinpath("Cargo.toml").as_posix(),
                "--package",
                "srb_gui",
                "--bin",
                "gui",
                "--",
                *forwarded_args,
            ),
            description="Launching GUI with the following command",
            kind=ProcessKind.INTERACTIVE,
            timeout=None,
        )
