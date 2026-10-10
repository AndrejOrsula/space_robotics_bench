"""``srb isaac`` -- Isaac Sim utilities (launch GUI, run Python interpreter, …)."""

from __future__ import annotations

import argparse
from typing import Sequence

from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter
from srb.utils.process_supervisor import ProcessKind, ProcessSpawnError


class IsaacCommand(BaseCommand):
    """Launch Isaac Sim or run its Python interpreter."""

    NAME = "isaac"
    ALIASES = ("isaacsim",)
    HELP = "Isaac Sim utilities"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )

        isaac_subs = parser.add_subparsers(
            title="Isaac Sim actions",
            dest="isaac_action",
            required=not is_alias,
        )
        if is_alias:
            parser.set_defaults(isaac_action="sim")

        isaac_subs.add_parser(
            "sim",
            help="Launch the Isaac Sim GUI application",
            formatter_class=SrbHelpFormatter,
        )

        isaac_subs.add_parser(
            "python",
            help="Run the Isaac Sim Python interpreter",
            formatter_class=SrbHelpFormatter,
        )

        return parser

    @classmethod
    def execute(
        cls,
        isaac_action: str,
        forwarded_args: Sequence[str] = (),
        **_kwargs,
    ) -> None:
        match isaac_action:
            case "sim":
                cls._execute_sim(forwarded_args)
            case "python":
                cls._execute_python(forwarded_args)
            case _:
                raise ValueError(f"Unknown isaac action: {isaac_action}")

    @classmethod
    def _execute_sim(cls, forwarded_args: Sequence[str] = ()) -> None:
        from srb.cli._utils import run_subprocess
        from srb.cli.capabilities import resolve_isaac_sim_launcher

        app_path = resolve_isaac_sim_launcher()
        try:
            run_subprocess(
                (app_path, *forwarded_args),
                description="Launching Isaac Sim with the following command",
                kind=ProcessKind.INTERACTIVE,
                timeout=None,
            )
        except ProcessSpawnError as exc:
            from srb.utils.failures import OptionalDependencyUnavailableError

            install_hint = (
                "a supported Isaac Sim build, set ISAACSIM_PATH, or put "
                "the isaacsim launcher on PATH"
            )
            raise OptionalDependencyUnavailableError(
                code="cli.isaac.launcher_unavailable",
                message=(
                    "The optional feature 'Isaac Sim launcher' is unavailable: "
                    f"the validated launcher could not be started ({exc.error}). "
                    f"Install {install_hint}."
                ),
                component="srb.cli.isaac.sim",
                details={
                    "feature": "Isaac Sim launcher",
                    "executable": app_path,
                    "reason": "process spawn failed after validation",
                    "exception_type": type(exc.error).__name__,
                    "exception": str(exc.error),
                    "install_hint": install_hint,
                },
            ) from exc

    @classmethod
    def _execute_python(cls, forwarded_args: Sequence[str] = ()) -> None:
        from srb.cli._utils import run_subprocess
        from srb.utils.isaacsim import get_isaacsim_python

        python_path = get_isaacsim_python()
        run_subprocess(
            (python_path, *forwarded_args),
            description="Running Isaac Sim Python with the following command",
            kind=ProcessKind.INTERACTIVE,
            timeout=None,
        )
