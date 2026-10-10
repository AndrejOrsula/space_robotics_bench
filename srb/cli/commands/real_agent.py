"""``srb real_agent`` -- Sim-to-real agent workflows.

Subcommands: gen, sim2real_gen, zero, rand, teleop, ros, train, eval, collect,
validate, rebuild-summary, replay-twin.
"""

from __future__ import annotations

import argparse
import sys
from importlib import import_module
from typing import Sequence

from srb.cli._args import (
    add_algo_args,
    add_collection_args,
    add_config_args,
    add_env_args,
    add_hardware_args,
    add_logging_args,
    add_teleop_args,
    add_train_args,
)
from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter

_LAZY_COMPAT_ATTRS = {
    "_build_validation_spec_overrides": (
        "srb.cli.commands.real_agent_validation",
        "build_validation_spec_overrides",
    ),
    "_infer_validation_checkpoint_artifact_algo": (
        "srb.cli.commands.real_agent_validation",
        "infer_validation_checkpoint_artifact_algo",
    ),
    "_read_validation_spec_algo": (
        "srb.cli.commands.real_agent_validation",
        "read_validation_spec_algo",
    ),
    "_build_real_agent_metadata_cli_kwargs": (
        "srb.cli.commands.real_agent_runtime",
        "_build_real_agent_metadata_cli_kwargs",
    ),
    "_dispatch_real_agent_workflow": (
        "srb.cli.commands.real_agent_runtime",
        "_dispatch_real_agent_workflow",
    ),
    "_resolve_real_agent_policy_inputs": (
        "srb.cli.commands.real_agent_runtime",
        "_resolve_real_agent_policy_inputs",
    ),
    "_run_real_agent_with_env": (
        "srb.cli.commands.real_agent_runtime",
        "_run_real_agent_with_env",
    ),
    "_generate_real_agent": (
        "srb.cli.commands.real_agent_generation",
        "_generate_real_agent",
    ),
    "_generate_real_agent_subprocess": (
        "srb.cli.commands.real_agent_generation",
        "_generate_real_agent_subprocess",
    ),
}


def __getattr__(name: str):
    """Preserve adapter exports without importing runtime modules at parse time."""
    try:
        module_name, attr_name = _LAZY_COMPAT_ATTRS[name]
    except KeyError as exc:
        raise AttributeError(name) from exc
    value = getattr(import_module(module_name), attr_name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted({*globals(), *_LAZY_COMPAT_ATTRS})


###############################################################################
# Command class
###############################################################################


class RealAgentCommand(BaseCommand):
    """Run a sim-to-real agent workflow."""

    NAME = "real_agent"
    HELP = "Run sim-to-real agent workflows"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
        env_choices: Sequence[str] | None = None,
        interface_choices: Sequence[str] = (),
        teleop_device_choices: Sequence[str] = (),
        algo_choices_rl: Sequence[str] = (),
        algo_choices_il: Sequence[str] = (),
        algo_choices_all: Sequence[str] = (),
        hardware_choices: Sequence[str] | None = None,
        real_env_choices: Sequence[str] | None = None,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )
        sub = parser.add_subparsers(
            title="Sim-to-real subcommands",
            dest="real_agent_subcommand",
            required=True,
        )

        # ---- Gen parsers ----
        gen_parsers = []
        for i, alias in enumerate(("gen", "sim2real_gen")):
            gen_p = sub.add_parser(
                alias,
                help=("Alias: gen" if i > 0 else "Generate sim-to-real setup"),
                formatter_class=SrbHelpFormatter,
            )
            add_env_args(
                gen_p,
                env_choices=env_choices,
                allow_all=True,
            )
            add_config_args(gen_p)
            add_hardware_args(gen_p, hardware_choices=hardware_choices)
            gen_p.add_argument(
                "--check",
                action="store_true",
                help=(
                    "Deterministic freshness gate: render into a temporary "
                    "path, byte-compare with the checked-in module, print a "
                    "diff, and exit non-zero on drift without overwriting."
                ),
            )
            gen_parsers.append(gen_p)

        # ---- Agent parsers ----
        zero_p = sub.add_parser(
            "zero",
            help="Real agent with zero-valued actions",
            formatter_class=SrbHelpFormatter,
        )
        rand_p = sub.add_parser(
            "rand",
            help="Real agent with random actions",
            formatter_class=SrbHelpFormatter,
        )
        teleop_p = sub.add_parser(
            "teleop",
            help="Teleoperate real agent",
            formatter_class=SrbHelpFormatter,
        )
        ros_p = sub.add_parser(
            "ros",
            help="Real agent with actions from ROS 2 | Space ROS",
            formatter_class=SrbHelpFormatter,
        )
        train_p = sub.add_parser(
            "train",
            help="Train real agent",
            formatter_class=SrbHelpFormatter,
        )
        eval_p = sub.add_parser(
            "eval",
            help="Evaluate real agent",
            formatter_class=SrbHelpFormatter,
        )
        collect_p = sub.add_parser(
            "collect",
            help="Collect demonstrations with real agent",
            formatter_class=SrbHelpFormatter,
        )

        validate_p = sub.add_parser(
            "validate",
            help="Run validation protocol on real hardware (N episodes, multi-sink telemetry, pass/fail).",
            formatter_class=SrbHelpFormatter,
        )
        validate_p.add_argument(
            "--spec",
            type=str,
            required=True,
            help="Path to validation YAML (e.g. hyperparams/validation/excavation.yaml)",
        )
        validate_p.add_argument(
            "--checkpoint",
            type=str,
            default=None,
            help="Override the spec.checkpoint field.",
        )
        validate_p.add_argument(
            "--episodes",
            type=int,
            default=None,
            help="Override spec.n_episodes.",
        )
        validate_p.add_argument(
            "--wandb",
            action="store_true",
            help="Enable optional W&B summary sink (local sinks remain active).",
        )
        validate_p.add_argument(
            "--no-tensorboard",
            action="store_true",
            help="Disable TensorBoard scalar sink (rerun + JSONL still active).",
        )
        validate_p.add_argument(
            "--no-rerun-web",
            action="store_true",
            help="Disable the rerun web viewer (still records .rrd locally).",
        )
        validate_p.add_argument(
            "--storage-root",
            type=str,
            default=None,
            help="Override spec.storage_root for validation session artifacts.",
        )
        validate_p.add_argument(
            "--post-status",
            action="store_true",
            help="Post pass/fail as GitHub commit status (requires gh CLI).",
        )
        validate_p.add_argument(
            "--dry-run",
            action="store_true",
            help="Skip env+policy creation; smoke-test the wiring only.",
        )

        rebuild_p = sub.add_parser(
            "rebuild-summary",
            help="Rebuild summary.json + report.md for a session dir from JSONL + .rrd.",
            formatter_class=SrbHelpFormatter,
        )
        rebuild_p.add_argument("session_dir", type=str)

        replay_p = sub.add_parser(
            "replay-twin",
            help="Open a rerun viewer with sim ghost + real solid time-aligned.",
            formatter_class=SrbHelpFormatter,
        )
        replay_p.add_argument(
            "--session",
            type=str,
            required=True,
            help="Validation session directory containing rollout.rrd",
        )
        replay_p.add_argument(
            "--baseline",
            type=str,
            required=True,
            help="Path to the baseline .rrd from a simulation capture.",
        )

        agent_parsers = (zero_p, rand_p, teleop_p, ros_p, train_p, eval_p, collect_p)

        # `gen` reads simulation tasks; runtime actions take `srb_real/*` ids.
        for p in agent_parsers:
            add_env_args(p, env_choices=real_env_choices, real=True)
            add_config_args(p)
            add_hardware_args(p, hardware_choices=hardware_choices)
            add_logging_args(p)

        # Real sessions accept native input only (ROS by default); simulator
        # devices such as keyboard and gamepad are refused by preflight.
        native_teleop_device_choices = tuple(
            choice
            for choice in teleop_device_choices
            if choice in ("haptic", "ros", "spacemouse")
        )
        for p in (teleop_p, collect_p):
            add_teleop_args(
                p,
                teleop_device_choices=native_teleop_device_choices,
                device_help=(
                    "One or more native input devices (ros, spacemouse, "
                    "haptic). When omitted, real sessions use ros."
                ),
            )
        add_collection_args(collect_p)

        for p in (train_p,):
            add_algo_args(
                p, choices=algo_choices_rl, required=True, include_model=False
            )
            add_train_args(p)

        for p in (eval_p,):
            add_algo_args(
                p, choices=algo_choices_all, required=False, include_model=True
            )

        # Real teleop takes direct native input only; policies drive collect.
        add_algo_args(
            collect_p,
            choices=algo_choices_all,
            required=False,
            include_model=True,
            group_name="Teleop Policy",
        )

        ros_group = ros_p.add_argument_group("ROS")
        ros_group.add_argument(
            "--ros_sync",
            "--ros-sync",
            action="store_true",
            default=False,
            help=(
                "Require a newer ROS action message for each step; otherwise "
                "reuse only a still-fresh sample."
            ),
        )
        ros_group.add_argument(
            "--ros_action_timeout",
            "--ros-action-timeout",
            type=float,
            default=None,
            help=(
                "Positive finite seconds to wait for startup input and set "
                "command lifetime. Defaults: 5 seconds startup wait and "
                "0.5 seconds command lifetime."
            ),
        )
        ros_group.add_argument(
            "--ros_unthrottled",
            "--ros-unthrottled",
            action="store_true",
            default=False,
            help=(
                "Disable ROS adapter pacing; this does not bypass RealEnv's "
                "own action rate."
            ),
        )

        return parser

    @classmethod
    def execute(
        cls,
        real_agent_subcommand: str,
        **kwargs,
    ) -> None:
        if (
            real_agent_subcommand == "eval"
            and not kwargs.get("algo")
            and not kwargs.get("model")
        ):
            raise SystemExit(
                "srb real_agent eval: error: at least one of --algo or --model is required"
            )

        match real_agent_subcommand:
            case "gen" | "sim2real_gen":
                from srb.cli.commands import real_agent_generation as _generation_cli

                _generation_cli.execute_generation_command(**kwargs)
            case "validate":
                from srb.cli.commands import real_agent_validation as _validation_cli

                sys.exit(_validation_cli.execute_validate_command(**kwargs))

            case "rebuild-summary":
                from srb.cli.commands import (
                    real_agent_validation_ops as _validation_ops_cli,
                )

                sys.exit(
                    _validation_ops_cli.execute_rebuild_summary_command(
                        session_dir=kwargs["session_dir"]
                    )
                )

            case "replay-twin":
                from srb.cli.commands import (
                    real_agent_validation_ops as _validation_ops_cli,
                )

                sys.exit(
                    _validation_ops_cli.execute_replay_twin_command(
                        session=kwargs["session"],
                        baseline=kwargs["baseline"],
                    )
                )

            case _:
                from srb.cli.commands import real_agent_runtime as _runtime_cli

                _runtime_cli.execute_runtime_command(
                    real_agent_subcommand=real_agent_subcommand, **kwargs
                )
