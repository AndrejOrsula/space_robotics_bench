"""CLI parser and adapter for ``srb agent`` workflows.

Simulator imports stay deferred until after app startup.
"""

from __future__ import annotations

import argparse
from importlib import import_module
from typing import Sequence

from srb.cli._args import (
    add_algo_args,
    add_collection_args,
    add_config_args,
    add_env_args,
    add_interface_args,
    add_launcher_args,
    add_learn_args,
    add_logging_args,
    add_model_artifact_args,
    add_obs_args,
    add_teleop_args,
    add_train_args,
    add_tune_args,
)
from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter

_LAZY_COMPAT_ATTRS = {
    "_build_agent_metadata_cli_kwargs": (
        "srb.cli.commands.agent_runtime",
        "_build_agent_metadata_cli_kwargs",
    ),
    "_run_agent_with_env": ("srb.cli.commands.agent_runtime", "_run_agent_with_env"),
    "_should_auto_enable_cameras": (
        "srb.cli.commands.agent_runtime",
        "_should_auto_enable_cameras",
    ),
    "_get_hydra_bool_override": (
        "srb.cli.commands.agent_runtime",
        "_get_hydra_bool_override",
    ),
    "_resolve_teleop_devices": (
        "srb.cli.commands.agent_collection",
        "_resolve_teleop_devices",
    ),
    "random_agent": ("srb.cli.commands.agent_baseline", "random_agent"),
    "zero_agent": ("srb.cli.commands.agent_baseline", "zero_agent"),
    "ros_agent": ("srb.cli.commands.agent_ros", "ros_agent"),
    "_resolve_ros_step_period": (
        "srb.cli.commands.agent_ros",
        "_resolve_ros_step_period",
    ),
    "teleop_agent": ("srb.cli.commands.agent_teleop", "teleop_agent"),
    "_teleop_agent_direct": (
        "srb.cli.commands.agent_teleop",
        "_teleop_agent_direct",
    ),
    "_teleop_agent_via_policy": (
        "srb.cli.commands.agent_teleop",
        "_teleop_agent_via_policy",
    ),
    "train_agent": ("srb.cli.commands.agent_train", "train_agent"),
    "eval_agent": ("srb.cli.commands.agent_eval", "eval_agent"),
    "_eval_agent_policy": ("srb.cli.commands.agent_eval", "_eval_agent_policy"),
    "_coerce_policy_action_for_env": (
        "srb.cli.commands.agent_eval",
        "_coerce_policy_action_for_env",
    ),
    "_coerce_policy_action": (
        "srb.cli.commands.agent_eval",
        "_coerce_policy_action",
    ),
    "_iter_policy_mapping_action": (
        "srb.cli.commands.agent_eval",
        "_iter_policy_mapping_action",
    ),
    "_validate_policy_mapping_action_keys": (
        "srb.cli.commands.agent_eval",
        "_validate_policy_mapping_action_keys",
    ),
    "_policy_env_action_space": (
        "srb.cli.commands.agent_eval",
        "_policy_env_action_space",
    ),
    "_policy_env_action_shape": (
        "srb.cli.commands.agent_eval",
        "_policy_env_action_shape",
    ),
    "_policy_action_space_shape": (
        "srb.cli.commands.agent_eval",
        "_policy_action_space_shape",
    ),
    "_reshape_policy_action": ("srb.cli.commands.agent_eval", "_reshape_policy_action"),
    "_truthy_any": ("srb.cli.commands.agent_eval", "_truthy_any"),
    "collect_agent": ("srb.cli.commands.agent_collection", "collect_agent"),
    "_collect_agent_builtin": (
        "srb.cli.commands.agent_collection",
        "_collect_agent_builtin",
    ),
    "_collect_agent_policy": (
        "srb.cli.commands.agent_collection",
        "_collect_agent_policy",
    ),
    "_collect_agent_teleop": (
        "srb.cli.commands.agent_collection",
        "_collect_agent_teleop",
    ),
    "_dataset_env_kwargs": (
        "srb.cli.commands.agent_collection",
        "_dataset_env_kwargs",
    ),
    "_resolve_collect_source": (
        "srb.cli.commands.agent_collection",
        "_resolve_collect_source",
    ),
    "_robomimic_action_space": (
        "srb.cli.commands.agent_collection",
        "_robomimic_action_space",
    ),
    "_resolve_agent_workflow_algo": (
        "srb.cli.commands.agent_policy",
        "resolve_agent_workflow_algo",
    ),
    "_should_use_latest_policy_logdir": (
        "srb.cli.commands.agent_policy",
        "should_use_latest_policy_logdir",
    ),
    "_resolve_model_artifact_policy_inputs": (
        "srb.cli.commands.agent_policy",
        "resolve_model_artifact_policy_inputs",
    ),
    "_resolve_model_artifact_reference": (
        "srb.cli.commands.agent_policy",
        "resolve_model_artifact_reference",
    ),
    "_maybe_package_model_artifact_from_run": (
        "srb.cli.commands.agent_policy",
        "maybe_package_model_artifact_from_run",
    ),
    "_infer_model_artifact_identity_from_env": (
        "srb.cli.commands.agent_policy",
        "infer_model_artifact_identity_from_env",
    ),
    "_resolve_model_path": ("srb.cli.commands.agent_policy", "resolve_model_path"),
    "_latest_file": ("srb.cli.commands.agent_policy", "latest_file"),
    "_load_policy": ("srb.cli.commands.agent_policy", "load_policy"),
    "_load_policy_from_checkpoint": (
        "srb.cli.commands.agent_policy",
        "load_policy_from_checkpoint",
    ),
    "learn_from_demonstrations": (
        "srb.cli.commands.agent_learn",
        "learn_from_demonstrations",
    ),
}


def __getattr__(name: str):
    """Preserve historical adapter exports without eager runtime imports."""
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


class AgentCommand(BaseCommand):
    """Run an agent workflow."""

    NAME = "agent"
    HELP = "Run simulated agent workflows"

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
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )
        sub = parser.add_subparsers(
            title="Agent subcommands",
            dest="agent_subcommand",
            required=True,
        )

        # ---- Sub-subcommand parsers ----
        zero_p = sub.add_parser(
            "zero",
            help="Agent with zero-valued actions",
            formatter_class=SrbHelpFormatter,
        )
        rand_p = sub.add_parser(
            "rand",
            help="Agent with random actions",
            formatter_class=SrbHelpFormatter,
        )
        teleop_p = sub.add_parser(
            "teleop",
            help="Teleoperate agent",
            formatter_class=SrbHelpFormatter,
        )
        ros_p = sub.add_parser(
            "ros",
            help="Agent with actions from ROS 2 | Space ROS",
            formatter_class=SrbHelpFormatter,
        )
        train_p = sub.add_parser(
            "train",
            help="Train agent",
            formatter_class=SrbHelpFormatter,
        )
        eval_p = sub.add_parser(
            "eval",
            help="Evaluate agent",
            formatter_class=SrbHelpFormatter,
        )
        collect_p = sub.add_parser(
            "collect",
            help="Collect demonstrations with agent",
            formatter_class=SrbHelpFormatter,
        )
        learn_p = sub.add_parser(
            "learn",
            help="Learn from demonstrations",
            formatter_class=SrbHelpFormatter,
        )

        sim_parsers = (
            zero_p,
            rand_p,
            teleop_p,
            ros_p,
            train_p,
            eval_p,
            collect_p,
        )

        # -- Shared argument groups for simulation-based subcommands --
        for p in sim_parsers:
            add_launcher_args(p)
            add_env_args(p, env_choices=env_choices)
            add_config_args(p)
            add_interface_args(p, interface_choices=interface_choices)
            add_logging_args(p)
            p.add_argument(
                "--repl",
                nargs="?",
                const="ptpython",
                default=None,
                choices=("ptpython", "bpython"),
                help="Start an interactive Python REPL in a background thread (default backend: ptpython). Can also be enabled via the SRB_REPL environment variable",
            )

        # Offline learning needs no simulator launcher or interface args.
        add_env_args(learn_p, env_choices=env_choices)
        add_config_args(learn_p)
        add_logging_args(learn_p)

        # Teleop args
        for p in (teleop_p, collect_p):
            add_teleop_args(p, teleop_device_choices=teleop_device_choices)

        add_collection_args(collect_p)

        ros_group = ros_p.add_argument_group("ROS")
        ros_group.add_argument(
            "--ros_sync",
            "--ros-sync",
            action="store_true",
            default=False,
            help=(
                "Wait for a fresh ROS action message before each simulation step. "
                "Useful for closed-loop external controllers; by default the ROS "
                "agent samples and holds the latest action."
            ),
        )
        ros_group.add_argument(
            "--ros_action_timeout",
            "--ros-action-timeout",
            type=float,
            default=None,
            help=(
                "Maximum seconds to wait for a fresh action with --ros_sync; "
                "waits indefinitely by default"
            ),
        )
        ros_group.add_argument(
            "--ros_unthrottled",
            "--ros-unthrottled",
            action="store_true",
            default=False,
            help=(
                "Do not pace non-synchronized ROS control to the agent step "
                "period (pacing gives controllers time to publish each action)"
            ),
        )

        # Algorithm args
        for p in (train_p,):
            add_algo_args(
                p,
                choices=algo_choices_rl,
                required=True,
                include_model=False,
            )
            add_train_args(p, include_demo=True)
            add_obs_args(p)
            add_model_artifact_args(p)

        for p in (eval_p,):
            add_algo_args(
                p,
                choices=algo_choices_all,
                required=False,
                include_model=True,
            )
            add_obs_args(p)

        for p in (teleop_p, collect_p):
            # Include the observation split to avoid cross-loading policy logdirs.
            add_obs_args(p)
            add_algo_args(
                p,
                choices=algo_choices_all,
                required=False,
                include_model=True,
                group_name="Teleop Policy",
            )

        for p in (learn_p,):
            add_algo_args(
                p,
                choices=algo_choices_il,
                required=True,
                include_model=False,
            )
            add_learn_args(p)
            add_model_artifact_args(p)

        # Tuning is offline and has run/status/export/repair subcommands.
        tune_p = sub.add_parser(
            "tune",
            help="Optimize hyperparameters with Optuna",
            formatter_class=SrbHelpFormatter,
        )
        tune_subs = tune_p.add_subparsers(
            title="Tune actions",
            dest="tune_action",
            required=True,
        )
        add_tune_args(
            tune_subs,
            env_choices=env_choices,
            algo_choices_rl=algo_choices_rl,
        )

        return parser

    @classmethod
    def execute(
        cls,
        agent_subcommand: str,
        **kwargs,
    ) -> None:
        if agent_subcommand == "learn":
            from srb.cli.commands import agent_learn as _learn_cli

            kwargs.pop("repl", None)
            _learn_cli.learn_from_demonstrations(**kwargs)
        elif agent_subcommand == "tune":
            from srb.cli.commands import agent_tune as _tune_cli

            _tune_cli.execute_tune(**kwargs)
        else:
            from srb.cli.commands import agent_runtime as _runtime_cli

            _runtime_cli.execute_runtime_command(
                agent_subcommand=agent_subcommand, **kwargs
            )
