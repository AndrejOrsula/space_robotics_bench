"""Shared argument-group builders for CLI subcommands."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Sequence

from srb.cli._parsing import (
    AutoNamespaceTaskAction,
    AutoRealNamespaceTaskAction,
    ExplicitAction,
    SrbHelpFormatter,
)
from srb.utils.path import SRB_LOGS_DIR

_TUNE_LOGDIR_DEFAULT = (SRB_LOGS_DIR / "tune").as_posix()
_CFG_SENTINELS = ("DEFAULT", "IGNORE", "NONE", "NULL")

###############################################################################
# Completion
###############################################################################
# Argcomplete calls completers with keyword arguments only when completing, so
# none of these imports argcomplete or runtime modules while parsing.


def _task_id_candidates(choices: Sequence[object] | None) -> list[str]:
    """Return canonical ``namespace/task`` choices plus the bare ids accepted."""
    canonical = [str(choice) for choice in choices or ()]
    bare = [choice.split("/", 1)[1] for choice in canonical if "/" in choice]
    return list(dict.fromkeys([*canonical, *bare]))


def complete_task_ids(*, action: argparse.Action, **_: object) -> list[str]:
    """Complete a task option from its own choices."""
    return _task_id_candidates(action.choices)  # type: ignore[arg-type]


def complete_algorithms(**_: object) -> list[str]:
    """Offer every registered algorithm label."""
    from srb.core.algorithms import known_algorithm_labels

    return sorted(known_algorithm_labels())


def complete_hydra_pins(**_: object) -> list[str]:
    """Offer no files; installed shell scripts complete ``KEY=VALUE`` pins."""
    return []


def complete_hydra_config(*, prefix: str, **_: object) -> list[str]:
    """Offer ``--cfg`` sentinels (case-insensitive) and YAML paths."""
    from argcomplete.completers import FilesCompleter

    sentinels = _CFG_SENTINELS if prefix.isupper() else map(str.lower, _CFG_SENTINELS)
    return [*sentinels, *FilesCompleter(allowednames=("yaml", "yml"))(prefix=prefix)]


###############################################################################
# Launcher
###############################################################################


def add_launcher_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Launcher")
    group.add_argument(
        "--headless",
        help="Run the simulation without a display",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--hide_ui",
        help="Hide most of the Isaac Sim UI and use fullscreen mode",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--livestream",
        help="Livestream mode, as for the `LIVESTREAM` environment variable (0: Disabled, 1: Native, 2: WebRTC). Default (-1) defers to LIVESTREAM.",
        type=int,
        choices=(-1, 0, 1, 2),
        default=-1,
    )
    group.add_argument(
        "--rendering_mode",
        # Isaac Lab removed these presets (isaac-sim/IsaacLab#6405); SRB's
        # AppLauncher applies its copy from srb/core/app/rendering_modes.py.
        # Choices stay literal so --help does not import Isaac Lab.
        help=(
            "Isaac RTX rendering preset applied after Kit starts; `xr` applies "
            "no preset (the XR experience selected by --xr carries its settings)"
        ),
        type=str,
        action=ExplicitAction,
        choices=("performance", "balanced", "quality", "xr"),
    )
    group.add_argument(
        "--xr",
        help="Enable XR mode for VR/AR applications",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--enable_cameras",
        help=(
            "Enable camera rendering without video recording; required for "
            "environments that spawn camera sensors"
        ),
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--kit_args",
        help=(
            "Space-separated Omniverse Kit CLI args. Attach a value beginning "
            "with '-' to the option (--kit_args=VALUE; e.g. "
            "--kit_args=--ext-folder=/path/to/ext1)."
        ),
        type=str,
        default="",
    )
    return group


###############################################################################
# Environment
###############################################################################


def add_env_args(
    parser: argparse.ArgumentParser,
    *,
    env_choices: Sequence[str] | None = None,
    real: bool = False,
    allow_all: bool = False,
) -> argparse._ArgumentGroup:
    choices: tuple | Sequence[str] | None = env_choices
    if allow_all and env_choices:
        choices = ("ALL", *env_choices)

    group = parser.add_argument_group("Environment")
    action = group.add_argument(
        "-e",
        "--env",
        "--task",
        dest="env_id",
        help="Name of the environment to select",
        type=str,
        action=AutoRealNamespaceTaskAction if real else AutoNamespaceTaskAction,
        choices=choices,
        required=True,
    )
    action.permissive_choices = True
    action.completer = complete_task_ids  # type: ignore[attr-defined]
    return group


###############################################################################
# Config
###############################################################################


def add_config_args(
    parser: argparse.ArgumentParser,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Config")
    action = group.add_argument(
        "--cfg",
        dest="config",
        help=(
            "Path to the Hydra configuration YAML file to override the default "
            "settings ('default': use the default config from the environment, "
            "'ignore'/'none'/'null': no config)"
        ),
        type=str,
        default="DEFAULT",
    )
    action.completer = complete_hydra_config  # type: ignore[attr-defined]
    return group


###############################################################################
# Interface / Video / Performance
###############################################################################


def add_interface_args(
    parser: argparse.ArgumentParser,
    *,
    interface_choices: Sequence[str],
) -> None:
    interfaces_group = parser.add_argument_group("Interface")
    action = interfaces_group.add_argument(
        "--interface",
        help="Sequence of interfaces to enable",
        type=str,
        nargs="*",
        choices=interface_choices,
        default=[],
    )
    action.permissive_choices = True

    video_recording_group = parser.add_argument_group("Video Recording")
    video_recording_group.add_argument(
        "--video",
        dest="video_enable",
        help="Record videos",
        action="store_true",
        default=False,
    )
    video_recording_group.add_argument(
        "--record-events",
        "--record_events",
        dest="record_events",
        help=(
            "Record event-camera HDF5 episodes and a provenance manifest. "
            "Needs `events` in env.active_sensors."
        ),
        action="store_true",
        default=False,
    )

    snapshot_group = parser.add_argument_group("Snapshot")
    snapshot_group.add_argument(
        "--snapshot",
        dest="snapshot_path",
        help=(
            "Capture one rendered PNG to this path after the initial reset "
            "(which settles and, if env.particles_freeze=true, freezes "
            "particles), then exit. Implies --enable_cameras; add --headless "
            "for offscreen capture. Frame via env.viewer.eye / env.viewer.lookat."
        ),
        type=_nonempty_snapshot_path_arg,
        default=None,
    )
    snapshot_group.add_argument(
        "--snapshot-warmup",
        "--snapshot_warmup",
        dest="snapshot_warmup",
        help=(
            "Number of render passes (no physics stepping) before the "
            "snapshot is grabbed, to let path-traced / anti-aliased "
            "accumulation converge. Needs --snapshot (default: 16)."
        ),
        type=_nonnegative_int_arg,
        default=None,
    )
    snapshot_group.add_argument(
        "--snapshot-settle",
        "--snapshot_settle",
        dest="snapshot_settle",
        help=(
            "Physics steps between the initial reset and the snapshot, letting "
            "robots fall and settle onto the terrain (pair with "
            "env.robot_spawn_height to drop them from above). Frozen particles "
            "stay frozen. Needs --snapshot (default: 0, no settling)."
        ),
        type=_nonnegative_int_arg,
        default=None,
    )

    performance_group = parser.add_argument_group("Performance")
    performance_group.add_argument(
        "--perf",
        dest="perf_enable",
        help="Measure environment performance",
        action="store_true",
        default=False,
    )
    performance_group.add_argument(
        "--perf_output",
        help="Output path for performance results; needs --perf (default: STDOUT)",
        type=str,
        default=None,
    )
    performance_group.add_argument(
        "--perf_duration",
        help="Maximum performance-test duration in seconds; needs --perf (default: 150, 0: no limit)",
        type=_nonnegative_finite_float_arg,
        default=None,
    )


###############################################################################
# Hardware
###############################################################################


def add_hardware_args(
    parser: argparse.ArgumentParser,
    *,
    hardware_choices: Sequence[str] | None = None,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Hardware")
    action = group.add_argument(
        "--hardware",
        "--hw",
        help="Sequence of hardware interfaces to use",
        type=str,
        nargs="*",
        choices=hardware_choices,
        default=[],
    )
    action.permissive_choices = True
    return group


###############################################################################
# Logging
###############################################################################


def add_logging_args(
    parser: argparse.ArgumentParser,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Logging")
    group.add_argument(
        "--logdir",
        "--logs",
        dest="logdir_path",
        help="Path to the root directory for storing logs",
        type=str,
        default=SRB_LOGS_DIR,
    )
    return group


###############################################################################
# Teleop
###############################################################################


def add_teleop_args(
    parser: argparse.ArgumentParser,
    *,
    teleop_device_choices: Sequence[str],
    device_help: str | None = None,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Teleop")
    action = group.add_argument(
        "--teleop_device",
        help=device_help
        or (
            "One or more teleop devices for interacting with the environment. "
            "When omitted, the selected environment may provide its own default."
        ),
        type=str,
        nargs="+",
        choices=teleop_device_choices,
        default=None,
    )
    action.permissive_choices = True
    group.add_argument(
        "--pos_sensitivity",
        help="Sensitivity factor for translation",
        type=_finite_float_arg,
        default=1.0,
    )
    group.add_argument(
        "--rot_sensitivity",
        help="Sensitivity factor for rotation (default: π rad/s)",
        type=_finite_float_arg,
        default=3.1415927,
    )
    group.add_argument(
        "--invert_controls",
        "--invert",
        help="Invert the controls for translation in manipulation environments",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return group


###############################################################################
# Collection
###############################################################################


def add_collection_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Collection")
    group.add_argument(
        "--num_episodes",
        type=_positive_int_arg,
        default=100,
        help="Number of episodes to collect",
    )
    group.add_argument(
        "--horizon",
        type=_positive_int_arg,
        default=400,
        help="Maximum number of steps per episode",
    )
    group.add_argument(
        "--success_only",
        action="store_true",
        default=False,
        help="Only save episodes that end in success",
    )
    group.add_argument(
        "--collector",
        choices=("human", "policy", "random", "zero"),
        default=None,
        help=(
            "Data source for collection. Default infers policy when --algo "
            "or --model is provided, otherwise human teleop."
        ),
    )
    return group


###############################################################################
# Algorithm
###############################################################################


def _canonical_algorithm_arg(value: str) -> str:
    from srb.core.algorithms import canonical_algorithm_label

    try:
        return canonical_algorithm_label(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def add_algo_args(
    parser: argparse.ArgumentParser,
    *,
    choices: Sequence[str],
    required: bool = True,
    include_model: bool = True,
    group_name: str = "Algorithm",
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group(group_name)
    group.add_argument(
        "--algo",
        help="Name of the algorithm",
        type=_canonical_algorithm_arg,
        choices=choices,
        required=required,
    )
    if include_model:
        group.add_argument(
            "--model",
            type=str,
            help=(
                "Path to a model checkpoint or SRB model artifact "
                "(algorithm is inferred when --algo is omitted)"
            ),
        )
    return group


###############################################################################
# Observation modality
###############################################################################


def add_obs_args(parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
    """Add ``--obs {state,visual}`` observation selection.

    This changes the agent-config entry point and logdir workflow name, not the
    algorithm label. ``visual`` requires visual observation profiles.
    """
    group = parser.add_argument_group("Observation")
    group.add_argument(
        "--obs",
        choices=("state", "visual"),
        default="state",
        help=(
            "Observation modality: 'state' (default) or 'visual' (image "
            "observations; requires visual sensor profiles)"
        ),
    )
    return group


###############################################################################
# Tune (Optuna hyperparameter optimization)
###############################################################################


def add_tune_args(
    tune_subs: argparse._SubParsersAction,
    *,
    env_choices: Sequence[str] | None = None,
    algo_choices_rl: Sequence[str] = (),
) -> None:
    """Register the ``srb agent tune`` verbs (run / status / export / repair).

    Isaac-free: these commands have no launcher args. Study artifacts default
    to ``logs/tune`` under ``<logdir>/``.
    """

    def complete_tasks(**_: object) -> list[str]:
        return _task_id_candidates(env_choices)

    def _add_logdir(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--logdir",
            "--logs",
            dest="logdir_path",
            type=str,
            default=_TUNE_LOGDIR_DEFAULT,
            help="Root directory for tuning study artifacts",
        )

    def _add_study(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--study",
            type=str,
            required=True,
            help="Study name (e.g. srb-skrl_ppo-state-pick-v1)",
        )

    # ---- run ----
    run = tune_subs.add_parser(
        "run",
        help="Run or resume a hyperparameter-optimization study",
        formatter_class=SrbHelpFormatter,
    )
    env_group = run.add_argument_group("Environment")
    action = env_group.add_argument(
        "-e",
        "--env",
        "--task",
        dest="env_id",
        type=str,
        choices=env_choices,
        help="Single task to tune (single-task study)",
    )
    action.permissive_choices = True  # type: ignore[attr-defined]
    action.completer = complete_task_ids  # type: ignore[attr-defined]
    tasks_action = env_group.add_argument(
        "--tasks",
        nargs="+",
        default=None,
        help="Task set for a portfolio study",
    )
    tasks_action.completer = complete_tasks  # type: ignore[attr-defined]
    algo_group = run.add_argument_group("Algorithm")
    algo_group.add_argument(
        "--algo",
        type=_canonical_algorithm_arg,
        choices=algo_choices_rl,
        required=True,
        help="Algorithm label to tune",
    )
    add_obs_args(run)
    study_group = run.add_argument_group("Study")
    study_group.add_argument(
        "--trial-steps",
        "--trial_steps",
        dest="trial_env_steps",
        type=_positive_int_arg,
        required=True,
        help="Total environment steps per trial (canonical budget)",
    )
    study_group.add_argument(
        "--n-trials",
        "--n_trials",
        dest="n_trials",
        type=int,
        default=20,
        help="Number of trials (finished) to reach",
    )
    study_group.add_argument(
        "--n-workers",
        "--n_workers",
        dest="n_workers",
        type=_positive_int_arg,
        default=1,
        help="Concurrent trial slots in this driver",
    )
    study_group.add_argument(
        "--gpus",
        type=str,
        default=None,
        help="Comma-separated GPU ids to round-robin (e.g. 0,1)",
    )
    study_group.add_argument(
        "--seed-base",
        "--seed_base",
        dest="seed_base",
        type=int,
        default=42,
        help="Base seed; child seed = seed-base + trial*seeds-per-trial + replicate",
    )
    study_group.add_argument(
        "--seeds-per-trial",
        "--seeds_per_trial",
        dest="seeds_per_trial",
        type=int,
        default=1,
        help="Replicates per trial, averaged into one lower-noise objective "
        "(pruning runs on replicate 0; surviving trials cost ~K× the compute)",
    )
    study_group.add_argument(
        "--trial-num-envs",
        "--trial_num_envs",
        dest="num_envs",
        type=int,
        default=None,
        help="Parallel envs per trial (pinned; drives budget conversion)",
    )
    study_group.add_argument(
        "--pin",
        dest="pins",
        nargs="*",
        default=[],
        help="Fix config values: agent.X=Y / env.X=Y (not sampled)",
    ).completer = complete_hydra_pins  # type: ignore[attr-defined]
    study_group.add_argument(
        "--max-wall-hours",
        "--max_wall_hours",
        dest="max_wall_hours",
        type=_nonnegative_finite_float_arg,
        default=None,
        help="Stop asking new trials past this wall-clock budget",
    )
    study_group.add_argument(
        "--trial-timeout",
        "--trial_timeout",
        dest="timeout_s",
        type=float,
        default=None,
        help="Kill a trial exceeding this many seconds",
    )
    study_group.add_argument(
        "--objective",
        choices=("ep_return_mean", "success_rate"),
        default="ep_return_mean",
        help="Objective field (tail-mean)",
    )
    study_group.add_argument(
        "--normalize",
        choices=("none", "baseline", "rank"),
        default="none",
        help="Portfolio per-task score normalization (multi-task studies)",
    )
    study_group.add_argument(
        "--aggregate",
        choices=("median", "mean", "min"),
        default="median",
        help="Portfolio cross-task aggregation (multi-task studies)",
    )
    study_group.add_argument(
        "--fps",
        dest="fps_vec_n",
        type=float,
        default=None,
        help="Measured env-steps/s for the wall-clock projection",
    )
    study_group.add_argument(
        "--n-startup",
        "--n_startup",
        dest="n_startup",
        type=int,
        default=None,
        help="TPE random-startup trials before model-based sampling",
    )
    study_group.add_argument(
        "--boot-stagger",
        "--boot_stagger",
        dest="boot_stagger_s",
        type=float,
        default=None,
        help="Seconds to serialize concurrent Isaac first-boots (default: 30 when --n-workers > 1, else 0)",
    )
    study_group.add_argument(
        "--version",
        type=int,
        default=1,
        help="Study version; bump for an incompatible re-run",
    )
    study_group.add_argument(
        "--keep-artifacts",
        "--keep_artifacts",
        dest="keep_artifacts",
        action="store_true",
        help="Keep PRUNED trials' child logdirs (checkpoints, TB events)",
    )
    study_group.add_argument(
        "--retry-failed",
        "--retry_failed",
        dest="retry_failed",
        type=int,
        default=0,
        help="Re-enqueue up to N FAILed param sets (skipping timeouts) at startup; "
        "raise --n-trials to give them budget to run",
    )
    study_group.add_argument(
        "--storage",
        type=str,
        default=None,
        help="PostgreSQL Optuna storage URL for shared workers (requires the tune-rdb extra); omit for the journal default",
    )
    _add_logdir(run)

    # ---- status ----
    status = tune_subs.add_parser(
        "status",
        help="Summarize a study (best trial + state counts)",
        formatter_class=SrbHelpFormatter,
    )
    _add_study(status)
    status.add_argument(
        "--storage",
        type=str,
        default=None,
        help="PostgreSQL Optuna storage URL used by the study",
    )
    _add_logdir(status)

    # ---- export ----
    export = tune_subs.add_parser(
        "export",
        help="Export a tuned config to the L0/L1 hyperparams slot",
        formatter_class=SrbHelpFormatter,
    )
    _add_study(export)
    export.add_argument(
        "--storage",
        type=str,
        default=None,
        help="PostgreSQL Optuna storage URL used by the study",
    )
    export.add_argument(
        "--trial",
        type=str,
        default="best",
        help="Trial number to export, or 'best'",
    )
    export.add_argument(
        "--to",
        choices=("task", "global"),
        default=None,
        help="Target slot: task-specific (L1) or global per-algorithm (L0). "
        "Default: task for single-task studies, global for portfolio studies.",
    )
    export.add_argument(
        "--dry-run",
        "--dry_run",
        dest="dry_run",
        action="store_true",
        help="Print the diff and write nothing",
    )
    export.add_argument(
        "--force",
        action="store_true",
        help="Overwrite an existing file",
    )
    _add_logdir(export)

    # ---- repair ----
    repair = tune_subs.add_parser(
        "repair",
        help="Fail stale RUNNING trials (single-operator cleanup)",
        formatter_class=SrbHelpFormatter,
    )
    _add_study(repair)
    repair.add_argument(
        "--storage",
        type=str,
        default=None,
        help="PostgreSQL Optuna storage URL used by the study",
    )
    repair.add_argument(
        "--stale-hours",
        "--stale_hours",
        dest="stale_hours",
        type=float,
        default=24.0,
        help="RUNNING trials older than this are failed",
    )
    _add_logdir(repair)

    # ---- baseline ----
    baseline = tune_subs.add_parser(
        "baseline",
        help="Capture floor/ref normalization baselines for portfolio studies",
        formatter_class=SrbHelpFormatter,
    )
    bl_env = baseline.add_argument_group("Environment")
    bl_action = bl_env.add_argument(
        "-e",
        "--env",
        "--task",
        dest="env_id",
        type=str,
        choices=env_choices,
        help="Single task to baseline",
    )
    bl_action.permissive_choices = True  # type: ignore[attr-defined]
    bl_action.completer = complete_task_ids  # type: ignore[attr-defined]
    tasks_action = bl_env.add_argument(
        "--tasks",
        nargs="+",
        default=None,
        help="Task set to baseline",
    )
    tasks_action.completer = complete_tasks  # type: ignore[attr-defined]
    baseline.add_argument(
        "--algo",
        type=_canonical_algorithm_arg,
        choices=algo_choices_rl,
        required=True,
        help="Algorithm label the baselines are measured for",
    )
    add_obs_args(baseline)
    baseline.add_argument(
        "--budget",
        "--ref-steps",
        dest="budget",
        type=_positive_int_arg,
        required=True,
        help="Reference-run env-steps (== the trial budget you will tune at)",
    )
    baseline.add_argument(
        "--floor-steps",
        "--floor_steps",
        dest="floor_steps",
        type=_positive_int_arg,
        default=200_000,
        help="Random-rollout env-steps for the floor",
    )
    baseline.add_argument(
        "--trial-num-envs",
        "--trial_num_envs",
        dest="num_envs",
        type=int,
        default=None,
        help="Parallel envs (pinned; drives budget conversion)",
    )
    baseline.add_argument(
        "--seed-base",
        "--seed_base",
        dest="seed_base",
        type=int,
        default=42,
        help="Base seed; task k uses seed-base + k",
    )
    baseline.add_argument(
        "--gpus",
        type=str,
        default=None,
        help="Comma-separated GPU ids to round-robin",
    )
    baseline.add_argument(
        "--objective",
        choices=("ep_return_mean", "success_rate"),
        default="ep_return_mean",
        help="Field floor/ref are measured on",
    )
    baseline.add_argument(
        "--trial-timeout",
        "--trial_timeout",
        dest="timeout_s",
        type=float,
        default=None,
        help="Kill a floor/ref run exceeding this many seconds",
    )
    baseline.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing baseline entries",
    )
    baseline.add_argument(
        "--dry-run",
        "--dry_run",
        dest="dry_run",
        action="store_true",
        help="Compute but do not write baselines.yaml",
    )
    _add_logdir(baseline)

    # ---- check ----
    check = tune_subs.add_parser(
        "check",
        help="Preflight a run config (Isaac-free): resolve algo/space/base "
        "config, check pins, dry-assemble one trial doc — no study, no training",
        formatter_class=SrbHelpFormatter,
    )
    c_env = check.add_argument_group("Environment")
    c_action = c_env.add_argument(
        "-e",
        "--env",
        "--task",
        dest="env_id",
        type=str,
        choices=env_choices,
        help="Single task to check (single-task study)",
    )
    c_action.permissive_choices = True  # type: ignore[attr-defined]
    c_action.completer = complete_task_ids  # type: ignore[attr-defined]
    tasks_action = c_env.add_argument(
        "--tasks",
        nargs="+",
        default=None,
        help="Task set for a portfolio study",
    )
    tasks_action.completer = complete_tasks  # type: ignore[attr-defined]
    check.add_argument(
        "--algo",
        type=_canonical_algorithm_arg,
        choices=algo_choices_rl,
        required=True,
        help="Algorithm label to check",
    )
    add_obs_args(check)
    check.add_argument(
        "--trial-steps",
        "--trial_steps",
        dest="trial_env_steps",
        type=int,
        required=True,
        help="Total environment steps per trial (drives the dry-assemble budget)",
    )
    check.add_argument(
        "--pin",
        dest="pins",
        nargs="*",
        default=[],
        help="Fix config values: agent.X=Y / env.X=Y (validated, not sampled)",
    ).completer = complete_hydra_pins  # type: ignore[attr-defined]
    check.add_argument(
        "--trial-num-envs",
        "--trial_num_envs",
        dest="num_envs",
        type=int,
        default=None,
        help="Parallel envs per trial (pinned; drives budget conversion)",
    )
    check.add_argument(
        "--seed-base",
        "--seed_base",
        dest="seed_base",
        type=int,
        default=42,
        help="Base seed; child seed = seed-base + trial*seeds-per-trial + replicate",
    )
    check.add_argument(
        "--seeds-per-trial",
        "--seeds_per_trial",
        dest="seeds_per_trial",
        type=int,
        default=1,
        help="Replicates per trial used by the run preflight projection",
    )
    check.add_argument(
        "--normalize",
        choices=("none", "baseline", "rank"),
        default="none",
        help="Portfolio normalization (checks baselines when 'baseline')",
    )
    check.add_argument(
        "--aggregate",
        choices=("median", "mean", "min"),
        default="median",
        help="Portfolio cross-task aggregation",
    )
    check.add_argument(
        "--objective",
        choices=("ep_return_mean", "success_rate"),
        default="ep_return_mean",
        help="Objective field (recorded in the manifest)",
    )
    check.add_argument(
        "--fps",
        dest="fps_vec_n",
        type=float,
        default=None,
        help="Measured env-steps/s for a wall-clock projection",
    )
    check.add_argument(
        "--n-trials",
        "--n_trials",
        dest="n_trials",
        type=int,
        default=20,
        help="Trial target (only used for the wall-clock projection)",
    )
    check.add_argument(
        "--n-workers",
        "--n_workers",
        dest="n_workers",
        type=_positive_int_arg,
        default=1,
        help="Worker count (only used for the wall-clock projection)",
    )
    check.add_argument(
        "--version",
        type=int,
        default=1,
        help="Study version",
    )
    _add_logdir(check)

    # ---- validate ----
    validate = tune_subs.add_parser(
        "validate",
        help="Re-run the top-K trials with fresh seeds at "
        "(optionally higher) fidelity and re-rank by the multi-seed mean (advisory)",
        formatter_class=SrbHelpFormatter,
    )
    _add_study(validate)
    validate.add_argument(
        "--storage",
        type=str,
        default=None,
        help="PostgreSQL Optuna storage URL used by the study",
    )
    validate.add_argument(
        "--top-k",
        "--top_k",
        dest="top_k",
        type=int,
        default=3,
        help="Re-run this many top COMPLETE trials (ranked by training value)",
    )
    validate.add_argument(
        "--seeds",
        type=int,
        default=3,
        help="Fresh seeds per candidate; the objective is averaged across them",
    )
    validate.add_argument(
        "--seed-base",
        "--seed_base",
        dest="seed_base",
        type=int,
        default=10_000,
        help="Validation seed base (kept disjoint from training seeds); seed = base + i",
    )
    validate.add_argument(
        "--min-ok",
        "--min_ok",
        dest="min_ok",
        type=int,
        default=None,
        help="Min COMPLETEd seeds for a candidate to be eligible as winner "
        "(default: a strict majority of --seeds)",
    )
    validate.add_argument(
        "--validate-steps",
        "--validate_steps",
        dest="validate_steps",
        type=int,
        default=None,
        help="Env-steps per re-run (default: the study's trial budget; raise for higher fidelity)",
    )
    validate.add_argument(
        "--gpus",
        type=str,
        default=None,
        help="Comma-separated GPU ids to round-robin across re-runs",
    )
    validate.add_argument(
        "--n-workers",
        "--n_workers",
        dest="n_workers",
        type=_positive_int_arg,
        default=1,
        help="Concurrent re-run slots",
    )
    validate.add_argument(
        "--trial-timeout",
        "--trial_timeout",
        dest="timeout_s",
        type=float,
        default=None,
        help="Kill a re-run exceeding this many seconds",
    )
    validate.add_argument(
        "--keep-artifacts",
        "--keep_artifacts",
        dest="keep_artifacts",
        action="store_true",
        help="Keep re-run child logdirs (checkpoints, TB events)",
    )
    _add_logdir(validate)


###############################################################################
# Train
###############################################################################


def _nonempty_demo_dataset_arg(value: str) -> str:
    """Keep the user path unchanged while rejecting an empty CLI value."""
    if not value.strip():
        raise argparse.ArgumentTypeError("demo dataset path must be non-empty")
    return value


def _nonempty_snapshot_path_arg(value: str) -> str:
    """Reject an empty path, which runtime checks would read as no snapshot."""
    if not value.strip():
        raise argparse.ArgumentTypeError("snapshot path must be non-empty")
    return value


def _nonnegative_int_arg(value: str) -> int:
    """Parse an integer that is zero or positive."""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid int value: {value!r}") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError(f"must not be negative, got {value!r}")
    return parsed


def _finite_float_arg(value: str) -> float:
    """Parse a finite float; NaN or infinity would reach device commands."""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid float value: {value!r}") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"must be a finite number, got {value!r}")
    return parsed


def _nonnegative_finite_float_arg(value: str) -> float:
    """Parse a finite float that is zero or positive."""
    parsed = _finite_float_arg(value)
    if parsed < 0.0:
        raise argparse.ArgumentTypeError(f"must not be negative, got {value!r}")
    return parsed


def _positive_int_arg(value: str) -> int:
    """Parse a positive integer."""
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid int value: {value!r}") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError(f"must be a positive integer, got {value!r}")
    return parsed


def _positive_demo_max_transitions_arg(value: str) -> int:
    """Parse the bounded demo transition cap for argparse."""
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(
            "demo max transitions must be a positive integer"
        ) from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError(
            "demo max transitions must be a positive integer"
        )
    return parsed


def add_train_args(
    parser: argparse.ArgumentParser,
    *,
    include_demo: bool = False,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Train")
    mutex = group.add_mutually_exclusive_group()
    mutex.add_argument(
        "--continue_training",
        "--continue",
        help=(
            "Continue training the model from the last checkpoint "
            "(framework-dependent; TD-MPC2 has no full trainer-state "
            "checkpoint and rejects this flag)"
        ),
        action="store_true",
        default=False,
    )
    mutex.add_argument(
        "--model",
        help=(
            "Continue training the model from the specified checkpoint "
            "(framework-dependent; TD-MPC2 imports weights only and restarts "
            "schedules and replay at step 0)"
        ),
        type=str,
    )
    if include_demo:
        group.add_argument(
            "--demo-dataset",
            "--demo_dataset",
            dest="demo_dataset",
            type=_nonempty_demo_dataset_arg,
            default=None,
            help=(
                "Canonical demonstration dataset used to prefill replay "
                "(requires --demo-max-transitions; "
                "skrl_sac, skrl_ddpg, skrl_td3, dreamer)"
            ),
        )
        group.add_argument(
            "--demo-max-transitions",
            "--demo_max_transitions",
            dest="demo_max_transitions",
            type=_positive_demo_max_transitions_arg,
            default=None,
            help=(
                "Maximum demonstration transitions to prefill; requires --demo-dataset"
            ),
        )
    return group


###############################################################################
# Model Artifact
###############################################################################


def add_model_artifact_args(
    parser: argparse.ArgumentParser,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Model Artifact")
    group.add_argument(
        "--model-artifact-output",
        type=Path,
        default=None,
        help=(
            "Package the completed run into this SRB model artifact directory "
            "after training succeeds"
        ),
    )
    group.add_argument(
        "--model-artifact-checkpoint",
        type=Path,
        default=None,
        help=(
            "Explicit checkpoint to package, absolute or relative to the run directory"
        ),
    )
    group.add_argument(
        "--model-artifact-task",
        default=None,
        help="Override artifact task identity",
    )
    group.add_argument(
        "--model-artifact-embodiment",
        default=None,
        help="Override artifact embodiment identity",
    )
    group.add_argument(
        "--model-artifact-action-mode",
        default=None,
        help="Override artifact action-mode identity",
    )
    group.add_argument(
        "--model-artifact-framework",
        default=None,
        help="Override artifact framework metadata",
    )
    group.add_argument(
        "--model-artifact-dataset",
        type=Path,
        default=None,
        help=(
            "Training dataset manifest, artifact directory, or dataset file "
            "used for model provenance and identity inference"
        ),
    )
    group.add_argument(
        "--model-artifact-dataset-repo-id",
        default=None,
        help="Training dataset repository id for the model artifact",
    )
    group.add_argument(
        "--model-artifact-dataset-revision",
        default=None,
        help="Training dataset revision for the model artifact",
    )
    group.add_argument(
        "--model-artifact-repo-id",
        default=None,
        help="Override Hugging Face model repository id",
    )
    return group


###############################################################################
# Learn (Imitation Learning)
###############################################################################


def add_learn_args(
    parser: argparse.ArgumentParser,
) -> argparse._ArgumentGroup:
    group = parser.add_argument_group("Learn")
    group.add_argument(
        "--dataset",
        type=str,
        help="Path to the HDF5 dataset for offline learning",
    )
    group.add_argument(
        "--num_epochs",
        type=_positive_int_arg,
        default=None,
        help="Number of training epochs (overrides config if provided)",
    )
    group.add_argument(
        "--normalize_actions",
        action="store_true",
        default=False,
        help="Normalize actions in the dataset to [-1, 1] range",
    )
    return group
