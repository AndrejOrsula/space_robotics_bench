"""CLI adapter for ``srb real_agent gen`` workflows."""

from __future__ import annotations

import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Callable

from srb.cli._utils import wrap_env
from srb.cli.commands._hydra_exit import (
    exit_nonzero_if_failed,
    invoke_hydra_entrypoint,
)
from srb.utils.process_supervisor import (
    ProcessCleanupError,
    ProcessKind,
    ProcessStatus,
    run_owned,
)

_GENERATION_TIMEOUT_S = 300.0
_PROCESS_TERM_GRACE_S = 0.5
_PROCESS_KILL_GRACE_S = 0.5


def _run_generation_process(cmd: Sequence[str], *, timeout: float) -> None:
    """Run one generation child through the shared ownership contract."""

    result = run_owned(
        cmd,
        kind=ProcessKind.FINITE,
        timeout=timeout,
        capture_output=True,
        term_grace_s=_PROCESS_TERM_GRACE_S,
        kill_grace_s=_PROCESS_KILL_GRACE_S,
    )
    if not result.cleanup_complete:
        raise ProcessCleanupError(result)
    if result.status is ProcessStatus.TIMED_OUT:
        raise subprocess.TimeoutExpired(
            list(cmd), timeout, output=result.stdout, stderr=result.stderr
        )

    if result.returncode:
        raise subprocess.CalledProcessError(
            result.returncode,
            list(result.args),
            output=result.stdout,
            stderr=result.stderr,
        )


def execute_generation_command(
    *,
    env_id: str,
    hardware: Sequence[str] = (),
    forwarded_args: Sequence[str] = (),
    check: bool = False,
    **kwargs: Any,
) -> None:
    """Generate real-agent environment code after parser-level validation."""
    _generate_real_agent(
        env_id=env_id,
        hardware=hardware,
        forwarded_args=forwarded_args,
        check=check,
        **kwargs,
    )


def generate_missing_real_env(
    *,
    env_id: str,
    hardware: Sequence[str] = (),
    forwarded_args: Sequence[str] = (),
) -> None:
    """Generate a missing RealEnv in an Isaac Sim subprocess."""
    _generate_real_agent_subprocess(
        env_id=env_id,
        forwarded_args=_forwarded_args_with_hardware(
            forwarded_args=forwarded_args,
            hardware=hardware,
        ),
    )


def _forwarded_args_with_hardware(
    *,
    forwarded_args: Sequence[str],
    hardware: Sequence[str],
) -> tuple[str, ...]:
    if not hardware:
        return tuple(forwarded_args)
    return (*forwarded_args, "--hardware", *hardware)


def _invoke_generation(generate: Callable[[], None], *, check: bool) -> bool:
    """Catch generation errors Kit would otherwise report as successful.

    Catch ``Exception`` broadly so check drift and schema failures become a
    non-zero exit. Let ``SystemExit``, ``KeyboardInterrupt``, and
    ``GeneratorExit`` propagate as control flow.
    """
    from srb.utils import logging

    try:
        generate()
    except Exception as exc:
        logging.error(
            f"[gen{' --check' if check else ''}] FAILED: {exc}", exc_info=True
        )
        return True
    return False


def _invoke_hydra_entrypoint(entrypoint: Callable[[], None], *, check: bool) -> bool:
    """Guard the full Hydra entrypoint, including env creation and reset.

    Those steps precede :func:`_invoke_generation`; failures there must also
    fail ``gen --env ALL --check`` rather than count an environment as verified.
    """
    return invoke_hydra_entrypoint(
        entrypoint,
        label=f"[gen{' --check' if check else ''}]",
    )


def _exit_nonzero_if_failed(failed: bool) -> None:
    """Force a non-zero process exit when generation failed."""
    exit_nonzero_if_failed(failed)


def _finalize_generation(launcher: Any, *, failed: bool) -> None:
    """Fail before close (which exits 0); otherwise close Isaac normally."""
    _exit_nonzero_if_failed(failed)
    launcher.app.close()


def _generate_real_agent(
    env_id: str,
    hardware: Sequence[str] = (),
    forwarded_args: Sequence[str] = (),
    check: bool = False,
    **kwargs: Any,
) -> None:
    if env_id.rsplit("/", 1)[-1] == "ALL":
        subprocess_args = _forwarded_args_with_hardware(
            forwarded_args=forwarded_args,
            hardware=hardware,
        )
        if check:
            subprocess_args = (*subprocess_args, "--check")
        _generate_real_agent_subprocess(
            env_id="ALL",
            forwarded_args=subprocess_args,
        )
        return

    from srb.cli.capabilities import require_isaac_sim

    require_isaac_sim()
    from srb.cli.commands.agent_launch import agent_experience_path
    from srb.cli.offline_cache_build import update_offline_srb_cache
    from srb.core.app import AppLauncher

    enable_cameras = False
    launcher = AppLauncher(
        headless=True,
        enable_cameras=enable_cameras,
        experience=agent_experience_path(
            headless=True,
            enable_cameras=enable_cameras,
            xr=False,
        ),
    )

    try:
        from srb.tasks import ensure_registered

        ensure_registered()
        update_offline_srb_cache()

        from srb.hydra.sim import hydra_task_config

        if not any(arg.startswith("hydra.output_subdir=") for arg in forwarded_args):
            sys.argv.extend(["hydra.output_subdir=null"])
    except Exception as exc:
        from srb.utils import logging

        logging.error(
            f"[gen{' --check' if check else ''}] FAILED: {exc}", exc_info=True
        )
        _finalize_generation(launcher, failed=True)

    generation_failed = False

    @hydra_task_config(
        task_name=env_id,
        agent_cfg_entry_point=None,
    )
    def hydra_main(
        env_cfg: dict[str, Any], agent_cfg: dict[str, Any] | None = None
    ) -> None:
        import gymnasium
        from omni.physx import get_physx_interface

        def _physx_gpu_enabled_from_env_cfg(cfg: Any) -> bool:
            sim = getattr(cfg, "sim", None)
            dev = getattr(sim, "device", None)
            dev_str = "" if dev is None else str(dev)
            return dev_str.startswith("cuda") or dev_str.startswith("gpu")

        get_physx_interface().overwrite_gpu_setting(
            1 if _physx_gpu_enabled_from_env_cfg(env_cfg) else 0
        )

        from srb.interfaces.sim_to_real import env as srb_real_env
        from srb.interfaces.sim_to_real.core.generator import RealEnvGenerator

        env_spec = gymnasium.spec(env_id)
        env = gymnasium.make(id=env_id, cfg=env_cfg, render_mode=None)
        env = wrap_env(env)
        env.reset()

        nonlocal generation_failed
        generation_failed = _invoke_generation(
            lambda: RealEnvGenerator().generate_offline(
                env=env,  # type: ignore[misc]
                env_spec=env_spec,
                output=Path(srb_real_env.__file__).parent.joinpath(
                    f"{env_id.removeprefix('srb/')}.py"
                ),
                hardware=hardware,
                check=check,
            ),
            check=check,
        )

        env.close()

    # _invoke_hydra_entrypoint fails closed and returns a verdict instead of
    # propagating, so there is nothing to unwind around here.
    entrypoint_failed = _invoke_hydra_entrypoint(hydra_main, check=check)  # type: ignore[arg-type]
    _finalize_generation(launcher, failed=generation_failed or entrypoint_failed)


GENERATED_ENV_DIR: Path = (
    Path(__file__).resolve().parents[2] / "interfaces" / "sim_to_real" / "env"
)


def find_orphan_generated_modules(
    env_list: Sequence[str],
    *,
    env_dir: Path | None = None,
) -> list[str]:
    """Return generated module stems with no matching cached environment."""
    if env_dir is None:
        env_dir = GENERATED_ENV_DIR
    owned = {env.rsplit("/", 1)[-1] for env in env_list}
    orphans: list[str] = []
    for module in sorted(env_dir.glob("*.py")):
        stem = module.stem
        if stem.startswith("_"):
            continue
        if stem not in owned:
            orphans.append(stem)
    return orphans


def generated_module_stems(*, env_dir: Path | None = None) -> frozenset[str]:
    """Return the checked-in generated ``env/`` module stems."""
    if env_dir is None:
        env_dir = GENERATED_ENV_DIR
    return frozenset(
        module.stem
        for module in env_dir.glob("*.py")
        if not module.stem.startswith("_")
    )


def _generate_real_agent_subprocess(
    env_id: str, forwarded_args: Sequence[str] = ()
) -> None:
    """Generate one or all cached environments in Isaac subprocesses.

    Check mode covers cached environments with generated modules and rejects
    orphan modules; write mode generates all cached environments. Empty check
    inventories and any failed child are errors, never vacuous passes.
    """
    from srb.utils import logging
    from srb.utils.cache import read_offline_srb_env_cache
    from srb.utils.isaacsim import get_kit_python

    if env_id == "ALL":
        env_list = read_offline_srb_env_cache()
        if not env_list:
            if "--check" in forwarded_args:
                logging.error(
                    "No environments found in cache, so `gen --env ALL "
                    "--check` verified nothing. Run an environment (or `srb "
                    "agent rand --env <id>`) to populate the cache, then retry."
                )
                raise SystemExit(1)
            logging.warning(
                "No environments found in cache. Please run an environment "
                "first to populate the cache."
            )
            return

        env_list = [
            env for env in env_list if not env.rsplit("/", 1)[-1].startswith("_")
        ]
        if "--check" in forwarded_args:
            orphans = find_orphan_generated_modules(env_list)
            if orphans:
                logging.error(
                    "Generated sim-to-real modules without a registered "
                    f"environment (stale or orphaned): {', '.join(orphans)}. "
                    "Remove or regenerate them; `gen --env ALL --check` "
                    "fails until the generated inventory is exact."
                )
                raise SystemExit(1)
            generated = generated_module_stems()
            not_generated = [
                env for env in env_list if env.rsplit("/", 1)[-1] not in generated
            ]
            env_list = [env for env in env_list if env.rsplit("/", 1)[-1] in generated]
            if not_generated:
                logging.info(
                    "Skipping cached environments without a generated module "
                    f"(not generated): {', '.join(sorted(not_generated))}"
                )
            if not env_list:
                logging.error(
                    "No cached environment has a generated sim-to-real module, "
                    "so `gen --env ALL --check` verified nothing."
                )
                raise SystemExit(1)
        logging.info(
            f"Generating sim-to-real setup for {len(env_list)} environments..."
        )
        env_list.sort()
    else:
        env_list = (env_id,)

    successful_envs = []
    failed_envs = []
    for i, env in enumerate(env_list, 1):
        logging.info(f"[{i}/{len(env_list)}] Processing environment: {env}")
        cmd = [
            get_kit_python(),
            "-m",
            "srb",
            "real_agent",
            "gen",
            "--env",
            env,
            *forwarded_args,
        ]
        try:
            _run_generation_process(cmd, timeout=_GENERATION_TIMEOUT_S)
            successful_envs.append(env)
            action = "verified" if "--check" in forwarded_args else "generated"
            logging.info(f"Successfully {action} sim-to-real setup for {env}")
        except ProcessCleanupError:
            raise
        except subprocess.TimeoutExpired:
            failed_envs.append((env, "Timeout after 5 minutes"))
            logging.error(f"Timeout generating sim-to-real setup for {env}")
        except subprocess.CalledProcessError as e:
            failed_envs.append((env, f"Process failed with code {e.returncode}"))
            logging.error(f"Failed to generate sim-to-real setup for {env}: {e.stderr}")
        except Exception as e:
            failed_envs.append((env, str(e)))
            logging.error(f"Exception while processing {env}: {e}")

    logging.info("\n=== Generation Summary ===")
    logging.info(
        f"Successfully processed: {len(successful_envs)}/{len(env_list)} environments"
    )
    if successful_envs:
        logging.info(f"Successful environments: {', '.join(successful_envs)}")
    if failed_envs:
        logging.error("Failed environments:")
        for env, reason in failed_envs:
            logging.error(f"  - {env}: {reason}")
        raise SystemExit(1)


__all__ = ("execute_generation_command", "generate_missing_real_env")
