"""Shared utilities used across CLI subcommands.

Everything here is safe to import before Isaac Sim is launched.  Functions
that require a running simulation accept the relevant handles as arguments.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

from srb.utils.process_supervisor import ProcessKind
from srb.wrappers.composition import wrap_env as wrap_env

if TYPE_CHECKING:
    from srb._typing import AnyEnv


###############################################################################
# Subprocess helper
###############################################################################


def run_subprocess(
    cmd: Sequence[str],
    *,
    description: str,
    kind: ProcessKind,
    timeout: float | None,
    capture_output: bool = False,
) -> None:
    """Run one owned subprocess, logging the command and exiting on failure.

    The logged command line is a redacted display copy; ``cmd`` itself is
    passed to the child unchanged, so forwarded credentials still work but
    never reach the console log.
    """
    import string

    from srb.utils import logging
    from srb.utils.process_supervisor import (
        DEFAULT_TERM_GRACE_S,
        INTERACTIVE_KILL_GRACE_S,
        ProcessStatus,
        interactive_term_grace_s,
        run_owned,
    )
    from srb.utils.redaction import redact_secret_arguments

    logging.info(
        f"{description}: "
        + " ".join(
            (f'"{arg}"' if any(c in string.whitespace for c in arg) else arg)
            for arg in redact_secret_arguments(cmd)
        )
    )
    # Interactive launches are Isaac Sim / GUI children: Kit needs tens of
    # seconds to unwind its renderer, physics, and USD stage, so a sub-second
    # TERM grace would turn every Ctrl-C into an effective SIGKILL. A repeated
    # Ctrl-C shortens this grace inside the supervisor.
    interactive = kind is ProcessKind.INTERACTIVE
    result = run_owned(
        cmd,
        kind=kind,
        timeout=timeout,
        capture_output=capture_output,
        term_grace_s=(
            interactive_term_grace_s() if interactive else DEFAULT_TERM_GRACE_S
        ),
        kill_grace_s=(
            INTERACTIVE_KILL_GRACE_S if interactive else DEFAULT_TERM_GRACE_S
        ),
        reap_grace_s=(
            INTERACTIVE_KILL_GRACE_S if interactive else DEFAULT_TERM_GRACE_S
        ),
    )
    if not result.cleanup_complete:
        logging.critical(
            f"{description} failed process cleanup after {result.cause or result.status}"
        )
        sys.exit(125)
    if result.status is ProcessStatus.TIMED_OUT:
        logging.critical(f"{description} timed out")
        sys.exit(124)
    if result.returncode is not None and result.returncode < 0:
        # `subprocess` reports death by signal N as -N, and `sys.exit(-N)`
        # would leave the interpreter with 256-N (SIGSEGV -> 245). Use the
        # shell's 128+N convention so wrappers and CI can name the signal.
        import signal

        signum = -result.returncode
        try:
            signal_name = signal.Signals(signum).name
        except ValueError:
            signal_name = str(signum)
        logging.critical(f"{description} terminated by signal {signal_name}")
        sys.exit(128 + signum)
    if result.returncode:
        logging.critical(f"{description} failed due to the exception above")
        sys.exit(result.returncode)


###############################################################################
# Performance test wrapper
###############################################################################


def wrap_env_in_performance_test(
    env: "AnyEnv",
    sim_app: Any,
    perf_output: str,
    perf_duration: float,
) -> Any:
    """Wrap *env* with a step-timing performance monitor.

    Returns the wrapped environment.
    """
    import time
    import traceback
    from collections import deque

    import torch
    from gymnasium.core import ActType, Env, ObsType, Wrapper

    class PerformanceTestWrapper(Wrapper):
        def __init__(
            self,
            env: Env[ObsType, ActType],
            output: str,
            duration: float,
            min_report_interval_fraction: float = 0.1,
            max_buffer_size: int = 1000000,
        ) -> None:
            super().__init__(env)
            self.__perf_output = output
            self.__perf_duration = duration
            # A non-positive duration means "no limit"; keep periodic reporting on a
            # sane cadence instead of duration*fraction==0, which would report every
            # single step and flood stdout / the output file.
            self.__perf_min_report_interval = (
                duration * min_report_interval_fraction if duration > 0 else 10.0
            )

            _env: "AnyEnv" = env.unwrapped  # type: ignore[misc]
            self.__num_envs = _env.num_envs
            self.__agent_rate = _env.cfg.agent_rate

            self._perf_num_steps = 0
            self._perf_num_episodes = 0
            self._perf_env_episode_steps: torch.Tensor | None = None
            self._perf_episode_lengths = []
            self._perf_total_time = 0.0
            self._perf_step_timings = deque(maxlen=max_buffer_size)
            self._perf_start_time = time.perf_counter()
            self._perf_last_report_time = self._perf_start_time

            if self.__perf_output != "STDOUT":
                # A bare filename (no dir component) yields ''; os.makedirs('')
                # raises FileNotFoundError, so only create a real parent dir.
                parent_dir = os.path.dirname(self.__perf_output)
                if parent_dir and not os.path.exists(parent_dir):
                    os.makedirs(parent_dir, exist_ok=True)

        def step(self, action) -> tuple[ObsType, Any, bool, bool, dict[str, Any]]:
            t_start = time.perf_counter()
            obs, reward, terminated, truncated, info = super().step(action)
            t_end = time.perf_counter()
            step_timing = t_end - t_start
            self._perf_step_timings.append(step_timing)
            self._perf_num_steps += self.__num_envs
            self._perf_total_time += step_timing

            done: torch.Tensor = terminated | truncated  # type: ignore[misc]
            num_done = torch.sum(done).item()
            self._perf_num_episodes += num_done
            if self._perf_env_episode_steps is None:
                self._perf_env_episode_steps = torch.zeros_like(done, dtype=torch.long)
            self._perf_env_episode_steps += 1
            if num_done:
                self._perf_episode_lengths.extend(
                    self._perf_env_episode_steps[done].tolist()
                )
                self._perf_env_episode_steps[done] = 0
            episode_ended = num_done > 0.5 * self.__num_envs

            if episode_ended or (
                t_end - self._perf_last_report_time > self.__perf_min_report_interval
            ):
                self.__perf_report()
                self._perf_last_report_time = t_end

            if (
                self.__perf_duration > 0
                and self._perf_total_time >= self.__perf_duration
            ):
                self.__perf_report(final=True)
                print("The performance test has finished (duration limit reached).")
                # ``sim_app.close()`` terminates the process and never returns,
                # so a failed env close is reported and turned into a non-zero
                # exit before it; Kit itself dies with the process.
                from srb.cli.commands._hydra_exit import exit_nonzero_if_failed
                from srb.utils.failures import attempt_cleanup

                _, close_error = attempt_cleanup(env.close)
                if close_error is not None:
                    traceback.print_exception(close_error)
                exit_nonzero_if_failed(close_error is not None)
                sim_app.close()
                sys.exit(0)

            return obs, reward, terminated, truncated, info

        def __perf_report(self, final: bool = False) -> None:
            steps = self._perf_num_steps
            episodes = self._perf_num_episodes
            total_time = self._perf_total_time
            timings = torch.tensor(list(self._perf_step_timings))
            steps_per_sec = steps / total_time if total_time > 0 else 0
            mean_step_time = torch.mean(timings).item() if timings.numel() > 0 else 0
            median_step_time = (
                torch.median(timings).item() if timings.numel() > 0 else 0
            )
            min_step_time = torch.min(timings).item() if timings.numel() > 0 else 0
            max_step_time = torch.max(timings).item() if timings.numel() > 0 else 0

            def torch_percentiles(t, percentiles) -> list[float]:
                if t.numel() == 0:
                    return [0 for _ in percentiles]
                t_sorted, _ = torch.sort(t)
                n = t.numel()
                result = []
                for p in percentiles:
                    k = (n - 1) * (p / 100)
                    f = torch.floor(torch.tensor(k)).long()
                    c = torch.ceil(torch.tensor(k)).long()
                    if f == c:
                        val = t_sorted[f]
                    else:
                        d0 = t_sorted[f] * (c.float() - k)
                        d1 = t_sorted[c] * (k - f.float())
                        val = d0 + d1
                    result.append(val.item())
                return result

            step_time_percentiles = torch_percentiles(timings, [10, 20, 80, 90])
            episode_lengths = torch.tensor(
                self._perf_episode_lengths, dtype=torch.float64
            )
            mean_episode_len = (
                torch.mean(episode_lengths).item() if episode_lengths.numel() > 0 else 0
            )
            median_episode_len = (
                torch.median(episode_lengths).item()
                if episode_lengths.numel() > 0
                else 0
            )
            min_episode_len = (
                torch.min(episode_lengths).item() if episode_lengths.numel() > 0 else 0
            )
            max_episode_len = (
                torch.max(episode_lengths).item() if episode_lengths.numel() > 0 else 0
            )
            episode_len_percentiles = torch_percentiles(
                episode_lengths, [10, 20, 80, 90]
            )
            report = (
                f"\nPerformance Report{' (final)' if final else ''}:\n"
                f"    Elapsed time (s)       : {total_time:.2f}\n"
                f"    Total steps (#)        : {steps}\n"
                f"    Total episodes (#)     : {episodes}\n"
                f"    Steps per second (#/s) : {steps_per_sec:.2f}\n"
                f"    Step time (ms)         : min={min_step_time * 1000:.3f}, p10={step_time_percentiles[0] * 1000:.3f}, p20={step_time_percentiles[1] * 1000:.3f}, mean={mean_step_time * 1000:.3f}, median={median_step_time * 1000:.3f}, p80={step_time_percentiles[2] * 1000:.3f}, p90={step_time_percentiles[3] * 1000:.3f}, max={max_step_time * 1000:.3f}\n"
                f"    Episode length (steps) : min={min_episode_len:.1f}, p10={episode_len_percentiles[0]:.1f}, p20={episode_len_percentiles[1]:.1f}, mean={mean_episode_len:.1f}, median={median_episode_len:.1f}, p80={episode_len_percentiles[2]:.1f}, p90={episode_len_percentiles[3]:.1f}, max={max_episode_len:.1f}\n"
                f"    Episode length (s)     : min={self.__agent_rate * min_episode_len:.1f}, p10={self.__agent_rate * episode_len_percentiles[0]:.1f}, p20={self.__agent_rate * episode_len_percentiles[1]:.1f}, mean={self.__agent_rate * mean_episode_len:.1f}, median={self.__agent_rate * median_episode_len:.1f}, p80={self.__agent_rate * episode_len_percentiles[2]:.1f}, p90={self.__agent_rate * episode_len_percentiles[3]:.1f}, max={self.__agent_rate * max_episode_len:.1f}\n"
            )
            if self.__perf_output == "STDOUT":
                print(report)
            else:
                with open(self.__perf_output, "a") as f:
                    f.write(report + "\n")

    return PerformanceTestWrapper(env, output=perf_output, duration=perf_duration)
