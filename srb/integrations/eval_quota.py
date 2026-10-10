"""Per-environment episode quotas for bounded vector evaluation.

Counting the first ``N`` completions biases results when episode length
correlates with outcome, such as early crashes or successes. Instead, each
independently autoresetting row gets a fixed quota of episodes started after
evaluation begins::

    targets[i] = N // num_envs + (1 if i < N % num_envs else 0)

Completions count while ``counts[i] < targets[i]``. Evaluation continues until
all quotas are met; zero-quota and completed rows keep stepping, but their
completions are ignored. Callers must reset first so no in-progress episode is
counted.

This plain-Python module is imported by the CLI without torch, numpy, or Isaac.
Callers convert done masks to environment ids.
"""

from __future__ import annotations

import json
import math
import operator
import os
import tempfile
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, SupportsIndex

EPISODE_SAMPLING = "per_env_quota"
"""Result-field value naming the episode selection rule above."""


def _as_int(name: str, value: object) -> int:
    """Return an integer or integer-like (numpy/torch scalar), rejecting bools."""
    # ``operator.index`` rejects ``numpy.bool_``; the check covers Python bools.
    if isinstance(value, bool) or not isinstance(value, SupportsIndex):
        raise ValueError(f"{name} must be an integer, got {value!r}")
    try:
        return operator.index(value)
    except TypeError as exc:
        raise ValueError(f"{name} must be an integer, got {value!r}") from exc


def _positive_int(name: str, value: object) -> int:
    number = _as_int(name, value)
    if number <= 0:
        raise ValueError(f"{name} must be a positive integer, got {value!r}")
    return number


def per_env_episode_targets(target_episodes: int, num_envs: int) -> tuple[int, ...]:
    """Split ``target_episodes`` over ``num_envs`` rows, remainder to low ids."""
    target_episodes = _positive_int("target_episodes", target_episodes)
    num_envs = _positive_int("num_envs", num_envs)
    base, remainder = divmod(target_episodes, num_envs)
    return tuple(base + (1 if env_id < remainder else 0) for env_id in range(num_envs))


def eval_episode_budget() -> int | None:
    """Read the optional ``SRB_EVAL_EPISODES`` bounded-evaluation episode count."""
    raw = os.environ.get("SRB_EVAL_EPISODES")
    if raw is None:
        return None
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"SRB_EVAL_EPISODES must be a positive integer, got {raw!r}"
        ) from exc
    if value <= 0:
        raise ValueError(f"SRB_EVAL_EPISODES must be a positive integer, got {raw!r}")
    return value


def eval_max_steps(env: Any, target_episodes: int) -> int:
    """Return a finite guard for environments that never emit a done row."""
    unwrapped = getattr(env, "unwrapped", env)
    try:
        max_episode_length = int(getattr(unwrapped, "max_episode_length", 0) or 0)
    except (OverflowError, TypeError, ValueError):
        max_episode_length = 0
    if max_episode_length <= 0:
        max_episode_length = 1000
    return max_episode_length * target_episodes * 2


def effective_eval_seed(env_cfg: Any) -> Any | None:
    """Return the caller's configured seed without inventing a protocol seed."""
    if env_cfg is None:
        return None
    if isinstance(env_cfg, Mapping):
        return env_cfg.get("seed")
    return getattr(env_cfg, "seed", None)


def finite_mean(values: list[float], framework: str) -> float:
    """Average finite episode values without overflowing the intermediate sum."""
    count = len(values)
    mean = math.fsum(value / count for value in values)
    if not math.isfinite(mean):
        raise ValueError(f"{framework} evaluation produced a non-finite mean return")
    return mean


class EpisodeQuota:
    """Track which finished episodes count toward a bounded evaluation."""

    def __init__(self, target_episodes: int, num_envs: int) -> None:
        self.targets = per_env_episode_targets(target_episodes, num_envs)
        self.target_episodes = sum(self.targets)
        self.num_envs = len(self.targets)
        self._counts = [0] * self.num_envs
        self._completed = 0

    @property
    def counts(self) -> tuple[int, ...]:
        """Counted episodes per env row."""
        return tuple(self._counts)

    @property
    def completed(self) -> int:
        """Total counted episodes."""
        return self._completed

    @property
    def complete(self) -> bool:
        """Whether every env row has met its quota."""
        return self._completed >= self.target_episodes

    def accept(self, done_env_ids: Iterable[int]) -> list[int]:
        """Record the finished rows and return those whose episode counts.

        ``done_env_ids`` are the rows that finished an episode on one vector
        step, in any order. A row counts at most once per call.
        """
        accepted: list[int] = []
        seen: set[int] = set()
        for raw_env_id in done_env_ids:
            env_id = _as_int("env id", raw_env_id)
            if not 0 <= env_id < self.num_envs:
                raise ValueError(
                    f"env id {env_id} is outside [0, {self.num_envs}) environments"
                )
            if env_id in seen or self._counts[env_id] >= self.targets[env_id]:
                continue
            seen.add(env_id)
            self._counts[env_id] += 1
            self._completed += 1
            accepted.append(env_id)
        return accepted

    def result_fields(self) -> dict[str, Any]:
        """Fields every bounded-eval result records about episode selection."""
        return {"num_envs": self.num_envs, "episode_sampling": EPISODE_SAMPLING}


def write_eval_result(
    result: Mapping[str, Any], *, evaluation_logdir: Path | None = None
) -> None:
    """Log the result; write it to ``SRB_EVAL_OUT``, else ``<dir>/eval.json``."""
    from srb.utils import logging
    from srb.utils.publication import default_new_file_mode

    encoded = json.dumps(dict(result), sort_keys=True, allow_nan=False)
    logging.info(f"SRB_EVAL_RESULT {encoded}")
    if "complete" in result:
        completed = result.get("episodes_completed", result.get("n_episodes", 0))
        requested = result.get("episodes_requested", completed)
        status = "complete" if result["complete"] is True else "incomplete"
        parts = [f"Evaluation {status}: {completed}/{requested} episodes"]
        mean_return = result.get("mean_return")
        if mean_return is not None:
            parts.append(f"mean return {mean_return:.1f}")
        success_rate = result.get("success_rate")
        if success_rate is not None:
            parts.append(f"success rate {success_rate:.0%}")
        logging.info(", ".join(parts))
    out_path = os.environ.get("SRB_EVAL_OUT")
    if not out_path and evaluation_logdir is not None:
        out_path = str(Path(evaluation_logdir) / "eval.json")
    if not out_path:
        return
    output_path = Path(out_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Replace atomically so an interrupted write never leaves truncated JSON.
    descriptor, tmp_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as tmp_file:
            json.dump(dict(result), tmp_file, indent=2, allow_nan=False)
            tmp_file.write("\n")
        os.chmod(tmp_name, default_new_file_mode())  # mkstemp creates 0600
        os.replace(tmp_name, output_path)
    finally:
        Path(tmp_name).unlink(missing_ok=True)
    logging.info(f"Wrote eval result to {output_path}")


__all__ = (
    "EPISODE_SAMPLING",
    "EpisodeQuota",
    "effective_eval_seed",
    "eval_episode_budget",
    "eval_max_steps",
    "finite_mean",
    "per_env_episode_targets",
    "write_eval_result",
)
