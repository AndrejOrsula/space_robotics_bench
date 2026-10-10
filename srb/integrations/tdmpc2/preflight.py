"""Validate TD-MPC2 task support and ending-mode semantics."""

from __future__ import annotations

import hashlib
import json
from typing import Any

#: Landscaping task IDs used by the early name-based capability check.
LANDSCAPING_TASK_IDS = (
    "terrain_landscaping",
    "terrain_landscaping_crater",
    "terrain_landscaping_mound",
)

#: Tasks whose step limit is an external cutoff, so they run non-episodic.
NON_EPISODIC_TASK_IDS = frozenset(LANDSCAPING_TASK_IDS)

#: User-facing reason also quoted in the landscaping training docs.
LANDSCAPING_BLOCK_MESSAGE = (
    "TD-MPC2 landscaping is disabled: velocity-controlled shovel extension is "
    "unobserved and TD-MPC2 keeps no recurrent or history state to infer it."
)

_BLOCK_DETAIL = (
    " Stock TD-MPC2 encodes the current observation feed-forward (the planner's "
    "`_prev_mean` is a warm-start for the action distribution, not a state "
    "estimate), so the same visible state can correspond to different shovel "
    "extensions. Support needs a measured shovel extension, an observation/action "
    "history or recurrent encoder, or another physically grounded state "
    "estimator; integrating the commanded velocity is not a measurement."
)


class Tdmpc2UnsupportedTaskError(RuntimeError):
    """The requested task is not supported by the stock TD-MPC2 contract."""


def normalize_task_id(env_id: str | None) -> str | None:
    """Normalize a Gym ID such as srb/terrain_landscaping-v1."""
    if not env_id:
        return None
    task = str(env_id).split("/")[-1].lower()
    if "-" in task:
        head, _, tail = task.rpartition("-")
        if head and tail.startswith("v") and tail[1:].isdigit():
            task = head
    return task.replace("-", "_")


def is_landscaping_request(
    *,
    env: Any = None,
    env_id: str | None = None,
) -> bool:
    """Return whether the task or environment declares the landscaping contract."""
    if normalize_task_id(env_id) in LANDSCAPING_TASK_IDS:
        return True
    if env is None:
        return False
    unwrapped = getattr(env, "unwrapped", env)
    if callable(getattr(unwrapped, "landscaping_contract_metadata", None)):
        return True
    declared = getattr(unwrapped, "IO_SCHEMA_FINGERPRINT", None)
    if declared:
        from srb.terrain.landscaping_contract import LANDSCAPING_IO_CONTRACT

        return str(declared) == LANDSCAPING_IO_CONTRACT.fingerprint()
    return False


def assert_task_supported(*, env: Any = None, env_id: str | None = None) -> None:
    """Fail closed on landscaping before any expensive side effect."""
    if is_landscaping_request(env=env, env_id=env_id):
        raise Tdmpc2UnsupportedTaskError(LANDSCAPING_BLOCK_MESSAGE + _BLOCK_DETAIL)


def resolve_episodic(task_id: str | None, requested: Any) -> bool:
    """Effective upstream ending mode for a task.

    Landscaping is pinned non-episodic (its 300/600-step limit is an external
    cutoff), so a config asking for ``episodic=true`` cannot
    silently turn a truncation into a terminal bootstrap target.
    """
    if normalize_task_id(task_id) in NON_EPISODIC_TASK_IDS:
        return False
    return bool(requested)


def ending_semantics_record(task_id: str | None, episodic: bool) -> dict[str, Any]:
    """Return the persisted, fingerprinted ending-mode contract."""
    record = {
        "task": normalize_task_id(task_id),
        "episodic": bool(episodic),
        "truncation_closes_sequence": True,
        "truncation_is_terminal": False,
        "bootstrap_on_truncation": True,
        "termination_requires_episodic": True,
    }
    canonical = json.dumps(
        record, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    )
    record["fingerprint"] = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    return record


__all__ = [
    "LANDSCAPING_BLOCK_MESSAGE",
    "LANDSCAPING_TASK_IDS",
    "NON_EPISODIC_TASK_IDS",
    "Tdmpc2UnsupportedTaskError",
    "assert_task_supported",
    "ending_semantics_record",
    "is_landscaping_request",
    "normalize_task_id",
    "resolve_episodic",
]
