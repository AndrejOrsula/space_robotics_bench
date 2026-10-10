"""Shared terminal-transition handling for SRB's SKRL agents."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import torch

_FINAL_OBSERVATION_KEYS = ("final_observation", "terminal_observation", "final_obs")
_FINAL_OBSERVATION_MASK_KEYS = (
    "final_observation_mask",
    "_final_observation",
    "_final_obs",
    "terminal_observation_mask",
    "_terminal_observation",
)
_FINAL_STATE_MASK_KEYS = ("final_state_mask", "_final_state")
_POLICY_TERMINAL_KEYS = frozenset(
    {
        *_FINAL_OBSERVATION_KEYS,
        "final_observation_ids",
        *_FINAL_OBSERVATION_MASK_KEYS,
    }
)


def _payload_key(info: Mapping[str, Any]) -> str | None:
    return next((key for key in _FINAL_OBSERVATION_KEYS if key in info), None)


def _mask_tensor(
    value: object,
    *,
    num_envs: int,
    device: torch.device,
    label: str,
) -> torch.Tensor:
    try:
        mask = torch.as_tensor(value, device=device, dtype=torch.bool).reshape(-1)
    except (TypeError, RuntimeError, ValueError) as exc:
        raise RuntimeError(f"SKRL {label} cannot be converted to a mask") from exc
    if mask.shape != (num_envs,):
        raise RuntimeError(
            f"SKRL {label} shape does not match the batch: "
            f"{tuple(mask.shape)} != {(num_envs,)}"
        )
    return mask


def _ids_tensor(
    value: object,
    *,
    num_envs: int,
    done_mask: torch.Tensor,
    device: torch.device,
) -> torch.Tensor:
    try:
        ids = torch.as_tensor(value, device=device, dtype=torch.long).reshape(-1)
    except (TypeError, RuntimeError, ValueError) as exc:
        raise RuntimeError("SKRL final_observation_ids cannot be converted") from exc
    if ids.numel() and (bool((ids < 0).any()) or bool((ids >= num_envs).any())):
        raise RuntimeError("SKRL final_observation_ids contains an out-of-range row")
    if ids.unique().numel() != ids.numel():
        raise RuntimeError("SKRL final_observation_ids contains duplicate rows")
    if ids.numel() and not bool(done_mask.index_select(0, ids).all()):
        raise RuntimeError("SKRL final_observation_ids contains a row that did not end")
    return ids


def _row_sequence(value: object) -> list[object] | None:
    if isinstance(value, torch.Tensor):
        if value.ndim == 0:
            return None
        return list(value)
    if isinstance(value, (list, tuple)):
        return list(value)
    # NumPy object arrays may contain None and need row-wise handling.
    if hasattr(value, "ndim") and hasattr(value, "__len__"):
        try:
            if int(value.ndim) > 0:
                return list(value)
        except (TypeError, ValueError):
            return None
    return None


def _stack_rows(
    rows: Sequence[object],
    *,
    next_states: torch.Tensor,
    valid: torch.Tensor | None = None,
    positions: torch.Tensor | None = None,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert dense or object rows to a full batch, retaining row validity."""

    num_envs = next_states.shape[0]
    full = next_states.clone()
    row_valid = torch.zeros(num_envs, dtype=torch.bool, device=next_states.device)
    if positions is None:
        positions = torch.arange(num_envs, device=next_states.device)
    if len(rows) != positions.numel():
        raise RuntimeError(
            "SKRL final_observation row count does not match its row ids"
        )
    for position, row in zip(positions.tolist(), rows, strict=True):
        if row is None:
            continue
        try:
            tensor = torch.as_tensor(
                row, device=next_states.device, dtype=next_states.dtype
            )
        except (TypeError, RuntimeError, ValueError) as exc:
            raise RuntimeError(
                f"SKRL final_observation row {position} cannot be converted"
            ) from exc
        if tensor.shape != next_states.shape[1:]:
            raise RuntimeError(
                f"SKRL final_observation row {position} shape does not match "
                f"next_states: {tuple(tensor.shape)} != "
                f"{tuple(next_states.shape[1:])}"
            )
        full[position] = tensor
        row_valid[position] = True
    if valid is not None:
        row_valid &= valid
    return full, row_valid


def _candidate_from_infos(
    infos: Any,
    next_states: torch.Tensor,
    done_mask: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor] | None:
    """Return a full terminal batch and its explicit or inferred validity mask."""

    num_envs = next_states.shape[0]
    if isinstance(infos, Mapping):
        key = _payload_key(infos)
        if key is None:
            return None
        candidate = infos[key]
        ids = (
            _ids_tensor(
                infos["final_observation_ids"],
                num_envs=num_envs,
                done_mask=done_mask,
                device=next_states.device,
            )
            if "final_observation_ids" in infos
            else None
        )
        explicit_mask = None
        for mask_key in _FINAL_OBSERVATION_MASK_KEYS:
            if mask_key in infos:
                explicit_mask = _mask_tensor(
                    infos[mask_key],
                    num_envs=num_envs,
                    device=next_states.device,
                    label=mask_key,
                )
                break

        if isinstance(candidate, torch.Tensor):
            try:
                final = torch.as_tensor(
                    candidate, device=next_states.device, dtype=next_states.dtype
                )
            except (TypeError, RuntimeError, ValueError) as exc:
                raise RuntimeError(
                    "SKRL final_observation cannot be converted to the next-state dtype"
                ) from exc

            if final.shape == next_states.shape:
                inferred = torch.ones(
                    num_envs, dtype=torch.bool, device=next_states.device
                )
                if explicit_mask is not None:
                    inferred &= explicit_mask
                if ids is not None:
                    id_mask = torch.zeros(
                        num_envs, dtype=torch.bool, device=next_states.device
                    )
                    id_mask[ids] = True
                    inferred &= id_mask
                return final, inferred

            if final.ndim > 0 and final.shape[0] == num_envs:
                # Preserve the full-batch interpretation for row-shape errors.
                raise RuntimeError(
                    "SKRL final_observation row 0 shape does not match "
                    f"next_states: {tuple(final.shape[1:])} != "
                    f"{tuple(next_states.shape[1:])}"
                )

            if ids is not None:
                if final.ndim == 0:
                    raise RuntimeError(
                        "SKRL sparse final_observation does not match "
                        "final_observation_ids"
                    )
                if final.shape[0] != ids.numel():
                    raise RuntimeError(
                        "SKRL final_observation row count does not match its row ids"
                    )
                if ids.numel() and final.shape[1:] != next_states.shape[1:]:
                    position = int(ids[0].item())
                    raise RuntimeError(
                        f"SKRL final_observation row {position} shape does not "
                        f"match next_states: {tuple(final.shape[1:])} != "
                        f"{tuple(next_states.shape[1:])}"
                    )
                full = next_states.clone()
                if ids.numel():
                    full.index_copy_(0, ids, final)
                inferred = torch.zeros(
                    num_envs, dtype=torch.bool, device=next_states.device
                )
                inferred[ids] = True
                if explicit_mask is not None:
                    inferred &= explicit_mask
                return full, inferred

            raise RuntimeError(
                "SKRL final_observation shape does not match next_states: "
                f"{tuple(final.shape)} != {tuple(next_states.shape)}"
            )

        rows = _row_sequence(candidate)
        if rows is not None and (
            (ids is not None and len(rows) != num_envs) or len(rows) == num_envs
        ):
            # Object arrays may contain None and cannot be reliably tensorized.
            positions = ids if ids is not None and len(rows) != num_envs else None
            full, inferred = _stack_rows(
                rows,
                next_states=next_states,
                valid=explicit_mask,
                positions=positions,
            )
            if ids is not None and len(rows) == num_envs:
                id_mask = torch.zeros(
                    num_envs, dtype=torch.bool, device=next_states.device
                )
                id_mask[ids] = True
                inferred &= id_mask
            return full, inferred
        try:
            final = torch.as_tensor(
                candidate, device=next_states.device, dtype=next_states.dtype
            )
        except (TypeError, RuntimeError, ValueError) as exc:
            raise RuntimeError(
                "SKRL final_observation cannot be converted to the next-state dtype"
            ) from exc
        if ids is not None:
            if final.ndim == 0 or final.shape[0] != ids.numel():
                raise RuntimeError(
                    "SKRL sparse final_observation does not match final_observation_ids"
                )
            return _stack_rows(
                list(final),
                next_states=next_states,
                valid=explicit_mask,
                positions=ids,
            )
        if final.shape != next_states.shape:
            raise RuntimeError(
                "SKRL final_observation shape does not match next_states: "
                f"{tuple(final.shape)} != {tuple(next_states.shape)}"
            )
        inferred = torch.ones(num_envs, dtype=torch.bool, device=next_states.device)
        if explicit_mask is not None:
            inferred &= explicit_mask
        return final, inferred

    if not isinstance(infos, (list, tuple)):
        return None
    if len(infos) != num_envs:
        raise RuntimeError(
            "SKRL terminal observation info list does not match the next-state batch"
        )
    rows: list[object] = []
    valid = torch.zeros(num_envs, dtype=torch.bool, device=next_states.device)
    for index, item in enumerate(infos):
        if not isinstance(item, Mapping):
            rows.append(None)
            continue
        key = _payload_key(item)
        if key is None:
            rows.append(None)
            continue
        rows.append(item[key])
        valid[index] = item[key] is not None
        for mask_key in _FINAL_OBSERVATION_MASK_KEYS:
            if mask_key in item:
                mask_value = item[mask_key]
                try:
                    valid[index] &= bool(mask_value)
                except (TypeError, RuntimeError, ValueError) as exc:
                    raise RuntimeError(
                        f"SKRL {mask_key} cannot be converted to a row mask"
                    ) from exc
                break
    if not bool(valid.any()):
        return None
    return _stack_rows(rows, next_states=next_states, valid=valid)


def terminal_next_states(
    next_states: torch.Tensor,
    infos: Any,
    done: torch.Tensor,
) -> torch.Tensor:
    """Use a shape-valid live terminal snapshot for every done row.

    SRB environments use same-step autoreset. Their returned observation is
    already the first frame of the next episode, so silently bootstrapping from
    it would mix episodes. A done transition without a valid final snapshot is
    rejected at this framework seam.
    """

    done_mask = torch.as_tensor(
        done, device=next_states.device, dtype=torch.bool
    ).reshape(-1)
    if done_mask.shape[0] != next_states.shape[0]:
        raise ValueError(
            "SKRL done mask does not match the next-state batch: "
            f"{tuple(done_mask.shape)} != {tuple(next_states.shape)}"
        )
    if not bool(done_mask.any()):
        return next_states

    candidate = _candidate_from_infos(infos, next_states, done_mask)
    if candidate is None:
        raise RuntimeError(
            "SKRL done transition has no live final_observation snapshot"
        )
    final, valid = candidate
    missing = done_mask & ~valid
    if bool(missing.any()):
        rows = missing.nonzero(as_tuple=False).flatten().tolist()
        raise RuntimeError(
            "SKRL done transition has missing or invalid final_observation rows: "
            f"{rows}"
        )
    mask = done_mask
    while mask.ndim < next_states.ndim:
        mask = mask.unsqueeze(-1)
    return torch.where(mask, final, next_states)


def _final_state_infos(infos: Any) -> Any | None:
    """Adapt explicit critic terminal snapshots to the shared validator."""

    state_keys = {
        "final_state",
        "final_state_ids",
        *_FINAL_STATE_MASK_KEYS,
    }
    if isinstance(infos, Mapping):
        if "final_state" not in infos:
            return None
        normalized = {
            key: value
            for key, value in infos.items()
            if key not in _POLICY_TERMINAL_KEYS and key not in state_keys
        }
        normalized["final_observation"] = infos["final_state"]
        if "final_state_ids" in infos:
            normalized["final_observation_ids"] = infos["final_state_ids"]
        for key in _FINAL_STATE_MASK_KEYS:
            if key in infos:
                normalized["final_observation_mask"] = infos[key]
                break
        return normalized
    if not isinstance(infos, (list, tuple)):
        return None

    normalized_rows: list[Any] = []
    found = False
    for item in infos:
        if not isinstance(item, Mapping):
            normalized_rows.append({})
            continue
        if "final_state" not in item:
            # Do not retain a policy-stream snapshot for this row. A critic
            # state must never be inferred from a same-width observation.
            normalized_rows.append({})
            continue
        found = True
        normalized = {
            key: value
            for key, value in item.items()
            if key not in _POLICY_TERMINAL_KEYS and key not in state_keys
        }
        normalized["final_observation"] = item["final_state"]
        for key in _FINAL_STATE_MASK_KEYS:
            if key in item:
                normalized["final_observation_mask"] = item[key]
                break
        normalized_rows.append(normalized)
    return normalized_rows if found else None


def terminal_final_states(
    next_states: torch.Tensor,
    infos: Any,
    done: torch.Tensor,
) -> torch.Tensor:
    """Repair critic states from explicit ``final_state`` snapshots only.

    Canonical demonstrations contain observations, not privileged runtime
    state. This seam is therefore intentionally strict: a supplied critic
    stream must carry its own terminal snapshot with ids and masks.
    """

    done_mask = torch.as_tensor(done, device=next_states.device, dtype=torch.bool)
    if not bool(done_mask.any()):
        return next_states
    normalized = _final_state_infos(infos)
    if normalized is None:
        raise RuntimeError(
            "SKRL done transition has no live final_state snapshot for the critic"
        )
    try:
        return terminal_next_states(next_states, normalized, done_mask)
    except RuntimeError as exc:
        raise RuntimeError(f"SKRL final_state snapshot is invalid: {exc}") from exc


__all__ = ["terminal_final_states", "terminal_next_states"]
