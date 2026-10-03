# srb/core/env/privileged.py
"""Privileged-observation helpers.

STATE_PRIVILEGED is populated by the env when
cfg.expose_privileged=True, but visibility to each network is decided by
the RL preset. Deploy-time policies MUST NEVER see privileged data;
onnx_export_input_whitelist() enforces that invariant.
"""

from __future__ import annotations

from pathlib import Path

from srb._typing import ObservationGroup

__all__ = [
    "assert_privileged_absent_from_onnx",
    "is_privileged_input_name",
    "onnx_export_input_whitelist",
]


def is_privileged_input_name(name: str) -> bool:
    """Whether an observation group or ONNX input name carries privileged data.

    Matches by prefix, so the flattened forms (`state_privileged__<term>`,
    `state_privileged/<term>`) count as privileged, like the group itself.
    """
    return name.startswith(ObservationGroup.STATE_PRIVILEGED)


def onnx_export_input_whitelist(actor_obs_groups: list[str]) -> set[str]:
    """Return the set of observation-dict keys (group names) permitted as ONNX inputs.

    Raises ValueError if STATE_PRIVILEGED, or a group flattened from it, is in the
    actor group list: that would mean the actor expects privileged inputs, which
    cannot be provided at deploy time.
    """
    privileged = [g for g in actor_obs_groups if is_privileged_input_name(g)]
    if privileged:
        raise ValueError(
            f"STATE_PRIVILEGED must never appear in actor_obs_groups (got "
            f"{privileged}); it is training-only and is stripped from ONNX exports."
        )
    return set(actor_obs_groups)


def assert_privileged_absent_from_onnx(model_path: Path) -> None:
    """CI helper: inspect ONNX input names, fail if any STATE_PRIVILEGED key is present.

    Looks for any input whose name starts with `state_privileged/` (the convention
    used by srb RL wrappers when flattening observation groups into ONNX inputs).
    """
    from srb.core.sensor.event_camera.extras import require_extra

    with require_extra("onnx"):
        import onnx  # optional extra; only this CI helper needs it

    model = onnx.load(str(model_path))
    bad = [i.name for i in model.graph.input if is_privileged_input_name(i.name)]
    if bad:
        raise AssertionError(
            f"STATE_PRIVILEGED leaked into ONNX inputs: {bad} (model: {model_path})"
        )
