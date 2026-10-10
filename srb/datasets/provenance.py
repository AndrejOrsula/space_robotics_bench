"""Dataset provenance helpers shared by collectors and recorders."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.utils.artifact_identity import ArtifactIdentity

_PROVENANCE_KEYS = frozenset(
    (
        "task",
        "embodiment",
        "action_mode",
        "split",
        "controller",
        "algo",
        "model",
        "model_artifact",
    )
)


@dataclass(frozen=True)
class DatasetProvenance:
    """Source identity for one collected SRB dataset artifact."""

    task: str
    embodiment: str
    action_mode: str
    split: str
    controller: str
    algo: str | None = None
    model: str | None = None
    model_artifact: str | None = None
    extra: Mapping[str, object] = field(default_factory=dict)

    def to_env_kwargs(self) -> dict[str, object]:
        """Return legacy recorder-compatible metadata."""
        # Canonical fields win; an extra ``model_artifact`` must not survive
        # when the canonical value is ``None`` (and so is omitted below).
        data = {
            key: value
            for key, value in self.extra.items()
            if key not in _PROVENANCE_KEYS
        }
        data.update(
            {
                "task": self.task,
                "embodiment": self.embodiment,
                "action_mode": self.action_mode,
                "split": self.split,
                "controller": self.controller,
                "algo": self.algo,
                "model": self.model,
            }
        )
        if self.model_artifact is not None:
            data["model_artifact"] = self.model_artifact
        return data

    def identity(self) -> "ArtifactIdentity":
        """Return the canonical artifact identity for this provenance."""
        from srb.utils.artifact_identity import ArtifactIdentity

        return ArtifactIdentity(
            task=self.task,
            embodiment=self.embodiment,
            action_mode=self.action_mode,
            algo=self.algo,
        )


def dataset_provenance_from_env(
    env: "AnyEnv",
    env_cfg: Any,
    *,
    split: str,
    controller: str,
    algo: str | None = None,
    model: str | Path | None = None,
    model_artifact: str | Path | None = None,
    extra: Mapping[str, object] | None = None,
) -> DatasetProvenance:
    """Infer dataset provenance from an SRB env/config pair."""
    robot_cfg = getattr(env_cfg, "robot", None)
    robot_class = (
        robot_cfg.__class__.__name__ if robot_cfg is not None else "unknown_embodiment"
    )
    task = (
        getattr(env_cfg, "task_name", None)
        or getattr(getattr(env, "spec", None), "id", None)
        or str(env.unwrapped.__class__.__name__)
    )
    return DatasetProvenance(
        task=str(task),
        embodiment=_asset_name(robot_cfg) or robot_class,
        action_mode=_action_mode(robot_cfg) or "unknown_action",
        split=str(split),
        controller=str(controller),
        algo=None if algo is None else str(algo),
        model=None if model is None else str(model),
        model_artifact=None if model_artifact is None else str(model_artifact),
        extra=dict(extra or {}),
    )


def _action_mode(robot_cfg: Any) -> str | None:
    """Return the selected action mode; ``None`` selects the first declared one."""
    mode = getattr(robot_cfg, "action_mode", None)
    if mode is None:
        mode = next(iter(getattr(robot_cfg, "action_modes", None) or ()), None)
    return None if mode is None else str(mode)


def _asset_name(asset_cfg: Any) -> str:
    """Return an asset config's declared name, calling classmethod values."""

    name = getattr(asset_cfg, "name", None)
    if callable(name):
        name = name()
    return "" if name is None else str(name)


def dataset_provenance_from_env_kwargs(
    *,
    env_name: str,
    env_kwargs: Mapping[str, object] | None,
) -> DatasetProvenance:
    """Build provenance from legacy recorder ``env_kwargs`` metadata."""
    env_kwargs = dict(env_kwargs or {})
    split = str(env_kwargs.get("split") or "human")
    extra = {
        key: value for key, value in env_kwargs.items() if key not in _PROVENANCE_KEYS
    }
    return DatasetProvenance(
        task=str(env_kwargs.get("task") or env_name),
        embodiment=str(env_kwargs.get("embodiment") or "unknown_embodiment"),
        action_mode=str(env_kwargs.get("action_mode") or "unknown_action"),
        split=split,
        controller=str(env_kwargs.get("controller") or split),
        algo=_optional_str(env_kwargs.get("algo")),
        model=_optional_str(env_kwargs.get("model")),
        model_artifact=_optional_str(env_kwargs.get("model_artifact")),
        extra=extra,
    )


def coerce_dataset_provenance(
    provenance: DatasetProvenance | Mapping[str, object] | None,
    *,
    env_name: str,
    env_kwargs: Mapping[str, object] | None = None,
) -> DatasetProvenance:
    """Normalize explicit or legacy provenance values."""
    if isinstance(provenance, DatasetProvenance):
        return provenance
    if provenance is not None:
        return dataset_provenance_from_env_kwargs(
            env_name=env_name,
            env_kwargs=provenance,
        )
    return dataset_provenance_from_env_kwargs(
        env_name=env_name,
        env_kwargs=env_kwargs,
    )


def _optional_str(value: object | None) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text if text else None
