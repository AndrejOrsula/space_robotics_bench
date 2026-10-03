"""Stable SRB artifact identities for datasets, models, and runs."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from typing import NotRequired, TypedDict

from srb.utils.json_types import validate_json_value

DEFAULT_HF_ORG = "space_robotics_bench"


class ArtifactIdentityDict(TypedDict):
    """Serialized canonical identity fields."""

    task: str
    embodiment: str
    action_mode: str
    algo: NotRequired[str | None]


_ARTIFACT_IDENTITY_KEYS = frozenset({"task", "embodiment", "action_mode", "algo"})
_REQUIRED_ARTIFACT_IDENTITY_KEYS = frozenset({"task", "embodiment", "action_mode"})


def slugify_underscore(value: str) -> str:
    """Return a community-friendly lowercase slug using underscores."""
    slug = re.sub(r"[^A-Za-z0-9]+", "_", value.strip().lower())
    slug = re.sub(r"_+", "_", slug).strip("_")
    if not slug:
        raise ValueError("artifact identity fields must not be empty")
    return slug


def stable_hash(value: object) -> str:
    """Hash canonical JSON with stable key ordering and no string fallback."""

    canonical = validate_json_value(value, context="stable hash input")
    payload = json.dumps(
        canonical,
        allow_nan=False,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


@dataclass(frozen=True, slots=True)
class ArtifactIdentity:
    """Canonical SRB identity for public datasets and model checkpoints."""

    task: str
    embodiment: str
    action_mode: str
    algo: str | None = None

    def __post_init__(self) -> None:
        for field_name in ("task", "embodiment", "action_mode"):
            if not isinstance(getattr(self, field_name), str):
                raise TypeError(
                    f"artifact identity field {field_name!r} must be a string"
                )
        if self.algo is not None and not isinstance(self.algo, str):
            raise TypeError("artifact identity field 'algo' must be a string or null")
        object.__setattr__(self, "task", slugify_underscore(self.task))
        object.__setattr__(self, "embodiment", slugify_underscore(self.embodiment))
        object.__setattr__(self, "action_mode", slugify_underscore(self.action_mode))
        if self.algo is not None:
            object.__setattr__(self, "algo", slugify_underscore(self.algo))

    @property
    def dataset_name(self) -> str:
        # A namespaced task (``srb/foo``) slugs to ``srb_foo``; keep one prefix.
        task = self.task.removeprefix("srb_")
        return f"srb_{task}_{self.embodiment}_{self.action_mode}"

    @property
    def model_name(self) -> str:
        if self.algo is None:
            raise ValueError("model artifact identity requires algo")
        return f"{self.dataset_name}_{self.algo}"

    def dataset_repo_id(self, org: str = DEFAULT_HF_ORG) -> str:
        return f"{org}/{self.dataset_name}"

    def model_repo_id(self, org: str = DEFAULT_HF_ORG) -> str:
        return f"{org}/{self.model_name}"

    def to_dict(self) -> ArtifactIdentityDict:
        payload: ArtifactIdentityDict = {
            "task": self.task,
            "embodiment": self.embodiment,
            "action_mode": self.action_mode,
            "algo": self.algo,
        }
        return payload

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> "ArtifactIdentity":
        """Parse an exact identity mapping without coercing corrupt fields."""

        if any(not isinstance(key, str) for key in data):
            raise TypeError("artifact identity keys must be strings")
        unknown = set(data) - _ARTIFACT_IDENTITY_KEYS
        if unknown:
            raise ValueError(
                "artifact identity contains unknown field(s): "
                + ", ".join(sorted(unknown))
            )
        missing = _REQUIRED_ARTIFACT_IDENTITY_KEYS - set(data)
        if missing:
            raise ValueError(
                "artifact identity is missing required field(s): "
                + ", ".join(sorted(missing))
            )

        def _required_str(key: str) -> str:
            value = data[key]
            if not isinstance(value, str):
                raise TypeError(
                    f"artifact identity field {key!r} must be a string, got "
                    f"{type(value).__name__}"
                )
            return value

        algo = data.get("algo")
        if algo is not None and not isinstance(algo, str):
            raise TypeError(
                f"artifact identity field 'algo' must be a string or null, got "
                f"{type(algo).__name__}"
            )
        return cls(
            task=_required_str("task"),
            embodiment=_required_str("embodiment"),
            action_mode=_required_str("action_mode"),
            algo=algo,
        )


__all__ = [
    "ArtifactIdentity",
    "ArtifactIdentityDict",
    "DEFAULT_HF_ORG",
    "slugify_underscore",
    "stable_hash",
]
