"""Format per-environment and shared ROS topics and TF frames."""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral


@dataclass(frozen=True, slots=True)
class RosNamespace:
    num_envs: int
    _env_topic_prefixes: tuple[str, ...]
    _envs_topic_prefix: str | None
    _env_frame_prefixes: tuple[str, ...]

    @classmethod
    def for_env_count(
        cls,
        num_envs: int,
        *,
        omit_single_env_ns: bool = True,
    ) -> "RosNamespace":
        if isinstance(num_envs, bool) or not isinstance(num_envs, Integral):
            raise ValueError("ROS namespace environment count must be an integer")
        num_envs = int(num_envs)
        if num_envs <= 0:
            raise ValueError("ROS namespace requires at least one environment")

        if omit_single_env_ns and num_envs == 1:
            return cls(
                num_envs=num_envs,
                _env_topic_prefixes=("",),
                _envs_topic_prefix=None,
                _env_frame_prefixes=("srb",),
            )

        return cls(
            num_envs=num_envs,
            _env_topic_prefixes=tuple(f"env{i}/" for i in range(num_envs)),
            _envs_topic_prefix="envs/",
            _env_frame_prefixes=tuple(f"srb/env{i}" for i in range(num_envs)),
        )

    @property
    def omit_env_ns(self) -> bool:
        return self._envs_topic_prefix is None

    def env_topic(self, env_id: int, suffix: str) -> str:
        env_id = self._check_env_id(env_id)
        suffix = self._relative_name(suffix)
        return f"{self._env_topic_prefixes[env_id]}{suffix}"

    def env_topics(self, suffix: str) -> tuple[str, ...]:
        return tuple(self.env_topic(i, suffix) for i in range(self.num_envs))

    def envs_topic(self, suffix: str) -> str | None:
        if self._envs_topic_prefix is None:
            return None
        suffix = self._relative_name(suffix)
        return f"{self._envs_topic_prefix}{suffix}"

    def env_frame(self, env_id: int, suffix: str | None = None) -> str:
        env_id = self._check_env_id(env_id)
        prefix = self._env_frame_prefixes[env_id]
        if suffix is None or suffix == "":
            return prefix
        suffix = self._relative_name(suffix)
        return f"{prefix}/{suffix}"

    def env_frames(self, suffix: str | None = None) -> tuple[str, ...]:
        return tuple(self.env_frame(i, suffix) for i in range(self.num_envs))

    def _check_env_id(self, env_id: int) -> int:
        if isinstance(env_id, bool) or not isinstance(env_id, Integral):
            raise ValueError(f"Environment id must be an integer, got {env_id!r}")
        env_id = int(env_id)
        if env_id < 0 or env_id >= self.num_envs:
            raise IndexError(
                f"Environment id {env_id} outside ROS namespace with "
                f"{self.num_envs} environments"
            )
        return env_id

    @staticmethod
    def _relative_name(name: str) -> str:
        if not isinstance(name, str) or not name:
            raise ValueError("ROS namespace suffix must be a non-empty string")
        if name.startswith("/"):
            raise ValueError(f"ROS namespace suffix must be relative: {name!r}")
        return name


__all__ = ["RosNamespace"]
