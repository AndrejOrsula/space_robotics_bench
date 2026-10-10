"""Video recording for agent loops that never reset the gym environment."""

from __future__ import annotations

from typing import Any

from gymnasium.wrappers import RecordVideo


class RolloutRecordVideo(RecordVideo):
    """Record step-only rollouts while preserving reset-based Gymnasium behavior."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._reset_by_caller = False

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        self._reset_by_caller = True
        return super().reset(seed=seed, options=options)

    def step(self, action):
        if not self._reset_by_caller and not self.recording:
            self.episode_id += 1
            self.start_recording(f"{self.name_prefix}-episode-{self.episode_id}")
            self._capture_frame()
        return super().step(action)


__all__ = ["RolloutRecordVideo"]
