"""Write vector-environment episode statistics to the tuning JSONL side channel."""

from __future__ import annotations

import json
import os
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from gymnasium.core import Wrapper

if TYPE_CHECKING:
    from srb._typing import AnyEnv

_ENV_METRICS_PATH = "SRB_TUNE_METRICS_PATH"
_ENV_METRICS_EVERY = "SRB_TUNE_METRICS_EVERY"
_ENV_METRICS_MIN_INTERVAL_S = "SRB_TUNE_METRICS_MIN_INTERVAL_S"


def _to_1d_numpy(x: Any) -> np.ndarray | None:
    """Coerce a torch/numpy/scalar signal to a 1-D numpy array (or None)."""
    if x is None:
        return None
    if hasattr(x, "detach"):  # torch tensor
        x = x.detach().cpu().numpy()
    return np.asarray(x).reshape(-1)


def _extract_success(info: Any, n: int) -> np.ndarray | None:
    """Read per-env success flags from a top-level ``success`` key."""
    if not isinstance(info, dict):
        return None
    val = info.get("success")
    arr = _to_1d_numpy(val)
    if arr is None or arr.size == 0:
        return None
    if arr.size == 1 and n > 1:
        arr = np.repeat(arr, n)
    elif arr.size != n:
        raise ValueError(
            f'info["success"] has {arr.size} entries; expected 1 or one per env ({n})'
        )
    return arr.astype(bool)


class TuneReporterWrapper(Wrapper):
    """Accumulate per-env episode returns/lengths and flush JSONL records."""

    def __init__(
        self,
        env: AnyEnv,
        metrics_path: str,
        *,
        flush_every: int = 10,
        min_interval_s: float | None = None,
    ):
        super().__init__(env)
        self._metrics_path = metrics_path
        self._flush_every = max(int(flush_every), 1)
        if min_interval_s is None:
            min_interval_s = float(
                os.environ.get(_ENV_METRICS_MIN_INTERVAL_S, "2.0") or "2.0"
            )
        # Rate-limit writes when many vector episodes finish per step.
        self._min_interval_s = max(float(min_interval_s), 0.0)
        self._last_flush_t = float("-inf")
        # Size accumulators from the first reward vector.
        self._running_return: np.ndarray | None = None
        self._running_len: np.ndarray | None = None
        # Keep Torch accumulators on the reward device.
        self._t_return = None
        self._t_len = None
        self._total_env_steps = 0
        self._total_episodes = 0
        # Episodes since the last flush.
        self._buf_returns: list[float] = []
        self._buf_lens: list[float] = []
        self._buf_success: list[float] = []
        self._buf_has_success = False
        os.makedirs(os.path.dirname(self._metrics_path) or ".", exist_ok=True)

    def reset(self, **kwargs: Any):  # type: ignore[override]
        result = self.env.reset(**kwargs)
        if self._running_return is not None:
            self._running_return.fill(0.0)
            self._running_len.fill(0.0)
        if self._t_return is not None:
            self._t_return.zero_()
            self._t_len.zero_()
        return result

    def step(self, action):  # type: ignore[override]
        obs, reward, terminated, truncated, info = self.env.step(action)
        if hasattr(reward, "detach"):  # torch tensor → on-device accumulation
            self._step_torch(reward, terminated, truncated, info)
            return obs, reward, terminated, truncated, info

        r = _to_1d_numpy(reward)
        if r is None:
            return obs, reward, terminated, truncated, info
        n = r.shape[0]
        if self._running_return is None:
            self._running_return = np.zeros(n, dtype=np.float64)
            self._running_len = np.zeros(n, dtype=np.float64)

        self._total_env_steps += n
        self._running_return += r
        self._running_len += 1.0

        term = _to_1d_numpy(terminated)
        trunc = _to_1d_numpy(truncated)
        done = np.zeros(n, dtype=bool)
        if term is not None:
            done |= term.astype(bool)
        if trunc is not None:
            done |= trunc.astype(bool)

        if done.any():
            success = _extract_success(info, n)
            for i in np.nonzero(done)[0]:
                self._buf_returns.append(float(self._running_return[i]))
                self._buf_lens.append(float(self._running_len[i]))
                if success is not None:
                    self._buf_success.append(float(bool(success[i])))
                    self._buf_has_success = True
                self._running_return[i] = 0.0
                self._running_len[i] = 0.0
            self._total_episodes += int(done.sum())
            self._maybe_flush()

        return obs, reward, terminated, truncated, info

    def _step_torch(self, reward, terminated, truncated, info) -> None:
        """Accumulate on-device; transfer episode data only at boundaries."""
        import torch  # reward is a torch tensor

        r = reward.detach().reshape(-1)
        n = r.shape[0]
        if self._t_return is None:
            self._t_return = torch.zeros(n, dtype=torch.float64, device=r.device)
            self._t_len = torch.zeros(n, dtype=torch.float64, device=r.device)

        self._total_env_steps += n
        self._t_return += r
        self._t_len += 1.0

        done = None
        for sig in (terminated, truncated):
            if sig is None:
                continue
            b = torch.as_tensor(sig, device=r.device).reshape(-1).to(torch.bool)
            done = b if done is None else (done | b)

        if done is not None and bool(done.any()):  # the one host sync
            idx = torch.nonzero(done).flatten()
            rets = self._t_return[idx].cpu().tolist()
            lens = self._t_len[idx].cpu().tolist()
            success = _extract_success(info, n)
            for j, i in enumerate(idx.cpu().tolist()):
                self._buf_returns.append(float(rets[j]))
                self._buf_lens.append(float(lens[j]))
                if success is not None:
                    self._buf_success.append(float(bool(success[i])))
                    self._buf_has_success = True
            self._t_return[idx] = 0.0
            self._t_len[idx] = 0.0
            self._total_episodes += len(rets)
            self._maybe_flush()

    def _maybe_flush(self) -> None:
        if (
            len(self._buf_returns) >= self._flush_every
            and time.monotonic() - self._last_flush_t >= self._min_interval_s
        ):
            self._flush()

    def _flush(self) -> None:
        if not self._buf_returns:
            return
        returns = np.asarray(self._buf_returns, dtype=np.float64)
        lens = np.asarray(self._buf_lens, dtype=np.float64)
        record = {
            "env_steps": int(self._total_env_steps),
            "episodes_done": int(self._total_episodes),
            "ep_return_mean": float(returns.mean()),
            "ep_return_std": float(returns.std()),
            "ep_len_mean": float(lens.mean()),
            "success_rate": (
                float(np.mean(self._buf_success))
                if len(self._buf_success) == len(self._buf_returns)
                else None
            ),
        }
        with open(self._metrics_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record) + "\n")
        self._last_flush_t = time.monotonic()
        self._buf_returns.clear()
        self._buf_lens.clear()
        self._buf_success.clear()
        self._buf_has_success = False

    def close(self):  # type: ignore[override]
        try:
            self._flush()
        finally:
            super().close()

    @classmethod
    def wrap(cls, env: AnyEnv) -> AnyEnv:
        """Wrap *env* iff ``SRB_TUNE_METRICS_PATH`` is set; else return it as-is."""
        metrics_path = os.environ.get(_ENV_METRICS_PATH)
        if not metrics_path:
            return env
        every = int(os.environ.get(_ENV_METRICS_EVERY, "10") or "10")
        return cls(env, metrics_path, flush_every=every)


__all__ = ("TuneReporterWrapper",)
