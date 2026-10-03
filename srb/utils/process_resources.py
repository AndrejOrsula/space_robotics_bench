"""Low-overhead peak RSS sampling for one process and its descendants."""

from __future__ import annotations

import math
import os
import threading
from collections.abc import Mapping
from types import TracebackType
from typing import Any

import psutil


class ProcessTreeRssSampler:
    """Sample root and process-tree RSS without retaining process handles."""

    def __init__(
        self,
        *,
        process: Any | None = None,
        interval_s: float = 0.1,
    ) -> None:
        # NaN and infinity pass a plain ``<= 0`` test but break Event.wait().
        if not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError(
                "process RSS sampling interval must be positive and finite"
            )
        self._process = process if process is not None else psutil.Process(os.getpid())
        self._interval_s = float(interval_s)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self._learner_peak_rss_bytes = 0
        self._process_tree_peak_rss_bytes = 0
        self._sample_count = 0

    def __enter__(self) -> ProcessTreeRssSampler:
        self.start()
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.stop()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("process RSS sampler already started")
        self._stop.clear()
        self._sample_once()
        self._thread = threading.Thread(
            target=self._run,
            name="srb-process-tree-rss",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        thread = self._thread
        if thread is None:
            return
        self._stop.set()
        thread.join(timeout=max(1.0, self._interval_s * 4))
        if thread.is_alive():
            with self._lock:
                if self._error is None:
                    self._error = RuntimeError("process RSS sampler did not stop")
        else:
            self._thread = None

    def metrics(self) -> Mapping[str, int]:
        self._sample_once()
        with self._lock:
            error = self._error
            learner_peak = self._learner_peak_rss_bytes
            tree_peak = self._process_tree_peak_rss_bytes
            sample_count = self._sample_count
        if error is not None:
            raise RuntimeError("process-tree RSS measurement failed") from error
        if sample_count <= 0 or learner_peak <= 0 or tree_peak < learner_peak:
            raise RuntimeError("process-tree RSS measurement produced invalid metrics")
        return {
            "learner_peak_rss_bytes": learner_peak,
            "process_tree_peak_rss_bytes": tree_peak,
            "process_tree_rss_samples": sample_count,
        }

    def _run(self) -> None:
        while not self._stop.wait(self._interval_s):
            try:
                self._sample_once()
            except BaseException as error:
                with self._lock:
                    self._error = error
                self._stop.set()
                return

    def _sample_once(self) -> None:
        try:
            root_rss = int(self._process.memory_info().rss)
            descendants = self._process.children(recursive=True)
        except (psutil.NoSuchProcess, psutil.ZombieProcess) as error:
            raise RuntimeError(
                "learner process disappeared during RSS sampling"
            ) from error

        tree_rss = root_rss
        for process in descendants:
            try:
                tree_rss += int(process.memory_info().rss)
            except (psutil.NoSuchProcess, psutil.ZombieProcess):
                continue
        with self._lock:
            self._learner_peak_rss_bytes = max(
                self._learner_peak_rss_bytes,
                root_rss,
            )
            self._process_tree_peak_rss_bytes = max(
                self._process_tree_peak_rss_bytes,
                tree_rss,
            )
            self._sample_count += 1
