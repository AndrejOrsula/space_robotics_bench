"""Shared cleanup primitives for Dreamer runtimes.

They run named steps through errors, bound thread joins, and retain first-run
diagnostics for attachment to the primary exception.
"""

from __future__ import annotations

import threading
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field

CleanupStep = tuple[str, Callable[[], object]]
CleanupErrors = list[tuple[str, BaseException]]

MAX_DIAGNOSTIC_ENTRIES = 8
MAX_DIAGNOSTIC_CHARS = 240


def run_cleanup_steps(steps: Iterable[CleanupStep]) -> CleanupErrors:
    """Run every step; collect failures instead of aborting the sequence."""

    errors: CleanupErrors = []
    for name, action in steps:
        try:
            action()
        except BaseException as error:  # noqa: BLE001 - cleanup must continue
            errors.append((name, error))
    return errors


def close_method_step(name: str, owner: object) -> CleanupStep:
    """Step that calls ``owner.close()`` or ``owner.stop()``, whichever exists."""

    def _close() -> None:
        method = getattr(owner, "close", None)
        if not callable(method):
            method = getattr(owner, "stop", None)
        if not callable(method):
            raise RuntimeError(f"{name} exposes no close() or stop() method")
        method()

    return (name, _close)


def join_thread_bounded(
    name: str,
    thread: threading.Thread | None,
    *,
    timeout_s: float,
) -> BaseException | None:
    """Join ``thread`` within ``timeout_s``; report instead of blocking."""

    if thread is None or not thread.is_alive():
        return None
    if thread is threading.current_thread():
        return RuntimeError(f"Cannot join {name} from itself.")
    try:
        thread.join(timeout=timeout_s)
    except BaseException as error:  # noqa: BLE001 - reported, never raised here
        return error
    if thread.is_alive():
        return RuntimeError(f"{name} did not stop within {timeout_s:g} seconds")
    return None


def format_cleanup_diagnostics(
    errors: Sequence[tuple[str, BaseException]],
) -> tuple[str, ...]:
    """Bounded, single-line diagnostics suitable for exception notes."""

    return tuple(
        f"{name}: {type(error).__name__}: "
        f"{' '.join(str(error).splitlines())[:MAX_DIAGNOSTIC_CHARS]}"
        for name, error in errors[:MAX_DIAGNOSTIC_ENTRIES]
    )


def annotate_primary(primary: BaseException | None, errors: CleanupErrors) -> None:
    """Attach cleanup diagnostics to the exception that triggered cleanup."""

    if primary is None or not errors:
        return
    diagnostics = format_cleanup_diagnostics(errors)
    if hasattr(primary, "add_note"):
        primary.add_note("Dreamer cleanup diagnostics: " + "; ".join(diagnostics))
    try:
        setattr(primary, "cleanup_diagnostics", diagnostics)
    except Exception:  # noqa: BLE001 - some exception types reject attributes
        pass


@dataclass
class CleanupLedger:
    """Idempotent cleanup runner retaining first-run diagnostics."""

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _started: bool = False
    errors: CleanupErrors = field(default_factory=list)

    def run_once(self, steps: Iterable[CleanupStep]) -> CleanupErrors:
        """Run ``steps`` on the first call only; later calls return ``[]``."""

        with self._lock:
            if self._started:
                return []
            self._started = True
        self.errors = run_cleanup_steps(steps)
        return list(self.errors)

    @property
    def diagnostics(self) -> tuple[str, ...]:
        return format_cleanup_diagnostics(self.errors)


__all__ = [
    "CleanupErrors",
    "CleanupLedger",
    "CleanupStep",
    "annotate_primary",
    "close_method_step",
    "format_cleanup_diagnostics",
    "join_thread_bounded",
    "run_cleanup_steps",
]
