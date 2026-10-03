"""Owned-process lifecycle and bounded group cleanup for Python launchers."""

from __future__ import annotations

import contextlib
import logging
import math
import os
import queue
import select
import signal
import subprocess
import sys
import threading
import time
import weakref
from collections.abc import Callable, Iterable, Mapping, Sequence
from concurrent.futures import Future
from dataclasses import dataclass
from enum import Enum
from types import FrameType, TracebackType
from typing import IO, Any, Final, Literal, NoReturn, Protocol, Self, cast

from srb.utils.failures import attempt_cleanup

_LOGGER = logging.getLogger(__name__)

type CommandArg = str | os.PathLike[str]
type _OwnedPopen = subprocess.Popen[bytes] | subprocess.Popen[str]
type _OutputTarget = int | IO[Any] | None
type _Popen = Callable[..., _OwnedPopen]
type _SignalHandler = (
    signal.Handlers | int | Callable[[int, FrameType | None], Any] | None
)


class ProcessKind(str, Enum):
    """Workload class, made explicit at every supported launch seam."""

    FINITE = "finite"
    INTERACTIVE = "interactive"
    MONITORED = "monitored"


class ProcessStatus(str, Enum):
    """Observed terminal lifecycle state of an owned process."""

    EXITED = "exited"
    TIMED_OUT = "timed_out"
    INTERRUPTED = "interrupted"
    TERMINATED = "terminated"
    KILLED = "killed"
    CLEANUP_FAILED = "cleanup_failed"


@dataclass(frozen=True, slots=True)
class ProcessResult:
    """Structured status, output, and cleanup evidence for one owned process."""

    args: tuple[str, ...]
    kind: ProcessKind
    pid: int
    process_group_id: int | None
    status: ProcessStatus
    returncode: int | None
    stdout: str | None
    stderr: str | None
    error: str | None
    cause: ProcessStatus | None
    first_signal: int | None
    term_sent: bool
    kill_sent: bool
    reaped: bool
    cleanup_complete: bool


class ProcessCleanupOwner(Protocol):
    """Retryable public owner for a process cleanup operation."""

    @property
    def result(self) -> ProcessResult | None:
        """Return the latest explicit cleanup result, if cleanup was attempted."""

    def cleanup(self) -> ProcessResult:
        """Retry cleanup and return its explicit result."""


class ProcessSpawnError(RuntimeError):
    """An owned process could not be created."""

    def __init__(self, command: tuple[str, ...], error: OSError) -> None:
        self.command = command
        self.error = error
        super().__init__(f"failed to spawn {command[0]!r}: {error}")


class ProcessOwnershipError(RuntimeError):
    """A compatibility handle does not own the group it would need to clean."""

    def __init__(
        self,
        pid: int | None,
        process_group_id: int | None,
        *,
        reason: str | None = None,
    ) -> None:
        self.pid = pid
        self.process_group_id = process_group_id
        if reason is None and pid is None:
            reason = "process ownership handle is stale or foreign"
        elif reason is None:
            reason = (
                f"process {pid} is not its process-group leader "
                f"(group={process_group_id!r}); descendant ownership is unproven"
            )
        super().__init__(reason)


class ProcessCleanupError(RuntimeError):
    """A context owner could not finish bounded group cleanup."""

    def __init__(self, result: ProcessResult) -> None:
        self.result = result
        super().__init__(result.error or "owned process cleanup failed")


class ProcessStartupCleanupError(RuntimeError):
    """Startup failed while cleanup is unproven; see :attr:`cause`.

    Retry :attr:`cleanup_owner` before releasing dependent resources.
    """

    def __init__(
        self,
        cause: BaseException,
        *,
        cleanup_owner: ProcessCleanupOwner,
        cleanup_error: BaseException | None = None,
    ) -> None:
        self.cause = self.error = cause
        self.cleanup_owner = self.owner = cleanup_owner
        self.cleanup_error = cleanup_error
        detail = f"process startup cleanup remains incomplete: {cause}"
        if cleanup_error is not None:
            detail = f"{detail}; cleanup attempt failed: {cleanup_error}"
        super().__init__(detail)

    @property
    def result(self) -> ProcessResult | None:
        """Return the owner's latest explicit cleanup result."""
        return self.cleanup_owner.result

    def cleanup(self) -> ProcessResult:
        """Retry the retained startup cleanup owner."""
        return self.cleanup_owner.cleanup()


class ProcessPlatformError(RuntimeError):
    """This platform cannot satisfy the owned-process supervision contract."""


#: TERM->KILL grace for short-lived, non-interactive helper processes.
DEFAULT_TERM_GRACE_S: Final[float] = 0.5
#: TERM->KILL grace for interactive Isaac Sim / GUI launches; a second parent
#: signal can shorten it via ``OwnedMonitoredProcess.request_escalation``.
INTERACTIVE_TERM_GRACE_S: Final[float] = 30.0
#: KILL->reap grace for interactive launches; Kit can take seconds to be reaped.
INTERACTIVE_KILL_GRACE_S: Final[float] = 5.0
#: Environment override for :data:`INTERACTIVE_TERM_GRACE_S` (seconds).
INTERACTIVE_TERM_GRACE_ENV: Final[str] = "SRB_INTERACTIVE_TERM_GRACE_S"


def interactive_term_grace_s() -> float:
    """Return the interactive TERM->KILL grace, honouring a valid env override."""
    try:
        value = float(os.environ.get(INTERACTIVE_TERM_GRACE_ENV, ""))
    except ValueError:
        return INTERACTIVE_TERM_GRACE_S
    return value if math.isfinite(value) and value > 0 else INTERACTIVE_TERM_GRACE_S


@dataclass(frozen=True, slots=True)
class _LinuxProcessIdentity:
    """PID-reuse-resistant `/proc` identity checked before every signal."""

    pid: int
    start_time: int
    session_id: int
    process_group_id: int


@dataclass(frozen=True, slots=True)
class _LinuxProcEntry:
    identity: _LinuxProcessIdentity
    parent_pid: int
    state: str


class _RunOwnedSignal(BaseException):
    """Internal control transfer for a signal received by ``run_owned``."""

    def __init__(self, signum: signal.Signals, frame: FrameType | None) -> None:
        self.signum = signum
        self.frame = frame
        super().__init__(int(signum))


class _GroupReaper(Protocol):
    def __call__(self, process_group_id: int, /, *, timeout_s: float) -> bool: ...


_GRACE_NAMES: Final = ("term_grace_s", "kill_grace_s", "reap_grace_s")
_PR_SET_CHILD_SUBREAPER = 36
_SUBREAPER_LOCK = threading.Lock()
_SUBREAPER_ATTEMPTED = False
_SUBREAPER_ENABLED = False
#: A ``/proc`` read of a process that exits mid-scan fails with ENOENT or
#: ESRCH. That process is gone, not ambiguous; any other error stays ambiguous.
_VANISHED_PROCESS_ERRORS: Final = (FileNotFoundError, ProcessLookupError)
_LINUX_STARTUP_GATE_TIMEOUT_S = 2.0
_LINUX_STARTUP_GATE_GO = b"G"
_LINUX_STARTUP_GATE_ABORT = b"A"
# The gated child arms PR_SET_PDEATHSIG (which survives exec) before checking
# its parent, waits for the owner's GO after the owner bound its identity, and
# reports an exec failure as ``errno\0message`` on the status pipe.
_LINUX_EXEC_BOOTSTRAP = r"""
import errno
import os
import sys

control_fd = int(sys.argv[1])
status_fd = int(sys.argv[2])
parent_pid = int(sys.argv[3])
argv = sys.argv[4:]


def write_status(error_number, message):
    payload = str(error_number).encode("ascii") + b"\0"
    payload += str(message).encode("utf-8", "replace")[:4096]
    try:
        while payload and (written := os.write(status_fd, payload)) > 0:
            payload = payload[written:]
    except OSError:
        pass


death_signal_error = None
try:
    import ctypes

    prctl = ctypes.CDLL(None, use_errno=True).prctl
    prctl.argtypes = [ctypes.c_int] + [ctypes.c_ulong] * 4
    prctl.restype = ctypes.c_int
    if prctl(1, 9, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG, SIGKILL
        error_number = ctypes.get_errno() or errno.EIO
        death_signal_error = (error_number, os.strerror(error_number))
except BaseException as error:
    death_signal_error = (errno.ENOSYS, "cannot arm parent-death signal: " + str(error))
if os.getppid() != parent_pid or os.read(control_fd, 1) != b"G":
    os._exit(125)
if death_signal_error is not None:
    write_status(*death_signal_error)
    os._exit(127)
try:
    os.close(control_fd)
    os.set_inheritable(status_fd, False)
    os.execvpe(argv[0], argv, os.environ)
except OSError as error:
    write_status(error.errno if error.errno is not None else errno.EIO, error)
except BaseException as error:
    write_status(errno.EIO, error)
os._exit(127)
"""


def _is_linux() -> bool:
    return sys.platform.startswith("linux")


def _can_observe_leader_without_reaping() -> bool:
    names = ("waitid", "P_PID", "WEXITED", "WNOHANG", "WNOWAIT")
    return _is_linux() and all(hasattr(os, name) for name in names)


def _validate_lifecycle(
    kind: ProcessKind, deadline_s: float | None, graces: tuple[float, float, float]
) -> None:
    checks: list[tuple[str, object]] = list(zip(_GRACE_NAMES, graces))
    if deadline_s is not None:
        checks.insert(0, ("deadline", deadline_s))
    for name, value in checks:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(
                f"{name} must be a finite positive number (bool is invalid)"
            )
    if kind is ProcessKind.FINITE and deadline_s is None:
        raise ValueError("finite processes require an explicit bounded deadline")
    if kind is ProcessKind.INTERACTIVE and deadline_s is not None:
        raise ValueError("interactive processes must not have an automatic deadline")


def _as_text(value: str | bytes | None) -> str | None:
    # Output is captured as bytes so an undecodable byte cannot fail a run;
    # newline translation keeps the text-mode result shape.
    if isinstance(value, bytes):
        text = value.decode("utf-8", errors="replace")
        return text.replace("\r\n", "\n").replace("\r", "\n")
    return value


def _join_output(chunks: Sequence[str | bytes]) -> str:
    if all(isinstance(chunk, str) for chunk in chunks):
        return "".join(cast(Sequence[str], chunks))
    joined = b"".join(
        chunk if isinstance(chunk, bytes) else chunk.encode("utf-8") for chunk in chunks
    )
    return _as_text(joined) or ""


def _close_fd(fd: int) -> None:
    with contextlib.suppress(OSError):
        os.close(fd)


def _write_fd_all(fd: int, payload: bytes) -> None:
    remaining = memoryview(payload)
    while remaining:
        written = os.write(fd, remaining)
        if written <= 0:
            raise OSError("startup gate write made no progress")
        remaining = remaining[written:]


def _abort_linux_startup_gate(
    process: _OwnedPopen, control_fd: int, *, wait_s: float
) -> bool:
    """Abort an unreleased gate; only a reaped leader may drop its pipes."""
    try:
        _write_fd_all(control_fd, _LINUX_STARTUP_GATE_ABORT)
    except OSError:
        pass
    finally:
        _close_fd(control_fd)
    try:
        process.wait(timeout=wait_s)
    except (OSError, subprocess.TimeoutExpired):
        with contextlib.suppress(OSError):
            process.kill()
        try:
            process.wait(timeout=wait_s)
        except (OSError, subprocess.TimeoutExpired):
            return False
    for pipe in (process.stdin, process.stdout, process.stderr):
        if pipe is not None:
            with contextlib.suppress(OSError):
                pipe.close()
    return True


def _read_linux_exec_error(status_fd: int, deadline_at: float | None) -> OSError | None:
    deadline = time.monotonic() + _LINUX_STARTUP_GATE_TIMEOUT_S
    if deadline_at is not None:
        deadline = min(deadline, deadline_at)
    payload = bytearray()
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([status_fd], [], [], remaining)[0]:
            return TimeoutError("Linux startup exec acknowledgement timed out")
        chunk = os.read(status_fd, 4096)
        if not chunk:
            break
        payload.extend(chunk)
        if len(payload) > 8192:
            return OSError("Linux startup exec error exceeded protocol limit")
    if not payload:
        return None
    encoded_errno, separator, encoded_message = bytes(payload).partition(b"\0")
    if not separator:
        return OSError("malformed Linux startup exec error")
    try:
        error_number = int(encoded_errno.decode("ascii"))
    except ValueError:
        return OSError("malformed Linux startup exec errno")
    return OSError(error_number, encoded_message.decode("utf-8", "replace"))


@dataclass(frozen=True, slots=True)
class _Launch:
    """Lifecycle parameters shared by a launch and its cleanup owners."""

    args: tuple[str, ...]
    kind: ProcessKind
    capture_output: bool
    graces: tuple[float, float, float]
    group_reaper: _GroupReaper | None = None

    def owner(
        self,
        process: _OwnedPopen,
        identity: _LinuxProcessIdentity | None,
        *,
        deadline_s: float | None = None,
        factory: type[OwnedMonitoredProcess] | None = None,
    ) -> OwnedMonitoredProcess:
        # Without an identity the owner has direct-child authority only.
        group_reaper = self.group_reaper
        if group_reaper is None:
            group_reaper = _reap_owned_group_children
        term_grace_s, kill_grace_s, reap_grace_s = self.graces
        return (factory or OwnedMonitoredProcess)(
            process,
            args=self.args,
            kind=self.kind,
            deadline_s=deadline_s,
            capture_output=self.capture_output,
            term_grace_s=term_grace_s,
            kill_grace_s=kill_grace_s,
            reap_grace_s=reap_grace_s,
            process_group_id=None if identity is None else identity.process_group_id,
            process_identity=identity,
            group_reaper=None if identity is None else group_reaper,
        )


class _LifetimeSpawner:
    """Fork owned children from a thread that lives as long as the process.

    ``PR_SET_PDEATHSIG`` fires when the *thread* that forked the child exits,
    so only the main thread forks directly; other threads use one persistent
    daemon thread.
    """

    def __init__(self) -> None:
        self.reset_after_fork()

    def reset_after_fork(self) -> None:
        self._lock = threading.Lock()
        self._requests: queue.SimpleQueue[tuple[Callable[[], Any], Future[Any]]] = (
            queue.SimpleQueue()
        )
        self._thread: threading.Thread | None = None

    def run[T](self, factory: Callable[[], T]) -> T:
        if threading.current_thread() is threading.main_thread():
            return factory()
        future: Future[T] = Future()
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(
                    target=self._serve, name="srb-owned-process-spawner", daemon=True
                )
                self._thread.start()
            self._requests.put((factory, future))
        return future.result()

    def _serve(self) -> None:
        while True:
            factory, future = self._requests.get()
            try:
                future.set_result(factory())
            except BaseException as error:  # noqa: BLE001
                future.set_exception(error)


_LIFETIME_SPAWNER = _LifetimeSpawner()
if hasattr(os, "register_at_fork"):
    os.register_at_fork(after_in_child=_LIFETIME_SPAWNER.reset_after_fork)


def _spawn_linux_identity_gated(
    launch: _Launch, popen: _Popen, deadline_at: float | None
) -> tuple[_OwnedPopen, _LinuxProcessIdentity]:
    """Spawn behind a bootstrap gate released only after identity binding."""
    args = launch.args
    control_read_fd, control_write_fd = os.pipe()
    try:
        status_read_fd, status_write_fd = os.pipe()
    except BaseException:
        _close_fd(control_read_fd)
        _close_fd(control_write_fd)
        raise
    try:
        fds = (control_read_fd, status_write_fd)
        argv = [sys.executable, "-I", "-S", "-c", _LINUX_EXEC_BOOTSTRAP]
        argv += [*map(str, (*fds, os.getpid())), *args]
        try:
            process = _LIFETIME_SPAWNER.run(
                lambda: popen(argv, start_new_session=True, pass_fds=fds)
            )
        except OSError as error:
            raise ProcessSpawnError(args, error) from error
        finally:
            _close_fd(control_read_fd)
            _close_fd(status_write_fd)

        try:
            identity = _read_linux_process_identity(process.pid)
            if (
                identity is None
                or identity.process_group_id != process.pid
                or identity.session_id != process.pid
            ):
                raise ProcessOwnershipError(
                    process.pid,
                    None if identity is None else identity.process_group_id,
                    reason="could not bind the gated process to a reliable Linux "
                    "PID/start-time/session/group identity",
                )
        except BaseException as error:
            abort_reaped, abort_error = attempt_cleanup(
                lambda: _abort_linux_startup_gate(
                    process, control_write_fd, wait_s=launch.graces[1]
                )
            )
            control_write_fd = -1
            if abort_reaped is True:
                raise
            raise ProcessStartupCleanupError(
                error,
                cleanup_owner=launch.owner(process, None),
                cleanup_error=abort_error,
            ) from error
        bound_identity = identity

        def fail_released(
            startup_error: BaseException, cause: BaseException | None = None
        ) -> NoReturn:
            # The gate may be released, so only the identity-bound owner can
            # prove descendant cleanup; otherwise the failure stays retryable.
            nonlocal control_write_fd
            owner = launch.owner(process, bound_identity)
            gate_fd, control_write_fd = control_write_fd, -1

            def release_gate() -> None:
                if gate_fd >= 0:
                    with contextlib.suppress(OSError):
                        _write_fd_all(gate_fd, _LINUX_STARTUP_GATE_ABORT)

            def finish_cleanup() -> ProcessResult:
                # Observe an already-exited child first so a vanished identity
                # is not mistaken for a foreign group.
                owner.poll()
                return owner.cleanup()

            result = None
            _, cleanup_error = attempt_cleanup(release_gate)
            _close_fd(gate_fd)
            if cleanup_error is None:
                result, cleanup_error = attempt_cleanup(finish_cleanup)
            if result is not None and result.cleanup_complete:
                if cause is None:
                    raise startup_error
                raise startup_error from cause
            raise ProcessStartupCleanupError(
                startup_error, cleanup_owner=owner, cleanup_error=cleanup_error
            ) from startup_error

        _, write_error = attempt_cleanup(
            lambda: _write_fd_all(control_write_fd, _LINUX_STARTUP_GATE_GO)
        )
        if write_error is not None:
            if not isinstance(write_error, OSError):
                fail_released(write_error)
            startup_error = ProcessOwnershipError(
                process.pid,
                identity.process_group_id,
                reason=f"failed to release identity-bound Linux process: {write_error}",
            )
            startup_error.__cause__ = write_error
            fail_released(startup_error, write_error)
        _close_fd(control_write_fd)
        control_write_fd = -1

        exec_error, read_error = attempt_cleanup(
            lambda: _read_linux_exec_error(status_read_fd, deadline_at)
        )
        if read_error is not None:
            fail_released(read_error)
        if exec_error is not None:
            if (
                isinstance(exec_error, TimeoutError)
                and deadline_at is not None
                and time.monotonic() >= deadline_at
            ):
                # The deadline is spent: the owner's wait() performs the same
                # retryable group cleanup as a workload timeout.
                return process, identity
            fail_released(ProcessSpawnError(args, exec_error), exec_error)
        return process, identity
    finally:
        if control_write_fd >= 0:
            _close_fd(control_write_fd)
        _close_fd(status_read_fd)


def _read_proc_stat(pid: int) -> tuple[int, list[bytes]]:
    with open(f"/proc/{pid}/stat", "rb") as stream:
        stat = stream.read()
    pid_and_name, fields_text = stat.rsplit(b") ", 1)
    return int(pid_and_name.split(b" ", 1)[0]), fields_text.split()


def _read_linux_process_identity(pid: int) -> _LinuxProcessIdentity | None:
    # ``None`` means the entry was proved absent. An unreadable or malformed
    # entry is ownership ambiguity and raises before any signal uses the PID.
    if not _is_linux() or pid <= 0:
        return None
    prefix = f"Linux process identity for PID {pid} is ambiguous: "
    try:
        observed_pid, fields = _read_proc_stat(pid)
        process_group_id = int(fields[2])
        session_id = int(fields[3])
        start_time = int(fields[19])
    except _VANISHED_PROCESS_ERRORS:
        return None
    except OSError as error:
        raise ProcessOwnershipError(
            pid, None, reason=f"{prefix}cannot read /proc entry: {error}"
        ) from error
    except (IndexError, ValueError) as error:
        raise ProcessOwnershipError(
            pid, None, reason=f"{prefix}the /proc stat entry is malformed"
        ) from error
    if min(process_group_id, session_id) <= 0 or start_time < 0 or observed_pid != pid:
        return None
    return _LinuxProcessIdentity(observed_pid, start_time, session_id, process_group_id)


def _parse_proc_entry(pid: int, fields: list[bytes]) -> _LinuxProcEntry:
    state, parent_pid = fields[0].decode("ascii"), int(fields[1])
    identity = _LinuxProcessIdentity(
        pid=pid,
        process_group_id=int(fields[2]),
        session_id=int(fields[3]),
        start_time=int(fields[19]),
    )
    return _LinuxProcEntry(identity, parent_pid, state)


def _read_linux_proc_entry(pid: int) -> _LinuxProcEntry | None:
    """Read identity, parent and state; ``None`` only for a proved absence."""
    try:
        observed_pid, fields = _read_proc_stat(pid)
        entry = _parse_proc_entry(pid, fields)
    except _VANISHED_PROCESS_ERRORS:
        return None
    except (OSError, IndexError, UnicodeError, ValueError) as error:
        raise ProcessOwnershipError(
            pid, None, reason=f"Linux process entry for PID {pid} is ambiguous: {error}"
        ) from error
    return entry if observed_pid == pid else None


def _scan_proc[T](parse: Callable[[int, list[bytes]], T]) -> dict[int, T] | None:
    """Parse live ``/proc/<pid>/stat`` entries; return ``None`` if ambiguous.

    Transient ambiguity can be retried; persistent ambiguity fails closed.
    """
    for attempt in range(3):
        if attempt:
            time.sleep(0.001)
        parsed: dict[int, T] = {}
        try:
            with os.scandir("/proc") as listing:
                pids = [int(item.name) for item in listing if item.name.isdigit()]
            for pid in pids:
                try:
                    observed_pid, fields = _read_proc_stat(pid)
                    if observed_pid == pid:
                        parsed[pid] = parse(pid, fields)
                except _VANISHED_PROCESS_ERRORS:
                    continue
        except (OSError, IndexError, UnicodeError, ValueError):
            continue
        return parsed
    return None


def _process_group_members(process_group_id: int) -> set[int] | None:
    """Read Linux group membership; ``None`` when it stays uninspectable."""
    groups = _scan_proc(lambda _, fields: int(fields[2])) if _is_linux() else None
    if groups is None:
        return None
    return {pid for pid, group in groups.items() if group == process_group_id}


def _owned_tree_escapees(
    leader_pid: int, process_group_id: int
) -> tuple[_LinuxProcessIdentity, ...] | None:
    """Return descendants of ``leader_pid`` outside ``process_group_id``."""
    entries = _scan_proc(_parse_proc_entry)
    if entries is None:
        return None
    children: dict[int, list[int]] = {}
    for pid, entry in entries.items():
        children.setdefault(entry.parent_pid, []).append(pid)
    escaped: list[_LinuxProcessIdentity] = []
    seen = {leader_pid}
    frontier = [leader_pid]
    while frontier:
        for child_pid in children.get(frontier.pop(), ()):
            if child_pid in seen:
                continue
            seen.add(child_pid)
            frontier.append(child_pid)
            identity = entries[child_pid].identity
            if identity.process_group_id != process_group_id:
                escaped.append(identity)
    return tuple(escaped)


def _enable_child_subreaper() -> bool:
    """Adopt orphaned descendants so the supervisor can wait for them.

    This process-wide setting may adopt unrelated children. Reaping stays
    scoped to a validated owned group; neither ``waitpid(-1, ...)`` nor a raw
    group identifier proves ownership.
    """
    global _SUBREAPER_ATTEMPTED, _SUBREAPER_ENABLED
    if not _is_linux():
        return False
    with _SUBREAPER_LOCK:
        if _SUBREAPER_ATTEMPTED:
            return _SUBREAPER_ENABLED
        _SUBREAPER_ATTEMPTED = True
        try:
            import ctypes

            prctl = ctypes.CDLL(None, use_errno=True).prctl
            prctl.argtypes = [ctypes.c_int] + [ctypes.c_ulong] * 4
            prctl.restype = ctypes.c_int
            _SUBREAPER_ENABLED = prctl(_PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) == 0
        except (AttributeError, OSError, TypeError):
            _SUBREAPER_ENABLED = False
        return _SUBREAPER_ENABLED


def _reap_owned_group_children(process_group_id: int, *, timeout_s: float) -> bool:
    """Prove that no waitable child remains in one owned process group.

    A zero from ``waitpid(..., WNOHANG)`` proves nothing. Timeout raises for
    the caller to record structurally.
    """
    if not _enable_child_subreaper():
        raise RuntimeError(
            "child subreaper unavailable; cannot prove owned descendant reaping"
        )
    deadline = time.monotonic() + timeout_s
    while True:
        members = _process_group_members(process_group_id)
        if members is None:
            raise RuntimeError(
                "cannot inspect owned process-group membership for descendant reaping"
            )
        if not (descendants := members - {process_group_id}):
            return True
        for descendant_pid in descendants:
            # A non-child is never reaped here; it stays a member and times out.
            with contextlib.suppress(ChildProcessError):
                os.waitpid(descendant_pid, os.WNOHANG)
        if time.monotonic() >= deadline:
            raise TimeoutError(
                "timed out waiting to reap descendants in owned process group "
                f"{process_group_id}"
            )
        time.sleep(0.01)


class _ProcessOwnershipRegistry:
    """Bind opaque cancellation handles to live supervisor owners."""

    def __init__(self) -> None:
        self._owners: weakref.WeakValueDictionary[object, OwnedMonitoredProcess] = (
            weakref.WeakValueDictionary()
        )

    def register(self, owner: OwnedMonitoredProcess) -> OwnedProcessHandle:
        token = object()
        self._owners[token] = owner
        return OwnedProcessHandle(self, token)

    def resolve(self, handle: OwnedProcessHandle) -> OwnedMonitoredProcess:
        owner = None
        if isinstance(handle, OwnedProcessHandle) and handle._registry is self:
            owner = self._owners.get(handle._token)
        if owner is None:
            raise ProcessOwnershipError(None, None)  # stale or foreign handle
        return owner


class OwnedProcessHandle:
    """Opaque, owner-bound cancellation capability for one supervised launch."""

    __slots__ = ("_registry", "_token")

    def __init__(self, registry: _ProcessOwnershipRegistry, token: object) -> None:
        self._registry = registry
        self._token = token

    def cancel(self, *, first_signal: signal.Signals = signal.SIGTERM) -> ProcessResult:
        """Cancel through TERM->KILL escalation unless KILL is explicit."""
        return cancel_owned_process(self, first_signal=first_signal)


_OWNERSHIP_REGISTRY = _ProcessOwnershipRegistry()


def cancel_owned_process(
    handle: OwnedProcessHandle, *, first_signal: signal.Signals = signal.SIGTERM
) -> ProcessResult:
    """Cancel one owned process without accepting a raw PID or process-group ID."""
    return _OWNERSHIP_REGISTRY.resolve(handle).cleanup(first_signal=first_signal)


def cancel_owned_processes(
    handles: Iterable[OwnedProcessHandle],
    *,
    first_signal: signal.Signals = signal.SIGTERM,
) -> tuple[ProcessResult, ...]:
    """Cancel a validated collection of owner-bound process handles."""
    validated = tuple(_OWNERSHIP_REGISTRY.resolve(handle) for handle in handles)
    return tuple(owner.cleanup(first_signal=first_signal) for owner in validated)


class OwnedMonitoredProcess:
    """The public owner for one process and its descendant process group."""

    def __init__(
        self,
        process: _OwnedPopen,
        *,
        args: tuple[str, ...],
        kind: ProcessKind,
        deadline_s: float | None,
        capture_output: bool,
        term_grace_s: float,
        kill_grace_s: float,
        reap_grace_s: float,
        process_group_id: int | None,
        process_identity: _LinuxProcessIdentity | None,
        group_reaper: _GroupReaper | None,
    ) -> None:
        self._process = process
        self._args = args
        self._kind = kind
        self._capture_output = capture_output
        self._term_grace_s = term_grace_s
        self._kill_grace_s = kill_grace_s
        self._reap_grace_s = reap_grace_s
        self._process_group_id = process_group_id
        self._process_identity = process_identity
        self._group_reaper = group_reaper
        self._deadline_at = (
            None if deadline_s is None else time.monotonic() + deadline_s
        )
        self._startup_expired = False
        self._stdout: str | None = None
        self._stderr: str | None = None
        self._output_collected = not capture_output
        self._stdout_chunks: list[str | bytes] = []
        self._stderr_chunks: list[str | bytes] = []
        self._output_threads: list[threading.Thread] = []
        self._output_errors: list[str] = []
        self._result: ProcessResult | None = None
        self._leader_reaped = False
        self._leader_exit_observed = False
        self._leader_returncode: int | None = None
        self._cleanup_lock = threading.RLock()
        self._escalation_requested = threading.Event()
        self._ownership_handle = _OWNERSHIP_REGISTRY.register(self)
        # Exactly one reader per pipe: drainer threads on the Linux poll path
        # (which never touches the pipes), else ``communicate()``. Running both
        # nondeterministically truncates captured output.
        if capture_output and self._observes_leader_without_reaping():
            for stream, chunks, name in (
                (process.stdout, self._stdout_chunks, "owned-stdout-reader"),
                (process.stderr, self._stderr_chunks, "owned-stderr-reader"),
            ):
                if stream is not None and hasattr(stream, "read"):
                    thread = threading.Thread(
                        target=self._drain_output,
                        args=(stream, chunks),
                        name=name,
                        daemon=True,
                    )
                    self._output_threads.append(thread)
                    thread.start()

    @classmethod
    def spawn(
        cls,
        cmd: Sequence[CommandArg],
        *,
        kind: ProcessKind,
        deadline_s: float | None,
        capture_output: bool = False,
        cwd: str | os.PathLike[str] | None = None,
        env: Mapping[str, str] | None = None,
        term_grace_s: float = 0.5,
        kill_grace_s: float = 0.5,
        reap_grace_s: float = 0.5,
        group_reaper: _GroupReaper | None = None,
        stdout: _OutputTarget = None,
        stderr: _OutputTarget = None,
    ) -> OwnedMonitoredProcess:
        """Spawn an argv process; startup time counts toward its deadline.

        An expired finite launch still returns an owner. Caller-owned stream
        targets cannot be combined with ``capture_output``.
        """
        started_at = time.monotonic()
        if capture_output and (stdout is not None or stderr is not None):
            raise ValueError("stdout/stderr redirection excludes capture_output")
        graces = (term_grace_s, kill_grace_s, reap_grace_s)
        _validate_lifecycle(kind, deadline_s, graces)
        owns_group = os.name == "posix"
        if owns_group and not _is_linux():
            # Group cleanup is proven by reading membership *after* the leader
            # is reaped, which only Linux ``/proc`` offers.
            raise ProcessPlatformError(
                "the owned-process contract requires Linux on POSIX: platform "
                f"{sys.platform!r} cannot inspect process-group membership after "
                "the group leader is reaped, so bounded group cleanup cannot be "
                "proven"
            )
        deadline_at = None if deadline_s is None else started_at + deadline_s
        args = tuple(os.fsdecode(os.fspath(arg)) for arg in cmd)
        if not args:
            raise ValueError("owned process command must not be empty")
        launch = _Launch(args, kind, capture_output, graces, group_reaper)
        pipe = subprocess.PIPE if capture_output else None

        def popen(argv: list[str], **extra: Any) -> _OwnedPopen:
            return subprocess.Popen(
                argv,
                cwd=cwd,
                env=None if env is None else dict(env),
                stdout=pipe or stdout,
                stderr=pipe or stderr,
                **extra,
            )

        identity: _LinuxProcessIdentity | None = None
        if owns_group:
            _enable_child_subreaper()
            process, identity = _spawn_linux_identity_gated(launch, popen, deadline_at)
        else:
            try:
                process = popen(list(args))
            except OSError as error:
                raise ProcessSpawnError(args, error) from error
        owner = launch.owner(process, identity, deadline_s=deadline_s, factory=cls)
        owner._deadline_at = deadline_at
        owner._startup_expired = owner.deadline_expired()
        return owner

    @property
    def pid(self) -> int:
        return self._process.pid

    @property
    def process_group_id(self) -> int | None:
        return self._process_group_id

    @property
    def ownership_handle(self) -> OwnedProcessHandle:
        """Return the opaque capability used by a supervising caller."""
        return self._ownership_handle

    @property
    def result(self) -> ProcessResult | None:
        return self._result

    def request_escalation(self) -> None:
        """Cut any in-flight TERM grace short; signal-handler safe."""
        self._escalation_requested.set()

    def _observes_leader_without_reaping(self) -> bool:
        return (
            self.process_group_id is not None and _can_observe_leader_without_reaping()
        )

    def _drain_output(self, stream: object, chunks: list[str | bytes]) -> None:
        # Bytes are decoded once after EOF. A read error is recorded and the
        # pipe is still drained (discarding) so the child never blocks on a
        # full pipe; a second error stops the drain.
        read = getattr(stream, "read1", None) or getattr(stream, "read")
        failed = False
        while True:
            try:
                chunk = read(65536)
            except BaseException as error:  # noqa: BLE001
                if failed:
                    return
                failed = True
                self._output_errors.append(
                    f"failed to drain captured output: {type(error).__name__}: {error}"
                )
                continue
            if not chunk:
                return
            if not failed:
                chunks.append(chunk)

    def _mark_leader_reaped(self) -> None:
        self._leader_reaped = self._leader_exit_observed = True
        self._leader_returncode = self._process.returncode

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> Literal[False]:
        result = self.cleanup(
            status=None if exc_type is None else ProcessStatus.INTERRUPTED
        )
        if not result.cleanup_complete:
            if exc_value is not None:
                raise ProcessCleanupError(result) from exc_value
            raise ProcessCleanupError(result)
        return False

    def poll(self) -> int | None:
        """Return the leader return code without surrendering group ownership."""
        if self._observes_leader_without_reaping():
            return self._poll_without_reaping()
        returncode = self._process.poll()
        if returncode is not None:
            self._mark_leader_reaped()
        return returncode

    def _poll_without_reaping(self) -> int | None:
        if self._leader_exit_observed:
            return self._leader_returncode
        try:
            info = os.waitid(os.P_PID, self.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        except ChildProcessError:
            self._mark_leader_reaped()
            return self._leader_returncode
        if info is None or info.si_pid == 0:
            return None
        self._leader_exit_observed = True
        status = int(info.si_status)
        self._leader_returncode = status if info.si_code == os.CLD_EXITED else -status
        return self._leader_returncode

    def deadline_expired(self) -> bool:
        """Return whether this owner's absolute workload deadline elapsed."""
        return self._deadline_at is not None and time.monotonic() >= self._deadline_at

    def wait(self) -> ProcessResult:
        """Wait to the declared deadline, then clean and reap the whole group."""
        if self._result is not None and self._result.cleanup_complete:
            return self._result
        if self._startup_expired:
            return self.cleanup(status=ProcessStatus.TIMED_OUT)
        try:
            if self._observes_leader_without_reaping():
                while self.poll() is None:
                    if self.deadline_expired():
                        return self.cleanup(status=ProcessStatus.TIMED_OUT)
                    time.sleep(0.01)
                return self.cleanup(status=ProcessStatus.EXITED)
            timeout_s = None
            if self._deadline_at is not None:
                timeout_s = max(0.0, self._deadline_at - time.monotonic())
            try:
                if self._capture_output:
                    stdout, stderr = self._process.communicate(timeout=timeout_s)
                    self._stdout, self._stderr = _as_text(stdout), _as_text(stderr)
                    self._output_collected = True
                else:
                    self._process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired as error:
                if self._capture_output:
                    self._stdout = _as_text(error.stdout)
                    self._stderr = _as_text(error.stderr)
                return self.cleanup(status=ProcessStatus.TIMED_OUT)
            self._mark_leader_reaped()
            return self.cleanup(status=ProcessStatus.EXITED)
        except BaseException as error:
            first_signal = signal.SIGTERM
            if isinstance(error, _RunOwnedSignal):
                first_signal = _forwarded_signal(error.signum)
            result = self.cleanup(
                first_signal=first_signal, status=ProcessStatus.INTERRUPTED
            )
            if not result.cleanup_complete:
                raise ProcessCleanupError(result) from error
            raise

    def terminate(self) -> ProcessResult:
        """Request bounded group termination and return its structured result."""
        return self.cleanup(first_signal=signal.SIGTERM)

    def cleanup(
        self,
        *,
        first_signal: signal.Signals = signal.SIGTERM,
        status: ProcessStatus | None = None,
    ) -> ProcessResult:
        """Boundedly signal, kill, reap, and retain ownership on any failure."""
        with self._cleanup_lock:
            if self._result is not None and self._result.cleanup_complete:
                return self._result
            errors: list[str] = []
            first_signal_sent: int | None = None
            kill_sent = False
            descendants_reaped = exited = True
            if self._owned_group_alive(errors):
                first_signal_sent = int(first_signal)
                exited = False
                if first_signal != signal.SIGKILL:
                    self._send_signal(first_signal, errors)
                    exited = self._wait_for_owned_exit(
                        self._term_grace_s, errors, allow_escalation=True
                    )
                if not exited:
                    kill_sent = True
                    descendants_reaped, escaped_gone = self._kill_owned_tree(errors)
                    exited = descendants_reaped and escaped_gone
            if not exited:
                self._wait_for_owned_exit(self._reap_grace_s, errors)
            descendants_reaped = (
                self._reap_group_children(errors) and descendants_reaped
            )
            reaped = descendants_reaped and self._reap_leader(errors)
            group_gone = not self._owned_group_alive(errors)
            if not reaped or not group_gone:
                errors.append(
                    "owned process group did not exit within cleanup deadline"
                )
            # Output is cleanup evidence: collect it before deciding the result.
            self._collect_output(errors)
            complete = reaped and group_gone and self._output_collected and not errors
            if not complete:
                result_status = ProcessStatus.CLEANUP_FAILED
            elif status is not None:
                result_status = status
            elif kill_sent:
                result_status = ProcessStatus.KILLED
            elif first_signal_sent is not None:
                result_status = ProcessStatus.TERMINATED
            else:
                result_status = ProcessStatus.EXITED
            returncode = self._leader_returncode
            if returncode is None:
                returncode = self._process.returncode
            self._result = ProcessResult(
                args=self._args,
                kind=self._kind,
                pid=self.pid,
                process_group_id=self.process_group_id,
                status=result_status,
                returncode=returncode,
                stdout=self._stdout,
                stderr=self._stderr,
                error="; ".join(errors) if errors else None,
                cause=None if complete else status,
                first_signal=first_signal_sent,
                term_sent=first_signal_sent == signal.SIGTERM,
                kill_sent=kill_sent,
                reaped=reaped,
                cleanup_complete=complete,
            )
            return self._result

    def _owned_group_alive(self, errors: list[str]) -> bool:
        if self.process_group_id is None:
            return self.poll() is None
        if self._leader_reaped:
            members = _process_group_members(self.process_group_id)
            if members is None:
                errors.append(
                    "cannot inspect process-group membership after leader reap"
                )
            elif members:
                errors.append(
                    "owned leader identity is gone while its process group still "
                    "has members; refusing to trust or signal a reused group"
                )
            return members is None or bool(members)
        if not self._validate_group_ownership(errors):
            return False
        if self._leader_exit_observed:
            members = _process_group_members(self.process_group_id)
            if members is None:
                errors.append("cannot inspect owned process-group membership")
                return True
            return bool(members - {self.pid})
        try:
            os.killpg(self.process_group_id, 0)
        except ProcessLookupError:
            return False
        except PermissionError as error:
            errors.append(f"cannot probe owned process group: {error}")
        return True

    def _validate_group_ownership(self, errors: list[str]) -> bool:
        """Refuse to signal after the stable group-leader identity disappears."""
        if self.process_group_id is None:
            return True
        if self._leader_reaped:
            errors.append(
                "owned process group leader was reaped; refusing to signal its PGID"
            )
            return False
        if self._process_identity is not None:
            try:
                current_identity = _read_linux_process_identity(self.pid)
            except ProcessOwnershipError as error:
                errors.append(str(error))
                return False
            if current_identity == self._process_identity:
                return True
            if current_identity is None:
                errors.append(
                    "owned process identity is unavailable; refusing to signal "
                    "its process group"
                )
            else:
                errors.append(
                    "owned process PID/start-time/session/group identity changed; "
                    "refusing to signal it"
                )
            return False
        if _is_linux():
            errors.append(
                "owned process has no reliable Linux identity; refusing to signal "
                "its process group"
            )
            return False
        try:
            current_group_id = os.getpgid(self.pid)
        except ProcessLookupError:
            errors.append(
                "owned process group leader identity is unavailable; refusing to "
                "signal its PGID"
            )
            return False
        except OSError as error:
            errors.append(f"cannot validate owned process group: {error}")
            return False
        if current_group_id != self.process_group_id:
            errors.append("owned process group identity changed; refusing to signal it")
            return False
        return True

    def _send_signal(self, signum: signal.Signals, errors: list[str]) -> None:
        if self.process_group_id is not None:
            if self._validate_group_ownership(errors):
                try:
                    os.killpg(self.process_group_id, signum)
                except ProcessLookupError:
                    pass
                except OSError as error:
                    errors.append(f"failed to signal owned process group: {error}")
            return
        if self.poll() is not None:
            return
        try:
            if signum == signal.SIGKILL:
                self._process.kill()
            elif signum == signal.SIGTERM:
                self._process.terminate()
            else:
                self._process.send_signal(signum)
        except OSError as error:
            errors.append(f"failed to signal owned process: {error}")

    def _wait_for_owned_exit(
        self, timeout_s: float, errors: list[str], *, allow_escalation: bool = False
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while True:
            self.poll()
            self._reap_exited_group_descendants()
            if not self._owned_group_alive(errors):
                return True
            # A repeated parent signal shortens only the graceful window; the
            # caller still escalates to KILL and proves the reap.
            if allow_escalation and self._escalation_requested.is_set():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(0.01, remaining))

    def _reap_exited_group_descendants(self) -> None:
        # Zombies adopted by this subreaper would hold the group "alive" for the
        # whole TERM grace. Reap them only between leader exit and leader reap,
        # while the leader identity still pins the group.
        if (
            self.process_group_id is None
            or self._group_reaper is None
            or not self._leader_exit_observed
            or self._leader_reaped
        ):
            return
        members = _process_group_members(self.process_group_id) or set()
        for member_pid in members - {self.pid}:
            with contextlib.suppress(ChildProcessError):
                os.waitpid(member_pid, os.WNOHANG)

    def _kill_owned_tree(self, errors: list[str]) -> tuple[bool, bool]:
        """Kill and reap the group plus descendants that escaped it.

        Snapshot ``/proc`` parent links while the live leader pins the tree.
        Descendants may create sessions with ``setsid``.
        """
        escaped: tuple[_LinuxProcessIdentity, ...] = ()
        if (
            self.process_group_id is not None
            and self._process_identity is not None
            and not self._leader_exit_observed
            and _is_linux()
            and self._validate_group_ownership([])
        ):
            snapshot = _owned_tree_escapees(self.pid, self.process_group_id)
            if snapshot is None:
                errors.append(
                    "cannot inspect the owned process tree for descendants outside "
                    "the owned process group"
                )
            escaped = snapshot or ()
        self._send_signal(signal.SIGKILL, errors)
        for identity in escaped:  # Re-verify PID/start-time before signaling.
            try:
                entry = _read_linux_proc_entry(identity.pid)
            except ProcessOwnershipError as error:
                errors.append(str(error))
                continue
            if (
                entry is None
                or entry.identity.start_time != identity.start_time
                or entry.state in ("Z", "X")
            ):
                continue
            # A group it leads lives in a session created inside the owned tree.
            leads_group = entry.identity.process_group_id == identity.pid
            try:
                (os.killpg if leads_group else os.kill)(identity.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except OSError as error:
                errors.append(
                    f"failed to signal owned descendant {identity.pid}: {error}"
                )
        descendants_reaped = self._reap_group_children(
            errors, timeout_s=self._kill_grace_s
        )
        return descendants_reaped, self._wait_for_escaped_exit(escaped, errors)

    def _wait_for_escaped_exit(
        self, escaped: tuple[_LinuxProcessIdentity, ...], errors: list[str]
    ) -> bool:
        if not escaped:
            return True
        own_pid = os.getpid()
        tracked = {identity.pid for identity in escaped} | {self.pid}
        deadline = time.monotonic() + self._kill_grace_s
        while True:
            members = None
            if self.process_group_id is not None:
                members = _process_group_members(self.process_group_id)
            parents = tracked | (members or set())
            pending: list[int] = []
            for identity in escaped:
                try:
                    entry = _read_linux_proc_entry(identity.pid)
                except ProcessOwnershipError:
                    pending.append(identity.pid)
                    continue
                if entry is None or entry.identity.start_time != identity.start_time:
                    continue
                if entry.state not in ("Z", "X"):
                    pending.append(identity.pid)
                elif entry.parent_pid == own_pid:
                    # Adopted by this subreaper; the recorded identity proves
                    # it is an owned descendant, not another owner's child.
                    with contextlib.suppress(ChildProcessError):
                        os.waitpid(identity.pid, os.WNOHANG)
                    pending.append(identity.pid)
                elif entry.parent_pid in parents:
                    # Its dying owned parent releases it to this subreaper.
                    pending.append(identity.pid)
            if not pending:
                return True
            if time.monotonic() >= deadline:
                errors.append(
                    "descendants outside the owned process group did not exit "
                    f"within cleanup deadline: {sorted(pending)}"
                )
                return False
            time.sleep(0.01)

    def _reap_leader(self, errors: list[str]) -> bool:
        try:
            self._process.wait(timeout=self._reap_grace_s)
        except subprocess.TimeoutExpired:
            self._leader_reaped = False
            return False
        except OSError as error:
            errors.append(f"failed to reap owned process leader: {error}")
            return False
        self._mark_leader_reaped()
        return True

    def _reap_group_children(
        self, errors: list[str], *, timeout_s: float | None = None
    ) -> bool:
        if self.process_group_id is None or self._group_reaper is None:
            return True
        try:
            proven = self._group_reaper(
                self.process_group_id,
                timeout_s=self._reap_grace_s if timeout_s is None else timeout_s,
            )
        except BaseException as error:  # noqa: BLE001
            errors.append(
                "failed to reap owned process descendants: "
                f"{type(error).__name__}: {error}"
            )
            return False
        if proven is not True:
            errors.append("owned process descendant reaper returned no cleanup proof")
        return proven is True

    def _collect_output(self, errors: list[str]) -> None:
        if self._output_collected:
            errors.extend(self._output_errors)
            return
        if self._output_threads:
            deadline = time.monotonic() + self._reap_grace_s
            for thread in self._output_threads:
                thread.join(timeout=max(0.0, deadline - time.monotonic()))
            self._stdout = _join_output(self._stdout_chunks)
            self._stderr = _join_output(self._stderr_chunks)
            if any(thread.is_alive() for thread in self._output_threads):
                # Closing a pipe here can hang on the lock a blocked drainer
                # holds (an escaped session may keep the writer). Leave it to
                # the daemon drainer and allow a later cleanup retry.
                errors.append(
                    "captured output did not close within cleanup deadline; "
                    "an unowned inherited writer may still be alive"
                )
            else:
                self._output_collected = True
                # Drainers reached EOF; release the pipe descriptors now.
                for pipe in (self._process.stdout, self._process.stderr):
                    try:
                        if pipe is not None:
                            pipe.close()
                    except BaseException as close_error:  # noqa: BLE001
                        errors.append(
                            "failed to close captured output: "
                            f"{type(close_error).__name__}: {close_error}"
                        )
            errors.extend(self._output_errors)
            return
        try:
            stdout, stderr = self._process.communicate(timeout=self._reap_grace_s)
        except subprocess.TimeoutExpired as error:
            self._stdout = _as_text(error.stdout) or self._stdout
            self._stderr = _as_text(error.stderr) or self._stderr
            for pipe in (self._process.stdout, self._process.stderr):
                try:
                    if pipe is not None:
                        pipe.close()
                except BaseException as close_error:  # noqa: BLE001
                    errors.append(
                        "failed to close captured output: "
                        f"{type(close_error).__name__}: {close_error}"
                    )
            errors.append("captured output did not close within cleanup deadline")
        except BaseException as error:  # noqa: BLE001
            errors.append(
                f"failed to collect captured output: {type(error).__name__}: {error}"
            )
        else:
            self._stdout = self._stdout if stdout is None else _as_text(stdout)
            self._stderr = self._stderr if stderr is None else _as_text(stderr)
            self._output_collected = True
            self._mark_leader_reaped()


_RELAYED_PARENT_SIGNALS: Final[tuple[signal.Signals, ...]] = tuple(
    getattr(signal, name)
    for name in ("SIGTERM", "SIGINT", "SIGHUP", "SIGQUIT")
    if hasattr(signal, name)
)


def _forwarded_signal(signum: signal.Signals) -> signal.Signals:
    # SIGQUIT is relayed as SIGTERM: a core dump of Isaac Sim is never wanted.
    return signal.SIGTERM if signum == getattr(signal, "SIGQUIT", None) else signum


def _restore_signal_handlers(
    handlers: Mapping[signal.Signals, _SignalHandler],
) -> BaseException | None:
    """Restore every handler independently; return the first failure."""
    first_error: BaseException | None = None
    for signum, handler in handlers.items():
        try:
            signal.signal(signum, handler)
        except BaseException as error:  # noqa: BLE001
            if first_error is None:
                first_error = error
    return first_error


def _propagate_previous_signal_handler(
    signum: signal.Signals, frame: FrameType | None, previous_handler: _SignalHandler
) -> None:
    """Continue the caller's signal policy after owned cleanup completes."""
    if previous_handler is signal.SIG_DFL:
        signal.raise_signal(signum)
    elif callable(previous_handler):
        previous_handler(int(signum), frame)
    elif previous_handler is not signal.SIG_IGN:
        raise RuntimeError(f"unsupported previous signal handler for {signum.name}")


def _scrub_internal_signal_cause(error: BaseException) -> None:
    """Strip the internal signal control-transfer from a public exception chain."""
    seen: set[int] = set()
    current: BaseException | None = error
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        cause, context = current.__cause__, current.__context__
        if isinstance(cause, _RunOwnedSignal) or isinstance(context, _RunOwnedSignal):
            if isinstance(cause, _RunOwnedSignal):
                current.__cause__ = None
            current.__context__ = None
            current.__suppress_context__ = True
            return
        current = cause if cause is not None else context


def run_owned(
    cmd: Sequence[CommandArg],
    *,
    kind: ProcessKind,
    timeout: float | None,
    capture_output: bool = False,
    cwd: str | os.PathLike[str] | None = None,
    env: Mapping[str, str] | None = None,
    term_grace_s: float = DEFAULT_TERM_GRACE_S,
    kill_grace_s: float = DEFAULT_TERM_GRACE_S,
    reap_grace_s: float = DEFAULT_TERM_GRACE_S,
    stdout: _OutputTarget = None,
    stderr: _OutputTarget = None,
) -> ProcessResult:
    """Run one classified process through the public owner contract.

    Main-thread POSIX calls relay SIGTERM/SIGINT/SIGHUP/SIGQUIT through bounded cleanup;
    SIGQUIT is sent to the group as SIGTERM. Prior policy resumes afterward.
    Ignored or C-installed handlers stay untouched. Later signals escalate;
    worker threads install no handlers.
    """
    owned: OwnedMonitoredProcess | None = None
    interruption: _RunOwnedSignal | None = None
    previous_handlers: dict[signal.Signals, _SignalHandler] = {}
    relay_active = True

    def relay_parent_signal(received: int, frame: FrameType | None) -> None:
        nonlocal interruption
        if not relay_active:
            return
        if interruption is not None:
            if owned is not None:
                owned.request_escalation()
            return
        interruption = _RunOwnedSignal(signal.Signals(received), frame)
        if owned is not None:
            raise interruption

    if os.name == "posix" and threading.current_thread() is threading.main_thread():
        try:
            for signum in _RELAYED_PARENT_SIGNALS:
                previous_handler = signal.getsignal(signum)
                if previous_handler is None or previous_handler is signal.SIG_IGN:
                    continue
                signal.signal(signum, relay_parent_signal)
                previous_handlers[signum] = previous_handler
        except BaseException as error:
            relay_active = False
            restore_error = _restore_signal_handlers(previous_handlers)
            if restore_error is not None:
                error.add_note(
                    f"restoring signal handlers also failed: {restore_error}"
                )
            raise

    result: ProcessResult
    primary_error: BaseException | None = None
    try:
        # The outer handler also covers the ``_RunOwnedSignal`` handler body,
        # so every escaping exception is scrubbed of the internal signal.
        try:
            try:
                owned = OwnedMonitoredProcess.spawn(
                    cmd,
                    kind=kind,
                    deadline_s=timeout,
                    capture_output=capture_output,
                    cwd=cwd,
                    env=env,
                    term_grace_s=term_grace_s,
                    kill_grace_s=kill_grace_s,
                    reap_grace_s=reap_grace_s,
                    stdout=stdout,
                    stderr=stderr,
                )
                if interruption is not None:
                    raise interruption
                result = owned.wait()
            except _RunOwnedSignal as received_signal:
                interruption = received_signal
                if owned is None:  # pragma: no cover - handler cannot raise yet
                    raise RuntimeError(
                        "owned process signal relay fired before ownership existed"
                    ) from None
                result = owned.result or owned.cleanup(
                    first_signal=_forwarded_signal(received_signal.signum),
                    status=ProcessStatus.INTERRUPTED,
                )
        except BaseException as error:
            _scrub_internal_signal_cause(error)
            primary_error = error
            raise
    finally:
        relay_active = False
        if (restore_error := _restore_signal_handlers(previous_handlers)) is not None:
            # Never replace the run's real result or exception.
            _LOGGER.error(
                "failed to restore a signal handler after an owned run: %s",
                restore_error,
            )
            if primary_error is not None:
                primary_error.add_note(
                    f"restoring signal handlers also failed: {restore_error}"
                )

    if interruption is not None:
        _propagate_previous_signal_handler(
            interruption.signum,
            interruption.frame,
            previous_handlers[interruption.signum],
        )
    return result


def terminate_owned_process(
    process: subprocess.Popen[str] | subprocess.Popen[bytes],
    *,
    signal_name: str = "TERM",
    term_grace_s: float = 0.5,
    kill_grace_s: float = 0.5,
) -> ProcessResult:
    """Compatibility adapter for a caller that already owns a ``Popen``.

    Linux adoption binds PID, start time, session, and group before signaling.
    The caller must retain exclusive ownership and not poll, wait for, or reap
    this ``Popen`` during cleanup.
    """
    graces = (term_grace_s, kill_grace_s, kill_grace_s)
    _validate_lifecycle(ProcessKind.MONITORED, None, graces)
    try:
        first_signal = signal.Signals(getattr(signal, f"SIG{signal_name}"))
    except (AttributeError, ValueError) as error:
        raise ValueError(f"unsupported process signal: {signal_name}") from error
    owns_group = os.name == "posix"
    if owns_group:
        _enable_child_subreaper()
    raw_args = process.args
    args: tuple[str, ...]
    if isinstance(raw_args, (str, bytes, os.PathLike)):
        args = (os.fsdecode(raw_args),)
    else:
        args = tuple(os.fsdecode(arg) for arg in raw_args)
    pid = process.pid
    if process.poll() is not None and _is_linux():
        # Once Popen reaped the leader, no descendant authority can be
        # recovered safely from this handle, even if the PID was reused.
        raise ProcessOwnershipError(
            pid,
            None,
            reason="cannot adopt a reaped Linux process: original descendant "
            "identity is unavailable after leader exit",
        )
    identity: _LinuxProcessIdentity | None = None
    if owns_group and not _is_linux():
        raise ProcessOwnershipError(
            pid,
            None,
            reason="legacy process-group adoption requires a reliable Linux "
            "identity; this platform cannot prove PID reuse safety",
        )
    if owns_group:
        identity = _read_linux_process_identity(pid)
        if identity is None:
            raise ProcessOwnershipError(
                pid,
                None,
                reason="legacy process adoption requires a reliable Linux identity "
                "containing PID, start time, session, and process group",
            )
        if identity.process_group_id != pid:
            raise ProcessOwnershipError(pid, identity.process_group_id)
        if identity.session_id != pid:
            raise ProcessOwnershipError(
                pid,
                identity.process_group_id,
                reason=f"process {pid} is not its session leader "
                f"(session={identity.session_id}); descendant ownership is unproven",
            )
    launch = _Launch(args, ProcessKind.MONITORED, False, graces)
    return launch.owner(process, identity).cleanup(first_signal=first_signal)
