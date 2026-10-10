use std::fmt;
#[cfg(target_os = "linux")]
use std::fs::OpenOptions;
use std::io;
#[cfg(target_os = "linux")]
use std::io::{Read, Write};
#[cfg(target_os = "linux")]
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
#[cfg(target_os = "linux")]
use std::os::unix::net::UnixStream;
#[cfg(target_os = "linux")]
use std::os::unix::process::CommandExt as _;
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus};
#[cfg(target_os = "linux")]
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
#[cfg(target_os = "linux")]
use std::sync::mpsc;

use std::time::{Duration, Instant};
#[cfg(target_os = "linux")]
use std::{fs, mem::MaybeUninit, sync::OnceLock};

const STARTUP_WAIT: Duration = Duration::from_secs(2);
const STOP_GRACE_PERIOD: Duration = Duration::from_millis(250);
const TERM_GRACE_PERIOD: Duration = Duration::from_millis(500);
const KILL_GRACE_PERIOD: Duration = Duration::from_millis(500);
const REAP_GRACE_PERIOD: Duration = Duration::from_millis(500);
// Flushing the captured log may first flush unrelated journal data; 500 ms
// proved too short on a loaded host and misreported clean stops as failures.

const OUTPUT_GRACE_PERIOD: Duration = Duration::from_secs(2);

#[cfg(target_os = "linux")]
const CLEANUP_RECONCILIATION_INTERVAL: Duration = Duration::from_millis(1);
#[cfg(target_os = "linux")]
const STARTUP_GATE_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(target_os = "linux")]
const STARTUP_GATE_GO: u8 = b'G';
#[cfg(target_os = "linux")]
const STARTUP_GATE_ABORT: u8 = b'A';

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiProcessKind {
    Interactive,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiProcessStatus {
    Exited,
    TimedOut,
    Interrupted,
    Terminated,
    Killed,
    CleanupFailed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiProcessSignal {
    Int,
    Term,
    Kill,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiCleanupErrorKind {
    #[cfg(not(target_os = "linux"))]
    UnsupportedPlatform,
    Ownership,
    Wait,
    Signal,
    Reap,
    Deadline,
    Output,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiCleanupError {
    pub kind: GuiCleanupErrorKind,
    pub message: String,
}

impl GuiCleanupError {
    fn new(kind: GuiCleanupErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
        }
    }
}

impl fmt::Display for GuiCleanupError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{:?}: {}", self.kind, self.message)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiProcessResult {
    pub argv: Vec<String>,
    pub kind: GuiProcessKind,
    pub pid: Option<u32>,
    pub process_group_id: Option<i32>,
    pub session_id: Option<i32>,
    pub status: GuiProcessStatus,
    pub exit_status: Option<ExitStatus>,
    pub output_path: PathBuf,
    pub error: Option<String>,
    pub cleanup_errors: Vec<GuiCleanupError>,
    pub cause: Option<GuiProcessStatus>,
    pub first_signal: Option<GuiProcessSignal>,
    pub last_signal: Option<GuiProcessSignal>,
    pub term_sent: bool,
    pub kill_sent: bool,
    pub reaped: bool,
    pub descendants_reaped: bool,
    pub group_empty: bool,
    pub session_empty: bool,
    pub output_complete: bool,
    pub cleanup_complete: bool,
}

#[derive(Clone, Default)]
struct GuiSignalEvidence {
    first: Option<GuiProcessSignal>,
    last: Option<GuiProcessSignal>,
    term_sent: bool,
    kill_sent: bool,
}

impl GuiSignalEvidence {
    #[cfg(target_os = "linux")]
    fn record(&mut self, signal: GuiProcessSignal) {
        self.first.get_or_insert(signal);
        self.last = Some(signal);
        match signal {
            GuiProcessSignal::Int => {}
            GuiProcessSignal::Term => self.term_sent = true,
            GuiProcessSignal::Kill => self.kill_sent = true,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct GuiProcessDeadlines {
    startup: Duration,
    stop: Duration,
    term: Duration,
    kill: Duration,
    reap: Duration,
    output: Duration,
}

impl Default for GuiProcessDeadlines {
    fn default() -> Self {
        Self {
            startup: STARTUP_WAIT,
            stop: STOP_GRACE_PERIOD,
            term: TERM_GRACE_PERIOD,
            kill: KILL_GRACE_PERIOD,
            reap: REAP_GRACE_PERIOD,
            output: OUTPUT_GRACE_PERIOD,
        }
    }
}

#[derive(Debug)]
pub(crate) enum OwnedProcessSpawnError {
    Io(io::Error),
    Supervision(String),
}

impl fmt::Display for OwnedProcessSpawnError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => error.fmt(formatter),
            Self::Supervision(message) => formatter.write_str(message),
        }
    }
}

pub(crate) struct OwnedProcess {
    child: Child,
    argv: Vec<String>,
    kind: GuiProcessKind,
    pid: u32,
    process_group_id: Option<i32>,
    session_id: Option<i32>,
    output_path: PathBuf,
    deadlines: GuiProcessDeadlines,
    exit_status: Option<ExitStatus>,
    failed_cleanup_cause: Option<GuiProcessStatus>,
    signal_evidence: GuiSignalEvidence,
    completed_result: Option<GuiProcessResult>,
    #[cfg(target_os = "linux")]
    leader_identity: LinuxLeaderIdentity,
    #[cfg(target_os = "linux")]
    leader_exit_observed: bool,
    #[cfg(target_os = "linux")]
    ownership_discharged: bool,
    #[cfg(target_os = "linux")]
    parent_signal_relay: Option<ScopedParentSignalRelay>,
    #[cfg(target_os = "linux")]
    /// In-flight output sync, tagged with its path to prevent retries from using
    /// a stale result after `output_path` changes.
    output_sync: Option<(PathBuf, mpsc::Receiver<io::Result<()>>)>,
}

impl OwnedProcess {
    pub(crate) fn spawn_bound(
        command: Command,
        argv: Vec<String>,
        kind: GuiProcessKind,
        output_path: PathBuf,
    ) -> Result<Self, OwnedProcessSpawnError> {
        #[cfg(target_os = "linux")]
        {
            Self::spawn_bound_with_probe(
                command,
                argv,
                kind,
                output_path,
                capture_linux_leader_identity,
            )
        }
        #[cfg(not(target_os = "linux"))]
        {
            let mut command = command;
            let child = command.spawn().map_err(OwnedProcessSpawnError::Io)?;
            let pid = child.id();
            #[cfg(all(unix, not(target_os = "linux")))]
            let (process_group_id, session_id) = (Some(pid as i32), Some(pid as i32));
            #[cfg(not(unix))]
            let (process_group_id, session_id) = (None, None);
            Ok(Self {
                child,
                argv,
                kind,
                pid,
                process_group_id,
                session_id,
                output_path,
                deadlines: GuiProcessDeadlines::default(),
                exit_status: None,
                failed_cleanup_cause: None,
                signal_evidence: GuiSignalEvidence::default(),
                completed_result: None,
            })
        }
    }

    #[cfg(target_os = "linux")]
    fn spawn_bound_with_probe(
        command: Command,
        argv: Vec<String>,
        kind: GuiProcessKind,
        output_path: PathBuf,
        identity_probe: fn(i32) -> Result<LinuxLeaderIdentity, String>,
    ) -> Result<Self, OwnedProcessSpawnError> {
        enable_linux_child_subreaper().map_err(OwnedProcessSpawnError::Supervision)?;
        let parent_signal_relay =
            ScopedParentSignalRelay::install().map_err(OwnedProcessSpawnError::Supervision)?;
        let (child, leader_identity) = spawn_linux_identity_gated(command, identity_probe)?;
        let pid = child.id();
        if pid as i32 != leader_identity.pid {
            let mut child = child;
            let _ = child.kill();
            let _ = child.wait();
            return Err(OwnedProcessSpawnError::Supervision(format!(
                "startup identity PID {} did not match child handle PID {pid}",
                leader_identity.pid
            )));
        }
        Ok(Self {
            child,
            argv,
            kind,
            pid,
            process_group_id: Some(leader_identity.process_group_id),
            session_id: Some(leader_identity.session_id),
            output_path,
            deadlines: GuiProcessDeadlines::default(),
            exit_status: None,
            failed_cleanup_cause: None,
            signal_evidence: GuiSignalEvidence::default(),
            completed_result: None,
            leader_identity,
            leader_exit_observed: false,
            ownership_discharged: false,
            parent_signal_relay: Some(parent_signal_relay),
            output_sync: None,
        })
    }

    pub(crate) fn wait_for_startup(&mut self) -> Result<bool, String> {
        self.wait_timeout(self.deadlines.startup)
    }

    pub(crate) fn is_running(&mut self) -> Result<bool, String> {
        if self.completed_result.is_some() {
            return Ok(false);
        }
        self.wait_timeout(Duration::ZERO)
    }

    pub(crate) fn leader_exited(&self) -> bool {
        #[cfg(target_os = "linux")]
        {
            self.leader_exit_observed || self.exit_status.is_some()
        }
        #[cfg(not(target_os = "linux"))]
        {
            self.exit_status.is_some()
        }
    }

    pub(crate) fn stop(&mut self) -> GuiProcessResult {
        if let Some(signal) = self.take_parent_signal() {
            return self.stop_after_parent_signal(signal);
        }
        self.stop_with_cause(None)
    }

    pub(crate) fn stop_with_cause(
        &mut self,
        requested_cause: Option<GuiProcessStatus>,
    ) -> GuiProcessResult {
        self.stop_with_first_signal(requested_cause, GuiProcessSignal::Term)
    }

    pub(crate) fn stop_after_parent_signal(
        &mut self,
        signal: GuiProcessSignal,
    ) -> GuiProcessResult {
        self.stop_with_first_signal(Some(Self::signal_cause(signal)), signal)
    }

    fn signal_cause(signal: GuiProcessSignal) -> GuiProcessStatus {
        match signal {
            GuiProcessSignal::Int => GuiProcessStatus::Interrupted,
            GuiProcessSignal::Term => GuiProcessStatus::Terminated,
            GuiProcessSignal::Kill => GuiProcessStatus::Killed,
        }
    }

    /// Consume parent SIGTERM/SIGINT recorded while this child was owned.
    /// The scoped relay is restored on drop. The first signal starts bounded
    /// cleanup while the GUI stays open to report the result, preventing an
    /// interrupted cleanup from orphaning a session-detached Isaac process.
    /// Later signals advance escalation one phase at a time; they do not
    /// shorten the SIGKILL wait.
    pub(crate) fn take_parent_signal(&self) -> Option<GuiProcessSignal> {
        #[cfg(target_os = "linux")]
        {
            self.parent_signal_relay
                .as_ref()
                .and_then(ScopedParentSignalRelay::take_signal)
        }
        #[cfg(not(target_os = "linux"))]
        {
            None
        }
    }

    fn stop_with_first_signal(
        &mut self,
        requested_cause: Option<GuiProcessStatus>,
        first_signal: GuiProcessSignal,
    ) -> GuiProcessResult {
        if let Some(result) = &self.completed_result {
            return result.clone();
        }

        #[cfg(target_os = "linux")]
        let result = self.stop_linux(requested_cause, GroupSignal::from_gui(first_signal));
        #[cfg(not(target_os = "linux"))]
        let result = self.stop_unsupported(requested_cause);

        if result.cleanup_complete {
            #[cfg(target_os = "linux")]
            drop(self.parent_signal_relay.take());
            self.completed_result = Some(result.clone());
        }
        result
    }

    #[cfg(target_os = "linux")]
    fn stop_linux(
        &mut self,
        requested_cause: Option<GuiProcessStatus>,
        first_signal: GroupSignal,
    ) -> GuiProcessResult {
        self.stop_linux_with_observer(requested_cause, first_signal, |owned| {
            owned.observe_linux_session()
        })
    }

    #[cfg(target_os = "linux")]
    fn stop_linux_with_observer<Observe>(
        &mut self,
        requested_cause: Option<GuiProcessStatus>,
        first_signal: GroupSignal,
        mut observe: Observe,
    ) -> GuiProcessResult
    where
        Observe: FnMut(&mut OwnedProcess) -> io::Result<LinuxSessionState>,
    {
        if self.ownership_discharged {
            return self.retry_linux_output_completion(requested_cause);
        }

        let mut errors = Vec::new();
        let mut proof_resolvable_errors = Vec::new();
        let mut signal_evidence = self.signal_evidence.clone();
        let mut attempt_cause = requested_cause;
        let mut result_cause = requested_cause;

        let mut wait_disappeared = false;
        let mut running = match self.wait_linux(self.deadlines.stop, ParentSignalWait::Escalate) {
            Ok((running, consumed_signal)) => {
                result_cause = consumed_signal.or(result_cause);
                running
            }
            Err(error) => {
                Self::record_io_cause(&error, &mut attempt_cause);
                wait_disappeared = Self::push_wait_error(
                    &error,
                    format!("failed during graceful wait: {error}"),
                    &mut errors,
                    &mut proof_resolvable_errors,
                );
                true
            }
        };

        if running {
            let signal_attempt = self.send_linux_signal(first_signal);
            if signal_attempt.target_signaled {
                signal_evidence.record(first_signal.to_gui());
            }
            if let Some(error) = signal_attempt.error {
                Self::record_io_cause(&error, &mut attempt_cause);
                let cleanup_error = GuiCleanupError::new(
                    GuiCleanupErrorKind::Signal,
                    format!(
                        "failed to send {} to owned process session: {error}",
                        first_signal.name()
                    ),
                );
                if Self::linux_signal_error_is_session_disappearance(&error) {
                    proof_resolvable_errors.push(cleanup_error);
                } else {
                    errors.push(cleanup_error);
                }
            }
            running = match self.wait_linux(self.deadlines.term, ParentSignalWait::Escalate) {
                Ok((running, consumed_signal)) => {
                    result_cause = consumed_signal.or(result_cause);
                    wait_disappeared = false;
                    running
                }
                Err(error) => {
                    Self::record_io_cause(&error, &mut attempt_cause);
                    wait_disappeared = Self::push_wait_error(
                        &error,
                        format!(
                            "failed while waiting after {}: {error}",
                            first_signal.name()
                        ),
                        &mut errors,
                        &mut proof_resolvable_errors,
                    );
                    true
                }
            };
        }

        if running {
            let signal_attempt = self.send_linux_signal(GroupSignal::Kill);
            if signal_attempt.target_signaled {
                signal_evidence.record(GuiProcessSignal::Kill);
            }
            if let Some(error) = signal_attempt.error {
                Self::record_io_cause(&error, &mut attempt_cause);
                let cleanup_error = GuiCleanupError::new(
                    GuiCleanupErrorKind::Signal,
                    format!("failed to send SIGKILL to owned process group: {error}"),
                );
                if Self::linux_signal_error_is_session_disappearance(&error) {
                    proof_resolvable_errors.push(cleanup_error);
                } else {
                    errors.push(cleanup_error);
                }
            }
            running = match self.wait_linux(self.deadlines.kill, ParentSignalWait::Continue) {
                Ok((running, consumed_signal)) => {
                    result_cause = consumed_signal.or(result_cause);
                    wait_disappeared = false;
                    running
                }
                Err(error) => {
                    Self::record_io_cause(&error, &mut attempt_cause);
                    wait_disappeared = Self::push_wait_error(
                        &error,
                        format!("failed while waiting after SIGKILL: {error}"),
                        &mut errors,
                        &mut proof_resolvable_errors,
                    );
                    true
                }
            };
        }

        if running && !wait_disappeared {
            errors.push(GuiCleanupError::new(
                GuiCleanupErrorKind::Deadline,
                "owned process session did not exit within cleanup deadline",
            ));
        }

        let reap_deadline = Instant::now() + self.deadlines.reap;
        let mut proof_errors = Vec::new();
        let mut descendants_reaped = false;
        let mut group_empty = false;
        let mut session_empty = false;
        while Instant::now() < reap_deadline {
            match observe(self) {
                Ok(state) => {
                    descendants_reaped = state.group_empty && state.session_empty;
                    if state.leader_exited && descendants_reaped {
                        match self.reap_linux_leader_until(reap_deadline) {
                            Ok(()) => {
                                let leader_reaped = self.exit_status.is_some();
                                // Group/session emptiness is public only after the retained zombie
                                // leader is reaped.
                                group_empty = leader_reaped && state.group_empty;
                                session_empty = leader_reaped && state.session_empty;
                                if leader_reaped && group_empty && session_empty {
                                    break;
                                }
                            }
                            Err(error) => {
                                Self::record_io_cause(&error, &mut attempt_cause);
                                let cleanup_error = GuiCleanupError::new(
                                    GuiCleanupErrorKind::Reap,
                                    format!("failed to reap owned process leader: {error}"),
                                );
                                if !proof_errors.contains(&cleanup_error) {
                                    proof_errors.push(cleanup_error);
                                }
                            }
                        }
                    }
                }
                Err(error) => {
                    Self::record_io_cause(&error, &mut attempt_cause);
                    let cleanup_error = GuiCleanupError::new(
                        GuiCleanupErrorKind::Reap,
                        format!("failed to prove owned descendant reaping: {error}"),
                    );
                    if !proof_errors.contains(&cleanup_error) {
                        proof_errors.push(cleanup_error);
                    }
                }
            }
            let remaining = reap_deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            std::thread::sleep(remaining.min(CLEANUP_RECONCILIATION_INTERVAL));
        }
        let reaped = self.exit_status.is_some();
        if !reaped {
            let cleanup_error = GuiCleanupError::new(
                GuiCleanupErrorKind::Reap,
                "owned subprocess leader was not reaped",
            );
            if !proof_errors.contains(&cleanup_error) {
                proof_errors.push(cleanup_error);
            }
        }
        if !descendants_reaped {
            let cleanup_error = GuiCleanupError::new(
                GuiCleanupErrorKind::Deadline,
                "owned subprocess descendants were not fully reaped within the cleanup deadline",
            );
            if !proof_errors.contains(&cleanup_error) {
                proof_errors.push(cleanup_error);
            }
        }
        let physical_cleanup_proven = reaped && descendants_reaped && group_empty && session_empty;
        if !physical_cleanup_proven {
            errors.extend(proof_resolvable_errors);
        }
        // Once process cleanup is proven, retries only prove output completion;
        // prior attempt errors stay in the result.
        self.ownership_discharged = physical_cleanup_proven;

        let mut output_complete = false;
        if physical_cleanup_proven {
            let output_deadline = Instant::now() + self.deadlines.output;
            while Instant::now() < output_deadline {
                match self.prove_output_complete_until(output_deadline) {
                    Ok(()) => {
                        output_complete = true;
                        break;
                    }
                    Err(error) => {
                        Self::record_io_cause(&error, &mut attempt_cause);
                        let cleanup_error = GuiCleanupError::new(
                            GuiCleanupErrorKind::Output,
                            format!("failed to prove captured output completion: {error}"),
                        );
                        if !proof_errors.contains(&cleanup_error) {
                            proof_errors.push(cleanup_error);
                        }
                    }
                }
                let remaining = output_deadline.saturating_duration_since(Instant::now());
                if remaining.is_zero() {
                    break;
                }
                std::thread::sleep(remaining.min(CLEANUP_RECONCILIATION_INTERVAL));
            }
            if !output_complete
                && !proof_errors
                    .iter()
                    .any(|error| error.kind == GuiCleanupErrorKind::Output)
            {
                proof_errors.push(GuiCleanupError::new(
                    GuiCleanupErrorKind::Output,
                    "captured output completion was not proven within the cleanup deadline",
                ));
            }
        }
        if output_complete {
            proof_errors.clear();
        }
        errors.extend(proof_errors);

        self.build_result(
            result_cause,
            attempt_cause,
            errors,
            signal_evidence,
            reaped,
            descendants_reaped,
            group_empty,
            session_empty,
            output_complete,
        )
    }

    #[cfg(target_os = "linux")]
    fn retry_linux_output_completion(
        &mut self,
        requested_cause: Option<GuiProcessStatus>,
    ) -> GuiProcessResult {
        debug_assert!(self.exit_status.is_some());
        let mut errors = Vec::new();
        let mut attempt_cause = requested_cause;
        let output_deadline = Instant::now() + self.deadlines.output;
        let mut output_complete = false;
        while Instant::now() < output_deadline {
            match self.prove_output_complete_until(output_deadline) {
                Ok(()) => {
                    output_complete = true;
                    break;
                }
                Err(error) => {
                    Self::record_io_cause(&error, &mut attempt_cause);
                    let cleanup_error = GuiCleanupError::new(
                        GuiCleanupErrorKind::Output,
                        format!("failed to prove captured output completion: {error}"),
                    );
                    if !errors.contains(&cleanup_error) {
                        errors.push(cleanup_error);
                    }
                }
            }
            let remaining = output_deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            std::thread::sleep(remaining.min(CLEANUP_RECONCILIATION_INTERVAL));
        }
        if !output_complete && errors.is_empty() {
            errors.push(GuiCleanupError::new(
                GuiCleanupErrorKind::Output,
                "captured output completion was not proven within the cleanup deadline",
            ));
        }
        self.build_result(
            requested_cause,
            attempt_cause,
            errors,
            self.signal_evidence.clone(),
            true,
            true,
            true,
            true,
            output_complete,
        )
    }

    #[cfg(not(target_os = "linux"))]
    fn stop_unsupported(&mut self, requested_cause: Option<GuiProcessStatus>) -> GuiProcessResult {
        let mut errors = vec![GuiCleanupError::new(
            GuiCleanupErrorKind::UnsupportedPlatform,
            unsupported_cleanup_reason(),
        )];
        let mut signal_evidence = self.signal_evidence.clone();
        // Dispose only the exact leader represented by Child: Windows uses
        // its stable process handle, while Unix keeps an unreaped direct child
        // PID reserved. No raw process-group signal is issued, and descendant
        // ownership is deliberately never claimed on these platforms.
        match self.child.try_wait() {
            Ok(Some(status)) => self.exit_status = Some(status),
            Ok(None) => match self.child.kill() {
                Ok(()) => {
                    signal_evidence.kill_sent = true;
                    signal_evidence.first = Some(GuiProcessSignal::Kill);
                    signal_evidence.last = Some(GuiProcessSignal::Kill);
                    match self.wait_timeout(self.deadlines.kill) {
                        Ok(false) => {}
                        Ok(true) => errors.push(GuiCleanupError::new(
                            GuiCleanupErrorKind::Deadline,
                            "exact subprocess leader did not exit within cleanup deadline",
                        )),
                        Err(error) => errors.push(GuiCleanupError::new(
                            GuiCleanupErrorKind::Reap,
                            format!("failed to reap exact subprocess leader: {error}"),
                        )),
                    }
                }
                Err(error) => errors.push(GuiCleanupError::new(
                    GuiCleanupErrorKind::Signal,
                    format!("failed to terminate exact subprocess leader: {error}"),
                )),
            },
            Err(error) => errors.push(GuiCleanupError::new(
                GuiCleanupErrorKind::Wait,
                format!("failed to query subprocess leader: {error}"),
            )),
        }
        self.build_result(
            requested_cause,
            requested_cause,
            errors,
            signal_evidence,
            self.exit_status.is_some(),
            false,
            false,
            false,
            false,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn build_result(
        &mut self,
        requested_cause: Option<GuiProcessStatus>,
        attempt_cause: Option<GuiProcessStatus>,
        cleanup_errors: Vec<GuiCleanupError>,
        signal_evidence: GuiSignalEvidence,
        reaped: bool,
        descendants_reaped: bool,
        group_empty: bool,
        session_empty: bool,
        output_complete: bool,
    ) -> GuiProcessResult {
        self.signal_evidence = signal_evidence.clone();
        let cleanup_complete = reaped
            && descendants_reaped
            && group_empty
            && session_empty
            && output_complete
            && cleanup_errors.is_empty();
        let failure_cause = attempt_cause.or(self.failed_cleanup_cause);
        let cause = if cleanup_complete {
            self.failed_cleanup_cause = None;
            None
        } else {
            self.failed_cleanup_cause = failure_cause;
            failure_cause
        };
        let status = if !cleanup_complete {
            GuiProcessStatus::CleanupFailed
        } else if let Some(status) = requested_cause {
            status
        } else if signal_evidence.kill_sent {
            GuiProcessStatus::Killed
        } else if signal_evidence.term_sent {
            GuiProcessStatus::Terminated
        } else {
            GuiProcessStatus::Exited
        };
        let error = (!cleanup_errors.is_empty()).then(|| {
            cleanup_errors
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join("; ")
        });
        GuiProcessResult {
            argv: self.argv.clone(),
            kind: self.kind,
            pid: Some(self.pid),
            process_group_id: self.process_group_id,
            session_id: self.session_id,
            status,
            exit_status: self.exit_status,
            output_path: self.output_path.clone(),
            error,
            cleanup_errors,
            cause,
            first_signal: signal_evidence.first,
            last_signal: signal_evidence.last,
            term_sent: signal_evidence.term_sent,
            kill_sent: signal_evidence.kill_sent,
            reaped,
            descendants_reaped,
            group_empty,
            session_empty,
            output_complete,
            cleanup_complete,
        }
    }

    fn close_child_handles(&mut self) {
        drop(self.child.stdin.take());
        drop(self.child.stdout.take());
        drop(self.child.stderr.take());
    }

    #[cfg(target_os = "linux")]
    fn prove_output_complete_until(&mut self, deadline: Instant) -> io::Result<()> {
        self.close_child_handles();
        // At most one synchronizer thread is in flight per owned process: a
        // retry after a timeout waits on the same outstanding `fsync` instead
        // of stacking another blocked thread on the same file. A synchronizer
        // started for a different `output_path`, or one that already finished
        // with an error, is never reused: its result describes an earlier
        // attempt, not the file this retry has to prove.
        let stale = match self.output_sync.take() {
            Some((path, receiver)) if path == self.output_path => Some(receiver),
            _ => None,
        };
        if let Some(receiver) = stale {
            match receiver.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                Ok(Ok(())) => return Ok(()),
                Ok(Err(_)) | Err(mpsc::RecvTimeoutError::Disconnected) => {}
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    self.output_sync = Some((self.output_path.clone(), receiver));
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "captured output synchronization exceeded its deadline",
                    ));
                }
            }
        }
        let output_path = self.output_path.clone();
        let (sender, receiver) = mpsc::sync_channel(1);
        std::thread::Builder::new()
            .name("srb-gui-output-sync".to_string())
            .spawn(move || {
                let result = OpenOptions::new()
                    .read(true)
                    .append(true)
                    .open(output_path)
                    .and_then(|file| file.sync_all());
                let _ = sender.send(result);
            })?;
        match receiver.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(result) => result,
            Err(mpsc::RecvTimeoutError::Timeout) => {
                self.output_sync = Some((self.output_path.clone(), receiver));
                Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "captured output synchronization exceeded its deadline",
                ))
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "captured output synchronizer disconnected",
            )),
        }
    }

    #[cfg(target_os = "linux")]
    fn wait_timeout(&mut self, timeout: Duration) -> Result<bool, String> {
        self.wait_linux(timeout, ParentSignalWait::Interrupt)
            .map(|(running, _)| running)
            .map_err(|error| error.to_string())
    }

    #[cfg(not(target_os = "linux"))]
    fn wait_timeout(&mut self, timeout: Duration) -> Result<bool, String> {
        let started = Instant::now();
        loop {
            match self.child.try_wait() {
                Ok(Some(status)) => {
                    self.exit_status = Some(status);
                    return Ok(false);
                }
                Ok(None) if started.elapsed() >= timeout => return Ok(true),
                Ok(None) => std::thread::sleep(Duration::from_millis(1)),
                Err(error) => {
                    return Err(format!("failed to query subprocess leader: {error}"));
                }
            }
        }
    }

    #[cfg(target_os = "linux")]
    fn wait_linux(
        &mut self,
        timeout: Duration,
        on_parent_signal: ParentSignalWait,
    ) -> io::Result<(bool, Option<GuiProcessStatus>)> {
        let started = Instant::now();
        loop {
            if let Some(relay) = self.parent_signal_relay.as_ref() {
                match on_parent_signal {
                    ParentSignalWait::Interrupt if relay.has_signal() => {
                        return Err(io::Error::new(
                            io::ErrorKind::Interrupted,
                            "parent SIGTERM/SIGINT is pending owned cleanup",
                        ));
                    }
                    ParentSignalWait::Escalate => {
                        if let Some(signal) = relay.take_signal() {
                            return Ok((true, Some(Self::signal_cause(signal))));
                        }
                    }
                    _ => {}
                }
            }
            let state = self.observe_linux_session()?;
            if !state.active() {
                return Ok((false, None));
            }
            let remaining = timeout.saturating_sub(started.elapsed());
            if remaining.is_zero() {
                return Ok((true, None));
            }
            std::thread::sleep(remaining.min(Duration::from_millis(10)));
        }
    }

    #[cfg(target_os = "linux")]
    fn observe_linux_session(&mut self) -> io::Result<LinuxSessionState> {
        let identity = self.linux_leader_identity();
        let snapshot = probe_linux_leader(identity.pid)?;
        validate_linux_leader_identity(identity, snapshot)?;
        self.leader_exit_observed |=
            linux_leader_exited_wnowait(identity.pid)? || snapshot.stat.state == b'Z';

        let mut members = scan_linux_owned_members(identity)?;
        let mut reaped_any = false;
        for member in &members {
            if member.state == b'Z' && member.parent_pid == std::process::id() as i32 {
                reaped_any |= reap_linux_owned_descendant(identity, *member)?;
            }
        }
        if reaped_any {
            members = scan_linux_owned_members(identity)?;
        }
        Ok(LinuxSessionState::from_members(
            identity,
            self.leader_exit_observed,
            &members,
        ))
    }

    #[cfg(target_os = "linux")]
    fn linux_leader_identity(&self) -> LinuxLeaderIdentity {
        self.leader_identity
    }

    #[cfg(target_os = "linux")]
    fn send_linux_signal(&self, signal: GroupSignal) -> LinuxSignalAttempt {
        let identity = self.linux_leader_identity();
        signal_linux_owned_members(identity, signal)
    }

    #[cfg(target_os = "linux")]
    fn reap_linux_leader_until(&mut self, deadline: Instant) -> io::Result<()> {
        if self.exit_status.is_some() {
            return Ok(());
        }
        loop {
            match self.child.try_wait()? {
                Some(status) => {
                    self.exit_status = Some(status);
                    return Ok(());
                }
                None if Instant::now() >= deadline => {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "owned leader reaping exceeded its deadline",
                    ));
                }
                None => {
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    std::thread::sleep(remaining.min(CLEANUP_RECONCILIATION_INTERVAL));
                }
            }
        }
    }

    #[cfg(target_os = "linux")]
    fn linux_signal_error_is_session_disappearance(error: &io::Error) -> bool {
        error.kind() == io::ErrorKind::NotFound
            || matches!(
                error.raw_os_error(),
                Some(code) if code == nix::libc::ESRCH
                    || code == nix::libc::ENOENT
                    || code == nix::libc::ECHILD
            )
    }

    /// Record a wait-phase failure.
    ///
    /// Session disappearance (`ESRCH`/`ENOENT`/`ECHILD`, or a `NotFound`
    /// `/proc` read) means the owned leader vanished while cleanup was
    /// waiting on it. That is only a defect when the later reaping proof
    /// fails, so it is kept proof-resolvable exactly like the equivalent
    /// signal-phase failure; every other wait error is a hard cleanup error.
    #[cfg(target_os = "linux")]
    fn push_wait_error(
        error: &io::Error,
        message: String,
        errors: &mut Vec<GuiCleanupError>,
        proof_resolvable_errors: &mut Vec<GuiCleanupError>,
    ) -> bool {
        let cleanup_error = GuiCleanupError::new(GuiCleanupErrorKind::Wait, message);
        let disappearance = Self::linux_signal_error_is_session_disappearance(error);
        if disappearance {
            proof_resolvable_errors.push(cleanup_error);
        } else {
            errors.push(cleanup_error);
        }
        disappearance
    }

    #[cfg(target_os = "linux")]
    fn record_io_cause(error: &io::Error, cause: &mut Option<GuiProcessStatus>) {
        match error.kind() {
            io::ErrorKind::Interrupted => *cause = Some(GuiProcessStatus::Interrupted),
            io::ErrorKind::TimedOut => *cause = Some(GuiProcessStatus::TimedOut),
            _ => {}
        }
    }
}

impl Drop for OwnedProcess {
    fn drop(&mut self) {
        let already_clean = self
            .completed_result
            .as_ref()
            .is_some_and(|result| result.cleanup_complete);
        if !already_clean {
            let result = self.stop();
            if !result.cleanup_complete {
                let mut exact_leader_reaped = false;
                match self.child.try_wait() {
                    Ok(Some(status)) => {
                        self.exit_status = Some(status);
                        exact_leader_reaped = true;
                    }
                    Ok(None) => {
                        let _ = self.child.kill();
                        let deadline = Instant::now() + self.deadlines.reap;
                        while Instant::now() < deadline {
                            match self.child.try_wait() {
                                Ok(Some(status)) => {
                                    self.exit_status = Some(status);
                                    exact_leader_reaped = true;
                                    break;
                                }
                                Ok(None) => std::thread::sleep(Duration::from_millis(1)),
                                Err(_) => break,
                            }
                        }
                    }
                    Err(_) => {}
                }
                eprintln!(
                    "bounded GUI process cleanup failed during Drop: pid={} \
                     exact_leader_reaped={} error={}",
                    self.pid,
                    exact_leader_reaped,
                    result.error.as_deref().unwrap_or("missing cleanup proof")
                );
            }
        }
        #[cfg(target_os = "linux")]
        drop(self.parent_signal_relay.take());
        self.close_child_handles();
    }
}

#[cfg(all(unix, not(target_os = "linux")))]
fn unsupported_cleanup_reason() -> &'static str {
    "stable session identity and descendant reaping are unsupported on this Unix platform"
}

#[cfg(windows)]
fn unsupported_cleanup_reason() -> &'static str {
    "Windows descendant cleanup requires a Job Object; leader-only cleanup is unsupported"
}

#[cfg(all(not(unix), not(windows)))]
fn unsupported_cleanup_reason() -> &'static str {
    "owned descendant cleanup is unsupported on this platform"
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy)]
enum GroupSignal {
    Int,
    Term,
    Kill,
}

#[cfg(target_os = "linux")]
struct LinuxSignalAttempt {
    target_signaled: bool,
    error: Option<io::Error>,
}

#[cfg(target_os = "linux")]
impl GroupSignal {
    fn from_gui(signal: GuiProcessSignal) -> Self {
        match signal {
            GuiProcessSignal::Int => Self::Int,
            GuiProcessSignal::Term => Self::Term,
            GuiProcessSignal::Kill => Self::Kill,
        }
    }

    fn to_gui(self) -> GuiProcessSignal {
        match self {
            Self::Int => GuiProcessSignal::Int,
            Self::Term => GuiProcessSignal::Term,
            Self::Kill => GuiProcessSignal::Kill,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Int => "SIGINT",
            Self::Term => "SIGTERM",
            Self::Kill => "SIGKILL",
        }
    }

    fn raw(self) -> nix::libc::c_int {
        match self {
            Self::Int => nix::libc::SIGINT,
            Self::Term => nix::libc::SIGTERM,
            Self::Kill => nix::libc::SIGKILL,
        }
    }
}

/// Set while one `ScopedParentSignalRelay` owns the process-wide handlers.
///
/// An atomic flag (not a held `MutexGuard`) keeps `OwnedProcess: Send`, so a
/// stop can run on a worker thread instead of blocking the UI frame thread.
#[cfg(target_os = "linux")]
static PARENT_SIGNAL_RELAY_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Releases `PARENT_SIGNAL_RELAY_ACTIVE` after the relay restored the
/// previous handlers (struct fields drop after `Drop::drop` runs).
#[cfg(target_os = "linux")]
struct ParentSignalRelayToken;

#[cfg(target_os = "linux")]
impl Drop for ParentSignalRelayToken {
    fn drop(&mut self) {
        PARENT_SIGNAL_RELAY_ACTIVE.store(false, Ordering::SeqCst);
    }
}

/// What a wait does when a parent SIGTERM/SIGINT is pending.
#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ParentSignalWait {
    /// Report it as `Interrupted` so the caller starts cleanup.
    Interrupt,
    /// Cleanup is running: end the current grace period and escalate.
    Escalate,
    /// Nothing stronger remains to send (after SIGKILL): keep waiting.
    Continue,
}

#[cfg(target_os = "linux")]
static PENDING_PARENT_SIGNAL: AtomicI32 = AtomicI32::new(0);

#[cfg(target_os = "linux")]
extern "C" fn record_parent_signal(signal: nix::libc::c_int) {
    let _ = PENDING_PARENT_SIGNAL.compare_exchange(0, signal, Ordering::SeqCst, Ordering::SeqCst);
}

#[cfg(target_os = "linux")]
struct ScopedParentSignalRelay {
    previous_sigterm: nix::sys::signal::SigAction,
    previous_sigint: nix::sys::signal::SigAction,
    _token: ParentSignalRelayToken,
}

#[cfg(target_os = "linux")]
impl ScopedParentSignalRelay {
    fn install() -> Result<Self, String> {
        use nix::sys::signal::{
            pthread_sigmask, sigaction, SaFlags, SigAction, SigHandler, SigSet, SigmaskHow, Signal,
        };

        if PARENT_SIGNAL_RELAY_ACTIVE
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            return Err("another GUI child already owns the process-wide signal relay".to_string());
        }
        let relay_token = ParentSignalRelayToken;
        let mut relay_mask = SigSet::empty();
        relay_mask.add(Signal::SIGTERM);
        relay_mask.add(Signal::SIGINT);
        let mut previous_mask = SigSet::empty();
        pthread_sigmask(
            SigmaskHow::SIG_BLOCK,
            Some(&relay_mask),
            Some(&mut previous_mask),
        )
        .map_err(|error| format!("failed to block parent signals during relay install: {error}"))?;

        PENDING_PARENT_SIGNAL.store(0, Ordering::SeqCst);
        let action = SigAction::new(
            SigHandler::Handler(record_parent_signal),
            SaFlags::SA_RESTART,
            relay_mask,
        );
        // SAFETY: the handler performs only an atomic store and both previous
        // actions are retained for scoped restoration below.
        let previous_sigterm = unsafe { sigaction(Signal::SIGTERM, &action) }.map_err(|error| {
            let _ = pthread_sigmask(SigmaskHow::SIG_SETMASK, Some(&previous_mask), None);
            format!("failed to install SIGTERM relay: {error}")
        })?;
        // SAFETY: same scoped handler and restoration contract as SIGTERM.
        let previous_sigint = match unsafe { sigaction(Signal::SIGINT, &action) } {
            Ok(previous) => previous,
            Err(error) => {
                // SAFETY: restore the action returned by the successful install.
                let _ = unsafe { sigaction(Signal::SIGTERM, &previous_sigterm) };
                let _ = pthread_sigmask(SigmaskHow::SIG_SETMASK, Some(&previous_mask), None);
                return Err(format!("failed to install SIGINT relay: {error}"));
            }
        };
        if let Err(error) = pthread_sigmask(SigmaskHow::SIG_SETMASK, Some(&previous_mask), None) {
            // SAFETY: both values came from successful sigaction calls above.
            let _ = unsafe { sigaction(Signal::SIGINT, &previous_sigint) };
            // SAFETY: both values came from successful sigaction calls above.
            let _ = unsafe { sigaction(Signal::SIGTERM, &previous_sigterm) };
            return Err(format!(
                "failed to restore signal mask after relay install: {error}"
            ));
        }
        Ok(Self {
            previous_sigterm,
            previous_sigint,
            _token: relay_token,
        })
    }

    fn has_signal(&self) -> bool {
        PENDING_PARENT_SIGNAL.load(Ordering::SeqCst) != 0
    }

    fn take_signal(&self) -> Option<GuiProcessSignal> {
        match PENDING_PARENT_SIGNAL.swap(0, Ordering::SeqCst) {
            nix::libc::SIGTERM => Some(GuiProcessSignal::Term),
            nix::libc::SIGINT => Some(GuiProcessSignal::Int),
            _ => None,
        }
    }
}

#[cfg(target_os = "linux")]
impl Drop for ScopedParentSignalRelay {
    fn drop(&mut self) {
        use nix::sys::signal::{pthread_sigmask, sigaction, SigSet, SigmaskHow, Signal};

        let mut relay_mask = SigSet::empty();
        relay_mask.add(Signal::SIGTERM);
        relay_mask.add(Signal::SIGINT);
        let mut previous_mask = SigSet::empty();
        let mask_blocked = match pthread_sigmask(
            SigmaskHow::SIG_BLOCK,
            Some(&relay_mask),
            Some(&mut previous_mask),
        ) {
            Ok(()) => true,
            Err(error) => {
                eprintln!("failed to block signals while restoring GUI relay: {error}");
                false
            }
        };
        // SAFETY: these are the exact actions retained at installation.
        if let Err(error) = unsafe { sigaction(Signal::SIGINT, &self.previous_sigint) } {
            eprintln!("failed to restore previous SIGINT handler: {error}");
        }
        // SAFETY: these are the exact actions retained at installation.
        if let Err(error) = unsafe { sigaction(Signal::SIGTERM, &self.previous_sigterm) } {
            eprintln!("failed to restore previous SIGTERM handler: {error}");
        }
        PENDING_PARENT_SIGNAL.store(0, Ordering::SeqCst);
        if mask_blocked {
            if let Err(error) = pthread_sigmask(SigmaskHow::SIG_SETMASK, Some(&previous_mask), None)
            {
                eprintln!("failed to restore signal mask after GUI relay: {error}");
            }
        }
    }
}

#[cfg(target_os = "linux")]
fn child_startup_gate(gate_fd: nix::libc::c_int) -> io::Result<()> {
    // SAFETY: every operation below is an async-signal-safe syscall between
    // fork and exec; the parent gate thread validates the reported identity.
    if unsafe { nix::libc::setsid() } == -1 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: scalar identity queries do not dereference caller memory.
    let identity = unsafe {
        [
            nix::libc::getpid(),
            nix::libc::getpgid(0),
            nix::libc::getsid(0),
        ]
    };
    if identity[0] <= 0 || identity[1] != identity[0] || identity[2] != identity[0] {
        return Err(io::Error::from_raw_os_error(nix::libc::EPERM));
    }
    // SAFETY: the fixed-size identity array remains valid for this write loop.
    let bytes = unsafe {
        std::slice::from_raw_parts(
            identity.as_ptr().cast::<u8>(),
            std::mem::size_of_val(&identity),
        )
    };
    let mut written = 0;
    while written < bytes.len() {
        // SAFETY: bytes points to initialized storage and gate_fd is inherited.
        let result = unsafe {
            nix::libc::write(
                gate_fd,
                bytes[written..].as_ptr().cast(),
                bytes.len() - written,
            )
        };
        if result > 0 {
            written += result as usize;
        } else if result == -1 && io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
            continue;
        } else {
            return Err(io::Error::last_os_error());
        }
    }
    let mut decision = STARTUP_GATE_ABORT;
    loop {
        // SAFETY: decision is one writable byte and gate_fd is inherited.
        let result = unsafe { nix::libc::read(gate_fd, (&mut decision as *mut u8).cast(), 1) };
        if result == 1 {
            break;
        }
        if result == -1 && io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
            continue;
        }
        return Err(if result == 0 {
            io::Error::from_raw_os_error(nix::libc::ECANCELED)
        } else {
            io::Error::last_os_error()
        });
    }
    if decision == STARTUP_GATE_GO {
        Ok(())
    } else {
        Err(io::Error::from_raw_os_error(nix::libc::ECANCELED))
    }
}

#[cfg(target_os = "linux")]
fn parent_startup_gate(
    mut gate: UnixStream,
    identity_probe: fn(i32) -> Result<LinuxLeaderIdentity, String>,
) -> Result<LinuxLeaderIdentity, String> {
    let mut bytes = [0_u8; std::mem::size_of::<[i32; 3]>()];
    let identity = gate
        .read_exact(&mut bytes)
        .map_err(|error| format!("failed to read child startup identity: {error}"))
        .and_then(|()| {
            let identity: [i32; 3] = std::array::from_fn(|index| {
                let start = index * std::mem::size_of::<i32>();
                let end = start + std::mem::size_of::<i32>();
                let mut fixed_bytes = [0_u8; std::mem::size_of::<i32>()];
                fixed_bytes.copy_from_slice(&bytes[start..end]);
                i32::from_ne_bytes(fixed_bytes)
            });
            let [pid, process_group_id, session_id] = identity;
            if pid <= 0 || process_group_id != pid || session_id != pid {
                return Err(format!(
                    "child startup identity is not a dedicated session: \
                     pid={pid} pgid={process_group_id} sid={session_id}"
                ));
            }
            let identity = identity_probe(pid)?;
            if identity.process_group_id != process_group_id || identity.session_id != session_id {
                return Err("child startup identity changed during binding".to_string());
            }
            Ok(identity)
        });
    let decision = if identity.is_ok() {
        STARTUP_GATE_GO
    } else {
        STARTUP_GATE_ABORT
    };
    if let Err(error) = gate.write_all(&[decision]) {
        return Err(match identity {
            Ok(_) => format!("failed to release identity-bound child: {error}"),
            Err(reason) => format!("{reason}; failed to abort gated child: {error}"),
        });
    }
    identity
}

#[cfg(target_os = "linux")]
fn spawn_linux_identity_gated(
    mut command: Command,
    identity_probe: fn(i32) -> Result<LinuxLeaderIdentity, String>,
) -> Result<(Child, LinuxLeaderIdentity), OwnedProcessSpawnError> {
    let (parent_gate, child_gate) = UnixStream::pair().map_err(OwnedProcessSpawnError::Io)?;
    parent_gate
        .set_read_timeout(Some(STARTUP_GATE_TIMEOUT))
        .map_err(OwnedProcessSpawnError::Io)?;
    parent_gate
        .set_write_timeout(Some(STARTUP_GATE_TIMEOUT))
        .map_err(OwnedProcessSpawnError::Io)?;
    child_gate
        .set_read_timeout(Some(STARTUP_GATE_TIMEOUT))
        .map_err(OwnedProcessSpawnError::Io)?;
    let child_gate_fd = child_gate.as_raw_fd();
    // SAFETY: child_startup_gate uses only async-signal-safe syscalls and the
    // descriptor remains open in both parent and child until spawn resolves.
    unsafe {
        command.pre_exec(move || child_startup_gate(child_gate_fd));
    }
    let gate_thread = std::thread::Builder::new()
        .name("srb-gui-startup-gate".to_string())
        .spawn(move || parent_startup_gate(parent_gate, identity_probe))
        .map_err(|error| OwnedProcessSpawnError::Supervision(error.to_string()))?;
    let child_result = command.spawn();
    drop(child_gate);
    let identity_result = gate_thread.join().map_err(|_| {
        OwnedProcessSpawnError::Supervision("child startup identity gate panicked".to_string())
    })?;
    match (child_result, identity_result) {
        (Ok(child), Ok(identity)) => Ok((child, identity)),
        (Ok(mut child), Err(reason)) => {
            let _ = child.kill();
            let _ = child.wait();
            Err(OwnedProcessSpawnError::Supervision(reason))
        }
        (Err(error), Err(_reason)) => Err(OwnedProcessSpawnError::Io(error)),
        (Err(error), Ok(_)) => Err(OwnedProcessSpawnError::Io(error)),
    }
}

#[cfg(target_os = "linux")]
const PR_SET_CHILD_SUBREAPER: nix::libc::c_int = 36;
#[cfg(target_os = "linux")]
const PR_GET_CHILD_SUBREAPER: nix::libc::c_int = 37;
#[cfg(target_os = "linux")]
static CHILD_SUBREAPER: OnceLock<Result<(), String>> = OnceLock::new();

/// Enable Linux child-subreaper behavior before launch.
///
/// This is intentionally process-wide. Orphaned GUI descendants are adopted
/// by this process, but cleanup only waits exact PIDs whose `/proc` identity is
/// proven to belong to the owned dedicated session or process group.
#[cfg(target_os = "linux")]
pub(crate) fn enable_linux_child_subreaper() -> Result<(), String> {
    CHILD_SUBREAPER
        .get_or_init(|| {
            // SAFETY: prctl receives scalar arguments and enables a documented
            // process-wide Linux attribute without dereferencing user memory.
            if unsafe { nix::libc::prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) } == -1 {
                return Err(io::Error::last_os_error().to_string());
            }
            let mut enabled: nix::libc::c_int = 0;
            // SAFETY: PR_GET_CHILD_SUBREAPER writes one c_int to the valid
            // pointer supplied below; the remaining variadic arguments are 0.
            if unsafe {
                nix::libc::prctl(
                    PR_GET_CHILD_SUBREAPER,
                    &mut enabled as *mut nix::libc::c_int,
                    0,
                    0,
                    0,
                )
            } == -1
            {
                return Err(io::Error::last_os_error().to_string());
            }
            if enabled != 1 {
                return Err("kernel did not retain child-subreaper state".to_string());
            }
            Ok(())
        })
        .clone()
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LinuxLeaderIdentity {
    pid: i32,
    process_group_id: i32,
    session_id: i32,
    start_time_ticks: u64,
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LinuxLeaderSnapshot {
    stat: LinuxProcessStat,
    getpgid_process_group_id: i32,
    getsid_session_id: i32,
}

#[cfg(target_os = "linux")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct LinuxProcessStat {
    pid: i32,
    state: u8,
    parent_pid: i32,
    process_group_id: i32,
    session_id: i32,
    start_time_ticks: u64,
}

#[cfg(target_os = "linux")]
#[derive(Debug)]
struct LinuxSessionState {
    leader_exited: bool,
    group_empty: bool,
    session_empty: bool,
}

#[cfg(target_os = "linux")]
impl LinuxSessionState {
    fn from_members(
        identity: LinuxLeaderIdentity,
        leader_exited: bool,
        members: &[LinuxProcessStat],
    ) -> Self {
        Self {
            leader_exited,
            group_empty: leader_exited
                && !members
                    .iter()
                    .any(|member| member.process_group_id == identity.process_group_id),
            session_empty: leader_exited
                && !members
                    .iter()
                    .any(|member| member.session_id == identity.session_id),
        }
    }

    fn active(&self) -> bool {
        !self.leader_exited || !self.group_empty || !self.session_empty
    }
}

#[cfg(target_os = "linux")]
fn capture_linux_leader_identity(pid: i32) -> Result<LinuxLeaderIdentity, String> {
    let snapshot = probe_linux_leader(pid).map_err(|error| error.to_string())?;
    if snapshot.stat.pid != pid
        || snapshot.stat.process_group_id != pid
        || snapshot.stat.session_id != pid
        || snapshot.getpgid_process_group_id != pid
        || snapshot.getsid_session_id != pid
    {
        return Err(format!(
            "process {pid} is not a dedicated session leader \
             (stat_pgid={}, stat_sid={}, getpgid={}, getsid={})",
            snapshot.stat.process_group_id,
            snapshot.stat.session_id,
            snapshot.getpgid_process_group_id,
            snapshot.getsid_session_id,
        ));
    }
    Ok(LinuxLeaderIdentity {
        pid,
        process_group_id: pid,
        session_id: pid,
        start_time_ticks: snapshot.stat.start_time_ticks,
    })
}

#[cfg(target_os = "linux")]
fn probe_linux_leader(pid: i32) -> io::Result<LinuxLeaderSnapshot> {
    let stat = read_linux_process_stat(pid)?;
    // SAFETY: getpgid/getsid only read kernel metadata for a numeric PID.
    let process_group_id = unsafe { nix::libc::getpgid(pid) };
    if process_group_id == -1 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: getsid only reads kernel metadata for a numeric PID.
    let session_id = unsafe { nix::libc::getsid(pid) };
    if session_id == -1 {
        return Err(io::Error::last_os_error());
    }
    Ok(LinuxLeaderSnapshot {
        stat,
        getpgid_process_group_id: process_group_id,
        getsid_session_id: session_id,
    })
}

#[cfg(target_os = "linux")]
fn validate_linux_leader_identity(
    expected: LinuxLeaderIdentity,
    current: LinuxLeaderSnapshot,
) -> io::Result<()> {
    if current.stat.pid != expected.pid
        || current.stat.start_time_ticks != expected.start_time_ticks
        || current.stat.process_group_id != expected.process_group_id
        || current.stat.session_id != expected.session_id
        || current.getpgid_process_group_id != expected.process_group_id
        || current.getsid_session_id != expected.session_id
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "owned session leader identity changed; refusing to signal \
                 (expected pid={} pgid={} sid={} start={}, current pid={} \
                 stat_pgid={} stat_sid={} getpgid={} getsid={} start={})",
                expected.pid,
                expected.process_group_id,
                expected.session_id,
                expected.start_time_ticks,
                current.stat.pid,
                current.stat.process_group_id,
                current.stat.session_id,
                current.getpgid_process_group_id,
                current.getsid_session_id,
                current.stat.start_time_ticks,
            ),
        ));
    }
    Ok(())
}

#[cfg(target_os = "linux")]
fn signal_linux_owned_members(
    identity: LinuxLeaderIdentity,
    signal: GroupSignal,
) -> LinuxSignalAttempt {
    let leader = match probe_linux_leader(identity.pid) {
        Ok(leader) => leader,
        Err(error) => {
            return LinuxSignalAttempt {
                target_signaled: false,
                error: Some(error),
            };
        }
    };
    if let Err(error) = validate_linux_leader_identity(identity, leader) {
        return LinuxSignalAttempt {
            target_signaled: false,
            error: Some(error),
        };
    }
    let mut members = match scan_linux_owned_members(identity) {
        Ok(members) => members,
        Err(error) => {
            return LinuxSignalAttempt {
                target_signaled: false,
                error: Some(error),
            };
        }
    };
    members.push(leader.stat);
    members.sort_unstable_by_key(|member| member.pid);
    members.dedup_by_key(|member| member.pid);

    let mut target_signaled = false;
    for member in members {
        if member.state == b'Z' {
            continue;
        }
        match send_bound_linux_process_signal(identity, member, signal) {
            Ok(sent) => target_signaled |= sent,
            Err(error) => {
                return LinuxSignalAttempt {
                    target_signaled,
                    error: Some(error),
                };
            }
        }
    }
    LinuxSignalAttempt {
        target_signaled,
        error: None,
    }
}

#[cfg(target_os = "linux")]
fn send_bound_linux_process_signal(
    identity: LinuxLeaderIdentity,
    expected: LinuxProcessStat,
    signal: GroupSignal,
) -> io::Result<bool> {
    if expected.pid <= 0
        || (expected.process_group_id != identity.process_group_id
            && expected.session_id != identity.session_id)
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("process {} is outside the owned session", expected.pid),
        ));
    }
    // SAFETY: pidfd_open takes only a numeric PID and flags. The returned file
    // descriptor pins that exact kernel process across the validation below.
    let raw_pidfd = unsafe {
        nix::libc::syscall(nix::libc::SYS_pidfd_open, expected.pid, 0) as nix::libc::c_int
    };
    if raw_pidfd == -1 {
        let error = io::Error::last_os_error();
        return if error.kind() == io::ErrorKind::NotFound {
            Ok(false)
        } else {
            Err(error)
        };
    }
    // SAFETY: raw_pidfd is a newly returned owned descriptor.
    let pidfd = unsafe { OwnedFd::from_raw_fd(raw_pidfd) };
    let current = match read_linux_process_stat(expected.pid) {
        Ok(current) => current,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(false),
        Err(error) => return Err(error),
    };
    if current.pid != expected.pid
        || current.start_time_ticks != expected.start_time_ticks
        || current.process_group_id != expected.process_group_id
        || current.session_id != expected.session_id
        || (current.process_group_id != identity.process_group_id
            && current.session_id != identity.session_id)
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "owned process {} identity changed before pidfd signal",
                expected.pid
            ),
        ));
    }
    if current.state == b'Z' {
        return Ok(false);
    }
    // SAFETY: pidfd pins the revalidated process; a null siginfo requests the
    // normal signal semantics and flags must be zero for this syscall.
    let result = unsafe {
        nix::libc::syscall(
            nix::libc::SYS_pidfd_send_signal,
            pidfd.as_raw_fd(),
            signal.raw(),
            std::ptr::null::<nix::libc::siginfo_t>(),
            0,
        )
    };
    if result == 0 {
        Ok(true)
    } else {
        let error = io::Error::last_os_error();
        if error.kind() == io::ErrorKind::NotFound {
            Ok(false)
        } else {
            Err(error)
        }
    }
}

#[cfg(target_os = "linux")]
fn linux_leader_exited_wnowait(pid: i32) -> io::Result<bool> {
    let mut info = MaybeUninit::<nix::libc::siginfo_t>::zeroed();
    // SAFETY: info is valid writable storage. WNOHANG bounds the call and
    // WNOWAIT deliberately keeps the leader waitable as the session anchor.
    let result = unsafe {
        nix::libc::waitid(
            nix::libc::P_PID,
            pid as nix::libc::id_t,
            info.as_mut_ptr(),
            nix::libc::WEXITED | nix::libc::WNOHANG | nix::libc::WNOWAIT,
        )
    };
    if result == -1 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: successful waitid initializes siginfo_t; si_pid is zero when no
    // exited child was observed.
    let observed_pid = unsafe { info.assume_init().si_pid() };
    match observed_pid {
        0 => Ok(false),
        observed if observed == pid => Ok(true),
        observed => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("waitid returned child {observed} for owned leader {pid}"),
        )),
    }
}

#[cfg(target_os = "linux")]
fn scan_linux_owned_members(identity: LinuxLeaderIdentity) -> io::Result<Vec<LinuxProcessStat>> {
    let mut members = Vec::new();
    for entry in fs::read_dir("/proc")? {
        let entry = entry?;
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|name| name.parse::<i32>().ok())
        else {
            continue;
        };
        if pid == identity.pid {
            continue;
        }
        match read_linux_process_stat(pid) {
            Ok(stat)
                if stat.process_group_id == identity.process_group_id
                    || stat.session_id == identity.session_id =>
            {
                members.push(stat);
            }
            Ok(_) => {}
            Err(error)
                if error.kind() == io::ErrorKind::NotFound
                    || error.raw_os_error() == Some(nix::libc::ESRCH) => {}
            Err(error) => return Err(error),
        }
    }
    Ok(members)
}

#[cfg(target_os = "linux")]
fn reap_linux_owned_descendant(
    identity: LinuxLeaderIdentity,
    expected: LinuxProcessStat,
) -> io::Result<bool> {
    let current = read_linux_process_stat(expected.pid)?;
    if current.start_time_ticks != expected.start_time_ticks
        || current.parent_pid != std::process::id() as i32
        || (current.process_group_id != identity.process_group_id
            && current.session_id != identity.session_id)
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "owned descendant {} identity changed before reap",
                expected.pid
            ),
        ));
    }
    let mut status = 0;
    // SAFETY: waitpid is scoped to one exact zombie PID whose start token,
    // parent, group, and session were revalidated immediately above.
    let waited = unsafe { nix::libc::waitpid(expected.pid, &mut status, nix::libc::WNOHANG) };
    match waited {
        pid if pid == expected.pid => Ok(true),
        0 => Ok(false),
        -1 => Err(io::Error::last_os_error()),
        other => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("waitpid returned unexpected descendant {other}"),
        )),
    }
}

#[cfg(target_os = "linux")]
fn read_linux_process_stat(pid: i32) -> io::Result<LinuxProcessStat> {
    let contents = fs::read(format!("/proc/{pid}/stat"))?;
    parse_linux_process_stat(&contents)
}

#[cfg(target_os = "linux")]
fn parse_linux_process_stat(contents: &[u8]) -> io::Result<LinuxProcessStat> {
    let command_start = contents
        .iter()
        .position(|&byte| byte == b'(')
        .ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                "missing /proc stat command start",
            )
        })?;
    let command_end = contents
        .iter()
        .rposition(|&byte| byte == b')')
        .ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "missing /proc stat command end")
        })?;
    if command_end <= command_start {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid /proc stat command field",
        ));
    }
    let pid = std::str::from_utf8(&contents[..command_start])
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?
        .trim()
        .parse::<i32>()
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    let fields: Vec<_> = contents[command_end + 1..]
        .split(|byte| byte.is_ascii_whitespace())
        .filter(|field| !field.is_empty())
        .collect();
    let parse_field = |index: usize, name: &str| -> io::Result<i32> {
        let field = fields
            .get(index)
            .copied()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, format!("missing {name}")))?;
        std::str::from_utf8(field)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?
            .parse::<i32>()
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))
    };
    let state = fields
        .first()
        .and_then(|field| field.first())
        .copied()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing process state"))?;
    let start_time = fields
        .get(19)
        .copied()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing process start time"))?;
    let start_time_ticks = std::str::from_utf8(start_time)
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?
        .parse::<u64>()
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))?;
    Ok(LinuxProcessStat {
        pid,
        state,
        parent_pid: parse_field(1, "parent pid")?,
        process_group_id: parse_field(2, "process group")?,
        session_id: parse_field(3, "session id")?,
        start_time_ticks,
    })
}
