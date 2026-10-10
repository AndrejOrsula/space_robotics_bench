use crate::command_contract::{redact_secret_tokens, redact_shell_command};
use crate::env_cfg::TaskConfig;
use crate::gui_launch::{GuiLaunchPlan, GuiLaunchRequest, GuiSpawnError};
use crate::process_supervisor::OwnedProcess;
pub use crate::process_supervisor::{GuiProcessResult, GuiProcessStatus};
use std::collections::VecDeque;
use std::fmt;
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use tracing::{error, info, warn};

const LOGFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../logs/gui.txt");

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiRuntimeSignalError(String);

impl GuiRuntimeSignalError {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

impl fmt::Display for GuiRuntimeSignalError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for GuiRuntimeSignalError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum GuiRuntimeFailureKind {
    OptionalDependencyUnavailable,
    UnsupportedCapability,
    TransientExternalFailure,
    IntegrityFailure,
    SafetyViolation,
    InternalInvariant,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum GuiRuntimeLogErrorKind {
    CreateDirectory,
    Open,
    Read,
    Clock,
    StartWrite,
    CommandWrite,
    Serialize,
    DataWrite,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct GuiRuntimeLogError {
    kind: GuiRuntimeLogErrorKind,
    path: PathBuf,
    message: String,
    raw_os_error: Option<i32>,
}

impl GuiRuntimeLogError {
    fn io(kind: GuiRuntimeLogErrorKind, path: &std::path::Path, error: std::io::Error) -> Self {
        Self {
            kind,
            path: path.to_path_buf(),
            message: error.to_string(),
            raw_os_error: error.raw_os_error(),
        }
    }

    fn message(
        kind: GuiRuntimeLogErrorKind,
        path: &std::path::Path,
        message: impl Into<String>,
    ) -> Self {
        Self {
            kind,
            path: path.to_path_buf(),
            message: message.into(),
            raw_os_error: None,
        }
    }
}

impl fmt::Display for GuiRuntimeLogError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "GUI runtime log {:?} failed for '{}': {} (raw_os_error={:?})",
            self.kind,
            self.path.display(),
            self.message,
            self.raw_os_error
        )
    }
}

impl std::error::Error for GuiRuntimeLogError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiRuntimeError {
    message: String,
    process_result: Option<Box<GuiProcessResult>>,
    spawn_error: Option<Box<GuiSpawnError>>,
    log_error: Option<Box<GuiRuntimeLogError>>,
    failure_kind: GuiRuntimeFailureKind,
}

impl GuiRuntimeError {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            process_result: None,
            spawn_error: None,
            log_error: None,
            failure_kind: GuiRuntimeFailureKind::TransientExternalFailure,
        }
    }

    pub(crate) fn initialization_disabled() -> Self {
        Self {
            message: String::from("GUI runtime is disabled because GUI initialization failed"),
            process_result: None,
            spawn_error: None,
            log_error: None,
            failure_kind: GuiRuntimeFailureKind::InternalInvariant,
        }
    }

    fn from_messages_with_result(
        messages: Vec<String>,
        process_result: Option<Box<GuiProcessResult>>,
    ) -> Self {
        debug_assert!(!messages.is_empty());
        let failure_kind = match process_result
            .as_deref()
            .map(Self::failure_kind_for_process_result)
        {
            Some(failure_kind) => failure_kind,
            None => GuiRuntimeFailureKind::TransientExternalFailure,
        };
        Self {
            message: messages.join("; "),
            process_result,
            spawn_error: None,
            log_error: None,
            failure_kind,
        }
    }

    pub(crate) fn from_process_result(result: GuiProcessResult) -> Self {
        let message = match result.error.clone() {
            Some(error) => error,
            None => String::from("owned process cleanup failed"),
        };
        let failure_kind = Self::failure_kind_for_process_result(&result);
        Self {
            message,
            process_result: Some(Box::new(result)),
            spawn_error: None,
            log_error: None,
            failure_kind,
        }
    }

    pub(crate) fn from_message_and_process_result(
        message: impl Into<String>,
        result: GuiProcessResult,
    ) -> Self {
        let mut messages = vec![message.into()];
        if let Some(error) = &result.error {
            messages.push(error.clone());
        }
        let failure_kind = Self::failure_kind_for_process_result(&result);
        Self {
            message: messages.join("; "),
            process_result: Some(Box::new(result)),
            spawn_error: None,
            log_error: None,
            failure_kind,
        }
    }

    pub(crate) fn from_spawn_error(error: GuiSpawnError) -> Self {
        let failure_kind = match error.kind {
            crate::gui_launch::GuiSpawnErrorKind::MissingExecutable => {
                GuiRuntimeFailureKind::OptionalDependencyUnavailable
            }
            crate::gui_launch::GuiSpawnErrorKind::SupervisionUnavailable => {
                GuiRuntimeFailureKind::UnsupportedCapability
            }
            crate::gui_launch::GuiSpawnErrorKind::Io => {
                GuiRuntimeFailureKind::TransientExternalFailure
            }
        };
        Self {
            message: format!("failed to launch GUI subprocess: {}", error.message),
            process_result: None,
            spawn_error: Some(Box::new(error)),
            log_error: None,
            failure_kind,
        }
    }

    fn from_log_error(error: GuiRuntimeLogError) -> Self {
        Self {
            message: error.to_string(),
            process_result: None,
            spawn_error: None,
            log_error: Some(Box::new(error)),
            failure_kind: GuiRuntimeFailureKind::IntegrityFailure,
        }
    }

    fn failure_kind_for_process_result(result: &GuiProcessResult) -> GuiRuntimeFailureKind {
        if result.cleanup_complete {
            GuiRuntimeFailureKind::TransientExternalFailure
        } else {
            GuiRuntimeFailureKind::SafetyViolation
        }
    }

    #[must_use]
    pub fn process_result(&self) -> Option<&GuiProcessResult> {
        self.process_result.as_deref()
    }

    #[must_use]
    pub fn spawn_error(&self) -> Option<&GuiSpawnError> {
        self.spawn_error.as_deref()
    }

    pub(crate) fn log_error(&self) -> Option<&GuiRuntimeLogError> {
        self.log_error.as_deref()
    }

    pub(crate) const fn failure_kind(&self) -> GuiRuntimeFailureKind {
        self.failure_kind
    }
}

impl fmt::Display for GuiRuntimeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Some(result) = self.process_result() else {
            if let Some(spawn_error) = self.spawn_error() {
                return write!(
                    formatter,
                    "{} (spawn_kind={:?}, argv={:?}, raw_os_error={:?})",
                    self.message,
                    spawn_error.kind,
                    redact_secret_tokens(&spawn_error.argv),
                    spawn_error.raw_os_error
                );
            }
            if let Some(log_error) = self.log_error() {
                return write!(formatter, "{} ({log_error})", self.message);
            }
            return formatter.write_str(&self.message);
        };
        write!(
            formatter,
            "{} (kind={:?}, argv={:?}, pid={:?}, process_group_id={:?}, session_id={:?}, status={:?}, cause={:?}, first_signal={:?}, last_signal={:?}, term_sent={}, kill_sent={}, output={}, leader_reaped={}, descendants_reaped={}, group_empty={}, session_empty={}, output_complete={}, cleanup_complete={})",
            self.message,
            result.kind,
            redact_secret_tokens(&result.argv),
            result.pid,
            result.process_group_id,
            result.session_id,
            result.status,
            result.cause,
            result.first_signal,
            result.last_signal,
            result.term_sent,
            result.kill_sent,
            result.output_path.display(),
            result.reaped,
            result.descendants_reaped,
            result.group_empty,
            result.session_empty,
            result.output_complete,
            result.cleanup_complete,
        )
    }
}

impl std::error::Error for GuiRuntimeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match (self.spawn_error(), self.log_error()) {
            (Some(error), _) => Some(error),
            (None, Some(error)) => Some(error),
            (None, None) => None,
        }
    }
}

pub trait GuiRuntimeSignals {
    fn request_shutdown(&self) -> Result<(), GuiRuntimeSignalError>;
    fn reset_discard_dataset(&self) -> Result<(), GuiRuntimeSignalError>;
}

/// Trajectory `DATA` JSON with top-level config retained for compatibility and additive
/// `runtime_params`.
#[derive(serde::Serialize)]
struct TrajectoryRecord<'a> {
    #[serde(flatten)]
    task_config: &'a TaskConfig,
    runtime_params: Option<&'a serde_json::Value>,
}

pub struct GuiRuntimeLog {
    file: std::fs::File,
    path: PathBuf,
    collected_trajectories: usize,
    /// Runtime parameters for the next `DATA` row.
    runtime_params: Option<serde_json::Value>,
}

impl GuiRuntimeLog {
    fn open_default() -> Result<Self, GuiRuntimeLogError> {
        Self::open(PathBuf::from(LOGFILE_PATH).as_path())
    }

    fn open(logfile_path: &std::path::Path) -> Result<Self, GuiRuntimeLogError> {
        if let Some(parent) = logfile_path.parent() {
            if !parent.exists() {
                std::fs::create_dir_all(parent).map_err(|error| {
                    GuiRuntimeLogError::io(GuiRuntimeLogErrorKind::CreateDirectory, parent, error)
                })?;
            }
        }

        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .read(true)
            .append(true)
            .open(logfile_path)
            .map_err(|error| {
                GuiRuntimeLogError::io(GuiRuntimeLogErrorKind::Open, logfile_path, error)
            })?;
        let collected_trajectories = Self::count_collected_trajectories(&mut file, logfile_path)?;

        writeln!(
            file,
            "START, {}, {}",
            chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_err(|error| {
                    GuiRuntimeLogError::message(
                        GuiRuntimeLogErrorKind::Clock,
                        logfile_path,
                        error.to_string(),
                    )
                })?
                .as_secs(),
        )
        .map_err(|error| {
            GuiRuntimeLogError::io(GuiRuntimeLogErrorKind::StartWrite, logfile_path, error)
        })?;

        Ok(Self {
            file,
            path: logfile_path.to_path_buf(),
            collected_trajectories,
            runtime_params: None,
        })
    }

    pub fn collected_trajectories(&self) -> usize {
        self.collected_trajectories
    }

    fn write_command(&mut self, command: &str) -> Result<(), GuiRuntimeLogError> {
        writeln!(self.file, "CMD, {}", command).map_err(|error| {
            GuiRuntimeLogError::io(GuiRuntimeLogErrorKind::CommandWrite, &self.path, error)
        })
    }

    fn set_runtime_params(&mut self, runtime_params: Option<serde_json::Value>) {
        self.runtime_params = runtime_params;
    }

    /// Append a `DATA` row with launched config and additive `runtime_params` (`null` if
    /// unpublished).
    fn record_trajectory(&mut self, task_config: &TaskConfig) -> Result<(), GuiRuntimeLogError> {
        let record = TrajectoryRecord {
            task_config,
            runtime_params: self.runtime_params.as_ref(),
        };
        let config_json = serde_json::to_string(&record).map_err(|error| {
            GuiRuntimeLogError::message(
                GuiRuntimeLogErrorKind::Serialize,
                &self.path,
                error.to_string(),
            )
        })?;

        writeln!(
            self.file,
            "DATA, {}, {}, {}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_err(|error| {
                    GuiRuntimeLogError::message(
                        GuiRuntimeLogErrorKind::Clock,
                        &self.path,
                        error.to_string(),
                    )
                })?
                .as_secs(),
            chrono::Local::now().format("%Y-%m-%d %H:%M:%S"),
            config_json
        )
        .map_err(|error| {
            GuiRuntimeLogError::io(GuiRuntimeLogErrorKind::DataWrite, &self.path, error)
        })?;
        self.collected_trajectories += 1;
        Ok(())
    }

    fn count_collected_trajectories(
        file: &mut std::fs::File,
        path: &std::path::Path,
    ) -> Result<usize, GuiRuntimeLogError> {
        let count = {
            let mut reader = BufReader::new(file);
            let mut line = Vec::new();
            let mut count = 0;
            loop {
                line.clear();
                let bytes_read = reader.read_until(b'\n', &mut line).map_err(|error| {
                    GuiRuntimeLogError::io(GuiRuntimeLogErrorKind::Read, path, error)
                })?;
                if bytes_read == 0 {
                    break;
                }
                if line.ends_with(b"\n") && line.starts_with(b"DATA, ") {
                    count += 1;
                }
            }
            count
        };
        Ok(count)
    }
}

pub struct GuiRuntimeSession {
    process: Option<OwnedProcess>,
    log: Option<GuiRuntimeLog>,
    initialization_error: Option<GuiRuntimeError>,
    completed_result: Option<GuiProcessResult>,
    pending_results: VecDeque<GuiProcessResult>,
    pending_stop: Option<PendingStop>,
    launched_task_config: Option<TaskConfig>,
    published_runtime_params: Option<serde_json::Value>,
    launch_generation: u64,
    stop_worker_failure: Option<GuiRuntimeError>,
}

/// Cleanup running off the UI thread; returns the process if cleanup fails so
/// a later retry can keep supervising it.
struct PendingStop {
    handle: std::thread::JoinHandle<(GuiProcessResult, Option<OwnedProcess>)>,
}

impl Default for GuiRuntimeSession {
    fn default() -> Self {
        Self::unavailable(GuiRuntimeError::initialization_disabled())
    }
}

impl GuiRuntimeSession {
    pub fn try_new() -> Result<Self, GuiRuntimeError> {
        let log = GuiRuntimeLog::open_default().map_err(GuiRuntimeError::from_log_error)?;
        Ok(Self {
            process: None,
            log: Some(log),
            initialization_error: None,
            completed_result: None,
            pending_results: VecDeque::new(),
            pending_stop: None,
            launched_task_config: None,
            published_runtime_params: None,
            launch_generation: 0,
            stop_worker_failure: None,
        })
    }

    pub(crate) fn unavailable(error: GuiRuntimeError) -> Self {
        Self {
            process: None,
            log: None,
            initialization_error: Some(error),
            completed_result: None,
            pending_results: VecDeque::new(),
            pending_stop: None,
            launched_task_config: None,
            published_runtime_params: None,
            launch_generation: 0,
            stop_worker_failure: None,
        }
    }

    pub fn launched_task_config(&self) -> Option<&TaskConfig> {
        self.launched_task_config.as_ref()
    }

    pub fn set_launched_task_config(&mut self, task_config: TaskConfig) {
        self.launched_task_config = Some(task_config);
    }

    pub fn set_published_runtime_params(&mut self, runtime_params: serde_json::Value) {
        self.published_runtime_params = Some(runtime_params);
    }

    pub(crate) fn forget_previous_launch(&mut self) {
        self.launched_task_config = None;
        self.published_runtime_params = None;
        self.launch_generation = self.launch_generation.wrapping_add(1);
    }

    pub fn launch_generation(&self) -> u64 {
        self.launch_generation
    }

    fn ensure_available(&self) -> Result<(), GuiRuntimeError> {
        match &self.initialization_error {
            Some(error) => Err(error.clone()),
            None => Ok(()),
        }
    }

    pub fn is_stopping(&mut self) -> bool {
        self.poll_pending_stop();
        self.pending_stop.is_some()
    }

    fn poll_pending_stop(&mut self) {
        let finished = self
            .pending_stop
            .as_ref()
            .is_some_and(|pending| pending.handle.is_finished());
        if finished {
            self.join_pending_stop();
        }
    }

    fn join_pending_stop(&mut self) {
        let Some(pending) = self.pending_stop.take() else {
            return;
        };
        match pending.handle.join() {
            Ok((result, process)) => {
                if let Some(process) = process {
                    self.process = Some(process);
                }
                // Surface worker failures through the next `is_running`.
                if result.error.is_some() || !result.cleanup_complete {
                    self.stop_worker_failure =
                        Some(GuiRuntimeError::from_process_result(result.clone()));
                } else if result.status == GuiProcessStatus::Exited
                    && result
                        .exit_status
                        .as_ref()
                        .is_some_and(|status| !status.success())
                {
                    let message = match result.exit_status.as_ref().and_then(|status| status.code())
                    {
                        Some(code) => format!("GUI subprocess exited with code {code}"),
                        None => String::from("GUI subprocess exited unsuccessfully"),
                    };
                    self.stop_worker_failure = Some(
                        GuiRuntimeError::from_message_and_process_result(message, result.clone()),
                    );
                }
                self.record_process_result(result);
            }
            Err(_) => {
                self.stop_worker_failure =
                    Some(GuiRuntimeError::new("GUI subprocess stop worker panicked"));
            }
        }
    }

    /// Stop asynchronously; remain running until bounded cleanup queues its result.
    pub fn request_stop<S: GuiRuntimeSignals>(
        &mut self,
        signals: &S,
    ) -> Result<(), GuiRuntimeError> {
        self.ensure_available()?;
        self.poll_pending_stop();
        if self.pending_stop.is_some() {
            return Ok(());
        }
        if self.process.is_none() {
            if self.completed_result.is_some() {
                return Ok(());
            }
            return Err(GuiRuntimeError::new(
                "no GUI subprocess or completed process result is available",
            ));
        }
        let signal_error = signals
            .request_shutdown()
            .err()
            .map(|error| format!("failed to request ROS shutdown: {error}"));
        info!("Stopping subprocess on worker thread...");
        // Hand over ownership only after spawn succeeds, avoiding unsupervised
        // cleanup in `Drop` if thread creation fails.
        let (handoff, receiver) = std::sync::mpsc::sync_channel::<OwnedProcess>(1);
        let spawned = std::thread::Builder::new()
            .name("srb-gui-stop".to_string())
            .spawn(move || {
                let mut process = receiver
                    .recv()
                    .expect("GUI stop worker must receive its owned process");
                let result = process.stop();
                let retained = if result.cleanup_complete {
                    None
                } else {
                    Some(process)
                };
                (result, retained)
            });
        match spawned {
            Ok(handle) => {
                let process = self
                    .process
                    .take()
                    .expect("owned process presence was checked above");
                if let Err(std::sync::mpsc::SendError(process)) = handoff.send(process) {
                    // Preserve ownership if the worker exits before handoff.
                    self.process = Some(process);
                    let _ = handle.join();
                    return Err(GuiRuntimeError::new(
                        "GUI stop worker exited before receiving the owned process",
                    ));
                }
                self.pending_stop = Some(PendingStop { handle });
            }
            Err(error) => {
                // Fall back to bounded synchronous cleanup while still owned.
                error!("failed to spawn GUI stop worker: {error}; stopping inline");
                let mut messages = vec![format!("failed to spawn GUI stop worker: {error}")];
                let mut process_result = None;
                if let Some(message) = &signal_error {
                    messages.push(message.clone());
                }
                if let Err(stop_error) = self.stop_process() {
                    process_result = stop_error.process_result.clone();
                    messages.push(stop_error.to_string());
                }
                return Err(Self::combine_errors_with_result(
                    messages.into_iter().map(Some),
                    process_result,
                ));
            }
        }
        match signal_error {
            Some(message) => Err(GuiRuntimeError::new(message)),
            None => Ok(()),
        }
    }

    pub fn is_running(&mut self) -> Result<bool, GuiRuntimeError> {
        self.ensure_available()?;
        self.poll_pending_stop();
        if let Some(error) = self.stop_worker_failure.take() {
            return Err(error);
        }
        if self.pending_stop.is_some() {
            return Ok(true);
        }
        if let Some(parent_signal) = self
            .process
            .as_ref()
            .and_then(OwnedProcess::take_parent_signal)
        {
            return self.stop_after_parent_signal(parent_signal);
        }
        let running = if let Some(process) = self.process.as_mut() {
            process.is_running()
        } else {
            return Ok(false);
        };

        match running {
            Ok(true) => Ok(true),
            Ok(false) => {
                info!("Subprocess and its owned session exited.");
                self.stop_process().map(|_| false)
            }
            Err(wait_error) => {
                let cleanup_error = self.stop_process().err();
                let process_result = cleanup_error
                    .as_ref()
                    .and_then(|error| error.process_result.clone());
                Err(Self::combine_errors_with_result(
                    [
                        Some(wait_error),
                        cleanup_error.map(|error| error.to_string()),
                    ],
                    process_result,
                ))
            }
        }
    }

    pub fn collected_trajectories(&self) -> usize {
        self.log
            .as_ref()
            .map_or(0, GuiRuntimeLog::collected_trajectories)
    }

    pub fn take_process_result(&mut self) -> Option<GuiProcessResult> {
        self.poll_pending_stop();
        self.pending_results.pop_front()
    }

    pub fn start(
        &mut self,
        launch_request: &GuiLaunchRequest,
        signals: &impl GuiRuntimeSignals,
    ) -> Result<(), GuiRuntimeError> {
        self.ensure_available()?;
        if !cfg!(target_os = "linux") {
            // Only Linux proves session identity and descendant cleanup.
            return Err(GuiRuntimeError::new(
                "launching GUI subprocesses is supported on Linux only; \
                 descendant cleanup cannot be proven on this platform",
            ));
        }
        self.poll_pending_stop();
        if self.pending_stop.is_some() {
            return Err(GuiRuntimeError::new(
                "a GUI subprocess stop is still in progress",
            ));
        }
        self.prepare_for_launch(signals)?;
        self.forget_previous_launch();

        let launch_plan = GuiLaunchPlan::from_request(launch_request);

        // Redact the logged/previewed command; execute the original argv.
        let command_string = redact_shell_command(launch_request.preview_command());
        info!("Starting subprocess command: {}", command_string);
        let log = self
            .log
            .as_mut()
            .ok_or_else(GuiRuntimeError::initialization_disabled)?;
        log.write_command(&command_string)
            .map_err(GuiRuntimeError::from_log_error)?;

        let output_path = PathBuf::from(LOGFILE_PATH);
        let mut process = launch_plan
            .spawn_owned(&output_path)
            .map_err(GuiRuntimeError::from_spawn_error)?;
        let startup_result = process.wait_for_startup();
        if let Some(parent_signal) = process.take_parent_signal() {
            return Err(self.failed_start_after_parent_signal(process, parent_signal));
        }
        match startup_result {
            Ok(true) => {
                if process.leader_exited() {
                    return Err(
                        self.failed_start(process, "subprocess exited during startup".to_string())
                    );
                }

                info!("Subprocess and its owned process group started.");
                self.completed_result = None;
                self.process = Some(process);
                Ok(())
            }
            Ok(false) => {
                Err(self.failed_start(process, "subprocess exited during startup".to_string()))
            }
            Err(wait_error) => Err(self.failed_start(
                process,
                format!("failed while waiting for subprocess startup: {wait_error}"),
            )),
        }
    }

    /// Clean up the previous process before launch; ROS shutdown failure does not block bounded
    /// cleanup.
    fn prepare_for_launch(
        &mut self,
        signals: &impl GuiRuntimeSignals,
    ) -> Result<(), GuiRuntimeError> {
        if self.process.is_none() {
            return Ok(());
        }
        if let Err(error) = signals.request_shutdown() {
            warn!("failed to request ROS shutdown before relaunch: {error}; stopping the previous subprocess directly");
        }
        self.stop_process().map(|_| ())
    }

    pub fn restart_episode(
        &mut self,
        signals: &impl GuiRuntimeSignals,
        collect_trajectory: bool,
        task_config: &TaskConfig,
    ) -> Result<bool, GuiRuntimeError> {
        self.ensure_available()?;
        if self.is_running()? {
            info!("Restarting episode");
            if collect_trajectory {
                info!("Logging collected trajectory data point (on restart)");
                let log = self
                    .log
                    .as_mut()
                    .ok_or_else(GuiRuntimeError::initialization_disabled)?;
                log.set_runtime_params(self.published_runtime_params.clone());
                log.record_trajectory(task_config)
                    .map_err(GuiRuntimeError::from_log_error)?;
            }
            signals.reset_discard_dataset().map_err(|error| {
                GuiRuntimeError::new(format!("failed to publish reset request: {error}"))
            })?;
            Ok(true)
        } else {
            error!("Cannot restart episode: subprocess is not running");
            Ok(false)
        }
    }

    fn stop_process(&mut self) -> Result<Option<GuiProcessResult>, GuiRuntimeError> {
        let Some(process) = self.process.as_mut() else {
            return Ok(None);
        };

        info!("Stopping subprocess...");
        let result = process.stop();
        if result.cleanup_complete {
            self.process = None;
        }
        self.record_process_result(result.clone());
        if result.error.is_some() || !result.cleanup_complete {
            return Err(GuiRuntimeError::from_process_result(result));
        }
        if result.status == GuiProcessStatus::Exited
            && result
                .exit_status
                .as_ref()
                .is_some_and(|status| !status.success())
        {
            let message = match result.exit_status.as_ref().and_then(|status| status.code()) {
                Some(code) => format!("GUI subprocess exited with code {code}"),
                None => String::from("GUI subprocess exited unsuccessfully"),
            };
            return Err(GuiRuntimeError::from_message_and_process_result(
                message, result,
            ));
        }

        info!(
            "Subprocess stopped: pid={:?}, status={:?}, first_signal={:?}, output={}",
            result.pid,
            result.status,
            result.first_signal,
            result.output_path.display(),
        );
        Ok(Some(result))
    }

    fn stop_after_parent_signal(
        &mut self,
        parent_signal: crate::process_supervisor::GuiProcessSignal,
    ) -> Result<bool, GuiRuntimeError> {
        let Some(process) = self.process.as_mut() else {
            return Ok(false);
        };
        let result = process.stop_after_parent_signal(parent_signal);
        if result.cleanup_complete {
            self.process = None;
        }
        self.record_process_result(result.clone());
        if result.cleanup_complete {
            info!(
                "Parent {:?} relayed to GUI subprocess; cleanup completed",
                parent_signal
            );
            Ok(false)
        } else {
            Err(GuiRuntimeError::from_process_result(result))
        }
    }

    fn failed_start(&mut self, mut process: OwnedProcess, message: String) -> GuiRuntimeError {
        let result = process.stop();
        if !result.cleanup_complete {
            self.process = Some(process);
        }
        self.record_process_result(result.clone());
        GuiRuntimeError::from_message_and_process_result(message, result)
    }

    fn failed_start_after_parent_signal(
        &mut self,
        mut process: OwnedProcess,
        parent_signal: crate::process_supervisor::GuiProcessSignal,
    ) -> GuiRuntimeError {
        let result = process.stop_after_parent_signal(parent_signal);
        if !result.cleanup_complete {
            self.process = Some(process);
        }
        self.record_process_result(result.clone());
        GuiRuntimeError::from_message_and_process_result(
            format!("parent {parent_signal:?} interrupted subprocess startup"),
            result,
        )
    }

    fn record_process_result(&mut self, result: GuiProcessResult) {
        if result.cleanup_complete {
            // A genuinely clean retry supersedes queued failed attempts for
            // the same owned process. Those failures were already returned
            // directly to their callers and must not remain as stale terminal
            // state after ownership has been fully discharged.
            self.pending_results
                .retain(|pending| pending.pid != result.pid || pending.cleanup_complete);
            self.completed_result = Some(result.clone());
        }
        self.pending_results.push_back(result);
    }

    fn combine_errors_with_result(
        messages: impl IntoIterator<Item = Option<String>>,
        process_result: Option<Box<GuiProcessResult>>,
    ) -> GuiRuntimeError {
        let messages: Vec<_> = messages.into_iter().flatten().collect();
        debug_assert!(!messages.is_empty());
        GuiRuntimeError::from_messages_with_result(messages, process_result)
    }
}

impl Drop for GuiRuntimeSession {
    fn drop(&mut self) {
        self.join_pending_stop();
        if let Err(error) = self.stop_process() {
            error!("Failed to clean up GUI subprocess during drop: {error}");
        }
    }
}
