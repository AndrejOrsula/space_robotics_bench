use crate::cache::GuiCatalogCacheError;
use crate::env_cfg::TaskConfig;
use crate::gui_catalog::GuiCatalogTaskError;
use crate::gui_interface_controls::GuiInterfaceControlEvent;
use crate::gui_launch::GuiLaunchRequest;
use crate::gui_runtime::{
    GuiProcessResult, GuiRuntimeError, GuiRuntimeFailureKind, GuiRuntimeSession, GuiRuntimeSignals,
};
use crate::gui_selection::GuiTaskSelectionOutcome;
use crate::gui_session::GuiSessionState;
use crate::gui_task_browser::GuiTaskBrowser;
use tracing::error;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiWorkflowErrorCategory {
    InvalidInput,
    UnsupportedCapability,
    OptionalDependencyUnavailable,
    TransientExternalFailure,
    IntegrityFailure,
    SafetyViolation,
    InternalInvariant,
}

impl GuiWorkflowErrorCategory {
    pub const ALL: [Self; 7] = [
        Self::InvalidInput,
        Self::UnsupportedCapability,
        Self::OptionalDependencyUnavailable,
        Self::TransientExternalFailure,
        Self::IntegrityFailure,
        Self::SafetyViolation,
        Self::InternalInvariant,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            Self::InvalidInput => "invalid input",
            Self::UnsupportedCapability => "unsupported capability",
            Self::OptionalDependencyUnavailable => "optional dependency unavailable",
            Self::TransientExternalFailure => "transient external failure",
            Self::IntegrityFailure => "integrity failure",
            Self::SafetyViolation => "safety violation",
            Self::InternalInvariant => "internal invariant",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GuiWorkflowError {
    CatalogLoad(GuiCatalogCacheError),
    CatalogDefaults(GuiCatalogTaskError),
    CatalogSelection(GuiCatalogTaskError),
    Runtime(GuiRuntimeError),
}

impl GuiWorkflowError {
    #[must_use]
    pub const fn category(&self) -> GuiWorkflowErrorCategory {
        match self {
            Self::CatalogLoad(_) | Self::CatalogDefaults(_) => {
                GuiWorkflowErrorCategory::IntegrityFailure
            }
            Self::CatalogSelection(_) => GuiWorkflowErrorCategory::InvalidInput,
            Self::Runtime(error) => match error.failure_kind() {
                GuiRuntimeFailureKind::OptionalDependencyUnavailable => {
                    GuiWorkflowErrorCategory::OptionalDependencyUnavailable
                }
                GuiRuntimeFailureKind::UnsupportedCapability => {
                    GuiWorkflowErrorCategory::UnsupportedCapability
                }
                GuiRuntimeFailureKind::TransientExternalFailure => {
                    GuiWorkflowErrorCategory::TransientExternalFailure
                }
                GuiRuntimeFailureKind::IntegrityFailure => {
                    GuiWorkflowErrorCategory::IntegrityFailure
                }
                GuiRuntimeFailureKind::SafetyViolation => GuiWorkflowErrorCategory::SafetyViolation,
                GuiRuntimeFailureKind::InternalInvariant => {
                    GuiWorkflowErrorCategory::InternalInvariant
                }
            },
        }
    }
}

impl From<GuiCatalogCacheError> for GuiWorkflowError {
    fn from(error: GuiCatalogCacheError) -> Self {
        Self::CatalogLoad(error)
    }
}

impl From<GuiCatalogTaskError> for GuiWorkflowError {
    fn from(error: GuiCatalogTaskError) -> Self {
        Self::CatalogSelection(error)
    }
}

impl From<GuiRuntimeError> for GuiWorkflowError {
    fn from(error: GuiRuntimeError) -> Self {
        Self::Runtime(error)
    }
}

impl std::fmt::Display for GuiWorkflowError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CatalogLoad(error) => write!(formatter, "GUI catalog could not load: {error}"),
            Self::CatalogDefaults(error) => {
                write!(formatter, "GUI catalog defaults are invalid: {error}")
            }
            Self::CatalogSelection(error) => {
                write!(formatter, "invalid GUI catalog selection: {error}")
            }
            Self::Runtime(error) => write!(formatter, "GUI runtime workflow failed: {error}"),
        }
    }
}

impl std::error::Error for GuiWorkflowError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::CatalogLoad(error) => Some(error),
            Self::CatalogDefaults(error) => Some(error),
            Self::CatalogSelection(error) => Some(error),
            Self::Runtime(error) => Some(error),
        }
    }
}

pub trait GuiWorkflowRuntime {
    fn is_running(&mut self) -> Result<bool, GuiRuntimeError>;

    fn start<S: GuiRuntimeSignals>(
        &mut self,
        launch_request: &GuiLaunchRequest,
        signals: &S,
    ) -> Result<(), GuiRuntimeError>;

    fn restart_episode<S: GuiRuntimeSignals>(
        &mut self,
        signals: &S,
        collect_trajectory: bool,
        task_config: &TaskConfig,
    ) -> Result<bool, GuiRuntimeError>;

    /// Configuration of the subprocess last launched through `start`.
    fn launched_task_config(&self) -> Option<&TaskConfig>;

    /// Record the configuration a successful `start` launched.
    fn set_launched_task_config(&mut self, task_config: TaskConfig);

    /// Request asynchronous stop; retrieve its terminal result with `take_process_result`.
    fn request_stop<S: GuiRuntimeSignals>(&mut self, signals: &S) -> Result<(), GuiRuntimeError>;

    fn take_process_result(&mut self) -> Option<GuiProcessResult>;
}

impl GuiWorkflowRuntime for GuiRuntimeSession {
    fn is_running(&mut self) -> Result<bool, GuiRuntimeError> {
        GuiRuntimeSession::is_running(self)
    }

    fn start<S: GuiRuntimeSignals>(
        &mut self,
        launch_request: &GuiLaunchRequest,
        signals: &S,
    ) -> Result<(), GuiRuntimeError> {
        GuiRuntimeSession::start(self, launch_request, signals)
    }

    fn restart_episode<S: GuiRuntimeSignals>(
        &mut self,
        signals: &S,
        collect_trajectory: bool,
        task_config: &TaskConfig,
    ) -> Result<bool, GuiRuntimeError> {
        GuiRuntimeSession::restart_episode(self, signals, collect_trajectory, task_config)
    }

    fn launched_task_config(&self) -> Option<&TaskConfig> {
        GuiRuntimeSession::launched_task_config(self)
    }

    fn set_launched_task_config(&mut self, task_config: TaskConfig) {
        GuiRuntimeSession::set_launched_task_config(self, task_config);
    }

    fn request_stop<S: GuiRuntimeSignals>(&mut self, signals: &S) -> Result<(), GuiRuntimeError> {
        GuiRuntimeSession::request_stop(self, signals)
    }

    fn take_process_result(&mut self) -> Option<GuiProcessResult> {
        GuiRuntimeSession::take_process_result(self)
    }
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GuiWorkflowOutcome {
    pub task_selection: Option<GuiTaskSelectionOutcome>,
    pub process_results: Vec<GuiProcessResult>,
    pub errors: Vec<GuiWorkflowError>,
}

impl GuiWorkflowOutcome {
    fn record_result(&mut self, result: GuiProcessResult) {
        if !self.process_results.contains(&result) {
            self.process_results.push(result);
        }
    }

    fn record_error<E>(&mut self, context: &str, error: E)
    where
        E: Into<GuiWorkflowError>,
    {
        let workflow_error = error.into();
        error!("{context}: {workflow_error}");
        if let GuiWorkflowError::Runtime(runtime_error) = &workflow_error {
            if let Some(result) = runtime_error.process_result() {
                self.record_result(result.clone());
            }
        }
        self.errors.push(workflow_error);
    }
}

pub struct GuiWorkflow;

impl GuiWorkflow {
    pub fn handle_event<R, S>(
        event: Option<GuiInterfaceControlEvent>,
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
        runtime: &mut R,
        signals: &S,
    ) -> GuiWorkflowOutcome
    where
        R: GuiWorkflowRuntime,
        S: GuiRuntimeSignals,
    {
        let mut outcome = GuiWorkflowOutcome::default();
        match event {
            Some(GuiInterfaceControlEvent::StartSelectedTask) => {
                if let Err(runtime_error) =
                    Self::start_selected_task(task_browser, session, runtime, signals)
                {
                    outcome.record_error("Failed to start selected GUI task", runtime_error);
                }
            }
            Some(GuiInterfaceControlEvent::StartLaunchRequest(launch_request)) => {
                if let Err(runtime_error) = Self::start_task_with_launch_request(
                    task_browser,
                    session,
                    runtime,
                    signals,
                    &launch_request,
                ) {
                    outcome.record_error("Failed to start GUI launch request", runtime_error);
                }
            }
            Some(GuiInterfaceControlEvent::SelectCatalogTask(task)) => {
                match task_browser
                    .apply_task(&task)
                    .map_err(GuiWorkflowError::from)
                {
                    Ok(selection_outcome) => {
                        if selection_outcome.is_applied()
                            && !Self::runtime_running(runtime, &mut outcome)
                        {
                            session.start_task(task_browser.selection.task_config());
                        }
                        outcome.task_selection = Some(selection_outcome);
                    }
                    Err(workflow_error) => {
                        outcome.record_error("Failed to select catalog GUI task", workflow_error);
                    }
                }
            }
            Some(GuiInterfaceControlEvent::StartCatalogTask(task)) => {
                match task_browser
                    .apply_task(&task)
                    .map_err(GuiWorkflowError::from)
                {
                    Ok(selection_outcome) => {
                        outcome.task_selection = Some(selection_outcome);
                        if selection_outcome.is_applied() {
                            if let Err(runtime_error) =
                                Self::start_selected_task(task_browser, session, runtime, signals)
                            {
                                outcome.record_error(
                                    "Failed to start catalog GUI task",
                                    runtime_error,
                                );
                            }
                        }
                    }
                    Err(workflow_error) => {
                        outcome.record_error("Failed to select catalog GUI task", workflow_error);
                    }
                }
            }
            Some(GuiInterfaceControlEvent::SelectTaskById {
                task_id,
                catalog_task,
            }) => match task_browser
                .select_task(catalog_task.as_ref(), &task_id)
                .map_err(GuiWorkflowError::from)
            {
                Ok(selection_outcome) => {
                    // A mid-episode selection targets the next launch; the
                    // running episode keeps the gravity it was launched with.
                    if selection_outcome.is_applied()
                        && !Self::runtime_running(runtime, &mut outcome)
                    {
                        session.start_task(task_browser.selection.task_config());
                    }
                    outcome.task_selection = Some(selection_outcome);
                }
                Err(workflow_error) => {
                    outcome.record_error("Failed to select GUI task by id", workflow_error);
                }
            },
            Some(GuiInterfaceControlEvent::RestartEpisode) => {
                let request = session.restart_episode_request(task_browser.selection.task_config());
                let episode_config = Self::episode_task_config(runtime, request.task_config);
                if let Err(runtime_error) =
                    runtime.restart_episode(signals, request.collect_trajectory, &episode_config)
                {
                    outcome.record_error("Failed to restart GUI episode", runtime_error);
                }
            }
            Some(GuiInterfaceControlEvent::ExitToTasks) => {
                let runtime_was_running = match runtime.is_running() {
                    Ok(running) => running,
                    Err(runtime_error) => {
                        outcome.record_error("Failed to query GUI subprocess state", runtime_error);
                        true
                    }
                };
                let request = session.exit_to_tasks(&mut task_browser.selection);
                if runtime_was_running {
                    let episode_config = Self::episode_task_config(runtime, request.task_config);
                    if let Err(runtime_error) = runtime.restart_episode(
                        signals,
                        request.collect_trajectory,
                        &episode_config,
                    ) {
                        outcome.record_error(
                            "Failed to finalize GUI episode before stop",
                            runtime_error,
                        );
                    }
                    // Cleanup runs off the UI thread; its terminal result
                    // arrives through `take_process_result` on a later frame.
                    if let Err(runtime_error) = runtime.request_stop(signals) {
                        outcome.record_error("Failed to stop GUI subprocess", runtime_error);
                    }
                }
            }
            None => {}
        }

        if let Some(result) = runtime.take_process_result() {
            outcome.record_result(result);
        }
        outcome
    }

    /// Treat query errors as running when deciding selection side effects.
    fn runtime_running<R: GuiWorkflowRuntime>(
        runtime: &mut R,
        outcome: &mut GuiWorkflowOutcome,
    ) -> bool {
        match runtime.is_running() {
            Ok(running) => running,
            Err(runtime_error) => {
                outcome.record_error("Failed to query GUI subprocess state", runtime_error);
                true
            }
        }
    }

    /// The configuration an episode actually ran with: the launched one.
    /// The editable selection is only a fallback for a runtime that never
    /// recorded a launch (it then has no running episode to log either).
    fn episode_task_config<R: GuiWorkflowRuntime>(
        runtime: &R,
        selection_config: TaskConfig,
    ) -> TaskConfig {
        runtime
            .launched_task_config()
            .cloned()
            .unwrap_or(selection_config)
    }

    fn start_selected_task<R, S>(
        task_browser: &mut GuiTaskBrowser,
        session: &mut GuiSessionState,
        runtime: &mut R,
        signals: &S,
    ) -> Result<(), GuiRuntimeError>
    where
        R: GuiWorkflowRuntime,
        S: GuiRuntimeSignals,
    {
        let launch_request = task_browser.selection.launch_request();
        Self::start_task_with_launch_request(
            task_browser,
            session,
            runtime,
            signals,
            &launch_request,
        )
    }

    fn start_task_with_launch_request<R, S>(
        task_browser: &GuiTaskBrowser,
        session: &mut GuiSessionState,
        runtime: &mut R,
        signals: &S,
        launch_request: &GuiLaunchRequest,
    ) -> Result<(), GuiRuntimeError>
    where
        R: GuiWorkflowRuntime,
        S: GuiRuntimeSignals,
    {
        session.start_task(task_browser.selection.task_config());
        runtime.start(launch_request, signals)?;
        runtime.set_launched_task_config(task_browser.selection.task_config().clone());
        Ok(())
    }
}
