use crate::command_contract::redact_secret_tokens;
use crate::command_contract::CommandContract;
#[cfg(target_os = "linux")]
use crate::process_supervisor::OwnedProcessSpawnError;
use crate::process_supervisor::{GuiProcessKind, OwnedProcess};
use std::fmt;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use tracing::trace;

const PYTHON_SCRIPT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../srb/__main__.py");
const PREVIEW_EXECUTABLE: &str = "srb";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GuiSpawnErrorKind {
    MissingExecutable,
    SupervisionUnavailable,
    Io,
}

#[derive(Clone, Eq, PartialEq)]
pub struct GuiSpawnError {
    pub kind: GuiSpawnErrorKind,
    pub argv: Vec<String>,
    pub message: String,
    pub raw_os_error: Option<i32>,
}

impl fmt::Debug for GuiSpawnError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("GuiSpawnError")
            .field("kind", &self.kind)
            .field("argv", &redact_secret_tokens(&self.argv))
            .field("message", &self.message)
            .field("raw_os_error", &self.raw_os_error)
            .finish()
    }
}

impl GuiSpawnError {
    fn new(kind: GuiSpawnErrorKind, argv: Vec<String>, error: impl fmt::Display) -> Self {
        Self {
            kind,
            argv,
            message: error.to_string(),
            raw_os_error: None,
        }
    }

    fn from_io(argv: Vec<String>, error: io::Error) -> Self {
        Self {
            kind: GuiSpawnErrorKind::Io,
            argv,
            message: error.to_string(),
            raw_os_error: error.raw_os_error(),
        }
    }
}

impl fmt::Display for GuiSpawnError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{:?}: {} (argv={:?})",
            self.kind,
            self.message,
            redact_secret_tokens(&self.argv)
        )
    }
}

impl std::error::Error for GuiSpawnError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiLaunchRequest {
    argv: Vec<String>,
    preview_command: String,
}

impl GuiLaunchRequest {
    #[must_use]
    pub fn from_command_contract(command_contract: CommandContract) -> Self {
        let argv = command_contract.argv();
        let preview_command = command_contract.shell_command(PREVIEW_EXECUTABLE);
        Self {
            argv,
            preview_command,
        }
    }

    #[must_use]
    pub fn preview_command(&self) -> &str {
        &self.preview_command
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiLaunchPlan {
    python_exe: String,
    argv: Vec<String>,
    env: Vec<(&'static str, String)>,
}

impl GuiLaunchPlan {
    #[must_use]
    pub fn from_request(launch_request: &GuiLaunchRequest) -> Self {
        Self::from_argv(launch_request.argv.clone())
    }

    fn from_argv(argv: Vec<String>) -> Self {
        Self {
            python_exe: isaacsim_python_exe(),
            argv,
            env: vec![
                ("DISPLAY", child_display_env()),
                (
                    "ROS_DOMAIN_ID",
                    std::env::var("ROS_DOMAIN_ID").unwrap_or_else(|_| "0".to_string()),
                ),
                (
                    "RMW_IMPLEMENTATION",
                    std::env::var("RMW_IMPLEMENTATION")
                        .unwrap_or_else(|_| "rmw_cyclonedds_cpp".to_string()),
                ),
            ],
        }
    }

    #[must_use]
    pub fn python_exe(&self) -> &str {
        &self.python_exe
    }

    pub(crate) fn command_argv(&self) -> Vec<String> {
        let mut argv = Vec::with_capacity(self.argv.len() + 2);
        argv.push(self.python_exe.clone());
        argv.extend(entrypoint_args(Path::new(PYTHON_SCRIPT)));
        argv.extend(self.argv.iter().cloned());
        argv
    }

    pub(crate) fn spawn_owned(&self, output_path: &Path) -> Result<OwnedProcess, GuiSpawnError> {
        let argv = self.command_argv();
        if self.python_exe().is_empty() {
            return Err(GuiSpawnError::new(
                GuiSpawnErrorKind::MissingExecutable,
                argv,
                "ISAACSIM_PYTHON is set to an empty string",
            ));
        }

        let stdout = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(output_path)
            .map_err(|error| GuiSpawnError::from_io(argv.clone(), error))?;
        let stderr = stdout
            .try_clone()
            .map_err(|error| GuiSpawnError::from_io(argv.clone(), error))?;

        let mut command = Command::new(self.python_exe());
        command
            .args(entrypoint_args(Path::new(PYTHON_SCRIPT)))
            .args(&self.argv)
            .envs(self.env.iter().map(|(key, value)| (*key, value)))
            .stdout(Stdio::from(stdout))
            .stderr(Stdio::from(stderr));
        OwnedProcess::spawn_bound(
            command,
            argv.clone(),
            GuiProcessKind::Interactive,
            output_path.to_path_buf(),
        )
        .map_err(|error| match error {
            // The log file is already open, so a spawn-time NotFound means the
            // interpreter (e.g. a stale ISAACSIM_PYTHON) is missing.
            OwnedProcessSpawnError::Io(error) if error.kind() == io::ErrorKind::NotFound => {
                GuiSpawnError {
                    kind: GuiSpawnErrorKind::MissingExecutable,
                    ..GuiSpawnError::from_io(argv, error)
                }
            }
            OwnedProcessSpawnError::Io(error) => GuiSpawnError::from_io(argv, error),
            OwnedProcessSpawnError::Supervision(message) => {
                GuiSpawnError::new(GuiSpawnErrorKind::SupervisionUnavailable, argv, message)
            }
        })
    }
}

fn child_display_env() -> String {
    std::env::var("SRB_DISPLAY")
        .or_else(|_| std::env::var("DISPLAY"))
        .unwrap_or_else(|_| ":0".to_string())
}

/// Use the checkout entry script when present, otherwise the installed `srb` module.
fn entrypoint_args(checkout_script: &Path) -> Vec<String> {
    if checkout_script.is_file() {
        vec![checkout_script.to_string_lossy().into_owned()]
    } else {
        vec!["-m".to_string(), "srb".to_string()]
    }
}

fn isaacsim_python_exe() -> String {
    if let Ok(python_exe) = std::env::var("ISAACSIM_PYTHON") {
        trace!("ISAACSIM_PYTHON: {}", python_exe);
        python_exe
    } else {
        let isaac_sim_path = ["ISAAC_PATH", "ISAACSIM_PATH"]
            .into_iter()
            .find_map(|name| std::env::var(name).ok().filter(|path| !path.is_empty()))
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                home::home_dir()
                    .unwrap_or_else(|| PathBuf::from("/root"))
                    .join("isaac-sim")
            });
        let isaac_sim_python_sh = isaac_sim_path.join("python.sh");
        if std::path::Path::new(&isaac_sim_python_sh).exists() {
            trace!("ISAACSIM_PYTHON: {}", isaac_sim_python_sh.display());
            isaac_sim_python_sh.display().to_string()
        } else {
            if let Some(path) = resolve_executable_on_path("python3") {
                trace!("ISAACSIM_PYTHON: {}", path.display());
                path.to_string_lossy().into_owned()
            } else {
                trace!("Unable to resolve python3 on PATH");
                "python3".to_owned()
            }
        }
    }
}

fn resolve_executable_on_path(executable: &str) -> Option<PathBuf> {
    let search_path = std::env::var_os("PATH")?;
    resolve_executable_in_path(executable, &search_path)
}

/// Find `executable` on `search_path` while preserving symlinks for virtualenv discovery.
fn resolve_executable_in_path(executable: &str, search_path: &std::ffi::OsStr) -> Option<PathBuf> {
    std::env::split_paths(search_path).find_map(|directory| {
        let candidate = directory.join(executable);
        is_executable_file(&candidate).then(|| std::path::absolute(&candidate).unwrap_or(candidate))
    })
}

fn is_executable_file(path: &Path) -> bool {
    let Ok(metadata) = path.metadata() else {
        return false;
    };
    if !metadata.is_file() {
        return false;
    }

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        metadata.permissions().mode() & 0o111 != 0
    }

    #[cfg(not(unix))]
    {
        true
    }
}
