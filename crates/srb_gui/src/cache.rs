use crate::gui_catalog::{GuiCatalog, GUI_CATALOG_SCHEMA_VERSION};
use std::{
    collections::BTreeMap,
    fmt, fs,
    path::{Path, PathBuf},
    sync::Arc,
};

/// Offline cache root resolved by Python and passed via `srb gui`; the GUI must not derive its
/// namespace.
pub(crate) const GUI_OFFLINE_CACHE_ROOT_ENV: &str = "SRB_GUI_OFFLINE_CACHE_ROOT";
const CURRENT_POINTER_FILE: &str = "current.json";
const GENERATIONS_DIR: &str = "generations";
const GENERATION_SCHEMA: &str = "srb-publication/1";
const GENERATION_MANIFEST_FILE: &str = ".srb-publication.json";
const GENERATION_PENDING_FILE: &str = ".srb-publication.pending";
const GUI_CATALOG_CACHE_FILE: &str = "gui_catalog.json";
const REQUIRED_CACHE_MEMBERS: [&str; 12] = [
    "env.json",
    "robot.json",
    "scenery.json",
    "object.json",
    "hardware.json",
    "config_keys.json",
    "enum_values.json",
    "agent_config_keys.json",
    "agent_enum_values.json",
    "action_modes.json",
    "task_defaults.json",
    GUI_CATALOG_CACHE_FILE,
];
const CACHE_STATUS_STRICT_COMMAND: &str = "srb cache status --json --strict";
const SRB_EXECUTABLE: &str = "srb";
const CACHE_STATUS_JSON_ARGS: [&str; 3] = ["cache", "status", "--json"];
const CACHE_STATUS_JSON_COMMAND: &str = "srb cache status --json";
const CACHE_STATUS_QUERY_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(30);
const CACHE_UPDATE_COMMAND: &str = "srb cache update";

#[derive(Debug, serde::Deserialize)]
struct GenerationPointer {
    schema: String,
    generation_id: String,
    manifest_sha256: String,
}

#[derive(Debug, serde::Deserialize)]
struct GenerationManifest {
    schema: String,
    generation_id: String,
    members: BTreeMap<String, GenerationMember>,
}

#[derive(Debug, serde::Deserialize)]
struct GenerationMember {
    sha256: String,
    size_bytes: u64,
}

pub type GuiCatalogCacheResult = Result<GuiCatalog, GuiCatalogCacheError>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct GuiCatalogCacheRecoveryCommand {
    label: &'static str,
    description: &'static str,
    command: &'static str,
}

impl GuiCatalogCacheRecoveryCommand {
    const fn new(label: &'static str, description: &'static str, command: &'static str) -> Self {
        Self {
            label,
            description,
            command,
        }
    }

    pub(crate) const fn label(&self) -> &'static str {
        self.label
    }

    pub(crate) const fn description(&self) -> &'static str {
        self.description
    }

    pub(crate) const fn command(&self) -> &'static str {
        self.command
    }
}

#[derive(Clone, Debug)]
enum GuiCatalogCacheCause {
    Io(Arc<std::io::Error>),
    Json(Arc<serde_json::Error>),
}

impl PartialEq for GuiCatalogCacheCause {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Io(left), Self::Io(right)) => {
                left.kind() == right.kind()
                    && left.raw_os_error() == right.raw_os_error()
                    && left.to_string() == right.to_string()
            }
            (Self::Json(left), Self::Json(right)) => {
                left.classify() == right.classify() && left.to_string() == right.to_string()
            }
            _ => false,
        }
    }
}

impl Eq for GuiCatalogCacheCause {}

impl GuiCatalogCacheCause {
    fn as_error(&self) -> &(dyn std::error::Error + 'static) {
        match self {
            Self::Io(error) => error.as_ref(),
            Self::Json(error) => error.as_ref(),
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuiCatalogCacheError {
    summary: String,
    recovery_hint: String,
    recovery_commands: Box<[GuiCatalogCacheRecoveryCommand; 2]>,
    cause: Option<GuiCatalogCacheCause>,
}

impl GuiCatalogCacheError {
    fn read(path: &Path, err: std::io::Error) -> Self {
        Self::with_cause(
            format!(
                "Failed to read GUI catalog cache file '{}': {err}",
                path.display()
            ),
            GuiCatalogCacheCause::Io(Arc::new(err)),
        )
    }

    fn parse(path: &Path, err: serde_json::Error) -> Self {
        Self::with_cause(
            format!(
                "Failed to parse GUI catalog cache file '{}': {err}",
                path.display()
            ),
            GuiCatalogCacheCause::Json(Arc::new(err)),
        )
    }

    fn contract(path: &Path, detail: impl Into<String>) -> Self {
        Self::new(format!(
            "Offline cache generation at '{}' failed identity validation: {}",
            path.display(),
            detail.into()
        ))
    }

    fn new(summary: String) -> Self {
        Self::with_optional_cause(summary, None)
    }

    fn with_cause(summary: String, cause: GuiCatalogCacheCause) -> Self {
        Self::with_optional_cause(summary, Some(cause))
    }

    fn with_optional_cause(summary: String, cause: Option<GuiCatalogCacheCause>) -> Self {
        let recovery_commands = gui_catalog_cache_recovery_commands();
        Self {
            summary,
            recovery_hint: cache_recovery_hint_from(recovery_commands),
            recovery_commands: Box::new(recovery_commands),
            cause,
        }
    }

    pub(crate) fn summary(&self) -> &str {
        &self.summary
    }

    pub(crate) fn recovery_hint(&self) -> &str {
        &self.recovery_hint
    }

    pub(crate) fn recovery_commands(&self) -> [GuiCatalogCacheRecoveryCommand; 2] {
        *self.recovery_commands
    }
}

impl fmt::Display for GuiCatalogCacheError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}\n{}", self.summary(), self.recovery_hint())
    }
}

impl std::error::Error for GuiCatalogCacheError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        self.cause.as_ref().map(GuiCatalogCacheCause::as_error)
    }
}

pub fn load_gui_catalog() -> GuiCatalogCacheResult {
    let root = offline_cache_root(std::env::var_os(GUI_OFFLINE_CACHE_ROOT_ENV), || {
        query_cache_status(std::ffi::OsStr::new(SRB_EXECUTABLE))
    })?;
    load_gui_catalog_from_generation(&root)
}

/// Resolve the generation root from the launcher or `srb cache status --json`.
/// Reject unknown layouts to avoid reading a stale, un-namespaced catalog.
fn offline_cache_root(
    launcher_root: Option<std::ffi::OsString>,
    query_cache_status: impl FnOnce() -> Result<Vec<u8>, String>,
) -> Result<PathBuf, GuiCatalogCacheError> {
    let Some(root) = launcher_root
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
    else {
        return query_cache_status()
            .and_then(|status| offline_root_from_cache_status(&status))
            .map_err(|reason| {
                GuiCatalogCacheError::new(format!(
                    "The offline cache location is unknown: {GUI_OFFLINE_CACHE_ROOT_ENV} is not \
                     set and `{CACHE_STATUS_JSON_COMMAND}` did not resolve it ({reason}). \
                     Launch the GUI through `srb gui`, or set {GUI_OFFLINE_CACHE_ROOT_ENV} to the \
                     absolute offline cache root: the directory above `generations/` in the entry \
                     paths that `{CACHE_STATUS_JSON_COMMAND}` prints \
                     (`<cache root>/namespaces/<namespace>/offline`)."
                ))
            });
    };
    if !root.is_absolute() {
        return Err(GuiCatalogCacheError::new(format!(
            "{GUI_OFFLINE_CACHE_ROOT_ENV} must be an absolute path, got '{}'. \
             Launch the GUI through `srb gui`.",
            root.display()
        )));
    }
    Ok(root)
}

fn query_cache_status(program: &std::ffi::OsStr) -> Result<Vec<u8>, String> {
    run_cache_status_query(program, &CACHE_STATUS_JSON_ARGS, CACHE_STATUS_QUERY_TIMEOUT)
}

/// Run `program args` with a timeout and process group so cleanup also stops
/// descendants holding the output pipes open.
fn run_cache_status_query(
    program: &std::ffi::OsStr,
    args: &[&str],
    timeout: std::time::Duration,
) -> Result<Vec<u8>, String> {
    let program_text = program.to_string_lossy();
    let command_text = format!("{program_text} {}", args.join(" "));
    let mut command = std::process::Command::new(program);
    command
        .args(args)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped());
    #[cfg(unix)]
    std::os::unix::process::CommandExt::process_group(&mut command, 0);
    let mut child = command
        .spawn()
        .map_err(|error| format!("could not run `{program_text}`: {error}"))?;
    let stdout = spawn_pipe_reader(child.stdout.take());
    let stderr = spawn_pipe_reader(child.stderr.take());
    let deadline = std::time::Instant::now() + timeout;
    let exit = loop {
        match child.try_wait() {
            Ok(Some(status)) => break Ok(status),
            Ok(None) if std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(20));
            }
            Ok(None) => {
                break Err(format!(
                    "`{command_text}` did not finish within {} s and was stopped",
                    timeout.as_secs_f64()
                ));
            }
            Err(error) => break Err(format!("could not wait for `{command_text}`: {error}")),
        }
    };
    if exit.is_err() {
        // The child is not reaped yet, so its group id cannot be reused.
        kill_process_group(child.id());
        let _ = child.kill();
        let _ = child.wait();
    } else if !pipe_readers_finish(
        &stdout,
        &stderr,
        std::time::Instant::now() + PIPE_DRAIN_GRACE,
    ) {
        // The child exited but a descendant still holds its pipes open; the
        // group id stays reserved while that member lives.
        kill_process_group(child.id());
    }
    let reader_deadline = std::time::Instant::now() + PIPE_DRAIN_GRACE;
    let stdout = join_pipe_reader(stdout, reader_deadline);
    let stderr = join_pipe_reader(stderr, reader_deadline);
    let status = exit?;
    let stdout =
        stdout.ok_or_else(|| format!("`{command_text}` exited but its output never closed"))?;
    if !status.success() {
        let stderr = stderr.unwrap_or_default();
        let stderr = String::from_utf8_lossy(&stderr);
        let last_line = stderr.lines().rev().find(|line| !line.trim().is_empty());
        return Err(format!(
            "`{command_text}` exited with {status}{}",
            last_line.map_or_else(String::new, |line| format!(": {}", line.trim()))
        ));
    }
    Ok(stdout)
}

/// Grace for pipe readers after the query exits or its group is killed.
const PIPE_DRAIN_GRACE: std::time::Duration = std::time::Duration::from_millis(500);

fn kill_process_group(group_leader: u32) {
    #[cfg(unix)]
    if let Ok(group) = i32::try_from(group_leader) {
        // SAFETY: `kill(2)` has no memory-safety preconditions; a negative
        // pid addresses the process group the query was spawned into.
        // ESRCH (group already empty) is expected and ignored.
        unsafe {
            let _ = nix::libc::kill(-group, nix::libc::SIGKILL);
        }
    }
}

fn pipe_readers_finish(
    stdout: &std::thread::JoinHandle<Vec<u8>>,
    stderr: &std::thread::JoinHandle<Vec<u8>>,
    deadline: std::time::Instant,
) -> bool {
    loop {
        if stdout.is_finished() && stderr.is_finished() {
            return true;
        }
        if std::time::Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
}

fn spawn_pipe_reader<R: std::io::Read + Send + 'static>(
    pipe: Option<R>,
) -> std::thread::JoinHandle<Vec<u8>> {
    std::thread::spawn(move || {
        let mut bytes = Vec::new();
        if let Some(mut pipe) = pipe {
            let _ = pipe.read_to_end(&mut bytes);
        }
        bytes
    })
}

fn join_pipe_reader(
    reader: std::thread::JoinHandle<Vec<u8>>,
    deadline: std::time::Instant,
) -> Option<Vec<u8>> {
    while !reader.is_finished() {
        if std::time::Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    reader.join().ok()
}

/// Derive one absolute offline root from agreeing entry paths shaped like
/// `<root>/generations/<id>/<member>`. The status `root` field is un-namespaced.
fn offline_root_from_cache_status(status: &[u8]) -> Result<PathBuf, String> {
    let status: serde_json::Value = serde_json::from_slice(status)
        .map_err(|error| format!("its output is not JSON: {error}"))?;
    let entries = status
        .get("entries")
        .and_then(serde_json::Value::as_array)
        .filter(|entries| !entries.is_empty())
        .ok_or("its output lists no cache entries")?;
    let mut resolved: Option<PathBuf> = None;
    for entry in entries {
        let path = entry
            .get("path")
            .and_then(serde_json::Value::as_str)
            .map(Path::new)
            .ok_or("a cache entry has no path")?;
        let generation_dir = path.parent().filter(|_| path.is_absolute());
        let generations_dir = generation_dir
            .and_then(Path::parent)
            .filter(|dir| dir.file_name() == Some(std::ffi::OsStr::new(GENERATIONS_DIR)));
        let root = generations_dir.and_then(Path::parent).ok_or_else(|| {
            format!(
                "entry path '{}' is not <offline root>/{GENERATIONS_DIR}/<id>/<member>",
                path.display()
            )
        })?;
        match &resolved {
            Some(previous) if previous != root => {
                return Err(format!(
                    "entries disagree on the offline root ('{}' vs '{}')",
                    previous.display(),
                    root.display()
                ));
            }
            Some(_) => {}
            None => resolved = Some(root.to_path_buf()),
        }
    }
    resolved.ok_or_else(|| String::from("its output lists no cache entries"))
}

pub(crate) fn gui_catalog_cache_recovery_commands() -> [GuiCatalogCacheRecoveryCommand; 2] {
    [
        GuiCatalogCacheRecoveryCommand::new(
            "Inspect cache",
            "Inspect generated cache health",
            CACHE_STATUS_STRICT_COMMAND,
        ),
        GuiCatalogCacheRecoveryCommand::new(
            "Regenerate cache",
            "Regenerate cache artifacts, then reopen the GUI",
            CACHE_UPDATE_COMMAND,
        ),
    ]
}

fn require_catalog_schema(path: &Path, catalog: &GuiCatalog) -> Result<(), GuiCatalogCacheError> {
    if catalog.schema_version != GUI_CATALOG_SCHEMA_VERSION {
        return Err(GuiCatalogCacheError::contract(
            path,
            format!(
                "unsupported GUI catalog schema_version {} (this build reads {})",
                catalog.schema_version, GUI_CATALOG_SCHEMA_VERSION
            ),
        ));
    }
    Ok(())
}

fn load_gui_catalog_from_generation(root: &Path) -> GuiCatalogCacheResult {
    let pointer_path = root.join(CURRENT_POINTER_FILE);
    let pointer_bytes = read_regular_file(&pointer_path)?;
    let pointer: GenerationPointer = parse_json(&pointer_path, &pointer_bytes)?;

    if pointer.schema != GENERATION_SCHEMA {
        return Err(GuiCatalogCacheError::contract(
            &pointer_path,
            format!("unsupported pointer schema {:?}", pointer.schema),
        ));
    }
    validate_generation_id(&pointer_path, &pointer.generation_id)?;
    validate_sha256(&pointer_path, &pointer.manifest_sha256, "pointer manifest")?;

    let generation = root.join(GENERATIONS_DIR).join(&pointer.generation_id);
    require_directory(&generation)?;
    let manifest_path = generation.join(GENERATION_MANIFEST_FILE);
    let manifest_bytes = read_regular_file(&manifest_path)?;
    let actual_manifest_sha256 = sha256_hex(&manifest_bytes);
    if actual_manifest_sha256 != pointer.manifest_sha256 {
        return Err(GuiCatalogCacheError::contract(
            &manifest_path,
            format!(
                "manifest SHA256 mismatch: expected {}, got {}",
                pointer.manifest_sha256, actual_manifest_sha256
            ),
        ));
    }
    let manifest: GenerationManifest = parse_json(&manifest_path, &manifest_bytes)?;
    if manifest.schema != GENERATION_SCHEMA {
        return Err(GuiCatalogCacheError::contract(
            &manifest_path,
            format!("unsupported manifest schema {:?}", manifest.schema),
        ));
    }
    if manifest.generation_id != pointer.generation_id {
        return Err(GuiCatalogCacheError::contract(
            &manifest_path,
            "manifest generation_id does not match the current pointer",
        ));
    }
    if manifest.members.len() != REQUIRED_CACHE_MEMBERS.len()
        || REQUIRED_CACHE_MEMBERS
            .iter()
            .any(|member| !manifest.members.contains_key(*member))
        || manifest
            .members
            .keys()
            .any(|member| !REQUIRED_CACHE_MEMBERS.contains(&member.as_str()))
    {
        return Err(GuiCatalogCacheError::contract(
            &manifest_path,
            "manifest does not contain exactly the twelve required cache members",
        ));
    }

    validate_generation_directory(&generation)?;
    let mut gui_catalog_bytes = None;
    for member_name in REQUIRED_CACHE_MEMBERS {
        let member_path = generation.join(member_name);
        let descriptor = manifest.members.get(member_name).ok_or_else(|| {
            GuiCatalogCacheError::contract(
                &manifest_path,
                format!("manifest is missing required member {member_name}"),
            )
        })?;
        validate_sha256(&manifest_path, &descriptor.sha256, member_name)?;
        let bytes = read_regular_file(&member_path)?;
        let actual_sha256 = sha256_hex(&bytes);
        if actual_sha256 != descriptor.sha256 {
            return Err(GuiCatalogCacheError::contract(
                &member_path,
                format!(
                    "member SHA256 mismatch: expected {}, got {}",
                    descriptor.sha256, actual_sha256
                ),
            ));
        }
        if bytes.len() as u64 != descriptor.size_bytes {
            return Err(GuiCatalogCacheError::contract(
                &member_path,
                format!(
                    "member size mismatch: expected {}, got {}",
                    descriptor.size_bytes,
                    bytes.len()
                ),
            ));
        }
        if member_name == GUI_CATALOG_CACHE_FILE {
            gui_catalog_bytes = Some(bytes);
        }
    }

    let gui_path = generation.join(GUI_CATALOG_CACHE_FILE);
    let bytes = gui_catalog_bytes.ok_or_else(|| {
        GuiCatalogCacheError::contract(&gui_path, "GUI catalog member was not read")
    })?;
    let catalog: GuiCatalog = serde_json::from_slice(&bytes)
        .map_err(|err| GuiCatalogCacheError::parse(&gui_path, err))?;
    require_catalog_schema(&gui_path, &catalog)?;
    Ok(catalog)
}

fn parse_json<T: serde::de::DeserializeOwned>(
    path: &Path,
    bytes: &[u8],
) -> Result<T, GuiCatalogCacheError> {
    serde_json::from_slice(bytes).map_err(|err| GuiCatalogCacheError::parse(path, err))
}

fn read_regular_file(path: &Path) -> Result<Vec<u8>, GuiCatalogCacheError> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;

        let file = fs::OpenOptions::new()
            .read(true)
            .custom_flags(nix::libc::O_NOFOLLOW | nix::libc::O_NONBLOCK)
            .open(path)
            .map_err(|err| {
                if err.raw_os_error() == Some(nix::libc::ELOOP) {
                    GuiCatalogCacheError::contract(
                        path,
                        "required cache identity is not a regular file",
                    )
                } else {
                    GuiCatalogCacheError::read(path, err)
                }
            })?;
        read_regular_file_from_descriptor(path, file)
    }

    #[cfg(not(unix))]
    {
        Err(GuiCatalogCacheError::contract(
            path,
            "cannot prove no-follow regular-file identity on this platform",
        ))
    }
}

#[cfg(unix)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct StableDescriptorMetadata {
    device: u64,
    inode: u64,
    mode: u32,
    link_count: u64,
    user: u32,
    group: u32,
    size: u64,
    modified_seconds: i64,
    modified_nanoseconds: i64,
    changed_seconds: i64,
    changed_nanoseconds: i64,
}

#[cfg(unix)]
impl StableDescriptorMetadata {
    fn from_metadata(metadata: &fs::Metadata) -> Self {
        use std::os::unix::fs::MetadataExt;

        Self {
            device: metadata.dev(),
            inode: metadata.ino(),
            mode: metadata.mode(),
            link_count: metadata.nlink(),
            user: metadata.uid(),
            group: metadata.gid(),
            size: metadata.size(),
            modified_seconds: metadata.mtime(),
            modified_nanoseconds: metadata.mtime_nsec(),
            changed_seconds: metadata.ctime(),
            changed_nanoseconds: metadata.ctime_nsec(),
        }
    }
}

#[cfg(unix)]
fn read_regular_file_from_descriptor(
    path: &Path,
    mut file: fs::File,
) -> Result<Vec<u8>, GuiCatalogCacheError> {
    use std::io::Read;

    let before = file
        .metadata()
        .map_err(|err| GuiCatalogCacheError::read(path, err))?;
    if !before.is_file() {
        return Err(GuiCatalogCacheError::contract(
            path,
            "required cache identity is not a regular file",
        ));
    }
    let before_identity = StableDescriptorMetadata::from_metadata(&before);

    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)
        .map_err(|err| GuiCatalogCacheError::read(path, err))?;

    let after = file
        .metadata()
        .map_err(|err| GuiCatalogCacheError::read(path, err))?;
    let after_identity = StableDescriptorMetadata::from_metadata(&after);
    if before_identity != after_identity || bytes.len() as u64 != after_identity.size {
        return Err(GuiCatalogCacheError::contract(
            path,
            "descriptor metadata changed while reading",
        ));
    }
    Ok(bytes)
}

fn require_directory(path: &Path) -> Result<(), GuiCatalogCacheError> {
    let metadata =
        fs::symlink_metadata(path).map_err(|err| GuiCatalogCacheError::read(path, err))?;
    if metadata.file_type().is_symlink() || !metadata.file_type().is_dir() {
        return Err(GuiCatalogCacheError::contract(
            path,
            "generation path is not a regular directory",
        ));
    }
    Ok(())
}

fn validate_generation_directory(path: &Path) -> Result<(), GuiCatalogCacheError> {
    for entry in fs::read_dir(path).map_err(|err| GuiCatalogCacheError::read(path, err))? {
        let entry = entry.map_err(|err| GuiCatalogCacheError::read(path, err))?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name == GENERATION_MANIFEST_FILE {
            continue;
        }
        if name == GENERATION_PENDING_FILE {
            let metadata = fs::symlink_metadata(entry.path())
                .map_err(|err| GuiCatalogCacheError::read(&entry.path(), err))?;
            if metadata.file_type().is_symlink() || !metadata.file_type().is_file() {
                return Err(GuiCatalogCacheError::contract(
                    &entry.path(),
                    "generation pending marker is not a regular file",
                ));
            }
            continue;
        }
        if !REQUIRED_CACHE_MEMBERS.contains(&name.as_ref()) {
            return Err(GuiCatalogCacheError::contract(
                &entry.path(),
                "generation contains an unexpected member",
            ));
        }
        let metadata = fs::symlink_metadata(entry.path())
            .map_err(|err| GuiCatalogCacheError::read(&entry.path(), err))?;
        if metadata.file_type().is_symlink() || !metadata.file_type().is_file() {
            return Err(GuiCatalogCacheError::contract(
                &entry.path(),
                "generation member is not a regular file",
            ));
        }
    }
    Ok(())
}

fn validate_generation_id(path: &Path, value: &str) -> Result<(), GuiCatalogCacheError> {
    if value.len() != 32 || !value.bytes().all(is_lower_hex) {
        return Err(GuiCatalogCacheError::contract(
            path,
            "generation_id is not a 32-character hexadecimal identity",
        ));
    }
    Ok(())
}

fn validate_sha256(path: &Path, value: &str, label: &str) -> Result<(), GuiCatalogCacheError> {
    if value.len() != 64 || !value.bytes().all(is_lower_hex) {
        return Err(GuiCatalogCacheError::contract(
            path,
            format!("{label} is not a 64-character hexadecimal SHA256"),
        ));
    }
    Ok(())
}

fn is_lower_hex(byte: u8) -> bool {
    byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)
}

fn sha256_hex(bytes: &[u8]) -> String {
    const ROUND_CONSTANTS: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut state = [
        0x6a09e667_u32,
        0xbb67ae85,
        0x3c6ef372,
        0xa54ff53a,
        0x510e527f,
        0x9b05688c,
        0x1f83d9ab,
        0x5be0cd19,
    ];
    let mut padded = bytes.to_vec();
    padded.push(0x80);
    while !(padded.len() + 8).is_multiple_of(64) {
        padded.push(0);
    }
    padded.extend_from_slice(&(bytes.len() as u64 * 8).to_be_bytes());

    for chunk in padded.as_chunks::<64>().0 {
        let mut schedule = [0_u32; 64];
        for (index, word) in schedule[..16].iter_mut().enumerate() {
            let start = index * 4;
            *word = u32::from_be_bytes([
                chunk[start],
                chunk[start + 1],
                chunk[start + 2],
                chunk[start + 3],
            ]);
        }
        for index in 16..64 {
            let s0 = schedule[index - 15].rotate_right(7)
                ^ schedule[index - 15].rotate_right(18)
                ^ (schedule[index - 15] >> 3);
            let s1 = schedule[index - 2].rotate_right(17)
                ^ schedule[index - 2].rotate_right(19)
                ^ (schedule[index - 2] >> 10);
            schedule[index] = schedule[index - 16]
                .wrapping_add(s0)
                .wrapping_add(schedule[index - 7])
                .wrapping_add(s1);
        }

        let mut working = state;
        for index in 0..64 {
            let s1 = working[4].rotate_right(6)
                ^ working[4].rotate_right(11)
                ^ working[4].rotate_right(25);
            let choose = (working[4] & working[5]) ^ ((!working[4]) & working[6]);
            let temp1 = working[7]
                .wrapping_add(s1)
                .wrapping_add(choose)
                .wrapping_add(ROUND_CONSTANTS[index])
                .wrapping_add(schedule[index]);
            let s0 = working[0].rotate_right(2)
                ^ working[0].rotate_right(13)
                ^ working[0].rotate_right(22);
            let majority =
                (working[0] & working[1]) ^ (working[0] & working[2]) ^ (working[1] & working[2]);
            let temp2 = s0.wrapping_add(majority);
            working[7] = working[6];
            working[6] = working[5];
            working[5] = working[4];
            working[4] = working[3].wrapping_add(temp1);
            working[3] = working[2];
            working[2] = working[1];
            working[1] = working[0];
            working[0] = temp1.wrapping_add(temp2);
        }
        for index in 0..8 {
            state[index] = state[index].wrapping_add(working[index]);
        }
    }

    state
        .iter()
        .map(|word| format!("{word:08x}"))
        .collect::<String>()
}

fn cache_recovery_hint_from(commands: [GuiCatalogCacheRecoveryCommand; 2]) -> String {
    commands
        .into_iter()
        .map(|command| format!("{}: `{}`.", command.description(), command.command()))
        .collect::<Vec<_>>()
        .join(" ")
}
