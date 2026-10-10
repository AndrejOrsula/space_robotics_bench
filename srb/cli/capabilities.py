"""Import-light optional dependency probes for CLI preflight.

The CLI parser avoids optional runtimes; probes only hint at missing dependencies.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, NoReturn

from srb.utils.capability_probe import CapabilityProbe, ModuleProbe
from srb.utils.failures import (
    FailureError,
    FailureInfo,
    InvalidInputError,
    OptionalDependencyUnavailableError,
    UnsupportedCapabilityError,
)
from srb.utils.isaacsim import require_isaac_sim

_GUI_BINARY_ENV_VARS = ("SRB_GUI_BINARY", "SRB_GUI_BIN", "SRB_GUI_PATH")
_GUI_PREBUILT_RELATIVE_PATHS = (
    Path("target") / "release" / "gui",
    Path("target") / "debug" / "gui",
)
_GUI_TOOLCHAIN = ("cargo", "rustc")
_GUI_TOOLCHAIN_INSTALL_HINT = (
    "the Rust toolchain with rustup (cargo and rustc) and the native GUI "
    "build dependencies; see docs/src/reference/gui.md"
)
_GUI_COMPONENT = "srb.cli.gui"
_ISAAC_COMPONENT = "srb.cli.isaac.sim"
_ISAAC_INSTALL_HINT = (
    "a supported Isaac Sim build, set ISAACSIM_PATH, or put the "
    "isaacsim launcher on PATH"
)


def _executable_file(path: Path) -> bool:
    """Return whether *path* is a regular executable file.

    Check mode bits as well as ``os.access`` because root can access non-executable
    files.
    """
    try:
        mode = path.stat().st_mode
    except OSError:
        return False
    if not stat.S_ISREG(mode):
        return False
    if os.name != "nt" and not mode & 0o111:
        return False
    return os.access(path, os.X_OK)


def _raise_executable_invalid(
    *,
    executable: str,
    feature: str,
    code: str,
    reason: str,
    install_hint: str,
    details: dict[str, Any] | None = None,
) -> NoReturn:
    error_details: dict[str, Any] = {
        "feature": feature,
        "executable": executable,
        "reason": reason,
        "install_hint": install_hint,
    }
    if details:
        error_details.update(details)
    raise InvalidInputError(
        code=code,
        message=(
            f"The {feature} executable is invalid: {executable!r} ({reason}). "
            f"Remediation: {install_hint}."
        ),
        component=_ISAAC_COMPONENT
        if feature == "Isaac Sim launcher"
        else _GUI_COMPONENT,
        details=error_details,
    )


def _raise_executable_unavailable(
    *,
    executable: str,
    feature: str,
    code: str,
    install_hint: str,
    details: dict[str, Any] | None = None,
) -> NoReturn:
    error_details: dict[str, Any] = {
        "feature": feature,
        "executable": executable,
        "install_hint": install_hint,
    }
    if details:
        error_details.update(details)
    raise OptionalDependencyUnavailableError(
        code=code,
        message=(
            f"The optional feature '{feature}' is unavailable: launcher/executable "
            f"{executable!r} was not found. Install {install_hint}."
        ),
        component=_ISAAC_COMPONENT
        if feature == "Isaac Sim launcher"
        else _GUI_COMPONENT,
        details=error_details,
    )


def _validate_executable(
    executable: str | Path,
    *,
    feature: str,
    unavailable_code: str,
    invalid_code: str,
    install_hint: str,
) -> None:
    """Validate a resolved executable without starting a process."""
    value = str(executable).strip()
    if not value:
        _raise_executable_invalid(
            executable=value,
            feature=feature,
            code=invalid_code,
            reason="the configured value is empty",
            install_hint=install_hint,
        )

    candidate = Path(value).expanduser()
    is_command_name = not candidate.is_absolute() and os.sep not in value
    if is_command_name:
        resolved = shutil.which(value)
        if resolved is None:
            _raise_executable_unavailable(
                executable=value,
                feature=feature,
                code=unavailable_code,
                install_hint=install_hint,
                details={"lookup": "PATH"},
            )
        candidate = Path(resolved)
    elif not candidate.exists():
        _raise_executable_unavailable(
            executable=value,
            feature=feature,
            code=unavailable_code,
            install_hint=install_hint,
            details={"lookup": "configured path"},
        )

    if not candidate.is_file():
        _raise_executable_invalid(
            executable=value,
            feature=feature,
            code=invalid_code,
            reason="the path is not a regular file",
            install_hint=install_hint,
            details={"resolved_path": candidate.as_posix()},
        )
    if not _executable_file(candidate):
        _raise_executable_invalid(
            executable=value,
            feature=feature,
            code=invalid_code,
            reason="the file is not executable",
            install_hint=install_hint,
            details={"resolved_path": candidate.as_posix()},
        )


def _missing_gui_toolchain() -> tuple[str, ...]:
    return tuple(tool for tool in _GUI_TOOLCHAIN if shutil.which(tool) is None)


def resolve_gui_prebuilt() -> Path | None:
    """Resolve a configured or local prebuilt GUI executable.

    Use the local target only without Rust; otherwise ``cargo run`` rebuilds current
    sources.
    """
    for variable in _GUI_BINARY_ENV_VARS:
        configured = os.environ.get(variable)
        if configured is None:
            continue
        configured = configured.strip()
        _validate_executable(
            configured,
            feature="native SRB GUI",
            unavailable_code="cli.gui.prebuilt_unavailable",
            invalid_code="cli.gui.prebuilt_invalid",
            install_hint=(
                "set the variable to an executable prebuilt GUI binary or unset it "
                "to use cargo"
            ),
        )
        return Path(configured).expanduser()

    if not _missing_gui_toolchain():
        return None

    repo_root = Path(__file__).resolve().parents[2]
    for relative_path in _GUI_PREBUILT_RELATIVE_PATHS:
        candidate = repo_root / relative_path
        if _executable_file(candidate):
            from srb.utils import logging

            logging.warning(
                "The Rust toolchain is unavailable; launching the prebuilt GUI "
                f"{candidate}, which may be older than the GUI sources."
            )
            return candidate
    return None


def require_gui_toolchain() -> None:
    """Require the native toolchain used by the source-built GUI path."""
    missing = _missing_gui_toolchain()
    if not missing:
        return
    raise OptionalDependencyUnavailableError(
        code="cli.gui.native_toolchain_unavailable",
        message=(
            "The native SRB GUI toolchain is unavailable: missing "
            f"{', '.join(missing)}. Install {_GUI_TOOLCHAIN_INSTALL_HINT}, or "
            "set SRB_GUI_BINARY to an executable prebuilt GUI."
        ),
        component=_GUI_COMPONENT,
        details={
            "feature": "native SRB GUI",
            "missing_tools": list(missing),
            "install_hint": _GUI_TOOLCHAIN_INSTALL_HINT,
            "prebuilt_env": "SRB_GUI_BINARY",
        },
    )


def require_isaac_sim_launcher(executable: str | Path) -> None:
    """Validate the Isaac Sim launcher before the process supervisor runs."""
    _validate_executable(
        executable,
        feature="Isaac Sim launcher",
        unavailable_code="cli.isaac.launcher_unavailable",
        invalid_code="cli.isaac.launcher_invalid",
        install_hint=_ISAAC_INSTALL_HINT,
    )


def resolve_isaac_sim_launcher() -> str:
    """Resolve and validate the Isaac Sim launcher at execution time."""
    from srb.utils.isaacsim import get_isaacsim_app

    try:
        executable = get_isaacsim_app()
    except OSError as exc:
        raise InvalidInputError(
            code="cli.isaac.launcher_invalid",
            message=(
                "The Isaac Sim launcher could not be inspected: "
                f"{exc}. Remediation: {_ISAAC_INSTALL_HINT}."
            ),
            component=_ISAAC_COMPONENT,
            details={
                "feature": "Isaac Sim launcher",
                "reason": "launcher path inspection failed",
                "exception_type": type(exc).__name__,
                "exception": str(exc),
                "install_hint": _ISAAC_INSTALL_HINT,
            },
        ) from exc
    require_isaac_sim_launcher(executable)
    return str(executable)


def require_dependency(
    *,
    feature: str,
    component: str,
    dependencies: tuple[str, ...],
    install_hint: str,
    required_symbols: tuple[str, ...] = (),
    callable_symbols: tuple[str, ...] = (),
    version_attribute: str | None = None,
) -> None:
    """Fail with a typed, actionable error when imports are not healthy."""
    CapabilityProbe(
        feature=feature,
        component=component,
        dependencies=dependencies,
        install_hint=install_hint,
        required_symbols=required_symbols,
        callable_symbols=callable_symbols,
        version_attribute=version_attribute,
    ).require()


def require_ephemeris() -> None:
    """Require SPICE symbols used by the ephemeris provider."""
    CapabilityProbe(
        feature="SPICE ephemeris",
        component="srb.cli.ephemeris",
        dependencies=("spiceypy",),
        install_hint="pip install 'srb[spice]'",
        module_probes=(
            ModuleProbe(
                "spiceypy",
                required_symbols=("furnsh", "str2et", "pxform", "spkpos"),
                callable_symbols=("furnsh", "str2et", "pxform", "spkpos"),
                version_attribute="__version__",
            ),
        ),
    ).require()


def require_hydra() -> None:
    """Require Hydra/OmegaConf symbols used by config adapters.

    Check before importing ``srb.hydra.real``, which imports these at module scope.
    """
    CapabilityProbe(
        feature="Hydra configuration",
        component="srb.cli.hydra",
        dependencies=("hydra", "hydra.core.config_store", "omegaconf"),
        install_hint="pip install 'srb[cli]'",
        module_probes=(
            ModuleProbe(
                "hydra",
                required_symbols=("main",),
                callable_symbols=("main",),
                version_attribute="__version__",
            ),
            ModuleProbe(
                "hydra.core.config_store",
                required_symbols=("ConfigStore",),
                callable_symbols=("ConfigStore",),
            ),
            ModuleProbe(
                "omegaconf",
                required_symbols=("DictConfig", "OmegaConf.to_container"),
                callable_symbols=("OmegaConf.to_container",),
                version_attribute="__version__",
            ),
        ),
    ).require()


def require_ros() -> None:
    """Require the rclpy/message symbols used by the ROS agent adapter."""
    CapabilityProbe(
        feature="ROS 2 agent interface",
        component="srb.cli.agent.ros",
        dependencies=(
            "rclpy",
            "rclpy.node",
            "rclpy.qos",
            "geometry_msgs.msg",
            "std_msgs.msg",
        ),
        install_hint="ROS 2 Jazzy plus the SRB ROS dependencies",
        module_probes=(
            ModuleProbe(
                "rclpy",
                required_symbols=("init", "ok"),
                callable_symbols=("init", "ok"),
            ),
            ModuleProbe(
                "rclpy.node", required_symbols=("Node",), callable_symbols=("Node",)
            ),
            ModuleProbe(
                "rclpy.qos",
                required_symbols=("QoSProfile",),
                callable_symbols=("QoSProfile",),
            ),
            ModuleProbe(
                "geometry_msgs.msg",
                required_symbols=("Twist",),
                callable_symbols=("Twist",),
            ),
            ModuleProbe(
                "std_msgs.msg",
                required_symbols=("Bool", "Float64"),
                callable_symbols=("Bool", "Float64"),
            ),
        ),
    ).require()


def require_event_backend(name: str) -> None:
    """Require the selected event-camera backend's adapter symbols."""
    if name == "noop":
        return
    if name in {"v2e", "v2e_streaming"}:
        CapabilityProbe(
            feature=f"event-camera backend {name!r}",
            component="srb.cli.event_camera.backend.v2e",
            dependencies=("v2ecore", "v2ecore.emulator"),
            # Git-pinned providers use dependency groups because PyPI rejects direct references in wheel metadata.
            install_hint=(
                "pip install 'srb[event-v2e]' "
                "'v2e @ git+https://github.com/SensorsINI/v2e.git"
                "@cac638444557a224ba6ee5c29260916f152c00bd'"
                + (
                    # The streaming provider includes SRB's compatibility patch.
                    "; then python -m "
                    "srb.core.sensor.event_camera.backends.v2e_install --apply"
                    if name == "v2e_streaming"
                    else ""
                )
            ),
            module_probes=(
                ModuleProbe(
                    "v2ecore.emulator",
                    required_symbols=("EventEmulator",),
                    callable_symbols=("EventEmulator",),
                ),
            ),
        ).require()
        return
    if name == "metavision":
        CapabilityProbe(
            feature="event-camera backend 'metavision'",
            component="srb.cli.event_camera.backend.metavision",
            dependencies=(
                "metavision_core_ml",
                "metavision_core_ml.event_simulator",
            ),
            install_hint="the openEB installation plus 'srb[event-metavision]'",
            module_probes=(
                ModuleProbe(
                    "metavision_core_ml.event_simulator",
                    required_symbols=("GPUEventSimulator",),
                    callable_symbols=("GPUEventSimulator",),
                ),
            ),
        ).require()
        return
    if name == "v2ce":
        _require_v2ce_backend()
        return
    raise UnsupportedCapabilityError(
        code="event_camera.backend.unknown",
        message=f"Unknown event-camera backend {name!r}; no adapter is registered.",
        component="srb.cli.event_camera",
        details={"backend": name},
    )


def _require_v2ce_backend() -> None:
    """Accept either the package or source-tree V2CE adapter shape."""
    package_probe = CapabilityProbe(
        feature="event-camera backend 'v2ce'",
        component="srb.cli.event_camera.backend.v2ce",
        dependencies=("v2ce.inference",),
        install_hint="pip install 'srb[event-v2ce]' or install the V2CE-Toolbox",
        module_probes=(
            ModuleProbe(
                "v2ce.inference",
                required_symbols=("V2CE",),
                callable_symbols=("V2CE",),
            ),
        ),
    )
    try:
        package_probe.require()
        return
    except OptionalDependencyUnavailableError as package_error:
        toolbox_probe = CapabilityProbe(
            feature="event-camera backend 'v2ce'",
            component="srb.cli.event_camera.backend.v2ce",
            dependencies=("scripts.LDATI", "scripts.v2ce_3d"),
            install_hint=(
                "pip install 'srb[event-v2ce]' or add the V2CE-Toolbox "
                "repository root to PYTHONPATH"
            ),
            module_probes=(
                ModuleProbe(
                    "scripts.LDATI",
                    required_symbols=("sample_voxel_statistical",),
                    callable_symbols=("sample_voxel_statistical",),
                ),
                ModuleProbe(
                    "scripts.v2ce_3d",
                    required_symbols=("V2ce3d",),
                    callable_symbols=("V2ce3d",),
                ),
            ),
        )
        try:
            toolbox_probe.require()
        except OptionalDependencyUnavailableError as toolbox_error:
            raise OptionalDependencyUnavailableError(
                code="cli.capability.v2ce_provider_unavailable",
                message=(
                    "The optional feature 'event-camera backend 'v2ce'' is "
                    "unavailable: neither the package provider nor the "
                    "V2CE-Toolbox provider is healthy. Install "
                    "pip install 'srb[event-v2ce]' or add the V2CE-Toolbox "
                    "repository root to PYTHONPATH."
                ),
                component="srb.cli.event_camera.backend.v2ce",
                details={
                    "package_error": package_error.info.message,
                    "toolbox_error": toolbox_error.info.message,
                },
            ) from toolbox_error


def require_event_reconstruction(*, operator: str, include_lpips: bool) -> None:
    """Probe optional reconstruction adapters before file/output mutation."""
    if operator == "e2vid":
        CapabilityProbe(
            feature="E2VID event reconstruction",
            component="srb.cli.dataset.event.reconstruction",
            dependencies=("e2vid.utils.loading_utils",),
            # Git-pinned provider; see the v2e hint in require_event_backend.
            install_hint=(
                "pip install 'srb[event-reconstruction]' "
                "'e2vid @ git+https://github.com/uzh-rpg/e2calib.git"
                "@f35317a47809688ad140a67d50a23654c98612ea#subdirectory=python'"
            ),
            module_probes=(
                ModuleProbe(
                    "e2vid.utils.loading_utils",
                    required_symbols=("load_model",),
                    callable_symbols=("load_model",),
                ),
            ),
        ).require()
    if include_lpips:
        CapabilityProbe(
            feature="LPIPS event reconstruction scoring",
            component="srb.cli.dataset.event.reconstruction",
            dependencies=("lpips", "torch"),
            install_hint="pip install 'srb[event-reconstruction]'",
            module_probes=(
                ModuleProbe(
                    "lpips", required_symbols=("LPIPS",), callable_symbols=("LPIPS",)
                ),
                ModuleProbe(
                    "torch",
                    required_symbols=("no_grad",),
                    callable_symbols=("no_grad",),
                    version_attribute="__version__",
                ),
            ),
        ).require()


_ISOLATED_ALGORITHM_PROBE_TIMEOUT_S = 300.0
_PROBE_EXCERPT_CHARS = 200

# A private verdict file keeps provider stdout out of the parent protocol.
_ISOLATED_ALGORITHM_PROBE_SCRIPT = """\
import faulthandler
import json
import sys
from pathlib import Path

# A provider that kills its own interpreter is a supported outcome here, and
# the fault report is the only thing that tells the operator which one did.
faulthandler.enable()

from srb.cli.capabilities import require_algorithm
from srb.utils.failures import FailureError

algo = sys.argv[1]
report = Path(sys.argv[2])
try:
    require_algorithm(algo)
except FailureError as exc:
    report.write_text(
        json.dumps(
            {
                "ok": False,
                "error_type": type(exc).__name__,
                "failure": exc.info.to_mapping(),
            }
        ),
        encoding="utf-8",
    )
    raise SystemExit(3)
report.write_text(json.dumps({"ok": True}), encoding="utf-8")
"""


def _run_isolated_algorithm_probe(algo: str) -> dict[str, Any]:
    """Validate *algo*'s provider in a clean interpreter and return its verdict."""

    with tempfile.TemporaryDirectory(prefix="srb-algorithm-probe-") as directory:
        report_path = Path(directory) / "verdict.json"
        command = [
            sys.executable,
            "-c",
            _ISOLATED_ALGORITHM_PROBE_SCRIPT,
            algo,
            str(report_path),
        ]
        try:
            result = subprocess.run(  # noqa: S603 - fixed argv, no shell
                command,
                capture_output=True,
                text=True,
                timeout=_ISOLATED_ALGORITHM_PROBE_TIMEOUT_S,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return {
                "ok": False,
                "process": {
                    "reason": "timeout",
                    "timeout_seconds": _ISOLATED_ALGORITHM_PROBE_TIMEOUT_S,
                },
            }

        if report_path.is_file():
            try:
                payload = json.loads(report_path.read_text(encoding="utf-8"))
            except ValueError:
                payload = None
            if isinstance(payload, dict):
                return payload

    # Search the full stream because provider crash dumps can exceed the tail window.
    fatal = next(
        (
            line.strip()
            for line in result.stderr.splitlines()
            if line.startswith("Fatal Python error")
        ),
        None,
    )
    return {
        "ok": False,
        "process": {
            "reason": "no_verdict",
            "returncode": int(result.returncode),
            "fatal_error": fatal,
            "stderr_tail": result.stderr[-2000:],
        },
    }


def _probe_process_summary(process: dict[str, Any]) -> str:
    """Summarize a probe child that died instead of answering."""

    if process.get("reason") == "timeout":
        return f"timed out after {process.get('timeout_seconds')} s"

    returncode = process.get("returncode")
    if isinstance(returncode, int) and returncode < 0:
        try:
            name = signal.Signals(-returncode).name
        except ValueError:
            name = f"signal {-returncode}"
        summary = f"crashed ({name})"
    else:
        summary = f"exited with code {returncode}"

    tail = [
        line.strip()
        for line in str(process.get("stderr_tail", "")).splitlines()
        if line.strip()
    ]
    # Prefer the faulthandler line, which identifies the crash among provider output.
    fatal = process.get("fatal_error") or next(
        (line for line in tail if line.startswith("Fatal Python error")), None
    )
    excerpt = str(fatal or (tail[-1] if tail else ""))
    if len(excerpt) > _PROBE_EXCERPT_CHARS:
        excerpt = f"{excerpt[:_PROBE_EXCERPT_CHARS]}..."
    if excerpt:
        summary = f"{summary}: {excerpt}"
    return summary


def _raise_from_isolated_probe(
    capability: "CapabilityProbe",
    algo: str,
    payload: dict[str, Any],
) -> None:
    """Re-raise an isolated verdict in this process, or return on success."""

    if payload.get("ok"):
        return

    process = payload.get("process")
    if isinstance(process, dict):
        capability._raise_unavailable(
            code="cli.capability.probe_failed",
            detail=f"the isolated probe for {algo!r} {_probe_process_summary(process)}",
            details={"algorithm": algo, "probe_process": process},
        )

    failure = payload.get("failure")
    if not isinstance(failure, dict):
        capability._raise_unavailable(
            code="cli.capability.probe_failed",
            detail=f"the isolated probe for {algo!r} returned an unreadable verdict",
            details={"algorithm": algo},
        )

    info = FailureInfo.from_mapping(failure)
    error_type = payload.get("error_type")
    error_class = getattr(sys.modules["srb.utils.failures"], str(error_type), None)
    if not (
        isinstance(error_class, type)
        and issubclass(error_class, FailureError)
        and error_class.category is info.category
    ):
        error_class = OptionalDependencyUnavailableError
        info = FailureInfo(
            category=OptionalDependencyUnavailableError.category,
            code=info.code,
            message=info.message,
            component=info.component,
            details=dict(info.details),
        )
    raise error_class(info)


def _require_algorithm_isolated(capability: "CapabilityProbe", algo: str) -> None:
    """Probe in a child process so RL imports cannot destabilize Kit startup."""

    # An absent distribution is already decisive and needs no subprocess.
    # TD-MPC2 may live in a configured checkout that is only importable inside
    # the isolated probe, where its integration adds the checkout to sys.path.
    if algo != "tdmpc2":
        capability.require_discoverable()
    _raise_from_isolated_probe(capability, algo, _run_isolated_algorithm_probe(algo))


def require_algorithm(algo: str | None, *, isolate: bool = False) -> None:
    """Probe the optional provider selected by an agent algorithm.

    Use ``isolate=True`` before AppLauncher to keep provider imports out of the Kit
    process.
    """
    if not algo:
        return
    from srb.core.algorithms import canonical_algorithm_label

    try:
        algo = canonical_algorithm_label(algo)
    except ValueError as exc:
        raise UnsupportedCapabilityError(
            code="cli.algorithm.unknown",
            message=str(exc),
            component="srb.cli.agent",
            details={"algorithm": algo},
        ) from exc

    probes: tuple[ModuleProbe, ...]
    if algo.startswith("skrl_"):
        probes = (
            ModuleProbe(
                "torch",
                required_symbols=("optim",),
                version_attribute="__version__",
            ),
            ModuleProbe(
                "torch._dynamo",
                required_symbols=("optimize",),
                callable_symbols=("optimize",),
            ),
            ModuleProbe(
                "skrl.utils.runner.torch",
                required_symbols=("Runner",),
                callable_symbols=("Runner",),
            ),
        )
        feature = f"algorithm provider '{algo}'"
        hint = "pip install 'srb[skrl]'"
    elif algo == "dreamer":
        probes = (
            ModuleProbe(
                "dreamerv3.agent",
                required_symbols=("Agent",),
                callable_symbols=("Agent",),
            ),
            ModuleProbe(
                "dreamerv3.main",
                required_symbols=("make_stream", "make_logger"),
                callable_symbols=("make_stream", "make_logger"),
            ),
        )
        feature = "Dreamer algorithm provider"
        hint = "pip install 'srb[dreamer]'"
    elif algo == "tdmpc2":
        # Resolve the source checkout through the same torch-free integration used at runtime.
        import srb.integrations.tdmpc2  # noqa: F401

        probes = (
            ModuleProbe(
                "tdmpc2", required_symbols=("TDMPC2",), callable_symbols=("TDMPC2",)
            ),
            ModuleProbe(
                "common.buffer",
                required_symbols=("Buffer",),
                callable_symbols=("Buffer",),
            ),
            ModuleProbe(
                "trainer.online_trainer",
                required_symbols=("OnlineTrainer",),
                callable_symbols=("OnlineTrainer",),
            ),
        )
        feature = "TD-MPC2 algorithm provider"
        hint = "the vendored or configured TD-MPC2 checkout"
    elif algo.startswith("rsl_rl_"):
        probes = (
            ModuleProbe(
                "rsl_rl.runners",
                required_symbols=("OnPolicyRunner",),
                callable_symbols=("OnPolicyRunner",),
            ),
        )
        feature = f"algorithm provider '{algo}'"
        hint = "pip install 'srb[rsl-rl]'"
    elif algo.startswith("robomimic_"):
        probes = (
            ModuleProbe(
                "robomimic.utils.file_utils",
                required_symbols=("policy_from_checkpoint",),
                callable_symbols=("policy_from_checkpoint",),
            ),
            ModuleProbe(
                "robomimic.utils.torch_utils",
                required_symbols=("get_torch_device",),
                callable_symbols=("get_torch_device",),
            ),
            ModuleProbe(
                "robomimic.algo",
                required_symbols=("algo_factory",),
                callable_symbols=("algo_factory",),
            ),
        )
        feature = f"algorithm provider '{algo}'"
        # robomimic is Git-pinned; the native installer applies the compatibility patch.
        hint = (
            "the pinned robomimic source: ./.docker/install/install_robomimic.bash "
            "<dest> in an SRB checkout (applies the compatibility patch), or the "
            "unpatched pin with 'pip install \"robomimic @ git+https://github.com/"
            "ARISE-Initiative/robomimic.git@e10526b9a40c78b41f1e37e60041dc0ec0a5f60f\"' "
            "(from a checkout: 'uv sync --all-extras --group source-robomimic')"
        )
    elif algo.startswith("sbx_"):
        probes = (
            ModuleProbe("sbx", required_symbols=("PPO",), callable_symbols=("PPO",)),
            ModuleProbe("rl_zoo3", required_symbols=("ALGOS",)),
        )
        feature = f"algorithm provider '{algo}'"
        hint = "pip install 'srb[sbx]'"
    elif algo.startswith("sb3_"):
        probes = (
            ModuleProbe("rl_zoo3", required_symbols=("ALGOS",)),
            ModuleProbe(
                "stable_baselines3.common.base_class",
                required_symbols=("BaseAlgorithm",),
                callable_symbols=("BaseAlgorithm",),
            ),
        )
        feature = f"algorithm provider '{algo}'"
        hint = "pip install 'srb[sb3]'"
    else:
        raise UnsupportedCapabilityError(
            code="cli.algorithm.unknown",
            message=f"No adapter-owned capability probe is registered for algorithm {algo!r}.",
            component="srb.cli.agent",
            details={"algorithm": algo},
        )

    capability = CapabilityProbe(
        feature=feature,
        component="srb.cli.agent.algorithm",
        dependencies=tuple(probe.module for probe in probes),
        install_hint=hint,
        module_probes=probes,
    )
    if isolate:
        _require_algorithm_isolated(capability, algo)
        return

    # Match runtime import order; otherwise the robomimic probe can segfault.
    from srb.utils.torch_runtime import warm_torch_native_stack

    warm_torch_native_stack()

    if algo != "dreamer":
        capability.require()
        return

    # dreamerv3.main adds its checkout parent to sys.path, which can shadow editable isaaclab.
    # Remove only paths introduced by the probe.
    path_before = list(sys.path)
    try:
        capability.require()
    finally:
        added = set(sys.path).difference(path_before)
        if added:
            sys.path[:] = [entry for entry in sys.path if entry not in added]


__all__ = [
    "CapabilityProbe",
    "ModuleProbe",
    "require_gui_toolchain",
    "require_algorithm",
    "require_dependency",
    "require_ephemeris",
    "require_event_backend",
    "require_event_reconstruction",
    "require_hydra",
    "require_isaac_sim",
    "require_isaac_sim_launcher",
    "require_ros",
    "resolve_gui_prebuilt",
    "resolve_isaac_sim_launcher",
]
