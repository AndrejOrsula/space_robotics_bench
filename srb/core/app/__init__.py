"""Start Kit through :func:`isaaclab.app.launch_simulation`.

SRB starts Kit before environment configuration exists, so :class:`_KitRuntime`
resolves the experience first and follows Isaac Lab's launch path. Kit telemetry
is disabled in subprocess tests because some builds crash in its extension.
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import os
from collections.abc import Callable
from typing import Any, NamedTuple

# Imported eagerly so ``_require_regular_isaaclab_package`` below sees how
# ``isaaclab`` resolved; ``launch_simulation`` is read from it at launch.
import isaaclab.app  # noqa: F401

from srb.core.app.rendering_modes import apply_rendering_mode, rendering_mode_settings
from srb.utils.failures import KitBindingError, attempt_cleanup

__all__ = ["AppLauncher", "KitBindingError", "start_simulation_app"]


def _require_regular_isaaclab_package() -> None:
    """Remove identifiable namespace-package shadows or raise an actionable error."""
    import importlib
    import sys

    module = sys.modules.get("isaaclab")
    if module is None or getattr(module, "__file__", None) is not None:
        return

    shadowing = [
        entry
        for entry in sys.path
        if os.path.isdir(os.path.join(entry or ".", "isaaclab"))
        and not os.path.isfile(os.path.join(entry or ".", "isaaclab", "__init__.py"))
    ]
    for entry in shadowing:
        sys.path.remove(entry)
    del sys.modules["isaaclab"]
    repaired = importlib.import_module("isaaclab")
    if getattr(repaired, "__file__", None) is None:
        raise KitBindingError(
            "the 'isaaclab' package resolved as a bare namespace package "
            f"(shadowing sys.path entries: {shadowing or 'unidentified'}); "
            "the Isaac boot would fail inside AppLauncher with a masked "
            "exit code 0. Remove the stray 'isaaclab' directory from "
            "sys.path or reinstall isaaclab."
        )


_require_regular_isaaclab_package()


def _bind_pip_warp() -> None:
    """Bind pip ``warp-lang`` before Kit can import its bundled Warp version."""
    import importlib

    importlib.import_module("warp")


def _capture_pip_warp_binding() -> tuple[Any, str | None] | None:
    """Record the Warp module and source bound before Kit starts."""
    import sys

    module = sys.modules.get("warp")
    if module is None:
        return None
    return module, _module_origin(module)


def _check_pip_warp_bound(binding: tuple[Any, str | None] | None) -> None:
    """Raise if ``warp`` is not from the installed ``warp-lang`` distribution."""
    import importlib.metadata
    import sys

    module = sys.modules.get("warp")
    try:
        expected = importlib.metadata.version("warp-lang")
    except importlib.metadata.PackageNotFoundError:
        expected = None
    loaded = str(getattr(module, "__version__", "")) if module is not None else ""
    if expected is not None and module is not None and loaded != expected:
        raise RuntimeError(
            f"'warp' resolved to {loaded or 'an unknown version'} at "
            f"{getattr(module, '__file__', '?')}, but warp-lang {expected} is "
            "installed; a Kit extension (omni.warp.core) imported its bundled "
            "copy first. Import warp before starting Kit."
        )
    if binding is not None:
        expected_module, expected_origin = binding
        actual_origin = _module_origin(module) if module is not None else None
        if module is not expected_module or actual_origin != expected_origin:
            raise RuntimeError(
                "'warp' binding changed during Kit startup: expected "
                f"{expected_origin or 'an unresolved origin'}, found "
                f"{actual_origin or 'no module'}; import pip warp before "
                "starting Kit."
            )


def _module_origin(module: Any) -> str | None:
    """Return the resolved source path for an imported module, when available."""
    import os

    spec = getattr(module, "__spec__", None)
    origin = getattr(spec, "origin", None) or getattr(module, "__file__", None)
    if origin is None:
        return None
    return os.path.realpath(origin)


def _bind_pip_numba() -> dict[str, tuple[Any, str | None]]:
    """Bind optional Numba and its dependencies before Kit changes import paths."""
    import importlib
    import os
    import sys

    try:
        importlib.import_module("numba")
    except ModuleNotFoundError as exc:
        if exc.name == "numba":
            return {}
        raise

    bindings = {
        name: (module, _module_origin(module))
        for name in ("numba", "llvmlite", "coverage")
        if (module := sys.modules.get(name)) is not None
    }
    for name, (_, origin) in bindings.items():
        if origin is not None and "pip_prebundle" in origin.split(os.sep):
            raise RuntimeError(
                f"'{name}' is already bound to Kit's pip_prebundle at {origin}; "
                "start a fresh process so SRB can bind the environment's Numba "
                "dependencies before Kit."
            )
    return bindings


def _check_pip_numba_bound(bindings: dict[str, tuple[Any, str | None]]) -> None:
    """Raise if Kit changes an optional Numba dependency binding."""
    import sys

    for name, (expected_module, expected_origin) in bindings.items():
        module = sys.modules.get(name)
        actual_origin = _module_origin(module) if module is not None else None
        if module is not expected_module or actual_origin != expected_origin:
            raise RuntimeError(
                f"'{name}' binding changed during Kit startup: expected "
                f"{expected_origin or 'an unresolved origin'}, found "
                f"{actual_origin or 'no module'}; import Numba before starting Kit."
            )


def _bind_pip_cv2() -> dict[str, tuple[Any, str, str]]:
    """Bind optional OpenCV before Kit can select its codec-less prebundle."""
    import importlib
    import os
    import sys

    try:
        importlib.import_module("cv2")
    except ModuleNotFoundError as exc:
        if exc.name == "cv2":
            return {}
        raise

    module = sys.modules.get("cv2")
    if module is None:
        return {}
    origin = _module_origin(module)
    if origin is None:
        raise RuntimeError(
            "'cv2' has an unresolved origin; import environment OpenCV before "
            "starting Kit."
        )
    if "pip_prebundle" in origin.split(os.sep):
        raise RuntimeError(
            f"'cv2' is already bound to Kit's pip_prebundle at {origin}; "
            "start a fresh process so SRB can bind environment OpenCV before Kit."
        )
    return {"cv2": (module, origin, str(getattr(module, "__version__", "")))}


def _check_pip_cv2_bound(bindings: dict[str, tuple[Any, str, str]]) -> None:
    """Raise if Kit changes the OpenCV module, origin, or version."""
    import sys

    for name, (expected_module, expected_origin, expected_version) in bindings.items():
        module = sys.modules.get(name)
        actual_origin = _module_origin(module) if module is not None else None
        actual_version = str(getattr(module, "__version__", "")) if module else ""
        if (
            module is not expected_module
            or actual_origin != expected_origin
            or actual_version != expected_version
        ):
            raise RuntimeError(
                f"'{name}' binding changed during Kit startup: expected "
                f"{expected_origin} version {expected_version or 'unknown'}, found "
                f"{actual_origin or 'no module'} version "
                f"{actual_version or 'unknown'}; import OpenCV before starting Kit."
            )


def _bind_kit_environment() -> tuple[
    dict[str, tuple[Any, str | None]],
    tuple[Any, str | None] | None,
    dict[str, tuple[Any, str, str]],
]:
    """Bind Numba, Warp, and OpenCV; report conflicts as ``KitBindingError``."""
    try:
        numba_bindings = _bind_pip_numba()
        _bind_pip_warp()
        warp_binding = _capture_pip_warp_binding()
        cv2_bindings = _bind_pip_cv2()
    except KitBindingError:
        raise
    except RuntimeError as error:
        raise KitBindingError(str(error)) from error
    return numba_bindings, warp_binding, cv2_bindings


def _verify_kit_bindings(
    close: Callable[..., object],
    numba_bindings: dict[str, tuple[Any, str | None]],
    warp_binding: tuple[Any, str | None] | None,
    cv2_bindings: dict[str, tuple[Any, str, str]],
) -> None:
    """Close Kit with status 1 and raise if a binding changed during startup."""
    try:
        _check_pip_numba_bound(numba_bindings)
        _check_pip_warp_bound(warp_binding)
        _check_pip_cv2_bound(cv2_bindings)
    except RuntimeError as error:
        import sys

        print(
            f"Package binding verification failed after Kit startup: {error}",
            file=sys.stderr,
            flush=True,
        )
        failure = KitBindingError(str(error))
        _, cleanup_error = attempt_cleanup(lambda: close(exit_code=1))
        if cleanup_error is not None:
            failure.add_note(
                f"closing the Isaac application after the binding failure "
                f"also failed: {cleanup_error!r}"
            )
        raise failure from error


def start_simulation_app(*args: Any, **kwargs: Any) -> Any:
    """Start Kit for direct callers after binding environment packages."""
    from isaacsim import SimulationApp

    numba_bindings, warp_binding, cv2_bindings = _bind_kit_environment()
    app = SimulationApp(*args, **kwargs)
    _verify_kit_bindings(app.close, numba_bindings, warp_binding, cv2_bindings)
    return app


def _truthy(value: str | None) -> bool:
    if value is None:
        return False
    return value.strip().lower() not in {"", "0", "false", "no", "off"}


def _as_dict(namespace_or_dict: Any) -> dict:
    if namespace_or_dict is None:
        return {}
    if isinstance(namespace_or_dict, dict):
        return dict(namespace_or_dict)
    if isinstance(namespace_or_dict, argparse.Namespace):
        return dict(namespace_or_dict.__dict__)
    raise TypeError(
        "AppLauncher expected launcher_args as dict | argparse.Namespace | None, "
        f"got {type(namespace_or_dict)!r}"
    )


def _int_env(name: str) -> int:
    try:
        return int(os.environ.get(name, 0))
    except ValueError:
        # KitLauncher rejects the value with its own message.
        return 0


class _LaunchIntent(NamedTuple):
    headless: bool
    livestream: int
    xr: bool
    visualizers: tuple[str, ...]
    enable_cameras: bool


def _resolve_launch_intent(args: dict) -> _LaunchIntent:
    """Resolve livestream, XR and headless flags like ``KitLauncher``."""
    livestream = args.get("livestream", -1)
    if livestream is None or int(livestream) < 0:
        livestream = _int_env("LIVESTREAM")
    livestream = int(livestream)

    visualizers = args.get("visualizer")
    if isinstance(visualizers, str):
        visualizers = [name.strip() for name in visualizers.split(",")]
    if visualizers is not None and "none" in visualizers:
        visualizers = []
    if livestream > 0 and "kit" not in (visualizers or []):
        visualizers = [*(visualizers or []), "kit"]
    if visualizers is not None:
        kit_visualizer = "kit" in visualizers
    else:
        intent = args.get("visualizer_intent") or {}
        kit_visualizer = bool(intent.get("has_kit_visualizer"))

    xr = args.get("xr", False) is True or bool(_int_env("XR"))
    xr_auto_start = xr and "kit" not in (visualizers or [])
    headless = (
        bool(args.get("headless", False))
        or livestream > 0
        or bool(_int_env("HEADLESS"))
    )
    if not headless and (xr_auto_start or not kit_visualizer):
        headless = True
    return _LaunchIntent(
        headless=headless,
        livestream=livestream,
        xr=xr,
        visualizers=tuple(visualizers or ()),
        enable_cameras=bool(args.get("enable_cameras", False)),
    )


def _default_experience(args: dict) -> str:
    """Choose SRB's bundled experience, which includes ``omni.warp.core``."""
    from srb.utils.path import SRB_APPS_DIR

    intent = _resolve_launch_intent(args)
    rendering = intent.enable_cameras or intent.xr
    return SRB_APPS_DIR.joinpath(
        f"srb.{'headless.' if intent.headless and not intent.livestream else ''}"
        f"{'rendering.' if rendering else ''}"
        f"{'xr.' if intent.xr else ''}kit"
    ).as_posix()


def _renders(args: dict) -> bool:
    """Whether rendering or a continuous visualizer needs Fabric transforms."""
    intent = _resolve_launch_intent(args)
    return bool(
        not intent.headless
        or intent.livestream >= 1
        or intent.enable_cameras
        or intent.xr
        or intent.visualizers
    )


def _publish_fabric_transform_updates(enabled: bool) -> None:
    from isaaclab.app.settings_manager import get_settings_manager

    from srb.utils.fabric import FABRIC_TRANSFORM_UPDATE_SETTING

    get_settings_manager().set(FABRIC_TRANSFORM_UPDATE_SETTING, enabled)


def _install_abort_signal_handlers(launcher: Any) -> None:
    """Close Kit on termination signals, then re-raise for the expected status."""
    import contextlib
    import signal

    closing = False

    def on_abort_signal(signum: int, _frame: Any) -> None:
        nonlocal closing
        if not closing:
            closing = True
            with contextlib.suppress(Exception):
                launcher.app.close(exit_code=128 + signum)
        signal.signal(signum, signal.SIG_DFL)
        signal.raise_signal(signum)

    def on_interrupt(signum: int, frame: Any) -> None:
        if launcher.app.interrupted:
            on_abort_signal(signum, frame)
        launcher.app.interrupted = True
        raise KeyboardInterrupt

    for signum in (signal.SIGTERM, signal.SIGABRT):
        signal.signal(signum, on_abort_signal)
    signal.signal(signal.SIGINT, on_interrupt)


_logger = logging.getLogger(__name__)

_RUNTIME = contextlib.ExitStack()
"""Kit runtimes SRB started; held for the process so they outlive the launcher."""


class _KitApp:
    """Running Kit app wrapper; ``close()`` stops the SRB-started runtime."""

    def __init__(self, app: Any) -> None:
        self._app = app
        self.interrupted = False

    def close(self, *args: Any, exit_code: int = 0, **kwargs: Any) -> None:
        # Unwind the launch as ``sys.exit(exit_code)``: ``launch_simulation``
        # hands the status to ``KitLauncher.close``, whose fast shutdown exits
        # with it.
        _RUNTIME.__exit__(SystemExit, SystemExit(exit_code), None)

    def is_running(self) -> bool:
        return not self.interrupted and self._app.is_running()

    def is_exiting(self) -> bool:
        return not self.is_running()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._app, name)


class _KitRuntime:
    """Start Kit through ``launch_simulation``; Kit cannot use ``newton_rtx``."""

    app: Any

    def __init__(self, args: dict) -> None:
        from isaaclab.app import launch_simulation

        args = {**args, "require_kit": True}
        visualizers = args.get("visualizer") or []
        if isinstance(visualizers, str):
            visualizers = [name.strip() for name in visualizers.split(",")]
        if "newton_rtx" in visualizers:
            _logger.warning(
                "SRB starts Kit, which cannot run the 'newton_rtx' visualizer; "
                "skipping it."
            )
            args["visualizer"] = [v for v in visualizers if v != "newton_rtx"]
        _RUNTIME.enter_context(launch_simulation(None, args))

        import omni.kit.app

        self.app = _KitApp(omni.kit.app.get_app())


class AppLauncher(_KitRuntime):
    def __init__(
        self, launcher_args: argparse.Namespace | dict | None = None, **kwargs
    ):
        args = _as_dict(launcher_args)

        if kwargs:
            overlapping = set(args).intersection(kwargs)
            if overlapping:
                raise ValueError(
                    f"Input `launcher_args` and `kwargs` both provided common attributes: {overlapping}. "
                    "Supply each argument only once."
                )
            args.update(kwargs)

        disable_telemetry = (
            _truthy(os.environ.get("SRB_DISABLE_OMNI_TELEMETRY"))
            or os.environ.get("PYTEST_CURRENT_TEST") is not None
        ) and not _truthy(os.environ.get("SRB_ENABLE_OMNI_TELEMETRY"))

        if disable_telemetry:
            extra_kit_args = [
                "--/app/extensions/excluded/++=omni.kit.telemetry",
                "--/app/extensions/excluded/++=omni.graph.telemetry",
                "--/exts/omni.kit.telemetry/skipDeferredStartup=true",
                "--/telemetry/enableAnonymousData=false",
                "--/telemetry/disableInternalSessionCheck=true",
            ]

            kit_args = str(args.get("kit_args", "") or "").strip()
            for extra in extra_kit_args:
                if extra in kit_args:
                    continue
                kit_args = f"{kit_args} {extra}".strip()
            if kit_args:
                args["kit_args"] = kit_args

        if not args.get("experience"):
            args["experience"] = _default_experience(args)
        # Reject an unknown preset before Kit starts.
        rendering_mode_settings(args.get("rendering_mode"))

        numba_bindings, warp_binding, cv2_bindings = _bind_kit_environment()
        super().__init__(args)
        _install_abort_signal_handlers(self)
        _publish_fabric_transform_updates(_renders(args))
        apply_rendering_mode(args.get("rendering_mode"))
        # Every check closes Kit on failure, not only OpenCV's, so a Numba or
        # Warp mismatch never raises with the app still running.
        _verify_kit_bindings(
            lambda **kwargs: self.app.close(**kwargs),
            numba_bindings,
            warp_binding,
            cv2_bindings,
        )
