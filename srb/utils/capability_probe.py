"""Import-light dependency discovery and symbol-validation contracts.

The probe owns only pure capability mechanics.  Adapter-specific registration
and install hints remain with their callers, while this lower utility can be
used by a worker process without importing the CLI surface.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from importlib.util import find_spec
from typing import Any

from srb.utils.failures import OptionalDependencyUnavailableError


@dataclass(frozen=True, slots=True)
class ModuleProbe:
    """The symbols and version metadata an adapter needs from one module."""

    module: str
    required_symbols: tuple[str, ...] = ()
    callable_symbols: tuple[str, ...] = ()
    version_attribute: str | None = None
    require_version: bool = True


@dataclass(frozen=True, slots=True)
class CapabilityProbe:
    """A dependency check owned by one adapter.

    ``dependencies`` remains the public, import-free discovery list.  For a
    single dependency, ``required_symbols``/``callable_symbols`` are a small
    convenience for tests and simple adapters.  Multi-module adapters use
    ``module_probes`` so each symbol is checked against the correct module.
    """

    feature: str
    component: str
    dependencies: tuple[str, ...]
    install_hint: str
    module_probes: tuple[ModuleProbe, ...] = ()
    required_symbols: tuple[str, ...] = ()
    callable_symbols: tuple[str, ...] = ()
    version_attribute: str | None = None

    def _probes(self) -> tuple[ModuleProbe, ...]:
        if self.module_probes:
            return self.module_probes
        required_symbols = self.required_symbols if len(self.dependencies) == 1 else ()
        callable_symbols = self.callable_symbols if len(self.dependencies) == 1 else ()
        version_attribute = (
            self.version_attribute if len(self.dependencies) == 1 else None
        )
        return tuple(
            ModuleProbe(
                module=dependency,
                required_symbols=required_symbols,
                callable_symbols=callable_symbols,
                version_attribute=version_attribute,
            )
            for dependency in self.dependencies
        )

    def missing(self) -> tuple[str, ...]:
        """Return dependencies that are definitely undiscoverable.

        Discovery is side-effect free and probes the distribution *root* only.
        ``find_spec`` on a dotted name imports every parent package, and for
        the RL adapters that pulls the whole framework — and, through
        ``stable_baselines3`` -> ``torch.utils.tensorboard``, TensorFlow — into
        a process that has not booted Isaac Sim yet. TensorFlow's bundled
        protobuf then collides with Kit's during Kit startup and the process
        dies with SIGSEGV before the simulator ever appears. ``require`` still
        imports the exact submodule and validates its symbols, so nothing is
        lost where importing is safe.

        A ``ValueError`` from ``find_spec`` commonly means a synthetic module
        in ``sys.modules`` has no ``__spec__``.  That is not proof that the
        import will fail, so ``require`` proceeds to the real import in that
        case.  This keeps discovery useful without treating it as health.
        """

        missing: list[str] = []
        names = (*self.dependencies, *(probe.module for probe in self._probes()))
        roots = dict.fromkeys(name.split(".", 1)[0] for name in names)
        for dependency in roots:
            try:
                available = find_spec(dependency) is not None
            except (ImportError, ModuleNotFoundError):
                available = False
            except ValueError:
                continue
            except Exception:  # noqa: BLE001 - discovery is only a hint
                continue
            if not available:
                missing.append(dependency)
        return tuple(missing)

    def require_discoverable(self) -> None:
        """Fail fast on an absent dependency without importing anything."""

        missing = self.missing()
        if missing:
            self._raise_unavailable(
                code="cli.capability.missing_dependency",
                detail=f"missing dependency {', '.join(missing)}",
                details={"missing_dependencies": list(missing)},
            )

    def require(self) -> None:
        """Import and validate every module required by this adapter."""

        self.require_discoverable()

        for probe in self._probes():
            try:
                module = import_module(probe.module)
            except Exception as exc:  # noqa: BLE001 - optional import boundary
                self._raise_unavailable(
                    code="cli.capability.import_failed",
                    detail=f"could not import {probe.module}: {exc}",
                    details={
                        "module": probe.module,
                        "exception_type": type(exc).__name__,
                        "exception": str(exc),
                    },
                    cause=exc,
                )

            # Lazily exported symbols import on access; keep the error so the
            # report names the real cause rather than only "missing".
            symbol_errors: dict[str, BaseException] = {}
            missing_symbols = [
                symbol
                for symbol in probe.required_symbols
                if _resolve_symbol(module, symbol, symbol_errors) is _MISSING
            ]
            non_callable_symbols = [
                symbol
                for symbol in probe.callable_symbols
                if (
                    _resolve_symbol(module, symbol, symbol_errors) is _MISSING
                    or not callable(_resolve_symbol(module, symbol))
                )
            ]
            if missing_symbols or non_callable_symbols:
                details: dict[str, Any] = {"module": probe.module}
                if missing_symbols:
                    details["missing_symbols"] = missing_symbols
                if non_callable_symbols:
                    details["non_callable_symbols"] = non_callable_symbols
                labels = []
                if missing_symbols:
                    labels.append("missing " + ", ".join(missing_symbols))
                if non_callable_symbols:
                    labels.append("not callable " + ", ".join(non_callable_symbols))
                labels.extend(
                    f"{symbol} raised {type(exc).__name__}: {exc}"
                    for symbol, exc in symbol_errors.items()
                )
                self._raise_unavailable(
                    code="cli.capability.invalid_module",
                    detail=(
                        f"{probe.module} is discoverable but not healthy: "
                        + "; ".join(labels)
                    ),
                    details=details,
                )

            if probe.version_attribute is not None and probe.require_version:
                version = _resolve_symbol(module, probe.version_attribute)
                if version is _MISSING or not _healthy_version(version):
                    self._raise_unavailable(
                        code="cli.capability.invalid_version",
                        detail=(
                            f"{probe.module} has no healthy "
                            f"{probe.version_attribute} version"
                        ),
                        details={
                            "module": probe.module,
                            "version_attribute": probe.version_attribute,
                        },
                    )

    def _raise_unavailable(
        self,
        *,
        code: str,
        detail: str,
        details: dict[str, Any],
        cause: BaseException | None = None,
    ) -> None:
        missing_text = f" {detail}."
        error = OptionalDependencyUnavailableError(
            code=code,
            message=(
                f"The optional feature '{self.feature}' is unavailable:"
                f"{missing_text} Install {self.install_hint}."
            ),
            component=self.component,
            details={
                "feature": self.feature,
                "install_hint": self.install_hint,
                **details,
            },
        )
        if cause is None:
            raise error
        raise error from cause


_MISSING = object()


def _resolve_symbol(
    module: object,
    symbol: str,
    errors: dict[str, BaseException] | None = None,
) -> object:
    current = module
    for part in symbol.split("."):
        try:
            current = getattr(current, part)
        except Exception as exc:  # noqa: BLE001 - classify unhealthy provider symbols
            if errors is not None:
                errors.setdefault(symbol, exc)
            return _MISSING
    return current


def _healthy_version(value: object) -> bool:
    if value is None:
        return False
    text = str(value).strip()
    return bool(text) and text.lower() not in {"unknown", "none", "null"}


__all__ = ["CapabilityProbe", "ModuleProbe"]
