"""Optional run logging with an owned provider lifecycle."""

from collections.abc import Callable, Mapping
from contextlib import AbstractContextManager
from importlib import import_module
from types import TracebackType
from typing import Any, Protocol, cast

__all__ = [
    "DisabledRunLogger",
    "RunLogger",
    "RunLoggingCloseError",
    "RunLoggingError",
    "RunLoggingInitializationError",
    "RunLoggingUnavailableError",
    "create_run_logger",
    "run_logging",
]


class RunLoggingError(RuntimeError):
    """Base error for optional run-logging lifecycle failures."""


class RunLoggingUnavailableError(RunLoggingError):
    """The explicitly requested run-logging provider is unavailable."""


class RunLoggingInitializationError(RunLoggingError):
    """The provider could not initialize or configure a run."""


class RunLoggingCloseError(RunLoggingError):
    """The provider run could not be finalized."""


class RunLogger(Protocol):
    """Minimal handle owned by the run-logging context."""

    def close(self) -> None:
        """Finalize the run, if one is enabled."""


class _ProviderConfig(Protocol):
    def update(self, values: Mapping[str, Any]) -> object: ...


class _ProviderRun(Protocol):
    config: _ProviderConfig

    def finish(self) -> object: ...


class _WandbProvider(Protocol):
    def init(
        self,
        *,
        name: str,
        sync_tensorboard: bool,
        monitor_gym: bool,
    ) -> _ProviderRun: ...


class DisabledRunLogger:
    """No-op logger used when optional logging is disabled."""

    def close(self) -> None:
        return None


class _WandbRunLogger:
    def __init__(self, provider_run: _ProviderRun) -> None:
        self._provider_run = provider_run
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._provider_run.finish()
        except Exception as exc:
            raise RunLoggingCloseError("The W&B run could not be finalized.") from exc


class _RunLoggingContext(AbstractContextManager[RunLogger]):
    def __init__(self, logger: RunLogger) -> None:
        self._logger = logger

    def __enter__(self) -> RunLogger:
        return self._logger

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> bool:
        del exc_type, traceback
        if exc_value is not None:
            try:
                self._logger.close()
            except RunLoggingCloseError as close_error:
                exc_value.add_note(f"Run-logging close also failed: {close_error}")
            return False

        self._logger.close()
        return False


_WANDB_UNAVAILABLE_MESSAGE = (
    "SB3 W&B run logging was requested, but the optional `wandb` dependency "
    "is unavailable. Install it with `pip install wandb` or disable the SB3 "
    "`wandb` option."
)


def _load_wandb() -> _WandbProvider:
    """Load W&B only when explicitly requested by the factory."""
    return cast(_WandbProvider, import_module("wandb"))


def create_run_logger(
    *,
    enabled: bool,
    name: str,
    run_metadata: Mapping[str, Any] | None = None,
    provider_loader: Callable[[], _WandbProvider] | None = None,
) -> RunLogger:
    """Create a disabled handle or initialize the current W&B provider."""
    if not enabled:
        return DisabledRunLogger()

    loader = provider_loader or _load_wandb
    try:
        provider = loader()
    except RunLoggingError:
        raise
    except ModuleNotFoundError as exc:
        raise RunLoggingUnavailableError(_WANDB_UNAVAILABLE_MESSAGE) from exc
    except Exception as exc:
        raise RunLoggingInitializationError(
            "The W&B run-logging provider could not be loaded."
        ) from exc

    provider_run: _ProviderRun | None = None
    try:
        provider_run = provider.init(
            name=name,
            sync_tensorboard=True,
            monitor_gym=True,
        )
        if run_metadata:
            provider_run.config.update(run_metadata)
    except Exception as exc:
        initialization_error = RunLoggingInitializationError(
            "The W&B run could not be initialized or configured."
        )
        if provider_run is not None:
            try:
                provider_run.finish()
            except Exception as close_error:
                initialization_error.add_note(
                    f"Run-logging close also failed: {close_error}"
                )
        raise initialization_error from exc

    return _WandbRunLogger(provider_run)


def run_logging(
    *,
    enabled: bool,
    name: str,
    run_metadata: Mapping[str, Any] | None = None,
    provider_loader: Callable[[], _WandbProvider] | None = None,
) -> AbstractContextManager[RunLogger]:
    """Own one optional provider run for the duration of a workflow."""
    return _RunLoggingContext(
        create_run_logger(
            enabled=enabled,
            name=name,
            run_metadata=run_metadata,
            provider_loader=provider_loader,
        )
    )
