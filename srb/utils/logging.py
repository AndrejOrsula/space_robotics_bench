import logging
import sys
import warnings
from os import environ
from typing import Any

# Suppress Gymnasium's cosmetic Box precision-cast warning at startup.
warnings.filterwarnings(
    "ignore",
    message=r".*Box (low|high)'s precision lowered by casting.*",
)

# Drop the viewer deprecation notice; retain forwarding-failure warnings.
logging.getLogger("isaaclab.envs.common").addFilter(
    lambda record: not str(record.msg).startswith("env_cfg.viewer is deprecated. Set")
)

# TRACE is below DEBUG.
TRACE: int = 5
logging.addLevelName(TRACE, "TRACE")


def _configured_log_level() -> int:
    """Return the configured level, falling back safely for unknown values."""
    variable = "LOG_LEVEL" if environ.get("LOG_LEVEL") else "SRB_LOG_LEVEL"
    value = environ.get(variable, "INFO")
    normalized = value.strip().upper()
    level = logging.getLevelNamesMapping().get(normalized)
    if level is not None:
        return level
    if normalized.isdecimal():
        return int(normalized)

    warnings.warn(
        f"unsupported {variable}={value!r}; using INFO",
        RuntimeWarning,
        stacklevel=2,
    )
    return logging.INFO


logger = logging.getLogger("srb")

# Under pytest, allow messages to propagate so caplog can capture them even
# though SRB installs its own handlers.
logger.propagate = "pytest" in sys.modules

logger.setLevel(_configured_log_level())


# Skip rich/logfire imports during shell completion and help parsing.
def _is_help_only_invocation() -> bool:
    argv = sys.argv[1:]
    return not argv or argv[0] in ("-h", "--help")


_FAST_PATH = (
    "_ARGCOMPLETE" in environ
    or environ.get("SRB_LOGGING_FAST_PATH", "0") == "1"
    or _is_help_only_invocation()
)


def _install_plain_handlers() -> None:
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(TRACE)
    stdout_handler.addFilter(lambda record: record.levelno < logging.WARNING)
    # Route warnings and errors to stderr.
    stderr_handler = logging.StreamHandler(sys.stderr)
    stderr_handler.setLevel(logging.WARNING)  # Allow WARNING and above
    formatter = logging.Formatter(
        fmt="[{asctime}] {levelname:8s} {message}",
        datefmt="%H:%M:%S",
        style="{",
    )
    stdout_handler.setFormatter(formatter)
    stderr_handler.setFormatter(formatter)
    logger.addHandler(stdout_handler)
    logger.addHandler(stderr_handler)


if _FAST_PATH:
    _install_plain_handlers()
else:
    # Rich and Logfire setup imports heavier dependencies.
    from srb.utils.tracing import with_logfire, with_rich  # noqa: E402

    if with_rich():
        from rich.console import Console
        from rich.logging import RichHandler

        stdout_handler = RichHandler(
            console=Console(stderr=False),
            omit_repeated_times=False,
            rich_tracebacks=True,
            log_time_format="[%X]",
        )
        stdout_handler.setLevel(TRACE)
        stdout_handler.addFilter(lambda record: record.levelno < logging.WARNING)
        stderr_handler = RichHandler(
            console=Console(stderr=True),
            omit_repeated_times=False,
            rich_tracebacks=True,
            log_time_format="[%X]",
        )
        stderr_handler.setLevel(logging.WARNING)
        logger.addHandler(stdout_handler)
        logger.addHandler(stderr_handler)
    else:
        _install_plain_handlers()

    if with_logfire():
        from logfire import LogfireLoggingHandler

        logger.addHandler(LogfireLoggingHandler())


__all__ = [
    "logger",
    "trace",
    "debug",
    "info",
    "warning",
    "error",
    "critical",
    "exception",
]

debug = logger.debug
info = logger.info
warning = logger.warning
error = logger.error
critical = logger.critical
exception = logger.exception


def trace(msg: object, *args: object, **kwargs: Any) -> None:
    """Log at SRB's trace level through the typed public logging API."""
    if logger.isEnabledFor(TRACE):
        logger.log(TRACE, msg, *args, **kwargs)
