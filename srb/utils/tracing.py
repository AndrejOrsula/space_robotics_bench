from functools import cache
from importlib.util import find_spec
from os import environ, path


# Latch the one-time traceback hook installation.
@cache
def with_rich() -> bool:
    if find_spec("rich") is None or (
        environ.get("RICH_TRACEBACK") or environ.get("SRB_RICH_TRACEBACK", "true")
    ).lower() not in ("true", "1"):
        return False

    from rich import traceback

    suppress = tuple(
        filter(
            None,
            (
                _module_suppress_path("numpy"),
                _module_suppress_path("pydantic"),
                _module_suppress_path("torch"),
                _module_suppress_path("omegaconf"),
            ),
        )
    )

    traceback.install(
        width=120,
        show_locals=(
            environ.get("RICH_TRACEBACK_LOCALS")
            or environ.get("SRB_RICH_TRACEBACK_LOCALS", "false")
        ).lower()
        in ("true", "1"),
        suppress=suppress,
    )

    return True


def _module_suppress_path(module_name: str) -> str | None:
    spec = find_spec(module_name)
    if spec is None:
        return None
    if spec.submodule_search_locations:
        return next(iter(spec.submodule_search_locations), None)
    if spec.origin:
        return path.dirname(spec.origin)
    return None


@cache
def with_logfire() -> bool:
    if find_spec("logfire") is None or (
        environ.get("LOGFIRE_ENABLE") or environ.get("SRB_LOGFIRE_ENABLE", "true")
    ).lower() not in ("true", "1"):
        return False

    import logfire

    logfire.configure(
        send_to_logfire=environ.get("LOGFIRE_SEND_TO_LOGFIRE", "false").lower()
        in ("true", "1"),
        service_name="srb",
        console=False,
    )
    logfire.instrument_pydantic()

    return True
