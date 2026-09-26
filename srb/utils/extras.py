"""Shared errors and import gates for optional SRB dependencies.

Use :class:`require_extra` around imports at an optional feature boundary:

.. code-block:: python

    with require_extra("event-metavision"):
        import metavision_core
        from metavision_core.event_io import EventsIterator

Import errors become :class:`MissingExtraError`; other exceptions pass
through. The v2e and e2vid providers are pinned Git requirements kept in
separate dependency groups because published metadata cannot contain direct
references. :func:`install_command` includes those pins in the install hint.
"""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType, TracebackType
from typing import Final

from srb.utils.failures import OptionalDependencyUnavailableError

V2E_SOURCE_REQUIREMENT: Final[str] = (
    "v2e @ git+https://github.com/SensorsINI/v2e.git"
    "@cac638444557a224ba6ee5c29260916f152c00bd"
)
E2VID_SOURCE_REQUIREMENT: Final[str] = (
    "e2vid @ git+https://github.com/uzh-rpg/e2calib.git"
    "@f35317a47809688ad140a67d50a23654c98612ea#subdirectory=python"
)
SOURCE_REQUIREMENTS: Final[Mapping[str, str]] = MappingProxyType(
    {
        "event-v2e": V2E_SOURCE_REQUIREMENT,
        "event-reconstruction": E2VID_SOURCE_REQUIREMENT,
    }
)


def install_command(extra: str) -> str:
    """Return the one ``pip`` command that makes ``extra``'s providers importable."""

    command = f"pip install 'srb[{extra}]'"
    requirement = SOURCE_REQUIREMENTS.get(extra)
    return command if requirement is None else f"{command} '{requirement}'"


class MissingExtraError(OptionalDependencyUnavailableError):
    """Raised at backend instantiation when an optional extra is not installed.

    Subclass of :class:`ImportError` so callers can ``except ImportError``
    generically; the message always includes the ``pip install srb[<extra>]``
    hint via :meth:`__str__`, followed by the pinned source requirement for
    extras listed in :data:`SOURCE_REQUIREMENTS`.
    """

    def __init__(self, extra: str, *, original: BaseException | None = None) -> None:
        self.extra = extra
        self.original = original
        self.source_requirement = SOURCE_REQUIREMENTS.get(extra)
        message = (
            f"Optional extra '{extra}' is not installed. "
            f"Install it with: {install_command(extra)}"
        )
        if original is not None:
            message += f" (underlying error: {original!r})"
        details: dict[str, object] = {"extra": extra}
        if self.source_requirement is not None:
            details["source_requirement"] = self.source_requirement
        super().__init__(
            code="optional_dependency.missing_extra",
            message=message,
            component=f"srb.extra.{extra}",
            details=details,
        )


class require_extra:
    """Convert import failures in a block to a hint for the named extra."""

    def __init__(self, extra: str) -> None:
        self.extra = extra

    def __enter__(self) -> "require_extra":
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> bool:
        if exc_type is None:
            return False
        if issubclass(exc_type, ImportError) and not isinstance(exc, MissingExtraError):
            raise MissingExtraError(self.extra, original=exc) from exc
        return False


__all__ = [
    "E2VID_SOURCE_REQUIREMENT",
    "SOURCE_REQUIREMENTS",
    "V2E_SOURCE_REQUIREMENT",
    "MissingExtraError",
    "install_command",
    "require_extra",
]
