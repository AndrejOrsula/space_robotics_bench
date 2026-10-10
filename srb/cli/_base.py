"""Abstract base class for SRB CLI subcommands."""

from __future__ import annotations

import argparse
from abc import ABC, abstractmethod
from typing import ClassVar


class BaseCommand(ABC):
    """Base class for CLI subcommands; subclasses define names and handlers."""

    NAME: ClassVar[str]
    HELP: ClassVar[str]
    ALIASES: ClassVar[tuple[str, ...]] = ()

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        for attr in ("NAME", "HELP"):
            if not hasattr(cls, attr):
                raise TypeError(
                    f"{cls.__name__} must define the class variable {attr!r}"
                )

    # -- Public API (called by __init__.py) --------------------------------

    @classmethod
    def register_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        **kwargs,
    ) -> list[argparse.ArgumentParser]:
        """Register parsers for the command and its aliases."""
        parsers: list[argparse.ArgumentParser] = []
        for i, name in enumerate((cls.NAME, *cls.ALIASES)):
            parser = cls._create_parser(
                subparsers,
                name=name,
                is_alias=(i > 0),
                **kwargs,
            )
            parsers.append(parser)
        return parsers

    @classmethod
    def all_names(cls) -> tuple[str, ...]:
        """Return the primary name followed by all aliases."""
        return (cls.NAME, *cls.ALIASES)

    # -- Abstract interface (subclasses must implement) ---------------------

    @classmethod
    @abstractmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
        **kwargs,
    ) -> argparse.ArgumentParser:
        """Build the parser for *name*, adjusting help when *is_alias*."""
        ...

    @classmethod
    @abstractmethod
    def execute(cls, **kwargs) -> None:
        """Run the command with parsed CLI arguments."""
        ...
