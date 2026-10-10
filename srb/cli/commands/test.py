"""``srb test`` -- Run the test suite, one pytest process per tier."""

from __future__ import annotations

import argparse
import os
from collections.abc import Sequence
from dataclasses import dataclass
from importlib.util import find_spec
from pathlib import Path

from srb.cli._base import BaseCommand
from srb.cli._parsing import Lang, SrbHelpFormatter
from srb.utils.path import SRB_DIR, SRB_IS_SOURCE_CHECKOUT
from srb.utils.process_supervisor import ProcessKind

_DEFAULT_PYTEST_PLUGINS = ("pytest_order",)
_TEST_PROCESS_TIMEOUT_S = 24.0 * 60.0 * 60.0


@dataclass(frozen=True, slots=True)
class TestTier:
    """One pytest invocation; tiers run in separate processes."""

    name: str
    description: str
    targets: tuple[str, ...]
    """Repository-relative pytest targets; globs such as ``tests/cli_*_test.py`` are expanded."""
    extra_args: tuple[str, ...] = ()


TEST_TIERS: tuple[TestTier, ...] = (
    TestTier(
        "unit",
        "host-runnable Python test",
        ("tests/import_test.py",),
    ),
    TestTier(
        "workflows",
        "CLI workflow tests (Isaac Sim)",
        ("tests/cli_*_test.py",),
    ),
)
_TIER_BY_NAME = {tier.name: tier for tier in TEST_TIERS}
ALL_TIERS = "all"


def resolve_tiers(names: Sequence[str]) -> tuple[TestTier, ...]:
    """Expand ``all`` to every tier; keep the canonical order."""
    selected: set[str] = set()
    for name in names:
        if name == ALL_TIERS:
            selected.update(tier.name for tier in TEST_TIERS)
        elif name in _TIER_BY_NAME:
            selected.add(name)
        else:
            raise ValueError(f"unknown test tier: {name!r}")
    return tuple(tier for tier in TEST_TIERS if tier.name in selected)


#: Python runs first because the Rust subprocess can exit the CLI on failure.
LANGUAGES: tuple[Lang, ...] = (Lang.PYTHON, Lang.RUST)


def resolve_languages(names: Sequence[str]) -> tuple[Lang, ...]:
    """Deduplicate languages while preserving the canonical order."""
    selected: set[Lang] = set()
    for name in names:
        # Direct callers bypass argparse; reject unknown names instead of
        # silently running no tests.
        lang = Lang.from_str(name)
        if lang is None:
            raise ValueError(f"unknown test language: {name!r}")
        selected.add(lang)
    return tuple(lang for lang in LANGUAGES if lang in selected)


_MODULE_PROBE_TIMEOUT_S = 120.0
_MODULE_PROBE_MARKER = "SRB_MODULE_FOUND:"
# Launchers such as Isaac Sim's python.sh may print banners; mark results.
_MODULE_PROBE_SCRIPT = (
    "import sys\n"
    "from importlib.util import find_spec\n"
    "for name in sys.argv[1:]:\n"
    "    if find_spec(name):\n"
    f"        print({_MODULE_PROBE_MARKER!r} + name)\n"
)


def _importable_modules(python: str, modules: Sequence[str]) -> frozenset[str]:
    """Return the ``modules`` that the interpreter running the tests can import."""
    import sys

    if python == sys.executable:
        return frozenset(name for name in modules if find_spec(name))

    from srb.utils.process_supervisor import ProcessStatus, run_owned

    result = run_owned(
        (python, "-c", _MODULE_PROBE_SCRIPT, *modules),
        kind=ProcessKind.FINITE,
        timeout=_MODULE_PROBE_TIMEOUT_S,
        capture_output=True,
    )
    if result.status is not ProcessStatus.EXITED or result.returncode:
        return frozenset()
    return frozenset(
        line.removeprefix(_MODULE_PROBE_MARKER)
        for line in (result.stdout or "").splitlines()
        if line.startswith(_MODULE_PROBE_MARKER)
    )


def expand_tier_targets(tier: TestTier, *, root: Path) -> tuple[str, ...]:
    """Return the tier's targets as paths under ``root`` (globs expanded, sorted)."""
    targets: list[str] = []
    for target in tier.targets:
        if any(char in target for char in "*?["):
            targets.extend(
                str(match) for match in sorted(root.glob(target)) if match.is_file()
            )
        else:
            path = root / target
            if path.is_file():
                targets.append(str(path))
    return tuple(targets)


def plan_python_test_commands(
    tiers: Sequence[TestTier],
    *,
    python: str,
    plugin_args: Sequence[str],
    forwarded_args: Sequence[str],
    root: Path = SRB_DIR,
) -> tuple[tuple[TestTier, tuple[str, ...]], ...]:
    """One ``pytest`` command per tier; forwarded arguments apply to every tier."""
    commands: list[tuple[TestTier, tuple[str, ...]]] = []
    for tier in tiers:
        targets = expand_tier_targets(tier, root=root)
        if not targets:
            continue
        commands.append(
            (
                tier,
                (
                    python,
                    "-m",
                    "pytest",
                    *plugin_args,
                    *targets,
                    *tier.extra_args,
                    *forwarded_args,
                ),
            )
        )
    return tuple(commands)


class TestCommand(BaseCommand):
    """Run Python and/or Rust test suites."""

    NAME = "test"
    HELP = "Run tests"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=(
                (f"Alias: {cls.NAME}" if is_alias else cls.HELP)
                + ("" if find_spec("pytest") else ' (MISSING: "pytest" Python package)')
            ),
            formatter_class=SrbHelpFormatter,
        )
        parser.add_argument(
            "-l",
            "--language",
            "--lang",
            help="Languages to test",
            nargs="+",
            type=str,
            choices=tuple(sorted(map(str, Lang))),
            default=[str(Lang.PYTHON)],
        )
        parser.add_argument(
            "-t",
            "--tier",
            help=(
                "Python test tiers to run, each in its own pytest process "
                "('all' = every tier): "
                + "; ".join(f"{tier.name}: {tier.description}" for tier in TEST_TIERS)
            ),
            nargs="+",
            type=str,
            choices=[ALL_TIERS, *(tier.name for tier in TEST_TIERS)],
            default=[ALL_TIERS],
        )
        return parser

    @classmethod
    def execute(
        cls,
        language: Sequence[str],
        tier: Sequence[str] = (ALL_TIERS,),
        forwarded_args: Sequence[str] = (),
        **_kwargs,
    ) -> None:
        from srb.cli._utils import run_subprocess
        from srb.utils import logging
        from srb.utils.isaacsim import get_kit_python
        from srb.utils.process_supervisor import (
            DEFAULT_TERM_GRACE_S,
            ProcessStatus,
            run_owned,
        )
        from srb.utils.redaction import redact_secret_arguments

        if not SRB_IS_SOURCE_CHECKOUT:
            raise RuntimeError("The test command requires an SRB source checkout")

        languages = resolve_languages(language)
        failures: list[str] = []
        for lang in languages:
            match lang:
                case Lang.PYTHON:
                    python = get_kit_python()
                    importable = _importable_modules(
                        python, ("pytest", *_DEFAULT_PYTEST_PLUGINS)
                    )
                    if "pytest" not in importable:
                        raise ImportError(
                            f'The "pytest" package is required by {python} to run tests'
                        )
                    selected_tiers = resolve_tiers(tier)
                    available_tiers = tuple(
                        test_tier
                        for test_tier in selected_tiers
                        if expand_tier_targets(test_tier, root=SRB_DIR)
                    )
                    if not available_tiers:
                        for unavailable in selected_tiers:
                            logging.info(
                                f"[srb test] {unavailable.name}  skipped (no tests available in this checkout)"
                            )
                        logging.critical("No Python tests matched the selected tiers")
                        failures.append("python:empty-selection")
                        continue
                    previous_pytest_plugin_autoload = os.environ.get(
                        "PYTEST_DISABLE_PLUGIN_AUTOLOAD"
                    )
                    if previous_pytest_plugin_autoload is None:
                        os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
                        plugin_args = tuple(
                            arg
                            for plugin in _DEFAULT_PYTEST_PLUGINS
                            if plugin in importable
                            for arg in ("-p", plugin)
                        )
                    else:
                        plugin_args = ()
                    commands = plan_python_test_commands(
                        available_tiers,
                        python=python,
                        plugin_args=plugin_args,
                        forwarded_args=forwarded_args,
                    )
                    outcomes: list[tuple[str, str]] = []
                    command_tiers = {test_tier.name for test_tier, _ in commands}
                    for unavailable in selected_tiers:
                        if unavailable.name not in command_tiers:
                            logging.info(
                                f"[srb test] {unavailable.name}  skipped (no tests available in this checkout)"
                            )
                    if not commands:
                        logging.critical("No Python tests matched the selected tiers")
                        failures.append("python:empty-selection")
                    try:
                        for test_tier, cmd in commands:
                            display_cmd = redact_secret_arguments(cmd)
                            logging.info(
                                f"Running {lang!s} test tier {test_tier.name!r} "
                                f"({test_tier.description}): " + " ".join(display_cmd)
                            )
                            result = run_owned(
                                cmd,
                                kind=ProcessKind.FINITE,
                                timeout=_TEST_PROCESS_TIMEOUT_S,
                                term_grace_s=DEFAULT_TERM_GRACE_S,
                                kill_grace_s=DEFAULT_TERM_GRACE_S,
                                reap_grace_s=DEFAULT_TERM_GRACE_S,
                            )
                            if result.status is ProcessStatus.TIMED_OUT:
                                outcome = "timed out"
                            elif result.status is not ProcessStatus.EXITED:
                                outcome = str(result.status.value)
                            elif result.returncode:
                                outcome = f"failed (exit {result.returncode})"
                            else:
                                outcome = "passed"
                            outcomes.append((test_tier.name, outcome))
                            if outcome != "passed":
                                failures.append(f"python:{test_tier.name}")
                            if (
                                result.status is ProcessStatus.INTERRUPTED
                                or not result.cleanup_complete
                            ):
                                break
                    finally:
                        if previous_pytest_plugin_autoload is None:
                            os.environ.pop("PYTEST_DISABLE_PLUGIN_AUTOLOAD", None)
                        else:
                            os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = (
                                previous_pytest_plugin_autoload
                            )
                    width = max((len(name) for name, _ in outcomes), default=0)
                    for name, outcome in outcomes:
                        logging.info(f"[srb test] {name:<{width}}  {outcome}")
                    continue
                case Lang.RUST:
                    cmd = (
                        "cargo",
                        "test",
                        "--manifest-path",
                        SRB_DIR.joinpath("Cargo.toml").as_posix(),
                        *forwarded_args,
                    )
            run_subprocess(
                cmd,
                description=f"Running {lang!s} tests with the following command",
                kind=ProcessKind.FINITE,
                timeout=_TEST_PROCESS_TIMEOUT_S,
            )
        if failures:
            logging.critical("Test tiers failed: " + ", ".join(failures))
            raise SystemExit(1)
