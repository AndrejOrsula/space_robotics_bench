import ast
import difflib
import os
import shutil
import stat
import sys
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import gymnasium
import gymnasium.envs.registration

from srb.interfaces.sim_to_real.core.hardware import HardwareInterfaceRegistry
from srb.interfaces.sim_to_real.deploy_profiles.generation import (
    canonical_generation_policy_for_env_spec_id,
    deploy_profile_for_env_spec_id,
)
from srb.utils.process_supervisor import (
    ProcessKind,
    ProcessPlatformError,
    ProcessSpawnError,
    ProcessStatus,
    run_owned,
)

from .generator_extract import EnvInfoExtractor
from .generator_kwargs import RegistrationKwargsRewriter
from .generator_render import RealEnvModuleRenderer, SpaceFormatter

if TYPE_CHECKING:
    from srb._typing import AnyEnv


def _existing_hardware(module: Path) -> tuple[str, ...]:
    """Return the `HARDWARE` tuple of an existing generated module, if any."""
    try:
        tree = ast.parse(module.read_text(encoding="utf-8"))
    except (OSError, SyntaxError, ValueError):
        return ()
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == "HARDWARE"
            and node.value is not None
        ):
            try:
                value = ast.literal_eval(node.value)
            except ValueError:
                return ()
            if isinstance(value, tuple) and all(isinstance(v, str) for v in value):
                return value
    return ()


class GenerationCheckError(RuntimeError):
    """``srb real_agent gen --check`` could not certify the checked-in module."""


class GeneratedModuleDriftError(GenerationCheckError):
    """The checked-in generated module differs from a fresh deterministic render."""


class FormatterUnavailableError(GenerationCheckError):
    """No formatter is available to format candidates for ``--check``.

    Comparing unformatted candidates with Ruff-formatted modules would report
    false drift. This error means no formatter was found on ``PATH``;
    :class:`FormatterFailedError` covers a formatter that ran and failed.
    """


class FormatterFailedError(GenerationCheckError):
    """A formatter on ``PATH`` failed; captured stderr carries its diagnostic."""


class GeneratedModuleSyntaxError(GenerationCheckError):
    """The formatted candidate failed Python parsing before replacing output."""


@dataclass(frozen=True)
class FormatOutcome:
    """Formatting result; ``attempted`` distinguishes missing from failed tools."""

    ok: bool
    attempted: bool
    formatter: str = ""
    stderr: str = ""


class RealEnvGenerator:
    def __init__(self):
        self._extractor = EnvInfoExtractor()
        self._kwargs_rewriter = RegistrationKwargsRewriter()
        self._spaces = SpaceFormatter()
        self._renderer = RealEnvModuleRenderer(space_formatter=self._spaces)

    def generate_offline(
        self,
        env: "AnyEnv",
        env_spec: gymnasium.envs.registration.EnvSpec,
        output: Path | str,
        hardware: Sequence[str] = (),
        *,
        check: bool = False,
    ) -> None:
        """Render a RealEnv module, or check its deterministic freshness.

        ``check=True`` formats a temporary candidate, compares bytes, and
        reports drift without writing. Normal generation atomically replaces
        output only after formatting and parsing succeed.
        """
        output = Path(output).resolve()

        env_info = self._extractor.extract(
            env,
            canonical_policy=canonical_generation_policy_for_env_spec_id(env_spec.id),
        )
        registration_kwargs, used_srb_paths = self._kwargs_rewriter.rewrite(
            env_spec.kwargs
        )

        # Without `--hardware`, keep the module's current selection.
        if not hardware:
            hardware = _existing_hardware(output)
        # Ensure all hardware interfaces are registered and unique
        for hw in dict.fromkeys(hardware):
            if HardwareInterfaceRegistry.get_by_name(hw) is None:
                raise ValueError(f"Hardware interface '{hw}' is not registered")

        content = self._renderer.render_offline_module(
            env_info=env_info,
            env_spec_id=env_spec.id,
            registration_kwargs=registration_kwargs,
            used_srb_paths=used_srb_paths,
            hardware=hardware,
            deploy_profile=deploy_profile_for_env_spec_id(env_spec.id),
        )

        if check:
            self._check_against_existing(content=content, output=output)
            return

        output.parent.mkdir(parents=True, exist_ok=True)
        self._write_atomically(content=content, output=output)

    def _write_atomically(self, *, content: str, output: Path) -> None:
        """Format and parse a sibling candidate before atomically replacing output."""
        candidate: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                dir=output.parent,
                prefix=f".{output.stem}.gen_write_",
                suffix=".py",
                mode="w",
                delete=False,
            ) as handle:
                candidate = Path(handle.name)
                handle.write(content)
            outcome = self._format_python_file(candidate)
            if not outcome.ok:
                if outcome.attempted:
                    raise FormatterFailedError(
                        f"cannot write {output}: formatting the fresh render "
                        f"with {outcome.formatter} failed, so the previous "
                        "module (if any) was left untouched. This usually "
                        "means the generator rendered invalid Python, not that "
                        f"a formatter is missing — {outcome.formatter} stderr:\n"
                        f"{outcome.stderr}"
                    )
                raise FormatterUnavailableError(
                    f"cannot write {output}: none of {self._formatter_names()} "
                    "is available on PATH, so the previous module (if any) was "
                    "left untouched. Every checked-in module is formatted by "
                    "the repo's pinned ruff pre-commit hook and `--check` "
                    "byte-compares formatted output. Install the pinned "
                    "formatter (e.g. `uvx ruff@0.15.4 --version` to warm the "
                    "cache, then put it on PATH) and re-run."
                )
            formatted = candidate.read_text()
            try:
                ast.parse(formatted, filename=output.as_posix())
            except SyntaxError as exc:
                raise GeneratedModuleSyntaxError(
                    f"cannot write {output}: the formatted render does not "
                    f"parse as Python ({exc}), so the previous module (if any) "
                    "was left untouched"
                ) from exc
            # NamedTemporaryFile creates 0600; a product replacing a
            # checked-in module keeps that module's mode, and a brand-new
            # product gets the ordinary umask-derived mode.
            if output.exists():
                mode = stat.S_IMODE(output.stat().st_mode)
            else:
                umask = os.umask(0)
                os.umask(umask)
                mode = 0o666 & ~umask
            os.chmod(candidate, mode)
            with candidate.open("rb+") as handle:
                os.fsync(handle.fileno())
            os.replace(candidate, output)
            candidate = None
        finally:
            if candidate is not None:
                candidate.unlink(missing_ok=True)

    def _check_against_existing(self, *, content: str, output: Path) -> None:
        """Compare a formatted render with output without changing output.

        The candidate sits beside output so the formatter finds the same
        configuration. A missing output fails before any filesystem writes.
        """
        if not output.is_file():
            raise GeneratedModuleDriftError(
                f"checked-in generated module {output} does not exist; run "
                "`srb real_agent gen` without --check to create it"
            )
        # The temp file is created *inside* the try/finally (not just the
        # cleanup): if `handle.write` (or the closing flush) raises, or the
        # process is killed between creation and formatting, `candidate`
        # must not leak a `.<stem>.gen_check_*.py` file into the package
        # directory.
        candidate: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                dir=output.parent,
                prefix=f".{output.stem}.gen_check_",
                suffix=".py",
                mode="w",
                delete=False,
            ) as handle:
                candidate = Path(handle.name)
                handle.write(content)
            outcome = self._format_python_file(candidate)
            fresh_bytes = candidate.read_bytes()
        finally:
            if candidate is not None:
                candidate.unlink(missing_ok=True)
        if not outcome.ok:
            if outcome.attempted:
                raise FormatterFailedError(
                    f"cannot verify {output}: formatting the fresh render with "
                    f"{outcome.formatter} failed. This usually means the "
                    "generator rendered invalid Python, not that a formatter is "
                    f"missing — {outcome.formatter} stderr:\n{outcome.stderr}"
                )
            raise FormatterUnavailableError(
                f"cannot verify {output}: --check byte-compares a *formatted* "
                f"fresh render, and none of {self._formatter_names()} is "
                "available on PATH. Every checked-in module is formatted by "
                "the repo's pinned ruff pre-commit hook, so comparing against "
                "an unformatted candidate would manufacture drift. Install "
                "the pinned formatter (e.g. `uvx ruff@0.15.4 --version` to "
                "warm the cache, then put it on PATH) and re-run."
            )
        existing_bytes = output.read_bytes()
        if existing_bytes == fresh_bytes:
            print(f"[gen --check] up to date: {output}")
            return
        existing = existing_bytes.decode("utf-8")
        fresh = fresh_bytes.decode("utf-8")
        diff = "".join(
            difflib.unified_diff(
                existing.splitlines(keepends=True),
                fresh.splitlines(keepends=True),
                fromfile=f"{output} (checked-in)",
                tofile=f"{output} (fresh render)",
            )
        )
        # Stderr: `gen --env ALL` reports only the child's stderr.
        print(diff, file=sys.stderr)
        raise GeneratedModuleDriftError(
            f"generated module drift detected for {output}; regenerate with "
            "`srb real_agent gen` after reviewing the diff above"
        )

    __FORMAT_CMDS: Sequence[Sequence[str]] = (
        ("ruff", "format"),
        ("black",),
    )
    __FORMAT_TIMEOUT_S = 30.0

    @classmethod
    def _formatter_names(cls) -> str:
        return " / ".join(cmd[0] for cmd in cls.__FORMAT_CMDS)

    def _format_python_file(self, file_path: Path) -> FormatOutcome:
        """Format ``file_path`` in place with the first formatter on PATH.

        Falls through to the next configured formatter on failure (matching
        the previous behaviour), but records the last failure's ``stderr``
        so a caller can tell "no formatter is installed" apart from "a
        formatter ran and rejected the candidate" — the latter usually means
        the *generator* produced invalid Python.
        """
        last_failure: FormatOutcome | None = None
        for cmd in self.__FORMAT_CMDS:
            if not shutil.which(cmd[0]):
                continue
            try:
                result = run_owned(
                    (*cmd, file_path.as_posix()),
                    kind=ProcessKind.FINITE,
                    timeout=self.__FORMAT_TIMEOUT_S,
                    capture_output=True,
                )
            except (ProcessPlatformError, ProcessSpawnError) as exc:
                last_failure = FormatOutcome(
                    ok=False, attempted=True, formatter=cmd[0], stderr=str(exc)
                )
                continue
            if (
                result.status is ProcessStatus.EXITED
                and result.returncode == 0
                and result.cleanup_complete
            ):
                return FormatOutcome(ok=True, attempted=True, formatter=cmd[0])
            last_failure = FormatOutcome(
                ok=False,
                attempted=True,
                formatter=cmd[0],
                stderr=result.error or result.stderr or "",
            )
        return last_failure or FormatOutcome(ok=False, attempted=False)
