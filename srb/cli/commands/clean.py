"""``srb clean`` -- Remove cached files for SRB, Isaac Sim, and SimForge."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import Sequence

from srb.cli._base import BaseCommand
from srb.cli._parsing import CacheTarget, SrbHelpFormatter
from srb.utils.owned_directory import (
    OWNERSHIP_MARKER_NAME,
    OwnedDirectoryError,
    clear_owned_directory,
    validate_owned_directory,
)
from srb.utils.path import (
    SRB_CACHE_OWNER,
    SRB_CACHE_PATH,
    SRB_DIR,
    SRB_ISAACSIM_CACHE_OWNER,
    isaacsim_cache_roots,
)


def _claim_target_for(directory: Path) -> str:
    """Name the ``srb cache claim-clean-root`` target that owns *directory*."""
    return "srb" if directory == SRB_CACHE_PATH else "isaacsim"


def _selected_targets(cache_target: str | Sequence[str]) -> list[CacheTarget]:
    """Expand the parsed ``cache_target`` into the caches to remove.

    Accepts a bare ``str`` as well as a sequence, for the same reason
    ``listing._print_offline`` does: this is a ``nargs="*"`` positional carrying
    ``choices``, and CPython's ``argparse._get_values`` validates the *default*
    against those choices when no value is supplied. A list default therefore
    makes the bare command unusable (``invalid choice: "['all']"``), so the
    default is the plain string ``"all"`` and both shapes reach here.

    ``CacheTarget.from_str`` answers ``None`` rather than raising, and a
    ``None`` matches no ``case`` in the removal loop below, so an unknown
    target would log "Nothing to clean" and exit 0. ``argparse`` rejects unknown
    names at the CLI, but ``execute`` is also called directly.
    """
    names = [cache_target] if isinstance(cache_target, str) else list(cache_target)
    targets: list[CacheTarget] = []
    for name in names:
        target = CacheTarget.from_str(str(name))
        if target is None:
            raise ValueError(
                f"unknown cache target: {name!r}; expected one of "
                + ", ".join(sorted(map(str, CacheTarget)))
            )
        targets.append(target)
    if CacheTarget.ALL in targets:
        return [CacheTarget.SRB, CacheTarget.ISAACSIM, CacheTarget.SIMFORGE]
    return targets


def _clear_directory(directory: Path, *, owner: str) -> None:
    """Clear one marker-owned directory without removing its root or marker.

    Confirmation and ``--yes`` do not reach this function's ownership or path
    checks. Removal stays anchored to the validated directory descriptor.
    """
    clear_owned_directory(
        directory,
        owner=owner,
        protected_roots=(SRB_DIR,),
    )


_SIMFORGE_CLEAN_TIMEOUT_S = 300.0


def _unclaimed_isaac_cache_roots(home: Path) -> tuple[Path, ...]:
    """Return present known roots that lack an ownership marker."""

    return tuple(
        root
        for root in isaacsim_cache_roots(home=home)
        if root.is_dir()
        and not root.is_symlink()
        and not (root / OWNERSHIP_MARKER_NAME).is_file()
    )


class CleanCommand(BaseCommand):
    """Remove cached files from SRB, Isaac Sim, and/or SimForge."""

    NAME = "clean"
    HELP = "Remove cached files from SRB, Isaac Sim, and/or SimForge"

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
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )
        parser.add_argument(
            "cache_target",
            help="Which caches to clean",
            nargs="*",
            type=str,
            choices=sorted(map(str, CacheTarget)),
            # Keep the default a string: argparse checks a `nargs="*"` default
            # against choices and rejects `["all"]`. `srb list` does likewise.
            default=str(CacheTarget.ALL),
        )
        parser.add_argument(
            "-y",
            "--yes",
            help="Skip confirmation prompt and remove immediately",
            action="store_true",
            default=False,
        )
        return parser

    @classmethod
    def execute(
        cls,
        cache_target: str | Sequence[str],
        yes: bool = False,
        **_kwargs,
    ) -> None:
        from srb.utils import logging

        targets = _selected_targets(cache_target)

        home = Path.home()

        # -- Collect phase: find what would be removed --
        dirs_to_clean: list[tuple[Path, str]] = []
        cmds_to_run: list[tuple[str, ...]] = []

        for target in targets:
            match target:
                case CacheTarget.SRB:
                    if SRB_CACHE_PATH.is_dir():
                        dirs_to_clean.append((SRB_CACHE_PATH, SRB_CACHE_OWNER))

                case CacheTarget.ISAACSIM:
                    for cache_root in _unclaimed_isaac_cache_roots(home):
                        logging.warning(
                            f"Isaac Sim cache root is present but unclaimed: {cache_root}; "
                            "run `srb cache claim-clean-root --target isaacsim` "
                            "before cleaning it."
                        )
                    for cache_dir in isaacsim_cache_roots(home=home):
                        if cache_dir.is_dir():
                            dirs_to_clean.append((cache_dir, SRB_ISAACSIM_CACHE_OWNER))

                case CacheTarget.SIMFORGE:
                    if shutil.which("simforge") is not None:
                        simforge_cmd = ["simforge", "clean"]
                        if yes:
                            simforge_cmd.append("-y")
                        cmds_to_run.append(tuple(simforge_cmd))
                    else:
                        logging.warning(
                            "SimForge CLI is not installed; cannot clean SimForge cache"
                        )
                        raise SystemExit(1)

        if not dirs_to_clean and not cmds_to_run:
            logging.info("Nothing to clean")
            return

        # Validate all targets first so a bad marker cannot follow earlier
        # removals.
        for d, owner in dirs_to_clean:
            try:
                validate_owned_directory(d, owner=owner, protected_roots=(SRB_DIR,))
            except OwnedDirectoryError as exc:
                raise OwnedDirectoryError(
                    f"Refusing to clean {d}: {exc}. Only marker-owned cache "
                    "directories are removable; run `srb cache claim-clean-root "
                    f"--target {_claim_target_for(d)}` to add the ownership "
                    "marker, or remove the directory manually.",
                    component="srb.cli.clean",
                    details={"path": str(d), "owner": owner},
                ) from exc

        # -- Confirm and remove directories --
        if dirs_to_clean:
            logging.info("The following will be removed:")
            for d, _owner in dirs_to_clean:
                logging.info(f"  {d}")
            if not yes:
                try:
                    answer = input("Remove? [y/N]: ").strip().lower()
                except EOFError:
                    # Non-interactive input defaults to "N"; use --yes to remove.
                    answer = ""
                if answer != "y":
                    logging.info("Aborted.")
                    return
            for d, owner in dirs_to_clean:
                logging.info(f"Removing: {d}")
                try:
                    _clear_directory(d, owner=owner)
                except OwnedDirectoryError as exc:
                    raise OwnedDirectoryError(
                        f"Stopped cleaning {d}: {exc}",
                        component="srb.cli.clean",
                        details={"path": str(d), "owner": owner},
                    ) from exc

        # -- Run simforge clean (handles its own confirmation) --
        simforge_failed = False
        for cmd in cmds_to_run:
            from srb.utils.process_supervisor import (
                ProcessKind,
                ProcessStatus,
                run_owned,
            )

            logging.info(f"Running: {' '.join(cmd)}")
            result = run_owned(
                cmd,
                kind=ProcessKind.FINITE,
                timeout=_SIMFORGE_CLEAN_TIMEOUT_S,
            )
            if not result.cleanup_complete:
                simforge_failed = True
                logging.error(
                    "SimForge cache clean did not complete process-group cleanup: "
                    f"{result.error}"
                )
            elif result.status is ProcessStatus.TIMED_OUT:
                simforge_failed = True
                logging.error(
                    "SimForge cache clean timed out after "
                    f"{_SIMFORGE_CLEAN_TIMEOUT_S:.0f}s"
                )
            elif result.returncode:
                simforge_failed = True
                logging.error(
                    f"SimForge cache clean failed with exit code {result.returncode}"
                )
            elif result.status is not ProcessStatus.EXITED:
                simforge_failed = True
                logging.error(
                    f"SimForge cache clean ended in unexpected state: "
                    f"{result.status.value}"
                )

        if simforge_failed:
            raise SystemExit(1)
        logging.info("Cache cleaning complete")
