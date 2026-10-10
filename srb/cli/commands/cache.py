from __future__ import annotations

import argparse
from datetime import datetime, timezone
from typing import ClassVar

from srb.cli._base import BaseCommand


def _inspect_cache_status():
    from srb.utils.cache import inspect_offline_srb_cache

    return inspect_offline_srb_cache()


def _doctor_cache_status():
    from srb.utils.cache import doctor_offline_srb_cache

    return doctor_offline_srb_cache()


def _claim_clean_roots(target: str):
    from srb.utils.path import (
        claim_isaacsim_cache_roots,
        claim_srb_cache_root,
    )

    claimed = []
    if target in {"srb", "all"}:
        claimed.append(claim_srb_cache_root())
    if target in {"isaacsim", "all"}:
        claimed.extend(claim_isaacsim_cache_roots())
    return tuple(claimed)


def _update_cache_with_app() -> None:
    # Claim ownership before startup or cache publication; populated custom
    # roots cannot be adopted.
    _claim_clean_roots("srb")
    from srb.cli.capabilities import require_isaac_sim

    require_isaac_sim()
    from srb.cli.commands.agent_launch import agent_experience_path
    from srb.core.app import AppLauncher

    # Registry-backed discovery needs Isaac Sim. Guard startup too: Kit may
    # return 0 after a launcher failure, hiding the traceback from the CLI.
    try:
        launcher = AppLauncher(
            headless=True,
            experience=agent_experience_path(
                headless=True,
                enable_cameras=False,
                xr=False,
            ),
        )
        from srb.cli.offline_cache_build import update_offline_srb_cache

        update_offline_srb_cache()
    except BaseException:
        # Report before closing the app: close can exit 0 and swallow the
        # failure, falsely reporting success without publishing the cache.
        import sys
        import traceback

        from srb.cli.commands._hydra_exit import exit_nonzero_if_failed

        traceback.print_exc()
        print(
            "srb cache update FAILED: no cache generation was published",
            file=sys.stderr,
        )
        exit_nonzero_if_failed(True)
    launcher.app.close()


def _cache_generation_is_complete(status) -> bool:
    from srb.utils.path import SRB_CACHE_PATH

    if status.root != SRB_CACHE_PATH:
        return True
    from srb.utils.cache import cache_generation_complete

    return cache_generation_complete()


class CacheCommand(BaseCommand):
    """Manage generated SRB offline caches."""

    NAME: ClassVar[str] = "cache"
    HELP: ClassVar[str] = "Manage generated SRB offline caches"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
        **kwargs,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
        )
        sub = parser.add_subparsers(dest="cache_action")
        sub.add_parser(
            "update",
            help="Regenerate SRB offline caches (requires Isaac Sim)",
        )
        claim_parser = sub.add_parser(
            "claim-clean-root",
            help="Authorize safe cleanup of SRB or Isaac Sim cache roots",
        )
        claim_parser.add_argument(
            "--target",
            choices=("srb", "isaacsim", "all"),
            default="srb",
            help="Fixed cache roots to mark as owned for later cleanup",
        )
        status_parser = sub.add_parser(
            "status",
            help="Inspect generated cache files without launching Isaac Sim",
        )
        status_parser.add_argument(
            "--json",
            action="store_true",
            help="Print machine-readable cache status",
        )
        status_parser.add_argument(
            "--strict",
            action="store_true",
            help="Exit with code 1 when any generated cache is missing or invalid",
        )

        doctor_parser = sub.add_parser(
            "doctor",
            help="Validate generated cache payload shapes without launching Isaac Sim",
        )
        doctor_parser.add_argument(
            "--json",
            action="store_true",
            help="Print machine-readable cache validation report",
        )
        doctor_parser.add_argument(
            "--strict",
            action="store_true",
            help=(
                "Report only by default; exit with code 1 when cache entries or "
                "the generation are incomplete or invalid"
            ),
        )
        return parser

    @classmethod
    def execute(cls, **kwargs) -> None:
        action = kwargs.get("cache_action")
        if action == "update":
            _update_cache_with_app()
            print("SRB offline caches regenerated.")
        elif action == "claim-clean-root":
            roots = _claim_clean_roots(kwargs.get("target", "srb"))
            if not roots:
                print("No matching cache roots exist to claim.")
            for root in roots:
                print(f"Cache root is marker-owned for safe cleanup: {root}")
        elif action == "status":
            status = _inspect_cache_status()
            generation_complete = _cache_generation_is_complete(status)
            if kwargs.get("json", False):
                from srb.cli._output import print_json

                print_json(_cache_status_payload(status, generation_complete))
            else:
                _print_cache_status(status, generation_complete=generation_complete)
            if kwargs.get("strict", False) and (
                not status.ok or not generation_complete
            ):
                raise SystemExit(1)
        elif action == "doctor":
            status = _doctor_cache_status()
            generation_complete = _cache_generation_is_complete(status)
            if kwargs.get("json", False):
                from srb.cli._output import print_json

                print_json(_cache_status_payload(status, generation_complete))
            else:
                _print_cache_doctor(status, generation_complete=generation_complete)
            if kwargs.get("strict", False) and (
                not status.ok or not generation_complete
            ):
                raise SystemExit(1)
        else:
            print("Usage: srb cache {status,doctor,update,claim-clean-root}")
            raise SystemExit(2)


def _cache_status_payload(status, generation_complete: bool) -> dict[str, object]:
    payload = status.to_dict()
    payload["generation_complete"] = generation_complete
    payload["ok"] = bool(payload.get("ok")) and generation_complete
    return payload


def _print_cache_status(status, *, generation_complete: bool = True) -> None:
    print(f"SRB offline cache root: {status.root}")
    print(f"generation: {'complete' if generation_complete else 'incomplete'}")
    print(
        f"present: {status.present_count}/{status.total_count} "
        f"(valid: {status.ok_count}/{status.total_count})"
    )
    if status.missing_names:
        print(f"missing: {', '.join(status.missing_names)}")
    if status.invalid_names:
        print(f"invalid: {', '.join(status.invalid_names)}")

    for entry in status.entries:
        size = "-" if entry.size_bytes is None else str(entry.size_bytes)
        modified = _format_modified_time(entry.modified_time)
        print(
            f"{entry.state:<7} {entry.name:<22} {size:>10} {modified:>25} {entry.path}"
        )


def _print_cache_doctor(status, *, generation_complete: bool = True) -> None:
    print(f"SRB offline cache root: {status.root}")
    print(f"generation: {'complete' if generation_complete else 'incomplete'}")
    print(f"valid: {status.ok_count}/{status.total_count}")

    for entry in status.entries:
        schema = "-" if entry.schema_version is None else str(entry.schema_version)
        wrapped = "yes" if entry.wrapped else "no"
        print(
            f"{entry.state:<14} {entry.name:<22} wrapped={wrapped:<3} schema={schema:<3} {entry.path}"
        )


def _format_modified_time(timestamp: float | None) -> str:
    if timestamp is None:
        return "-"
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat(timespec="seconds")
