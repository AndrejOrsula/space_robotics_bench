"""CLI subcommands for the terrain pipeline.

This module provides `validate-manifest`; the `download`, `bake`, `clean` and
`cache-stats` commands live under `srb asset`.

Primary entry point: ``python -m srb.terrain.cli validate-manifest``
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from srb.terrain.errors import ManifestInvalidError, TerrainError
from srb.terrain.manifest_loader import clear_manifest_cache, load_manifest
from srb.terrain.sources.acquisition import (
    CachedSourceStatus,
    check_cached_sources,
    check_source_urls,
)


def validate_manifest_cmd(
    manifest_path: Optional[Path],
    *,
    check_urls: bool = False,
    check_shas: bool = False,
) -> int:
    """Validate the manifest schema. Returns exit code (0 = ok)."""
    # Always re-read the manifest from disk for validate (don't trust the
    # process-wide cache).
    clear_manifest_cache()
    try:
        manifest = load_manifest(manifest_path)
    except (FileNotFoundError, OSError) as e:
        print(f"manifest not found: {e}", file=sys.stderr)
        return 2
    except ManifestInvalidError as e:
        print(f"manifest invalid: {e}", file=sys.stderr)
        return 1
    except TerrainError as e:
        print(f"manifest error: {e}", file=sys.stderr)
        return 1

    print(f"ok: manifest version {manifest.version} with {len(manifest.sites)} sites")

    issues: list[str] = []

    if check_urls:
        issues.extend(_check_source_urls(manifest))

    if check_shas:
        issues.extend(_check_source_shas(manifest))

    if issues:
        for issue in issues:
            print(f"error: {issue}", file=sys.stderr)
        return 1

    return 0


def _check_source_shas(manifest) -> list[str]:
    issues: list[str] = []
    for check in check_cached_sources(manifest):
        label = check.label
        if check.status is CachedSourceStatus.PLACEHOLDER:
            issues.append(f"{label}: placeholder sha256")
            continue
        if check.status is CachedSourceStatus.MISSING:
            print(
                f"warning: {label}: raw cache missing for sha {check.plan.sha256}",
                file=sys.stderr,
            )
            continue
        if check.status is CachedSourceStatus.MISMATCH:
            issues.append(
                f"{label}: cached sha mismatch at {check.plan.raw_path}: "
                f"expected {check.plan.sha256}, got {check.actual_sha256}"
            )
            continue
        if check.status is CachedSourceStatus.INVALID_PAYLOAD:
            issues.append(f"{label}: {check.error}")
    return issues


def _check_source_urls(manifest) -> list[str]:
    return [check.issue for check in check_source_urls(manifest) if check.issue]


def _main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="srb asset")
    sub = parser.add_subparsers(dest="cmd", required=True)

    vm = sub.add_parser("validate-manifest", help="validate the curated-site manifest")
    vm.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="path to manifest.yaml (defaults to bundled)",
    )
    vm.add_argument(
        "--check-urls",
        action="store_true",
        help="HEAD-check every source URL (network)",
    )
    vm.add_argument(
        "--check-shas",
        action="store_true",
        help="verify cached _raw/ payloads against manifest sha256",
    )

    args = parser.parse_args(argv)
    if args.cmd == "validate-manifest":
        return validate_manifest_cmd(
            manifest_path=args.manifest,
            check_urls=args.check_urls,
            check_shas=args.check_shas,
        )
    parser.error(f"unknown command: {args.cmd}")


if __name__ == "__main__":
    raise SystemExit(_main())
