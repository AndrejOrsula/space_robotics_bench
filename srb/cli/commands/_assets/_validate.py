"""``srb asset validate-manifest`` -- schema-validate the curated-site manifest."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional


def run(
    *,
    manifest: Optional[str] = None,
    check_urls: bool = False,
    check_shas: bool = False,
    **_unused,
) -> int:
    """Delegate to the standalone ``srb.terrain.cli.validate_manifest_cmd``."""
    from srb.terrain.cli import validate_manifest_cmd

    if manifest == "":
        # An empty path must not silently validate the packaged manifest.
        print("validate-manifest: --manifest must not be empty", file=sys.stderr)
        return 2
    manifest_path = Path(manifest) if manifest is not None else None
    return validate_manifest_cmd(
        manifest_path,
        check_urls=check_urls,
        check_shas=check_shas,
    )
