"""Apply or verify the pinned local v2e streaming patch.

The ``event-v2e`` extra plus the pinned ``source-event-v2e`` requirement
(:data:`~srb.core.sensor.event_camera.extras.V2E_SOURCE_REQUIREMENT`) install
the reproducible upstream base. This module is the second install step for
``v2e_streaming``: it reads the patch and its digest from the installed SRB
package, applies the patch to the installed ``v2ecore`` source, and fails
closed if the source no longer matches the recorded base context.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.resources
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path
from typing import Any

from srb.core.sensor.event_camera.extras import install_command

_PATCH_RESOURCE = "patches/0001-event-emulator-iter-events.patch"
_MANIFEST_RESOURCE = "v2e_pin.toml"


class V2ePatchError(RuntimeError):
    """The installed v2e source cannot receive the recorded patch."""


def _resources() -> importlib.resources.abc.Traversable:
    return importlib.resources.files("srb.core.sensor.event_camera.backends")


def _load_manifest() -> dict[str, Any]:
    manifest = tomllib.loads(
        _resources().joinpath(_MANIFEST_RESOURCE).read_text(encoding="utf-8")
    )
    if manifest.get("schema_version") != 1:
        raise V2ePatchError("unsupported v2e patch manifest schema")
    return manifest


def _patch_bytes(manifest: dict[str, Any]) -> bytes:
    patch_resource = str(manifest.get("patch_path", _PATCH_RESOURCE))
    if patch_resource != ("srb/core/sensor/event_camera/backends/" + _PATCH_RESOURCE):
        raise V2ePatchError(f"unexpected v2e patch path in manifest: {patch_resource}")
    patch = _resources().joinpath(_PATCH_RESOURCE).read_bytes()
    digest = hashlib.sha256(patch).hexdigest()
    if digest != manifest.get("patch_sha256"):
        raise V2ePatchError(
            "installed v2e patch digest does not match v2e_pin.toml: "
            f"expected {manifest.get('patch_sha256')}, got {digest}"
        )
    return patch


def _manifest_sha256(manifest: dict[str, Any], key: str) -> str:
    value = manifest.get(key)
    if (
        not isinstance(value, str)
        or len(value) != hashlib.sha256().digest_size * 2
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise V2ePatchError(f"invalid SHA-256 value for v2e manifest field {key!r}")
    return value


def _source_sha256(source: Path) -> str:
    digest = hashlib.sha256()
    with source.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _provider_source() -> Path:
    try:
        spec = importlib.util.find_spec("v2ecore.emulator")
    except (ImportError, ModuleNotFoundError, ValueError) as exc:
        raise V2ePatchError(
            "v2ecore is unavailable; install the pinned v2e source with: "
            + install_command("event-v2e")
        ) from exc
    if spec is None or spec.origin is None or spec.origin in {"built-in", "frozen"}:
        raise V2ePatchError("v2ecore.emulator has no patchable source file")
    source = Path(spec.origin).resolve()
    if source.name != "emulator.py" or not source.is_file():
        raise V2ePatchError(f"v2ecore.emulator source is not a file: {source}")
    return source


def apply_v2e_patch(*, check: bool = False) -> Path:
    """Apply the recorded patch and return the provider source path.

    ``check=True`` performs no mutation and succeeds only when the provider
    bytes exactly match the recorded patched source. Applying stages the
    provider in a temporary package tree, verifies the complete resulting
    bytes, and atomically replaces the installed source only after the
    verification succeeds.
    """

    manifest = _load_manifest()
    patch = _patch_bytes(manifest)
    base_source_sha256 = _manifest_sha256(manifest, "base_source_sha256")
    patched_source_sha256 = _manifest_sha256(manifest, "patched_source_sha256")
    source = _provider_source()
    actual_source_sha256 = _source_sha256(source)
    if actual_source_sha256 == patched_source_sha256:
        return source
    if actual_source_sha256 != base_source_sha256:
        raise V2ePatchError(
            "v2ecore.emulator source SHA-256 is not the recorded base or patched "
            f"source: expected base={base_source_sha256} or "
            f"patched={patched_source_sha256}, got {actual_source_sha256}"
        )
    if check:
        raise V2ePatchError(
            "v2ecore.emulator matches the recorded base but is not patched; run "
            "python -m srb.core.sensor.event_camera.backends.v2e_install --apply"
        )

    package_root = source.parent.parent
    try:
        with tempfile.TemporaryDirectory(
            prefix="srb-v2e-patch-",
            dir=str(package_root.parent),
        ) as staging_name:
            staging_root = Path(staging_name)
            staged_source = staging_root / source.parent.name / source.name
            staged_source.parent.mkdir(parents=True)
            shutil.copy2(source, staged_source)
            result = subprocess.run(
                [
                    "patch",
                    "--batch",
                    "--forward",
                    "--fuzz=0",
                    "-p1",
                    "-d",
                    str(staging_root),
                ],
                input=patch,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout).decode(
                    "utf-8", errors="replace"
                )
                raise V2ePatchError(
                    "v2e patch failed against the recorded base source; the "
                    f"provider is not accepted for v2e_streaming:\n{detail.strip()}"
                )
            staged_sha256 = _source_sha256(staged_source)
            if staged_sha256 != patched_source_sha256:
                raise V2ePatchError(
                    "v2e patch produced unexpected source bytes: expected "
                    f"{patched_source_sha256}, got {staged_sha256}"
                )
            if _source_sha256(source) != base_source_sha256:
                raise V2ePatchError(
                    "v2ecore.emulator changed while the v2e patch was staged; "
                    "the installed source was left untouched"
                )
            os.replace(staged_source, source)
    except V2ePatchError:
        raise
    except OSError as exc:
        raise V2ePatchError(
            "could not stage or install the v2e patch; the installed source was "
            "left untouched"
        ) from exc
    return source


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--apply",
        action="store_true",
        help="apply the recorded patch (the default)",
    )
    mode.add_argument(
        "--check",
        action="store_true",
        help="verify that the installed provider is already patched",
    )
    args = parser.parse_args(argv)
    try:
        source = apply_v2e_patch(check=args.check)
    except V2ePatchError as exc:
        print(f"v2e streaming patch unavailable: {exc}", file=sys.stderr)
        return 2
    manifest = _load_manifest()
    mode_name = "verified" if args.check else "applied or already present"
    base_source_sha256 = _manifest_sha256(manifest, "base_source_sha256")
    patched_source_sha256 = _manifest_sha256(manifest, "patched_source_sha256")
    print(
        f"v2e streaming patch {mode_name}: source={source} "
        f"base={manifest['base_sha']} "
        f"base_source_sha256={base_source_sha256} "
        f"patched_source_sha256={patched_source_sha256} "
        f"patch_sha256={manifest['patch_sha256']}"
    )
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised by install docs
    raise SystemExit(main())
