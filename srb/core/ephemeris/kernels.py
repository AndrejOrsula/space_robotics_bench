"""Pinned NAIF kernel bundle: resolution, download, and cache."""

from __future__ import annotations

import os
import stat as stat_module
import threading
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from srb.utils.artifact_fetch import (
    HashMismatchError,
    PlaceholderError,
    _urllib_download,
)
from srb.utils.artifact_fetch import fetch_artifact as _fetch_artifact_seam
from srb.utils.path import SRB_CACHE_PATH
from srb.utils.publication import sha256_file

_VERIFY_LOCK = threading.Lock()
_VERIFIED: dict[tuple[int, str, int, int, int, int, int, str], bool] = {}


def _reset_verification_after_fork() -> None:
    """Reset inherited verification state and locks after fork."""
    global _VERIFY_LOCK, _VERIFIED
    _VERIFY_LOCK = threading.Lock()
    _VERIFIED = {}


if hasattr(os, "register_at_fork"):
    os.register_at_fork(after_in_child=_reset_verification_after_fork)

_NAIF = "https://naif.jpl.nasa.gov/pub/naif/generic_kernels"
_HF_REPO_ENV = "SRB_EPHEMERIS_HF_REPO_ID"  # Repository used to publish kernels.


class KernelsMissingError(RuntimeError):
    pass


@dataclass(frozen=True)
class KernelSpec:
    name: str
    sha256: str
    urls: tuple[str, ...]  # tried in order; NAIF last


def _urls(subpath: str, name: str) -> tuple[str, ...]:
    urls: list[str] = []
    hf_repo = os.environ.get(_HF_REPO_ENV)
    if hf_repo:
        urls.append(
            f"https://huggingface.co/datasets/{hf_repo}/resolve/main/kernels/{name}"
        )
    urls.append(f"{_NAIF}/{subpath}/{name}")
    return tuple(urls)


KERNEL_BUNDLE: tuple[KernelSpec, ...] = (  # order == furnish order
    KernelSpec(
        "naif0012.tls",
        "678e32bdb5a744117a467cd9601cd6b373f0e9bc9bbde1371d5eee39600a039b",
        _urls("lsk", "naif0012.tls"),
    ),
    KernelSpec(
        "pck00011.tpc",
        "3dff7b1dbeceaa01f25467767d3fa25816051c85d162d1edf04acb310ee28bb1",
        _urls("pck", "pck00011.tpc"),
    ),
    KernelSpec(
        "de440s.bsp",
        "c1c7feeab882263fc493a9d5a5b2ddd71b54826cdf65d8d17a76126b260a49f2",
        _urls("spk/planets", "de440s.bsp"),
    ),
    KernelSpec(
        "moon_pa_de440_200625.bpc",
        "60cd55aa401ea2ea97360636f567554bfe4e37bb829f901b4460a455dfaf783f",
        _urls("pck", "moon_pa_de440_200625.bpc"),
    ),
    KernelSpec(
        "moon_de440_250416.tf",
        "a47c71e9c9f33796bdafb2c9d69a7ee447b6016ecad80f71cd6f3e479f9cf768",
        _urls("fk/satellites", "moon_de440_250416.tf"),
    ),
)


def kernel_cache_root() -> Path:
    return Path(
        os.environ.get("SRB_EPHEMERIS_CACHE_ROOT", SRB_CACHE_PATH / "ephemeris")
    )


def _auto_fetch_default() -> bool:
    return os.environ.get("SRB_EPHEMERIS_AUTO_FETCH", "1").lower() not in ("0", "false")


@dataclass(frozen=True)
class _KernelArtifact:
    """Adapts a :class:`KernelSpec` URL attempt to the ``ArtifactLike``
    protocol expected by :func:`srb.utils.artifact_fetch.fetch_artifact`."""

    name: str
    url: str
    sha256: str


def _kernel_placeholder_error(artifact: _KernelArtifact) -> PlaceholderError:
    # Bundle entries have pinned URLs and hashes, so placeholders are
    # unreachable. This callback satisfies fetch_artifact's interface.
    return PlaceholderError(
        artifact, f"kernel artifact {artifact.name!r} is unexpectedly a placeholder"
    )


def _kernel_mismatch_error(
    *, expected: str, actual: str, path: Path
) -> HashMismatchError:
    return HashMismatchError(
        expected=expected,
        actual=actual,
        path=path,
        message=(
            f"kernel integrity check failed for {path}: "
            f"expected sha256={expected}, got sha256={actual}"
        ),
    )


def fetch_artifact(
    url: str, dest: Path, sha256: str, *, fallback_urls: Sequence[str] = ()
) -> Path:
    """Fetch and verify a kernel through the shared artifact cache."""
    dest = Path(dest)
    return _fetch_artifact_seam(
        _KernelArtifact(name=dest.name, url=url, sha256=sha256),
        cache_dir=dest.parent,
        downloader=_urllib_download,
        force=False,
        placeholder_exc=_kernel_placeholder_error,
        mismatch_exc=_kernel_mismatch_error,
        fallback_urls=fallback_urls,
    )


def download_kernels() -> list[Path]:
    root = kernel_cache_root()
    root.mkdir(parents=True, exist_ok=True)
    return [
        fetch_artifact(
            spec.urls[0], root / spec.name, spec.sha256, fallback_urls=spec.urls[1:]
        )
        for spec in KERNEL_BUNDLE
    ]


def kernel_bundle_present() -> bool:
    """Return whether every pinned kernel exists and matches its SHA256.

    This offline probe never resolves, downloads, or imports a SPICE runtime.
    """
    return _verified_kernel_bundle() is not None


def _verified_kernel_bundle() -> tuple[list[Path], tuple[tuple[int, ...], ...]] | None:
    """Return verified paths and their file identities for later rechecking."""
    root = kernel_cache_root()
    pid = os.getpid()
    paths: list[Path] = []
    identities: list[tuple[int, ...]] = []
    try:
        for spec in KERNEL_BUNDLE:
            path = root / spec.name
            before = path.lstat()
            if not stat_module.S_ISREG(before.st_mode):
                return None
            file_identity = (
                before.st_dev,
                before.st_ino,
                before.st_size,
                before.st_mtime_ns,
                before.st_ctime_ns,
            )
            key = (
                pid,
                str(path.absolute()),
                *file_identity,
                spec.sha256,
            )
            with _VERIFY_LOCK:
                verified = _VERIFIED.get(key)
                if verified is None:
                    digest = sha256_file(path)
                    after = path.lstat()
                    stable = (
                        before.st_dev,
                        before.st_ino,
                        before.st_size,
                        before.st_mtime_ns,
                        before.st_ctime_ns,
                    ) == (
                        after.st_dev,
                        after.st_ino,
                        after.st_size,
                        after.st_mtime_ns,
                        after.st_ctime_ns,
                    ) and stat_module.S_ISREG(after.st_mode)
                    verified = stable and digest == spec.sha256
                    # Discard stale verification entries for this path.
                    for old_key in tuple(_VERIFIED):
                        if old_key[1] == key[1] and old_key != key:
                            _VERIFIED.pop(old_key, None)
                    _VERIFIED[key] = verified
            if not verified:
                return None
            paths.append(path)
            identities.append(file_identity)
        return paths, tuple(identities)
    except FileNotFoundError:
        return None


def _resolve_verified_kernels(
    auto_fetch: bool | None = None,
) -> tuple[list[Path], tuple[tuple[int, ...], ...]]:
    """Resolve kernels while retaining the identities that passed verification."""
    verified = _verified_kernel_bundle()
    if verified is not None:
        return verified
    root = kernel_cache_root()
    if not (_auto_fetch_default() if auto_fetch is None else auto_fetch):
        raise KernelsMissingError(
            f"SPICE kernels missing under {root} and auto-fetch is disabled "
            "(SRB_EPHEMERIS_AUTO_FETCH=0); run: srb ephemeris download"
        )
    download_kernels()
    verified = _verified_kernel_bundle()
    if verified is None:
        raise KernelsMissingError(
            f"SPICE kernels could not be verified under {root}; "
            "run: srb ephemeris download"
        )
    return verified


def resolve_kernels(auto_fetch: bool | None = None) -> list[Path]:
    """Paths in furnish order; fetches missing files unless disabled."""
    return _resolve_verified_kernels(auto_fetch)[0]
