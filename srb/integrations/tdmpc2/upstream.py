"""The exact qualified source revision is part of the learning contract. SRB carries
Q-init and Ensemble fixes without upstream dependency pins."""

from __future__ import annotations

import inspect
import re
import tomllib
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any

from srb.utils.process_supervisor import (
    ProcessKind,
    ProcessSpawnError,
    ProcessStatus,
    run_owned,
)

_PIN_MANIFEST_KEYS = frozenset(
    {
        "schema_version",
        "base_sha",
        "backport_sha",
        "qualified_sha",
        "rejected_dependency_pin_sha",
        "patch_path",
    }
)
_SHA_RE = re.compile(r"[0-9a-f]{40}")
_PACKAGE_REPO_PATH = PurePosixPath("srb/integrations/tdmpc2")


class _PinManifestError(ValueError):
    """Invalid adjacent TD-MPC2 pin manifest."""


@dataclass(frozen=True)
class _PinManifest:
    schema_version: int
    base_sha: str
    backport_sha: str
    qualified_sha: str
    rejected_dependency_pin_sha: str
    patch_path: str

    def package_patch_path(self, package_dir: Path) -> Path:
        """Resolve the validated patch path within this package."""
        try:
            package_relative = PurePosixPath(self.patch_path).relative_to(
                _PACKAGE_REPO_PATH
            )
        except ValueError as exc:
            raise _PinManifestError(
                "pin manifest patch_path must point inside the TD-MPC2 package"
            ) from exc
        if not package_relative.parts:
            raise _PinManifestError("pin manifest patch_path must name a file")
        return Path(package_dir).joinpath(*package_relative.parts)


def _validate_sha(field: str, value: object) -> str:
    if type(value) is not str or _SHA_RE.fullmatch(value) is None:
        raise _PinManifestError(
            f"pin manifest field {field!r} must be a lowercase 40-hex SHA"
        )
    return value


def _validate_repo_relative_path(value: object) -> str:
    if type(value) is not str or not value:
        raise _PinManifestError("pin manifest patch_path must be a non-empty string")
    if (
        value.startswith(("/", "\\"))
        or "\\" in value
        or "\x00" in value
        or any(ord(char) < 0x20 or ord(char) == 0x7F for char in value)
    ):
        raise _PinManifestError(
            "pin manifest patch_path must be a safe POSIX repository-relative path"
        )
    parts = value.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        raise _PinManifestError(
            "pin manifest patch_path must not contain empty, '.', or '..' parts"
        )
    windows_path = PureWindowsPath(value)
    if PurePosixPath(value).is_absolute() or windows_path.drive or windows_path.root:
        raise _PinManifestError(
            "pin manifest patch_path must be repository-relative, not absolute"
        )
    return value


def _load_pin_manifest(path: Path | str | None = None) -> _PinManifest:
    """Load and validate the adjacent pin manifest."""
    manifest_path = (
        Path(path) if path is not None else Path(__file__).with_name("pin.toml")
    )
    try:
        with manifest_path.open("rb") as stream:
            raw = tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise _PinManifestError(
            f"unable to load TD-MPC2 pin manifest {manifest_path}: {exc}"
        ) from exc

    if type(raw) is not dict or set(raw) != _PIN_MANIFEST_KEYS:
        raise _PinManifestError(
            "TD-MPC2 pin manifest has an unexpected schema or unknown fields"
        )
    schema_version = raw["schema_version"]
    if type(schema_version) is not int or schema_version != 1:
        raise _PinManifestError("pin manifest schema_version must be integer 1")
    return _PinManifest(
        schema_version=schema_version,
        base_sha=_validate_sha("base_sha", raw["base_sha"]),
        backport_sha=_validate_sha("backport_sha", raw["backport_sha"]),
        qualified_sha=_validate_sha("qualified_sha", raw["qualified_sha"]),
        rejected_dependency_pin_sha=_validate_sha(
            "rejected_dependency_pin_sha", raw["rejected_dependency_pin_sha"]
        ),
        patch_path=_validate_repo_relative_path(raw["patch_path"]),
    )


_PIN_MANIFEST = _load_pin_manifest()


# Base SHA, backport SHA, and qualified SHA identify upstream, Q-init correction,
# and cumulative patch. Installer metadata makes the result reproducible.
PINNED_BASE_SHA = _PIN_MANIFEST.base_sha


BACKPORT_COMMIT_SHA = _PIN_MANIFEST.backport_sha


PINNED_SHA = _PIN_MANIFEST.qualified_sha


REJECTED_DEPENDENCY_PIN_SHA = _PIN_MANIFEST.rejected_dependency_pin_sha

BACKPORT_PATCH = _PIN_MANIFEST.package_patch_path(Path(__file__).parent)


EXPECTED_WEIGHT_STD = 0.02
WEIGHT_STD_TOLERANCE = 0.25

_BACKPORT_REQUIRED_MESSAGE = (
    "TD-MPC2 upstream checkout is missing the qualified Q-ensemble source "
    "correction (upstream backport {backport} plus the SRB Ensemble lifecycle "
    "fix). Without it every Q MLP keeps PyTorch's default initialization "
    "(non-zero biases, ~5x wider weights) instead of TD-MPC2's "
    "trunc_normal_(std=0.02), which changes critic learning, or WorldModel.to() "
    "fails while copying the stateless meta-device template. Apply the recorded "
    "patch to the checkout at {checkout}:\n"
    "    .docker/install/install_tdmpc2.bash {checkout}\n"
    "(or, in place, reproduce the recorded pin exactly:)\n"
    "    git -C {checkout} checkout -B srb-pin-qualified {base}\n"
    "    GIT_COMMITTER_NAME='SRB Maintainers' "
    "GIT_COMMITTER_EMAIL='srb-maintainers@localhost' \\\n"
    "      git -C {checkout} -c commit.gpgsign=false am "
    "--ignore-whitespace --committer-date-is-author-date {patch}\n"
    "SRB is qualified against upstream {base} plus the recorded cumulative "
    "source patch (= {pinned})."
)


class Tdmpc2UpstreamError(RuntimeError):
    """The installed upstream TD-MPC2 checkout is not the qualified pin."""


def import_upstream(name: str, attribute: str | None = None) -> Any:
    """Import an upstream module with actionable checkout guidance."""
    from srb.integrations.tdmpc2 import apply_tdmpc2_runtime_policy

    # Set SRB's Torch runtime policy before importing upstream modules.
    apply_tdmpc2_runtime_policy()
    try:
        module = __import__(name, fromlist=["_"])
    except ImportError as e:
        raise ImportError(
            f"TD-MPC2 upstream module '{name}' not importable. Ensure the "
            "TD-MPC2 checkout is present at the default location "
            "(`<repo_root>/tdmpc2/tdmpc2`) or set the `TDMPC2_PATH` "
            f"environment variable to point at your clone. Original error: {e}"
        ) from e
    _require_module_from_upstream_root(name, module)
    return getattr(module, attribute) if attribute else module


def _require_module_from_upstream_root(name: str, module: Any) -> None:
    """Reject upstream imports resolved outside the qualified checkout."""
    from srb.integrations.tdmpc2 import TDMPC2_PATH

    if TDMPC2_PATH is None:
        return
    module_file = (
        getattr(module, "__file__", None) or getattr(module, "__path__", [None])[0]
    )
    if module_file is None:
        return
    root = Path(TDMPC2_PATH).resolve()
    if root not in Path(module_file).resolve().parents:
        raise Tdmpc2UpstreamError(
            f"upstream TD-MPC2 module '{name}' resolved to {module_file}, which "
            f"is outside the qualified checkout at {root}. A same-named "
            "top-level package is shadowing it; rename it or set TDMPC2_PATH."
        )


def upstream_module_root() -> Path | None:
    """Return the importable upstream package directory, if present."""
    try:
        import tdmpc2 as _tdmpc2_module
    except ImportError:
        return None
    module_file = getattr(_tdmpc2_module, "__file__", None)
    if not module_file:
        return None
    return Path(module_file).resolve().parent


def upstream_checkout_root() -> Path | None:
    """Return the upstream Git work tree, if present."""
    module_root = upstream_module_root()
    if module_root is None:
        return None
    try:
        result = run_owned(
            ["git", "-C", str(module_root), "rev-parse", "--show-toplevel"],
            kind=ProcessKind.FINITE,
            capture_output=True,
            timeout=15,
        )
    except (OSError, ProcessSpawnError):
        return None
    if (
        result.status is not ProcessStatus.EXITED
        or result.returncode != 0
        or not result.cleanup_complete
    ):
        return None
    return Path((result.stdout or "").strip())


def upstream_revision() -> str | None:
    """Return the upstream checkout HEAD, if available."""
    checkout = upstream_checkout_root()
    if checkout is None:
        return None
    try:
        result = run_owned(
            ["git", "-C", str(checkout), "rev-parse", "HEAD"],
            kind=ProcessKind.FINITE,
            capture_output=True,
            timeout=15,
        )
    except (OSError, ProcessSpawnError):
        return None
    if (
        result.status is not ProcessStatus.EXITED
        or result.returncode != 0
        or not result.cleanup_complete
    ):
        return None
    return (result.stdout or "").strip() or None


def dependency_versions() -> dict[str, str]:
    """Return versions of effective learner dependencies."""
    versions: dict[str, str] = {}
    for name in ("torch", "tensordict", "torchrl", "numpy", "gymnasium"):
        try:
            module = __import__(name)
        except ImportError:  # pragma: no cover - all are Isaac-provided
            continue
        versions[name] = str(getattr(module, "__version__", "unknown"))
    return versions


def q_ensemble_backport_applied() -> bool:
    """Inspect Ensemble construction: Q MLPs need ``weight_init`` before wrapping in
    ``TensorDictParams``, which blocks later ``WorldModel.apply``."""
    WorldModel = import_upstream("common.world_model", "WorldModel")

    try:
        source = inspect.getsource(WorldModel.__init__)
    except (OSError, TypeError):  # pragma: no cover - source always available
        return False
    match = re.search(r"self\._Qs\s*=\s*layers\.Ensemble\(.*", source)
    if match is None:
        return False
    return "weight_init" in match.group(0)


def ensemble_device_conversion_applied() -> bool:
    """The registered template is populated at call time. Converting it from ``meta``
    makes ``WorldModel.to()`` fail on Torch 2.11."""
    Ensemble = import_upstream("common.layers", "Ensemble")
    try:
        source = inspect.getsource(Ensemble._apply)
    except (AttributeError, OSError, TypeError):  # pragma: no cover - source exists
        return False
    return "self.params._apply(fn)" in source and "recurse=False" in source


def assert_upstream_compatible() -> None:
    """Missing checkout preserves ``import_upstream``'s ``TDMPC2_PATH`` ImportError;
    a present but unpatched checkout raises ``Tdmpc2UpstreamError`` with re-pin
    guidance."""
    backport_applied = q_ensemble_backport_applied()
    device_conversion_applied = ensemble_device_conversion_applied()
    revision = upstream_revision()
    if revision != PINNED_SHA:
        checkout = upstream_checkout_root() or upstream_module_root() or "<tdmpc2>"
        raise Tdmpc2UpstreamError(
            f"TD-MPC2 checkout at {checkout} has revision {revision!r}; SRB "
            f"requires the exact qualified revision {PINNED_SHA}. Reinstall "
            "the pinned checkout before training or evaluation."
        )
    if backport_applied and device_conversion_applied:
        return
    checkout = upstream_checkout_root() or upstream_module_root() or "<tdmpc2>"
    raise Tdmpc2UpstreamError(
        _BACKPORT_REQUIRED_MESSAGE.format(
            backport=BACKPORT_COMMIT_SHA,
            base=PINNED_BASE_SHA,
            pinned=PINNED_SHA,
            patch=BACKPORT_PATCH,
            checkout=checkout,
        )
    )


def q_ensemble_init_report(model: Any) -> dict[str, Any]:
    """Measure Q ensemble initialization statistics."""
    import torch

    params = model._Qs.params
    layers: dict[str, dict[str, Any]] = {}
    for key, value in params.items(include_nested=True, leaves_only=True):
        name = (
            "/".join(str(part) for part in key) if isinstance(key, tuple) else str(key)
        )
        layer, _, kind = name.rpartition("/")
        tensor = value.detach()
        entry = layers.setdefault(layer, {})
        entry[kind] = tensor

    # Top-level numeric entries are Linear layers; nested entries like ``0/ln`` are
    # LayerNorm and retain PyTorch defaults.
    linear_layers = sorted((name for name in layers if name.isdigit()), key=int)
    output_layer = linear_layers[-1] if linear_layers else None

    report: dict[str, Any] = {
        "num_members": int(len(model._Qs)),
        "layers": {},
        "output_layer": output_layer,
    }
    for name in linear_layers:
        entry = layers[name]
        weight = entry.get("weight")
        bias = entry.get("bias")
        member_spread = None
        if weight is not None and weight.shape[0] > 1:
            member_spread = float(
                min(
                    (weight[i] - weight[j]).abs().max().item()
                    for i in range(weight.shape[0])
                    for j in range(i + 1, weight.shape[0])
                )
            )
        member_weight_std = None
        if weight is not None and weight.ndim >= 1:
            member_weight_std = [
                float(weight[i].std().item()) for i in range(weight.shape[0])
            ]
        report["layers"][name] = {
            "weight_std": float(weight.std().item()) if weight is not None else None,
            "member_weight_std": member_weight_std,
            "weight_absmax": float(weight.abs().max().item())
            if weight is not None
            else None,
            "bias_absmax": float(bias.abs().max().item()) if bias is not None else None,
            "min_member_difference": member_spread,
            "finite": bool(
                torch.isfinite(weight).all().item() if weight is not None else True
            ),
        }
    return report


def assert_q_ensemble_initialized(model: Any) -> dict[str, Any]:
    """Assert TD-MPC2 initialization invariants and return their measurements."""
    report = q_ensemble_init_report(model)
    output_layer = report["output_layer"]
    low = EXPECTED_WEIGHT_STD * (1.0 - WEIGHT_STD_TOLERANCE)
    high = EXPECTED_WEIGHT_STD * (1.0 + WEIGHT_STD_TOLERANCE)

    if not report["layers"]:
        raise Tdmpc2UpstreamError("Q ensemble exposes no Linear layers to check")

    for name, stats in report["layers"].items():
        if not stats["finite"]:
            raise Tdmpc2UpstreamError(f"Q ensemble layer '{name}' is non-finite")
        if stats["bias_absmax"] not in (None, 0.0):
            raise Tdmpc2UpstreamError(
                f"Q ensemble layer '{name}' bias is not zero "
                f"({stats['bias_absmax']}): TD-MPC2's weight initializer did "
                "not reach the ensemble members"
            )
        if name == output_layer:
            if stats["weight_absmax"] not in (None, 0.0):
                raise Tdmpc2UpstreamError(
                    f"Q ensemble output layer '{name}' weight is not zeroed "
                    f"({stats['weight_absmax']})"
                )
            continue
        if stats["min_member_difference"] is not None and (
            stats["min_member_difference"] <= 0.0
        ):
            raise Tdmpc2UpstreamError(
                f"Q ensemble layer '{name}' is degenerate: two members share "
                "identical weights"
            )
        member_stds = stats["member_weight_std"] or (
            [stats["weight_std"]] if stats["weight_std"] is not None else []
        )
        if not member_stds:
            raise Tdmpc2UpstreamError(
                f"Q ensemble layer '{name}' exposes no weights to measure"
            )

        # Per-member bounds do not imply pooled bounds: differing member means can
        # push pooled std outside the band.
        pooled_std = stats["weight_std"]
        if pooled_std is None or not (low <= pooled_std <= high):
            raise Tdmpc2UpstreamError(
                f"Q ensemble layer '{name}' pooled weight std {pooled_std} is "
                f"outside [{low:.4f}, {high:.4f}]: expected "
                f"trunc_normal_(std={EXPECTED_WEIGHT_STD})"
            )
        for member, std in enumerate(member_stds):
            if std is None or not (low <= std <= high):
                raise Tdmpc2UpstreamError(
                    f"Q ensemble layer '{name}' member {member} weight std "
                    f"{std} is outside [{low:.4f}, {high:.4f}]: expected "
                    f"trunc_normal_(std={EXPECTED_WEIGHT_STD})"
                )
    return report


def upstream_pin_record() -> dict[str, Any]:
    """Correction flags are tri-state; semantic checks do not authenticate the pinned
    revision, whose exact SHA match is recorded separately."""
    try:
        backport_applied: bool | None = q_ensemble_backport_applied()
    except ImportError:
        backport_applied = None
    try:
        device_conversion_applied: bool | None = ensemble_device_conversion_applied()
    except ImportError:
        device_conversion_applied = None
    try:
        installed_revision = upstream_revision()
    except ImportError:
        installed_revision = None
    installed_revision_matches_pin: bool | None = (
        None if installed_revision is None else installed_revision == PINNED_SHA
    )
    return {
        "pinned_base_sha": PINNED_BASE_SHA,
        "backport_commit_sha": BACKPORT_COMMIT_SHA,
        "pinned_sha": PINNED_SHA,
        "rejected_dependency_pin_sha": REJECTED_DEPENDENCY_PIN_SHA,
        "backport_patch": BACKPORT_PATCH.name,
        "installed_revision": installed_revision,
        "installed_revision_matches_pin": installed_revision_matches_pin,
        "q_ensemble_backport_applied": backport_applied,
        "ensemble_device_conversion_applied": device_conversion_applied,
        "dependencies": dependency_versions(),
    }


__all__ = [
    "BACKPORT_COMMIT_SHA",
    "BACKPORT_PATCH",
    "PINNED_BASE_SHA",
    "PINNED_SHA",
    "REJECTED_DEPENDENCY_PIN_SHA",
    "Tdmpc2UpstreamError",
    "assert_q_ensemble_initialized",
    "assert_upstream_compatible",
    "dependency_versions",
    "ensemble_device_conversion_applied",
    "import_upstream",
    "q_ensemble_backport_applied",
    "q_ensemble_init_report",
    "upstream_checkout_root",
    "upstream_module_root",
    "upstream_pin_record",
    "upstream_revision",
]
