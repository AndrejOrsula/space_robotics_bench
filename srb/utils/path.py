import datetime
import hashlib
import os
import platform
import re
import tempfile
from pathlib import Path

# In wheels, Hatch-bundled resources live under ``srb/_resources`` rather than
# beside ``site-packages/srb`` as they do in a checkout.
_PACKAGE_DIR = Path(__file__).resolve().parents[1]
_CHECKOUT_ROOT = _PACKAGE_DIR.parent
SRB_IS_SOURCE_CHECKOUT = (
    (_CHECKOUT_ROOT / "pyproject.toml").is_file()
    and (_CHECKOUT_ROOT / "srb").is_dir()
    and (_CHECKOUT_ROOT / "apps").is_dir()
)
_RESOURCE_ROOT = (
    _CHECKOUT_ROOT if SRB_IS_SOURCE_CHECKOUT else _PACKAGE_DIR / "_resources"
)

# Checkout root, or installed package root.
SRB_DIR = _CHECKOUT_ROOT if SRB_IS_SOURCE_CHECKOUT else _PACKAGE_DIR

# Apps (experience) directory.
SRB_APPS_DIR = _RESOURCE_ROOT.joinpath("apps")

# Assets directory.
SRB_ASSETS_DIR = _RESOURCE_ROOT.joinpath("assets")
SRB_ASSETS_DIR_SRB = SRB_ASSETS_DIR.joinpath("srb_assets")
SRB_ASSETS_DIR_SRB_LIGHT = SRB_ASSETS_DIR_SRB.joinpath("light")
SRB_ASSETS_DIR_SRB_OBJECT = SRB_ASSETS_DIR_SRB.joinpath("object")
SRB_ASSETS_DIR_SRB_ROBOT = SRB_ASSETS_DIR_SRB.joinpath("robot")
SRB_ASSETS_DIR_SRB_SCENERY = SRB_ASSETS_DIR_SRB.joinpath("scenery")
SRB_ASSETS_DIR_SRB_SKYDOME = SRB_ASSETS_DIR_SRB.joinpath("skydome")
SRB_ASSETS_DIR_SRB_SKYDOME_LOW_RES = SRB_ASSETS_DIR_SRB_SKYDOME.joinpath("low_res")
SRB_ASSETS_DIR_SRB_SKYDOME_HIGH_RES = SRB_ASSETS_DIR_SRB_SKYDOME.joinpath("high_res")
SRB_ASSETS_DIR_SRB_LIGHTS = SRB_ASSETS_DIR_SRB.joinpath("lights")
SRB_ASSETS_DIR_SRB_ROCKS = SRB_ASSETS_DIR_SRB.joinpath("scenery", "rocks")

# Task sidecar consulted by CLI preflight.
SRB_TASKS_SIDECAR = _RESOURCE_ROOT.joinpath("docs", "data", "tasks.toml")

# Graphics directory (``docs/src/_graphics`` in a checkout).
SRB_GRAPHICS_DIR = _RESOURCE_ROOT.joinpath("docs", "src", "_graphics")
SRB_GRAPHICS_DIR_ASSETS = SRB_GRAPHICS_DIR.joinpath("assets")
SRB_GRAPHICS_DIR_TASKS = SRB_GRAPHICS_DIR.joinpath("tasks")


def _resolve_default_cache_root() -> Path:
    """Choose a checkout-local or per-user cache root."""

    if SRB_IS_SOURCE_CHECKOUT:
        return SRB_DIR.joinpath(".cache")
    xdg_cache_home = os.environ.get("XDG_CACHE_HOME", "")
    if xdg_cache_home:
        candidate = Path(xdg_cache_home).expanduser()
        if candidate.is_absolute():
            return candidate.joinpath("srb")
    return Path.home().joinpath(".cache", "srb")


def _resolve_cache_root() -> Path:
    """Resolve the process cache root selected by the operator."""

    configured_root = os.environ.get("SRB_CACHE_ROOT")
    if not configured_root:
        return _resolve_default_cache_root()
    return Path(configured_root).expanduser().resolve(strict=False)


# Cache directories.
SRB_DEFAULT_CACHE_PATH = _resolve_default_cache_root()
SRB_CACHE_PATH = _resolve_cache_root()
SRB_CACHE_OWNER = "srb.cache"
SRB_ISAACSIM_CACHE_OWNER = "srb.isaacsim-cache"


def _identity_value(*names: str, default: str) -> str:
    """Return the first non-empty environment value for a cache identity."""

    for name in names:
        value = os.environ.get(name)
        if value:
            return value.strip()
    return default


def _identity_component(value: str, *, fallback: str = "unattested") -> str:
    """Keep cache namespace components deterministic and path-safe.

    Full digests and ordinary version/provider labels remain readable. An
    invalid value is represented by a short digest rather than being allowed
    to introduce path separators or an empty namespace component.
    """

    candidate = value.strip().lower()
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,127}", candidate):
        return candidate
    if not candidate:
        return fallback
    return f"sha256-{hashlib.sha256(candidate.encode('utf-8')).hexdigest()[:16]}"


def _normalise_architecture(value: str) -> str:
    aliases = {
        "amd64": "x86_64",
        "x86-64": "x86_64",
        "arm64": "aarch64",
    }
    candidate = value.strip().lower()
    return _identity_component(aliases.get(candidate, candidate), fallback="unknown")


SRB_CACHE_IDENTITY: dict[str, str] = {
    "schema": "1",
    "isaac_sim": _identity_component(
        _identity_value("SRB_ISAACSIM_VERSION", "ISAACSIM_VERSION", default="6.1.0")
    ),
    "isaac_lab": _identity_component(
        _identity_value("SRB_ISAACLAB_VERSION", "ISAACLAB_VERSION", default="3.0.0")
    ),
    "provider": _identity_component(
        _identity_value("SRB_CACHE_PROVIDER", default="srb")
    ),
    "architecture": _normalise_architecture(
        _identity_value("SRB_CACHE_ARCH", default=platform.machine())
    ),
    "source_digest": _identity_component(
        _identity_value(
            "SRB_SOURCE_TREE_DIGEST", "SRB_CACHE_SOURCE_DIGEST", default="unattested"
        )
    ),
    "asset_digest": _identity_component(
        _identity_value(
            "SRB_ASSET_CONTENT_DIGEST", "SRB_CACHE_ASSET_DIGEST", default="unattested"
        )
    ),
    "asset_gitlink": _identity_component(
        _identity_value("SRB_ASSETS_GITLINK_SHA", default="unattested")
    ),
}
SRB_CACHE_NAMESPACE = "__".join(
    f"{key}-{value}" for key, value in SRB_CACHE_IDENTITY.items()
)
SRB_CACHE_NAMESPACE_PATH = SRB_CACHE_PATH / "namespaces" / SRB_CACHE_NAMESPACE

# Kit and Docker write here; cleanup requires an explicit ownership marker.
ISAACSIM_CACHE_RELPATHS: tuple[str, ...] = (
    ".cache/isaac-sim",
    ".cache/ov",
    ".cache/nvidia/GLCache",
    ".nv/ComputeCache",
    ".cache/packman",
)
# Entries mounted by Docker's NGC profiles. Adoption rejects unexpected data
# to protect unrelated contents from cleanup.
NGC_CACHE_LAYOUT: frozenset[str] = frozenset(
    ("cache", "compute", "kit", "logs", "config", "data", "pkg", "hub")
)
# Root for immutable offline cache generations.
SRB_OFFLINE_CACHE_GENERATION_ROOT_PATH = SRB_CACHE_NAMESPACE_PATH / "offline"
SRB_CACHE_GENERATION_PATH = SRB_CACHE_NAMESPACE_PATH / "generation.json"
SRB_GUI_CATALOG_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH / "gui_catalog.json"

# Hyperparameters directory.
SRB_HYPERPARAMS_DIR = _RESOURCE_ROOT.joinpath("hyperparams")


# Logs are checkout-local in development and follow Isaac Lab's working
# directory convention in installed wheels.
SRB_LOGS_DIR = (
    SRB_DIR.joinpath("logs") if SRB_IS_SOURCE_CHECKOUT else Path.cwd().joinpath("logs")
)

DEFAULT_DATETIME_FORMAT = "%Y%m%dT%H%M%S"


def stamp_dir(directory: Path, timestamp_format: str = DEFAULT_DATETIME_FORMAT) -> Path:
    return directory.joinpath(datetime.datetime.now().strftime(timestamp_format))


#: Upper bound on same-timestamp collision suffixes tried by ``new_logdir``.
_MAX_LOGDIR_COLLISIONS = 999


def new_logdir(
    env_id: str,
    workflow: str,
    root: Path = SRB_LOGS_DIR,
    timestamp_format: str = DEFAULT_DATETIME_FORMAT,
    namespace: str = "srb",
) -> Path:
    """Atomically reserve a unique ``<root>/<env>/<workflow>/<stamp>`` directory.

    Collisions receive sortable ``-001``, ``-002``, ... suffixes, preserving
    ``last_logdir`` and ``--continue`` selection of the newest reservation.
    """

    stamped = stamp_dir(
        root.joinpath(env_id.removeprefix(f"{namespace}/")).joinpath(workflow),
        timestamp_format=timestamp_format,
    )
    stamped.parent.mkdir(parents=True, exist_ok=True)
    for collision in range(_MAX_LOGDIR_COLLISIONS + 1):
        candidate = (
            stamped
            if collision == 0
            else stamped.with_name(f"{stamped.name}-{collision:03d}")
        )
        try:
            candidate.mkdir()
        except FileExistsError:
            continue
        return candidate
    raise FileExistsError(
        f"Could not reserve a unique run directory for {stamped}: "
        f"{_MAX_LOGDIR_COLLISIONS + 1} same-timestamp runs already exist"
    )


def new_eval_logdir(training_logdir: Path) -> Path:
    """Atomically reserve a timestamped evaluation directory under a training run."""

    eval_root = Path(training_logdir).joinpath("eval")
    eval_root.mkdir(parents=True, exist_ok=True)
    prefix = datetime.datetime.now().strftime(DEFAULT_DATETIME_FORMAT) + "-"
    return Path(tempfile.mkdtemp(prefix=prefix, dir=str(eval_root)))


# Path to a cached list of registered entities
SRB_ENV_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("env.json")
SRB_OBJECT_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("object.json")
SRB_ROBOT_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("robot.json")
SRB_SCENERY_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("scenery.json")
SRB_HARDWARE_INTERFACE_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("hardware.json")
SRB_CONFIG_KEYS_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("config_keys.json")
SRB_ENUM_VALUES_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath("enum_values.json")
SRB_AGENT_CONFIG_KEYS_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath(
    "agent_config_keys.json"
)
SRB_AGENT_ENUM_VALUES_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH.joinpath(
    "agent_enum_values.json"
)
SRB_ACTION_MODES_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH / "action_modes.json"
SRB_TASK_DEFAULTS_CACHE_PATH = SRB_CACHE_NAMESPACE_PATH / "task_defaults.json"


def claim_srb_cache_root(
    *,
    cache_root: Path | None = None,
    repository_root: Path | None = None,
) -> Path:
    """Authorize later cleanup of one safe SRB cache root.

    The fixed repository-local ``.cache`` directory can be explicitly adopted
    even when it contains legacy cache data. A custom root must be new, empty,
    or already carry a matching marker.
    """

    from srb.utils.owned_directory import (
        adopt_owned_directory,
        claim_owned_directory,
    )

    selected_root = Path(cache_root) if cache_root is not None else SRB_CACHE_PATH
    selected_repository = (
        Path(repository_root) if repository_root is not None else SRB_DIR
    )
    default_root = (selected_repository / ".cache").resolve(strict=False)
    selected_resolved = selected_root.resolve(strict=False)
    if selected_resolved == default_root and selected_root.exists():
        return adopt_owned_directory(
            selected_root,
            owner=SRB_CACHE_OWNER,
            expected_root=default_root,
            protected_roots=(selected_repository,),
        )
    return claim_owned_directory(
        selected_root,
        owner=SRB_CACHE_OWNER,
        protected_roots=(selected_repository,),
    )


def isaacsim_cache_roots(*, home: Path | None = None) -> tuple[Path, ...]:
    """Return the fixed vendor roots used by Kit and Docker cache profiles."""

    selected_home = Path(home) if home is not None else Path.home()
    cache_roots = [selected_home / relpath for relpath in ISAACSIM_CACHE_RELPATHS]
    ngc_cache_root = _ngc_cache_root(selected_home)
    if ngc_cache_root is not None:
        cache_roots.append(ngc_cache_root)

    return tuple(cache_roots)


def claim_isaacsim_cache_roots(*, home: Path | None = None) -> tuple[Path, ...]:
    """Explicitly adopt existing fixed Isaac cache roots for safe cleanup."""

    from srb.utils.owned_directory import (
        OWNERSHIP_MARKER_NAME,
        OwnedDirectoryError,
        adopt_owned_directory,
    )

    selected_home = (Path(home) if home is not None else Path.home()).resolve(
        strict=True
    )
    ngc_cache_root = _ngc_cache_root(selected_home)
    claimed: list[Path] = []
    for cache_root in isaacsim_cache_roots(home=selected_home):
        if not cache_root.is_dir() or cache_root.is_symlink():
            continue
        if cache_root == ngc_cache_root:
            unexpected = sorted(
                entry.name
                for entry in cache_root.iterdir()
                if entry.name not in NGC_CACHE_LAYOUT
                and entry.name != OWNERSHIP_MARKER_NAME
            )
            if unexpected:
                raise OwnedDirectoryError(
                    f"refusing to adopt NGC cache root {cache_root}: it holds "
                    f"entries outside the NGC cache layout: {unexpected}"
                )
        claimed.append(
            adopt_owned_directory(
                cache_root,
                owner=SRB_ISAACSIM_CACHE_OWNER,
                expected_root=cache_root,
                protected_roots=(selected_home,),
            )
        )
    return tuple(claimed)


def _ngc_cache_root(home: Path) -> Path | None:
    """Return the one NGC cache root selected by ``.docker/run.bash``."""

    configured = os.environ.get("SRB_NGC_CACHE_DIR") or None
    if configured is not None:
        root = Path(configured)
        if not root.is_absolute():
            return None
        return root

    platform_name = next(
        (
            os.environ[name]
            for name in (
                "SRB_NGC_PLATFORM",
                "SRB_DOCKER_PLATFORM",
                "DOCKER_DEFAULT_PLATFORM",
                "TARGETPLATFORM",
            )
            if os.environ.get(name)
        ),
        platform.machine(),
    ).lower()
    if platform_name in {"linux/amd64", "amd64", "x86_64"}:
        architecture = "amd64"
    elif platform_name in {"linux/arm64", "arm64", "aarch64"}:
        architecture = "arm64"
    else:
        return None
    return home / ".cache/space_robotics_bench/ngc/isaac-sim-6.1" / architecture
