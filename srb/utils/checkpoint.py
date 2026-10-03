"""Checkpoint discovery for RL integrations.

A supplied model is returned unchanged. Otherwise, evaluation and resume
search native checkpoint layouts; missing evaluation checkpoints raise
FileNotFoundError, while fresh training returns an empty string. Stdlib-only
so these helpers can be tested without Isaac Sim.
"""

from __future__ import annotations

import os
import stat
from pathlib import Path
from typing import Union

CheckpointResult = Union[Path, str]
_DREAMER_MARKER_MAX_BYTES = 4096


def _is_regular_file(path: Path) -> bool:
    """Reject symlinks during automatic checkpoint discovery."""
    try:
        return path.is_file() and not path.is_symlink()
    except OSError:
        return False


def _regular_files(paths) -> list[Path]:
    return [p for p in paths if _is_regular_file(p)]


def _read_dreamer_marker(path: Path) -> str | None:
    """Read a bounded regular marker without following symlinks."""
    if not _is_regular_file(path):
        return None
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError:
        return None

    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            return None
        with os.fdopen(descriptor, "rb") as marker_file:
            descriptor = -1
            contents = marker_file.read(_DREAMER_MARKER_MAX_BYTES + 1)
    except OSError:
        return None
    finally:
        if descriptor >= 0:
            os.close(descriptor)

    if len(contents) > _DREAMER_MARKER_MAX_BYTES:
        return None
    try:
        return contents.decode("utf-8").strip()
    except UnicodeDecodeError:
        return None


def _step_then_mtime(path: Path) -> tuple[int, float]:
    """Order ``<name>_<step>`` files by step; copied runs may not keep mtimes."""
    step = path.stem.rpartition("_")[2]
    return (int(step) if step.isdigit() else -1, path.stat().st_mtime)


def _latest_by_mtime(
    directory: Path, suffix: str | None = None, prefix: str = "", by_step: bool = False
) -> Path | None:
    """Return the newest (or highest-step) matching file in the directory, or None."""
    if not directory.is_dir():
        return None
    entries = [directory / name for name in os.listdir(directory)]
    files = [
        p
        for p in entries
        if _is_regular_file(p)
        and (suffix is None or p.suffix == suffix)
        and p.name.startswith(prefix)
    ]
    if not files:
        return None
    if by_step:
        return max(files, key=_step_then_mtime)
    return max(files, key=lambda p: p.stat().st_mtime)


def find_sb3_checkpoint(
    logdir: Path,
    model: CheckpointResult | None,
    workflow: str,
    continue_training: bool | None,
) -> CheckpointResult:
    """Find the newest SB3/SBX .zip checkpoint.

    The suffix filter excludes replay-buffer .pkl files saved beside models.
    """
    if model:
        return model
    if workflow == "eval" or continue_training:
        ckpt_dir = logdir / "ckpt"
        latest = _latest_by_mtime(ckpt_dir, suffix=".zip")
        if latest is not None:
            return latest
        if workflow == "eval":
            raise FileNotFoundError(f"No checkpoint found in {ckpt_dir} for eval")
    return ""


def find_skrl_checkpoint(
    logdir: Path,
    model: CheckpointResult | None,
    workflow: str,
    continue_training: bool | None,
) -> CheckpointResult:
    """Find the newest SKRL .pt checkpoint.

    Only combined ``agent_<N>.pt`` files qualify: ``best_agent.pt`` is written
    after the latest one but holds an older training cursor, and
    ``store_separately`` per-module files cannot restore an agent.
    """
    if model:
        return model
    if workflow == "eval" or continue_training:
        ckpt_dir = logdir / "checkpoints"
        latest = _latest_by_mtime(ckpt_dir, suffix=".pt", prefix="agent_", by_step=True)
        if latest is not None:
            return latest
        if workflow == "eval":
            raise FileNotFoundError(f"No checkpoint found in {ckpt_dir} for eval")
    return ""


def find_rsl_rl_checkpoint(
    logdir: Path,
    model: CheckpointResult | None,
    workflow: str,
    continue_training: bool | None,
) -> str:
    """Find the newest model_*.pt checkpoint; return a string for RSL-RL."""
    if model:
        return str(model)
    if workflow == "eval" or continue_training:
        if logdir.is_dir():
            pt_files = sorted(
                _regular_files(logdir.glob("model_*.pt")), key=_step_then_mtime
            )
            if pt_files:
                return str(pt_files[-1])
        if workflow == "eval":
            raise FileNotFoundError(f"No checkpoint found in {logdir} for eval")
    return ""


def find_dreamer_checkpoint(
    logdir: Path,
    model: CheckpointResult | None,
    workflow: str,
    continue_training: bool | None,
    logger=None,
) -> CheckpointResult:
    """Resolve Dreamer's ckpt/latest marker.

        Reject missing or out-of-directory targets; warn for a dangling contained
    target.
    """
    if model:
        return model
    if workflow == "eval" or continue_training:
        ckpt_dir = logdir / "ckpt"
        latest_marker = ckpt_dir / "latest"
        marker = _read_dreamer_marker(latest_marker)
        if marker is not None:
            marker_path = Path(marker)
            candidate = ckpt_dir / marker_path
            ckpt_root = ckpt_dir.resolve()
            try:
                resolved_candidate = candidate.resolve(strict=False)
                candidate_is_contained = (
                    not marker_path.is_absolute()
                    and ".." not in marker_path.parts
                    and resolved_candidate.is_relative_to(ckpt_root)
                )
            except (OSError, RuntimeError):
                candidate_is_contained = False

            if candidate_is_contained and _is_dreamer_checkpoint(candidate):
                return candidate
            if logger is not None and candidate_is_contained:
                logger.warning(
                    f"'latest' marker does not point to a valid checkpoint: {candidate}"
                )
        if workflow == "eval":
            raise FileNotFoundError(f"No checkpoint found in {ckpt_dir} for eval")
    return ""


def _is_dreamer_checkpoint(path: Path) -> bool:
    """Accept a checkpoint directory containing agent.pkl or a legacy regular file.

    Reject symlinks, including a symlinked agent.pkl inside the directory.
    """
    if path.is_symlink():
        return False
    if path.is_dir():
        return _is_regular_file(path / "agent.pkl")
    return _is_regular_file(path)


def find_tdmpc2_checkpoint(
    logdir: Path,
    model: CheckpointResult | None,
    workflow: str,
    continue_training: bool | None,
) -> Path | None:
    """Find a TD-MPC2 checkpoint under models/.

        The highest numeric step competes with nonnumeric files by modification time;
    lower steps cannot win.
    """
    if model is not None:
        return Path(model)
    if continue_training or workflow == "eval":
        models_dir = logdir / "models"
        if models_dir.is_dir():
            pt_files = _regular_files(models_dir.glob("*.pt"))
            if pt_files:
                numeric_files: list[tuple[int, Path]] = []
                other_files: list[Path] = []
                for pt_file in pt_files:
                    try:
                        numeric_files.append((int(pt_file.stem), pt_file))
                    except ValueError:
                        other_files.append(pt_file)

                # Highest step is the only numeric candidate, so a lower step
                # can never win on mtime; a non-numeric file competes on mtime
                # against that highest step only.
                candidates: list[Path] = list(other_files)
                if numeric_files:
                    numeric_files.sort(key=lambda item: item[0])
                    candidates.append(numeric_files[-1][1])
                candidates.sort(key=lambda p: (p.stat().st_mtime_ns, p.name))
                return candidates[-1]
        if workflow == "eval":
            raise FileNotFoundError(f"No checkpoint found in {models_dir} for eval")
    return None
