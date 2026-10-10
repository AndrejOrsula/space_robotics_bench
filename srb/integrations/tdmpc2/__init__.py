import os
import sys
from pathlib import Path

_RUNTIME_POLICY_APPLIED = False


def apply_tdmpc2_runtime_policy() -> None:
    """Apply process-wide torch and environment settings when TD-MPC2 runs."""

    global _RUNTIME_POLICY_APPLIED
    if _RUNTIME_POLICY_APPLIED:
        return
    _RUNTIME_POLICY_APPLIED = True

    os.environ.setdefault("LAZY_LEGACY_OP", "0")
    os.environ.setdefault("TORCHDYNAMO_INLINE_INBUILT_NN_MODULES", "1")

    import torch

    torch.backends.cudnn.benchmark = True
    torch.set_float32_matmul_precision("high")


# Append a validated TD-MPC2 source path for upstream's top-level imports.
# Appending avoids shadowing modules already importable; invalid paths leave
# ``sys.path`` untouched so ``import_upstream`` can report the problem.
TDMPC2_ROOT_MARKERS: tuple[str, ...] = ("common", "envs")


def _looks_like_tdmpc2_root(candidate: Path) -> bool:
    return candidate.is_dir() and all(
        (candidate / marker).is_dir() for marker in TDMPC2_ROOT_MARKERS
    )


_tdmpc2_path = os.environ.get(
    "TDMPC2_PATH",
    str(Path(__file__).resolve().parents[3] / "tdmpc2" / "tdmpc2"),
)
TDMPC2_PATH: str | None = None
if _looks_like_tdmpc2_root(Path(_tdmpc2_path)):
    TDMPC2_PATH = _tdmpc2_path
    if _tdmpc2_path not in sys.path:
        sys.path.append(_tdmpc2_path)
