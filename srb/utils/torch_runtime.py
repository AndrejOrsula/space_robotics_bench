"""Process-level ordering guards for torch's lazily loaded native extensions."""

from __future__ import annotations

__all__ = [
    "warm_torch_compile_stack",
    "warm_torch_native_stack",
    "warm_torchvision_ops",
]


def warm_torch_compile_stack() -> bool:
    """Load ``torch._dynamo`` before RL providers can load TensorFlow.

    This imports Triton before TensorFlow's LLVM can interfere with its native
    extension load. Return whether the compile stack is resident; missing
    ``torch._dynamo`` is treated as a best-effort no-op.
    """

    try:
        import torch._dynamo  # noqa: F401
    except ImportError:
        return False
    return True


def warm_torchvision_ops() -> bool:
    """Load torchvision's custom operators before providers such as robomimic.

    Loading the extension after robomimic's native dependencies can segfault.
    Return whether it is resident; missing torchvision is a best-effort no-op.
    """

    try:
        import torchvision.ops  # noqa: F401
    except ImportError:
        return False
    return True


def warm_torch_native_stack() -> None:
    """Load optional torch native extensions before provider imports."""

    warm_torch_compile_stack()
    warm_torchvision_ops()
