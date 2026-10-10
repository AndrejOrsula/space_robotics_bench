from os import environ

DREAMER_XLA_FLAGS = (
    "--xla_gpu_triton_gemm_any=True --xla_gpu_enable_latency_hiding_scheduler=true "
)


def configure_xla_flags(*, overwrite: bool = False) -> str:
    """Install Dreamer's preferred XLA flags for explicit Dreamer entrypoints."""

    if overwrite or not environ.get("XLA_FLAGS"):
        environ["XLA_FLAGS"] = DREAMER_XLA_FLAGS
    # Dreamer imports initialize JAX before config exists. In Isaac, JAX's
    # default preallocation can starve PhysX/torch and break cuBLAS setup.
    # Profiles disable it; an explicit environment value still wins.
    environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    return environ["XLA_FLAGS"]


__all__ = ["DREAMER_XLA_FLAGS", "configure_xla_flags"]
