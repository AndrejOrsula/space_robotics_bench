from os import environ

environ["XLA_FLAGS"] = (
    "--xla_gpu_triton_gemm_any=True "
    "--xla_gpu_enable_latency_hiding_scheduler=true "
    "--xla_gpu_enable_command_buffer='' "
    "--xla_disable_hlo_passes=collective-permute-motion "
    "--xla_gpu_experimental_pipeline_parallelism_opt_level=PIPELINE_PARALLELISM_OPT_LEVEL_ENABLE "
)
