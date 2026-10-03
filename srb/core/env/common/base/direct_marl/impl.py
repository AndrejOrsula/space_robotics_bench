from isaaclab.envs import DirectMARLEnv as __DirectMARLEnv

from srb.core.env.common.base.seed import with_replicator_seed
from srb.core.sim.spawners.multi_asset_compat import (
    convert_scene_multi_variant_spawners,
)
from srb.utils.failures import attempt_cleanup

from .cfg import DirectMarlEnvCfg

# Bound outside the class body, where a leading ``__`` name is mangled.
_UPSTREAM_SEED = __DirectMARLEnv.seed


class DirectMarlEnv(__DirectMARLEnv):
    cfg: DirectMarlEnvCfg

    seed = with_replicator_seed(_UPSTREAM_SEED)

    def __init__(self, cfg: DirectMarlEnvCfg, **kwargs):
        # Materialize entries added by subclass post-init hooks.
        cfg.finalize()
        # Expose variant counts before the clone planner runs.
        convert_scene_multi_variant_spawners(cfg.scene)
        super().__init__(cfg, **kwargs)

        try:
            self.cfg.visuals.func(self.cfg.visuals)
        except BaseException as error:
            # Release the constructed simulation so construction can be retried.
            _, close_error = attempt_cleanup(self.close)
            if close_error is not None:
                error.add_note(
                    f"environment close failed during constructor cleanup: {close_error!r}"
                )
            raise
