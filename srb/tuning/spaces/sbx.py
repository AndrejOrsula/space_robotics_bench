"""SBX state spaces, reusing SB3 spaces for the compatible config format."""

from __future__ import annotations

from srb.tuning.spaces.sb3 import (
    sb3_crossq_space,
    sb3_ppo_space,
    sb3_sac_space,
    sb3_td3_space,
)


def _register() -> None:
    from srb.tuning.spaces import register_space

    # SBX configs match SB3's per-algorithm keys.
    register_space("sbx", "ppo", "state", sb3_ppo_space)
    register_space("sbx", "sac", "state", sb3_sac_space)
    register_space("sbx", "tqc", "state", sb3_sac_space)
    register_space("sbx", "td3", "state", sb3_td3_space)
    register_space("sbx", "crossq", "state", sb3_crossq_space)


_register()
