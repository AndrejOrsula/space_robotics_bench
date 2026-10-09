from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch
from simforge import TexResConfig

from srb import assets
from srb.core.asset import AssetVariant, Object, RigidObjectCfg
from srb.core.domain import Domain
from srb.core.manager import SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform
from srb.core.randomization import RandomizationCategory, SrbEventTermCfg
from srb.core.sim import SimforgeAssetCfg
from srb.utils.cfg import configclass

if TYPE_CHECKING:
    from .task import TaskCfg


@configclass
class SampleCfg:
    asset_cfg: RigidObjectCfg = MISSING
    state_randomizer: SrbEventTermCfg = MISSING


def select_sample(
    env_cfg: "TaskCfg",
    *,
    prim_path: str = "{ENV_REGEX_NS}/sample",
    asset_cfg: SceneEntityCfg | None = None,
    seed: int = 0,
    init_state: RigidObjectCfg.InitialStateCfg | None = None,
    scale: tuple[float, float, float] = (0.05, 0.05, 0.05),
    texture_resolution: TexResConfig | None = None,
    **kwargs,
) -> SampleCfg:
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("sample")
    if init_state is None:
        init_state = RigidObjectCfg.InitialStateCfg()

    pose_range = {
        "x": (-0.15, 0.15),
        "y": (-0.3, 0.3),
        "z": (0.1, 0.1),
        "roll": (-torch.pi, torch.pi),
        "pitch": (-torch.pi, torch.pi),
        "yaw": (-torch.pi, torch.pi),
    }

    if isinstance(env_cfg.sample, Object):
        if isinstance(env_cfg.sample, assets.SampleTube):
            pose_range.update(
                {
                    "z": (0.05, 0.05),
                    "roll": (torch.pi / 7, torch.pi / 7),
                    "pitch": (
                        87.5 * torch.pi / 180,
                        87.5 * torch.pi / 180,
                    ),
                    "yaw": (-torch.pi, torch.pi),
                }
            )
        sample_cfg = env_cfg.sample.asset_cfg.copy()  # type: ignore[attr-defined]
        assert isinstance(sample_cfg, RigidObjectCfg)
    else:
        match env_cfg.sample:
            case AssetVariant.PRIMITIVE:
                pose_range["z"] = (0.1, 0.1)
                sample_cfg = assets.RandomShape(scale=scale).asset_cfg

            case AssetVariant.DATASET:
                match env_cfg.domain:
                    case Domain.MARS:
                        pose_range.update(
                            {
                                "z": (0.05, 0.05),
                                "roll": (torch.pi / 7, torch.pi / 7),
                                "pitch": (
                                    87.5 * torch.pi / 180,
                                    87.5 * torch.pi / 180,
                                ),
                                "yaw": (-torch.pi, torch.pi),
                            }
                        )
                        sample_cfg = assets.SampleTube().asset_cfg
                    case _:
                        pose_range["z"] = (0.07, 0.07)
                        sample_cfg = assets.ShortProfilePeg().asset_cfg

            case AssetVariant.PROCEDURAL:
                pose_range["z"] = (0.06, 0.06)
                match env_cfg.domain:
                    case Domain.MOON:
                        sample_cfg = assets.MoonRock(
                            scale=scale, texture_resolution=texture_resolution
                        ).asset_cfg

                    case Domain.MARS:
                        sample_cfg = assets.MarsRock(
                            scale=scale, texture_resolution=texture_resolution
                        ).asset_cfg

                    case _:
                        raise ValueError(
                            "Procedural samples exist only for the moon and mars "
                            f"domains, got env.domain={env_cfg.domain!s}; use "
                            "env.sample=dataset, env.sample=primitive or an Object"
                        )

            case _:
                raise ValueError(
                    "Unsupported sample for sample collection: "
                    f"{env_cfg.sample!r}; expected an Object or "
                    "AssetVariant.PRIMITIVE, DATASET or PROCEDURAL"
                )

    if isinstance(sample_cfg.spawn, SimforgeAssetCfg):
        sample_cfg.spawn.seed = seed

    sample_cfg.prim_path = prim_path
    sample_cfg.init_state = init_state
    sample_cfg.spawn = sample_cfg.spawn.replace(**kwargs)  # type: ignore[union-attr]
    return SampleCfg(
        asset_cfg=sample_cfg,
        state_randomizer=SrbEventTermCfg(
            func=reset_root_state_uniform,
            mode="reset",
            category=RandomizationCategory.TASK,
            params={
                "asset_cfg": asset_cfg,
                "pose_range": pose_range,
                "velocity_range": {},
            },
        ),
    )
