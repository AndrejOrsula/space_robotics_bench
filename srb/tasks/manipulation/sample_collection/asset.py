from typing import TYPE_CHECKING, Tuple

import torch
from pydantic import BaseModel
from simforge import TexResConfig

from srb import assets
from srb.core.asset import AssetVariant, Object, RigidObjectCfg
from srb.core.domain import Domain
from srb.core.manager import EventTermCfg, SceneEntityCfg
from srb.core.mdp import reset_root_state_uniform

if TYPE_CHECKING:
    from .task import TaskCfg


class SampleCfg(BaseModel, arbitrary_types_allowed=True):
    asset_cfg: RigidObjectCfg
    state_randomizer: EventTermCfg


def select_sample(
    env_cfg: "TaskCfg",
    *,
    prim_path: str = "{ENV_REGEX_NS}/sample",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("sample"),
    seed: int = 0,
    init_state: RigidObjectCfg.InitialStateCfg = RigidObjectCfg.InitialStateCfg(),
    scale: Tuple[float, float, float] = (0.05, 0.05, 0.05),
    texture_resolution: TexResConfig | None = None,
    **kwargs,
) -> SampleCfg:
    pose_range = {
        "x": (-0.125, 0.1),
        "y": (-0.25, 0.25),
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
        sample_cfg = env_cfg.sample.asset_cfg.copy()  # type: ignore
        assert isinstance(sample_cfg, RigidObjectCfg)

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
                    sample_cfg = assets.LunarRock(
                        scale=scale, texture_resolution=texture_resolution
                    ).asset_cfg

                case Domain.MARS:
                    sample_cfg = assets.MarsRock(
                        scale=scale, texture_resolution=texture_resolution
                    ).asset_cfg

            sample_cfg.spawn.seed = seed  # type: ignore

    sample_cfg.prim_path = prim_path
    sample_cfg.init_state = init_state
    sample_cfg.spawn.replace(**kwargs)  # type: ignore

    return SampleCfg(
        asset_cfg=sample_cfg,
        state_randomizer=EventTermCfg(
            func=reset_root_state_uniform,
            mode="reset",
            params={
                "asset_cfg": asset_cfg,
                "pose_range": pose_range,
                "velocity_range": {},
            },
        ),
    )
