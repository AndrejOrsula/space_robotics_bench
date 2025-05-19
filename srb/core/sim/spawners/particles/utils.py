from typing import TYPE_CHECKING, List, Sequence

import torch
from pxr import UsdGeom, Vt

if TYPE_CHECKING:
    from srb._typing import AnyEnv
    from srb.core.asset import AssetBase


def get_particles_pos_w(
    env: "AnyEnv",
    particles: "AssetBase",
    env_ids: Sequence[int] | None = None,
) -> torch.Tensor:
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = range(_env.num_envs)
    pos_w: List[torch.Tensor] = []
    for i in env_ids:
        pos_w.append(
            torch.tensor(
                UsdGeom.Points(
                    particles._prims[i]  # type: ignore
                )
                .GetPointsAttr()
                .Get(),
                device=_env.device,
                dtype=torch.float32,
            )
        )
    return torch.stack(pos_w, dim=0)


def get_particles_vel_w(
    env: "AnyEnv",
    particles: "AssetBase",
    env_ids: Sequence[int] | None = None,
) -> torch.Tensor:
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = range(_env.num_envs)
    pos_w: List[torch.Tensor] = []
    for i in env_ids:
        pos_w.append(
            torch.tensor(
                UsdGeom.Points(
                    particles._prims[i]  # type: ignore
                )
                .GetVelocitiesAttr()
                .Get(),
                device=_env.device,
                dtype=torch.float32,
            )
        )
    return torch.stack(pos_w, dim=0)


def set_particles_pos_w(
    env: "AnyEnv",
    particles: "AssetBase",
    positions: torch.Tensor,
    env_ids: Sequence[int] | None = None,
) -> None:
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = range(_env.num_envs)
    for i in env_ids:
        UsdGeom.Points(
            particles._prims[i]  # type: ignore
        ).GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(positions[i].cpu().numpy()))


def set_particles_vel_w(
    env: "AnyEnv",
    particles: "AssetBase",
    velocities: torch.Tensor,
    env_ids: Sequence[int] | None = None,
) -> None:
    _env: "AnyEnv" = env.unwrapped  # type: ignore
    if env_ids is None:
        env_ids = range(_env.num_envs)
    for i in env_ids:
        UsdGeom.Points(
            particles._prims[i]  # type: ignore
        ).GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(velocities[i].cpu().numpy()))
