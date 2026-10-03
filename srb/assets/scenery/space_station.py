from __future__ import annotations

import math
from typing import TYPE_CHECKING, ClassVar

from srb.assets.robot.mobile.orbital import ISS, Gateway
from srb.core.asset import AssetBaseCfg, ExtravehicularScenery
from srb.core.asset.common.thumbnail import ThumbnailCameraPose
from srb.core.reward.spacewalk import legacy_handrail_specs_to_segments
from srb.core.sim import (
    CollisionPropertiesCfg,
    MeshCollisionPropertiesCfg,
    MultiShapeSpawnerCfg,
    UsdFileCfg,
)

if TYPE_CHECKING:
    from srb._typing import AnyEnvCfg

HandrailSpec = tuple[tuple[float, float, float], float, float]
HandrailSegmentSpec = tuple[
    tuple[float, float, float], tuple[float, float, float], float
]

# The task waypoints and spawned handrails share this path, keeping them aligned.
#
# Mesh-surface sampling found only 0.104 m clearance at the original placement;
# the humanoid and viewers were obscured by station structure.
#
# Translating by ``(0, -6, +16) m`` raises clearance to 8.73 m. Path shape,
# rail lengths, reset ranges, and reward scales remain unchanged.
EVA_WAYPOINT_PATH: tuple[tuple[float, float, float], ...] = (
    (-6.0, -6.0, 19.0),
    (-2.0, -6.0, 19.0),
    (2.0, -4.0, 19.0),
    (6.0, -4.0, 19.0),
)
DEFAULT_HANDRAIL_RADIUS: float = 0.03
EVA_HANDRAIL_SEGMENT_SPECS: tuple[HandrailSegmentSpec, ...] = tuple(
    (EVA_WAYPOINT_PATH[i], EVA_WAYPOINT_PATH[i + 1], DEFAULT_HANDRAIL_RADIUS)
    for i in range(len(EVA_WAYPOINT_PATH) - 1)
)


class ISSGatewayScenery(ExtravehicularScenery):
    ## Primary station bodies (easy to override with custom subclasses/templates)
    iss: ISS = ISS()
    gateway: Gateway = Gateway()
    iss_usd_path: str | None = None
    gateway_usd_path: str | None = None
    station_template_usd_path: str | None = None
    use_exact_station_mesh_collision: bool = False

    THUMBNAIL_CAMERA_POSE: ClassVar[ThumbnailCameraPose] = ThumbnailCameraPose(
        eye=(35.0, -41.0, 31.0), lookat=(0.0, -5.0, 19.0)
    )

    ## Base asset (replaced and pinned in ``setup_extras``)
    asset_cfg: AssetBaseCfg = ISS().as_asset_base_cfg(remove_rigid_props=True)

    ## Optional handrails following the default waypoint path:
    ## start position (x, y, z), end position (x, y, z), radius (m).
    ## Derived from EVA_WAYPOINT_PATH so rails stay aligned with the task path.
    handrail_segment_specs: tuple[HandrailSegmentSpec, ...] = EVA_HANDRAIL_SEGMENT_SPECS
    ## Legacy rail shorthand: centre position (x, y, z), X-axis length, radius (m).
    handrail_specs: tuple[HandrailSpec, ...] = ()
    add_handrails: bool = True

    ## Optional combined USD station for easier scene replacement.
    ## When set, this single file is used instead of
    ## ``iss_usd_path`` / ``gateway_usd_path``.

    @staticmethod
    def _build_station_template_cfg(
        template_path: str, prim_path: str, use_exact_mesh: bool
    ) -> AssetBaseCfg:
        return AssetBaseCfg(
            prim_path=prim_path,
            spawn=UsdFileCfg(
                usd_path=template_path,
                collision_props=CollisionPropertiesCfg(),
                mesh_collision_props=MeshCollisionPropertiesCfg(
                    mesh_approximation="none"
                    if use_exact_mesh
                    else "convexDecomposition"
                ),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
        )

    @staticmethod
    def _make_static_collision(asset_cfg: AssetBaseCfg) -> None:
        if asset_cfg.spawn is not None and hasattr(asset_cfg.spawn, "rigid_props"):
            setattr(asset_cfg.spawn, "rigid_props", None)

    @staticmethod
    def _apply_station_overrides(
        body_cfg: AssetBaseCfg,
        usd_path: str | None = None,
        use_exact_mesh: bool = False,
    ) -> None:
        if usd_path is not None and hasattr(body_cfg, "spawn"):
            spawn = body_cfg.spawn
            if spawn is not None and hasattr(spawn, "usd_path"):
                setattr(spawn, "usd_path", usd_path)

        if (
            use_exact_mesh
            and body_cfg.spawn is not None
            and hasattr(body_cfg.spawn, "mesh_collision_props")
        ):
            body_cfg.spawn.mesh_collision_props = MeshCollisionPropertiesCfg(
                mesh_approximation="none"
            )

    @staticmethod
    def _quat_from_x_axis_to_vector(
        vector: tuple[float, float, float],
    ) -> tuple[float, float, float, float]:
        length = math.sqrt(sum(component * component for component in vector))
        if length <= 1e-8:
            return (0.0, 0.0, 0.0, 1.0)

        target = tuple(component / length for component in vector)
        dot = max(-1.0, min(1.0, target[0]))
        if dot > 1.0 - 1e-8:
            return (0.0, 0.0, 0.0, 1.0)
        if dot < -1.0 + 1e-8:
            return (0.0, 0.0, 1.0, 0.0)

        scale = math.sqrt((1.0 + dot) * 2.0)
        inv_scale = 1.0 / scale
        # Cross product of the X axis and target vector.
        return (
            0.0,
            -target[2] * inv_scale,
            target[1] * inv_scale,
            0.5 * scale,
        )

    @staticmethod
    def _build_handrail_segment_cfg(
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        radius: float,
    ) -> AssetBaseCfg:
        if radius <= 0.0:
            raise ValueError("Handrail radius must be positive")

        vector = (
            end[0] - start[0],
            end[1] - start[1],
            end[2] - start[2],
        )
        length = math.sqrt(sum(component * component for component in vector))
        if length <= 1e-8:
            raise ValueError("Handrail segment length must be positive")

        center = tuple(0.5 * (start[i] + end[i]) for i in range(3))
        return AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/handrail",
            spawn=MultiShapeSpawnerCfg(
                shapes=("cylinder",),
                axis="X",
                radius=radius,
                height=length,
                random_choice=False,
                collision_props=CollisionPropertiesCfg(),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=center,
                rot=ISSGatewayScenery._quat_from_x_axis_to_vector(vector),
            ),
        )

    def setup_extras(self, env_cfg: "AnyEnvCfg"):
        scene = env_cfg.scene

        base_prim_path = "/World" if env_cfg.stack else "{ENV_REGEX_NS}"

        if self.station_template_usd_path:
            station_cfg = self._build_station_template_cfg(
                self.station_template_usd_path,
                f"{base_prim_path}/station",
                self.use_exact_station_mesh_collision,
            )
            self._make_static_collision(station_cfg)
            scene.scenery = station_cfg
            scene.gateway = None
        else:
            # Keep the scenery entity name stable for sensors and paths.
            iss_cfg = self.iss.as_asset_base_cfg(remove_rigid_props=True)
            self._apply_station_overrides(
                iss_cfg,
                usd_path=self.iss_usd_path,
                use_exact_mesh=self.use_exact_station_mesh_collision,
            )
            iss_cfg.prim_path = f"{base_prim_path}/iss"
            self._make_static_collision(iss_cfg)
            scene.scenery = iss_cfg

            # Add Gateway as a second collider.
            gateway_cfg = self.gateway.as_asset_base_cfg(remove_rigid_props=True)
            self._apply_station_overrides(
                gateway_cfg,
                usd_path=self.gateway_usd_path,
                use_exact_mesh=self.use_exact_station_mesh_collision,
            )
            gateway_cfg.prim_path = f"{base_prim_path}/gateway"
            self._make_static_collision(gateway_cfg)
            scene.gateway = gateway_cfg

        # Optional handrails for EVA rail-adjacent traversal geometry.
        if self.add_handrails:
            handrail_segments = (
                self.handrail_segment_specs
                + legacy_handrail_specs_to_segments(self.handrail_specs)
            )
            for i, (start, end, radius) in enumerate(handrail_segments):
                rail_cfg = self._build_handrail_segment_cfg(
                    start=start,
                    end=end,
                    radius=radius,
                )
                rail_cfg.prim_path = f"{base_prim_path}/handrail_{i}"
                self._make_static_collision(rail_cfg)
                setattr(scene, f"handrail_{i}", rail_cfg)
