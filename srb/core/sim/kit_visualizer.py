"""Keep the Kit viewport visualizer aligned with an env cfg's viewer pose."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from isaaclab.envs import ViewerCfg

    from srb._typing import AnyEnvCfg


def set_kit_visualizer_camera(visualizer_cfg: Any, viewer: "ViewerCfg") -> None:
    """Copy the viewer pose to supported visualizer camera fields.

    Field names vary across Isaac Lab releases; write each supported spelling.
    """
    if hasattr(visualizer_cfg, "eye"):
        visualizer_cfg.eye = viewer.eye
    if hasattr(visualizer_cfg, "lookat"):
        visualizer_cfg.lookat = viewer.lookat
    if hasattr(visualizer_cfg, "cam_source"):
        visualizer_cfg.cam_source = "cfg"

    if hasattr(visualizer_cfg, "camera_position"):
        visualizer_cfg.camera_position = viewer.eye
    if hasattr(visualizer_cfg, "camera_target"):
        visualizer_cfg.camera_target = viewer.lookat
    if hasattr(visualizer_cfg, "camera_source"):
        visualizer_cfg.camera_source = "cfg"


def sync_viewer_to_kit_visualizer(env_cfg: "AnyEnvCfg") -> None:
    """Point ``env_cfg.sim``'s Kit visualizer at ``env_cfg.viewer``.

    No-op if Isaac Lab has no Kit visualizer config class. Normalize
    ``sim.visualizer_cfgs``, updating an existing ``"kit"`` entry or appending
    a new :class:`KitVisualizerCfg`.
    """
    try:
        from isaaclab_visualizers.kit import KitVisualizerCfg
    except (ImportError, ModuleNotFoundError):
        try:
            from isaaclab.visualizers.kit_visualizer_cfg import KitVisualizerCfg
        except (ImportError, ModuleNotFoundError):
            return

    visualizer_cfgs = env_cfg.sim.visualizer_cfgs
    if visualizer_cfgs is None:
        visualizer_cfgs = []
    elif not isinstance(visualizer_cfgs, list):
        visualizer_cfgs = [visualizer_cfgs]
    else:
        visualizer_cfgs = list(visualizer_cfgs)

    for visualizer_cfg in visualizer_cfgs:
        if getattr(visualizer_cfg, "visualizer_type", None) == "kit":
            set_kit_visualizer_camera(visualizer_cfg, env_cfg.viewer)
            break
    else:
        visualizer_cfg = KitVisualizerCfg()
        set_kit_visualizer_camera(visualizer_cfg, env_cfg.viewer)
        visualizer_cfgs.append(visualizer_cfg)

    env_cfg.sim.visualizer_cfgs = visualizer_cfgs


__all__ = ["set_kit_visualizer_camera", "sync_viewer_to_kit_visualizer"]
