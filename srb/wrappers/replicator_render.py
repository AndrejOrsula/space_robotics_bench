"""Provide Replicator ``rgb_array`` frames for Isaac Sim 6 video capture."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import gymnasium
import numpy as np

if TYPE_CHECKING:
    from srb._typing import AnyEnv

#: Distinct from the snapshot camera so a snapshot can never re-frame a recording.
_CAMERA_STAGE_PATH = "/World/VideoCaptureCamera"


class ReplicatorRenderShim(gymnasium.Wrapper):
    """Render ``rgb_array`` frames through a Replicator annotator."""

    def __init__(self, env: "AnyEnv", *, warmup_frames: int = 16):
        super().__init__(env)
        self._warmup_frames = max(int(warmup_frames), 1)
        self._annotator: Any = None
        self._render_product: Any = None
        self._camera_xform_op: Any = None
        self._resolution: tuple[int, int] = (1280, 720)
        # RecordVideo uses one frame per policy step.
        step_dt = getattr(env.unwrapped, "step_dt", None)
        self.metadata = {
            **getattr(env, "metadata", {}),
            "render_fps": int(round(1.0 / step_dt)) if step_dt else 30,
        }

    @property
    def render_mode(self) -> str:
        return "rgb_array"

    def render(self) -> np.ndarray:
        import omni.kit.app

        from srb.utils.isaacsim import update_app_without_physics

        first = self._annotator is None
        if first:
            self._create_capture()
        self._update_camera_pose()
        update_app_without_physics(
            omni.kit.app.get_app(), self._warmup_frames if first else 1
        )

        frame = np.asarray(self._annotator.get_data())
        width, height = self._resolution
        if frame.ndim != 3 or frame.shape[:2] != (height, width) or frame.shape[2] < 3:
            raise RuntimeError(
                f"RGB annotator returned shape {frame.shape}, expected "
                f"({height}, {width}, 3+); video capture needs camera rendering "
                "(--enable_cameras)"
            )
        return np.ascontiguousarray(frame[..., :3], dtype=np.uint8)

    def close(self):
        annotator, self._annotator = self._annotator, None
        render_product, self._render_product = self._render_product, None
        self._camera_xform_op = None
        if annotator is not None:
            # Detach may fail in Replicator; still close the environment.
            try:
                annotator.detach([render_product])
            except Exception:  # noqa: BLE001
                pass
        if render_product is not None:
            try:
                render_product.destroy()
            except Exception:  # noqa: BLE001
                pass
        return self.env.close()

    def _create_capture(self) -> None:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, UsdGeom

        viewer = getattr(getattr(self.env.unwrapped, "cfg", None), "viewer", None)
        resolution = getattr(viewer, "resolution", None) or self._resolution
        self._resolution = (int(resolution[0]), int(resolution[1]))

        stage = omni.usd.get_context().get_stage()
        camera = UsdGeom.Camera.Define(stage, _CAMERA_STAGE_PATH)
        camera.CreateHorizontalApertureAttr().Set(36.0)
        camera.CreateFocalLengthAttr().Set(35.0)
        camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))
        xformable = UsdGeom.Xformable(camera.GetPrim())
        xformable.ClearXformOpOrder()
        self._camera_xform_op = xformable.AddTransformOp()

        self._render_product = rep.create.render_product(
            camera.GetPath().pathString, self._resolution
        )
        self._annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self._annotator.attach([self._render_product])

    def _update_camera_pose(self) -> None:
        """Aim the camera at the current viewer pose."""
        from pxr import Gf

        from srb.core.scene_introspection import viewer_pose_w
        from srb.utils.spatial import pick_camera_up

        unwrapped = self.env.unwrapped
        viewer = getattr(getattr(unwrapped, "cfg", None), "viewer", None)
        eye, lookat = viewer_pose_w(unwrapped, viewer)
        up = pick_camera_up(eye, lookat, preferred_up=(0.0, 0.0, 1.0))
        look_at = Gf.Matrix4d()
        look_at.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*lookat), Gf.Vec3d(*up))
        self._camera_xform_op.Set(look_at.GetInverse())


__all__ = ["ReplicatorRenderShim"]
