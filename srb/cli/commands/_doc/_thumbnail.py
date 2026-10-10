"""Render asset and task WebP thumbnails in headless Isaac Sim.

Assets spawn directly; tasks capture their configured viewer pose. Multi-variant
assets render as a grid. Simulator imports stay lazy until after app startup.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Sequence

from ._image import fit_to_canvas

_RENDER_SCALE = 2
_SKIP_PREFIXES: tuple[str, ...] = ()
MAX_GRID_VARIANTS = 8
_GRID_FRIENDLY_COUNTS = (1, 2, 3, 4, 5, 6, 8)
_THUMBNAIL_PROCESS_TIMEOUT_S = 300.0
# Simulated time that lets reset bodies come to rest before capture.
_TASK_SETTLE_S = 1.0


def _atomic_save_webp(img, output_path: Path) -> None:
    """Atomically publish a PIL image, preserving existing target mode; new files use
    the default.

    Explicit WEBP format handles the suffix-free staging file.
    """
    import stat

    from srb.utils.publication import ArtifactPublication, default_new_file_mode

    target_mode = (
        stat.S_IMODE(output_path.stat().st_mode)
        if output_path.exists()
        else default_new_file_mode()
    )

    def write(staging: Path) -> None:
        img.save(staging, format="WEBP", quality=90)
        os.chmod(staging, target_mode)

    ArtifactPublication.file(output_path).publish(write)


@dataclass(frozen=True)
class ThumbnailPlan:
    mode: Literal["asset", "task"]
    output_dir: Path
    task_ids: list[str]


def _grid_layout(n: int) -> tuple[int, int]:
    """Return (cols, rows) for *n* items in a landscape-oriented grid."""
    if n <= 1:
        return (1, 1)
    elif n == 2:
        return (2, 1)
    elif n == 3:
        return (3, 1)
    elif n == 4:
        return (2, 2)
    elif n == 5:
        return (3, 2)
    elif n == 6:
        return (3, 2)
    else:
        return (4, 2)


def _grid_friendly_count(n: int) -> int:
    """Return the largest grid-friendly count <= *n*."""
    for c in reversed(_GRID_FRIENDLY_COUNTS):
        if c <= n:
            return c
    return 1


def _canonical_task_name(task_id: object) -> str:
    value = str(task_id).strip()
    if not value:
        return ""
    return value.rsplit("/", 1)[-1]


def _namespace_task_name(task_id: str) -> str:
    return task_id if "/" in task_id else f"srb/{task_id}"


def _validate_thumbnail_filter_arguments(
    *,
    task: Sequence[str] | str | None,
    asset: str | None,
    asset_type: Sequence[str] | None,
) -> None:
    requested_tasks = (
        [task] if isinstance(task, str) else list(task) if task is not None else []
    )
    has_task_filter = any(_canonical_task_name(value) for value in requested_tasks)
    if has_task_filter and (asset or asset_type):
        raise ValueError(
            "'--task' cannot be combined with '--asset' or '--asset-type'."
        )


def _normalize_task_selection(
    task: Sequence[str] | str | None,
    *,
    cached_task_ids: Sequence[str],
) -> list[str]:
    if task is None:
        return []
    requested = [task] if isinstance(task, str) else list(task)
    if not requested:
        return []

    if any(str(value).upper() == "ALL" or str(value) == "*" for value in requested):
        task_names = sorted(
            {
                task_name
                for cached_id in cached_task_ids
                if (task_name := _canonical_task_name(cached_id))
            }
        )
        if not task_names:
            raise ValueError(
                "No cached SRB tasks are available for '--task ALL'. Run "
                "'srb list env' or refresh the SRB cache first."
            )
    else:
        task_names = []
        seen: set[str] = set()
        for value in requested:
            task_name = _canonical_task_name(value)
            if not task_name or task_name in seen:
                continue
            seen.add(task_name)
            task_names.append(task_name)

    return [_namespace_task_name(task_name) for task_name in task_names]


def _resolve_thumbnail_plan(
    *,
    output_dir: str | None,
    task: Sequence[str] | str | None,
    asset: str | None,
    asset_type: Sequence[str] | None,
    cached_task_ids: Sequence[str],
) -> ThumbnailPlan:
    from srb.utils.path import SRB_GRAPHICS_DIR_ASSETS, SRB_GRAPHICS_DIR_TASKS

    _validate_thumbnail_filter_arguments(
        task=task,
        asset=asset,
        asset_type=asset_type,
    )
    task_ids = _normalize_task_selection(task, cached_task_ids=cached_task_ids)

    if task_ids:
        return ThumbnailPlan(
            mode="task",
            output_dir=Path(output_dir)
            if output_dir is not None
            else SRB_GRAPHICS_DIR_TASKS,
            task_ids=task_ids,
        )

    return ThumbnailPlan(
        mode="asset",
        output_dir=Path(output_dir)
        if output_dir is not None
        else SRB_GRAPHICS_DIR_ASSETS,
        task_ids=[],
    )


def _should_skip(asset_cls: type) -> bool:
    name = asset_cls.name()
    if any(name.startswith(prefix) for prefix in _SKIP_PREFIXES):
        return True
    model_fields = getattr(asset_cls, "model_fields", {})
    field_info = model_fields.get("asset_cfg")
    if field_info is None or field_info.is_required():
        return True
    asset_cfg = field_info.default
    spawner = getattr(asset_cfg, "spawn", None)
    if spawner is None:
        return True
    return False


def _get_variant_spawners(
    spawner,
    *,
    MultiAssetSpawnerCfg,
    MultiShapeSpawnerCfg,
    SimforgeAssetCfg,
    copy_mod,
) -> list | None:
    """Return spawn configs for multi-variant assets, else ``None``.

    Simulator types stay parameters to keep imports lazy.
    """
    if isinstance(spawner, MultiAssetSpawnerCfg):
        sub_cfgs = list(spawner.assets_cfg[:MAX_GRID_VARIANTS])
        if len(sub_cfgs) > 1:
            return sub_cfgs
        return None
    elif isinstance(spawner, MultiShapeSpawnerCfg):
        from srb.core.sim.spawners.shapes.multi_shape.impl import (
            multi_shape_asset_cfgs,
        )

        sub_cfgs = multi_shape_asset_cfgs(spawner)
        if len(sub_cfgs) > 1:
            return sub_cfgs[:MAX_GRID_VARIANTS]
        return None
    elif isinstance(spawner, SimforgeAssetCfg):
        variants = []
        for i in range(MAX_GRID_VARIANTS):
            v = copy_mod.copy(spawner)
            v.seed = i
            v.num_assets = 1
            variants.append(v)
        return variants
    return None


def _compose_grid(images: list, w: int, h: int):
    """Center pre-fitted cells on a transparent RGBA canvas."""
    from PIL import Image

    n = len(images)
    if n == 1:
        return images[0]

    cols, rows = _grid_layout(n)
    cell_w = images[0].width
    cell_h = images[0].height
    grid_w = cell_w * cols
    grid_h = cell_h * rows

    canvas = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    x0 = (w - grid_w) // 2
    y0 = (h - grid_h) // 2
    for idx, img in enumerate(images):
        col = idx % cols
        row = idx // cols
        canvas.alpha_composite(
            img.convert("RGBA"), (x0 + col * cell_w, y0 + row * cell_h)
        )
    return canvas


def _apply_initial_joint_positions(
    stage, prim_path: str, asset_cfg, *, Usd, UsdPhysics, math_mod
) -> bool:
    """Set joint drive targets from ``asset_cfg.init_state.joint_pos``.

    Return whether any target was set; high gains speed convergence.
    """
    init_state = getattr(asset_cfg, "init_state", None)
    if init_state is None:
        return False
    joint_pos = getattr(init_state, "joint_pos", None)
    if not joint_pos:
        return False

    import re as _re

    root_prim = stage.GetPrimAtPath(prim_path)
    if not root_prim.IsValid():
        return False

    matched = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive_type = "angular"
            is_angular = True
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            drive_type = "linear"
            is_angular = False
        elif prim.IsA(UsdPhysics.Joint):
            if prim.HasAPI(UsdPhysics.DriveAPI, instanceName="linear"):
                drive_type = "linear"
                is_angular = False
            else:
                drive_type = "angular"
                is_angular = True
        else:
            continue

        joint_name = prim.GetName()
        for pattern, value in joint_pos.items():
            if _re.fullmatch(pattern, joint_name):
                drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                target = math_mod.degrees(value) if is_angular else value
                drive.CreateTargetPositionAttr().Set(target)
                drive.CreateStiffnessAttr().Set(1e8)
                drive.CreateDampingAttr().Set(1e6)
                matched += 1
                break

    return matched > 0


def _setup_scene(stage, *, Gf, UsdGeom, UsdLux, UsdPhysics) -> None:
    """Create the lights and physics scene used for rendering."""
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr().Set(1000.0)
    dome.CreateColorAttr().Set(Gf.Vec3f(0.125, 0.125, 0.125))

    distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant.CreateIntensityAttr().Set(3000.0)
    distant.CreateAngleAttr().Set(0.53)
    distant.CreateColorTemperatureAttr().Set(6500.0)
    distant.CreateEnableColorTemperatureAttr().Set(True)
    xformable = UsdGeom.Xformable(distant.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 30.0, 0.0))

    # Joint drives require a physics scene.
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(0.0)


def _set_camera_transform(
    stage, cam, eye, target, up=(0.0, 0.0, 1.0), *, Gf, UsdGeom
) -> None:
    xformable = UsdGeom.Xformable(cam.GetPrim())
    xformable.ClearXformOpOrder()
    xf_op = xformable.AddTransformOp()
    look_at = Gf.Matrix4d()
    look_at.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    xf_op.Set(look_at.GetInverse())


def _compute_camera_from_bbox(stage, prim_path: str, *, Usd, UsdGeom, Gf, math_mod):
    """Compute a camera pose from the bounding box of the prim at *prim_path*."""
    from srb.core.asset.common.thumbnail import ThumbnailCameraPose

    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    bbox = bbox_cache.ComputeWorldBound(prim)
    bbox_range = bbox.ComputeAlignedRange()

    center = bbox_range.GetMidpoint()
    size = bbox_range.GetSize()
    max_dim = max(size[0], size[1], size[2], 0.01)

    # Scale distance so the object fills ~40% of the frame width.
    # With focal_length=35mm, aperture=36mm the half-FOV is ~27.2°.
    # target_fill=0.4 → distance = max_dim / (2 * tan(27.2°) * 0.4)
    half_fov = math_mod.atan(36.0 / (2.0 * 35.0))  # ~0.474 rad
    target_fill = 0.4
    distance = max(max_dim / (2.0 * math_mod.tan(half_fov) * target_fill), 0.15)
    elevation = math_mod.radians(25)
    azimuth = math_mod.radians(45)

    eye_x = center[0] + distance * math_mod.cos(elevation) * math_mod.cos(azimuth)
    eye_y = center[1] + distance * math_mod.cos(elevation) * math_mod.sin(azimuth)
    eye_z = center[2] + distance * math_mod.sin(elevation)

    return ThumbnailCameraPose(
        eye=(eye_x, eye_y, eye_z),
        lookat=(
            float(center[0]),
            float(center[1]),
            float(center[2]),
        ),
    )


def _get_camera_pose(asset_cls, stage, prim_path, *, Usd, UsdGeom, Gf, math_mod):
    pose = getattr(asset_cls, "THUMBNAIL_CAMERA_POSE", None)
    if pose is not None:
        return pose
    return _compute_camera_from_bbox(
        stage, prim_path, Usd=Usd, UsdGeom=UsdGeom, Gf=Gf, math_mod=math_mod
    )


def _render_single_variant(
    variant_spawner,
    stage,
    cam,
    actual_prim_path: str,
    asset_cls,
    asset_cfg,
    w: int,
    h: int,
    rgb_annot,
    depth_annot,
    *,
    simulation_app,
    timeline,
    logging,
    np,
    Usd,
    UsdGeom,
    UsdPhysics,
    Gf,
    math_mod,
    ThumbnailCameraPose,
    Image,
):
    """Spawn, render, and clean up one variant; return ``None`` on failure."""
    name = asset_cls.name()
    try:
        variant_spawner.func(
            prim_path=actual_prim_path,
            cfg=variant_spawner,
            translation=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
    except Exception as e:
        logging.warning(f"Failed to spawn variant for {name}: {e}")
        stage.RemovePrim(actual_prim_path)
        simulation_app.update()
        return None

    try:
        # Apply initial joint configuration
        has_joints = _apply_initial_joint_positions(
            stage,
            actual_prim_path,
            asset_cfg,
            Usd=Usd,
            UsdPhysics=UsdPhysics,
            math_mod=math_mod,
        )
        if has_joints:
            timeline.play()
            try:
                for _ in range(60):
                    simulation_app.update()
            finally:
                timeline.pause()

        simulation_app.update()

        try:
            pose = _get_camera_pose(
                asset_cls,
                stage,
                actual_prim_path,
                Usd=Usd,
                UsdGeom=UsdGeom,
                Gf=Gf,
                math_mod=math_mod,
            )
        except Exception:
            pose = getattr(
                asset_cls, "THUMBNAIL_CAMERA_POSE", None
            ) or ThumbnailCameraPose(eye=(3.0, 3.0, 2.0))

        _set_camera_transform(
            stage, cam, pose.eye, pose.lookat, pose.up, Gf=Gf, UsdGeom=UsdGeom
        )

        # Let the path tracer accumulate samples
        for _ in range(2):
            simulation_app.update()

        # --- RGB pass ---
        data = rgb_annot.get_data()
        result = None
        if data is not None and data.size > 0:
            img_array = np.asarray(data)
            rgb = img_array[:, :, :3] if img_array.ndim == 3 else img_array
            rgb = rgb.astype(np.uint8)

            # --- Depth-based alpha extraction ---
            depth_data = depth_annot.get_data()
            if depth_data is not None and depth_data.size > 0:
                depth = np.asarray(depth_data).astype(np.float32)
                foreground = (depth > 0.001) & (depth < 99000.0)
                alpha = np.where(foreground, 255, 0).astype(np.uint8)
            else:
                alpha = np.full((rgb.shape[0], rgb.shape[1]), 255, dtype=np.uint8)

            rgba = np.dstack([rgb, alpha])
            result = Image.fromarray(rgba, mode="RGBA")

        return result
    finally:
        stage.RemovePrim(actual_prim_path)
        simulation_app.update()


def _render_asset(
    asset_cls,
    stage,
    cam,
    out_dir,
    w: int,
    h: int,
    rgb_annot,
    depth_annot,
    *,
    simulation_app,
    timeline,
    logging,
    np,
    Usd,
    UsdGeom,
    UsdPhysics,
    Gf,
    math_mod,
    ThumbnailCameraPose,
    Image,
    MultiAssetSpawnerCfg,
    MultiShapeSpawnerCfg,
    SimforgeAssetCfg,
    copy_mod,
) -> bool:
    """Render one asset or a variant grid and save it as WebP."""
    name = asset_cls.name()

    model_fields = getattr(asset_cls, "model_fields", {})
    field_info = model_fields.get("asset_cfg")
    if field_info is None or field_info.is_required():
        logging.warning(f"SKIP: No asset_cfg for {name}")
        return False
    asset_cfg = field_info.default

    spawner = getattr(asset_cfg, "spawn", None)
    if spawner is None:
        logging.warning(f"SKIP: No spawner for {name}")
        return False

    original_prim_path = asset_cfg.prim_path
    actual_prim_path = original_prim_path.replace("{ENV_REGEX_NS}", "/World/assets")

    # Shared kwargs for _render_single_variant
    _render_kw = dict(
        simulation_app=simulation_app,
        timeline=timeline,
        logging=logging,
        np=np,
        Usd=Usd,
        UsdGeom=UsdGeom,
        UsdPhysics=UsdPhysics,
        Gf=Gf,
        math_mod=math_mod,
        ThumbnailCameraPose=ThumbnailCameraPose,
        Image=Image,
    )

    variant_spawners = _get_variant_spawners(
        spawner,
        MultiAssetSpawnerCfg=MultiAssetSpawnerCfg,
        MultiShapeSpawnerCfg=MultiShapeSpawnerCfg,
        SimforgeAssetCfg=SimforgeAssetCfg,
        copy_mod=copy_mod,
    )

    if variant_spawners is None:
        # ---- Single-variant path ----
        img = _render_single_variant(
            spawner,
            stage,
            cam,
            actual_prim_path,
            asset_cls,
            asset_cfg,
            w,
            h,
            rgb_annot,
            depth_annot,
            **_render_kw,
        )
        if img is not None:
            img = fit_to_canvas(img, w, h)
            output_path = out_dir / f"{name}.webp"
            _atomic_save_webp(img, output_path)
            logging.info(f"Saved: {output_path}")
            return True
        else:
            logging.warning(f"No image data for {name}")
            return False
    else:
        # ---- Multi-variant path: render grid ----
        variant_spawners = variant_spawners[
            : _grid_friendly_count(len(variant_spawners))
        ]
        _g_cols, _g_rows = _grid_layout(len(variant_spawners))
        cell_w = w // _g_cols
        cell_h = h // _g_rows
        images = []
        for i, vs in enumerate(variant_spawners):
            logging.debug(
                f"  Rendering variant {i + 1}/{len(variant_spawners)} for {name}"
            )
            img = _render_single_variant(
                vs,
                stage,
                cam,
                actual_prim_path,
                asset_cls,
                asset_cfg,
                w,
                h,
                rgb_annot,
                depth_annot,
                **_render_kw,
            )
            if img is not None:
                images.append(fit_to_canvas(img, cell_w, cell_h))

        if not images:
            logging.warning(f"No variants rendered for {name}")
            return False

        grid = _compose_grid(images, w, h)
        output_path = out_dir / f"{name}.webp"
        _atomic_save_webp(grid, output_path)
        logging.info(f"Saved grid ({len(images)} variants): {output_path}")
        return True


def _task_output_path(out_dir: Path, task_id: str) -> Path:
    return out_dir / f"{_canonical_task_name(task_id)}.webp"


def _task_thumbnail_render_succeeded(ok: bool, output_path: Path) -> bool:
    return ok and output_path.exists()


def _task_hydra_dir(task_id: str) -> Path:
    from srb.utils.path import SRB_CACHE_PATH

    return SRB_CACHE_PATH / "doc_thumbnail" / _canonical_task_name(task_id)


def _prepare_thumbnail_environment() -> None:
    """Enable baking; untextured thumbnails are unusable.

    Export the override so per-task child processes inherit it.
    """

    import os

    os.environ["SF_BAKER"] = "1"


def _prepare_task_thumbnail_environment() -> None:
    import os

    os.environ.setdefault("SRB_TERRAIN_AUTO_FETCH", "0")


def _simulation_app_launch_args(
    *,
    width: int,
    height: int,
) -> tuple[dict[str, object], str]:
    from srb.utils.path import SRB_APPS_DIR

    return (
        {
            "headless": True,
            "width": width * _RENDER_SCALE,
            "height": height * _RENDER_SCALE,
            "renderer": "PathTracing",
        },
        SRB_APPS_DIR.joinpath("srb.headless.rendering.kit").as_posix(),
    )


def _enable_isaaclab_camera_rendering() -> None:
    import carb.settings as carb_settings

    settings = carb_settings.get_settings()
    try:
        from isaaclab.app.settings_manager import get_settings_manager
    except ModuleNotFoundError:
        pass
    else:
        # The thumbnail renderer starts SimulationApp itself, so no Kit
        # launcher has backed Isaac Lab's settings with carb; without this the
        # values below never reach Isaac Lab.
        get_settings_manager().set_backend(settings)

    for path, value in (
        ("/isaaclab/cameras_enabled", True),
        ("/isaaclab/render/offscreen", True),
        ("/isaaclab/render/active_viewport", False),
        ("/isaaclab/render/rtx_sensors", False),
        ("/physics/fabricUpdateTransformations", True),
        ("/app/player/useFixedTimeStepping", False),
    ):
        settings.set_bool(path, value)


def _raise_if_task_thumbnail_failures(
    *,
    successes: int,
    skipped: int,
    failures: int,
    allow_failures: bool,
) -> None:
    if failures <= 0:
        return
    rendered = f"{successes} rendered, {skipped} skipped, {failures} failed"
    if allow_failures:
        return
    noun = "render" if failures == 1 else "renders"
    raise RuntimeError(f"{failures} task thumbnail {noun} failed ({rendered})")


@dataclass(frozen=True)
class TaskThumbnailBatchSummary:
    successes: int
    skipped: int
    failures: int


def _task_thumbnail_batch_child_command(
    task_id: str,
    *,
    output_dir: Path,
    width: int,
    height: int,
    force: bool,
) -> list[str]:
    import sys

    command = [
        sys.executable,
        "-m",
        "srb",
        "doc",
        "thumbnail",
        "--task",
        task_id,
        "--width",
        str(width),
        "--height",
        str(height),
        "--output-dir",
        output_dir.as_posix(),
    ]
    if force:
        command.append("--force")
    return command


def _execute_task_thumbnail_batch_subprocesses(
    plan: ThumbnailPlan,
    *,
    width: int,
    height: int,
    force: bool,
    allow_failures: bool,
    run=None,
    refresh_catalog: bool = True,
) -> TaskThumbnailBatchSummary:
    import subprocess

    from srb.utils import logging
    from srb.utils.process_supervisor import (
        ProcessCleanupError,
        ProcessKind,
        ProcessStatus,
        run_owned,
    )

    if run is None:

        def run(command, *, check=False):
            result = run_owned(
                command,
                kind=ProcessKind.FINITE,
                timeout=_THUMBNAIL_PROCESS_TIMEOUT_S,
            )
            if not result.cleanup_complete:
                raise ProcessCleanupError(result)
            if result.status is ProcessStatus.TIMED_OUT:
                raise subprocess.TimeoutExpired(command, _THUMBNAIL_PROCESS_TIMEOUT_S)
            if check and result.returncode:
                raise subprocess.CalledProcessError(result.returncode, command)
            return result

    successes = 0
    skipped = 0
    failures = 0

    logging.info(
        f"Rendering thumbnails for {len(plan.task_ids)} tasks in isolated processes..."
    )
    for i, task_id in enumerate(plan.task_ids):
        output_path = _task_output_path(plan.output_dir, task_id)
        if not force and output_path.exists():
            logging.debug(
                f"[{i + 1}/{len(plan.task_ids)}] {task_id} (skipped, already exists)"
            )
            skipped += 1
            continue

        logging.info(f"[{i + 1}/{len(plan.task_ids)}] {task_id}")
        command = _task_thumbnail_batch_child_command(
            task_id,
            output_dir=plan.output_dir,
            width=width,
            height=height,
            force=force,
        )
        try:
            result = run(command, check=False)
        except subprocess.TimeoutExpired:
            # A hung task is one failed render; --allow-failures decides below.
            # Cleanup failures still propagate because orphaned children remain.
            logging.error(
                f"{task_id} timed out after {_THUMBNAIL_PROCESS_TIMEOUT_S:.0f} s"
            )
            failures += 1
            continue
        if getattr(result, "returncode", 1) == 0 and output_path.exists():
            successes += 1
        else:
            failures += 1

    logging.info(
        f"Rendering complete: {successes} rendered, {skipped} skipped, {failures} failed."
    )
    if failures and allow_failures:
        logging.warning(
            "Continuing because --allow-failures was set; inspect the log for failed tasks."
        )

    if refresh_catalog and (successes or skipped):
        try:
            from srb.cli.gui_catalog import update_offline_srb_gui_catalog_cache

            update_offline_srb_gui_catalog_cache()
        except Exception as e:
            logging.warning(f"Could not refresh Rust GUI catalog cache: {e}")

    _raise_if_task_thumbnail_failures(
        successes=successes,
        skipped=skipped,
        failures=failures,
        allow_failures=allow_failures,
    )
    return TaskThumbnailBatchSummary(
        successes=successes,
        skipped=skipped,
        failures=failures,
    )


def _clear_hydra_state() -> None:
    try:
        from hydra.core.global_hydra import GlobalHydra
    except Exception:
        return
    try:
        GlobalHydra.instance().clear()
    except Exception:
        return


def _vec3(value, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    if value is None:
        return fallback
    try:
        return float(value[0]), float(value[1]), float(value[2])
    except (IndexError, KeyError, TypeError, ValueError):
        return fallback


def _task_camera_pose(
    env,
    env_cfg,
    stage,
    *,
    Usd,
    UsdGeom,
    Gf,
    math_mod,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    from srb.core.scene_introspection import viewer_pose_w

    viewer = getattr(env_cfg, "viewer", None)
    # ``origin_type`` decides what eye/lookat are relative to; resolving only
    # the ``"env"`` case left the two ``origin_type="asset_root"`` tasks framed
    # on the world origin instead of on their satellite.
    eye, lookat = viewer_pose_w(
        env,
        viewer,
        default_eye=(3.5, 3.5, 2.5),
        default_lookat=(0.0, 0.0, 0.0),
    )

    if viewer is not None:
        return eye, lookat

    for prim_path in ("/World/envs/env_0", "/World"):
        try:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                pose = _compute_camera_from_bbox(
                    stage,
                    prim_path,
                    Usd=Usd,
                    UsdGeom=UsdGeom,
                    Gf=Gf,
                    math_mod=math_mod,
                )
                return _vec3(pose.eye, eye), _vec3(pose.lookat, lookat)
        except Exception as exc:
            from srb.utils import logging

            logging.debug("Could not infer task camera from %s: %s", prim_path, exc)
            continue
    return eye, lookat


def _settle_physics(env, duration_s: float) -> None:
    dt = float(env.physics_dt)

    def substep() -> None:
        # Explicit actuators apply torque only through the scene write.
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(dt)

    for _ in range(max(1, round(duration_s / dt))):
        substep()
    # Sync task-driven bodies, such as a mounted payload; one step writes them to USD.
    env.advance_step_state()
    substep()


def _capture_task_image(
    rgb_annot,
    *,
    simulation_app,
    np,
    Image,
    width: int,
    height: int,
):
    for _ in range(16):
        simulation_app.update()
    data = rgb_annot.get_data()
    if data is None or data.size == 0:
        return None
    img_array = np.asarray(data)
    rgb = img_array[:, :, :3] if img_array.ndim == 3 else img_array
    rgb = rgb.astype(np.uint8)
    return Image.fromarray(rgb, mode="RGB").resize((width, height), Image.LANCZOS)


def _render_task_thumbnail(
    task_id: str,
    stage,
    cam,
    out_dir: Path,
    w: int,
    h: int,
    rgb_annot,
    *,
    simulation_app,
    logging,
    np,
    Usd,
    UsdGeom,
    Gf,
    math_mod,
    Image,
) -> bool:
    import os
    import sys

    from srb.hydra.sim import hydra_task_config

    rendered = False
    old_argv = list(sys.argv)
    hydra_dir = _task_hydra_dir(task_id)
    hydra_dir.mkdir(parents=True, exist_ok=True)
    sys.argv = [
        old_argv[0],
        "env.scene.num_envs=1",
        "env.num_envs=1",
        # Headless capture renders USD, which PhysX updates only without Fabric.
        "env.sim.use_fabric=false",
        f"hydra.run.dir={hydra_dir.as_posix()}",
        "hydra.output_subdir=null",
        "hydra.job.chdir=false",
    ]
    old_cwd = os.getcwd()

    @hydra_task_config(task_name=task_id)
    def _render(env_cfg, _agent_cfg=None) -> None:
        nonlocal rendered
        import gymnasium

        env = gymnasium.make(id=task_id, cfg=env_cfg, render_mode=None)
        try:
            env.reset()
            # Settle without actions so every robot holds its reset configuration.
            _settle_physics(env.unwrapped, _TASK_SETTLE_S)
            # App updates step a playing timeline without actuator writes.
            env.unwrapped.sim.pause()
            eye, lookat = _task_camera_pose(
                env,
                env_cfg,
                stage,
                Usd=Usd,
                UsdGeom=UsdGeom,
                Gf=Gf,
                math_mod=math_mod,
            )
            _set_camera_transform(stage, cam, eye, lookat, Gf=Gf, UsdGeom=UsdGeom)
            img = _capture_task_image(
                rgb_annot,
                simulation_app=simulation_app,
                np=np,
                Image=Image,
                width=w,
                height=h,
            )
            if img is None:
                logging.warning(f"No image data for task {task_id}")
                return
            output_path = _task_output_path(out_dir, task_id)
            _atomic_save_webp(img, output_path)
            logging.info(f"Saved: {output_path}")
            rendered = True
        finally:
            env.close()

    try:
        _clear_hydra_state()
        _render()
    finally:
        sys.argv = old_argv
        _clear_hydra_state()
        os.chdir(old_cwd)

    return rendered


def _render_asset_thumbnails(
    all_assets,
    *,
    output_dir: Path,
    width: int,
    height: int,
    force: bool,
    rgb_annot,
    depth_annot,
    render_product,
    timeline,
    logging,
    render_asset_kwargs: dict,
) -> None:
    """Render assets and fail the process on errors."""
    successes = 0
    skipped = 0
    failures = 0

    logging.info(f"Rendering thumbnails for {len(all_assets)} assets...")
    for i, asset_cls in enumerate(all_assets):
        name = asset_cls.name()
        output_path = output_dir / f"{name}.webp"
        if not force and output_path.exists():
            logging.debug(
                f"[{i + 1}/{len(all_assets)}] {name} (skipped, already exists)"
            )
            skipped += 1
            continue
        logging.info(f"[{i + 1}/{len(all_assets)}] {name}")
        try:
            if _render_asset(
                asset_cls,
                render_asset_kwargs["stage"],
                render_asset_kwargs["cam"],
                output_dir,
                width,
                height,
                rgb_annot,
                depth_annot,
                **render_asset_kwargs["kwargs"],
            ):
                successes += 1
            else:
                failures += 1
        except Exception as exc:
            logging.error(str(exc))
            failures += 1

    logging.info(
        f"Rendering complete: {successes} rendered, {skipped} skipped, {failures} failed."
    )

    try:
        rgb_annot.detach([render_product])
        depth_annot.detach([render_product])
        render_product.destroy()
    except Exception as exc:
        logging.warning(f"Could not clean up asset render product: {exc}")

    from srb.cli.commands._hydra_exit import exit_nonzero_if_failed

    # A failing stop must not skip the failure exit: Kit's close exits 0.
    try:
        timeline.stop()
    finally:
        exit_nonzero_if_failed(failures > 0)
    logging.info("Done.")


def execute_thumbnail(
    width: int = 1024,
    height: int = 768,
    output_dir: str | None = None,
    asset: str | None = None,
    task: list[str] | None = None,
    asset_type: list[str] | None = None,
    force: bool = False,
    allow_failures: bool = False,
    **_kwargs,
) -> None:
    """Render asset or task thumbnails; requires Isaac Sim."""
    _validate_thumbnail_filter_arguments(
        task=task,
        asset=asset,
        asset_type=asset_type,
    )

    from srb.utils.cache import read_offline_srb_env_cache

    plan = _resolve_thumbnail_plan(
        output_dir=output_dir,
        task=task,
        asset=asset,
        asset_type=asset_type,
        cached_task_ids=read_offline_srb_env_cache() or [],
    )

    from srb.cli.capabilities import require_isaac_sim

    require_isaac_sim()
    import copy
    import math
    import os
    import sys

    _prepare_thumbnail_environment()
    if plan.mode == "task":
        _prepare_task_thumbnail_environment()

    resolved_output_dir = plan.output_dir
    resolved_output_dir.mkdir(parents=True, exist_ok=True)
    if plan.mode == "task" and len(plan.task_ids) > 1:
        _execute_task_thumbnail_batch_subprocesses(
            plan,
            width=width,
            height=height,
            force=force,
            allow_failures=allow_failures,
        )
        return

    # Launch Isaac Sim before importing omni/pxr.
    from srb.core.app import start_simulation_app
    from srb.utils import logging

    launch_config, experience = _simulation_app_launch_args(
        width=width,
        height=height,
    )
    simulation_app = start_simulation_app(launch_config, experience=experience)

    app_closed = False
    succeeded = False

    def _close_simulation_app() -> None:
        nonlocal app_closed
        if app_closed:
            return
        app_closed = True
        simulation_app.close()

    try:
        import srb.core.sim  # noqa: F401 (Ensures compat shim loads before IsaacLab/Simforge)

        if plan.mode == "task":
            _enable_isaaclab_camera_rendering()

        import carb.settings as _carb_settings_mod
        import numpy as np
        import omni.replicator.core as rep
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

        _cs = _carb_settings_mod.get_settings()
        _cs.set("/rtx/pathtracing/spp", 128)
        _cs.set("/rtx/pathtracing/totalSpp", 128)
        _cs.set("/rtx/pathtracing/maxBounces", 16)
        _cs.set("/rtx/pathtracing/optixDenoiser/enabled", True)

        from omni.timeline import get_timeline_interface

        stage = simulation_app.context.get_stage()
        timeline = get_timeline_interface()

        cam = UsdGeom.Camera.Define(stage, "/World/ThumbnailCamera")
        cam.CreateHorizontalApertureAttr().Set(36.0)
        cam.CreateFocalLengthAttr().Set(35.0)
        cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 100000.0))

        # Create a single render product + annotators (reused for all assets)
        rp = rep.create.render_product(
            str(cam.GetPath()), (width * _RENDER_SCALE, height * _RENDER_SCALE)
        )
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])
        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        depth_annot.attach([rp])

        # Warm up the renderer
        simulation_app.update()

        if plan.mode == "task":
            # Trigger task registration only after Isaac Sim is live.
            from srb.cli.gui_catalog import update_offline_srb_gui_catalog_cache
            from srb.tasks import register_all

            register_all()

            logging.info(f"Rendering thumbnails for {len(plan.task_ids)} tasks...")
            successes = 0
            skipped = 0
            failures = 0
            for i, task_id in enumerate(plan.task_ids):
                output_path = _task_output_path(resolved_output_dir, task_id)
                if not force and output_path.exists():
                    logging.debug(
                        f"[{i + 1}/{len(plan.task_ids)}] {task_id} (skipped, already exists)"
                    )
                    skipped += 1
                    continue
                logging.info(f"[{i + 1}/{len(plan.task_ids)}] {task_id}")
                try:
                    ok = _render_task_thumbnail(
                        task_id,
                        stage,
                        cam,
                        resolved_output_dir,
                        width,
                        height,
                        rgb_annot,
                        simulation_app=simulation_app,
                        logging=logging,
                        np=np,
                        Usd=Usd,
                        UsdGeom=UsdGeom,
                        Gf=Gf,
                        math_mod=math,
                        Image=Image,
                    )
                    if _task_thumbnail_render_succeeded(ok, output_path):
                        successes += 1
                    else:
                        if ok:
                            logging.error(
                                f"ERROR: {task_id} did not create expected thumbnail: {output_path}"
                            )
                        failures += 1
                except Exception as e:
                    logging.error(str(e))
                    if os.environ.get("HYDRA_FULL_ERROR"):
                        import traceback

                        logging.error(traceback.format_exc())
                    failures += 1

            logging.info(
                f"Rendering complete: {successes} rendered, {skipped} skipped, {failures} failed."
            )
            if failures and allow_failures:
                logging.warning(
                    "Continuing because --allow-failures was set; inspect the log for failed tasks."
                )
            try:
                update_offline_srb_gui_catalog_cache()
            except Exception as e:
                logging.warning(f"Could not refresh Rust GUI catalog cache: {e}")
            try:
                rgb_annot.detach([rp])
                depth_annot.detach([rp])
                rp.destroy()
            except Exception as exc:
                logging.warning(f"Could not clean up task render product: {exc}")
            # Raise before closing: Kit's close ends the process with status 0.
            _raise_if_task_thumbnail_failures(
                successes=successes,
                skipped=skipped,
                failures=failures,
                allow_failures=allow_failures,
            )
            succeeded = True
            _close_simulation_app()
            logging.info("Done.")
            return

        from isaaclab.sim.spawners.wrappers import MultiAssetSpawnerCfg
        from simforge.integrations.isaaclab.spawner import SimforgeAssetCfg

        # Trigger asset auto-registration only for asset thumbnails.
        import srb.assets  # noqa: F401
        from srb.core.asset.common.thumbnail import ThumbnailCameraPose
        from srb.core.asset.object.object import ObjectRegistry
        from srb.core.asset.robot.robot import RobotRegistry
        from srb.core.asset.scenery.scenery import SceneryRegistry
        from srb.core.sim import MultiShapeSpawnerCfg, use_stage

        # Collect assets from selected registries
        registries: list = []
        types_to_render = (
            set(asset_type) if asset_type else {"robot", "object", "scenery"}
        )
        if "robot" in types_to_render:
            registries.append(RobotRegistry)
        if "object" in types_to_render:
            registries.append(ObjectRegistry)
        if "scenery" in types_to_render:
            registries.append(SceneryRegistry)

        all_assets: list[type] = [
            cls
            for registry in registries
            for cls in registry.values_inner()
            if not _should_skip(cls)
        ]

        if asset:
            all_assets = [cls for cls in all_assets if cls.name() == asset]
            if not all_assets:
                logging.critical(f"Asset '{asset}' not found in selected registries.")
                sys.exit(1)

        # Set up the scene once
        _setup_scene(
            stage, Gf=Gf, UsdGeom=UsdGeom, UsdLux=UsdLux, UsdPhysics=UsdPhysics
        )
        for _ in range(3):
            simulation_app.update()

        # Shared kwargs for _render_asset
        _render_asset_kw = dict(
            simulation_app=simulation_app,
            timeline=timeline,
            logging=logging,
            np=np,
            Usd=Usd,
            UsdGeom=UsdGeom,
            UsdPhysics=UsdPhysics,
            Gf=Gf,
            math_mod=math,
            ThumbnailCameraPose=ThumbnailCameraPose,
            Image=Image,
            MultiAssetSpawnerCfg=MultiAssetSpawnerCfg,
            MultiShapeSpawnerCfg=MultiShapeSpawnerCfg,
            SimforgeAssetCfg=SimforgeAssetCfg,
            copy_mod=copy,
        )

        # Isaac Lab spawners resolve the stage from a thread-local context.
        with use_stage(stage):
            _render_asset_thumbnails(
                all_assets,
                output_dir=resolved_output_dir,
                width=width,
                height=height,
                force=force,
                rgb_annot=rgb_annot,
                depth_annot=depth_annot,
                render_product=rp,
                timeline=timeline,
                logging=logging,
                render_asset_kwargs={
                    "stage": stage,
                    "cam": cam,
                    "kwargs": _render_asset_kw,
                },
            )
        succeeded = True
        _close_simulation_app()
    finally:
        if not succeeded:
            # Kit's close ends the process with status 0, so a failure must
            # leave through ``os._exit(1)`` before the app is closed below.
            from srb.cli.commands._hydra_exit import exit_nonzero_if_failed

            logging.error(
                f"Thumbnail rendering failed: {sys.exc_info()[1]!r}",
                exc_info=sys.exc_info()[0] is not None,
            )
            exit_nonzero_if_failed(True)
        # Outer app ownership: the simulator closes on every exit path,
        # including asset-lookup failures and render exceptions.
        _close_simulation_app()
