"""Markdown rendering functions for docgen output."""

from __future__ import annotations

import math
from pathlib import Path

from srb.cli.commands._doc._introspect import (
    ObjectInfo,
    RobotInfo,
    SceneryInfo,
    TaskInfo,
)
from srb.cli.commands._doc._toml import AssetMeta, SidecarData, TaskMeta

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_IMAGE_REL_PATH = "../../_graphics/assets"
_TASK_IMAGE_REL_PATH = "../_graphics/tasks"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _has_thumbnail(name: str, graphics_dir: Path) -> bool:
    """Return whether a thumbnail exists."""
    return (graphics_dir / f"{name}.webp").is_file()


def _youtube_iframe(video_id: str) -> str:
    """Return a YouTube iframe embed."""
    return (
        f'<iframe style="width:100%;aspect-ratio:16/9" '
        f'src="https://www.youtube.com/embed/{video_id}'
        f'?si=docgen&mute=1&autoplay=1&loop=1&playlist={video_id}" '
        f'frameborder="0" allow="accelerometer; autoplay; clipboard-write; '
        f'encrypted-media; gyroscope; picture-in-picture; web-share" '
        f'referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>'
        "</iframe>"
    )


def _fmt(val: float | None, unit: str = "", precision: int = 6) -> str:
    """Format a number, using an en dash for `None`."""
    if val is None:
        return "–"
    # Avoid displaying negative zero.
    if val == 0.0:
        val = 0.0
    # Preserve small nonzero values when rounding.
    if val != 0.0 and math.isfinite(val):
        precision = max(precision, 1 - math.floor(math.log10(abs(val))))
    # Remove trailing zeros.
    s = f"{val:.{precision}f}".rstrip("0").rstrip(".")
    if unit:
        return f"{s} {unit}"
    return s


def _fmt_actuator(val: float | tuple[float, float] | None, unit: str = "") -> str:
    """Format an actuator value; a per-joint ``(min, max)`` span renders as a range."""
    if isinstance(val, tuple):
        span = f"{_fmt(val[0])}–{_fmt(val[1])}"
        return f"{span} {unit}" if unit else span
    return _fmt(val, unit)


def _rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def _specs_row(label: str, value: str) -> str:
    return f"| **{label}** | {value} |"


# ---------------------------------------------------------------------------
# Robot entry
# ---------------------------------------------------------------------------


def render_robot_entry(
    robot: RobotInfo,
    graphics_dir: Path,
    sidecar: AssetMeta,
) -> str:
    """Render a single robot entry as markdown."""
    lines: list[str] = []

    lines.append(f'<a id="{robot.anchor}"></a>')
    lines.append("")
    lines.append(f"#### {robot.display_name} (`{robot.name}`)")
    lines.append("")

    tagline = sidecar.tagline
    if tagline:
        lines.append(f"> {tagline}")
        lines.append("")

    if _has_thumbnail(robot.name, graphics_dir):
        lines.append(f"![{robot.name}]({_IMAGE_REL_PATH}/{robot.name}.webp)")
        lines.append("")

    lines.append("| | |")
    lines.append("|---|---|")
    lines.append(
        _specs_row("DoF", str(robot.dof) if robot.dof > 0 else "runtime-resolved")
    )
    lines.append(_specs_row("End Effector", robot.end_effector))
    if robot.frame_base:
        lines.append(_specs_row("Base Frame", f"`{robot.frame_base}`"))
    if robot.frame_flange:
        lines.append(_specs_row("Flange Frame", f"`{robot.frame_flange}`"))
    lines.append(_specs_row("Sensors", f"{robot.sensor_count}"))
    if robot.action_mode_count > 1:
        lines.append(
            _specs_row(
                "Action Modes",
                f"{robot.action_mode_count} (default: `{robot.default_action_mode}`)",
            )
        )
    elif robot.action_mode_count == 1:
        # A count of 1 adds no information beyond the default itself
        lines.append(
            _specs_row("Action Modes", f"default: `{robot.default_action_mode}`")
        )
    else:
        lines.append(_specs_row("Action Modes", "–"))
    if robot.mass_kg is not None:
        lines.append(_specs_row("Mass", _fmt(robot.mass_kg, "kg")))
    if sidecar.source:
        lines.append(_specs_row("Source", sidecar.source))
    lines.append("")

    # --- Collapsible: Action Modes ---
    if robot.action_modes:
        lines.append("<details><summary>Action Modes</summary>")
        lines.append("")
        lines.append("| Mode | Controller | Dims | Scale |")
        lines.append("|---|---|---|---|")
        for am in robot.action_modes:
            default_marker = " \\*" if am.is_default else ""
            dims = str(am.dims) if am.dims >= 0 else "runtime-resolved"
            scale = am.scale if am.scale else "–"
            lines.append(
                f"| `{am.key}`{default_marker} | {am.controller_type} | {dims} | {scale} |"
            )
        lines.append("")
        lines.append("</details>")
        lines.append("")

    # --- Collapsible: Actuators ---
    if robot.actuators:
        lines.append("<details><summary>Actuators</summary>")
        lines.append("")
        lines.append("| Group | Joints | Effort | Velocity | Stiffness | Damping |")
        lines.append("|---|---|---|---|---|---|")
        for act in robot.actuators:
            joints = ", ".join(f"`{j}`" for j in act.joint_names_expr)
            effort = _fmt_actuator(act.effort_limit, "N·m")
            velocity = _fmt_actuator(act.velocity_limit, "rad/s")
            stiffness = _fmt_actuator(act.stiffness)
            damping = _fmt_actuator(act.damping)
            lines.append(
                f"| {act.group_name} | {joints} | {effort} | {velocity} | {stiffness} | {damping} |"
            )
        lines.append("")
        lines.append("</details>")
        lines.append("")

    # --- Collapsible: Sensors ---
    if robot.sensors:
        lines.append("<details><summary>Sensors</summary>")
        lines.append("")
        lines.append("| Name | Type | Location |")
        lines.append("|---|---|---|")
        for s in robot.sensors:
            loc = s.location.replace("|", "\\|")
            lines.append(f"| {s.name} | {s.sensor_type} | `{loc}` |")
        lines.append("")
        lines.append("</details>")
        lines.append("")

    # --- Collapsible: Initial Joint Configuration ---
    if robot.init_joint_pos:
        lines.append("<details><summary>Initial Joint Configuration</summary>")
        lines.append("")
        lines.append("| Joint | Position |")
        lines.append("|---|---|")
        for joint, pos in robot.init_joint_pos.items():
            # Regex keys set every matching joint; show them as code.
            if any(c in joint for c in "*?|[]()+"):
                joint = "`" + joint.replace("|", "\\|") + "`"
            deg = _rad_to_deg(pos)
            if abs(pos) < 1e-9:
                lines.append(f"| {joint} | 0.0 rad |")
            else:
                lines.append(f"| {joint} | {_fmt(pos)} rad ({_fmt(deg)} deg) |")
        lines.append("")
        lines.append("</details>")
        lines.append("")

    if sidecar.notes:
        lines.append(sidecar.notes)
        lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Task entry
# ---------------------------------------------------------------------------


def render_task_entry(
    task: TaskInfo,
    sidecar: TaskMeta,
    compatible_robots: list[RobotInfo],
    graphics_dir: Path,
) -> str:
    """Render a single task entry as markdown."""
    lines: list[str] = []

    # Heading (H3 for tasks)
    lines.append(f"### {task.display_name} (`{task.name}`)")
    lines.append("")

    # Tagline (prefer sidecar, fall back to introspected)
    tagline = sidecar.tagline or task.tagline
    if tagline:
        lines.append(f"> {tagline}")
        lines.append("")

    # Media: YouTube iframe > image fallback > nothing
    if sidecar.video_id:
        lines.append(_youtube_iframe(sidecar.video_id))
        lines.append("")
    elif _has_thumbnail(task.name, graphics_dir):
        lines.append(f"![{task.name}]({_TASK_IMAGE_REL_PATH}/{task.name}.webp)")
        lines.append("")

    # Preserve hand-written intro prose across docgen regeneration.
    lines.append(f"<!-- PRESERVE:intro_{task.name} -->")
    lines.append(f"<!-- /PRESERVE:intro_{task.name} -->")
    lines.append("")

    # Specs card
    details = []
    if task.agent_rate > 0:
        details.append(f"{int(task.episode_length_s / task.agent_rate)} steps")
    if not task.truncate_episodes:
        details.append("truncation disabled")
    episode_steps = f"{task.episode_length_s}s"
    if details:
        episode_steps += f" ({'; '.join(details)})"

    lines.append("| | |")
    lines.append("|---|---|")
    lines.append(_specs_row("Episode Length", episode_steps))
    # Tasks without configured terms compute their reward at runtime.
    if task.reward_term_count:
        lines.append(_specs_row("Reward Terms", str(task.reward_term_count)))

    # Default robot with cross-link
    if task.default_robot_name:
        default_robot_link = task.default_robot_name
        # Link to the matching robot when available.
        for r in compatible_robots:
            if r.name == task.default_robot_name:
                default_robot_link = f"[{r.display_name}](../{r.page_path}#{r.anchor})"
                break
        lines.append(_specs_row("Default Robot", default_robot_link))

    if task.multi_variant:
        lines.append(_specs_row("Multi Variant", f"`{task.multi_variant}`"))
    lines.append("")

    # Compatible robots.
    if compatible_robots:
        lines.append("#### Robots")
        lines.append("")
        # Build links to compatible robots.
        robot_links: list[str] = []
        for r in compatible_robots:
            link = f"[{r.display_name}](../{r.page_path}#{r.anchor})"
            robot_links.append(link)
        lines.append(" | ".join(robot_links))
        lines.append("")

    # Collapsible: Rewards
    if task.reward_terms:
        lines.append("<details><summary>Rewards</summary>")
        lines.append("")
        lines.append("| Term | Type | Weight | Scale | Notes |")
        lines.append("|---|---|---|---|---|")
        for rt in task.reward_terms:
            scale = _fmt(rt.scale) if rt.scale is not None else "–"
            notes = rt.notes or ""
            lines.append(
                f"| {rt.name} | {rt.term_type} | {_fmt(rt.weight)} | {scale} | {notes} |"
            )
        lines.append("")
        lines.append("</details>")
        lines.append("")

    # Preserve hand-written notes across docgen regeneration.
    lines.append(f"<!-- PRESERVE:notes_{task.name} -->")
    lines.append(f"<!-- /PRESERVE:notes_{task.name} -->")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Object entry
# ---------------------------------------------------------------------------


def render_object_entry(
    obj: ObjectInfo,
    graphics_dir: Path,
    sidecar: AssetMeta,
) -> str:
    """Render a single object entry as markdown."""
    lines: list[str] = []

    lines.append(f'<a id="{obj.name}"></a>')
    lines.append("")
    lines.append(f"#### {obj.display_name} (`{obj.name}`)")
    lines.append("")

    tagline = sidecar.tagline
    if tagline:
        lines.append(f"> {tagline}")
        lines.append("")

    if _has_thumbnail(obj.name, graphics_dir):
        lines.append(f"![{obj.name}]({_IMAGE_REL_PATH}/{obj.name}.webp)")
        lines.append("")

    lines.append("| | |")
    lines.append("|---|---|")
    lines.append(_specs_row("Type", obj.object_type))
    if obj.variant:
        lines.append(_specs_row("Variant", obj.variant))
    if obj.domains:
        lines.append(_specs_row("Domains", ", ".join(obj.domains)))
    lines.append(_specs_row("Collision", obj.collision_approx or "–"))
    if obj.density is not None:
        lines.append(_specs_row("Density", _fmt(obj.density, "kg/m³")))
    lines.append("")

    if sidecar.notes:
        lines.append(sidecar.notes)
        lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Scenery entry
# ---------------------------------------------------------------------------


def render_scenery_entry(
    sc: SceneryInfo,
    graphics_dir: Path,
    sidecar: AssetMeta,
) -> str:
    """Render a single scenery entry as markdown."""
    lines: list[str] = []

    lines.append(f'<a id="{sc.name}"></a>')
    lines.append("")
    lines.append(f"#### {sc.display_name} (`{sc.name}`)")
    lines.append("")

    tagline = sidecar.tagline
    if tagline:
        lines.append(f"> {tagline}")
        lines.append("")

    if _has_thumbnail(sc.name, graphics_dir):
        lines.append(f"![{sc.name}]({_IMAGE_REL_PATH}/{sc.name}.webp)")
        lines.append("")

    lines.append("| | |")
    lines.append("|---|---|")
    lines.append(_specs_row("Type", sc.scenery_type))
    if sc.variant:
        lines.append(_specs_row("Variant", sc.variant))
    if sc.domains:
        lines.append(_specs_row("Domains", ", ".join(sc.domains)))
    if sc.sub_assets:
        lines.append(_specs_row("Sub-assets", ", ".join(sc.sub_assets)))
    lines.append("")

    if sidecar.notes:
        lines.append(sidecar.notes)
        lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Cross-cutting page renderers
# ---------------------------------------------------------------------------


def render_compatibility_matrix(
    tasks_by_category: dict[str, list[TaskInfo]],
    robot_types: list[str],
) -> str:
    """Render the compatibility matrix page: tasks (rows) vs robot types (cols)."""
    lines: list[str] = []
    lines.append("# Compatibility Matrix")
    lines.append("")
    lines.append(
        "The matrix shows supported robot categories for each task. A ✓ marks "
        "a supported combination; a blank cell means unavailable. Tasks prefixed "
        "with `_` (for example, `_manipulation`) are reusable templates, not "
        "preconfigured goal scenarios; they expose a generic environment and "
        "require robot and task-specific goal configuration."
    )
    lines.append("")

    # Keep the scope note above the table; docgen preserves this block.
    lines.append("<!-- PRESERVE:scope_compatibility -->")
    lines.append("<!-- /PRESERVE:scope_compatibility -->")
    lines.append("")

    type_headers = " | ".join(f"**{t.replace('_', ' ').title()}**" for t in robot_types)
    lines.append(f"| Task | {type_headers} |")
    sep = " | ".join(":---:" for _ in robot_types)
    lines.append(f"|---|{sep}|")

    for _category, tasks in sorted(tasks_by_category.items()):
        for task in tasks:
            # Mark every annotated member category (e.g. `LeggedRobot | Humanoid`).
            member_types = {
                member_type for member_type, _cls in task._compatible_robot_members
            } or {task.compatible_robot_type}
            cells: list[str] = []
            for rt in robot_types:
                if rt in member_types:
                    cells.append("✓")
                else:
                    cells.append("")
            row = " | ".join(cells)
            lines.append(f"| `{task.name}` | {row} |")

    # This category matrix does not claim learner or hardware support.
    lines.append("")
    lines.append("<!-- PRESERVE:notes_compatibility -->")
    lines.append("<!-- /PRESERVE:notes_compatibility -->")

    lines.append("")
    return "\n".join(lines)


def render_action_space_catalog(
    robots_by_type: dict[str, list[RobotInfo]],
) -> str:
    """Render the action space catalog page."""
    lines: list[str] = []
    lines.append("# Action Space Catalog")
    lines.append("")
    lines.append(
        "Each robot offers one or more action modes, chosen at launch via "
        "`env.robot.action_mode=<mode>` (the default mode is marked `*`). "
        "**Dims** gives action-vector length; **Scale** multiplies policy "
        "output before it reaches the controller. `runtime-resolved` length "
        "depends on the loaded asset (e.g. a joint-space mode with pattern "
        "`.*`) and is read from the action manager when the environment is "
        "built."
    )
    lines.append("")
    lines.append(
        "The machine-readable action spec (`env.srb_spec.action`) targets the "
        "action manager's runtime terms. Schema-version-2 descriptors name "
        "runtime terms directly, e.g. the multicopter `cmd_vel`, "
        "body-acceleration `body_vel` and task-space `delta_twist` terms. "
        "A schema-version-1 descriptor may split one runtime term into "
        "semantic fields, such as `lin` and `ang` for the wheeled-drive "
        "`cmd_vel` term, but its "
        "summed field dimensions must equal the summed runtime term "
        "dimensions. A mismatch of either kind fails the spec build instead "
        "of publishing a wrong layout. Groups without descriptors use the "
        "schema-version-0 fallback: no semantics, no dimension check. Metadata that carries an unsupported descriptor "
        "(the multicopter `thrust` / `multicopter.rotor_thrusts`, the "
        "task-space quaternion `ee_rot` / `end_effector.orientation`, or the "
        "velocity-labelled body `lin` / `body.linear_velocity`) is rejected on "
        "load."
    )
    lines.append("")
    lines.append(
        "Task-space modes publish one runtime-length `delta_twist` term: three "
        "position-delta channels (m), then three axis-angle rotation-delta "
        "channels (rad); position-only inverse kinematics uses the first "
        "three. Variable-stiffness "
        "operational-space modes append six stiffness channels, and "
        "full-impedance modes append six stiffness and then six damping-ratio "
        "channels."
    )
    lines.append("")
    lines.append(
        "Multicopter Body Acceleration (`body_acc` on the multicopters) is a "
        "velocity command despite its name: four channels (`linear_x`, "
        "`linear_y`, `linear_z`, `angular_z`) are clamped to [-1, 1] and "
        "multiplied by the scale into m/s and rad/s, and x and y are rotated "
        "by the current yaw. Body Acceleration (`body_acc` on the orbital "
        "bodies) is an acceleration command: six channels are clamped to "
        "[-1, 1] and multiplied by the scale into a linear (m/s²) and angular "
        "(rad/s²) acceleration in the world frame, which is integrated into "
        "the root velocity as `acceleration * physics_dt` on every physics "
        "step. The velocity change over one environment step is therefore "
        "`acceleration * step_dt` whatever the decimation. The relative "
        "controller variant, when configured, expresses the same command in "
        "the body frame."
    )
    lines.append("")
    lines.append(
        "Task-space modes (`diff_ik`, `osc_*`, `smooth_osc`) act on the tool "
        "centre point that tasks observe as `robot_ee` (the `tf_end_effector` "
        "target): the manipulator's flange frame composed with the mounted "
        "tool's TCP, or the bare flange when no tool is mounted. The "
        "environment config sets every such term's `body_offset` to that "
        "frame. Isaac Lab's task-space terms carry the body Jacobian to it in "
        "the articulation root frame, the frame the end-effector pose and "
        "velocity are measured in (a term's `base_name`, when set, must name "
        "the root body), and the operational-space terms damp the TCP velocity "
        "derived from the flange link frame rather than its centre of mass. A "
        "pure rotation command therefore turns the tool about its TCP."
    )
    lines.append("")
    lines.append("| Robot | Mode | Controller | Dims | Scale |")
    lines.append("|---|---|---|---|---|")

    for _rtype, robots in sorted(robots_by_type.items()):
        for robot in robots:
            for am in robot.action_modes:
                # Match mdformat's unescaped marker for stable regeneration.
                default_marker = " *" if am.is_default else ""
                dims = str(am.dims) if am.dims >= 0 else "runtime-resolved"
                scale = am.scale if am.scale else "–"
                lines.append(
                    f"| {robot.display_name} | `{am.key}`{default_marker} "
                    f"| {am.controller_type} | {dims} | {scale} |"
                )

    lines.append("")
    return "\n".join(lines)


def render_sensor_catalog(
    robots_by_type: dict[str, list[RobotInfo]],
) -> str:
    """Render the sensor catalog page."""
    lines: list[str] = []
    lines.append("# Sensor Catalog")
    lines.append("")
    lines.append(
        "Each robot ships with configured sensors. **Location** is the prim-path regex "
        "relative to the robot prim where a sensor is attached. Types: **Camera** (RGB / "
        "depth images), **Imu** (linear acceleration and angular velocity), "
        "**Contact Sensor** (per-link contact state), and **Heightmap Sensor** "
        "(ray-cast elevation grid)."
    )
    lines.append("")
    lines.append("| Robot | Sensor Name | Type | Location |")
    lines.append("|---|---|---|---|")

    for _rtype, robots in sorted(robots_by_type.items()):
        for robot in robots:
            for s in robot.sensors:
                # Escape pipes so regex locations do not split table cells.
                loc = s.location.replace("|", "\\|")
                lines.append(
                    f"| {robot.display_name} | {s.name} | {s.sensor_type} | `{loc}` |"
                )

    lines.append("")
    return "\n".join(lines)


def render_stats_dashboard(
    robots_by_type: dict[str, list[RobotInfo]],
    objects_by_type: dict[str, list[ObjectInfo]],
    scenery_by_type: dict[str, list[SceneryInfo]],
    tasks_by_category: dict[str, list[TaskInfo]],
) -> str:
    """Render the stats dashboard page."""
    lines: list[str] = []
    lines.append("# Stats Dashboard")
    lines.append("")

    # Count entries.
    total_robots = sum(len(v) for v in robots_by_type.values())
    total_objects = sum(len(v) for v in objects_by_type.values())
    total_scenery = sum(len(v) for v in scenery_by_type.values())
    total_tasks = sum(len(v) for v in tasks_by_category.values())

    total_action_modes = sum(
        len(r.action_modes) for robots in robots_by_type.values() for r in robots
    )
    sensor_types: set[str] = set()
    for robots in robots_by_type.values():
        for r in robots:
            for s in r.sensors:
                sensor_types.add(s.sensor_type)

    lines.append("| | Count |")
    lines.append("|---|---|")
    lines.append(f"| **Robots** | {total_robots} |")
    for rtype, robots in sorted(robots_by_type.items()):
        lines.append(f"| - {rtype.replace('_', ' ').title()} | {len(robots)} |")
    lines.append(f"| **Objects** | {total_objects} |")
    lines.append(f"| **Scenery** | {total_scenery} |")
    lines.append(f"| **Tasks** | {total_tasks} |")
    for cat, tasks in sorted(tasks_by_category.items()):
        lines.append(f"| - {cat.replace('_', ' ').title()} | {len(tasks)} |")
    lines.append(f"| **Action Modes** (total) | {total_action_modes} |")
    lines.append(f"| **Sensor Types** | {len(sensor_types)} |")
    lines.append("")

    return "\n".join(lines)


def render_attribution_matrix(sidecar: SidecarData) -> str:
    """Render the attribution matrix page."""
    lines: list[str] = []
    lines.append("# Attributions")
    lines.append("")

    # Keep assets with attribution data.
    entries = [
        (name, meta)
        for name, meta in sorted(sidecar.assets.items())
        if meta.source or meta.license or meta.author
    ]

    if not entries:
        lines.append("*No attribution data available.*")
        lines.append("")
        return "\n".join(lines)

    lines.append("| Asset | Source | License | Author |")
    lines.append("|---|---|---|---|")
    for name, meta in entries:
        source = meta.source or "–"
        lic = meta.license or "–"
        author = meta.author or "–"
        lines.append(f"| `{name}` | {source} | {lic} | {author} |")

    lines.append("")
    lines.extend(_FONT_ATTRIBUTIONS)
    return "\n".join(lines)


_FONT_ATTRIBUTIONS = (
    "## Fonts",
    "",
    "The GUI (`crates/srb_gui`) bundles the following fonts. Their licence texts are next to the font files in `crates/srb_gui/assets/fonts/`.",
    "",
    "| Font | Source | License | Author |",
    "|---|---|---|---|",
    "| Inter | [rsms/inter](https://github.com/rsms/inter) | OFL-1.1 | The Inter Project Authors |",
    "| Monaspace Neon | [githubnext/monaspace](https://github.com/githubnext/monaspace) | OFL-1.1 | GitHub |",
    "| Material Icons Round | [google/material-design-icons](https://github.com/google/material-design-icons) | Apache-2.0 | Google |",
    "",
)
