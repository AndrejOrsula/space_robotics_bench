"""Docgen orchestrator — ties introspection, templates, TOML, and collage generation together."""

from __future__ import annotations

import re
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from srb.utils.path import SRB_DIR

_DOCS_SRC_DIR = SRB_DIR / "docs" / "src"
_GRAPHICS_DIR = _DOCS_SRC_DIR / "_graphics" / "assets"
_TASK_GRAPHICS_DIR = _DOCS_SRC_DIR / "_graphics" / "tasks"


def _simulation_app_launch_args() -> tuple[dict[str, object], str]:
    from srb.utils.path import SRB_APPS_DIR

    return (
        {"headless": True},
        SRB_APPS_DIR.joinpath("srb.headless.kit").as_posix(),
    )


@contextmanager
def _temporary_environment_override(name: str, value: str) -> Iterator[None]:
    import os

    previous = os.environ.get(name)
    os.environ[name] = value
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = previous


_PAGE_INTROS = {
    "manipulators.md": (
        "# Robot Arms & Manipulators\n\n"
        "Serial and dual-arm manipulators registered in the Space Robotics Bench "
        "for dexterous manipulation, assembly, and maintenance across terrestrial "
        "and space environments."
    ),
    "mobile_robots.md": (
        "# Mobile Robots\n\n"
        "Mobile platforms registered in the Space Robotics Bench include "
        "wheeled rovers, legged robots, aerial vehicles, landers, spacecraft, "
        "and a surface vessel for navigation and locomotion tasks."
    ),
    "mobile_manipulators.md": (
        "# Mobile Manipulators\n\n"
        "Mobile manipulators registered in the Space Robotics Bench: generic "
        "platforms that mount a manipulator on an aerial, ground, or orbital "
        "base, and humanoid robots. Generic platforms assemble their parts at "
        "runtime, so their listed DoF, sensors, and action modes cover only the "
        "class defaults; the composed robot inherits those of its parts."
    ),
    "tools.md": (
        "# Tools\n\n"
        "Grippers, hands, and other end-of-arm tools registered in the Space "
        "Robotics Bench. They attach to robot arms for dexterous manipulation, "
        "sample collection, and assembly."
    ),
    "objects.md": (
        "# Objects\n\n"
        "Objects registered in the Space Robotics Bench, from manipulable items "
        "such as pegs, bolts, blocks, and rock samples to payloads, pedestals, "
        "and lights."
    ),
    "scenery.md": (
        "# Scenery\n\n"
        "Environmental scenery registered in the Space Robotics Bench includes "
        "terrains, facilities, and other scene elements."
    ),
}

_TASK_PAGE_INTROS = {
    "manipulation.md": (
        "# Environments -- Manipulation (Static)\n\n"
        "Fixed-base robot arm manipulation environments."
    ),
    "mobile_robotics.md": (
        "# Environments -- Mobile Robotics\n\n"
        "Mobile robot navigation and locomotion tasks for Isaac Sim 6.1.0 with Isaac\n"
        "Lab 3.0.0."
    ),
    "mobile_manipulation.md": (
        "# Environments -- Mobile Manipulation\n\n"
        "Environments combining mobility and manipulation."
    ),
    "digital_twin.md": (
        "# Environments -- Digital Twins\n\n"
        "Facility-inspired simulator environments, not validated against their "
        "physical counterparts."
    ),
}

_ROBOT_PAGE_MAP: dict[str, tuple[str, str]] = {
    "manipulator": ("assets/robots", "manipulators.md"),
    "mobile_robot": ("assets/robots", "mobile_robots.md"),
    "mobile_manipulator": ("assets/robots", "mobile_manipulators.md"),
}

_TASK_PAGE_MAP: dict[str, tuple[str, str]] = {
    "manipulation": ("envs", "manipulation.md"),
    "mobile": ("envs", "mobile_robotics.md"),
    "mobile_manipulation": ("envs", "mobile_manipulation.md"),
    "digital_twin": ("envs", "digital_twin.md"),
}

_PRESERVE_RE = re.compile(
    r"<!-- PRESERVE:(\w+) -->\n(.*?)\n<!-- /PRESERVE:\1 -->",
    re.DOTALL,
)


_EMPTY_PRESERVE_RE = re.compile(
    r"\n*<!-- PRESERVE:(\w+) -->\n\s*<!-- /PRESERVE:\1 -->\n*"
)


def _preserve_blocks(existing_content: str) -> dict[str, str]:
    """Extract PRESERVE blocks from existing markdown."""
    return {m.group(1): m.group(2) for m in _PRESERVE_RE.finditer(existing_content)}


def _insert_preserve_block(content: str, key: str, block: str) -> str:
    """Insert preserved block back into generated content."""
    placeholder = f"<!-- PRESERVE:{key} -->\n<!-- /PRESERVE:{key} -->"
    filled = f"<!-- PRESERVE:{key} -->\n{block}\n<!-- /PRESERVE:{key} -->"
    if placeholder in content:
        return content.replace(placeholder, filled)
    pattern = re.compile(
        rf"<!-- PRESERVE:{re.escape(key)} -->\n.*?\n<!-- /PRESERVE:{re.escape(key)} -->",
        re.DOTALL,
    )
    if pattern.search(content):
        # A callable keeps backslashes literal instead of reading them as group references.
        return pattern.sub(lambda _match: filled, content)
    return content


def _write_page(path: Path, content: str, preserved: dict[str, str]) -> None:
    """Write page content, re-inserting any preserved blocks."""
    for key, block in preserved.items():
        content = _insert_preserve_block(content, key, block)
    content = _EMPTY_PRESERVE_RE.sub("\n\n", content).rstrip("\n") + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _read_existing(path: Path) -> tuple[str, dict[str, str]]:
    """Read an existing file and extract preserved blocks."""
    if path.is_file():
        text = path.read_text(encoding="utf-8")
        return text, _preserve_blocks(text)
    return "", {}


def _collage_banner(filename: str) -> str:
    """Return a markdown image line for the page's collage banner, or empty string."""
    from srb.cli.commands._doc._collage import PAGE_COLLAGE_NAMES

    collage_name = PAGE_COLLAGE_NAMES.get(filename)
    # Collages are generated before pages; link only ones that exist now.
    if collage_name and (_GRAPHICS_DIR / collage_name).is_file():
        return f"![Overview](../../_graphics/assets/{collage_name})"
    return ""


def _generate_robot_pages(
    robots_by_type: dict,
    sidecar,
    render_fn,
) -> None:
    from srb.utils import logging

    for robot_type, (subdir, filename) in _ROBOT_PAGE_MAP.items():
        robots = robots_by_type.get(robot_type, [])

        out_path = _DOCS_SRC_DIR / subdir / filename
        _existing, preserved = _read_existing(out_path)

        lines: list[str] = []
        intro = _PAGE_INTROS.get(filename, f"# {filename}")
        lines.append(intro)
        lines.append("")
        banner = _collage_banner(filename)
        if banner:
            lines.append(banner)
            lines.append("")

        by_category: dict[str, list] = {}
        for r in robots:
            by_category.setdefault(r.category, []).append(r)

        for category, group in sorted(by_category.items()):
            lines.append(f"## {category}")
            lines.append("")
            for robot in sorted(group, key=lambda r: r.name):
                from srb.cli.commands._doc._toml import AssetMeta

                meta = sidecar.assets.get(robot.name, AssetMeta())
                entry = render_fn(robot, _GRAPHICS_DIR, meta)
                lines.append(entry)
            lines.append("")

        content = "\n".join(lines)
        _write_page(out_path, content, preserved)
        logging.logger.info(f"Generated {out_path.relative_to(SRB_DIR)}")


def _generate_tools_page(
    objects_by_type: dict,
    sidecar,
    render_fn,
) -> None:
    """Generate the tools page from ObjectType.TOOL entries."""
    from srb.utils import logging

    tools = objects_by_type.get("tool", [])
    if not tools:
        return

    out_path = _DOCS_SRC_DIR / "assets" / "robots" / "tools.md"
    _existing, preserved = _read_existing(out_path)

    lines: list[str] = []
    intro = _PAGE_INTROS.get("tools.md", "# Tools")
    lines.append(intro)
    lines.append("")
    banner = _collage_banner("tools.md")
    if banner:
        lines.append(banner)
        lines.append("")

    by_category: dict[str, list] = {}
    for tool in tools:
        by_category.setdefault(tool.category, []).append(tool)

    for category, group in sorted(by_category.items()):
        lines.append(f"## {category}")
        lines.append("")
        for tool in sorted(group, key=lambda t: t.name):
            from srb.cli.commands._doc._toml import AssetMeta

            meta = sidecar.assets.get(tool.name, AssetMeta())
            entry = render_fn(tool, _GRAPHICS_DIR, meta)
            lines.append(entry)
        lines.append("")

    content = "\n".join(lines)
    _write_page(out_path, content, preserved)
    logging.logger.info(f"Generated {out_path.relative_to(SRB_DIR)}")


def _generate_object_page(
    objects_by_type: dict,
    sidecar,
    render_fn,
) -> None:
    from srb.utils import logging

    out_path = _DOCS_SRC_DIR / "assets" / "objects" / "objects.md"
    _existing, preserved = _read_existing(out_path)

    lines: list[str] = []
    intro = _PAGE_INTROS.get("objects.md", "# Objects")
    lines.append(intro)
    lines.append("")
    banner = _collage_banner("objects.md")
    if banner:
        lines.append(banner)
        lines.append("")

    by_category: dict[str, list] = {}
    for obj_type, objs in objects_by_type.items():
        if obj_type == "tool":
            continue
        for obj in objs:
            by_category.setdefault(obj.category, []).append(obj)

    for category, group in sorted(by_category.items()):
        lines.append(f"## {category}")
        lines.append("")
        for obj in sorted(group, key=lambda o: o.name):
            from srb.cli.commands._doc._toml import AssetMeta

            meta = sidecar.assets.get(obj.name, AssetMeta())
            entry = render_fn(obj, _GRAPHICS_DIR, meta)
            lines.append(entry)
        lines.append("")

    content = "\n".join(lines)
    _write_page(out_path, content, preserved)
    logging.logger.info(f"Generated {out_path.relative_to(SRB_DIR)}")


def _generate_scenery_page(
    scenery_by_type: dict,
    sidecar,
    render_fn,
) -> None:
    from srb.utils import logging

    out_path = _DOCS_SRC_DIR / "assets" / "scenery" / "scenery.md"
    _existing, preserved = _read_existing(out_path)

    lines: list[str] = []
    intro = _PAGE_INTROS.get("scenery.md", "# Scenery")
    lines.append(intro)
    lines.append("")
    banner = _collage_banner("scenery.md")
    if banner:
        lines.append(banner)
        lines.append("")

    by_category: dict[str, list] = {}
    for sceneries in scenery_by_type.values():
        for sc in sceneries:
            by_category.setdefault(sc.category, []).append(sc)

    for category, group in sorted(by_category.items()):
        lines.append(f"## {category}")
        lines.append("")
        for sc in sorted(group, key=lambda s: s.name):
            from srb.cli.commands._doc._toml import AssetMeta

            meta = sidecar.assets.get(sc.name, AssetMeta())
            entry = render_fn(sc, _GRAPHICS_DIR, meta)
            lines.append(entry)
        lines.append("")

    content = "\n".join(lines)
    _write_page(out_path, content, preserved)
    logging.logger.info(f"Generated {out_path.relative_to(SRB_DIR)}")


def _generate_task_pages(
    tasks_by_category: dict,
    sidecar,
    all_robots: dict,
    render_fn,
) -> None:
    from srb.utils import logging

    for task_category, (subdir, filename) in _TASK_PAGE_MAP.items():
        tasks = tasks_by_category.get(task_category, [])

        out_path = _DOCS_SRC_DIR / subdir / filename
        _existing, preserved = _read_existing(out_path)

        lines: list[str] = []
        intro = _TASK_PAGE_INTROS.get(filename, f"# {filename}")
        lines.append(intro)
        lines.append("")

        for task in sorted(tasks, key=lambda t: t.name):
            from srb.cli.commands._doc._toml import TaskMeta

            meta = sidecar.tasks.get(task.name, TaskMeta())

            compatible_robots = _compatible_robots(task, all_robots)
            entry = render_fn(task, meta, compatible_robots, _TASK_GRAPHICS_DIR)
            lines.append(entry)

        lines.append("")

        content = "\n".join(lines)
        _write_page(out_path, content, preserved)
        logging.logger.info(f"Generated {out_path.relative_to(SRB_DIR)}")


def _generate_collages(
    robots_by_type: dict,
    objects_by_type: dict,
    scenery_by_type: dict,
    generate_collage_fn,
) -> None:
    from srb.cli.commands._doc._collage import (
        COLLAGE_MASTER_HEIGHT,
        MASTER_COLLAGE_NAME,
        PAGE_COLLAGE_NAMES,
    )
    from srb.utils import logging

    _GRAPHICS_DIR.mkdir(parents=True, exist_ok=True)

    page_assets: dict[str, list[str]] = {
        "manipulators.md": [r.name for r in robots_by_type.get("manipulator", [])],
        "mobile_robots.md": [r.name for r in robots_by_type.get("mobile_robot", [])],
        "mobile_manipulators.md": [
            r.name for r in robots_by_type.get("mobile_manipulator", [])
        ],
        "tools.md": [o.name for o in objects_by_type.get("tool", [])],
        "objects.md": [
            o.name
            for otype, objs in objects_by_type.items()
            if otype != "tool"
            for o in objs
        ],
        "scenery.md": [s.name for scns in scenery_by_type.values() for s in scns],
    }

    all_names: list[str] = []

    for page_key, collage_filename in PAGE_COLLAGE_NAMES.items():
        names = page_assets.get(page_key, [])
        all_names.extend(names)
        output_path = _GRAPHICS_DIR / collage_filename
        if names and generate_collage_fn(_GRAPHICS_DIR, names, output_path):
            logging.logger.info(
                f"Collage generated: {output_path.relative_to(SRB_DIR)}"
            )
        else:
            # A collage from a previous run no longer reflects the thumbnails.
            output_path.unlink(missing_ok=True)

    master_path = _GRAPHICS_DIR / MASTER_COLLAGE_NAME
    if all_names and generate_collage_fn(
        _GRAPHICS_DIR,
        all_names,
        master_path,
        banner_height=COLLAGE_MASTER_HEIGHT,
    ):
        logging.logger.info(
            f"Master collage generated: {master_path.relative_to(SRB_DIR)}"
        )
    else:
        master_path.unlink(missing_ok=True)


def _generate_assets_index() -> None:
    from srb.utils import logging

    out_path = _DOCS_SRC_DIR / "assets" / "index.md"
    _existing, preserved = _read_existing(out_path)

    from srb.cli.commands._doc._collage import MASTER_COLLAGE_NAME

    lines: list[str] = []
    lines.append("# Assets")
    lines.append("")
    master_img = _GRAPHICS_DIR / MASTER_COLLAGE_NAME
    if master_img.is_file():
        lines.append(f"![Overview](../_graphics/assets/{MASTER_COLLAGE_NAME})")
        lines.append("")
    lines.append("Robots, objects, and scenery available in the Space Robotics Bench.")
    lines.append("")
    lines.append("- [Robot Arms & Manipulators](robots/manipulators.md)")
    lines.append("- [Mobile Robots](robots/mobile_robots.md)")
    lines.append("- [Mobile Manipulators](robots/mobile_manipulators.md)")
    lines.append("- [End Effectors & Tools](robots/tools.md)")
    lines.append("- [Objects](objects/objects.md)")
    lines.append("- [Scenery](scenery/scenery.md)")
    lines.append("")

    content = "\n".join(lines)
    _write_page(out_path, content, preserved)
    logging.logger.info(f"Generated {out_path.relative_to(SRB_DIR)}")


def _execute_static_docgen(
    *, assets_only: bool = False, tasks_only: bool = False
) -> None:
    """Validate the static docgen inputs without writing or launching anything."""
    import sys

    from srb.cli.commands._doc._toml import load_sidecar, validate_sidecar

    errors = validate_sidecar()
    if errors:
        for error in errors:
            print(f"SRB static docgen check: {error}", file=sys.stderr)
        raise SystemExit(1)
    sidecar = load_sidecar()
    sections: list[str] = []
    if not tasks_only:
        sections.append(f"assets={len(sidecar.assets)}")
    if not assets_only:
        sections.append(f"tasks={len(sidecar.tasks)}")
    print("SRB static docgen check: " + ", ".join(sections))


def execute_docgen(
    static_only: bool = False,
    assets_only: bool = False,
    tasks_only: bool = False,
    **_kwargs,
) -> None:
    """Run static validation or simulator-backed documentation generation."""
    if static_only:
        _execute_static_docgen(assets_only=assets_only, tasks_only=tasks_only)
        return

    from srb.cli.capabilities import require_isaac_sim

    require_isaac_sim()
    with _temporary_environment_override("SRB_TERRAIN_AUTO_FETCH", "0"):
        _execute_non_static_docgen(
            assets_only=assets_only,
            tasks_only=tasks_only,
            **_kwargs,
        )


def _execute_non_static_docgen(
    assets_only: bool = False,
    tasks_only: bool = False,
    **_kwargs,
) -> None:
    """Generate enriched documentation pages from code introspection.

    This body runs only after the adapter-owned Isaac capability preflight.
    """
    from srb.cli.commands._doc._toml import load_sidecar
    from srb.utils import logging

    logging.logger.info("Loading sidecar TOML data...")
    sidecar = load_sidecar()

    logging.logger.info("Launching Isaac Sim (headless) for introspection...")
    from srb.core.app import start_simulation_app

    launch_config, experience = _simulation_app_launch_args()
    simulation_app = start_simulation_app(launch_config, experience=experience)
    completed = False
    try:
        from srb.cli.commands._doc._collage import generate_collage
        from srb.cli.commands._doc._introspect import (
            introspect_objects,
            introspect_robots,
            introspect_scenery,
            introspect_tasks,
        )
        from srb.cli.commands._doc._templates import (
            render_action_space_catalog,
            render_attribution_matrix,
            render_compatibility_matrix,
            render_object_entry,
            render_robot_entry,
            render_scenery_entry,
            render_sensor_catalog,
            render_stats_dashboard,
            render_task_entry,
        )

        robots_by_type: dict = {}
        objects_by_type: dict = {}
        scenery_by_type: dict = {}
        tasks_by_category: dict = {}

        if not tasks_only:
            logging.logger.info("Introspecting robots...")
            robots_by_type = introspect_robots()

            logging.logger.info("Introspecting objects...")
            objects_by_type = introspect_objects()

            logging.logger.info("Introspecting scenery...")
            scenery_by_type = introspect_scenery()

            # Collages first: page banners link only collages that exist.
            logging.logger.info("Generating collages...")
            _generate_collages(
                robots_by_type, objects_by_type, scenery_by_type, generate_collage
            )

            logging.logger.info("Generating robot pages...")
            _generate_robot_pages(robots_by_type, sidecar, render_robot_entry)

            logging.logger.info("Generating tools page...")
            _generate_tools_page(objects_by_type, sidecar, render_object_entry)

            logging.logger.info("Generating object page...")
            _generate_object_page(objects_by_type, sidecar, render_object_entry)

            logging.logger.info("Generating scenery page...")
            _generate_scenery_page(scenery_by_type, sidecar, render_scenery_entry)

            logging.logger.info("Generating assets index page...")
            _generate_assets_index()

        if not assets_only:
            logging.logger.info("Introspecting tasks...")
            tasks_by_category = introspect_tasks()

            # If we skipped assets introspection, we still need robots for
            # compatible-robot cross-links in task pages
            if tasks_only:
                logging.logger.info("Introspecting robots for task cross-links...")
                robots_by_type = introspect_robots()

            logging.logger.info("Generating task pages...")
            _generate_task_pages(
                tasks_by_category, sidecar, robots_by_type, render_task_entry
            )

        if not assets_only and not tasks_only:
            # Both sections above already introspected every registry.
            robot_types = sorted(robots_by_type.keys())

            logging.logger.info("Generating compatibility matrix...")
            matrix_content = render_compatibility_matrix(tasks_by_category, robot_types)
            matrix_path = _DOCS_SRC_DIR / "reference" / "compatibility.md"
            _existing, preserved = _read_existing(matrix_path)
            _write_page(matrix_path, matrix_content, preserved)

            logging.logger.info("Generating action space catalog...")
            action_content = render_action_space_catalog(robots_by_type)
            action_path = _DOCS_SRC_DIR / "reference" / "action_spaces.md"
            _existing, preserved = _read_existing(action_path)
            _write_page(action_path, action_content, preserved)

            logging.logger.info("Generating sensor catalog...")
            sensor_content = render_sensor_catalog(robots_by_type)
            sensor_path = _DOCS_SRC_DIR / "reference" / "sensors.md"
            _existing, preserved = _read_existing(sensor_path)
            _write_page(sensor_path, sensor_content, preserved)

            logging.logger.info("Generating stats dashboard...")
            stats_content = render_stats_dashboard(
                robots_by_type, objects_by_type, scenery_by_type, tasks_by_category
            )
            stats_path = _DOCS_SRC_DIR / "reference" / "stats.md"
            _existing, preserved = _read_existing(stats_path)
            _write_page(stats_path, stats_content, preserved)

            logging.logger.info("Generating attribution matrix...")
            attr_content = render_attribution_matrix(sidecar)
            attr_path = _DOCS_SRC_DIR / "misc" / "attributions.md"
            _existing, preserved = _read_existing(attr_path)
            _write_page(attr_path, attr_content, preserved)

        logging.logger.info("Docgen complete.")
        completed = True
    finally:
        if not completed:
            # SimulationApp.close() can end the process with exit code 0,
            # which would swallow the propagating failure and leave partially
            # written pages behind a successful `srb doc docgen`; report and
            # exit non-zero first. A flag rather than a broad handler keeps
            # the failure propagating untouched.
            import sys
            import traceback

            from srb.cli.commands._hydra_exit import exit_nonzero_if_failed

            traceback.print_exc()
            print("srb doc docgen FAILED", file=sys.stderr)
            exit_nonzero_if_failed(True)
        # Outer app ownership: the simulator closes on every exit path.
        simulation_app.close()


def _compatible_robots(task: Any, all_robots: dict) -> list:
    """Robots accepted by any member of the task's ``robot`` annotation.

    Each member filters its robot type's list by its exact annotation type
    (e.g. ``OrbitalRobot``, not all mobile robots). A member whose type has no
    registered robots, such as the root ``Robot`` class, is matched across
    every type only when it is the sole member: next to a specific type it is
    an annotation catch-all, and the task's env still requires that specific
    type (``GroundManipulator | Robot`` needs a manipulator articulation).
    """
    members = task._compatible_robot_members or (
        (task.compatible_robot_type, task._compatible_robot_cls),
    )
    specific = [member for member in members if all_robots.get(member[0])]
    if specific:
        members = specific
    every_robot = [robot for robots in all_robots.values() for robot in robots]
    selected: list = []
    seen: set[int] = set()
    for member_type, member_cls in members:
        candidates = all_robots.get(member_type) or (
            every_robot if member_cls is not None else []
        )
        if member_cls is not None:
            candidates = [
                robot
                for robot in candidates
                if robot._cls is not None and issubclass(robot._cls, member_cls)
            ]
        for robot in candidates:
            if id(robot) not in seen:
                seen.add(id(robot))
                selected.append(robot)
    return selected
