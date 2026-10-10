"""``srb list`` / ``srb ls`` -- List registered assets and environments."""

from __future__ import annotations

import argparse
from typing import Sequence

from srb.cli._base import BaseCommand
from srb.cli._parsing import EntityToList, SrbHelpFormatter
from srb.cli.offline import (
    flatten_cache_names,
    read_cache_payload,
    warn_if_offline_cache_incomplete,
)


class ListCommand(BaseCommand):
    @staticmethod
    def _format_link(obj, display_text: str | None = None) -> str:
        text = display_text or getattr(obj, "__name__", str(obj))
        try:
            import inspect

            file_path = inspect.getabsfile(obj)
            line_no = inspect.getsourcelines(obj)[1]
            return f"[link=vscode://file/{file_path}:{line_no}]{text}[/link]"
        except Exception:
            return text

    """List registered assets, action groups, and environments."""

    NAME = "list"
    ALIASES = ("ls",)
    HELP = "List registered assets and environments"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )
        parser.add_argument(
            "category",
            help="Filter of categories to list",
            nargs="*",
            type=str,
            choices=sorted(map(str, EntityToList)),
            default=str(EntityToList.ALL),
        )
        parser.add_argument(
            "--live",
            action="store_true",
            help="Launch Isaac Sim for live registry discovery",
        )
        return parser

    @classmethod
    def execute(
        cls,
        category: str | Sequence[str],
        live: bool = False,
        **_kwargs,
    ) -> None:
        if not live:
            cls._print_offline(category)
            return

        from importlib.util import find_spec

        from srb.cli.capabilities import require_isaac_sim

        require_isaac_sim()
        from srb.cli.commands.agent_launch import agent_experience_path
        from srb.core.app import AppLauncher

        if not find_spec("rich"):
            from srb.utils.failures import OptionalDependencyUnavailableError

            raise OptionalDependencyUnavailableError(
                code="cli.list.rich_unavailable",
                message="srb list --live requires rich; install srb[rich]",
                component="cli.list",
            )

        # Launch Isaac Sim
        launcher = AppLauncher(
            headless=True,
            experience=agent_experience_path(
                headless=True,
                enable_cameras=False,
                xr=False,
            ),
        )

        succeeded = False
        try:
            from srb.cli.offline_cache_build import update_offline_srb_cache

            update_offline_srb_cache()

            cls._print_tables(category)
            succeeded = True
        finally:
            # SimulationApp.close() exits with status 0 before failures can
            # propagate. Report and flush the error, then use os._exit(1) to
            # avoid hangs during interpreter shutdown on Kit's threads.
            # Track success explicitly: execute() may run inside another except.
            if not succeeded:
                import os
                import sys
                import traceback

                from srb.utils import logging

                logging.critical(traceback.format_exc())
                sys.stdout.flush()
                sys.stderr.flush()
                os._exit(1)

            launcher.app.close()

    @staticmethod
    def _print_offline(category: str | Sequence[str]) -> None:
        """Print the last generated registry snapshot without live discovery."""
        selected = (
            {EntityToList.from_str(category)}
            if isinstance(category, str)
            else set(map(EntityToList.from_str, category))
        )
        if EntityToList.ALL in selected:
            selected = {
                EntityToList.ACTION,
                EntityToList.SCENERY,
                EntityToList.OBJECT,
                EntityToList.ROBOT,
                EntityToList.ENV,
            }
        if EntityToList.ASSET in selected:
            selected.discard(EntityToList.ASSET)
            selected.update(
                {EntityToList.SCENERY, EntityToList.OBJECT, EntityToList.ROBOT}
            )

        # Warn on missing, incomplete, or stale registry snapshots.
        warn_if_offline_cache_incomplete()

        print("SRB offline registry snapshot (use --live for Isaac Sim discovery)")
        cache_by_category = {
            EntityToList.ENV: ("Environments", "env"),
            EntityToList.ROBOT: ("Robots", "robot"),
            EntityToList.OBJECT: ("Objects", "object"),
            EntityToList.SCENERY: ("Scenery", "scenery"),
            EntityToList.ACTION: ("Hardware interfaces", "hardware"),
        }
        for kind in (
            EntityToList.ROBOT,
            EntityToList.OBJECT,
            EntityToList.SCENERY,
            EntityToList.ACTION,
            EntityToList.ENV,
        ):
            if kind not in selected:
                continue
            title, cache_name = cache_by_category[kind]
            payload = read_cache_payload(cache_name)
            names = flatten_cache_names(payload)
            print(f"{title}: {len(names)}")
            for name in names:
                print(f"  {name}")

    # ------------------------------------------------------------------
    # Internal helpers (all require a running simulation)
    # ------------------------------------------------------------------

    @classmethod
    def _print_tables(
        cls,
        category: str | Sequence[str],
    ) -> None:
        import importlib
        import inspect
        from os import path
        from pathlib import Path

        from rich import print
        from rich.table import Table

        # Normalise category set
        category = (  # type: ignore[misc]
            {EntityToList.from_str(category)}
            if isinstance(category, str)
            else set(map(EntityToList.from_str, category))
        )
        if EntityToList.ALL in category:
            category = {
                EntityToList.ACTION,
                EntityToList.ASSET,
                EntityToList.ENV,
            }
        if EntityToList.ASSET in category:
            category.remove(EntityToList.ASSET)  # type: ignore[misc]
            category.add(EntityToList.SCENERY)  # type: ignore[misc]
            category.add(EntityToList.OBJECT)  # type: ignore[misc]
            category.add(EntityToList.ROBOT)  # type: ignore[misc]

        # ---- Assets ----
        if (
            EntityToList.SCENERY in category
            or EntityToList.OBJECT in category
            or EntityToList.ROBOT in category
        ):
            from srb.core.asset import AssetType

            table = Table(title="Assets")
            table.add_column("#", justify="right", style="cyan", no_wrap=True)
            table.add_column("Name", justify="left", style="blue", no_wrap=True)
            table.add_column("Type", justify="left", style="magenta", no_wrap=True)
            table.add_column("Subtype", justify="left", style="red", no_wrap=True)
            table.add_column(
                "Parent Class", justify="left", style="green", no_wrap=True
            )
            table.add_column("Asset Config", justify="left", style="yellow")
            table.add_column("Path", justify="left", style="white")
            table.add_column("Action Modes", justify="left", style="bright_white")
            i = 0

            if EntityToList.SCENERY in category:
                i = cls._add_asset_rows(table, i, AssetType.SCENERY)
            if EntityToList.OBJECT in category:
                i = cls._add_asset_rows(table, i, AssetType.OBJECT)
            if EntityToList.ROBOT in category:
                i = cls._add_asset_rows(table, i, AssetType.ROBOT)
            print(table)

        # ---- Action groups ----
        if EntityToList.ACTION in category:
            from srb.core.action import ActionGroupRegistry
            from srb.core.action import group as srb_action_groups

            table = Table(title="Action Groups")
            table.add_column("#", justify="right", style="cyan", no_wrap=True)
            table.add_column("Name", justify="left", style="blue", no_wrap=True)
            table.add_column("Path", justify="left", style="white")

            for i, action_group_class in enumerate(ActionGroupRegistry.registry, 1):
                action_group_name = action_group_class.name()
                action_group_path = Path(
                    inspect.getabsfile(
                        importlib.import_module(action_group_class.__module__)
                    )
                )
                try:
                    action_group_relpath = action_group_path.relative_to(
                        Path(inspect.getabsfile(srb_action_groups)).parent
                    )
                except ValueError:
                    action_group_relpath = path.join("EXT", action_group_path.name)
                table.add_row(
                    str(i),
                    ListCommand._format_link(action_group_class, action_group_name),
                    f"[link=vscode://file/{action_group_path}]{action_group_relpath}[/link]",
                )
            print(table)

        # ---- Hardware interfaces ----
        if EntityToList.ACTION in category:
            from srb.interfaces.sim_to_real import (
                hardware as srb_hardware_interfaces,
            )
            from srb.interfaces.sim_to_real.core.hardware import (
                HardwareInterfaceRegistry,
            )

            table = Table(title="Hardware Interfaces")
            table.add_column("#", justify="right", style="cyan", no_wrap=True)
            table.add_column("Name", justify="left", style="blue", no_wrap=True)
            table.add_column("Path", justify="left", style="white")

            for i, hw_cls in enumerate(HardwareInterfaceRegistry.registry, 1):
                hw_name = hw_cls.class_name()
                hw_path = Path(
                    inspect.getabsfile(importlib.import_module(hw_cls.__module__))
                )
                try:
                    hw_relpath = hw_path.relative_to(
                        Path(inspect.getabsfile(srb_hardware_interfaces)).parent
                    )
                except ValueError:
                    hw_relpath = path.join("EXT", hw_path.name)
                table.add_row(
                    str(i),
                    ListCommand._format_link(hw_cls, hw_name),
                    f"[link=vscode://file/{hw_path}]{hw_relpath}[/link]",
                )
            print(table)

        # ---- Environments ----
        if EntityToList.ENV in category:
            import gymnasium

            from srb import tasks as srb_tasks
            from srb.utils.registry import get_srb_tasks

            table = Table(title="Environments")
            table.add_column("#", justify="right", style="cyan", no_wrap=True)
            table.add_column("ID", justify="left", style="blue", no_wrap=True)
            table.add_column("Entrypoint", justify="left", style="green")
            table.add_column("Config", justify="left", style="yellow")
            table.add_column("Path", justify="left", style="white")
            i = 0
            for short_id, task_id in get_srb_tasks().items():
                i += 1
                env = gymnasium.registry[task_id]
                entrypoint_str = env.entry_point
                entrypoint_module, entrypoint_class = str(entrypoint_str).split(":")
                env_module_path = Path(
                    inspect.getabsfile(
                        importlib.import_module(entrypoint_module.rsplit(".", 1)[0])
                    )
                )
                try:
                    env_module_relpath = env_module_path.parent.relative_to(
                        Path(inspect.getabsfile(srb_tasks)).parent
                    )
                except ValueError:
                    env_module_relpath = path.join("EXT", env_module_path.name)
                entrypoint_module = importlib.import_module(entrypoint_module)
                entrypoint_class = getattr(entrypoint_module, entrypoint_class)
                entrypoint_parent = entrypoint_class.__bases__[0]
                cfg_class = env.kwargs["task_cfg"]
                cfg_parent = cfg_class.__bases__[0]
                table.add_row(
                    str(i),
                    short_id + (" <template>" if short_id.startswith("_") else ""),
                    f"{ListCommand._format_link(entrypoint_class)}([red]{ListCommand._format_link(entrypoint_parent)}[/red])",
                    f"{ListCommand._format_link(cfg_class)}([magenta]{ListCommand._format_link(cfg_parent)}[/magenta])",
                    f"[link=vscode://file/{env_module_path}]{env_module_relpath}[/link]",
                )
            print(table)

    @staticmethod
    def _add_asset_rows(
        table,
        row_idx: int,
        asset_type,
    ) -> int:
        """Append rows for a given asset type and return the updated ``row_idx``."""
        import importlib
        import inspect
        from dataclasses import _MISSING_TYPE
        from os import path
        from pathlib import Path

        from srb.core.asset import AssetType

        registry_map = {
            AssetType.SCENERY: lambda: (
                __import__("srb.assets.scenery", fromlist=["scenery"]),
                __import__(
                    "srb.core.asset", fromlist=["SceneryRegistry"]
                ).SceneryRegistry,
            ),
            AssetType.OBJECT: lambda: (
                __import__("srb.assets.object", fromlist=["object"]),
                __import__(
                    "srb.core.asset", fromlist=["ObjectRegistry"]
                ).ObjectRegistry,
            ),
            AssetType.ROBOT: lambda: (
                __import__("srb.assets.robot", fromlist=["robot"]),
                __import__("srb.core.asset", fromlist=["RobotRegistry"]).RobotRegistry,
            ),
        }

        assets_module, registry = registry_map[asset_type]()
        i = row_idx

        for asset_subtype, asset_classes in registry.items():
            for j, asset_class in enumerate(asset_classes):
                i += 1
                asset_name = asset_class.name()
                parent_class = asset_class.__bases__[0]
                try:
                    asset_cfg_class = asset_class().asset_cfg.__class__  # type: ignore[misc]
                except Exception as e:
                    from srb.utils import logging

                    logging.debug(
                        f"Could not instantiate {asset_class.__module__}:{asset_class.__name__} for config lookup: {e}"
                    )
                    asset_cfg_class = None
                asset_module_path = Path(
                    inspect.getabsfile(importlib.import_module(asset_class.__module__))
                )
                try:
                    asset_module_relpath = asset_module_path.relative_to(
                        Path(inspect.getabsfile(assets_module)).parent
                    )
                except ValueError:
                    asset_module_relpath = path.join("EXT", asset_module_path.name)
                action_modes_cell = ""
                modes = getattr(asset_class, "action_modes", None)
                if modes:
                    keys = list(modes.keys())
                    # Mark the first (default) key with a leading asterisk
                    formatted = [f"*{k}" if i == 0 else k for i, k in enumerate(keys)]
                    action_modes_cell = ", ".join(formatted)
                table.add_row(
                    str(i),
                    ListCommand._format_link(asset_class, asset_name),
                    str(asset_type),
                    str(asset_subtype),
                    ListCommand._format_link(parent_class),
                    ListCommand._format_link(asset_cfg_class)
                    if asset_cfg_class is not None
                    and asset_cfg_class is not _MISSING_TYPE
                    else "<DYNAMIC>",
                    f"[link=vscode://file/{asset_module_path}]{asset_module_relpath}[/link]",
                    action_modes_cell,
                    end_section=(j + 1) == len(asset_classes),
                )
        return i
