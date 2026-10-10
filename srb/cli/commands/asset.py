"""``srb asset`` -- Manage assets (download, bake, inspect and clean caches)."""

from __future__ import annotations

import argparse
import sys

from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter
from srb.cli.commands._assets._catalog import (
    BODY_CHOICES,
    TERRAIN_LOD_CHOICES,
    asset_kind_choices,
    terrain_asset_kind_choices,
)


def _complete_asset_target(
    *, parsed_args: argparse.Namespace, **_: object
) -> list[str]:
    """Complete catalogued target names for the already-parsed asset kind."""
    kind = getattr(parsed_args, "kind", None)
    try:
        if kind == "terrain":
            from srb.terrain.manifest_loader import load_manifest

            return sorted(load_manifest().sites)
        if kind == "rocks":
            from srb.terrain.rocks.catalog_loader import load_rocks_catalog

            return sorted(load_rocks_catalog().sets)
        if kind == "hdri":
            from srb.terrain.hdri.catalog_loader import load_hdri_catalog

            return sorted(load_hdri_catalog().skydomes)
        if kind == "lights":
            from srb.terrain.lights.catalog_loader import load_lighting_catalog

            return sorted(load_lighting_catalog().presets)
    except Exception:  # noqa: BLE001 - completion must never break the shell
        return []
    return []


class AssetCommand(BaseCommand):
    """Manage assets: download, bake, inspect and clean asset caches."""

    NAME = "asset"
    HELP = "Manage assets"
    ALIASES = ("assets",)

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

        asset_subs = parser.add_subparsers(
            title="Asset actions",
            dest="asset_action",
            required=True,
        )

        # ---- list ----
        list_p = asset_subs.add_parser(
            "list",
            help="List available assets",
            formatter_class=SrbHelpFormatter,
        )
        list_p.add_argument(
            "kind", choices=asset_kind_choices(), help="Asset kind to list"
        )
        list_p.add_argument(
            "--body",
            choices=BODY_CHOICES,
            default=None,
            help="Restrict the listing to one celestial body",
        )

        # ---- info ----
        info_p = asset_subs.add_parser(
            "info",
            help="Show details for one asset",
            formatter_class=SrbHelpFormatter,
        )
        info_p.add_argument(
            "kind", choices=asset_kind_choices(), help="Asset kind to inspect"
        )
        info_p.add_argument(
            "target", help="site / set / dome / preset name"
        ).completer = _complete_asset_target  # type: ignore[attr-defined]

        # ---- doctor ----
        doctor_p = asset_subs.add_parser(
            "doctor",
            help="Preflight asset cache health",
            formatter_class=SrbHelpFormatter,
        )
        doctor_p.add_argument(
            "kind", choices=terrain_asset_kind_choices(), help="Asset kind to verify"
        )
        doctor_p.add_argument(
            "target", nargs="?", default=None, help="Terrain site name to verify"
        ).completer = _complete_asset_target  # type: ignore[attr-defined]
        doctor_p.add_argument(
            "--body",
            choices=BODY_CHOICES,
            default=None,
            help="Celestial body the site belongs to",
        )
        doctor_p.add_argument("--patch", default="default", help="Patch name to verify")
        doctor_p.add_argument(
            "--lod",
            choices=TERRAIN_LOD_CHOICES,
            default="medium",
            help="Level of detail to verify",
        )
        doctor_p.add_argument(
            "--patch-size", type=float, default=None, help="Patch size in meters"
        )
        doctor_p.add_argument(
            "--center", default=None, help="x,y in meters (e.g. 100.0,200.0)"
        )
        doctor_p.add_argument(
            "--no-ortho",
            action="store_true",
            help="Skip checking/generating the orthophoto",
        )
        doctor_p.add_argument(
            "--seed", type=int, default=0, help="Random seed for procedurals"
        )
        doctor_p.add_argument(
            "--deshade",
            action="store_true",
            help=(
                "Assess the illumination-corrected (deshaded) variant of "
                "the bake -- deshade is cache-keyed, so a deshaded "
                "request has its own baked/not-baked status"
            ),
        )

        # ---- download ----
        dl_p = asset_subs.add_parser(
            "download",
            help="Fetch and bake an asset",
            description=(
                "Fetch and bake an asset. Terrain takes either a curated site (`srb "
                "asset download terrain <site>`) or a region (`--lat <deg> --lon <deg>"
                " --size-m <m>`), not both."
            ),
            formatter_class=SrbHelpFormatter,
        )
        dl_p.add_argument(
            "kind", choices=asset_kind_choices(), help="Asset kind to fetch and bake"
        )
        dl_p.add_argument(
            "target",
            nargs="?",
            default=None,
            help=(
                "curated name (e.g. a terrain site); omit when using the "
                "--lat/--lon/--size-m region form"
            ),
        ).completer = _complete_asset_target  # type: ignore[attr-defined]
        dl_p.add_argument(
            "--all",
            action="store_true",
            help="Fetch and bake every registered asset of this kind",
        )
        dl_p.add_argument(
            "--body",
            choices=BODY_CHOICES,
            default=None,
            help="(Terrain only) Celestial body name",
        )
        dl_p.add_argument("--patch", default=None, help="(Terrain only) Patch name")
        dl_p.add_argument(
            "--lod",
            choices=TERRAIN_LOD_CHOICES,
            default="medium",
            help="(Terrain only) Level of detail",
        )
        dl_p.add_argument(
            "--patch-size",
            type=float,
            default=None,
            help="(Terrain only) Patch size in meters",
        )
        dl_p.add_argument(
            "--center", default=None, help="(Terrain only) x,y center in meters"
        )
        dl_p.add_argument(
            "--no-ortho",
            action="store_true",
            help="(Terrain only) Skip orthophoto baking",
        )
        dl_p.add_argument(
            "--seed",
            type=int,
            default=None,
            help="(Terrain only) Random seed for procedurals",
        )
        dl_p.add_argument(
            "--force",
            action="store_true",
            help="Re-download and re-bake even when a valid cache entry exists",
        )
        dl_p.add_argument(
            "--debug",
            action="store_true",
            help="Verbose debug logging of the download/bake pipelines",
        )
        # Terrain regions require all three flags and exclude target/--all;
        # `_assets/_download.py` validates this combination.
        dl_p.add_argument(
            "--lat",
            type=float,
            default=None,
            help="Region latitude in degrees (requires --lon and --size-m)",
        )
        dl_p.add_argument(
            "--lon",
            type=float,
            default=None,
            help="Region longitude in degrees (requires --lat and --size-m)",
        )
        dl_p.add_argument(
            "--size-m",
            type=float,
            default=None,
            help="Region size in meters (requires --lat and --lon)",
        )
        dl_p.add_argument(
            "--deshade",
            action="store_true",
            help=(
                "Bake illumination-corrected (deshaded) color (cache-keyed). Omitting "
                "it never disables a site's manifest `deshade: true`."
            ),
        )
        # Location variety is region-only, like --patch-size and --center.
        # `_validate_region_form` enforces this.
        dl_p.add_argument(
            "--num-locations",
            type=int,
            default=1,
            help="Bake K location-variety patches around --lat/--lon "
            "(region form only; K > 1 derives deterministic offsets within "
            "--location-spread-m, default --size-m)",
        )
        dl_p.add_argument(
            "--location-spread-m",
            type=float,
            default=None,
            help="Offset-disk radius in metres for --num-locations > 1 "
            "(default: --size-m)",
        )

        # ---- bake (user-supplied DEM) ----
        bake_p = asset_subs.add_parser(
            "bake",
            help="Bake from a user-supplied DEM",
            formatter_class=SrbHelpFormatter,
        )
        bake_p.add_argument(
            "kind", choices=terrain_asset_kind_choices(), help="Asset kind to bake"
        )
        bake_p.add_argument(
            "--dem", required=True, help="Path to the user-supplied DEM raster"
        )
        bake_p.add_argument(
            "--body",
            choices=BODY_CHOICES,
            required=True,
            help="Target celestial body",
        )
        bake_p.add_argument(
            "--site-name",
            required=True,
            help="Name to register this custom terrain site under",
        )
        bake_p.add_argument(
            "--patch-size", type=float, default=None, help="Patch size in meters"
        )
        bake_p.add_argument(
            "--lod",
            choices=TERRAIN_LOD_CHOICES,
            default="medium",
            help="Level of detail for the baked mesh",
        )
        bake_p.add_argument(
            "--attribution",
            required=True,
            help="Data source/attribution string embedded in the metadata",
        )
        bake_p.add_argument(
            "--seed",
            type=int,
            default=0,
            help="Random seed for procedural rocks/displacement",
        )
        bake_p.add_argument(
            "--debug",
            action="store_true",
            help="Verbose debug logging during mesh generation",
        )

        # ---- clean ----
        clean_p = asset_subs.add_parser(
            "clean",
            help="Remove raw archives or baked caches",
            formatter_class=SrbHelpFormatter,
        )
        clean_p.add_argument(
            "kind", choices=asset_kind_choices(), help="Asset kind to clean"
        )
        clean_p.add_argument(
            "target",
            nargs="?",
            default=None,
            help="Clean only this baked target (cannot be combined with --raw or --all)",
        ).completer = _complete_asset_target  # type: ignore[attr-defined]
        clean_p.add_argument(
            "--raw",
            action="store_true",
            help="Remove shared content-addressed raw archives (omit target)",
        )
        clean_p.add_argument(
            "--baked",
            action="store_true",
            help="Remove only the baked USD/USDZ cache",
        )
        clean_p.add_argument(
            "--all",
            action="store_true",
            help="Remove shared raw archives and baked entries (omit target)",
        )

        # ---- validate-manifest ----
        val_p = asset_subs.add_parser(
            "validate-manifest",
            help="Validate the curated-site manifest",
            formatter_class=SrbHelpFormatter,
        )
        val_p.add_argument(
            "--manifest",
            default=None,
            help="Path to a custom manifest YAML (default: the packaged one)",
        )
        val_p.add_argument(
            "--check-urls",
            action="store_true",
            help="Send HTTP requests to verify every mirror is reachable",
        )
        val_p.add_argument(
            "--check-shas",
            action="store_true",
            help="Verify downloaded raw datasets against the manifest SHA256s",
        )

        # ---- cache-stats ----
        asset_subs.add_parser(
            "cache-stats",
            help="Print cache size and entries",
            formatter_class=SrbHelpFormatter,
        )

        # ---- stack ----
        stack_p = asset_subs.add_parser(
            "stack",
            help="Preview the resolved v2 composition plan (prints only, never downloads)",
            formatter_class=SrbHelpFormatter,
        )
        stack_p.add_argument(
            "--body",
            choices=BODY_CHOICES,
            required=True,
            help="Target celestial body",
        )
        stack_p.add_argument(
            "--lat", type=float, required=True, help="Latitude in degrees"
        )
        stack_p.add_argument(
            "--lon", type=float, required=True, help="Longitude in degrees"
        )
        stack_p.add_argument(
            "--size-m",
            "--size",
            dest="size",
            type=float,
            required=True,
            help="Region size in meters",
        )
        stack_p.add_argument(
            "--lod",
            choices=TERRAIN_LOD_CHOICES,
            default="medium",
            help="Level of detail",
        )
        stack_p.add_argument(
            "--target-gsd",
            type=float,
            default=None,
            help="Override the target ground-sample distance in meters",
        )
        stack_p.add_argument(
            "--catalog",
            type=str,
            default=None,
            help="Override the shipped catalog with a YAML file at PATH",
        )
        stack_p.add_argument(
            "--allow-large",
            action="store_true",
            help=(
                "preview the plan as if large downloads were approved "
                "(this command never downloads)"
            ),
        )
        stack_p.add_argument(
            "--json",
            action="store_true",
            help="Emit the plan as JSON instead of a formatted table",
        )
        stack_p.add_argument(
            "--deshade",
            action="store_true",
            help=(
                "preview with deshading requested; a region request "
                "has no manifest entry to consult, so this is always the "
                "printed effective value for the preview"
            ),
        )

        # ---- prefetch-fixtures ----
        asset_subs.add_parser(
            "prefetch-fixtures",
            help="Pre-bake all manifest sites at lod=low",
            formatter_class=SrbHelpFormatter,
        )

        return parser

    @classmethod
    def execute(
        cls,
        **kwargs,
    ) -> None:
        action = kwargs.pop("asset_action")
        hydra_args = kwargs.get("hydra_args") or ()
        if hydra_args:
            unexpected = ", ".join(repr(token) for token in hydra_args)
            print(
                f"srb asset {action}: unexpected Hydra-style arguments: {unexpected}",
                file=sys.stderr,
            )
            raise SystemExit(2)
        kwargs.pop("hydra_args", None)
        kwargs.pop("forwarded_args", None)

        # These actions delegate to _assets/.
        from srb.cli.commands._assets._common import print_error
        from srb.terrain.errors import TerrainError

        try:
            sys.exit(cls._dispatch_assets(action, **kwargs))
        except TerrainError as e:
            print_error(e, debug=kwargs.get("debug", False))
            sys.exit(1)

    @classmethod
    def _dispatch_assets(cls, action: str, **kwargs) -> int:
        # Each handler module ships a ``run(**kwargs) -> int`` entry point.
        if action == "validate-manifest":
            from srb.cli.commands._assets._validate import run
        elif action == "list":
            from srb.cli.commands._assets._list import run
        elif action == "info":
            from srb.cli.commands._assets._info import run
        elif action == "doctor":
            from srb.cli.commands._assets._doctor import run
        elif action == "download":
            from srb.cli.commands._assets._download import run
        elif action == "bake":
            from srb.cli.commands._assets._bake import run
        elif action == "clean":
            from srb.cli.commands._assets._clean import run
        elif action == "cache-stats":
            from srb.cli.commands._assets._cache_stats import run
        elif action == "stack":
            from srb.cli.commands._assets._stack import run
        elif action == "prefetch-fixtures":
            from srb.cli.commands._assets._prefetch import run
        else:
            raise ValueError(f"unknown asset action: {action!r}")
        return run(**kwargs)
