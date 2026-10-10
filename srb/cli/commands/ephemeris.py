"""``srb ephemeris`` — query/download the optional SPICE ephemeris subsystem.

Parser construction stays Isaac-free and import-light: ``spiceypy``/``numpy``
(via ``srb.core.ephemeris``) are only imported inside ``execute()``, never at
parser-build time, so ``srb --help`` and tab-completion remain fast even when
the optional ``spice`` extra is not installed.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter


class EphemerisCommand(BaseCommand):
    """Query or download real sun/planet geometry from NAIF SPICE kernels."""

    NAME = "ephemeris"
    HELP = "Real sun/planet geometry from NAIF SPICE kernels (optional feature)"

    @classmethod
    def _create_parser(
        cls,
        subparsers: argparse._SubParsersAction,
        *,
        name: str,
        is_alias: bool = False,
        **_kwargs,
    ) -> argparse.ArgumentParser:
        parser = subparsers.add_parser(
            name,
            help=f"Alias: {cls.NAME}" if is_alias else cls.HELP,
            formatter_class=SrbHelpFormatter,
        )
        actions = parser.add_subparsers(
            title="Ephemeris actions",
            dest="ephemeris_action",
            required=True,
        )

        actions.add_parser(
            "download",
            help="Fetch the pinned kernel bundle (~43 MB) into the cache",
            formatter_class=SrbHelpFormatter,
        )

        info = actions.add_parser(
            "info",
            help="Print az/el/distance/angular-diameter for a site+epoch",
            formatter_class=SrbHelpFormatter,
        )
        info.add_argument(
            "--body",
            required=True,
            choices=("moon", "mars", "earth"),
            help="Observer's home body (surface site)",
        )
        info.add_argument(
            "--lat", type=float, required=True, help="Site latitude [deg]"
        )
        info.add_argument(
            "--lon", type=float, required=True, help="Site longitude [deg]"
        )
        info.add_argument("--alt", type=float, default=0.0, help="Site altitude [m]")
        info.add_argument(
            "--utc",
            required=True,
            help="ISO-8601 epoch, e.g. 2026-11-01T12:00:00",
        )
        info.add_argument(
            "--bodies",
            default="sun,earth",
            help="Comma-separated bodies to query in addition to the geometry",
        )

        imp = actions.add_parser(
            "import",
            help="Import an external illumination product onto a terrain-patch grid",
            formatter_class=SrbHelpFormatter,
        )
        imp.add_argument(
            "--source",
            required=True,
            choices=("lunarlab-zarr", "lunarlab-npz", "pgda-lpsr"),
            help="External illumination-product format to import",
        )
        imp.add_argument("--input", required=True, help="Path to the source product")
        imp.add_argument(
            "--baked-dir",
            default=None,
            help=(
                "Baked terrain patch directory to target "
                "(grid/site inferred from its meta.json)"
            ),
        )
        imp.add_argument(
            "--out",
            default=None,
            help="Output .npz path (required in explicit-target mode)",
        )
        imp.add_argument("--crs", default=None, help="Explicit-target mode: target CRS")
        imp.add_argument(
            "--center-xy",
            default=None,
            help="Explicit-target mode: 'CX,CY' patch center in the target CRS [m]",
        )
        imp.add_argument(
            "--size-m",
            type=float,
            default=None,
            help="Explicit-target mode: patch size [m]",
        )
        imp.add_argument(
            "--lat",
            type=float,
            default=None,
            help="Site latitude [deg] (azimuth correction)",
        )
        imp.add_argument(
            "--lon",
            type=float,
            default=None,
            help="Site longitude [deg] (azimuth correction)",
        )
        imp.add_argument(
            "--gsd-m",
            type=float,
            default=None,
            help="Target ground sample distance [m]",
        )
        imp.add_argument(
            "--epoch-range",
            default=None,
            help="'T0,T1' ISO-8601 UTC epoch bounds to keep (inclusive)",
        )
        imp.add_argument(
            "--epoch-stride", type=int, default=1, help="Keep every Nth epoch"
        )
        imp.add_argument(
            "--body",
            choices=("moon", "mars", "earth"),
            default="moon",
            help="Target body the product's terrain patch sits on",
        )

        return parser

    @classmethod
    def execute(
        cls,
        ephemeris_action: str,
        body: str | None = None,
        lat: float | None = None,
        lon: float | None = None,
        alt: float = 0.0,
        utc: str | None = None,
        bodies: str = "sun,earth",
        source: str | None = None,
        input: str | None = None,
        baked_dir: str | None = None,
        out: str | None = None,
        crs: str | None = None,
        center_xy: str | None = None,
        size_m: float | None = None,
        gsd_m: float | None = None,
        epoch_range: str | None = None,
        epoch_stride: int = 1,
        **_kwargs,
    ) -> None:
        if ephemeris_action == "download":
            from srb.cli.capabilities import require_ephemeris

            require_ephemeris()
            cls._execute_download()
        elif ephemeris_action == "info":
            from srb.cli.capabilities import require_ephemeris

            require_ephemeris()
            cls._execute_info(
                body=body, lat=lat, lon=lon, alt=alt, utc=utc, bodies=bodies
            )
        elif ephemeris_action == "import":
            cls._execute_import(
                source=source,
                input=input,
                baked_dir=baked_dir,
                out=out,
                crs=crs,
                center_xy=center_xy,
                size_m=size_m,
                lat=lat,
                lon=lon,
                gsd_m=gsd_m,
                epoch_range=epoch_range,
                epoch_stride=epoch_stride,
                # The parser defaults `--body` to "moon", so it supplies a non-None value.
                body=body,
            )
        else:
            raise ValueError(f"unknown ephemeris action: {ephemeris_action!r}")

    @classmethod
    def _execute_download(cls) -> None:
        from srb.core.ephemeris.kernels import download_kernels

        try:
            paths = download_kernels()
        except (ValueError, OSError) as e:
            print(f"srb ephemeris download: {e}", file=sys.stderr)
            raise SystemExit(1) from e
        for path in paths:
            print(f"ok {path.name}  {path.stat().st_size / 1e6:.1f} MB")

    @classmethod
    def _execute_info(
        cls,
        *,
        body: str | None,
        lat: float | None,
        lon: float | None,
        alt: float,
        utc: str | None,
        bodies: str,
    ) -> None:
        from srb.core.ephemeris import SiteSpec, query
        from srb.core.ephemeris.kernels import KernelsMissingError

        site = SiteSpec(body=body, lat_deg=lat, lon_deg=lon, alt_m=alt)
        try:
            state = query(site, utc, bodies=tuple(bodies.split(",")))
        except (KernelsMissingError, ValueError, OSError) as e:
            print(f"srb ephemeris info: {e}", file=sys.stderr)
            raise SystemExit(1) from e
        for body_name, body_state in state.bodies.items():
            print(
                f"{body_name}: az={body_state.azimuth_deg:.3f}° "
                f"el={body_state.elevation_deg:.3f}° "
                f"dist={body_state.distance_m / 1e3:.0f} km "
                f"diam={body_state.angular_diameter_deg:.3f}°"
            )
        print(f"irradiance_scale={state.irradiance_scale:.4f}")

    @classmethod
    def _execute_import(
        cls,
        *,
        source: str | None,
        input: str | None,
        baked_dir: str | None,
        out: str | None,
        crs: str | None,
        center_xy: str | None,
        size_m: float | None,
        lat: float | None,
        lon: float | None,
        gsd_m: float | None,
        epoch_range: str | None,
        epoch_stride: int,
        body: str = "moon",
    ) -> None:
        if baked_dir is not None and (lat is not None or lon is not None):
            print(
                "srb ephemeris import: --baked-dir derives the site lat/lon "
                "from the bake's meta.json; --lat/--lon are only for "
                "explicit-target mode (remove them)",
                file=sys.stderr,
            )
            raise SystemExit(2)
        for flag, value in (
            ("--size-m", size_m),
            ("--lat", lat),
            ("--lon", lon),
            ("--gsd-m", gsd_m),
        ):
            if value is not None and not math.isfinite(value):
                print(
                    f"srb ephemeris import: {flag} must be a finite number, "
                    f"got {value}",
                    file=sys.stderr,
                )
                raise SystemExit(2)
        if size_m is not None and size_m <= 0:
            print(
                f"srb ephemeris import: --size-m must be positive, got {size_m}",
                file=sys.stderr,
            )
            raise SystemExit(2)
        if gsd_m is not None and gsd_m <= 0:
            print(
                f"srb ephemeris import: --gsd-m must be positive, got {gsd_m}",
                file=sys.stderr,
            )
            raise SystemExit(2)
        if size_m is not None and gsd_m is not None:
            ratio = size_m / gsd_m
            remainder = abs(ratio - round(ratio))
            if remainder > 1e-9:
                print(
                    f"srb ephemeris import: --size-m {size_m} is not an "
                    f"integer multiple of --gsd-m {gsd_m} (remainder "
                    f"{size_m % gsd_m}); the product grid would skew the "
                    "patch center by a sub-pixel offset",
                    file=sys.stderr,
                )
                raise SystemExit(2)
        if source == "pgda-lpsr" and (epoch_range is not None or epoch_stride != 1):
            print(
                "srb ephemeris import: --epoch-range/--epoch-stride do not "
                "apply to pgda-lpsr (static-only product; it has no epoch "
                "axis)",
                file=sys.stderr,
            )
            raise SystemExit(2)
        # PGDA LPSR is Moon-only; unlike LunarLab products, it cannot target
        # other bodies. Reject mismatched body/source pairs.
        if source == "pgda-lpsr" and body != "moon":
            print(
                f"srb ephemeris import: --source pgda-lpsr is a moon-only "
                f"product (PGDA Lunar Polar Shadowed Region); got --body "
                f"{body!r}",
                file=sys.stderr,
            )
            raise SystemExit(2)

        from srb.core.ephemeris.importers import (
            import_lunarlab_npz,
            import_lunarlab_zarr,
            import_pgda_lpsr,
        )
        from srb.core.ephemeris.products import ProductGrid

        importers = {
            "lunarlab-zarr": import_lunarlab_zarr,
            "lunarlab-npz": import_lunarlab_npz,
            "pgda-lpsr": import_pgda_lpsr,
        }
        importer = importers[source]

        kwargs: dict = {"gsd_m": gsd_m, "body": body}

        if baked_dir is not None:
            kwargs["baked_dir"] = Path(baked_dir)
            if out is not None:
                kwargs["out_path"] = Path(out)
        else:
            required = {
                "--out": out,
                "--crs": crs,
                "--center-xy": center_xy,
                "--size-m": size_m,
                "--lat": lat,
                "--lon": lon,
                "--gsd-m": gsd_m,
            }
            missing = [flag for flag, value in required.items() if value is None]
            if missing:
                print(
                    "srb ephemeris import: explicit-target mode (no --baked-dir) "
                    f"requires {', '.join(missing)}",
                    file=sys.stderr,
                )
                raise SystemExit(2)

            try:
                cx_str, cy_str = center_xy.split(",")
                cx, cy = float(cx_str), float(cy_str)
                if not (math.isfinite(cx) and math.isfinite(cy)):
                    raise ValueError(center_xy)
            except ValueError:
                print(
                    "srb ephemeris import: --center-xy must be two numbers "
                    f"'X,Y' in metres, got {center_xy!r}",
                    file=sys.stderr,
                )
                raise SystemExit(2) from None
            g = float(gsd_m)
            size = float(size_m)
            n = round(size / g)
            grid = ProductGrid(
                crs=crs,
                transform=(g, 0.0, cx - size / 2.0, 0.0, -g, cy + size / 2.0),
                shape=(n, n),
                patch_center_xy_m=(cx, cy),
            )
            kwargs["target"] = grid
            kwargs["out_path"] = Path(out)
            kwargs["site_latlon"] = (lat, lon)

        if source != "pgda-lpsr":
            kwargs["epoch_stride"] = epoch_stride
            kwargs["epoch_range"] = (
                tuple(epoch_range.split(",")) if epoch_range is not None else None
            )

        try:
            result = importer(input, **kwargs)
        except (ValueError, OSError, ImportError) as e:
            print(f"srb ephemeris import: {e}", file=sys.stderr)
            raise SystemExit(1) from e

        print(result)
