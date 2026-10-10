"""Preview a region's resolved v2 ``CompositionPlan`` without network access.

The plan's ``consented`` and ``consent_required_layer_ids`` fields are
descriptive only. Fetches independently enforce per-layer consent through
``check_layer_consent``; ``--allow-large`` here changes the preview only.
"""

from __future__ import annotations

import dataclasses
import json as json_module
import sys
from pathlib import Path
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from srb.terrain.catalog.resolve import (
        CompositionPlan,
        LayerContribution,
        RegionRequest,
    )

# --- fixed-width table ------------------------------------------------------
#
# Fixed widths and locale-independent values keep the table deterministic.

_ROLE_W = 5
_ID_W = 22
_GSD_W = 5
_PRIO_W = 4
_COV_W = 8
_FETCH_W = 11


def _fmt_bytes_mb(n: int) -> str:
    """Render bytes in decimal MB with one decimal place."""
    return f"{n / 1e6:.1f} MB"


def _fmt_pct(fraction: float) -> str:
    return f"{fraction * 100:.1f}%"


def _row(
    idx: str,
    role: str,
    layer_id: str,
    gsd: str,
    prio: str,
    coverage: str,
    fetch: str,
    access: str,
) -> str:
    return (
        f"    {idx:>2}  {role:<{_ROLE_W}}  {layer_id:<{_ID_W}}  {gsd:>{_GSD_W}}  "
        f"{prio:>{_PRIO_W}}  {coverage:>{_COV_W}}  {fetch:>{_FETCH_W}}  {access}"
    )


_COLUMN_HEADER = _row(
    "#", "role", "layer_id", "gsd_m", "prio", "coverage", "fetch", "access"
)


_SHARED_ARCHIVE_NOTE = " (shared archive)"
"""Two `access: zip_member` layers over ONE archive (the
real case: the Zenodo SfS A3CLR DEM + orthomosaic pair, 2.2 GB) each carry
`est_fetch_bytes = <the whole archive>`, because fetching either one
downloads the whole archive -- while `total fetch` counts that archive ONCE
(`resolve._fetch_dedup_key`, rule 9). Unannotated, the table printed two
2.2 GB rows above a 2.2 GB total, which reads as an arithmetic error. The
dedup is correct and the byte figures are correct; only the presentation
was missing the reason they do not add up."""


def _contribution_row(
    i: int, c: LayerContribution, shared: frozenset[str] = frozenset()
) -> str:
    # Append after the final column so fixed-width alignment stays intact.
    row = _row(
        str(i),
        c.role,
        c.layer_id,
        f"{c.native_gsd_m:.1f}",
        str(c.priority_class),
        _fmt_pct(c.coverage_fraction),
        _fmt_bytes_mb(c.est_fetch_bytes),
        c.access,
    )
    return row + _SHARED_ARCHIVE_NOTE if c.layer_id in shared else row


def _optional_section(
    title: str,
    stack: tuple[LayerContribution, ...],
    shared: frozenset[str] = frozenset(),
) -> list[str]:
    """Format an optional stack section, omitting it when empty."""
    if not stack:
        return []
    return [
        f"  {title}",
        *[_contribution_row(i, c, shared) for i, c in enumerate(stack)],
    ]


def _deshade_status_line(deshade: bool) -> str:
    """Format deshade status from the region request flag."""
    return f"  deshade: {'on (request)' if deshade else 'off'}"


def format_plan(
    plan: CompositionPlan,
    *,
    deshade: bool = False,
    shared_fetch_layer_ids: frozenset[str] = frozenset(),
) -> str:
    """Format a non-empty plan, including optional shared-fetch annotations.

    ``run`` handles empty plans. ``deshade`` reflects the request flag, and
    ``shared_fetch_layer_ids`` marks rows whose archive is counted once in
    the total.
    """
    request = plan.request
    lines = [
        f"composition plan: {request.body} ({request.lat_deg:.6f}, "
        f"{request.lon_deg:.6f}) size={request.size_m:g}m",
        f"  crs: {plan.crs}   center_xy_m: ({plan.center_xy_m[0]:.3f}, "
        f"{plan.center_xy_m[1]:.3f})   target_gsd_m: {plan.target_gsd_m:.1f}",
        "",
        "  DEM stack",
        _COLUMN_HEADER,
        *[
            _contribution_row(i, c, shared_fetch_layer_ids)
            for i, c in enumerate(plan.dem_stack)
        ],
    ]
    for title, stack in (
        ("color stack (sunlit)", plan.color_stack),
        ("color stack (PSR)", plan.color_stack_psr),
        ("mask layers", plan.mask_layers),
        ("quality layers", plan.quality_layers),
    ):
        lines.extend(_optional_section(title, stack, shared_fetch_layer_ids))

    lines.append("")
    lines.append(
        # Totals count each distinct fetch unit once, not each layer.
        f"  total fetch: {_fmt_bytes_mb(plan.total_fetch_bytes)} (unique downloads)"
    )
    ids = plan.consent_required_layer_ids
    lines.append(f"  consent required: {', '.join(ids) if ids else 'none'}")
    lines.append(_deshade_status_line(deshade))
    return "\n".join(lines)


def plan_as_json(plan: CompositionPlan) -> dict:
    """Return the plan fields unchanged for ``--json`` output."""
    return dataclasses.asdict(plan)


_JSON_CONSENT_DISCLAIMER = (
    "warning: this JSON's 'consented'/'consent_required_layer_ids' fields "
    "are a DESCRIPTIVE preview only (allow_large OR SRB_TERRAIN_LARGE_OK) "
    "-- not an authorization gate. The real, independent fetch-time "
    "consent check (check_layer_consent/layer_max_bytes) never reads them. "
    "See `srb asset stack --help` / docs/src/development/real_terrain_assets.md."
)


def _fallback_message(request: RegionRequest) -> str:
    """Return the fallback line for a request without v2 DEM coverage."""
    return (
        f"no v2 DEM coverage for {request.body} at "
        f"({request.lat_deg:.6f}, {request.lon_deg:.6f}) — falling back "
        "to the v1 single-source path"
    )


def run(
    *,
    body: str,
    lat: float,
    lon: float,
    size: float,
    lod: str = "medium",
    target_gsd: float | None = None,
    catalog: str | None = None,
    allow_large: bool = False,
    json: bool = False,
    deshade: bool = False,
    **_,
) -> int:
    """Print the resolved plan without fetching data.

    Consent fields describe the preview only; fetch-time checks remain
    independent. JSON output sends its consent disclaimer to stderr.
    """
    from srb.terrain.catalog.loader import load_source_catalog
    from srb.terrain.catalog.resolve import (
        RegionRequest,
        resolve_stacks,
        shared_fetch_unit_layer_ids,
    )
    from srb.terrain.schema import Body, Lod
    from srb.terrain.sources.acquisition import large_download_consented

    request = RegionRequest(
        body=cast(Body, body),
        lat_deg=lat,
        lon_deg=lon,
        size_m=size,
        target_gsd_m=target_gsd,
        lod=cast(Lod, lod),
    )
    source_catalog = load_source_catalog(
        body=cast(Body, body), path=Path(catalog) if catalog is not None else None
    )
    consented = allow_large or large_download_consented()
    plan = resolve_stacks(request, source_catalog, consented=consented)

    if json:
        print(json_module.dumps(plan_as_json(plan), indent=2))
        print(_JSON_CONSENT_DISCLAIMER, file=sys.stderr)
    elif plan.is_empty:
        if (
            plan.color_stack
            or plan.color_stack_psr
            or plan.mask_layers
            or plan.quality_layers
        ):
            print(
                format_plan(
                    plan,
                    deshade=deshade,
                    shared_fetch_layer_ids=shared_fetch_unit_layer_ids(
                        plan, source_catalog
                    ),
                )
            )
        print(_fallback_message(request))
        print(_deshade_status_line(deshade))
    else:
        print(
            format_plan(
                plan,
                deshade=deshade,
                shared_fetch_layer_ids=shared_fetch_unit_layer_ids(
                    plan, source_catalog
                ),
            )
        )
    return 0
