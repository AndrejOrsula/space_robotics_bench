"""``srb doc`` — Documentation generation, serving, and building."""

from __future__ import annotations

import argparse
import shutil
from typing import Sequence

from srb.cli._base import BaseCommand
from srb.cli._parsing import SrbHelpFormatter
from srb.utils.path import SRB_IS_SOURCE_CHECKOUT


class DocsCommand(BaseCommand):
    """Generate, build, and serve the SRB documentation."""

    NAME = "doc"
    ALIASES = ("docs",)
    HELP = "Build, serve, and generate documentation"

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

        doc_subs = parser.add_subparsers(
            title="Documentation actions",
            dest="doc_action",
            required=False,
        )
        # Serve when no action is specified.
        parser.set_defaults(doc_action="serve")

        # --- serve ---
        _mdbook_note = "" if shutil.which("mdbook") else ' (MISSING: "mdbook" tool)'
        doc_subs.add_parser(
            "serve",
            help=f"Live-reload documentation server{_mdbook_note}",
            formatter_class=SrbHelpFormatter,
        )

        # --- build ---
        doc_subs.add_parser(
            "build",
            help=f"Build static documentation{_mdbook_note}",
            formatter_class=SrbHelpFormatter,
        )

        # --- docgen ---
        docgen_parser = doc_subs.add_parser(
            "docgen",
            help="Generate enriched documentation pages from code introspection",
            formatter_class=SrbHelpFormatter,
        )
        docgen_parser.add_argument(
            "--static-only",
            action="store_true",
            default=False,
            help="Check docs/data TOML files and print counts without launching or writing pages",
        )
        # These scopes are mutually exclusive; either one skips the other's pages.
        docgen_scope = docgen_parser.add_mutually_exclusive_group()
        docgen_scope.add_argument(
            "--assets-only",
            action="store_true",
            default=False,
            help="Generate only asset pages (skip tasks + cross-cutting)",
        )
        docgen_scope.add_argument(
            "--tasks-only",
            action="store_true",
            default=False,
            help="Generate only task pages (skip assets + cross-cutting)",
        )

        # --- thumbnail ---
        thumb_parser = doc_subs.add_parser(
            "thumbnail",
            help="Render asset or task thumbnail images (requires Isaac Sim)",
            formatter_class=SrbHelpFormatter,
        )
        thumb_parser.add_argument("--width", type=int, default=1024, help="Image width")
        thumb_parser.add_argument(
            "--height", type=int, default=768, help="Image height"
        )
        thumb_parser.add_argument(
            "--output-dir", type=str, default=None, help="Output directory"
        )
        thumb_parser.add_argument(
            "--asset", type=str, default=None, help="Render single asset"
        )
        thumb_parser.add_argument(
            "--task",
            type=str,
            nargs="+",
            default=None,
            help="Render task environment thumbnails by task ID; use ALL for every cached task",
        )
        thumb_parser.add_argument(
            "--asset-type",
            type=str,
            nargs="+",
            default=None,
            choices=["robot", "object", "scenery"],
            help="Asset types to render",
        )
        thumb_parser.add_argument(
            "--force",
            action="store_true",
            default=False,
            help="Re-render even if file exists",
        )
        thumb_parser.add_argument(
            "--allow-failures",
            action="store_true",
            default=False,
            help=(
                "Keep a task thumbnail batch successful even when some tasks fail; "
                "failures are still reported in the final summary"
            ),
        )

        return parser

    @classmethod
    def execute(
        cls,
        doc_action: str = "serve",
        forwarded_args: Sequence[str] = (),
        **kwargs,
    ) -> None:
        if not SRB_IS_SOURCE_CHECKOUT:
            raise RuntimeError("Documentation commands require an SRB source checkout")
        match doc_action:
            case "serve":
                from srb.cli.commands._doc._serve import execute_serve

                execute_serve(forwarded_args=forwarded_args, **kwargs)
            case "build":
                from srb.cli.commands._doc._build import execute_build

                execute_build(forwarded_args=forwarded_args, **kwargs)
            case "docgen":
                if forwarded_args:
                    raise ValueError(
                        "`srb doc docgen` does not accept forwarded arguments: "
                        f"{list(forwarded_args)!r}"
                    )
                from srb.cli.commands._doc._docgen import execute_docgen

                execute_docgen(**kwargs)
            case "thumbnail":
                if forwarded_args:
                    raise ValueError(
                        "`srb doc thumbnail` does not accept forwarded arguments: "
                        f"{list(forwarded_args)!r}"
                    )
                from srb.cli.commands._doc._thumbnail import execute_thumbnail

                execute_thumbnail(**kwargs)
            case _:
                raise ValueError(f"Unknown doc action: {doc_action}")
