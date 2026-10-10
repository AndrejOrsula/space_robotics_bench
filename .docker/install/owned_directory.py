#!/usr/bin/env python3
"""CLI bridge for installer-owned directories."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT))

from srb.utils.owned_directory import (  # noqa: E402
    OwnedDirectoryError,
    claim_owned_directory,
    remove_owned_directory,
    validate_owned_directory,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Mark or remove an installer-created SRB directory",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)
    for action in ("mark-created", "validate", "remove"):
        command = subparsers.add_parser(action)
        command.add_argument("--path", required=True, type=Path)
        command.add_argument("--owner", required=True)
    return parser


def _exclude_marker_from_git(root: Path) -> None:
    exclude = root / ".git" / "info" / "exclude"
    if not exclude.is_file() or exclude.is_symlink():
        return
    marker_rule = "/.srb-owned-directory.json"
    existing = exclude.read_text(encoding="utf-8").splitlines()
    if marker_rule in existing:
        return
    with exclude.open("a", encoding="utf-8") as stream:
        if exclude.stat().st_size:
            stream.write("\n")
        stream.write(f"{marker_rule}\n")


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.action == "mark-created":
            root = claim_owned_directory(
                args.path,
                owner=args.owner,
                protected_roots=(REPOSITORY_ROOT,),
            )
            _exclude_marker_from_git(root)
        elif args.action == "validate":
            validate_owned_directory(
                args.path,
                owner=args.owner,
                protected_roots=(REPOSITORY_ROOT,),
            )
        else:
            remove_owned_directory(
                args.path,
                owner=args.owner,
                protected_roots=(REPOSITORY_ROOT,),
            )
    except (OSError, UnicodeError, OwnedDirectoryError) as exc:
        print(f"owned-directory operation refused: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
