"""mdbook serve wrapper."""

from __future__ import annotations

import shutil
from typing import Sequence

from srb.utils.path import SRB_DIR
from srb.utils.process_supervisor import ProcessKind

_DOCS_DIR = SRB_DIR / "docs"


def execute_serve(forwarded_args: Sequence[str] = (), **_kwargs) -> None:
    from srb.cli._utils import run_subprocess

    if not shutil.which("mdbook"):
        raise FileNotFoundError(
            'The "mdbook" tool is required to serve the docs; install it with '
            "`cargo install mdbook`."
        )

    run_subprocess(
        ("mdbook", "serve", _DOCS_DIR.as_posix(), "--open", *forwarded_args),
        description="Serving the docs with the following command",
        kind=ProcessKind.INTERACTIVE,
        timeout=None,
    )
