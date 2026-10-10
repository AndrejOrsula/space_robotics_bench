"""mdbook build wrapper."""

from __future__ import annotations

import shutil
from typing import Sequence

from srb.utils.path import SRB_DIR
from srb.utils.process_supervisor import ProcessKind

_DOCS_DIR = SRB_DIR / "docs"
_DOC_BUILD_TIMEOUT_S = 30.0 * 60.0


def execute_build(forwarded_args: Sequence[str] = (), **_kwargs) -> None:
    from srb.cli._utils import run_subprocess

    if not shutil.which("mdbook"):
        raise FileNotFoundError(
            'The "mdbook" tool is required to build the docs; install it with '
            "`cargo install mdbook`."
        )

    run_subprocess(
        ("mdbook", "build", _DOCS_DIR.as_posix(), *forwarded_args),
        description="Building the docs with the following command",
        kind=ProcessKind.FINITE,
        timeout=_DOC_BUILD_TIMEOUT_S,
    )
