import subprocess

import pytest

from srb.utils import logging
from srb.utils.cache import open_offline_srb_cache_snapshot
from srb.utils.failures import IntegrityFailureError
from srb.utils.isaacsim import get_kit_python
from srb.utils.publication import PublicationNotFoundError


@pytest.mark.order(2)
def test_cli_ls():
    try:
        open_offline_srb_cache_snapshot()
    except PublicationNotFoundError:
        pass  # Fresh clone has no cache; `srb ls` must still work.
    except IntegrityFailureError as exc:
        cause = str(exc.info.details.get("cause", "")).lower()
        if "permission denied" in cause:
            pytest.skip("offline cache is not readable in this environment")
        pytest.fail(f"Offline cache integrity invalid before ls command: {exc}")

    cmd = (
        get_kit_python(),
        "-m",
        "srb",
        "ls",
    )

    try:
        process = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=600.0,
        )
        stdout = process.stdout or ""
        stderr = process.stderr or ""

        if process.returncode != 0:
            logging.critical(f"Failed command: {' '.join(cmd)}")
            pytest.fail(f"Process failed\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}")

        assert "SRB offline registry snapshot" in stdout
        assert "Robots:" in stdout
        assert "Environments:" in stdout
        # A fresh clone may have an empty snapshot; the command contract still applies.
        assert process.returncode == 0

        # Allow ordinary stderr noise, but reject hard process faults.
        lower = stderr.lower()
        if any(
            token in lower
            for token in (
                "traceback",
                "uncaught",
                "fatal",
            )
        ):
            logging.critical(f"Failed command: {' '.join(cmd)}")
            pytest.fail(f"Process failed\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}")
    except (OSError, subprocess.TimeoutExpired) as e:
        pytest.fail(f"Failed to start process\nException: {e}")
