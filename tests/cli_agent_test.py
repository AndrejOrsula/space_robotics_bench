import os
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Iterable

import pytest

from srb.utils import logging
from srb.utils.cache import read_offline_srb_env_cache
from srb.utils.failures import IntegrityFailureError
from srb.utils.isaacsim import get_isaacsim_python, get_kit_python
from srb.utils.subprocess import terminate_process

NUM_ENVS: int = 4
MAX_DURATION_RAND: float = 40.0
MAX_DURATION_TRAIN: float = 80.0

HEADLESS: bool = False
STACK: bool = True
TEST_TEMPLATE_ENVS: bool = False
TEST_EXCAVATION: bool = True

PERF: bool = False
PERF_DURATION: float = 600.0
PERF_OUTPUT_DIR: str = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs", "_perf"
)

TEST_START_TIME: float = time.time()


#: Isaac Sim release series this matrix is valid on.
SUPPORTED_ISAAC_SIM_SERIES: str = "6.1"


def _require_supported_isaac_sim() -> None:
    """Skip when the local Isaac Sim release is unsupported.

    This matrix can launch Kit for minutes, so a mismatched version gives no
    useful evidence. Read the release from the adjacent `VERSION` file; keep
    prior behavior when that file is absent.
    """
    version_file = Path(get_isaacsim_python()).parent / "VERSION"
    try:
        version = version_file.read_text(encoding="utf-8").strip()
    except OSError:
        return
    series = ".".join(version.split("+", 1)[0].split("-", 1)[0].split(".")[:2])
    if series != SUPPORTED_ISAAC_SIM_SERIES:
        pytest.skip(
            f"{Path(__file__).name}: Isaac Sim {version} at {version_file.parent} "
            f"is not the supported {SUPPORTED_ISAAC_SIM_SERIES} series; "
            "run this matrix in the container (`srb test` there)",
            allow_module_level=True,
        )


_require_supported_isaac_sim()


def _is_cache_permission_issue(exc: BaseException) -> bool:
    if not isinstance(exc, IntegrityFailureError):
        return False
    cause = str(exc.info.details.get("cause", "")).lower()
    return "permission denied" in cause


def _list_envs() -> Iterable[str]:
    """Return cached environments, or skip with the reason the cache is unusable."""
    test_filepath = Path(__file__)
    label = f"{test_filepath.parent.name}/{test_filepath.name}"
    try:
        envs = read_offline_srb_env_cache()
    except IntegrityFailureError as exc:
        if _is_cache_permission_issue(exc):
            pytest.skip(
                f"{label}: offline cache is not readable in this environment ({exc})",
                allow_module_level=True,
            )
        raise

    if not envs:
        pytest.skip(
            f"{label}: no environments in the offline cache; run `srb cache update`",
            allow_module_level=True,
        )

    if not TEST_TEMPLATE_ENVS:
        envs = filter(lambda env: not env.startswith("_"), envs)
    if not TEST_EXCAVATION:
        envs = filter(lambda env: "excavation" not in env, envs)
    return sorted(envs)


@pytest.mark.parametrize(
    "env,num_envs",
    tuple((env, num_envs) for env in _list_envs() for num_envs in (NUM_ENVS,)),
)
def test_cli_agent_rand(env: str, num_envs: int):
    with tempfile.TemporaryDirectory(prefix="srb_") as tmpdir:
        cmd = (
            get_kit_python(),
            "-m",
            "srb",
            "agent",
            "rand",
            "--headless" if HEADLESS else "--hide_ui",
            f"env.scene.num_envs={num_envs}",
            "--env",
            env,
            "--logs",
            tmpdir,
            f"env.stack={STACK}",
        )

        if PERF:
            cmd += (
                "--perf",
                "--perf_duration",
                str(PERF_DURATION),
                "--perf_output",
                os.path.join(
                    PERF_OUTPUT_DIR,
                    f"perf_{env.replace('/', '_')}_{num_envs}_{int(TEST_START_TIME)}.txt",
                ),
            )

            if not os.path.exists(PERF_OUTPUT_DIR):
                os.makedirs(PERF_OUTPUT_DIR, exist_ok=True)
            with open(
                os.path.join(PERF_OUTPUT_DIR, f"commands_{int(TEST_START_TIME)}.bash"),
                "a",
            ) as f:
                f.write(" ".join(cmd) + "\n")

        environ = os.environ.copy()
        environ["SF_BAKER"] = "0"

        process = None
        stdout_file = None
        stderr_file = None
        try:
            stdout_file = open(os.path.join(tmpdir, "stdout.log"), "w+")
            stderr_file = open(os.path.join(tmpdir, "stderr.log"), "w+")
            process = subprocess.Popen(
                cmd,
                env=environ,
                stdout=stdout_file,
                stderr=stderr_file,
                start_new_session=True,
            )

            if PERF:
                assert process.wait() == 0
            else:
                start = time.time()
                while time.time() - start < MAX_DURATION_RAND:
                    if process.poll() is not None:
                        logging.critical(f"[{env}] Failed command: {' '.join(cmd)}")
                        stdout_file.seek(0)
                        stderr_file.seek(0)
                        stdout, stderr = stdout_file.read(), stderr_file.read()
                        pytest.fail(
                            f'Process failed for env "{env}"\n[env={env}] STDOUT:\n{stdout}\n[env={env}] STDERR:\n{stderr}'
                        )
                    time.sleep(0.01 * MAX_DURATION_RAND)
                assert process.poll() is None
        except (OSError, ValueError) as e:
            logging.critical(f"[{env}] Failed command: {' '.join(cmd)}")
            pytest.fail(
                f'Failed to start process for env "{env}"\n[env={env}] Exception: {e}'
            )
        finally:
            terminate_process("srb", process)
            if stdout_file is not None:
                stdout_file.close()
            if stderr_file is not None:
                stderr_file.close()


# Dreamer training can take up to MAX_DURATION_TRAIN seconds per environment;
# opt in with `-m long_training`.
@pytest.mark.long_training
@pytest.mark.parametrize(
    "env,num_envs,algo",
    tuple(
        (env, num_envs, algo)
        for env in _list_envs()
        for num_envs in (NUM_ENVS,)
        for algo in ("dreamer",)
    ),
)
def test_cli_agent_train(env: str, num_envs: int, algo: str):
    with tempfile.TemporaryDirectory(prefix="srb_") as tmpdir:
        match algo:
            case _:
                agent_params = ()

        cmd = (
            get_kit_python(),
            "-m",
            "srb",
            "agent",
            "train",
            # Dreamer trains in headless workers and rejects --hide_ui
            "--headless" if HEADLESS or algo == "dreamer" else "--hide_ui",
            f"env.scene.num_envs={num_envs}",
            "--env",
            env,
            "--algo",
            algo,
            "--logs",
            tmpdir,
            *agent_params,
        )

        environ = os.environ.copy()
        environ["SF_BAKER"] = "0"

        process = None
        stdout_file = None
        stderr_file = None
        try:
            stdout_file = open(os.path.join(tmpdir, "stdout.log"), "w+")
            stderr_file = open(os.path.join(tmpdir, "stderr.log"), "w+")
            process = subprocess.Popen(
                cmd,
                env=environ,
                stdout=stdout_file,
                stderr=stderr_file,
                start_new_session=True,
            )

            start = time.time()
            while time.time() - start < MAX_DURATION_TRAIN:
                if process.poll() is not None:
                    logging.critical(f"[{env}] Failed command: {' '.join(cmd)}")
                    stdout_file.seek(0)
                    stderr_file.seek(0)
                    stdout, stderr = stdout_file.read(), stderr_file.read()
                    pytest.fail(
                        f'Process failed for env "{env}"\n[env={env}] STDOUT:\n{stdout}\n[env={env}] STDERR:\n{stderr}'
                    )
                time.sleep(0.01 * MAX_DURATION_TRAIN)
            assert process.poll() is None
        except (OSError, ValueError) as e:
            logging.critical(f"[{env}] Failed command: {' '.join(cmd)}")
            pytest.fail(
                f'Failed to start process for env "{env}"\n[env={env}] Exception: {e}'
            )
        finally:
            terminate_process("srb", process)
            if stdout_file is not None:
                stdout_file.close()
            if stderr_file is not None:
                stderr_file.close()
