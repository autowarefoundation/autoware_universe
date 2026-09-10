# cspell:ignore execv virtualenv wheelhouse
# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""``scenario_runner`` CLI: provision the scenario runner in a venv and exec it.

The scenario runner (``autoware-carla-scenario`` + a scenario package) is a
rclpy-free Python distribution that exposes a ``scenario`` console entrypoint and
hosts the ``AutowareBridge`` gRPC server.  Rather than a Docker image, this
installs it into a dedicated virtualenv and then ``exec``s the entrypoint, so the
launched process *becomes* the runner.  The venv keeps the runner's dependencies
(its CPython-3.10 CARLA 0.10 wheel, protobuf 4.x, ...) isolated from the ROS 2
Python environment -- which may even be a different Python version -- so the two
never clash, and it is built with ``python3-venv`` + ``python3-pip`` (both
rosdep-resolvable), so ``autoware_carla_interface`` stays declarable through
``package.xml``.

Two source kinds are accepted (``with_scenario:=<source>#<scenario-name>``):

* a **wheelhouse** -- a ``.zip`` of wheels (extracted first) or a directory of
  wheels, holding the scenario and its full dependency closure.  It is installed
  offline with ``pip install --no-index --no-deps <wheels...>`` -- no ``uv``, git,
  or network.  This is the primary, self-contained path.
* any **pip install source** (a name, path, or VCS URL), installed with
  ``pip install <source>`` (plus ``scenario_pip_args``).

Process lifecycle (start/stop) is owned by ROS 2 launch, which runs this via an
``<executable>`` and signals it directly -- the ``os.execv`` means launch's child
is the runner itself, not a wrapper.  The venv (and any extraction) is
content-addressed under the user cache and reused across launches, so provisioning
is paid once per source.

This whole path is optional: when the scenario server is started out of band,
leave ``with_scenario`` empty and point ``bridge_address`` at it directly.
"""

from __future__ import annotations

import argparse
import hashlib
import logging
import os
from pathlib import Path
import shlex
import shutil
import subprocess
from typing import NoReturn
from typing import Optional
from typing import Sequence
import zipfile

logger = logging.getLogger(__name__)

__all__ = ["ScenarioVenvRunner", "parse_spec", "main"]

#: Console script the scenario runner installs.
_ENTRYPOINT = "scenario"


def parse_spec(spec: str) -> tuple[str, str]:
    """Split a ``with_scenario`` spec into ``(source, scenario_name)``.

    The spec is ``<source>#<scenario-name>``; ``#`` separates the two because
    sources already use ``:`` / ``@`` / ``/`` (paths, VCS URLs).  It splits on the
    first ``#`` only, so the scenario name may itself contain ``#``.  A spec with
    no ``#`` is taken as the source alone (empty scenario name -> entrypoint
    default).  An empty/whitespace spec yields ``("", "")``.
    """
    source, _, scenario = spec.strip().partition("#")
    return source.strip(), scenario.strip()


def _cache_root() -> Path:
    """Return the user-cache root under which the runner's venvs/wheels are stored."""
    base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return base / "autoware_carla_scenario_bridge"


def _digest(*parts: str) -> str:
    """Return a short content hash of *parts* (NUL-joined)."""
    return hashlib.sha256("\0".join(parts).encode()).hexdigest()[:16]


def _find_wheels(root: Path) -> list[Path]:
    """Return the ``*.whl`` files under *root* (recursively), sorted."""
    return sorted(root.rglob("*.whl"))


def _extract_zip(zip_path: Path) -> Path:
    """Extract *zip_path* into a stable cache dir; return the extraction root.

    Content-addressed on the archive path + mtime + size, so re-launches of the
    same archive reuse the extraction; a re-downloaded archive extracts fresh.
    """
    stat = zip_path.stat()
    dest = (
        _cache_root()
        / "wheelhouses"
        / _digest(str(zip_path), str(stat.st_mtime_ns), str(stat.st_size))
    )
    marker = dest / ".extracted"
    if not marker.exists():
        shutil.rmtree(dest, ignore_errors=True)
        dest.mkdir(parents=True, exist_ok=True)
        logger.info("Extracting wheelhouse %s -> %s", zip_path, dest)
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(dest)
        marker.touch()
    return dest


def _wheelhouse_install_args(source: str) -> list[str]:
    """Return the ``pip install`` args for a wheelhouse *source*.

    *source* is a wheelhouse ``.zip`` (extracted first) or a directory of wheels.
    Installs the whole wheel set offline -- ``--no-index`` so nothing is fetched,
    ``--no-deps`` because the wheelhouse already carries the full closure.

    Raises:
        FileNotFoundError: If the wheelhouse holds no ``*.whl`` files.
    """
    path = Path(source).expanduser().resolve()
    root = _extract_zip(path) if path.suffix.lower() == ".zip" else path
    wheels = _find_wheels(root)
    if not wheels:
        raise FileNotFoundError(f"no *.whl found in wheelhouse {source}")
    return ["--no-index", "--no-deps", *(str(w) for w in wheels)]


def _is_wheelhouse(source: str) -> bool:
    """Whether *source* is a wheelhouse (a ``.zip`` or an existing directory)."""
    path = Path(source).expanduser()
    return source.lower().endswith(".zip") or path.is_dir()


class ScenarioVenvRunner:
    """Installs the scenario runner into a venv and execs its gRPC server.

    Args:
        install_args: Arguments after ``pip install`` -- either a wheelhouse's
            ``--no-index --no-deps <wheels...>`` or ``[*pip_args, <source>]``.
        scenario_name: Scenario passed to the ``scenario`` entrypoint's Hydra CLI
            as ``scenario=<name>`` (empty -> the entrypoint's default).
        python: Interpreter used to build the venv.  Must be CPython 3.10 -- the
            runner's CARLA 0.10.0 wheel is cp310-only -- independent of whatever
            Python the ROS 2 node itself runs.

    The venv lives at a stable path under the user cache (keyed on the install args)
    and is reused across launches -- the install is skipped when its entrypoint
    already exists.
    """

    def __init__(
        self,
        install_args: Sequence[str],
        scenario_name: str,
        *,
        python: str = "python3.10",
    ) -> None:
        self._install_args = list(install_args)
        self._scenario_name = scenario_name
        self._python = python
        self._venv_dir = _cache_root() / "venvs" / _digest(python, *self._install_args)

    # -- command construction (pure; unit-tested without touching the system) --

    def _bin(self, name: str) -> Path:
        return self._venv_dir / "bin" / name

    def _venv_cmd(self) -> list[str]:
        return [self._python, "-m", "venv", str(self._venv_dir)]

    def _pip_cmd(self) -> list[str]:
        return [str(self._bin("python")), "-m", "pip", "install", *self._install_args]

    def _launch_cmd(self) -> list[str]:
        # The scenario name is passed as a list argument (never a shell string, so
        # it needs no escaping). The exact serve-mode overrides are coordinated
        # with the framework side (issue #10); extend this if it needs more.
        command = [str(self._bin(_ENTRYPOINT))]
        if self._scenario_name:
            command.append(f"scenario={self._scenario_name}")
        return command

    # -- lifecycle -------------------------------------------------------------

    def provision(self) -> None:
        """Create the venv and install the runner, unless already provisioned.

        Raises:
            subprocess.CalledProcessError: If creating the venv or installing the
                wheels fails.
            FileNotFoundError: If *python* is not on the system.
        """
        if self._bin(_ENTRYPOINT).exists():
            logger.info("Reusing scenario venv at %s", self._venv_dir)
            return
        self._venv_dir.parent.mkdir(parents=True, exist_ok=True)
        logger.info("Creating scenario venv at %s (python=%s)", self._venv_dir, self._python)
        subprocess.run(self._venv_cmd(), check=True)
        logger.info("Installing scenario runner (pip install %s)", " ".join(self._install_args))
        subprocess.run(self._pip_cmd(), check=True)

    def exec_runner(self) -> NoReturn:
        """Replace this process with the runner's ``scenario`` entrypoint.

        Never returns: ``os.execv`` hands the process (and thus launch's signals)
        straight to the runner, so no wrapper lingers between launch and the server.
        """
        command = self._launch_cmd()
        logger.info("Exec scenario runner: %s", " ".join(command))
        os.execv(command[0], command)


def _make_runner(source: str, scenario_name: str, args: argparse.Namespace) -> ScenarioVenvRunner:
    """Build the runner for *source*: a wheelhouse (.zip/dir) or a pip install source."""
    if _is_wheelhouse(source):
        install_args = _wheelhouse_install_args(source)
    else:
        install_args = [*shlex.split(args.pip_args), source]
    return ScenarioVenvRunner(install_args, scenario_name, python=args.python)


def main(argv: Optional[Sequence[str]] = None) -> NoReturn:
    """``scenario_runner`` entrypoint: provision the venv, then exec the runner."""
    logging.basicConfig(level=logging.INFO, format="[scenario_runner] %(message)s")
    parser = argparse.ArgumentParser(description="Provision + exec the CARLA scenario runner.")
    parser.add_argument(
        "spec",
        help="'<source>#<scenario-name>': <source> is a wheelhouse (.zip of wheels or a "
        "directory of wheels) or a pip install source",
    )
    parser.add_argument(
        "--python", default="python3.10", help="Interpreter used to build the venv (CPython 3.10)"
    )
    parser.add_argument(
        "--pip-args", default="", help="Extra 'pip install' args (shlex-split) for a pip source"
    )
    args = parser.parse_args(argv)

    source, scenario_name = parse_spec(args.spec)
    if not source:
        parser.error("empty source in spec")

    runner = _make_runner(source, scenario_name, args)
    runner.provision()
    runner.exec_runner()  # never returns
