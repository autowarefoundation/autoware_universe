# cspell:ignore execv virtualenv
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

"""``scenario_runner`` CLI: provision the scenario-runner server and exec it.

The scenario runner (``autoware-carla-scenario``) is a rclpy-free Python
distribution that exposes a ``scenario`` console entrypoint and hosts the
``AutowareBridge`` gRPC server.  Rather than a Docker image, this provisions it in
a dedicated environment and then ``exec``s the entrypoint, so the launched process
*becomes* the runner.  That environment keeps the runner's dependencies (its
CPython-3.10 CARLA 0.10 wheel, protobuf 4.x, ...) isolated from the ROS 2 Python
environment -- which may even be a different Python version -- so the two never
clash.

Two source kinds are accepted (``with_scenario:=<source>#<scenario-name>``):

* a **local scenario ``.zip``** (as produced by the Scenario Editor) -- a
  self-contained ``uv`` project (``uv.lock`` pins the framework + CARLA wheel,
  which ``pip`` cannot resolve from ``[tool.uv.sources]``); it is extracted and
  ``uv sync``-ed, then its packaged ``scenario`` server is exec'd (see
  :class:`ScenarioZipRunner`).  Needs ``uv`` on ``PATH``.
* any **pip install source** -- installed into a fresh venv with
  ``python3-venv`` + ``python3-pip`` (both rosdep-resolvable, so
  ``autoware_carla_interface`` stays declarable through ``package.xml``); see
  :class:`ScenarioVenvRunner`.

Process lifecycle (start/stop) is owned by ROS 2 launch, which runs this via an
``<executable>`` and signals it directly -- the ``os.execv`` means launch's child
is the runner itself, not a wrapper.  The venv / extraction is content-addressed
under the user cache and reused across launches, so provisioning is paid once per
source.

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

__all__ = ["ScenarioVenvRunner", "ScenarioZipRunner", "parse_spec", "main"]

#: Console script the scenario runner installs.
_ENTRYPOINT = "scenario"


def parse_spec(spec: str) -> tuple[str, str]:
    """Split a ``with_scenario`` spec into ``(install_source, scenario_name)``.

    The spec is ``<install-source>#<scenario-name>``; ``#`` separates the two
    because pip install sources already use ``:`` / ``@`` / ``/`` (VCS URLs,
    paths).  It splits on the first ``#`` only, so the scenario name may itself
    contain ``#``.  A spec with no ``#`` is taken as the source alone (empty
    scenario name -> entrypoint default).  An empty/whitespace spec yields
    ``("", "")``.
    """
    source, _, scenario = spec.strip().partition("#")
    return source.strip(), scenario.strip()


def _cache_root() -> Path:
    """Return the user-cache root under which the runner's venvs/zips are stored."""
    base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return base / "autoware_carla_scenario_bridge"


def _digest(*parts: str) -> str:
    """Return a short content hash of *parts* (NUL-joined)."""
    return hashlib.sha256("\0".join(parts).encode()).hexdigest()[:16]


def _cache_venv_dir(install_source: str, pip_args: Sequence[str], python: str) -> Path:
    """Return a stable venv path keyed on ``(python, install_source, pip_args)``.

    Content-addressed under the user cache so repeated launches of the same runner
    reuse the venv (skipping the install), while a changed source or args builds a
    fresh one.
    """
    return _cache_root() / _digest(python, install_source, *pip_args)


def _find_project_dir(root: Path) -> Path:
    """Return the directory holding ``pyproject.toml`` at or one level under *root*.

    A scenario ``.zip`` normally wraps its project in a single top-level folder, so
    look at *root* itself first, then its immediate subdirectories.
    """
    if (root / "pyproject.toml").is_file():
        return root
    for child in sorted(p for p in root.iterdir() if p.is_dir()):
        if (child / "pyproject.toml").is_file():
            return child
    raise FileNotFoundError(f"no pyproject.toml found in {root}")


def _extract_scenario_zip(zip_path: str) -> Path:
    """Extract a scenario ``.zip`` into a stable cache dir; return its project root.

    Content-addressed on the archive path + mtime + size, so re-launches of the same
    archive reuse the extraction (and its ``uv``-synced ``.venv``); a re-downloaded
    archive extracts fresh.
    """
    archive = Path(zip_path).expanduser().resolve()
    stat = archive.stat()
    dest = (
        _cache_root()
        / "scenario_zips"
        / _digest(str(archive), str(stat.st_mtime_ns), str(stat.st_size))
    )
    marker = dest / ".extracted"
    if not marker.exists():
        shutil.rmtree(dest, ignore_errors=True)
        dest.mkdir(parents=True, exist_ok=True)
        logger.info("Extracting scenario package %s -> %s", archive, dest)
        with zipfile.ZipFile(archive) as zf:
            zf.extractall(dest)
        marker.touch()
    return _find_project_dir(dest)


class ScenarioVenvRunner:
    """Provisions a venv for the scenario runner and execs its gRPC server.

    Args:
        install_source: Anything ``pip install`` accepts -- a local path, a VCS
            URL, or a distribution name -- resolving to ``autoware-carla-scenario``.
        scenario_name: Scenario passed to the ``scenario`` entrypoint's Hydra CLI
            as ``scenario=<name>`` (empty -> the entrypoint's default).
        python: Interpreter used to build the venv.  Must be CPython 3.10 -- the
            runner's CARLA 0.10.0 wheel is cp310-only -- independent of whatever
            Python the ROS 2 node itself runs.
        pip_args: Extra ``pip install`` arguments (e.g. ``--find-links`` for the
            vendored CARLA wheel, or ``-e``).

    The venv lives at a stable per-source path under the user cache (see
    :func:`_cache_venv_dir`) and is reused across launches -- the install is
    skipped when its entrypoint already exists.
    """

    def __init__(
        self,
        install_source: str,
        scenario_name: str,
        *,
        python: str = "python3.10",
        pip_args: Sequence[str] = (),
    ) -> None:
        self._install_source = install_source
        self._scenario_name = scenario_name
        self._python = python
        self._pip_args = list(pip_args)
        self._venv_dir = _cache_venv_dir(install_source, self._pip_args, python)

    # -- command construction (pure; unit-tested without touching the system) --

    def _bin(self, name: str) -> Path:
        return self._venv_dir / "bin" / name

    def _venv_cmd(self) -> list[str]:
        return [self._python, "-m", "venv", str(self._venv_dir)]

    def _pip_cmd(self) -> list[str]:
        return [
            str(self._bin("python")),
            "-m",
            "pip",
            "install",
            *self._pip_args,
            self._install_source,
        ]

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
                runner fails.
            FileNotFoundError: If *python* is not on the system.
        """
        if self._bin(_ENTRYPOINT).exists():
            logger.info("Reusing scenario venv at %s", self._venv_dir)
            return
        self._venv_dir.parent.mkdir(parents=True, exist_ok=True)
        logger.info("Creating scenario venv at %s (python=%s)", self._venv_dir, self._python)
        subprocess.run(self._venv_cmd(), check=True)
        logger.info("Installing scenario runner from %r", self._install_source)
        subprocess.run(self._pip_cmd(), check=True)

    def exec_runner(self) -> NoReturn:
        """Replace this process with the runner's ``scenario`` entrypoint.

        Never returns: ``os.execv`` hands the process (and thus launch's signals)
        straight to the runner, so no wrapper lingers between launch and the server.
        """
        command = self._launch_cmd()
        logger.info("Exec scenario runner: %s", " ".join(command))
        os.execv(command[0], command)


class ScenarioZipRunner:
    """Runs a scenario packaged as a local ``.zip`` (a ``uv`` project).

    A scenario ``.zip`` (as produced by the Scenario Editor) is a self-contained
    ``uv`` project: it pins the framework (``autoware-carla-scenario``) and the rest
    via ``uv.lock`` and registers itself through the
    ``autoware_carla_scenario.scenarios`` entry point.  ``pip`` cannot resolve its
    ``[tool.uv.sources]`` git dependencies, so this path uses ``uv``: extract the
    archive, ``uv sync --locked`` its ``.venv``, then ``exec`` the ``scenario`` server.

    Args:
        zip_path: Local path to the scenario ``.zip`` (``~`` is expanded).
        scenario_name: Scenario passed to the entrypoint as ``scenario=<name>``.
        uv: The ``uv`` executable (must satisfy the package's pinned version).
    """

    def __init__(self, zip_path: str, scenario_name: str, *, uv: str = "uv") -> None:
        self._zip_path = zip_path
        self._scenario_name = scenario_name
        self._uv = uv
        self._project_dir: Path | None = None

    def _entrypoint(self) -> Path:
        assert self._project_dir is not None  # set by provision()
        return self._project_dir / ".venv" / "bin" / _ENTRYPOINT

    def _sync_cmd(self) -> list[str]:
        assert self._project_dir is not None  # set by provision()
        return [self._uv, "sync", "--locked", "--project", str(self._project_dir)]

    def _launch_cmd(self) -> list[str]:
        command = [str(self._entrypoint())]
        if self._scenario_name:
            command.append(f"scenario={self._scenario_name}")
        return command

    def provision(self) -> None:
        """Extract the archive and ``uv sync`` its locked ``.venv``.

        Raises:
            subprocess.CalledProcessError: If ``uv sync`` fails.
            FileNotFoundError: If ``uv`` is not on the system, or the archive holds
                no ``pyproject.toml``.
        """
        self._project_dir = _extract_scenario_zip(self._zip_path)
        logger.info("uv sync scenario package at %s", self._project_dir)
        subprocess.run(self._sync_cmd(), check=True)

    def exec_runner(self) -> NoReturn:
        """Replace this process with the packaged runner's ``scenario`` entrypoint."""
        command = self._launch_cmd()
        logger.info("Exec scenario runner: %s", " ".join(command))
        os.execv(command[0], command)


def _make_runner(source: str, scenario_name: str, args: argparse.Namespace):
    """Pick the runner for *source*: a local ``.zip`` (uv) or a pip install source."""
    if source.lower().endswith(".zip"):
        return ScenarioZipRunner(source, scenario_name, uv=args.uv)
    return ScenarioVenvRunner(
        source, scenario_name, python=args.python, pip_args=shlex.split(args.pip_args)
    )


def main(argv: Optional[Sequence[str]] = None) -> NoReturn:
    """``scenario_runner`` entrypoint: provision the venv, then exec the runner."""
    logging.basicConfig(level=logging.INFO, format="[scenario_runner] %(message)s")
    parser = argparse.ArgumentParser(description="Provision + exec the CARLA scenario runner.")
    parser.add_argument(
        "spec",
        help="'<source>#<scenario-name>': <source> is a local scenario .zip (extracted "
        "+ uv-synced) or a pip install source",
    )
    parser.add_argument(
        "--python", default="python3.10", help="Interpreter used to build the venv (CPython 3.10)"
    )
    parser.add_argument(
        "--pip-args", default="", help="Extra 'pip install' args (shlex-split), e.g. --find-links"
    )
    parser.add_argument("--uv", default="uv", help="The 'uv' executable used for .zip sources")
    args = parser.parse_args(argv)

    source, scenario_name = parse_spec(args.spec)
    if not source:
        parser.error("empty install source in spec")

    runner = _make_runner(source, scenario_name, args)
    runner.provision()
    runner.exec_runner()  # never returns
