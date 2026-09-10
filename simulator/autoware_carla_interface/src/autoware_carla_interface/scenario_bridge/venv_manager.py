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

"""virtualenv launcher for the scenario-runner gRPC server.

The scenario runner (``autoware-carla-scenario``) is a rclpy-free Python
distribution that exposes a ``scenario`` console entrypoint and hosts the
``AutowareBridge`` gRPC server.  Rather than a Docker image, this installs it
into a dedicated virtualenv and runs the entrypoint as a subprocess:

* the venv keeps the runner's dependencies (its CPython-3.10 CARLA 0.10 wheel,
  protobuf 4.x, ...) isolated from the ROS 2 Python environment -- which may even
  be a different Python version -- so the two never clash;
* both primitives it relies on, ``python3-venv`` and ``python3-pip``, are
  rosdep-resolvable, unlike ``pipx`` / ``uv``, so ``autoware_carla_interface``
  stays declarable through ``package.xml``.

The venv is content-addressed under the user cache and reused across launches, so
the (multi-minute) install is paid once per install source rather than every run.

This is optional: when the scenario server is started out of band, leave
``with_scenario`` empty and point ``bridge_address`` at it directly.
"""

from __future__ import annotations

import hashlib
import logging
import os
from pathlib import Path
import subprocess
from typing import Sequence

logger = logging.getLogger(__name__)

__all__ = ["ScenarioVenvRunner"]

#: Console script the scenario runner installs.
_ENTRYPOINT = "scenario"


def _cache_venv_dir(install_source: str, pip_args: Sequence[str], python: str) -> Path:
    """Return a stable venv path keyed on ``(python, install_source, pip_args)``.

    Content-addressed under the user cache so repeated launches of the same runner
    reuse the venv (skipping the install), while a changed source or args builds a
    fresh one.
    """
    key = "\0".join([python, install_source, *pip_args])
    digest = hashlib.sha256(key.encode()).hexdigest()[:16]
    cache_root = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return cache_root / "autoware_carla_scenario_bridge" / digest


class ScenarioVenvRunner:
    """Installs the scenario runner into a venv and runs its gRPC server.

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
        venv_dir: Explicit venv location; ``None`` (the default) derives a stable
            per-source path under the user cache and reuses it across launches (the
            install is skipped when its entrypoint already exists).
    """

    def __init__(
        self,
        install_source: str,
        scenario_name: str,
        *,
        python: str = "python3.10",
        pip_args: Sequence[str] = (),
        venv_dir: str | None = None,
    ) -> None:
        self._install_source = install_source
        self._scenario_name = scenario_name
        self._python = python
        self._pip_args = list(pip_args)
        self._venv_dir = (
            Path(venv_dir) if venv_dir else _cache_venv_dir(install_source, self._pip_args, python)
        )
        self._process: subprocess.Popen | None = None

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

    def start(self) -> None:
        """Provision the venv (once per source) if needed, then launch the runner.

        Raises:
            subprocess.CalledProcessError: If creating the venv or installing the
                runner fails.
            FileNotFoundError: If *python* is not on the system.
        """
        if not self._bin(_ENTRYPOINT).exists():
            self._venv_dir.parent.mkdir(parents=True, exist_ok=True)
            logger.info("Creating scenario venv at %s (python=%s)", self._venv_dir, self._python)
            subprocess.run(self._venv_cmd(), check=True)
            logger.info("Installing scenario runner from %r", self._install_source)
            subprocess.run(self._pip_cmd(), check=True)
        command = self._launch_cmd()
        logger.info("Starting scenario runner: %s", " ".join(command))
        self._process = subprocess.Popen(command)  # noqa: S603 - argv list, no shell

    def stop(self) -> None:
        """Terminate the runner.  Idempotent; the cached venv is left in place."""
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                process.kill()
