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

"""``scenario_runner`` CLI: provision a venv and exec the scenario-runner server.

The scenario runner (``autoware-carla-scenario``) is a rclpy-free Python
distribution that exposes a ``scenario`` console entrypoint and hosts the
``AutowareBridge`` gRPC server.  Rather than a Docker image, this installs it into
a dedicated virtualenv and then ``exec``s the entrypoint, so the launched process
*becomes* the runner:

* the venv keeps the runner's dependencies (its CPython-3.10 CARLA 0.10 wheel,
  protobuf 4.x, ...) isolated from the ROS 2 Python environment -- which may even
  be a different Python version -- so the two never clash;
* both primitives it relies on, ``python3-venv`` and ``python3-pip``, are
  rosdep-resolvable, unlike ``pipx`` / ``uv``, so ``autoware_carla_interface``
  stays declarable through ``package.xml``.

Process lifecycle (start/stop) is owned by ROS 2 launch, which runs this via an
``<executable>`` and signals it directly -- the ``os.execv`` means launch's child
is the runner itself, not a wrapper.  The venv is content-addressed under the user
cache and reused across launches, so the install is paid once per source.

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
import subprocess
from typing import NoReturn
from typing import Optional
from typing import Sequence

logger = logging.getLogger(__name__)

__all__ = ["ScenarioVenvRunner", "parse_spec", "main"]

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


def main(argv: Optional[Sequence[str]] = None) -> int:
    """``scenario_runner`` entrypoint: provision the venv, then exec the runner."""
    logging.basicConfig(level=logging.INFO, format="[scenario_runner] %(message)s")
    parser = argparse.ArgumentParser(description="Provision + exec the CARLA scenario runner.")
    parser.add_argument("spec", help="'<install-source>#<scenario-name>' (the with_scenario value)")
    parser.add_argument(
        "--python", default="python3.10", help="Interpreter used to build the venv (CPython 3.10)"
    )
    parser.add_argument(
        "--pip-args", default="", help="Extra 'pip install' args (shlex-split), e.g. --find-links"
    )
    args = parser.parse_args(argv)

    source, scenario_name = parse_spec(args.spec)
    if not source:
        parser.error("empty install source in spec")

    runner = ScenarioVenvRunner(
        source, scenario_name, python=args.python, pip_args=shlex.split(args.pip_args)
    )
    runner.provision()
    runner.exec_runner()  # never returns
    return 0  # pragma: no cover - unreachable after execv
