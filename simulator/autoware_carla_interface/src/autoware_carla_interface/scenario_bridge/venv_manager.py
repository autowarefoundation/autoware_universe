# cspell:ignore execv virtualenv wheelhouse abi3 manylinux rosdistro
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
(its compiled CARLA 0.10 client, protobuf 4.x, ...) isolated from the ROS 2
Python environment so the two never clash, and it is built with ``python3-venv``
+ ``python3-pip`` (both rosdep-resolvable), so ``autoware_carla_interface`` stays
declarable through ``package.xml``.

The *interpreter* the venv is built with is chosen from the wheelhouse itself:
a wheelhouse carries a wheel per interpreter it supports, and the tags on those
wheels say which.  The system Python is preferred when it is one of them, which
on both Jazzy (3.12) and Humble (3.10) it is -- so nothing has to be installed
out of band and ``scenario_python`` only has to be passed when overriding the
choice.  See :func:`select_python`.

Two source kinds are accepted (``with_scenario:=<source>#<scenario-name>``):

* a **wheelhouse** -- a ``.zip`` of wheels (extracted first) or a directory of
  wheels, holding the scenario and its full dependency closure.  It is installed
  offline with ``pip install --no-index --no-deps <wheels...>`` -- no ``uv``, git,
  or network.  This is the primary, self-contained path.  Only the wheels the
  chosen interpreter can install are named: a wheelhouse built for several
  carries the others' compiled wheels too, and pip refuses a wheel whose tag
  does not match however it was asked for it.
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
import re
import shlex
import shutil
import subprocess
import sys
from typing import NoReturn
from typing import Optional
from typing import Sequence
import zipfile

logger = logging.getLogger(__name__)

__all__ = ["ScenarioVenvRunner", "parse_spec", "select_python", "main"]

#: Console script the scenario runner installs.
_ENTRYPOINT = "scenario"

#: ``--python=auto``: pick the interpreter from the wheelhouse's own wheel tags.
AUTO_PYTHON = "auto"

#: A wheel built for one CPython minor, e.g. ``cp310``.
_CPYTHON_TAG = re.compile(r"^cp3(\d+)$")

#: A wheel that is not built for a particular minor, e.g. ``py3`` or ``py36``.
_GENERIC_TAG = re.compile(r"^py3(\d*)$")


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


def _wheelhouse_wheels(source: str) -> list[Path]:
    """Return every wheel in the wheelhouse at *source*.

    *source* is a wheelhouse ``.zip`` (extracted first) or a directory of wheels.

    Raises:
        FileNotFoundError: If the wheelhouse holds no ``*.whl`` files.
    """
    path = Path(source).expanduser().resolve()
    root = _extract_zip(path) if path.suffix.lower() == ".zip" else path
    wheels = _find_wheels(root)
    if not wheels:
        raise FileNotFoundError(
            f"no *.whl under {source}, so there is nothing to install. A directory "
            "or .zip given as the scenario source is taken to be a wheelhouse -- a "
            "tree of wheels carrying the runner and its full dependency closure, "
            "installed offline. A source checkout is not one: build it first "
            "(`uv build --wheel`, or `pip wheel`) and point this at the wheels. "
            "Installing from a source tree is not supported yet."
        )
    return wheels


def _wheel_tags(wheel: Path) -> tuple[list[str], str]:
    """Return a wheel's ``(python tags, abi tag)``, or ``([], "")`` if unreadable.

    ``name-version[-build]-python-abi-platform.whl``, and the python field holds
    several tags joined by ``.`` when one wheel serves several interpreters.
    """
    fields = wheel.name[: -len(".whl")].split("-")
    if len(fields) < 5:
        return [], ""
    return fields[-3].split("."), fields[-2]


def wheelhouse_pythons(wheels: Sequence[Path]) -> list[int]:
    """Return the CPython minors *wheels* were built for, ascending.

    Only wheels pinned to one interpreter are counted -- ``cp310-cp310``, the
    shape a compiled extension has.  ``py3-none-any`` and ``cp37-abi3`` install
    under a whole range and so say nothing about which interpreters the
    wheelhouse was resolved for.

    Returns:
        e.g. ``[10, 12]`` for a wheelhouse serving Humble and Jazzy, or an empty
        list when nothing in it is interpreter-specific.
    """
    minors = set()
    for wheel in wheels:
        pythons, abi = _wheel_tags(wheel)
        for tag in pythons:
            matched = _CPYTHON_TAG.match(tag)
            if matched is not None and abi == tag:
                minors.add(int(matched.group(1)))
    return sorted(minors)


def _installable(wheel: Path, minor: int) -> bool:
    """Whether CPython 3.*minor* can install *wheel*."""
    pythons, abi = _wheel_tags(wheel)
    if not pythons:
        # Not a name this can read; let pip be the one to refuse it.
        return True
    for tag in pythons:
        generic = _GENERIC_TAG.match(tag)
        if generic is not None:
            # `py3` is any 3.x; `py36` is 3.6 and up.
            if not generic.group(1) or int(generic.group(1)) <= minor:
                return True
            continue
        cpython = _CPYTHON_TAG.match(tag)
        if cpython is None:
            continue
        built = int(cpython.group(1))
        # A stable-ABI wheel installs on its own minor and every later one.
        if built == minor or (abi == "abi3" and built <= minor):
            return True
    return False


def _interpreter(minor: int) -> str:
    """Return the command name for CPython 3.*minor*."""
    return f"python3.{minor}"


def _running_minor() -> int:
    """Return the CPython minor of the process asking -- the ROS 2 distribution's."""
    return sys.version_info.minor


def select_python(requested: str, wheels: Sequence[Path]) -> str:
    """Return the interpreter to build the venv with.

    Anything other than :data:`AUTO_PYTHON` is taken as given -- an explicit
    ``scenario_python:=`` is an instruction, not a hint, and it is checked for
    existence later where the message can say what to install.

    Automatically, the wheelhouse decides. It holds a wheel per interpreter it
    was resolved for, so the tags on those wheels are the list of interpreters
    that can install it, and the one running this process is preferred whenever
    it is on that list: it is the ROS 2 distribution's own Python, it is
    certainly installed, and its ``python3-venv`` is what ``package.xml``
    already pulls in. Otherwise the newest supported interpreter that is
    actually installed wins.

    Args:
        requested: ``scenario_python``; :data:`AUTO_PYTHON` to choose here.
        wheels: The wheelhouse's wheels, or empty for a pip source -- there is
            nothing to read tags off then, so the running interpreter is used.

    Returns:
        The interpreter command, e.g. ``python3.10``.

    Raises:
        RuntimeError: If the wheelhouse supports no interpreter that is
            installed here.
    """
    if requested != AUTO_PYTHON:
        return requested

    running = _running_minor()
    supported = wheelhouse_pythons(wheels)
    if not supported or running in supported:
        return _interpreter(running)

    for minor in reversed(supported):
        if shutil.which(_interpreter(minor)) is not None:
            return _interpreter(minor)

    raise RuntimeError(
        "This wheelhouse holds wheels for "
        + ", ".join(_interpreter(minor) for minor in supported)
        + f", and none of them is installed -- this process runs {_interpreter(running)}. "
        "Install one of them (on Ubuntu, `python3.X python3.X-venv`), export a "
        "wheelhouse covering this interpreter, or pass scenario_python:= to choose "
        "one yourself."
    )


def _wheelhouse_install_args(wheels: Sequence[Path], python: str) -> list[str]:
    """Return the ``pip install`` args for *wheels* under the *python* venv.

    Installs offline -- ``--no-index`` so nothing is fetched, ``--no-deps``
    because the wheelhouse already carries the full closure.

    Naming the wheels individually is what makes those two flags enough, and it
    is also why they have to be filtered: a wheelhouse built for 3.10 and 3.12
    holds both interpreters' compiled wheels, and pip fails the whole install on
    the first one tagged for the other (`is not a supported wheel on this
    platform`). Filtering by tag here, rather than handing pip the directory and
    a requirement set, keeps the install exactly as pinned as the wheelhouse is.

    Raises:
        RuntimeError: If no wheel in the wheelhouse matches *python*.
    """
    matched = re.search(r"3\.(\d+)$", python)
    if matched is None:
        # An interpreter named something this cannot parse (`/opt/py/bin/python`)
        # is taken at its word: pip refuses what does not fit, with its own message.
        return ["--no-index", "--no-deps", *(str(wheel) for wheel in wheels)]
    minor = int(matched.group(1))
    installable = [wheel for wheel in wheels if _installable(wheel, minor)]
    if not installable:
        raise RuntimeError(
            f"No wheel in this wheelhouse can be installed by {python}. It holds "
            "wheels for "
            + (
                ", ".join(_interpreter(each) for each in wheelhouse_pythons(wheels))
                or "no interpreter this can identify"
            )
            + "."
        )
    return ["--no-index", "--no-deps", *(str(wheel) for wheel in installable)]


def _is_wheelhouse(source: str) -> bool:
    """Whether *source* is a wheelhouse (a ``.zip`` or an existing directory).

    Every local path is one: a wheelhouse is the only local source supported
    today, so a directory without wheels is a mistake to report rather than a
    second kind of source to guess at. :func:`_wheelhouse_install_args` says so
    when it finds none. Anything that is not a local path falls through to pip,
    which is how a PyPI name or a VCS URL reaches it.
    """
    path = Path(source).expanduser()
    return source.lower().endswith(".zip") or path.is_dir()


class ScenarioVenvRunner:
    """Installs the scenario runner into a venv and execs its gRPC server.

    Args:
        install_args: Arguments after ``pip install`` -- either a wheelhouse's
            ``--no-index --no-deps <wheels...>`` or ``[*pip_args, <source>]``.
        scenario_name: Scenario passed to the ``scenario`` entrypoint's Hydra CLI
            as ``scenario=<name>`` (empty -> the entrypoint's default).
        overrides: Further Hydra overrides appended after it, e.g.
            ``["map=town10hd_opt"]``.  A scenario that is authored for a map other
            than the entrypoint's default needs its map named here: Hydra resolves
            the ``map`` group after the ``scenario`` one, so the group's default
            wins over what the scenario config sets unless the map is overridden
            too.
        python: Interpreter used to build the venv.  It has to be one the
            wheelhouse holds wheels for; :func:`select_python` is what picks it
            from the wheelhouse rather than from a default that can only be
            right on one ROS 2 distribution.

    The venv lives at a stable path under the user cache (keyed on the install args)
    and is reused across launches -- the install is skipped when its entrypoint
    already exists.
    """

    def __init__(
        self,
        install_args: Sequence[str],
        scenario_name: str,
        *,
        overrides: Sequence[str] = (),
        python: str,
    ) -> None:
        self._install_args = list(install_args)
        self._scenario_name = scenario_name
        self._overrides = list(overrides)
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
        # The scenario name and the overrides are passed as list arguments (never a
        # shell string, so they need no escaping). The exact serve-mode overrides
        # are coordinated with the framework side (issue #10).
        command = [str(self._bin(_ENTRYPOINT))]
        if self._scenario_name:
            command.append(f"scenario={self._scenario_name}")
        command.extend(self._overrides)
        return command

    # -- lifecycle -------------------------------------------------------------

    def _check_python(self) -> None:
        """Fail with an actionable message when the interpreter is not installed.

        ``python3 -m venv`` does not provide an interpreter, it links the one running
        it, so the venv's Python version is whatever *python* already is on this
        system. ``python3-venv`` and ``python3-pip`` (both rosdep-resolvable, both in
        ``package.xml``) add venv support for the distribution's own Python and for
        no other: rosdep cannot express a versioned interpreter, since no
        ``python3.X`` keys exist in rosdistro.

        Chosen automatically that is exactly what gets picked, so reaching this
        means ``scenario_python:=`` named something that is not installed.

        Without this check the failure is a bare ``FileNotFoundError: [Errno 2] ...
        'python3.12'`` from ``subprocess.run``, which says nothing about what to
        install.
        """
        if shutil.which(self._python) is not None:
            return
        raise RuntimeError(
            f"Interpreter '{self._python}' not found, so the scenario runner's venv cannot "
            "be built. It was named by scenario_python:= -- drop that argument to have the "
            "interpreter chosen from the wheelhouse's own wheel tags, or install this one "
            "(on Ubuntu, `python3.X python3.X-venv`; versions the archive does not carry "
            "come from a PPA such as deadsnakes)."
        )

    def provision(self) -> None:
        """Create the venv and install the runner, unless already provisioned.

        Raises:
            RuntimeError: If *python* is not installed (see :meth:`_check_python`).
            subprocess.CalledProcessError: If creating the venv or installing the
                wheels fails.
        """
        if self._bin(_ENTRYPOINT).exists():
            logger.info("Reusing scenario venv at %s", self._venv_dir)
            return
        self._check_python()
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
    """Build the runner for *source*: a wheelhouse (.zip/dir) or a pip install source.

    The interpreter is settled before the install args, because a wheelhouse's
    args are the subset of its wheels that interpreter can install.
    """
    if _is_wheelhouse(source):
        wheels = _wheelhouse_wheels(source)
        python = select_python(args.python, wheels)
        install_args = _wheelhouse_install_args(wheels, python)
    else:
        python = select_python(args.python, ())
        install_args = [*shlex.split(args.pip_args), source]
    logger.info("Scenario venv interpreter: %s", python)
    return ScenarioVenvRunner(
        install_args,
        scenario_name,
        overrides=shlex.split(args.overrides),
        python=python,
    )


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
        "--python",
        default=AUTO_PYTHON,
        help="Interpreter used to build the venv; 'auto' (the default) picks one the "
        "wheelhouse has wheels for, preferring the Python running this process",
    )
    parser.add_argument(
        "--pip-args", default="", help="Extra 'pip install' args (shlex-split) for a pip source"
    )
    parser.add_argument(
        "--overrides",
        default="",
        help="Extra Hydra overrides (shlex-split) for the scenario entrypoint, "
        "e.g. 'map=town10hd_opt'",
    )
    args = parser.parse_args(argv)

    source, scenario_name = parse_spec(args.spec)
    if not source:
        parser.error("empty source in spec")

    runner = _make_runner(source, scenario_name, args)
    runner.provision()
    runner.exec_runner()  # never returns
