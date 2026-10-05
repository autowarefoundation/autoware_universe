# cspell:ignore abi3 manylinux wheelhouse
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

"""ROS-free tests for the scenario runner's spec parsing and command construction.

These exercise the pure helpers without creating a real venv or installing
anything (``provision`` / ``exec_runner`` touch the system and are covered by a
live run, not here).
"""

import argparse
from pathlib import Path
import sys
import zipfile

from autoware_carla_interface.scenario_bridge import venv_manager
from autoware_carla_interface.scenario_bridge.venv_manager import AUTO_PYTHON
from autoware_carla_interface.scenario_bridge.venv_manager import ScenarioVenvRunner
from autoware_carla_interface.scenario_bridge.venv_manager import _extract_zip
from autoware_carla_interface.scenario_bridge.venv_manager import _find_wheels
from autoware_carla_interface.scenario_bridge.venv_manager import _is_wheelhouse
from autoware_carla_interface.scenario_bridge.venv_manager import _make_runner
from autoware_carla_interface.scenario_bridge.venv_manager import _wheelhouse_install_args
from autoware_carla_interface.scenario_bridge.venv_manager import _wheelhouse_wheels
from autoware_carla_interface.scenario_bridge.venv_manager import parse_spec
from autoware_carla_interface.scenario_bridge.venv_manager import select_python
from autoware_carla_interface.scenario_bridge.venv_manager import wheelhouse_pythons
import pytest

#: The interpreter running these tests, which is also the one `auto` prefers.
RUNNING = sys.version_info.minor
HERE = f"python3.{RUNNING}"


def _args(**kw) -> argparse.Namespace:
    return argparse.Namespace(**{"python": AUTO_PYTHON, "pip_args": "", "overrides": "", **kw})


def _wheelhouse(tmp_path) -> "tuple":
    """Create a wheelhouse dir with two wheels; return (dir, sorted wheel paths).

    Tagged for the interpreter running the tests, so the checks that are about
    collecting wheels are not also checks of which ones get filtered out.
    """
    wh = tmp_path / "wheelhouse"
    (wh / "sub").mkdir(parents=True)
    tag = f"cp3{RUNNING}"
    a = wh / "scenario-0.1.0-py3-none-any.whl"
    b = wh / "sub" / f"carla-0.10.0-{tag}-{tag}-linux_x86_64.whl"
    a.write_bytes(b"")
    b.write_bytes(b"")
    return wh, sorted([a, b])


# -- spec parsing --------------------------------------------------------------


def test_parse_spec_splits_on_first_hash():
    assert parse_spec("wh.zip#town10") == ("wh.zip", "town10")
    assert parse_spec("git+https://x/r@v#a#b") == ("git+https://x/r@v", "a#b")


def test_parse_spec_without_hash_is_source_only():
    assert parse_spec("pkg") == ("pkg", "")


def test_parse_spec_empty():
    assert parse_spec("   ") == ("", "")


# -- venv command construction -------------------------------------------------


def _runner(tmp_path, install_args) -> ScenarioVenvRunner:
    # Pin the venv dir (production derives it under the user cache) so the command
    # builders can be asserted without touching the real cache.
    runner = ScenarioVenvRunner(install_args, "town10_x", python="python3.12")
    runner._venv_dir = tmp_path / "venv"
    return runner


def test_venv_cmd_uses_configured_python(tmp_path):
    runner = ScenarioVenvRunner(["pkg"], "s", python="python3.12")
    runner._venv_dir = tmp_path / "venv"
    assert runner._venv_cmd() == ["python3.12", "-m", "venv", str(tmp_path / "venv")]


def test_pip_cmd_passes_install_args(tmp_path):
    runner = _runner(tmp_path, ["--no-index", "--no-deps", "/wh/a.whl"])
    assert runner._pip_cmd() == [
        str(tmp_path / "venv" / "bin" / "python"),
        "-m",
        "pip",
        "install",
        "--no-index",
        "--no-deps",
        "/wh/a.whl",
    ]


def test_launch_cmd_appends_scenario_name(tmp_path):
    runner = _runner(tmp_path, ["pkg"])
    assert runner._launch_cmd() == [
        str(tmp_path / "venv" / "bin" / "scenario"),
        "scenario=town10_x",
    ]


def test_launch_cmd_appends_overrides_after_the_scenario(tmp_path):
    # A scenario authored for another map needs 'map=' too: Hydra resolves the map
    # group after the scenario one, so the group default would otherwise win.
    runner = ScenarioVenvRunner(
        ["pkg"], "town10_x", overrides=["map=town10hd_opt"], python="python3.12"
    )
    runner._venv_dir = tmp_path
    assert runner._launch_cmd() == [
        str(tmp_path / "bin" / "scenario"),
        "scenario=town10_x",
        "map=town10hd_opt",
    ]


def test_launch_cmd_omits_empty_scenario(tmp_path):
    runner = ScenarioVenvRunner(["pkg"], "", python="python3.12")
    runner._venv_dir = tmp_path
    assert runner._launch_cmd() == [str(tmp_path / "bin" / "scenario")]


def test_default_venv_dir_is_content_addressed():
    # No pinned dir -> a cache path keyed on (python, *install_args); the scenario
    # name is not part of the key.
    same_a = ScenarioVenvRunner(["pkg-a"], "scenario-1", python="python3.12")
    same_b = ScenarioVenvRunner(["pkg-a"], "scenario-2", python="python3.12")
    other = ScenarioVenvRunner(["pkg-b"], "scenario-1", python="python3.12")
    assert same_a._venv_dir == same_b._venv_dir
    assert same_a._venv_dir != other._venv_dir


# -- wheelhouse ----------------------------------------------------------------


def test_is_wheelhouse(tmp_path):
    (tmp_path / "wh").mkdir()
    assert _is_wheelhouse(str(tmp_path / "x.zip")) is True
    assert _is_wheelhouse(str(tmp_path / "X.ZIP")) is True
    assert _is_wheelhouse(str(tmp_path / "wh")) is True  # existing directory
    assert _is_wheelhouse("some-pip-pkg") is False


def test_find_wheels_is_recursive_and_sorted(tmp_path):
    wh, wheels = _wheelhouse(tmp_path)
    assert _find_wheels(wh) == wheels


def test_wheelhouse_install_args_from_dir(tmp_path):
    wh, wheels = _wheelhouse(tmp_path)
    assert _wheelhouse_install_args(_wheelhouse_wheels(str(wh)), HERE) == [
        "--no-index",
        "--no-deps",
        *map(str, wheels),
    ]


def test_wheelhouse_install_args_from_zip(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    tag = f"cp3{RUNNING}"
    archive = tmp_path / "wh.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("scenario-0.1.0-py3-none-any.whl", b"")
        zf.writestr(f"deps/carla-0.10.0-{tag}-{tag}-linux_x86_64.whl", b"")
    args = _wheelhouse_install_args(_wheelhouse_wheels(str(archive)), HERE)
    assert args[:2] == ["--no-index", "--no-deps"]
    names = sorted(Path(p).name for p in args[2:])
    assert names == [
        f"carla-0.10.0-{tag}-{tag}-linux_x86_64.whl",
        "scenario-0.1.0-py3-none-any.whl",
    ]


def test_wheelhouse_install_args_empty_raises(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError):
        _wheelhouse_wheels(str(tmp_path / "empty"))


# -- which interpreter, and which wheels it can have ---------------------------


def _multi_wheelhouse(tmp_path) -> Path:
    """A wheelhouse exported for 3.10 and 3.12, the two ROS 2 Pythons."""
    wh = tmp_path / "multi"
    wh.mkdir(exist_ok=True)
    for name in (
        "scenario-0.1.0-py3-none-any.whl",
        "carla-0.10.0-cp310-cp310-linux_x86_64.whl",
        "carla-0.10.0-cp312-cp312-linux_x86_64.whl",
        "numpy-2.1.0-cp310-cp310-manylinux_2_17_x86_64.whl",
        "numpy-2.1.0-cp312-cp312-manylinux_2_17_x86_64.whl",
        # Stable ABI: one wheel for 3.7 and everything after it.
        "cryptography-44.0-cp37-abi3-manylinux_2_28_x86_64.whl",
    ):
        (wh / name).write_bytes(b"")
    return wh


def test_a_wheelhouse_says_which_interpreters_it_was_built_for(tmp_path):
    # py3-none-any and cp37-abi3 install under any of them, so neither is a claim
    # that the wheelhouse was resolved for that interpreter.
    assert wheelhouse_pythons(_find_wheels(_multi_wheelhouse(tmp_path))) == [10, 12]


def test_a_single_interpreter_wheelhouse_still_says_so(tmp_path):
    wh, _ = _wheelhouse(tmp_path)
    assert wheelhouse_pythons(_find_wheels(wh)) == [RUNNING]


def test_auto_prefers_the_interpreter_running_the_launch(tmp_path, monkeypatch):
    # It is the ROS 2 distribution's own Python: certainly installed, and the one
    # `python3-venv` in package.xml covers.
    monkeypatch.setattr(venv_manager, "_running_minor", lambda: 10)
    assert select_python(AUTO_PYTHON, _find_wheels(_multi_wheelhouse(tmp_path))) == "python3.10"
    monkeypatch.setattr(venv_manager, "_running_minor", lambda: 12)
    assert select_python(AUTO_PYTHON, _find_wheels(_multi_wheelhouse(tmp_path))) == "python3.12"


def test_auto_falls_back_to_the_newest_supported_interpreter_installed(tmp_path, monkeypatch):
    monkeypatch.setattr(venv_manager, "_running_minor", lambda: 11)
    monkeypatch.setattr(
        venv_manager.shutil, "which", lambda name: "/usr/bin/x" if name == "python3.10" else None
    )
    assert select_python(AUTO_PYTHON, _find_wheels(_multi_wheelhouse(tmp_path))) == "python3.10"


def test_auto_without_any_supported_interpreter_says_what_is_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(venv_manager, "_running_minor", lambda: 11)
    monkeypatch.setattr(venv_manager.shutil, "which", lambda _name: None)
    with pytest.raises(RuntimeError) as caught:
        select_python(AUTO_PYTHON, _find_wheels(_multi_wheelhouse(tmp_path)))
    message = str(caught.value)
    assert "python3.10" in message and "python3.12" in message
    assert "scenario_python" in message


def test_a_named_interpreter_is_taken_as_given(tmp_path):
    # An explicit scenario_python:= is an instruction, not a hint -- including when
    # the wheelhouse says nothing about it.
    assert select_python("python3.11", _find_wheels(_multi_wheelhouse(tmp_path))) == "python3.11"
    assert select_python("/opt/py/bin/python", []) == "/opt/py/bin/python"


def test_a_pip_source_has_no_tags_to_read_so_the_running_python_is_used(monkeypatch):
    monkeypatch.setattr(venv_manager, "_running_minor", lambda: 10)
    assert select_python(AUTO_PYTHON, []) == "python3.10"


def test_only_the_wheels_the_interpreter_can_install_are_named(tmp_path):
    # pip fails the whole install on the first wheel tagged for another
    # interpreter, so a multi-interpreter wheelhouse has to be filtered.
    wheels = _find_wheels(_multi_wheelhouse(tmp_path))
    named = sorted(Path(a).name for a in _wheelhouse_install_args(wheels, "python3.10")[2:])
    assert named == [
        "carla-0.10.0-cp310-cp310-linux_x86_64.whl",
        # Stable ABI and pure Python both install under 3.10.
        "cryptography-44.0-cp37-abi3-manylinux_2_28_x86_64.whl",
        "numpy-2.1.0-cp310-cp310-manylinux_2_17_x86_64.whl",
        "scenario-0.1.0-py3-none-any.whl",
    ]


def test_an_interpreter_older_than_every_wheel_is_refused(tmp_path):
    wh = tmp_path / "newer"
    wh.mkdir()
    (wh / "carla-0.10.0-cp312-cp312-linux_x86_64.whl").write_bytes(b"")
    with pytest.raises(RuntimeError) as caught:
        _wheelhouse_install_args(_find_wheels(wh), "python3.10")
    assert "python3.12" in str(caught.value)


def test_extract_zip_is_reused(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    archive = tmp_path / "wh.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("a.whl", b"")
    first = _extract_zip(archive)
    assert (first / "a.whl").is_file()
    assert _extract_zip(archive) == first  # content-addressed reuse


# -- runner selection ----------------------------------------------------------


def test_make_runner_wheelhouse_dir_installs_offline(tmp_path):
    wh, wheels = _wheelhouse(tmp_path)
    runner = _make_runner(str(wh), "s", _args())
    assert runner._install_args == ["--no-index", "--no-deps", *map(str, wheels)]
    assert runner._python == HERE


def test_make_runner_installs_one_interpreter_out_of_a_multi_interpreter_wheelhouse(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(venv_manager, "_running_minor", lambda: 10)
    runner = _make_runner(str(_multi_wheelhouse(tmp_path)), "s", _args())
    assert runner._python == "python3.10"
    assert not [a for a in runner._install_args if "cp312" in a]


def test_make_runner_pip_source_keeps_source_and_pip_args(tmp_path):
    runner = _make_runner("some-pip-pkg", "s", _args(pip_args="--find-links /w"))
    assert runner._install_args == ["--find-links", "/w", "some-pip-pkg"]


def test_make_runner_shlex_splits_overrides(tmp_path):
    runner = _make_runner("some-pip-pkg", "s", _args(overrides="map=town10hd_opt server.port=2010"))
    assert runner._overrides == ["map=town10hd_opt", "server.port=2010"]


def test_missing_interpreter_is_reported_with_what_to_install(tmp_path, monkeypatch):
    # rosdep cannot supply a versioned interpreter, so a missing one must say so
    # rather than surface as a bare FileNotFoundError from subprocess.
    runner = _runner(tmp_path, ["pkg"])
    monkeypatch.setattr(venv_manager.shutil, "which", lambda _name: None)

    with pytest.raises(RuntimeError) as caught:
        runner.provision()
    message = str(caught.value)
    assert "python3.12" in message
    assert "scenario_python" in message
    assert "deadsnakes" in message


def test_provision_does_not_touch_the_system_when_the_interpreter_is_missing(tmp_path, monkeypatch):
    runner = _runner(tmp_path, ["pkg"])
    monkeypatch.setattr(venv_manager.shutil, "which", lambda _name: None)
    monkeypatch.setattr(
        venv_manager.subprocess,
        "run",
        lambda *a, **k: pytest.fail("provision ran a command for a missing interpreter"),
    )

    with pytest.raises(RuntimeError):
        runner.provision()
    assert not (tmp_path / "venv").exists()


def test_present_interpreter_provisions(tmp_path, monkeypatch):
    runner = _runner(tmp_path, ["pkg"])
    monkeypatch.setattr(venv_manager.shutil, "which", lambda name: f"/usr/bin/{name}")
    commands = []
    monkeypatch.setattr(venv_manager.subprocess, "run", lambda cmd, **k: commands.append(cmd))

    runner.provision()
    assert commands == [runner._venv_cmd(), runner._pip_cmd()]


def test_provisioned_venv_is_reused_without_checking_the_interpreter(tmp_path, monkeypatch):
    # The interpreter is only needed to build the venv; an existing one is reused
    # even if the interpreter has since gone away.
    runner = _runner(tmp_path, ["pkg"])
    entrypoint = tmp_path / "venv" / "bin" / "scenario"
    entrypoint.parent.mkdir(parents=True)
    entrypoint.touch()
    monkeypatch.setattr(
        venv_manager.shutil, "which", lambda _name: pytest.fail("checked a reused venv")
    )
    monkeypatch.setattr(
        venv_manager.subprocess, "run", lambda *a, **k: pytest.fail("reprovisioned a reused venv")
    )

    runner.provision()
