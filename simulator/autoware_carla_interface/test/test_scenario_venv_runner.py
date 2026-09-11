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
import zipfile

from autoware_carla_interface.scenario_bridge.venv_manager import ScenarioVenvRunner
from autoware_carla_interface.scenario_bridge.venv_manager import _extract_zip
from autoware_carla_interface.scenario_bridge.venv_manager import _find_wheels
from autoware_carla_interface.scenario_bridge.venv_manager import _is_wheelhouse
from autoware_carla_interface.scenario_bridge.venv_manager import _make_runner
from autoware_carla_interface.scenario_bridge.venv_manager import _wheelhouse_install_args
from autoware_carla_interface.scenario_bridge.venv_manager import parse_spec
import pytest


def _args(**kw) -> argparse.Namespace:
    return argparse.Namespace(**{"python": "python3.12", "pip_args": "", "overrides": "", **kw})


def _wheelhouse(tmp_path) -> "tuple":
    """Create a wheelhouse dir with two wheels; return (dir, sorted wheel paths)."""
    wh = tmp_path / "wheelhouse"
    (wh / "sub").mkdir(parents=True)
    a = wh / "scenario-0.1.0-py3-none-any.whl"
    b = wh / "sub" / "carla-0.10.0-cp312-cp312-linux_x86_64.whl"
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
    runner = ScenarioVenvRunner(install_args, "town10_x")
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
    runner = ScenarioVenvRunner(["pkg"], "town10_x", overrides=["map=town10hd_opt"])
    runner._venv_dir = tmp_path
    assert runner._launch_cmd() == [
        str(tmp_path / "bin" / "scenario"),
        "scenario=town10_x",
        "map=town10hd_opt",
    ]


def test_launch_cmd_omits_empty_scenario(tmp_path):
    runner = ScenarioVenvRunner(["pkg"], "")
    runner._venv_dir = tmp_path
    assert runner._launch_cmd() == [str(tmp_path / "bin" / "scenario")]


def test_default_venv_dir_is_content_addressed():
    # No pinned dir -> a cache path keyed on (python, *install_args); the scenario
    # name is not part of the key.
    same_a = ScenarioVenvRunner(["pkg-a"], "scenario-1")
    same_b = ScenarioVenvRunner(["pkg-a"], "scenario-2")
    other = ScenarioVenvRunner(["pkg-b"], "scenario-1")
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
    assert _wheelhouse_install_args(str(wh)) == ["--no-index", "--no-deps", *map(str, wheels)]


def test_wheelhouse_install_args_from_zip(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    archive = tmp_path / "wh.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("scenario-0.1.0-py3-none-any.whl", b"")
        zf.writestr("deps/carla-0.10.0-cp312-cp312-linux_x86_64.whl", b"")
    args = _wheelhouse_install_args(str(archive))
    assert args[:2] == ["--no-index", "--no-deps"]
    names = sorted(Path(p).name for p in args[2:])
    assert names == [
        "carla-0.10.0-cp312-cp312-linux_x86_64.whl",
        "scenario-0.1.0-py3-none-any.whl",
    ]


def test_wheelhouse_install_args_empty_raises(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError):
        _wheelhouse_install_args(str(tmp_path / "empty"))


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


def test_make_runner_pip_source_keeps_source_and_pip_args(tmp_path):
    runner = _make_runner("some-pip-pkg", "s", _args(pip_args="--find-links /w"))
    assert runner._install_args == ["--find-links", "/w", "some-pip-pkg"]


def test_make_runner_shlex_splits_overrides(tmp_path):
    runner = _make_runner("some-pip-pkg", "s", _args(overrides="map=town10hd_opt server.port=2010"))
    assert runner._overrides == ["map=town10hd_opt", "server.port=2010"]
