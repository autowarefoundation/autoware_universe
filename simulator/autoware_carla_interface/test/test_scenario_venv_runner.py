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
import zipfile

from autoware_carla_interface.scenario_bridge.venv_manager import ScenarioVenvRunner
from autoware_carla_interface.scenario_bridge.venv_manager import ScenarioZipRunner
from autoware_carla_interface.scenario_bridge.venv_manager import _extract_scenario_zip
from autoware_carla_interface.scenario_bridge.venv_manager import _find_project_dir
from autoware_carla_interface.scenario_bridge.venv_manager import _make_runner
from autoware_carla_interface.scenario_bridge.venv_manager import parse_spec

_SOURCE = "git+https://example.invalid/repo#subdirectory=pkg"


def _runner(tmp_path, **kwargs) -> ScenarioVenvRunner:
    # Pin the venv dir (production derives it under the user cache) so the command
    # builders can be asserted without touching the real cache.
    runner = ScenarioVenvRunner(_SOURCE, "town10_straight", **kwargs)
    runner._venv_dir = tmp_path / "venv"
    return runner


def test_parse_spec_splits_on_first_hash():
    assert parse_spec("pkg#town10") == ("pkg", "town10")
    assert parse_spec("git+https://x/r@v#a#b") == ("git+https://x/r@v", "a#b")


def test_parse_spec_without_hash_is_source_only():
    assert parse_spec("pkg") == ("pkg", "")


def test_parse_spec_empty():
    assert parse_spec("   ") == ("", "")


def test_venv_cmd_uses_configured_python(tmp_path):
    runner = _runner(tmp_path, python="python3.10")
    assert runner._venv_cmd() == ["python3.10", "-m", "venv", str(tmp_path / "venv")]


def test_pip_cmd_includes_extra_args_and_source(tmp_path):
    runner = _runner(tmp_path, pip_args=["--find-links", "/wheels"])
    cmd = runner._pip_cmd()
    assert cmd[:4] == [str(tmp_path / "venv" / "bin" / "python"), "-m", "pip", "install"]
    assert "--find-links" in cmd and "/wheels" in cmd
    assert cmd[-1] == _SOURCE


def test_launch_cmd_appends_scenario_name(tmp_path):
    runner = _runner(tmp_path)
    assert runner._launch_cmd() == [
        str(tmp_path / "venv" / "bin" / "scenario"),
        "scenario=town10_straight",
    ]


def test_launch_cmd_omits_empty_scenario(tmp_path):
    runner = ScenarioVenvRunner("pkg", "")
    runner._venv_dir = tmp_path
    assert runner._launch_cmd() == [str(tmp_path / "bin" / "scenario")]


def test_default_venv_dir_is_content_addressed():
    # No venv_dir -> a cache path keyed on (python, source, pip_args); the scenario
    # name is not part of the key, so it does not change the venv.
    same_a = ScenarioVenvRunner("pkg-a", "scenario-1")
    same_b = ScenarioVenvRunner("pkg-a", "scenario-2")
    other = ScenarioVenvRunner("pkg-b", "scenario-1")
    assert same_a._venv_dir == same_b._venv_dir
    assert same_a._venv_dir != other._venv_dir
    # pip_args participate in the key.
    with_wheels = ScenarioVenvRunner("pkg-a", "scenario-1", pip_args=["--find-links", "/wheels"])
    assert with_wheels._venv_dir != same_a._venv_dir


# -- local .zip scenario package (uv) -----------------------------------------


def _args(**kw) -> argparse.Namespace:
    return argparse.Namespace(python="python3.10", pip_args="", uv="uv", **kw)


def test_make_runner_selects_zip_vs_pip(tmp_path):
    zip_src = str(tmp_path / "scn.zip")
    assert isinstance(_make_runner(zip_src, "s", _args()), ScenarioZipRunner)
    assert isinstance(_make_runner("some-pip-pkg", "s", _args()), ScenarioVenvRunner)
    # case-insensitive on the extension
    assert isinstance(_make_runner(str(tmp_path / "SCN.ZIP"), "s", _args()), ScenarioZipRunner)


def test_zip_runner_sync_and_launch_cmd(tmp_path):
    runner = ScenarioZipRunner("scn.zip", "town10_x", uv="uv")
    runner._project_dir = tmp_path / "proj"
    assert runner._sync_cmd() == ["uv", "sync", "--locked", "--project", str(tmp_path / "proj")]
    assert runner._launch_cmd() == [
        str(tmp_path / "proj" / ".venv" / "bin" / "scenario"),
        "scenario=town10_x",
    ]


def test_find_project_dir_at_root_and_one_level_down(tmp_path):
    (tmp_path / "pyproject.toml").write_text("[project]\n")
    assert _find_project_dir(tmp_path) == tmp_path

    nested = tmp_path / "b"
    pkg = nested / "the_scenario"
    pkg.mkdir(parents=True)
    (pkg / "pyproject.toml").write_text("[project]\n")
    assert _find_project_dir(nested) == pkg


def test_extract_scenario_zip_finds_project_root(tmp_path, monkeypatch):
    # Point the cache at a temp dir so extraction doesn't touch the user cache.
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    archive = tmp_path / "town10.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("town10_scenario/pyproject.toml", "[project]\n")
        zf.writestr("town10_scenario/conf/scenario/town10.yaml", "scenario: {}\n")

    project = _extract_scenario_zip(str(archive))
    assert project.name == "town10_scenario"
    assert (project / "pyproject.toml").is_file()

    # Re-extraction reuses the same content-addressed directory.
    assert _extract_scenario_zip(str(archive)) == project
