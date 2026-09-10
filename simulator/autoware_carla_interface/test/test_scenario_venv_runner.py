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

from autoware_carla_interface.scenario_bridge.venv_manager import ScenarioVenvRunner
from autoware_carla_interface.scenario_bridge.venv_manager import parse_spec

_SOURCE = "git+https://example.invalid/repo#subdirectory=pkg"


def _runner(tmp_path, **kwargs) -> ScenarioVenvRunner:
    return ScenarioVenvRunner(
        _SOURCE,
        "town10_straight",
        venv_dir=str(tmp_path / "venv"),
        **kwargs,
    )


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
    runner = ScenarioVenvRunner("pkg", "", venv_dir=str(tmp_path))
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
