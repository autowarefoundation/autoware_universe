# Copyright 2026 Tier IV, Inc.
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

"""Unit tests for attaching to the ego the scenario placed."""

import pytest

# The module imports the CARLA Python package at import time.
pytest.importorskip("carla")

from autoware_carla_interface.carla_autoware import InitializeInterface  # noqa: E402


class _Logger:
    def __init__(self):
        self.infos = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):  # pragma: no cover - the path must not warn any more
        raise AssertionError(f"unexpected warning: {message}")


class _Actor:
    def __init__(self, actor_id, role_name):
        self.id = actor_id
        self.attributes = {"role_name": role_name}


def _interface(actors, timeout=0.05):
    """Build an interface carrying only the state the attach path touches."""
    interface = InitializeInterface.__new__(InitializeInterface)
    interface.agent_role_name = "ego_vehicle"
    interface.ego_attach_timeout = timeout
    interface.logger = _Logger()
    interface._find_ego_actor = lambda: next(
        (a for a in actors if a.attributes.get("role_name") == interface.agent_role_name), None
    )
    return interface


def test_attaches_to_the_scenario_s_ego():
    ego = _Actor(2397, "ego_vehicle")
    interface = _interface([_Actor(11, "other"), ego])

    assert interface._attach_to_existing_ego_actor() is ego
    assert "id=2397" in interface.logger.infos[0]


def test_missing_ego_fails_instead_of_spawning_one_here():
    # Spawning here would silently run the scenario from this node's own spawn
    # point rather than the one the scenario placed.
    interface = _interface([])

    with pytest.raises(RuntimeError) as caught:
        interface._attach_to_existing_ego_actor()
    message = str(caught.value)
    assert "ego_vehicle" in message
    assert "scenario" in message


def test_an_ego_that_appears_late_is_still_adopted():
    actors = []
    interface = _interface(actors, timeout=5.0)
    calls = {"n": 0}
    ego = _Actor(7, "ego_vehicle")

    def _find():
        calls["n"] += 1
        return ego if calls["n"] > 2 else None

    interface._find_ego_actor = _find
    assert interface._attach_to_existing_ego_actor() is ego
