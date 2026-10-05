"""Unit tests for attaching to the ego the scenario placed.

``modules.scenario_world`` never imports ``carla`` -- it only uses the world and
the actors through a handful of methods -- so these run without a simulator.
"""

import pytest

# ``modules/__init__.py`` eagerly imports the ROS publisher manager, so the ROS
# message packages have to be importable; nothing below needs carla or a server.
pytest.importorskip("geometry_msgs")

from autoware_carla_interface.modules import scenario_world  # noqa: E402
from autoware_carla_interface.modules.scenario_world import ScenarioEgoMissing  # noqa: E402
from autoware_carla_interface.modules.scenario_world import attach_to_scenario_ego  # noqa: E402
from autoware_carla_interface.modules.scenario_world import find_ego_actor  # noqa: E402


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


class _ActorList:
    def __init__(self, actors):
        self._actors = actors

    def filter(self, pattern):
        assert pattern == "vehicle.*"
        return list(self._actors)


class _World:
    """A world whose actor list is whatever ``actors`` holds when it is asked."""

    def __init__(self, actors):
        self._actors = actors

    def get_actors(self):
        return _ActorList(self._actors)


def test_finds_the_actor_carrying_the_ego_role():
    ego = _Actor(2397, "ego_vehicle")
    world = _World([_Actor(11, "other"), ego])

    assert find_ego_actor(world, "ego_vehicle") is ego
    assert find_ego_actor(world, "nobody") is None


def test_attaches_to_the_scenario_s_ego():
    ego = _Actor(2397, "ego_vehicle")
    logger = _Logger()

    adopted = attach_to_scenario_ego(
        _World([_Actor(11, "other"), ego]), "ego_vehicle", 0.05, logger
    )

    assert adopted is ego
    assert "id=2397" in logger.infos[0]


def test_missing_ego_fails_instead_of_spawning_one_here():
    with pytest.raises(ScenarioEgoMissing) as caught:
        attach_to_scenario_ego(_World([]), "ego_vehicle", 0.05, _Logger())
    message = str(caught.value)
    assert "ego_vehicle" in message
    assert "scenario" in message


def test_an_ego_that_appears_late_is_still_adopted(monkeypatch):
    # The scenario brings its ego up while this node is between polls.
    actors = []
    ego = _Actor(7, "ego_vehicle")
    monkeypatch.setattr(scenario_world.time, "sleep", lambda _seconds: actors.append(ego))

    assert attach_to_scenario_ego(_World(actors), "ego_vehicle", 5.0, _Logger()) is ego
