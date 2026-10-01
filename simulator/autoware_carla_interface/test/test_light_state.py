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

"""Unit tests for how the bridge degrades when CARLA rejects light-state calls."""

import threading

import pytest

# The module imports the CARLA Python package at import time.
pytest.importorskip("carla")

from autoware_carla_interface import carla_ros  # noqa: E402
from autoware_vehicle_msgs.msg import HazardLightsCommand  # noqa: E402
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand  # noqa: E402
import carla  # noqa: E402

LEFT = int(carla.VehicleLightState.LeftBlinker)
RIGHT = int(carla.VehicleLightState.RightBlinker)


class _Logger:
    def __init__(self):
        self.warnings = []
        self.infos = []

    def warning(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


class _EgoActor:
    """Ego stub whose get_light_state() raises while ``broken`` is set."""

    def __init__(self, light_state=0):
        self.light_state = light_state
        self.broken = False
        self.get_calls = 0

    def get_light_state(self):
        self.get_calls += 1
        if self.broken:
            # Exactly what libcarla raises for any RPC it cannot serve.
            raise RuntimeError("std::exception")
        return self.light_state

    def set_light_state(self, state):
        if self.broken:
            raise RuntimeError("std::exception")
        self.light_state = int(state)


def _bridge(ego):
    """Build a bridge instance carrying only the state the light-state path touches."""
    bridge = carla_ros.carla_ros2_interface.__new__(carla_ros.carla_ros2_interface)
    bridge.ego_actor = ego
    bridge.logger = _Logger()
    bridge.current_turn_indicator = TurnIndicatorsCommand.DISABLE
    bridge.current_hazard_lights = HazardLightsCommand.DISABLE
    bridge._light_state_available = True
    bridge._light_state_retry_at = 0.0
    bridge._state_lock = threading.Lock()
    return bridge


@pytest.fixture
def clock(monkeypatch):
    """Monotonic clock the test drives by hand."""

    class _Clock:
        now = 1000.0

        def advance(self, seconds):
            self.now += seconds

    fake = _Clock()
    monkeypatch.setattr(carla_ros.time, "monotonic", lambda: fake.now)
    return fake


@pytest.mark.parametrize(
    "turn_cmd, hazard_cmd, expected",
    [
        (TurnIndicatorsCommand.DISABLE, HazardLightsCommand.DISABLE, 0),
        (TurnIndicatorsCommand.ENABLE_LEFT, HazardLightsCommand.DISABLE, LEFT),
        (TurnIndicatorsCommand.ENABLE_RIGHT, HazardLightsCommand.DISABLE, RIGHT),
        # Hazard wins over the turn indicator.
        (TurnIndicatorsCommand.ENABLE_LEFT, HazardLightsCommand.ENABLE, LEFT | RIGHT),
    ],
)
def test_commanded_blinker_bits(turn_cmd, hazard_cmd, expected):
    assert carla_ros.carla_ros2_interface._commanded_blinker_bits(turn_cmd, hazard_cmd) == expected


def test_read_light_state_echoes_the_command_when_carla_rejects_the_call(clock):
    ego = _EgoActor()
    bridge = _bridge(ego)
    bridge.current_turn_indicator = TurnIndicatorsCommand.ENABLE_LEFT
    ego.broken = True

    # The failure is absorbed and reported as the commanded state, not as DISABLE.
    assert bridge._read_ego_light_state() == LEFT
    assert not bridge._light_state_available
    assert len(bridge.logger.warnings) == 1
    assert "std::exception" in bridge.logger.warnings[0]


def test_failed_light_state_backs_off_and_warns_once(clock):
    ego = _EgoActor()
    bridge = _bridge(ego)
    ego.broken = True

    bridge._read_ego_light_state()
    calls_after_first_failure = ego.get_calls

    # While backing off, CARLA is left alone and the warning is not repeated.
    for _ in range(10):
        clock.advance(carla_ros.LIGHT_STATE_RETRY_PERIOD_S / 100.0)
        assert bridge._read_ego_light_state() == 0
    assert ego.get_calls == calls_after_first_failure
    assert len(bridge.logger.warnings) == 1


def test_light_state_recovers_after_the_retry_period(clock):
    ego = _EgoActor(light_state=RIGHT)
    bridge = _bridge(ego)
    ego.broken = True
    bridge._read_ego_light_state()

    ego.broken = False
    clock.advance(carla_ros.LIGHT_STATE_RETRY_PERIOD_S)

    # The retry goes through, so CARLA's own state is reported again.
    assert bridge._read_ego_light_state() == RIGHT
    assert bridge._light_state_available
    assert len(bridge.logger.infos) == 1


def test_apply_light_state_survives_a_rejected_call_and_resumes(clock):
    ego = _EgoActor()
    bridge = _bridge(ego)
    bridge.current_turn_indicator = TurnIndicatorsCommand.ENABLE_RIGHT
    ego.broken = True

    bridge.apply_light_state()  # must not raise
    assert not bridge._light_state_available

    ego.broken = False
    clock.advance(carla_ros.LIGHT_STATE_RETRY_PERIOD_S)
    bridge.apply_light_state()

    assert bridge._light_state_available
    assert ego.light_state & RIGHT
    assert not ego.light_state & LEFT
