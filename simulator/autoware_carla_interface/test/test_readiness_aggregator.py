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

"""Unit tests for the ROS-free readiness aggregation logic."""

from autoware_carla_interface.autoware_bridge.ad_api import LOCALIZATION_STATE_INITIALIZED
from autoware_carla_interface.autoware_bridge.ad_api import OPERATION_MODE_AUTONOMOUS
from autoware_carla_interface.autoware_bridge.ad_api import ROUTE_STATE_SET
from autoware_carla_interface.autoware_bridge.ad_api import ReadinessAggregator


def _initialized_and_routed() -> ReadinessAggregator:
    agg = ReadinessAggregator()
    agg.update_localization(LOCALIZATION_STATE_INITIALIZED)
    agg.update_routing(ROUTE_STATE_SET)
    return agg


def test_not_ready_or_engageable_when_empty():
    agg = ReadinessAggregator()
    assert agg.can_engage is False
    assert agg.ready is False


def test_can_engage_needs_init_route_and_availability():
    agg = _initialized_and_routed()
    assert agg.can_engage is False  # autonomous not available yet

    agg.update_operation_mode(mode=1, is_control_enabled=False, is_autonomous_available=True)
    assert agg.can_engage is True
    assert agg.ready is False  # available != engaged


def test_ready_needs_autonomous_engaged_with_control():
    agg = _initialized_and_routed()
    # Autonomous mode but control not yet enabled -> not ready.
    agg.update_operation_mode(
        mode=OPERATION_MODE_AUTONOMOUS, is_control_enabled=False, is_autonomous_available=True
    )
    assert agg.ready is False

    agg.update_operation_mode(
        mode=OPERATION_MODE_AUTONOMOUS, is_control_enabled=True, is_autonomous_available=True
    )
    assert agg.ready is True


def test_missing_localization_blocks_ready():
    agg = ReadinessAggregator()
    agg.update_routing(ROUTE_STATE_SET)
    agg.update_operation_mode(
        mode=OPERATION_MODE_AUTONOMOUS, is_control_enabled=True, is_autonomous_available=True
    )
    assert agg.ready is False
    assert agg.can_engage is False


def test_regressed_localization_clears_flags():
    agg = _initialized_and_routed()
    agg.update_operation_mode(
        mode=OPERATION_MODE_AUTONOMOUS, is_control_enabled=True, is_autonomous_available=True
    )
    assert agg.ready is True

    agg.update_localization(2)  # INITIALIZING
    assert agg.ready is False
