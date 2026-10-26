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

"""Unit tests for the publish throttle of SensorRegistry."""

from autoware_carla_interface.modules.sensor_manager import SensorConfig
from autoware_carla_interface.modules.sensor_manager import SensorRegistry

STEP = 1.0 / 60.0


def _registry(frequency_hz, parameters=None):
    registry = SensorRegistry()
    registry.register_sensor(
        SensorConfig(
            sensor_id="cam",
            sensor_type="camera",
            carla_type="sensor.camera.rgb",
            frame_id="cam_link",
            topic_image="/image",
            frequency_hz=frequency_hz,
            parameters=parameters or {},
        )
    )
    # The first frame always publishes; start the clock from there.
    registry.should_publish("cam", 0.0)
    registry.update_sensor_timestamp("cam", 0.0)
    return registry


def test_frame_exactly_on_the_period_publishes():
    registry = _registry(25.0)
    # Accumulated the way a simulation clock accumulates it, the elapsed time
    # can fall a rounding step short of the period.
    assert registry.should_publish("cam", 0.04 - 1e-12)


def test_frame_a_whole_step_early_is_throttled_without_a_sensor_tick():
    registry = _registry(25.0)
    # Two steps of a 1/60 s simulation, against a 0.04 s period.
    assert not registry.should_publish("cam", 2 * STEP)


def test_frame_a_step_early_publishes_when_the_mapping_sets_a_sensor_tick():
    registry = _registry(25.0, {"sensor_tick": 0.04})
    # CARLA holds a 0.04 s tick at a 1/60 s step by alternating two- and
    # three-step gaps. The two-step arrival is 6.7 ms early; dropping it costs
    # the sensor 40% of the frames CARLA was asked to produce.
    assert registry.should_publish("cam", 2 * STEP)


def test_a_frame_of_the_next_capture_cycle_is_still_throttled():
    registry = _registry(12.5, {"sensor_tick": 0.04})
    # Publishing at half the capture rate: the capture one tick in is early by
    # a whole tick, which the tolerance must not admit.
    assert not registry.should_publish("cam", 0.04)
    assert registry.should_publish("cam", 0.08 - 1e-12)
