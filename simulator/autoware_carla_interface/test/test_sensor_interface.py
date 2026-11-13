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

"""Unit tests for the frame matching SensorInterface performs on its queue."""

import pytest

# The module imports the CARLA Python package at import time.
pytest.importorskip("carla")

from autoware_carla_interface.modules.carla_wrapper import SensorInterface  # noqa: E402


def _interface(*tags):
    interface = SensorInterface()
    for tag in tags:
        interface.register_sensor(tag, object())
    return interface


def test_returns_measurements_captured_at_or_before_the_frame():
    interface = _interface("camera", "lidar")
    interface.update_sensor("camera", "image-10", 10)
    interface.update_sensor("lidar", "cloud-10", 10)

    assert interface.get_data(10) == [("camera", 10, "image-10"), ("lidar", 10, "cloud-10")]


def test_holds_a_later_frame_until_the_loop_reaches_it():
    interface = _interface("camera")
    interface.update_sensor("camera", "image-11", 11)

    # Frame 11 has not been processed yet: nothing is due, and the measurement
    # is not dropped either.
    assert interface.get_data(10) == []
    assert interface.get_data(11) == [("camera", 11, "image-11")]


def test_keeps_every_measurement_of_a_sensor_faster_than_the_step():
    interface = _interface("imu")
    for frame in (8, 9, 10):
        interface.update_sensor("imu", f"imu-{frame}", frame)

    # An IMU ticking faster than the loop delivers several measurements between
    # two calls; each carries the frame it was captured on.
    assert interface.get_data(10) == [
        ("imu", 8, "imu-8"),
        ("imu", 9, "imu-9"),
        ("imu", 10, "imu-10"),
    ]


def test_a_late_callback_keeps_the_frame_it_was_captured_on():
    interface = _interface("camera")
    assert interface.get_data(10) == []

    # The callback for frame 10 arrives while the loop is on frame 12.
    interface.update_sensor("camera", "image-10", 10)
    assert interface.get_data(12) == [("camera", 10, "image-10")]
