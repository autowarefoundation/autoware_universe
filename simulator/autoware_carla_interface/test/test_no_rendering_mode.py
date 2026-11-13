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

"""Unit tests for the camera sensors left out of the spawn list while rendering is off."""

import pytest

# The module imports the CARLA Python package at import time.
pytest.importorskip("carla")

from autoware_carla_interface.carla_ros import carla_ros2_interface  # noqa: E402

LIDAR = {"type": "sensor.lidar.ray_cast", "id": "top"}
IMU = {"type": "sensor.other.imu", "id": "imu"}
GNSS = {"type": "sensor.other.gnss", "id": "gnss"}
CAM_FRONT = {"type": "sensor.camera.rgb", "id": "CAM_FRONT"}
CAM_BACK = {"type": "sensor.camera.rgb", "id": "CAM_BACK"}
CAM_DEPTH = {"type": "sensor.camera.depth", "id": "CAM_DEPTH"}


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def _bridge(no_rendering_mode):
    """Build a bridge instance carrying only the state the sensor filter touches."""
    bridge = carla_ros2_interface.__new__(carla_ros2_interface)
    bridge.param_values = {"no_rendering_mode": no_rendering_mode}
    bridge.logger = _Logger()
    return bridge


def test_cameras_are_spawned_while_rendering():
    bridge = _bridge(False)
    specs = [LIDAR, CAM_FRONT, CAM_BACK, IMU, GNSS]
    assert bridge._skip_cameras_in_no_rendering_mode(specs) == specs
    assert bridge.logger.warnings == []


def test_cameras_are_skipped_without_rendering():
    bridge = _bridge(True)
    # The surviving sensors keep their order.
    assert bridge._skip_cameras_in_no_rendering_mode([LIDAR, CAM_FRONT, IMU, CAM_BACK, GNSS]) == [
        LIDAR,
        IMU,
        GNSS,
    ]


def test_every_skipped_camera_is_named_once():
    bridge = _bridge(True)
    bridge._skip_cameras_in_no_rendering_mode([CAM_FRONT, CAM_BACK, CAM_DEPTH, LIDAR])
    assert len(bridge.logger.warnings) == 1
    warning = bridge.logger.warnings[0]
    assert "CAM_FRONT" in warning
    assert "CAM_BACK" in warning
    # A depth camera needs the renderer just the same.
    assert "CAM_DEPTH" in warning


def test_camera_free_mapping_passes_through_silently():
    bridge = _bridge(True)
    specs = [LIDAR, IMU, GNSS]
    assert bridge._skip_cameras_in_no_rendering_mode(specs) == specs
    assert bridge.logger.warnings == []


def test_the_caller_s_list_is_left_alone():
    bridge = _bridge(True)
    specs = [LIDAR, CAM_FRONT]
    bridge._skip_cameras_in_no_rendering_mode(specs)
    assert specs == [LIDAR, CAM_FRONT]
