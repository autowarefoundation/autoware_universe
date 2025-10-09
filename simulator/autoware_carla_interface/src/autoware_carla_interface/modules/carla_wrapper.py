#!/usr/bin/env python3

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

from __future__ import print_function

import logging
from queue import Queue

import carla
import numpy as np

from .carla_data_provider import CarlaDataProvider


# Sensor Wrapper for Agent
class SensorReceivedNoData(Exception):
    """Exceptions when no data received from the sensors."""


class GenericMeasurement(object):
    def __init__(self, data, frame):
        self.data = data
        self.frame = frame


class CallBack(object):
    def __init__(self, tag, sensor, data_provider):
        self._tag = tag
        self._data_provider = data_provider

        self._data_provider.register_sensor(tag, sensor)

    def __call__(self, data):
        if isinstance(data, carla.Image):
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, carla.LidarMeasurement):
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, carla.GnssMeasurement):
            self._parse_gnss_cb(data, self._tag)
        elif isinstance(data, carla.IMUMeasurement):
            self._parse_imu_cb(data, self._tag)
        elif isinstance(data, GenericMeasurement):
            self._parse_pseudo_sensor(data, self._tag)
        else:
            logging.error("No callback method for this sensor.")

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image, tag):
        self._data_provider.update_sensor(tag, image, image.frame)

    def _parse_lidar_cb(self, lidar_data, tag):
        self._data_provider.update_sensor(tag, lidar_data, lidar_data.frame)

    def _parse_imu_cb(self, imu_data, tag):
        self._data_provider.update_sensor(tag, imu_data, imu_data.frame)

    def _parse_gnss_cb(self, gnss_data, tag):
        array = np.array(
            [gnss_data.latitude, gnss_data.longitude, gnss_data.altitude], dtype=np.float64
        )
        self._data_provider.update_sensor(tag, array, gnss_data.frame)

    def _parse_pseudo_sensor(self, package, tag):
        self._data_provider.update_sensor(tag, package.data, package.frame)


class SensorInterface(object):
    """Interface for collecting sensor data from CARLA."""

    def __init__(self):
        """Initialize sensor interface."""
        self._sensors_objects = {}
        self._new_data_buffers = Queue()
        self.tag = ""  # Current sensor tag

    def register_sensor(self, tag, sensor):
        self.tag = tag
        if tag in self._sensors_objects:
            raise ValueError(f"Duplicated sensor tag [{tag}]")

        self._sensors_objects[tag] = sensor

    def update_sensor(self, tag, data, timestamp):
        if tag not in self._sensors_objects:
            raise ValueError(f"Sensor with tag [{tag}] has not been created")

        self._new_data_buffers.put((tag, timestamp, data))

    def get_data(self):
        """Get all available sensor data without blocking for all sensors."""
        from queue import Empty

        data_dict = {}

        # Non-blocking: get all available data from queue
        while True:
            try:
                sensor_data = self._new_data_buffers.get(block=False)
                data_dict[sensor_data[0]] = (sensor_data[1], sensor_data[2])
            except Empty:
                # Queue is empty, break and return whatever data we have
                break

        # Return available data immediately (could be partial sensor set)
        return data_dict


# Sensor Wrapper


class SensorWrapper(object):
    """Wrapper for managing CARLA sensors attached to a vehicle."""

    def __init__(self, agent):
        """Initialize sensor wrapper.

        Args:
            agent: Agent instance containing sensor configuration
        """
        self._agent = agent
        self._sensors_list = []  # Instance variable, not class variable

    def __call__(self):
        return self._agent()

    def setup_sensors(self, vehicle, debug_mode=False):
        """Create and attach sensors defined via sensor configuration.

        Args:
            vehicle: CARLA vehicle actor to attach sensors to
            debug_mode: Enable debug logging (unused)

        Raises:
            RuntimeError: If sensor spawning fails critically
        """
        bp_library = CarlaDataProvider.get_world().get_blueprint_library()

        for sensor_spec in self._agent.sensors["sensors"]:
            try:
                sensor_type = sensor_spec["type"]
                sensor_id = sensor_spec.get("id", "unknown")

                # Find sensor blueprint
                bp = bp_library.find(str(sensor_type))
                if bp is None:
                    logging.error(f"Blueprint not found for sensor type: {sensor_type}")
                    continue

                # Set sensor-specific attributes
                if sensor_type.startswith("sensor.camera"):
                    bp.set_attribute("image_size_x", str(sensor_spec["image_size_x"]))
                    bp.set_attribute("image_size_y", str(sensor_spec["image_size_y"]))
                    bp.set_attribute("fov", str(sensor_spec["fov"]))

                elif sensor_type.startswith("sensor.lidar"):
                    bp.set_attribute("range", str(sensor_spec["range"]))
                    bp.set_attribute("rotation_frequency", str(sensor_spec["rotation_frequency"]))
                    bp.set_attribute("channels", str(sensor_spec["channels"]))
                    bp.set_attribute("upper_fov", str(sensor_spec["upper_fov"]))
                    bp.set_attribute("lower_fov", str(sensor_spec["lower_fov"]))
                    bp.set_attribute("points_per_second", str(sensor_spec["points_per_second"]))

                elif sensor_type.startswith("sensor.other.gnss"):
                    # Set zero noise for clean simulation
                    bp.set_attribute("noise_alt_stddev", str(0.0))
                    bp.set_attribute("noise_lat_stddev", str(0.0))
                    bp.set_attribute("noise_lon_stddev", str(0.0))
                    bp.set_attribute("noise_alt_bias", str(0.0))
                    bp.set_attribute("noise_lat_bias", str(0.0))
                    bp.set_attribute("noise_lon_bias", str(0.0))

                elif sensor_type.startswith("sensor.other.imu"):
                    # Set zero noise for clean simulation
                    bp.set_attribute("noise_accel_stddev_x", str(0.0))
                    bp.set_attribute("noise_accel_stddev_y", str(0.0))
                    bp.set_attribute("noise_accel_stddev_z", str(0.0))
                    bp.set_attribute("noise_gyro_stddev_x", str(0.0))
                    bp.set_attribute("noise_gyro_stddev_y", str(0.0))
                    bp.set_attribute("noise_gyro_stddev_z", str(0.0))

                elif not sensor_type.startswith("sensor."):
                    # Unknown sensor type - skip
                    logging.warning(f"Unknown sensor type: {sensor_type}, skipping spawn")
                    continue

                # Create sensor transform (common for all sensor types)
                spawn_point = sensor_spec["spawn_point"]
                sensor_location = carla.Location(
                    x=spawn_point["x"], y=spawn_point["y"], z=spawn_point["z"]
                )
                sensor_rotation = carla.Rotation(
                    pitch=spawn_point["pitch"], roll=spawn_point["roll"], yaw=spawn_point["yaw"]
                )
                sensor_transform = carla.Transform(sensor_location, sensor_rotation)

                # Spawn sensor and attach to vehicle
                sensor = CarlaDataProvider.get_world().spawn_actor(bp, sensor_transform, vehicle)
                if sensor is None:
                    logging.error(
                        f"Failed to spawn sensor '{sensor_id}' of type {sensor_type}. "
                        f"Check spawn position and vehicle attachment."
                    )
                    continue

                sensor.listen(CallBack(sensor_id, sensor, self._agent.sensor_interface))
                self._sensors_list.append(sensor)
                logging.info(f"Successfully spawned sensor '{sensor_id}' ({sensor_type})")

            except KeyError as e:
                logging.error(
                    f"Missing required key {e} in sensor spec: {sensor_spec.get('id', 'unknown')}"
                )
                continue
            except Exception as e:
                logging.error(
                    f"Failed to setup sensor '{sensor_spec.get('id', 'unknown')}': {e}"
                )
                continue

        if not self._sensors_list:
            raise RuntimeError("No sensors were successfully spawned. Check sensor configuration.")

        # Tick once to spawn the sensors
        CarlaDataProvider.get_world().tick()

    def cleanup(self):
        """Cleanup sensors robustly.

        Stops and destroys all spawned sensors, continuing even if individual
        sensors fail to clean up. This prevents resource leaks.
        """
        cleanup_errors = []

        for i, sensor in enumerate(self._sensors_list):
            if sensor is not None:
                try:
                    # Stop sensor callbacks first
                    sensor.stop()
                except Exception as e:
                    cleanup_errors.append(f"Failed to stop sensor {i}: {e}")

                try:
                    # Destroy the actor
                    sensor.destroy()
                except Exception as e:
                    cleanup_errors.append(f"Failed to destroy sensor {i}: {e}")

                self._sensors_list[i] = None

        self._sensors_list = []

        # Log any cleanup errors but don't raise (we want to continue cleanup)
        if cleanup_errors:
            for error in cleanup_errors:
                logging.warning(error)
