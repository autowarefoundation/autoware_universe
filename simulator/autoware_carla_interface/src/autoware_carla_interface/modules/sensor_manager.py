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
"""Sensor management module for CARLA-Autoware interface."""

from dataclasses import dataclass
from dataclasses import field
import logging
from typing import Any
from typing import Dict
from typing import List
from typing import Optional


@dataclass
class SensorConfig:
    """Configuration for a single sensor."""

    sensor_id: str
    sensor_type: str
    carla_type: str
    frame_id: str
    topic: Optional[str] = None
    topic_image: Optional[str] = None
    topic_info: Optional[str] = None
    frequency_hz: float = 20.0
    qos_profile: str = "reliable"
    parameters: Dict[str, Any] = field(default_factory=dict)
    transform: Optional[Dict[str, float]] = None
    covariance: Optional[Dict[str, float]] = None
    enabled: bool = True
    publisher: Optional[Any] = None
    publisher_info: Optional[Any] = None
    last_publish_time: Optional[float] = None
    first_data: bool = True


class SensorRegistry:
    """Centralized registry for managing sensors."""

    def __init__(self, logger: Optional[logging.Logger] = None):
        """
        Initialize sensor registry.

        Args:
        ----
            logger: Logger instance for output

        """
        self.logger = logger or logging.getLogger(__name__)
        self.sensors: Dict[str, SensorConfig] = {}
        self.type_mapping: Dict[str, str] = {}

        # Add pseudo-sensors for control frequencies
        self._add_control_sensors()

    def _add_control_sensors(self):
        """Add pseudo-sensors for ego status and pose frequency control."""
        # Ego status pseudo-sensor
        self.sensors["status"] = SensorConfig(
            sensor_id="status",
            sensor_type="pseudo.ego_status",
            carla_type="pseudo.ego_status",
            frame_id="base_link",
            frequency_hz=50.0,
            enabled=True,
        )

        # Pose pseudo-sensor
        self.sensors["pose"] = SensorConfig(
            sensor_id="pose",
            sensor_type="pseudo.pose",
            carla_type="pseudo.pose",
            frame_id="map",
            frequency_hz=2.0,
            enabled=True,
        )

    def register_sensor(self, config: SensorConfig) -> bool:
        """
        Register a new sensor.

        Args:
        ----
            config: Sensor configuration

        Returns
        -------
            True if successfully registered, False otherwise

        """
        if config.sensor_id in self.sensors:
            self.logger.warning(f"Sensor {config.sensor_id} already registered, updating...")

        self.sensors[config.sensor_id] = config
        self.type_mapping[config.sensor_id] = config.sensor_type

        self.logger.info(
            f"Registered sensor: {config.sensor_id} "
            f"(type: {config.sensor_type}, topic: {config.topic or config.topic_image})"
        )
        return True

    def get_sensor(self, sensor_id: str) -> Optional[SensorConfig]:
        """
        Get sensor configuration by ID.

        Args:
        ----
            sensor_id: Sensor identifier

        Returns
        -------
            Sensor configuration or None if not found

        """
        return self.sensors.get(sensor_id)

    def get_sensors_by_type(self, sensor_type: str) -> List[SensorConfig]:
        """
        Get all sensors of a specific type.

        Args:
        ----
            sensor_type: CARLA sensor type

        Returns
        -------
            List of matching sensor configurations

        """
        return [sensor for sensor in self.sensors.values() if sensor.carla_type == sensor_type]

    def get_enabled_sensors(self) -> List[SensorConfig]:
        """
        Get all enabled sensors.

        Returns
        -------
            List of enabled sensor configurations

        """
        return [sensor for sensor in self.sensors.values() if sensor.enabled]

    def update_sensor_timestamp(self, sensor_id: str, timestamp: float) -> bool:
        """
        Update sensor last publish timestamp.

        Args:
        ----
            sensor_id: Sensor identifier
            timestamp: New timestamp

        Returns
        -------
            True if updated, False if sensor not found

        """
        sensor = self.get_sensor(sensor_id)
        if sensor:
            sensor.last_publish_time = timestamp
            sensor.first_data = False
            return True
        return False

    def should_publish(self, sensor_id: str, current_time: float) -> bool:
        """
        Check if sensor should publish based on frequency.

        Args:
        ----
            sensor_id: Sensor identifier
            current_time: Current timestamp

        Returns
        -------
            True if should publish, False otherwise

        """
        sensor = self.get_sensor(sensor_id)
        if not sensor or not sensor.enabled:
            return False

        if sensor.first_data or sensor.last_publish_time is None:
            return True

        time_diff = current_time - sensor.last_publish_time
        return time_diff >= (1.0 / sensor.frequency_hz)

    def get_all_sensors(self) -> Dict[str, SensorConfig]:
        """
        Get all registered sensors.

        Returns
        -------
            Dictionary of all sensors

        """
        return self.sensors.copy()

    def clear(self):
        """Clear all registered sensors except control sensors."""
        control_sensors = {
            k: v for k, v in self.sensors.items() if v.sensor_type.startswith("pseudo.")
        }
        self.sensors.clear()
        self.type_mapping.clear()
        self.sensors.update(control_sensors)

    def __len__(self) -> int:
        """Get number of registered sensors."""
        return len(self.sensors)

    def __contains__(self, sensor_id: str) -> bool:
        """Check if sensor is registered."""
        return sensor_id in self.sensors
