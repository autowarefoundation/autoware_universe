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

import logging
import os
from typing import Dict
from typing import List
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml


class SensorKitParser:
    """Parser for Autoware sensor kit calibration files."""

    def __init__(self, sensor_kit_name: str = "sample_sensor_kit"):
        """
        Initialize sensor kit parser.

        Args:
            sensor_kit_name: Name of the sensor kit (e.g., 'sample_sensor_kit', 'awsim_sensor_kit')
        """
        self.sensor_kit_name = sensor_kit_name
        self.logger = logging.getLogger(__name__)
        self.calibration_data = {}
        self.sensor_kit_calibration_data = {}
        self._load_calibration_files()

    def _load_calibration_files(self):
        """Load calibration YAML files from the sensor kit package."""
        try:
            # Get description package directory directly (more reliable in install space)
            desc_pkg_name = f"{self.sensor_kit_name}_description"
            desc_pkg_dir = get_package_share_directory(desc_pkg_name)

            # Load sensors_calibration.yaml (base_link to sensor_kit_base_link)
            sensors_calib_path = os.path.join(desc_pkg_dir, "config", "sensors_calibration.yaml")
            if os.path.exists(sensors_calib_path):
                with open(sensors_calib_path, "r") as f:
                    self.calibration_data = yaml.safe_load(f)
                    self.logger.info(f"Loaded sensors_calibration.yaml from {sensors_calib_path}")
            else:
                self.logger.warning(f"sensors_calibration.yaml not found at {sensors_calib_path}")

            # Load sensor_kit_calibration.yaml (sensor_kit_base_link to individual sensors)
            sensor_kit_calib_path = os.path.join(
                desc_pkg_dir, "config", "sensor_kit_calibration.yaml"
            )
            if os.path.exists(sensor_kit_calib_path):
                with open(sensor_kit_calib_path, "r") as f:
                    self.sensor_kit_calibration_data = yaml.safe_load(f)
                    self.logger.info(
                        f"Loaded sensor_kit_calibration.yaml from {sensor_kit_calib_path}"
                    )
            else:
                self.logger.warning(
                    f"sensor_kit_calibration.yaml not found at {sensor_kit_calib_path}"
                )

        except Exception as e:
            self.logger.warning(f"Failed to load calibration files for {self.sensor_kit_name}: {e}")
            self.logger.warning("Using default calibration values")
            self._set_default_calibration()

    def _set_default_calibration(self):
        """Set default calibration values if files cannot be loaded."""
        self.calibration_data = {
            "base_link": {
                "sensor_kit_base_link": {
                    "x": 0.9,
                    "y": 0.0,
                    "z": 2.0,
                    "roll": -0.001,
                    "pitch": 0.015,
                    "yaw": -0.0364,
                }
            }
        }
        self.sensor_kit_calibration_data = {"sensor_kit_base_link": {}}

    def get_sensor_kit_to_base_link_transform(self) -> Dict[str, float]:
        """
        Get transformation from base_link to sensor_kit_base_link.

        Returns:
            Dictionary with x, y, z, roll, pitch, yaw
        """
        if (
            "base_link" in self.calibration_data
            and "sensor_kit_base_link" in self.calibration_data["base_link"]
        ):
            return self.calibration_data["base_link"]["sensor_kit_base_link"]
        return {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    def get_sensor_transform(self, sensor_name: str) -> Optional[Dict[str, float]]:
        """
        Get transformation for a specific sensor relative to sensor_kit_base_link.

        Args:
            sensor_name: Name of the sensor (e.g., 'velodyne_top_base_link', 'camera0/camera_link')

        Returns:
            Dictionary with x, y, z, roll, pitch, yaw or None if not found
        """
        if "sensor_kit_base_link" in self.sensor_kit_calibration_data:
            sensors = self.sensor_kit_calibration_data["sensor_kit_base_link"]

            # Try exact match first
            if sensor_name in sensors:
                return sensors[sensor_name]

            # Try with _base_link suffix
            sensor_with_suffix = f"{sensor_name}_base_link"
            if sensor_with_suffix in sensors:
                return sensors[sensor_with_suffix]

            # Try with /camera_link suffix
            sensor_with_camera = f"{sensor_name}/camera_link"
            if sensor_with_camera in sensors:
                return sensors[sensor_with_camera]

            # Try with _link suffix
            sensor_with_link = f"{sensor_name}_link"
            if sensor_with_link in sensors:
                return sensors[sensor_with_link]

        return None

    def get_available_sensors(self) -> List[str]:
        """
        Get list of all available sensors in the sensor kit.

        Returns:
            List of sensor names
        """
        if "sensor_kit_base_link" in self.sensor_kit_calibration_data:
            return list(self.sensor_kit_calibration_data["sensor_kit_base_link"].keys())
        return []

    def get_sensor_full_transform(self, sensor_name: str) -> Optional[Dict[str, float]]:
        """
        Get full transformation from base_link to sensor.

        Args:
            sensor_name: Name of the sensor

        Returns:
            Combined transformation from base_link to sensor
        """
        # Get base_link to sensor_kit_base_link transform
        base_to_kit = self.get_sensor_kit_to_base_link_transform()

        # Get sensor_kit_base_link to sensor transform
        kit_to_sensor = self.get_sensor_transform(sensor_name)

        if kit_to_sensor is None:
            self.logger.warning(f"Sensor {sensor_name} not found in calibration")
            return None

        # Combine transformations
        combined_transform = self._combine_transforms(base_to_kit, kit_to_sensor)
        return combined_transform

    def _combine_transforms(
        self, transform1: Dict[str, float], transform2: Dict[str, float]
    ) -> Dict[str, float]:
        """
        Combine two transformations.

        Args:
            transform1: First transformation (base to intermediate)
            transform2: Second transformation (intermediate to final)

        Returns:
            Combined transformation
        """
        # Convert to transformation matrices
        T1 = self._dict_to_transform_matrix(transform1)
        T2 = self._dict_to_transform_matrix(transform2)

        # Combine transforms
        T_combined = np.dot(T1, T2)

        # Convert back to dict
        return self._transform_matrix_to_dict(T_combined)

    def _dict_to_transform_matrix(self, transform: Dict[str, float]) -> np.ndarray:
        """Convert transform dictionary to 4x4 transformation matrix."""
        x = transform.get("x", 0.0)
        y = transform.get("y", 0.0)
        z = transform.get("z", 0.0)
        roll = transform.get("roll", 0.0)
        pitch = transform.get("pitch", 0.0)
        yaw = transform.get("yaw", 0.0)

        # Create rotation matrix from Euler angles
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)

        matrix = np.eye(4)
        matrix[0, 0] = cy * cp
        matrix[0, 1] = cy * sp * sr - sy * cr
        matrix[0, 2] = cy * sp * cr + sy * sr
        matrix[1, 0] = sy * cp
        matrix[1, 1] = sy * sp * sr + cy * cr
        matrix[1, 2] = sy * sp * cr - cy * sr
        matrix[2, 0] = -sp
        matrix[2, 1] = cp * sr
        matrix[2, 2] = cp * cr

        matrix[0, 3] = x
        matrix[1, 3] = y
        matrix[2, 3] = z

        return matrix

    def _transform_matrix_to_dict(self, matrix: np.ndarray) -> Dict[str, float]:
        """Convert 4x4 transformation matrix to transform dictionary."""
        x = matrix[0, 3]
        y = matrix[1, 3]
        z = matrix[2, 3]

        # Extract Euler angles from rotation matrix
        sy = np.sqrt(matrix[0, 0] ** 2 + matrix[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(matrix[2, 1], matrix[2, 2])
            pitch = np.arctan2(-matrix[2, 0], sy)
            yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
        else:
            roll = np.arctan2(-matrix[1, 2], matrix[1, 1])
            pitch = np.arctan2(-matrix[2, 0], sy)
            yaw = 0.0

        return {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
        }

    def map_autoware_to_carla_sensor_type(self, sensor_name: str) -> Optional[str]:
        """
        Map Autoware sensor name to CARLA sensor type.

        Args:
            sensor_name: Autoware sensor name

        Returns:
            CARLA sensor type string or None if no mapping exists
        """
        # Define mapping rules
        if "velodyne" in sensor_name.lower() or "lidar" in sensor_name.lower():
            return "sensor.lidar.ray_cast"
        elif "camera" in sensor_name.lower() and "rgb" not in sensor_name.lower():
            return "sensor.camera.rgb"
        elif "imu" in sensor_name.lower():
            return "sensor.other.imu"
        elif "gnss" in sensor_name.lower():
            return "sensor.other.gnss"

        return None
