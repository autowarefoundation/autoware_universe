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

"""Sensor kit configuration loader for CARLA-Autoware interface."""

import logging
import os
from pathlib import Path
from typing import Any
from typing import Dict
from typing import List
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import yaml

from .coordinate_transformer import CoordinateTransformer
from .sensor_manager import SensorConfig

# Default vehicle wheelbase in meters (distance between front and rear axles)
DEFAULT_WHEELBASE = 2.850


class SensorKitLoader:
    """Loader for Autoware sensor kit configurations."""

    def __init__(self, logger: Optional[logging.Logger] = None):
        """Initialize sensor kit loader.

        Args:
            logger: Logger instance for output
        """
        self.logger = logger or logging.getLogger(__name__)
        self.sensor_mapping: Dict[str, Any] = {}
        self.sensor_kit_path: Optional[Path] = None
        self.wheelbase: float = DEFAULT_WHEELBASE

    def load_sensor_mapping(self, mapping_file: Optional[str] = None) -> bool:
        """Load sensor mapping configuration.

        Args:
            mapping_file: Path to sensor mapping YAML file

        Returns:
            True if successfully loaded, False otherwise
        """
        try:
            if not mapping_file:
                pkg_dir = get_package_share_directory("autoware_carla_interface")
                mapping_file = os.path.join(pkg_dir, "config", "sensor_mapping.yaml")

            if not os.path.exists(mapping_file):
                source_mapping = (
                    Path(__file__).resolve().parents[3] / "config" / "sensor_mapping.yaml"
                )
                if source_mapping.exists():
                    mapping_file = str(source_mapping)
                else:
                    self.logger.error(f"Sensor mapping file not found: {mapping_file}")
                    return False

            with open(mapping_file, "r") as f:
                self.sensor_mapping = yaml.safe_load(f)

            # Validate YAML structure
            if not self.sensor_mapping or not isinstance(self.sensor_mapping, dict):
                raise ValueError(
                    f"Invalid or empty YAML file: {mapping_file}. "
                    f"File must contain a valid YAML dictionary."
                )

            # Validate required sections
            required_keys = ["sensor_mappings", "enabled_sensors"]
            missing = [k for k in required_keys if k not in self.sensor_mapping]
            if missing:
                raise ValueError(
                    f"Missing required keys in sensor mapping file: {missing}. "
                    f"Required keys: {required_keys}"
                )

            # Validate sensor_mappings structure
            if not isinstance(self.sensor_mapping["sensor_mappings"], dict):
                raise ValueError("sensor_mappings must be a dictionary")

            # Validate enabled_sensors structure
            if not isinstance(self.sensor_mapping["enabled_sensors"], list):
                raise ValueError("enabled_sensors must be a list")

            # Load vehicle configuration if present
            if "vehicle_config" in self.sensor_mapping:
                vehicle_config = self.sensor_mapping["vehicle_config"]
                if not isinstance(vehicle_config, dict):
                    raise ValueError("vehicle_config must be a dictionary")

                if "wheelbase" in vehicle_config:
                    wheelbase_value = vehicle_config["wheelbase"]
                    if not isinstance(wheelbase_value, (int, float)):
                        raise ValueError(
                            f"wheelbase must be a number, got: {type(wheelbase_value).__name__}"
                        )
                    if wheelbase_value <= 0:
                        raise ValueError(f"wheelbase must be positive, got: {wheelbase_value}")
                    self.wheelbase = float(wheelbase_value)
                    self.logger.info(f"Using wheelbase from config: {self.wheelbase}m")
                else:
                    self.logger.info(f"Using default wheelbase: {self.wheelbase}m")
            else:
                self.logger.info(f"Using default wheelbase: {self.wheelbase}m")

            self.logger.info(f"Loaded sensor mapping from: {mapping_file}")
            return True

        except ValueError as e:
            self.logger.error(f"YAML validation error: {e}")
            return False
        except yaml.YAMLError as e:
            self.logger.error(f"YAML parsing error in {mapping_file}: {e}")
            return False
        except Exception as e:
            self.logger.error(f"Failed to load sensor mapping: {e}")
            return False

    def find_sensor_kit_path(self, sensor_kit_name: str) -> Optional[Path]:
        """Find sensor kit calibration directory using ament_index.

        This method works in both source and install spaces by using ROS2's
        package discovery mechanism instead of hardcoded paths.

        Args:
            sensor_kit_name: Name of the sensor kit package (e.g., 'carla_sensor_kit_launch')

        Returns:
            Path to sensor kit config directory or None if not found

        Raises:
            FileNotFoundError: If sensor kit calibration cannot be found
        """
        from ament_index_python.packages import PackageNotFoundError

        tried_packages = []

        # Try 1: Use the provided package name directly
        try:
            sensor_kit_dir = get_package_share_directory(sensor_kit_name)
            calib_path = Path(sensor_kit_dir) / "config"

            if (calib_path / "sensor_kit_calibration.yaml").exists():
                self.sensor_kit_path = calib_path
                self.logger.info(f"Found sensor kit at: {calib_path}")
                return calib_path
            else:
                self.logger.debug(
                    f"Package '{sensor_kit_name}' found but missing "
                    f"config/sensor_kit_calibration.yaml"
                )
        except PackageNotFoundError:
            self.logger.debug(f"Package '{sensor_kit_name}' not found in ament index")
            tried_packages.append(sensor_kit_name)

        # Try 2: Replace '_launch' with '_description' suffix
        # Common pattern: carla_sensor_kit_launch -> carla_sensor_kit_description
        desc_name = sensor_kit_name.replace("_launch", "_description")
        if desc_name != sensor_kit_name:
            try:
                desc_dir = get_package_share_directory(desc_name)
                calib_path = Path(desc_dir) / "config"

                if (calib_path / "sensor_kit_calibration.yaml").exists():
                    self.sensor_kit_path = calib_path
                    self.logger.info(f"Found sensor kit at: {calib_path} (using {desc_name})")
                    return calib_path
                else:
                    self.logger.debug(
                        f"Package '{desc_name}' found but missing "
                        f"config/sensor_kit_calibration.yaml"
                    )
            except PackageNotFoundError:
                self.logger.debug(f"Package '{desc_name}' not found in ament index")
                tried_packages.append(desc_name)

        # Try 3: If sensor_kit_name doesn't end with _launch or _description, try adding them
        if not sensor_kit_name.endswith(("_launch", "_description")):
            for suffix in ["_description", "_launch"]:
                variant = sensor_kit_name + suffix
                try:
                    variant_dir = get_package_share_directory(variant)
                    calib_path = Path(variant_dir) / "config"

                    if (calib_path / "sensor_kit_calibration.yaml").exists():
                        self.sensor_kit_path = calib_path
                        self.logger.info(f"Found sensor kit at: {calib_path} (using {variant})")
                        return calib_path
                except PackageNotFoundError:
                    tried_packages.append(variant)

        # All attempts failed - raise detailed error
        raise FileNotFoundError(
            f"Sensor kit calibration not found for '{sensor_kit_name}'.\n"
            f"Attempted packages: {tried_packages}\n"
            f"Required file: config/sensor_kit_calibration.yaml\n"
            f"Ensure the sensor kit description package is:\n"
            f"  1. Built and installed in your ROS2 workspace\n"
            f"  2. Contains config/sensor_kit_calibration.yaml\n"
            f"  3. Visible to 'ros2 pkg list' command"
        )

    def parse_sensor_kit_calibration(self, sensor_kit_path: Path) -> Dict[str, Any]:
        """Parse sensor calibration from sensor kit.

        Currently only loads extrinsic calibration (sensor positions/orientations).
        Camera intrinsics and other sensor-specific parameters are handled by CARLA
        and the sensor_mapping.yaml configuration.

        Args:
            sensor_kit_path: Path to sensor kit calibration directory

        Returns:
            Dictionary of sensor configurations with transforms
        """
        sensors = {}

        # Parse extrinsic calibration (sensor poses)
        extrinsic_file = sensor_kit_path / "sensor_kit_calibration.yaml"
        if extrinsic_file.exists():
            try:
                with open(extrinsic_file, "r") as f:
                    calibration_data = yaml.safe_load(f)
                    sensors = self._parse_extrinsic_calibration(calibration_data)
            except Exception as e:
                self.logger.error(f"Failed to parse extrinsic calibration: {e}")

        return sensors

    def _parse_extrinsic_calibration(self, calibration_data: Dict) -> Dict[str, Any]:
        """Parse extrinsic calibration data.

        Args:
            calibration_data: Raw calibration YAML data

        Returns:
            Dictionary of sensor configurations
        """
        sensors = {}

        # Extract sensor_kit_base_link wrapper if present in calibration data
        if "sensor_kit_base_link" in calibration_data:
            sensor_base_dict = calibration_data["sensor_kit_base_link"]
        else:
            # Fallback: assume data is already unwrapped
            sensor_base_dict = calibration_data

        for sensor_name, transform_data in sensor_base_dict.items():
            if not isinstance(transform_data, dict):
                continue

            # Now transform_data should have x, y, z, etc.
            if "x" in transform_data:
                # Direct transform format
                sensors[sensor_name] = {
                    "frame_id": sensor_name,
                    "transform": self._extract_transform(transform_data),
                }
            elif "base_link" in transform_data:
                # Nested format with base_link
                base_data = transform_data["base_link"]
                sensors[sensor_name] = {
                    "frame_id": sensor_name,
                    "transform": self._extract_transform(base_data),
                }

        return sensors

    def _extract_transform(self, transform_data: Dict) -> Dict[str, float]:
        """Extract transform values from calibration data.

        Autoware sensor calibration files use radians for angles.

        Args:
            transform_data: Transform dictionary with x, y, z, roll, pitch, yaw
                           Angles are expected in radians (Autoware standard)

        Returns:
            Transform dictionary with angles in radians
        """
        return {
            "x": float(transform_data.get("x", 0.0)),
            "y": float(transform_data.get("y", 0.0)),
            "z": float(transform_data.get("z", 0.0)),
            "roll": float(transform_data.get("roll", 0.0)),
            "pitch": float(transform_data.get("pitch", 0.0)),
            "yaw": float(transform_data.get("yaw", 0.0)),
        }

    def normalize_sensor_name(self, sensor_name: str) -> str:
        """Normalize sensor name for matching.

        Args:
            sensor_name: Original sensor name

        Returns:
            Normalized sensor name
        """
        normalized = sensor_name

        # Apply normalization rules from mapping
        if "normalization" in self.sensor_mapping:
            rules = self.sensor_mapping["normalization"]
            if "strip_suffixes" in rules:
                for suffix in rules["strip_suffixes"]:
                    if normalized.endswith(suffix):
                        normalized = normalized[: -len(suffix)]
                        break

        return normalized

    def is_sensor_enabled(self, sensor_name: str) -> bool:
        """Check if sensor is enabled in configuration.

        Args:
            sensor_name: Sensor name to check

        Returns:
            True if enabled, False otherwise
        """
        if "enabled_sensors" not in self.sensor_mapping:
            return True  # Enable all by default if no list specified

        enabled_list = self.sensor_mapping["enabled_sensors"]
        normalized = self.normalize_sensor_name(sensor_name)

        # Check both original and normalized names
        return sensor_name in enabled_list or normalized in enabled_list

    def build_sensor_configs(self, sensor_kit_name: Optional[str] = None) -> List[SensorConfig]:
        """Build sensor configurations from sensor kit calibration.

        Args:
            sensor_kit_name: Name of sensor kit to use

        Returns:
            List of sensor configurations

        Raises:
            RuntimeError: If sensor kit is not found or calibration cannot be loaded
        """
        if not sensor_kit_name:
            raise RuntimeError(
                "sensor_kit_name is required. The mapping file alone does not contain "
                "sensor transforms, which would result in all sensors spawning at (0,0,0). "
                "Please specify a valid sensor_kit_name parameter."
            )

        # Load from sensor kit (required for transforms)
        # find_sensor_kit_path now raises FileNotFoundError with detailed message
        try:
            sensor_kit_path = self.find_sensor_kit_path(sensor_kit_name)
        except FileNotFoundError as e:
            # Re-raise as RuntimeError for consistency with existing error handling
            raise RuntimeError(str(e)) from e

        kit_sensors = self.parse_sensor_kit_calibration(sensor_kit_path)
        if not kit_sensors:
            raise RuntimeError(
                f"No sensors found in calibration at {sensor_kit_path}. "
                f"Check that sensor_kit_calibration.yaml exists and is valid."
            )

        configs = self._create_configs_from_kit(kit_sensors)
        return configs

    def _create_configs_from_kit(self, kit_sensors: Dict) -> List[SensorConfig]:
        """Create sensor configs from sensor kit data.

        Args:
            kit_sensors: Parsed sensor kit data

        Returns:
            List of sensor configurations

        Raises:
            RuntimeError: If enabled sensors lack transform data
        """
        configs = []
        mappings = self.sensor_mapping.get("sensor_mappings", {})
        missing_transforms = []

        for sensor_name, sensor_data in kit_sensors.items():
            if not self.is_sensor_enabled(sensor_name):
                continue

            normalized = self.normalize_sensor_name(sensor_name)

            # Find mapping for this sensor
            mapping = None
            for mapping_key in mappings:
                if self.normalize_sensor_name(mapping_key) == normalized:
                    mapping = mappings[mapping_key]
                    break

            if not mapping:
                self.logger.debug(f"No mapping found for sensor: {sensor_name}")
                continue

            # Validate transform is present
            transform = sensor_data.get("transform")
            if not transform:
                missing_transforms.append(sensor_name)
                self.logger.error(
                    f"Sensor '{sensor_name}' is enabled but has no transform in calibration"
                )
                continue

            config = self._create_sensor_config(
                sensor_name=sensor_name, mapping=mapping, transform=transform
            )
            configs.append(config)

        if missing_transforms:
            raise RuntimeError(
                f"Enabled sensors missing transforms in calibration: {missing_transforms}. "
                f"All enabled sensors must have pose calibration to avoid spawning at (0,0,0)."
            )

        return configs

    def _create_sensor_config(
        self, sensor_name: str, mapping: Dict, transform: Optional[Dict] = None
    ) -> SensorConfig:
        """Create a sensor configuration.

        Args:
            sensor_name: Name of the sensor
            mapping: Sensor mapping data
            transform: Optional transform data

        Returns:
            Sensor configuration
        """
        ros_config = mapping.get("ros_config", {})

        # Build topic names
        topic = None
        topic_image = None
        topic_info = None

        if "topic" in ros_config:
            topic = ros_config["topic"]
        elif "topic_base" in ros_config and "topic_suffix" in ros_config:
            topic = ros_config["topic_base"] + ros_config["topic_suffix"]

        if "topic_image" in ros_config:
            topic_image = ros_config["topic_image"]
        if "topic_info" in ros_config:
            topic_info = ros_config["topic_info"]

        # Convert base_link coordinates to vehicle center if needed
        if transform:
            transform = self._carla_baselink_to_vehicle_center_transform(transform)

        config = SensorConfig(
            sensor_id=mapping.get("id", sensor_name),
            sensor_type=mapping.get("carla_type", "unknown"),
            carla_type=mapping.get("carla_type", "unknown"),
            frame_id=ros_config.get("frame_id", sensor_name),
            topic=topic,
            topic_image=topic_image,
            topic_info=topic_info,
            frequency_hz=ros_config.get("frequency_hz", 20.0),
            qos_profile=ros_config.get("qos_profile", "reliable"),
            parameters=mapping.get("parameters", {}),
            transform=transform,
            enabled=True,
        )

        return config

    def _carla_baselink_to_vehicle_center_transform(
        self, baselink_transform: Dict
    ) -> Dict[str, float]:
        """Convert CARLA base_link coordinates to CARLA vehicle center coordinates.

        The carla_sensor_kit calibration uses CARLA coordinate conventions (Y-right).
        This function applies the wheelbase offset to translate from base_link
        (rear axle center) to vehicle center, without coordinate system conversion.

        Args:
            baselink_transform: Transform dict with x, y, z, roll, pitch, yaw in CARLA coords

        Returns:
            Transform dict in vehicle center coordinates
        """
        # Extract values
        x = baselink_transform["x"]
        y = baselink_transform["y"]
        z = baselink_transform["z"]
        roll = baselink_transform["roll"]
        pitch = baselink_transform["pitch"]
        yaw = baselink_transform["yaw"]

        # Convert location (only wheelbase offset, no coordinate flip)
        location = CoordinateTransformer.carla_base_link_to_vehicle_center_location(
            x, y, z, wheelbase=self.wheelbase
        )

        # Convert rotation (only unit conversion, no angle negation)
        rotation = CoordinateTransformer.carla_rotation_to_carla_rotation(roll, pitch, yaw)

        return {
            "x": location.x,
            "y": location.y,
            "z": location.z,
            "roll": rotation.roll,
            "pitch": rotation.pitch,
            "yaw": rotation.yaw,
        }
