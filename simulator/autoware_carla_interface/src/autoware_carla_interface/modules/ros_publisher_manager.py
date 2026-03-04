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
"""ROS publisher management for CARLA sensors."""

import logging
from typing import Any
from typing import Dict
from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2

from .sensor_manager import SensorConfig
from .sensor_manager import SensorRegistry


class ROSPublisherManager:
    """Manager for ROS publishers."""

    def __init__(self, ros_node: Node, logger: Optional[logging.Logger] = None):
        """
        Initialize ROS publisher manager.

        Args:
        ----
            ros_node: ROS 2 node instance
            logger: Logger instance

        """
        self.ros_node = ros_node
        self.logger = logger or logging.getLogger(__name__)

        # Publisher storage by type
        self.camera_publishers: Dict[str, Any] = {}
        self.camera_info_publishers: Dict[str, Any] = {}
        self.lidar_publishers: Dict[str, Any] = {}
        self.imu_publishers: Dict[str, Any] = {}
        self.gnss_publishers: Dict[str, Any] = {}

        # QoS profiles
        self.qos_profiles = {
            "reliable": self._create_reliable_qos(),
            "best_effort": self._create_best_effort_qos(),
        }

    def _create_reliable_qos(self) -> QoSProfile:
        """Create reliable QoS profile."""
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        return qos

    def _create_best_effort_qos(self) -> QoSProfile:
        """Create best effort QoS profile."""
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        return qos

    def get_qos_profile(self, profile_name: str) -> QoSProfile:
        """
        Get QoS profile by name.

        Args:
        ----
            profile_name: Name of QoS profile

        Returns
        -------
            QoS profile

        """
        profile = self.qos_profiles.get(profile_name.lower())
        if not profile:
            self.logger.warning(f"Unknown QoS profile '{profile_name}', using reliable")
            profile = self.qos_profiles["reliable"]
        return profile

    def create_publishers_for_registry(self, sensor_registry: SensorRegistry):
        """
        Create publishers for all sensors in registry.

        Args:
        ----
            sensor_registry: Sensor registry

        """
        for sensor_id, sensor_config in sensor_registry.get_all_sensors().items():
            if sensor_config.sensor_type.startswith("pseudo."):
                continue  # Skip pseudo sensors

            self.create_publisher_for_sensor(sensor_config)

    def create_publisher_for_sensor(self, sensor_config: SensorConfig) -> bool:
        """
        Create publisher for a sensor.

        Args:
        ----
            sensor_config: Sensor configuration

        Returns
        -------
            True if publisher created, False otherwise

        """
        try:
            sensor_type = sensor_config.carla_type

            if sensor_type == "sensor.camera.rgb":
                return self._create_camera_publishers(sensor_config)
            elif sensor_type == "sensor.lidar.ray_cast":
                return self._create_lidar_publisher(sensor_config)
            elif sensor_type == "sensor.other.imu":
                return self._create_imu_publisher(sensor_config)
            elif sensor_type == "sensor.other.gnss":
                return self._create_gnss_publisher(sensor_config)
            else:
                self.logger.warning(f"Unknown sensor type: {sensor_type}")
                return False

        except Exception as e:
            self.logger.error(f"Failed to create publisher for {sensor_config.sensor_id}: {e}")
            return False

    def _create_camera_publishers(self, sensor_config: SensorConfig) -> bool:
        """
        Create camera image and info publishers.

        Args:
        ----
            sensor_config: Sensor configuration

        Returns
        -------
            True if created successfully

        """
        qos = self.get_qos_profile(sensor_config.qos_profile)

        # Image publisher
        if sensor_config.topic_image:
            publisher = self.ros_node.create_publisher(Image, sensor_config.topic_image, qos)
            self.camera_publishers[sensor_config.sensor_id] = publisher
            sensor_config.publisher = publisher

            self.logger.info(f"Created camera publisher: {sensor_config.topic_image}")

        # Camera info publisher
        if sensor_config.topic_info:
            info_publisher = self.ros_node.create_publisher(
                CameraInfo, sensor_config.topic_info, qos
            )
            self.camera_info_publishers[sensor_config.sensor_id] = info_publisher
            sensor_config.publisher_info = info_publisher

            self.logger.info(f"Created camera info publisher: {sensor_config.topic_info}")

        return True

    def _create_single_topic_publisher(
        self,
        sensor_config: SensorConfig,
        msg_type: type,
        publisher_dict: Dict,
        sensor_type_name: str,
    ) -> bool:
        """
        Create a single-topic publisher (LiDAR, IMU, or GNSS).

        Args:
        ----
            sensor_config: Sensor configuration
            msg_type: ROS message type class
            publisher_dict: Dictionary to store publisher
            sensor_type_name: Human-readable sensor type name for logging

        Returns
        -------
            True if created successfully

        """
        if not sensor_config.topic:
            self.logger.error(
                f"No topic specified for {sensor_type_name} {sensor_config.sensor_id}"
            )
            return False

        qos = self.get_qos_profile(sensor_config.qos_profile)
        publisher = self.ros_node.create_publisher(msg_type, sensor_config.topic, qos)
        publisher_dict[sensor_config.sensor_id] = publisher
        sensor_config.publisher = publisher

        self.logger.info(f"Created {sensor_type_name} publisher: {sensor_config.topic}")
        return True

    def _create_lidar_publisher(self, sensor_config: SensorConfig) -> bool:
        """Create LiDAR pointcloud publisher."""
        return self._create_single_topic_publisher(
            sensor_config, PointCloud2, self.lidar_publishers, "LiDAR"
        )

    def _create_imu_publisher(self, sensor_config: SensorConfig) -> bool:
        """Create IMU data publisher."""
        return self._create_single_topic_publisher(sensor_config, Imu, self.imu_publishers, "IMU")

    def _create_gnss_publisher(self, sensor_config: SensorConfig) -> bool:
        """Create GNSS pose publisher."""
        return self._create_single_topic_publisher(
            sensor_config, PoseWithCovarianceStamped, self.gnss_publishers, "GNSS"
        )

    def publish_camera_data(self, sensor_id: str, image_msg: Image, camera_info_msg: CameraInfo):
        """
        Publish camera image and info.

        Args:
        ----
            sensor_id: Sensor identifier
            image_msg: Image message
            camera_info_msg: Camera info message

        """
        if sensor_id in self.camera_publishers:
            self.camera_publishers[sensor_id].publish(image_msg)

        if sensor_id in self.camera_info_publishers:
            self.camera_info_publishers[sensor_id].publish(camera_info_msg)

    def _publish_sensor_data(self, sensor_id: str, msg: Any, publishers_dict: Dict[str, Any]):
        """
        Publish sensor data to the appropriate publisher.

        Args:
        ----
            sensor_id: Sensor identifier
            msg: Message to publish
            publishers_dict: Dictionary containing publishers

        """
        if sensor_id in publishers_dict:
            publishers_dict[sensor_id].publish(msg)

    def publish_lidar_data(self, sensor_id: str, pointcloud_msg: PointCloud2):
        """
        Publish LiDAR pointcloud.

        Args:
        ----
            sensor_id: Sensor identifier
            pointcloud_msg: PointCloud2 message

        """
        self._publish_sensor_data(sensor_id, pointcloud_msg, self.lidar_publishers)

    def publish_imu_data(self, sensor_id: str, imu_msg: Imu):
        """
        Publish IMU data.

        Args:
        ----
            sensor_id: Sensor identifier
            imu_msg: IMU message

        """
        self._publish_sensor_data(sensor_id, imu_msg, self.imu_publishers)

    def publish_gnss_data(self, sensor_id: str, pose_msg: PoseWithCovarianceStamped):
        """
        Publish GNSS pose.

        Args:
        ----
            sensor_id: Sensor identifier
            pose_msg: Pose with covariance message

        """
        self._publish_sensor_data(sensor_id, pose_msg, self.gnss_publishers)

    def get_publisher(self, sensor_id: str, publisher_type: str = "main") -> Optional[Any]:
        """
        Get publisher by sensor ID.

        Args:
        ----
            sensor_id: Sensor identifier
            publisher_type: Type of publisher ('main' or 'info')

        Returns
        -------
            Publisher instance or None

        """
        if publisher_type == "info":
            return self.camera_info_publishers.get(sensor_id)

        # Check all publisher dictionaries
        for publishers in [
            self.camera_publishers,
            self.lidar_publishers,
            self.imu_publishers,
            self.gnss_publishers,
        ]:
            if sensor_id in publishers:
                return publishers[sensor_id]

        return None

    def destroy_all_publishers(self):
        """Destroy all created publishers."""
        all_publishers = [
            self.camera_publishers,
            self.camera_info_publishers,
            self.lidar_publishers,
            self.imu_publishers,
            self.gnss_publishers,
        ]

        for publishers_dict in all_publishers:
            for publisher in publishers_dict.values():
                self.ros_node.destroy_publisher(publisher)
            publishers_dict.clear()

        self.logger.info("Destroyed all publishers")
