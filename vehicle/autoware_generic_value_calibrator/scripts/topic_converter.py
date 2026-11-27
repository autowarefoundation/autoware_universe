#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 TIER IV, Inc. All rights reserved.
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

"""
Topic converter for Generic Value Calibrator.

This script converts various ROS message types to tier4_debug_msgs/Float64Stamped
for use with the generic value calibrator.

Examples:
  - Extract acceleration from autoware_control_msgs/Control
  - Extract velocity from nav_msgs/Odometry
  - Extract throttle from autoware_vehicle_msgs/VehicleCommand
  - Custom field extraction from any message type
"""

import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float64Stamped

# Import common message types
try:
    from autoware_control_msgs.msg import Control

    CONTROL_AVAILABLE = True
except ImportError:
    CONTROL_AVAILABLE = False

try:
    from nav_msgs.msg import Odometry

    ODOMETRY_AVAILABLE = True
except ImportError:
    ODOMETRY_AVAILABLE = False

try:
    from geometry_msgs.msg import AccelStamped
    from geometry_msgs.msg import TwistStamped

    GEOMETRY_AVAILABLE = True
except ImportError:
    GEOMETRY_AVAILABLE = False


class TopicConverter(Node):
    """
    Convert various message types to Float64Stamped.

    Parameters:
      - input_topic: Input topic name
      - output_topic: Output Float64Stamped topic name
      - conversion_type: Type of conversion (see CONVERSION_MAP)
      - custom_field: Custom field path (e.g., "data.x", "twist.linear.x")
    """

    # Map of conversion types to (message_type, extraction_function)
    CONVERSION_MAP = {}

    def __init__(self):
        super().__init__("topic_converter")

        # Declare parameters
        self.declare_parameter("input_topic", "/control/command/control_cmd")
        self.declare_parameter("output_topic", "/generic/input/value")
        self.declare_parameter("conversion_type", "control_acceleration")
        self.declare_parameter("custom_field", "")
        self.declare_parameter("scale_factor", 1.0)
        self.declare_parameter("offset", 0.0)

        # Get parameters
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.conversion_type = self.get_parameter("conversion_type").value
        self.custom_field = self.get_parameter("custom_field").value
        self.scale_factor = self.get_parameter("scale_factor").value
        self.offset = self.get_parameter("offset").value

        # Setup conversion functions
        self._setup_conversions()

        # Create publisher
        self.pub = self.create_publisher(Float64Stamped, self.output_topic, 10)

        # Create subscriber based on conversion type
        if self.conversion_type not in self.CONVERSION_MAP:
            self.get_logger().error(
                f"Unknown conversion type: {self.conversion_type}. "
                f"Available types: {list(self.CONVERSION_MAP.keys())}"
            )
            raise ValueError(f"Unknown conversion type: {self.conversion_type}")

        msg_type, _ = self.CONVERSION_MAP[self.conversion_type]
        self.sub = self.create_subscription(msg_type, self.input_topic, self.callback, 10)

        self.get_logger().info("Topic Converter Started:")
        self.get_logger().info(f"  Input: {self.input_topic} ({msg_type.__name__})")
        self.get_logger().info(f"  Output: {self.output_topic} (Float64Stamped)")
        self.get_logger().info(f"  Conversion: {self.conversion_type}")
        self.get_logger().info(f"  Scale: {self.scale_factor}, Offset: {self.offset}")

    def _setup_conversions(self):
        """Set up available conversion functions."""
        if CONTROL_AVAILABLE:
            self.CONVERSION_MAP["control_acceleration"] = (
                Control,
                self._extract_control_acceleration,
            )
            self.CONVERSION_MAP["control_velocity"] = (Control, self._extract_control_velocity)
            self.CONVERSION_MAP["control_steering_angle"] = (
                Control,
                self._extract_control_steering_angle,
            )
            self.CONVERSION_MAP["control_steering_rate"] = (
                Control,
                self._extract_control_steering_rate,
            )

        if ODOMETRY_AVAILABLE:
            self.CONVERSION_MAP["odom_linear_x"] = (Odometry, self._extract_odom_linear_x)
            self.CONVERSION_MAP["odom_linear_y"] = (Odometry, self._extract_odom_linear_y)
            self.CONVERSION_MAP["odom_angular_z"] = (Odometry, self._extract_odom_angular_z)

        if GEOMETRY_AVAILABLE:
            self.CONVERSION_MAP["twist_linear_x"] = (TwistStamped, self._extract_twist_linear_x)
            self.CONVERSION_MAP["twist_angular_z"] = (TwistStamped, self._extract_twist_angular_z)
            self.CONVERSION_MAP["accel_linear_x"] = (AccelStamped, self._extract_accel_linear_x)

        # Custom field extraction
        if self.custom_field:
            self.get_logger().warn(
                "Custom field extraction is experimental. "
                "Message type must be set correctly in conversion_type parameter."
            )

    # Extraction functions for Control message
    def _extract_control_acceleration(self, msg):
        """Extract longitudinal acceleration from Control message."""
        return msg.stamp, msg.longitudinal.acceleration

    def _extract_control_velocity(self, msg):
        """Extract longitudinal velocity from Control message."""
        return msg.stamp, msg.longitudinal.velocity

    def _extract_control_steering_angle(self, msg):
        """Extract steering angle from Control message."""
        return msg.stamp, msg.lateral.steering_tire_angle

    def _extract_control_steering_rate(self, msg):
        """Extract steering rate from Control message."""
        return msg.stamp, msg.lateral.steering_tire_rotation_rate

    # Extraction functions for Odometry message
    def _extract_odom_linear_x(self, msg):
        """Extract linear velocity X from Odometry."""
        return msg.header.stamp, msg.twist.twist.linear.x

    def _extract_odom_linear_y(self, msg):
        """Extract linear velocity Y from Odometry."""
        return msg.header.stamp, msg.twist.twist.linear.y

    def _extract_odom_angular_z(self, msg):
        """Extract angular velocity Z from Odometry."""
        return msg.header.stamp, msg.twist.twist.angular.z

    # Extraction functions for TwistStamped message
    def _extract_twist_linear_x(self, msg):
        """Extract linear velocity X from TwistStamped."""
        return msg.header.stamp, msg.twist.linear.x

    def _extract_twist_angular_z(self, msg):
        """Extract angular velocity Z from TwistStamped."""
        return msg.header.stamp, msg.twist.angular.z

    # Extraction functions for AccelStamped message
    def _extract_accel_linear_x(self, msg):
        """Extract linear acceleration X from AccelStamped."""
        return msg.header.stamp, msg.accel.linear.x

    def _extract_custom_field(self, msg):
        """Extract custom field from message using dot notation."""
        field_path = self.custom_field.split(".")
        value = msg
        for field in field_path:
            if hasattr(value, field):
                value = getattr(value, field)
            else:
                self.get_logger().error(f"Field not found: {field} in {type(value)}")
                return None, 0.0

        # Try to extract timestamp
        if hasattr(msg, "stamp"):
            stamp = msg.stamp
        elif hasattr(msg, "header"):
            stamp = msg.header.stamp
        else:
            stamp = self.get_clock().now().to_msg()

        return stamp, float(value)

    def callback(self, msg):
        """Convert and publish message."""
        _, extract_fn = self.CONVERSION_MAP[self.conversion_type]

        try:
            # Extract timestamp and value
            if self.custom_field:
                stamp, value = self._extract_custom_field(msg)
            else:
                stamp, value = extract_fn(msg)

            if stamp is None:
                return

            # Apply scale and offset
            converted_value = value * self.scale_factor + self.offset

            # Create and publish Float64Stamped message
            output_msg = Float64Stamped()
            output_msg.stamp = stamp
            output_msg.data = converted_value

            self.pub.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TopicConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
