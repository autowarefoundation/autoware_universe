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

from __future__ import annotations

import math
from typing import Dict

import numpy as np

try:
    import carla
except ImportError:
    # CARLA not available - likely running in test environment
    # This module won't be functional but imports will succeed
    carla = None  # type: ignore


class CoordinateTransformer:
    """
    Transformer for converting between ROS and CARLA coordinate systems.

    ROS uses right-handed coordinate system: X-forward, Y-left, Z-up
    CARLA (Unreal Engine) uses left-handed: X-forward, Y-right, Z-up
    """

    @staticmethod
    def base_link_to_vehicle_center(
        x: float, y: float, z: float, wheelbase: float = 2.850
    ) -> tuple:
        """
        Convert base_link coordinates to vehicle center coordinates.

        base_link is at rear axle center, vehicle center is at geometric center.

        Args:
            x, y, z: Position in base_link frame
            wheelbase: Vehicle wheelbase in meters (default 2.850)

        Returns:
            (x, y, z) in vehicle center frame
        """
        x_vehicle_center = x - (wheelbase / 2.0)
        return (x_vehicle_center, y, z)

    @staticmethod
    def carla_base_link_to_vehicle_center_location(
        x: float, y: float, z: float, wheelbase: float = 2.850
    ) -> carla.Location:
        """
        Convert CARLA base_link coordinates to CARLA vehicle center location.

        For carla_sensor_kit which already uses CARLA coordinate conventions (Y-right),
        we only need to apply the wheelbase offset, NOT the coordinate system flip.

        Args:
            x, y, z: Position in base_link frame (CARLA coordinates)
            wheelbase: Vehicle wheelbase in meters (default 2.850)

        Returns:
            CARLA Location object
        """
        # Only apply wheelbase offset (base_link is at rear axle, vehicle center is at geometric center)
        x_vehicle_center = x - (wheelbase / 2.0)
        return carla.Location(x=x_vehicle_center, y=y, z=z)

    @staticmethod
    def ros_base_link_to_carla_location(
        x: float, y: float, z: float, wheelbase: float = 2.850
    ) -> carla.Location:
        """
        Convert ROS base_link coordinates directly to CARLA vehicle location.

        Combines frame origin conversion + coordinate system conversion.

        Args:
            x, y, z: Position in base_link frame (ROS coordinates)
            wheelbase: Vehicle wheelbase in meters (default 2.850)

        Returns:
            CARLA Location object
        """
        # Step 1: base_link to vehicle center
        x_vc, y_vc, z_vc = CoordinateTransformer.base_link_to_vehicle_center(x, y, z, wheelbase)

        # Step 2: ROS to CARLA coordinate system
        return CoordinateTransformer.ros_to_carla_location(x_vc, y_vc, z_vc)

    @staticmethod
    def ros_to_carla_location(x: float, y: float, z: float) -> carla.Location:
        """
        Convert ROS position to CARLA location.

        Args:
            x: X position in ROS (forward)
            y: Y position in ROS (left)
            z: Z position in ROS (up)

        Returns:
            CARLA Location object
        """
        # In CARLA: X-forward, Y-right (opposite of ROS), Z-up
        return carla.Location(x=x, y=-y, z=z)

    @staticmethod
    def _convert_rotation_to_carla(
        angles: tuple[float, float, float], in_degrees: bool, negate_pitch_yaw: bool
    ) -> carla.Rotation:
        """Convert rotation angles to CARLA Rotation object.

        Args:
            angles: Tuple of (roll, pitch, yaw) in radians (or degrees if in_degrees=True)
            in_degrees: If True, input angles are already in degrees
            negate_pitch_yaw: If True, negate pitch and yaw for coordinate system conversion

        Returns:
            CARLA Rotation object (in degrees)
        """
        roll, pitch, yaw = angles
        roll_deg = roll if in_degrees else math.degrees(roll)
        pitch_deg = pitch if in_degrees else math.degrees(pitch)
        yaw_deg = yaw if in_degrees else math.degrees(yaw)

        if negate_pitch_yaw:
            pitch_deg = -pitch_deg
            yaw_deg = -yaw_deg

        return carla.Rotation(roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg)

    @staticmethod
    def carla_rotation_to_carla_rotation(
        roll: float, pitch: float, yaw: float, in_degrees: bool = False
    ) -> carla.Rotation:
        """Convert CARLA rotation angles to CARLA Rotation (no coordinate flip)."""
        return CoordinateTransformer._convert_rotation_to_carla(
            (roll, pitch, yaw), in_degrees, negate_pitch_yaw=False
        )

    @staticmethod
    def ros_to_carla_rotation(
        roll: float, pitch: float, yaw: float, in_degrees: bool = False
    ) -> carla.Rotation:
        """Convert ROS rotation to CARLA rotation (with coordinate flip)."""
        return CoordinateTransformer._convert_rotation_to_carla(
            (roll, pitch, yaw), in_degrees, negate_pitch_yaw=True
        )

    @staticmethod
    def ros_transform_to_carla_transform(transform_dict: Dict[str, float]) -> carla.Transform:
        """
        Convert ROS transform dictionary to CARLA Transform.

        Args:
            transform_dict: Dictionary with x, y, z, roll, pitch, yaw

        Returns:
            CARLA Transform object
        """
        x = transform_dict.get("x", 0.0)
        y = transform_dict.get("y", 0.0)
        z = transform_dict.get("z", 0.0)
        roll = transform_dict.get("roll", 0.0)
        pitch = transform_dict.get("pitch", 0.0)
        yaw = transform_dict.get("yaw", 0.0)

        location = CoordinateTransformer.ros_to_carla_location(x, y, z)
        rotation = CoordinateTransformer.ros_to_carla_rotation(roll, pitch, yaw)

        return carla.Transform(location, rotation)

    @staticmethod
    def carla_to_ros_location(location: carla.Location) -> Dict[str, float]:
        """
        Convert CARLA location to ROS position.

        Args:
            location: CARLA Location object

        Returns:
            Dictionary with x, y, z in ROS coordinates
        """
        return {
            "x": location.x,
            "y": -location.y,  # Negate Y for coordinate system change
            "z": location.z,
        }

    @staticmethod
    def carla_to_ros_rotation(rotation: carla.Rotation) -> Dict[str, float]:
        """
        Convert CARLA rotation (degrees) to ROS rotation (radians).

        Args:
            rotation: CARLA Rotation object (in degrees)

        Returns:
            Dictionary with roll, pitch, yaw in radians
        """
        return {
            "roll": math.radians(rotation.roll),
            "pitch": math.radians(-rotation.pitch),  # Negate pitch
            "yaw": math.radians(-rotation.yaw),  # Negate yaw
        }

    @staticmethod
    def carla_transform_to_ros_transform(transform: carla.Transform) -> Dict[str, float]:
        """
        Convert CARLA Transform to ROS transform dictionary.

        Args:
            transform: CARLA Transform object

        Returns:
            Dictionary with x, y, z, roll, pitch, yaw in ROS coordinates
        """
        location = CoordinateTransformer.carla_to_ros_location(transform.location)
        rotation = CoordinateTransformer.carla_to_ros_rotation(transform.rotation)

        return {
            "x": location["x"],
            "y": location["y"],
            "z": location["z"],
            "roll": rotation["roll"],
            "pitch": rotation["pitch"],
            "yaw": rotation["yaw"],
        }

    @staticmethod
    def apply_transform_offset(
        base_transform: carla.Transform, offset_dict: Dict[str, float]
    ) -> carla.Transform:
        """
        Apply an offset transform to a base transform.

        Args:
            base_transform: Base CARLA Transform
            offset_dict: Offset in ROS coordinates (x, y, z, roll, pitch, yaw)

        Returns:
            Combined CARLA Transform
        """
        # Convert offset to CARLA transform
        offset_transform = CoordinateTransformer.ros_transform_to_carla_transform(offset_dict)

        # Combine transforms using matrix multiplication
        base_matrix = CoordinateTransformer._transform_to_matrix(base_transform)
        offset_matrix = CoordinateTransformer._transform_to_matrix(offset_transform)

        combined_matrix = np.dot(base_matrix, offset_matrix)

        return CoordinateTransformer._matrix_to_transform(combined_matrix)

    @staticmethod
    def _transform_to_matrix(transform: carla.Transform) -> np.ndarray:
        """Convert CARLA Transform to 4x4 transformation matrix."""
        # Get rotation in radians
        roll = math.radians(transform.rotation.roll)
        pitch = math.radians(transform.rotation.pitch)
        yaw = math.radians(transform.rotation.yaw)

        # Create rotation matrix
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        matrix = np.eye(4)

        # Rotation matrix for ZYX Euler angles (CARLA order)
        matrix[0, 0] = cp * cy
        matrix[0, 1] = cp * sy
        matrix[0, 2] = sp
        matrix[1, 0] = sr * sp * cy - cr * sy
        matrix[1, 1] = sr * sp * sy + cr * cy
        matrix[1, 2] = -sr * cp
        matrix[2, 0] = -(cr * sp * cy + sr * sy)
        matrix[2, 1] = cr * cy - cr * sp * sy
        matrix[2, 2] = cr * cp

        # Translation
        matrix[0, 3] = transform.location.x
        matrix[1, 3] = transform.location.y
        matrix[2, 3] = transform.location.z

        return matrix

    @staticmethod
    def _matrix_to_transform(matrix: np.ndarray) -> carla.Transform:
        """Convert 4x4 transformation matrix to CARLA Transform."""
        # Extract translation
        x = matrix[0, 3]
        y = matrix[1, 3]
        z = matrix[2, 3]

        # Extract rotation (ZYX Euler angles)
        sp = matrix[0, 2]

        if abs(sp) < 0.99999:
            pitch = math.asin(sp)
            yaw = math.atan2(matrix[0, 1], matrix[0, 0])
            roll = math.atan2(-matrix[1, 2], matrix[2, 2])
        else:
            # Gimbal lock case
            pitch = math.copysign(math.pi / 2, sp)
            if sp > 0:
                yaw = math.atan2(matrix[1, 0], matrix[1, 1])
                roll = 0.0
            else:
                yaw = math.atan2(-matrix[1, 0], matrix[1, 1])
                roll = 0.0

        # Convert to degrees for CARLA
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        return carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg),
        )
