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
    def carla_base_link_to_vehicle_center_location(
        x: float, y: float, z: float, wheelbase: float = 2.850
    ) -> carla.Location:
        """
        Convert CARLA base_link coordinates to CARLA vehicle center location.

        For carla_sensor_kit which already uses CARLA coordinate conventions (Y-right),
        we only need to apply the wheelbase offset, NOT the coordinate system flip.

        Args
        ----
            x, y, z: Position in base_link frame (CARLA coordinates)
            wheelbase: Vehicle wheelbase in meters (default 2.850)

        Returns
        -------
            CARLA Location object

        """
        # Only apply wheelbase offset (base_link is at rear axle, vehicle center
        # is at geometric center)
        x_vehicle_center = x - (wheelbase / 2.0)
        return carla.Location(x=x_vehicle_center, y=y, z=z)

    @staticmethod
    def _convert_rotation_to_carla(
        angles: tuple[float, float, float], in_degrees: bool, negate_pitch_yaw: bool
    ) -> carla.Rotation:
        """
        Convert rotation angles to CARLA Rotation object.

        Args
        ----
            angles: Tuple of (roll, pitch, yaw) in radians (or degrees if in_degrees=True)
            in_degrees: If True, input angles are already in degrees
            negate_pitch_yaw: If True, negate pitch and yaw for coordinate system conversion

        Returns
        -------
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
