# Copyright 2026 TIER IV, Inc.
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

"""Python interface to the distortion corrector core, for offline use.

Twists, IMU samples and point clouds are passed serialized, as stored in rosbags, or as rclpy
message objects, which are serialized first. The C++ core deserializes them. Transforms are passed
as rclpy message objects. No ROS node, executor or clock is involved, so results depend only on the
inputs.
``undistort_pointcloud`` returns a new point cloud and a status object and leaves its input
unchanged.
"""

from typing import NamedTuple
from typing import Optional
from typing import Tuple
from typing import Union

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2

from ._serialization import serialized
from . import _distortion_corrector_pybind as _ext

PointcloudValidity = _ext.PointcloudValidity

__all__ = ["DistortionCorrector", "PointcloudValidity", "UndistortionStatus"]


def _serialize(msg, msg_type):
    if not isinstance(msg, msg_type):
        raise TypeError(f"expected {msg_type.__name__}, got {type(msg).__name__}")
    return serialize_message(msg)


class UndistortionStatus(NamedTuple):
    """Outcome of :meth:`DistortionCorrector.undistort_pointcloud`.

    Attributes:
        validity: Whether the cloud has the fields and layout the corrector needs. For any value
            other than ``VALID``, the cloud is returned unchanged.
        twist_queue_empty: No twist sample was queued, so the cloud is returned unchanged.
        twist_timestamp_too_late: Some point has no twist sample within 0.1 s of its time.
        imu_timestamp_too_late: Some point has no IMU sample within 0.1 s of its time.
        timestamp_mismatch_count: Number of points without a matching twist sample, or without a
            matching IMU sample when IMU is used.
        timestamp_mismatch_fraction: ``timestamp_mismatch_count`` divided by the number of points.
        angle_conversion_error: Why the azimuth conversion could not be computed on this call, or
            ``None`` if it was not attempted or succeeded.
    """

    validity: "PointcloudValidity"
    twist_queue_empty: bool
    twist_timestamp_too_late: bool
    imu_timestamp_too_late: bool
    timestamp_mismatch_count: int
    timestamp_mismatch_fraction: float
    angle_conversion_error: Optional[str]


class DistortionCorrector:
    """Corrects motion distortion of a ``PointCloud2`` using twist and, optionally, IMU data.

    Transforms have ``header.frame_id`` = base frame and ``child_frame_id`` = sensor frame, with
    unit-quaternion rotations.
    """

    def __init__(self, use_3d_distortion_correction: bool = False) -> None:
        """Create a corrector with empty twist and IMU queues.

        Args:
            use_3d_distortion_correction: Correct in 3D (x, y, z, roll, pitch, yaw) instead of 2D
                (x, y, yaw).
        """
        self._impl = _ext.DistortionCorrector(use_3d_distortion_correction)

    def insert_twist(self, twist: Union[bytes, TwistWithCovarianceStamped]) -> None:
        """Queue a twist sample.

        Samples are expected in timestamp order. A sample older than the queue front clears the
        queue, and samples older than the newest one by more than 1 s are pruned.

        Args:
            twist: Vehicle twist in the base frame, as a serialized ``TwistWithCovarianceStamped``
                in the format that rosbags store (CDR), or as a message object, which is slower
                since it has to be serialized first.

        Raises:
            TypeError: If ``twist`` is neither bytes nor a ``TwistWithCovarianceStamped``.
            ValueError: If ``twist`` cannot be deserialized as a ``TwistWithCovarianceStamped``.
        """
        self._impl.insert_twist(serialized(twist, TwistWithCovarianceStamped))

    def set_imu_transform(self, imu_to_base_link: TransformStamped) -> None:
        """Set the transform that rotates IMU angular velocities into the base frame.

        Must be called before :meth:`insert_imu`.

        Args:
            imu_to_base_link: Transform with ``child_frame_id`` = IMU frame. Only its rotation is
                used.

        Raises:
            TypeError: If ``imu_to_base_link`` is not a ``TransformStamped``.
        """
        self._impl.set_imu_transform(_serialize(imu_to_base_link, TransformStamped))

    def insert_imu(self, imu: Union[bytes, Imu]) -> None:
        """Queue an IMU sample's angular velocity, rotated into the base frame.

        Ordering and pruning are the same as in :meth:`insert_twist`.

        Args:
            imu: IMU message, serialized in the format that rosbags store (CDR), or as a message
                object, which is slower since it has to be serialized first. Only its angular
                velocity is used.

        Raises:
            TypeError: If ``imu`` is neither bytes nor an ``Imu``.
            ValueError: If ``imu`` cannot be deserialized as an ``Imu``.
            RuntimeError: If :meth:`set_imu_transform` has not been called.
        """
        self._impl.insert_imu(serialized(imu, Imu))

    def undistort_pointcloud(
        self,
        pointcloud: Union[bytes, PointCloud2],
        lidar_to_base_link: TransformStamped,
        use_imu: bool = True,
        update_azimuth_and_distance: bool = False,
    ) -> Tuple[PointCloud2, UndistortionStatus]:
        """Undistort a point cloud with the queued twist and IMU samples.

        Args:
            pointcloud: Cloud with the PointXYZIRCAEDT layout, serialized in the format that
                rosbags store (CDR), or as a message object, which is slower since it has to be
                serialized first. It is not modified.
            lidar_to_base_link: Transform with ``child_frame_id`` = sensor frame. Equal frame ids
                mean the cloud is in the base frame.
            use_imu: Use the queued IMU angular velocity instead of the twist's.
            update_azimuth_and_distance: Rewrite the azimuth and distance fields from the corrected
                positions. The Cartesian-to-azimuth conversion is computed on the first cloud that
                allows it and reused for later clouds.

        Returns:
            The undistorted cloud and its :class:`UndistortionStatus`.

        Raises:
            TypeError: If an argument has the wrong message type.
            ValueError: If ``pointcloud`` cannot be deserialized as a ``PointCloud2``.
            RuntimeError: If ``update_azimuth_and_distance`` is set for a cloud in the base frame
                and an azimuth conversion is available.
        """
        out_bytes, result, angle_conversion_error = self._impl.undistort_pointcloud(
            serialized(pointcloud, PointCloud2),
            _serialize(lidar_to_base_link, TransformStamped),
            use_imu,
            update_azimuth_and_distance,
        )
        status = UndistortionStatus(
            validity=result.validity,
            twist_queue_empty=result.twist_queue_empty,
            twist_timestamp_too_late=result.twist_timestamp_too_late,
            imu_timestamp_too_late=result.imu_timestamp_too_late,
            timestamp_mismatch_count=self._impl.timestamp_mismatch_count,
            timestamp_mismatch_fraction=self._impl.timestamp_mismatch_fraction,
            angle_conversion_error=angle_conversion_error,
        )
        return deserialize_message(out_bytes, PointCloud2), status
