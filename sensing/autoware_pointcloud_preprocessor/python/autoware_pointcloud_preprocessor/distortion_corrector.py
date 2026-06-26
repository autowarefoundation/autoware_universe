# Copyright 2024 TIER IV, Inc.
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

"""Deterministic, offline Python interface to the distortion corrector core.

ROS messages are passed as their rclpy message objects; under the hood they are CDR-serialized and
handed to the C++ core (no wall clock is used, so results are reproducible). ``undistort_pointcloud``
returns a *new* point cloud and a status object rather than mutating its input.
"""

from typing import NamedTuple
from typing import Tuple

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2

from . import _distortion_corrector_pybind as _ext

# Re-export the bound enum so callers don't need to import the extension module directly.
PointcloudValidity = _ext.PointcloudValidity

__all__ = ["DistortionCorrector", "PointcloudValidity", "UndistortionStatus"]


class UndistortionStatus(NamedTuple):
    """Outcome of :meth:`DistortionCorrector.undistort_pointcloud`."""

    validity: "PointcloudValidity"
    twist_queue_empty: bool
    twist_timestamp_too_late: bool
    imu_timestamp_too_late: bool
    timestamp_mismatch_count: int
    timestamp_mismatch_fraction: float


class DistortionCorrector:
    """Corrects motion distortion of a ``PointCloud2`` using twist and (optionally) IMU data.

    Feed twist/IMU messages in timestamp order; the internal queues keep ~1 s of history keyed by
    the message stamps (no wall clock). Provide the extrinsics once via the ``set_*_transform``
    methods (``header.frame_id`` = base frame, ``child_frame_id`` = sensor/IMU frame; rotations must
    be unit quaternions, as TF provides).
    """

    def __init__(self, use_3d_distortion_correction: bool = False) -> None:
        self._impl = _ext.DistortionCorrector(use_3d_distortion_correction)

    def set_pointcloud_transform(self, lidar_to_base_link: TransformStamped) -> None:
        self._impl.set_pointcloud_transform(serialize_message(lidar_to_base_link))

    def set_imu_transform(self, imu_to_base_link: TransformStamped) -> None:
        self._impl.set_imu_transform(serialize_message(imu_to_base_link))

    def process_twist_message(self, twist: TwistWithCovarianceStamped) -> None:
        self._impl.process_twist_message(serialize_message(twist))

    def process_imu_message(self, imu: Imu) -> None:
        self._impl.process_imu_message(serialize_message(imu))

    def undistort_pointcloud(
        self,
        pointcloud: PointCloud2,
        use_imu: bool = True,
        update_azimuth_and_distance: bool = False,
    ) -> Tuple[PointCloud2, UndistortionStatus]:
        """Undistort ``pointcloud`` and return ``(undistorted_pointcloud, status)``.

        The input is not modified. ``update_azimuth_and_distance`` recomputes the azimuth/distance
        fields for sensor-frame clouds (the Cartesian-to-azimuth conversion is computed once and
        cached).
        """
        out_bytes, result = self._impl.undistort_pointcloud(
            serialize_message(pointcloud), use_imu, update_azimuth_and_distance
        )
        status = UndistortionStatus(
            validity=result.validity,
            twist_queue_empty=result.twist_queue_empty,
            twist_timestamp_too_late=result.twist_timestamp_too_late,
            imu_timestamp_too_late=result.imu_timestamp_too_late,
            timestamp_mismatch_count=self._impl.timestamp_mismatch_count,
            timestamp_mismatch_fraction=self._impl.timestamp_mismatch_fraction,
        )
        return deserialize_message(out_bytes, PointCloud2), status
