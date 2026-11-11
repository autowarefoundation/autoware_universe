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

Messages are rclpy message objects; they are CDR-serialized and handed to the C++ core. No ROS
node, executor or clock is involved, so results depend only on the inputs.
``undistort_pointcloud`` returns a new point cloud and a status object and leaves its input
unchanged.
"""

from typing import NamedTuple
from typing import Optional
from typing import Tuple

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2

from . import _distortion_corrector_pybind as _ext

PointcloudValidity = _ext.PointcloudValidity

__all__ = ["DistortionCorrector", "PointcloudValidity", "UndistortionStatus"]


def _serialize(msg, msg_type):
    if not isinstance(msg, msg_type):
        raise TypeError(f"expected {msg_type.__name__}, got {type(msg).__name__}")
    return serialize_message(msg)


class UndistortionStatus(NamedTuple):
    """Outcome of :meth:`DistortionCorrector.undistort_pointcloud`."""

    validity: "PointcloudValidity"
    twist_queue_empty: bool
    twist_timestamp_too_late: bool
    imu_timestamp_too_late: bool
    timestamp_mismatch_count: int
    timestamp_mismatch_fraction: float
    # Why the azimuth conversion failed on this call; None if it was not attempted or succeeded.
    angle_conversion_error: Optional[str]


class DistortionCorrector:
    """Corrects motion distortion of a ``PointCloud2`` using twist and (optionally) IMU data.

    Feed twist and IMU messages in timestamp order. A sample older than the queue front clears the
    queue; samples older than the newest one by more than 1 s are dropped. Transforms have
    ``header.frame_id`` = base frame and ``child_frame_id`` = sensor frame, with unit-quaternion
    rotations.
    """

    def __init__(self, use_3d_distortion_correction: bool = False) -> None:
        self._impl = _ext.DistortionCorrector(use_3d_distortion_correction)

    def insert_twist(self, twist: TwistWithCovarianceStamped) -> None:
        self._impl.insert_twist(_serialize(twist, TwistWithCovarianceStamped))

    def set_imu_transform(self, imu_to_base_link: TransformStamped) -> None:
        """Set the IMU extrinsics. Must be called before :meth:`insert_imu`."""
        self._impl.set_imu_transform(_serialize(imu_to_base_link, TransformStamped))

    def insert_imu(self, imu: Imu) -> None:
        self._impl.insert_imu(_serialize(imu, Imu))

    def undistort_pointcloud(
        self,
        pointcloud: PointCloud2,
        lidar_to_base_link: TransformStamped,
        use_imu: bool = True,
        update_azimuth_and_distance: bool = False,
    ) -> Tuple[PointCloud2, UndistortionStatus]:
        """Undistort ``pointcloud`` and return ``(undistorted_pointcloud, status)``.

        ``update_azimuth_and_distance`` rewrites the azimuth and distance fields from the corrected
        positions; the Cartesian-to-azimuth conversion is computed on the first cloud and reused.
        """
        out_bytes, result, angle_conversion_error = self._impl.undistort_pointcloud(
            _serialize(pointcloud, PointCloud2),
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
