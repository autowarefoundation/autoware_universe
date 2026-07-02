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

import struct

from autoware_pointcloud_preprocessor.distortion_corrector import DistortionCorrector
from autoware_pointcloud_preprocessor.distortion_corrector import PointcloudValidity
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

# PointXYZIRCAEDT layout (matches the C++ unit test): 10 fields, point_step = 32, no padding.
_POINT_STEP = 32
_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
    PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
    PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
    PointField(name="azimuth", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="elevation", offset=20, datatype=PointField.FLOAT32, count=1),
    PointField(name="distance", offset=24, datatype=PointField.FLOAT32, count=1),
    PointField(name="time_stamp", offset=28, datatype=PointField.UINT32, count=1),
]

_CLOUD_SEC = 10
_CLOUD_NANOSEC = 100_000_000
_POINT_INTERVAL_NS = 10_000_000  # 10 ms


def _make_pointcloud(points):
    msg = PointCloud2()
    msg.header.stamp.sec = _CLOUD_SEC
    msg.header.stamp.nanosec = _CLOUD_NANOSEC
    msg.header.frame_id = "base_link"
    msg.height = 1
    msg.width = len(points)
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * len(points)
    msg.is_dense = True

    data = bytearray()
    for i, (x, y, z) in enumerate(points):
        distance = (x * x + y * y + z * z) ** 0.5
        time_stamp = i * _POINT_INTERVAL_NS
        data += struct.pack("<fffBBHfffI", x, y, z, 0, 0, 0, 0.0, 0.0, distance, time_stamp)
    msg.data = bytes(data)
    return msg


def _read_xyz(msg):
    return [struct.unpack_from("<fff", msg.data, i * msg.point_step) for i in range(msg.width)]


def _identity_transform(child_frame_id):
    tf = TransformStamped()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = child_frame_id
    tf.transform.rotation.w = 1.0
    return tf


def _make_twist(sec, nanosec, linear_x=10.0, angular_z=0.02):
    twist = TwistWithCovarianceStamped()
    twist.header.stamp.sec = sec
    twist.header.stamp.nanosec = nanosec
    twist.header.frame_id = "base_link"
    twist.twist.twist.linear.x = linear_x
    twist.twist.twist.angular.z = angular_z
    return twist


_DEFAULT_POINTS = [
    (10.0, 0.0, 1.0),
    (5.0, -5.0, 2.0),
    (0.0, -10.0, 3.0),
    (-5.0, -5.0, 4.0),
    (-10.0, 0.0, 5.0),
]


def _feed_twists(corrector):
    # Twist stamps spanning the point times (10.10 .. 10.14 s).
    for ms in (95, 110, 125, 140, 155):
        corrector.process_twist_message(_make_twist(_CLOUD_SEC, 90_000_000 + ms * 1_000_000))


def test_empty_pointcloud_reports_empty():
    corrector = DistortionCorrector()
    empty = _make_pointcloud([])
    _, status = corrector.undistort_pointcloud(empty, use_imu=False)
    assert status.validity == PointcloudValidity.EMPTY


def test_empty_twist_queue_leaves_cloud_unchanged():
    corrector = DistortionCorrector()
    corrector.set_pointcloud_transform(_identity_transform("base_link"))
    cloud = _make_pointcloud(_DEFAULT_POINTS)
    out, status = corrector.undistort_pointcloud(cloud, use_imu=False)

    assert status.validity == PointcloudValidity.VALID
    assert status.twist_queue_empty is True
    assert _read_xyz(out) == _read_xyz(cloud)


def test_undistortion_changes_points_2d():
    corrector = DistortionCorrector(use_3d_distortion_correction=False)
    corrector.set_pointcloud_transform(_identity_transform("base_link"))
    _feed_twists(corrector)

    cloud = _make_pointcloud(_DEFAULT_POINTS)
    out, status = corrector.undistort_pointcloud(cloud, use_imu=False)

    assert status.validity == PointcloudValidity.VALID
    assert status.twist_queue_empty is False
    # Later points accumulate motion correction, so at least one point must move.
    assert _read_xyz(out) != _read_xyz(cloud)


def test_undistortion_runs_3d():
    corrector = DistortionCorrector(use_3d_distortion_correction=True)
    corrector.set_pointcloud_transform(_identity_transform("base_link"))
    _feed_twists(corrector)

    cloud = _make_pointcloud(_DEFAULT_POINTS)
    out, status = corrector.undistort_pointcloud(cloud, use_imu=False)

    assert status.validity == PointcloudValidity.VALID
    assert _read_xyz(out) != _read_xyz(cloud)
