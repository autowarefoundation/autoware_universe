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

# Tests of the Python binding: what crosses the boundary and how. The concatenation logic itself
# is covered by the C++ tests (test_cloud_concatenator, test_concatenation_diagnostics, ...).

import gc
import math
import struct

from autoware_pointcloud_preprocessor.concatenate_pointclouds import Concatenator
from autoware_pointcloud_preprocessor.concatenate_pointclouds import FrameStatus
from autoware_pointcloud_preprocessor.concatenate_pointclouds import MotionCompensationStatus
from autoware_sensing_msgs.msg import ConcatenatedPointCloudInfo
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

_INPUT_TOPICS = ["lidar_top", "lidar_left", "lidar_right"]
_OUTPUT_FRAME = "base_link"

# x/y/z-only input (point_step 12); the core converts it to the 16-byte PointXYZIRC layout.
_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
_POINTS = [(10.0, 0.0, 1.0), (0.0, 10.0, 2.0), (0.0, 0.0, 10.0)]

# 90 deg about +Z, then (10, 20, 30): (x, y, z) -> (10 - y, 20 + x, 30 + z).
_ROTATION_Z_90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
_TRANSLATION = (10.0, 20.0, 30.0)
_UNIT_POINTS = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
_UNIT_POINTS_IN_OUTPUT_FRAME = [(10.0, 21.0, 30.0), (9.0, 20.0, 30.0), (10.0, 20.0, 31.0)]


def _make_cloud(sec, nanosec, points=_POINTS, frame_id=_OUTPUT_FRAME):
    msg = PointCloud2()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = _FIELDS
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    return msg


def _read_xyz(msg):
    return [struct.unpack_from("<fff", msg.data, i * msg.point_step) for i in range(msg.width)]


def _assert_points(got, expected):
    assert len(got) == len(expected)
    for point, want in zip(got, expected):
        assert point == pytest.approx(want, abs=1e-4)


def _make_transform(child_frame, parent_frame=_OUTPUT_FRAME):
    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (
        _TRANSLATION
    )
    rotation = msg.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = _ROTATION_Z_90
    return msg


def _make_twist(sec, nanosec, vx):
    msg = TwistWithCovarianceStamped()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.twist.twist.linear.x = vx
    return msg


def _make_odometry(sec, nanosec, vx):
    msg = Odometry()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.twist.twist.linear.x = vx
    return msg


def _make_concatenator(input_topics=_INPUT_TOPICS, **kwargs):
    kwargs.setdefault("timeout_sec", 0.2)
    kwargs.setdefault("is_motion_compensated", False)
    return Concatenator(input_topics, _OUTPUT_FRAME, **kwargs)


def _feed_one_frame(concatenator, topic_to_cloud):
    """Feed one cloud per topic, 10 ms apart, and return what the last one resolves."""
    frames = []
    for index, (topic, cloud) in enumerate(topic_to_cloud.items()):
        frames = concatenator.process_cloud(topic, cloud, arrival_time=100.0 + 0.01 * index)
    return frames


def test_concatenates_one_cloud_per_topic_into_a_frame():
    concatenator = _make_concatenator()

    frames = _feed_one_frame(concatenator, {topic: _make_cloud(10, 0) for topic in _INPUT_TOPICS})

    assert len(frames) == 1
    frame = frames[0]
    assert frame.status is FrameStatus.COMPLETE
    cloud = frame.result.concatenated_cloud
    assert isinstance(cloud, PointCloud2)
    assert cloud.header.frame_id == _OUTPUT_FRAME
    assert cloud.width == len(_INPUT_TOPICS) * len(_POINTS)
    assert [field.name for field in cloud.fields] == [
        "x",
        "y",
        "z",
        "intensity",
        "return_type",
        "channel",
    ]
    assert cloud.point_step == 16
    assert cloud.row_step == cloud.width * cloud.point_step
    assert sorted(_read_xyz(cloud)) == sorted(_POINTS * len(_INPUT_TOPICS))
    assert isinstance(frame.result.concatenation_info, ConcatenatedPointCloudInfo)
    assert frame.result.transformed_clouds is None
    assert frame.result.topic_to_original_stamp == {topic: 10.0 for topic in _INPUT_TOPICS}
    assert frame.result.dropped_sources_missing_transform == {}
    assert frame.result.motion_compensation_status is MotionCompensationStatus.VALID
    assert frame.first_arrival_time == 100.0


def test_set_transform_moves_sensor_clouds_into_the_output_frame():
    concatenator = _make_concatenator(["lidar_x"])
    concatenator.set_transform(_make_transform("lidar_x"))

    frames = concatenator.process_cloud(
        "lidar_x", _make_cloud(10, 0, _UNIT_POINTS, "lidar_x"), arrival_time=100.0
    )

    _assert_points(_read_xyz(frames[0].result.concatenated_cloud), _UNIT_POINTS_IN_OUTPUT_FRAME)


@pytest.mark.parametrize(
    "transform",
    [_make_transform("lidar_x", parent_frame="lidar_parent"), _make_transform("")],
    ids=["parent_is_not_the_output_frame", "no_child_frame"],
)
def test_set_transform_rejects_a_transform_it_cannot_use(transform):
    concatenator = _make_concatenator(["lidar_x"])

    with pytest.raises(ValueError, match="rejected transform"):
        concatenator.set_transform(transform)


def test_a_cloud_without_a_transform_is_reported_and_left_out():
    concatenator = _make_concatenator(["lidar_x"])

    frames = concatenator.process_cloud(
        "lidar_x", _make_cloud(10, 0, frame_id="lidar_x_frame"), arrival_time=100.0
    )

    result = frames[0].result
    assert result.concatenated_cloud.width == 0
    assert result.dropped_sources_missing_transform == {"lidar_x": "lidar_x_frame"}
    status = concatenator.build_diagnostics(frames[0])
    values = {kv.key: kv.value for kv in status.values}
    assert status.level == DiagnosticStatus.ERROR
    assert values["Frames without a transform to the output frame"] == "lidar_x_frame"


def test_synchronized_clouds_can_stay_in_the_sensor_frame():
    concatenator = _make_concatenator(
        ["lidar_x"],
        publish_synchronized_pointcloud=True,
        keep_input_frame_in_synchronized_pointcloud=True,
    )
    concatenator.set_transform(_make_transform("lidar_x"))

    frames = concatenator.process_cloud(
        "lidar_x", _make_cloud(10, 0, _UNIT_POINTS, "lidar_x"), arrival_time=100.0
    )

    synchronized = frames[0].result.transformed_clouds["lidar_x"]
    assert synchronized.header.frame_id == "lidar_x"
    _assert_points(_read_xyz(synchronized), _UNIT_POINTS)


@pytest.mark.parametrize("make_velocity", [_make_twist, _make_odometry], ids=["twist", "odometry"])
def test_velocity_drives_motion_compensation(make_velocity):
    concatenator = _make_concatenator(["lidar_a", "lidar_b"], is_motion_compensated=True)
    for ms in range(9900, 10250, 50):
        velocity = make_velocity(ms // 1000, (ms % 1000) * 1_000_000, vx=1.0)
        if isinstance(velocity, Odometry):
            concatenator.process_odometry(velocity)
        else:
            concatenator.process_twist(velocity)

    # The newer cloud (0.1 s later at 1 m/s) is moved 0.1 m into the older cloud's time.
    frames = _feed_one_frame(
        concatenator,
        {
            "lidar_a": _make_cloud(10, 0, [(1.0, 0.0, 0.0)]),
            "lidar_b": _make_cloud(10, 100_000_000, [(2.0, 0.0, 0.0)]),
        },
    )

    result = frames[0].result
    assert result.motion_compensation_status is MotionCompensationStatus.VALID
    _assert_points(sorted(_read_xyz(result.concatenated_cloud)), [(1.0, 0.0, 0.0), (2.1, 0.0, 0.0)])


def test_missing_velocity_is_reported():
    concatenator = _make_concatenator(["lidar_a", "lidar_b"], is_motion_compensated=True)

    frames = _feed_one_frame(
        concatenator, {"lidar_a": _make_cloud(10, 0), "lidar_b": _make_cloud(10, 100_000_000)}
    )

    assert (
        frames[0].result.motion_compensation_status is MotionCompensationStatus.NO_TWIST_AVAILABLE
    )


def test_arrival_time_drives_the_timeout():
    concatenator = _make_concatenator()

    # Same header stamp, but the second cloud arrives after the first one's collector timed out.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.0) == []
    frames = concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.5)

    assert [frame.status for frame in frames] == [FrameStatus.TIMEOUT]
    assert frames[0].result.concatenated_cloud.width == len(_POINTS)


def test_close_expired_collectors_and_flush_return_the_open_collectors():
    concatenator = _make_concatenator()
    # The second lidar_top cloud cannot join the first collector, so it opens another one.
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.0)
    concatenator.process_cloud("lidar_top", _make_cloud(10, 100_000_000), arrival_time=100.1)

    assert concatenator.close_expired_collectors(100.15) == []
    expired = concatenator.close_expired_collectors(100.25)
    remaining = concatenator.flush()

    assert [frame.first_arrival_time for frame in expired] == [100.0]
    assert [frame.first_arrival_time for frame in remaining] == [100.1]
    assert all(frame.status is FrameStatus.TIMEOUT for frame in expired + remaining)


def test_advanced_strategy_uses_the_given_offsets_and_noise_window():
    concatenator = _make_concatenator(
        matching_strategy="advanced",
        lidar_timestamp_offsets=(0.0, 0.04, 0.08),
        lidar_timestamp_noise_window=(0.01, 0.01, 0.01),
    )

    frames = _feed_one_frame(
        concatenator,
        {
            "lidar_top": _make_cloud(10, 0),
            "lidar_left": _make_cloud(10, 40_000_000),
            "lidar_right": _make_cloud(10, 80_000_000),
        },
    )

    assert frames[0].status is FrameStatus.COMPLETE
    assert frames[0].reference_time == pytest.approx(10.0)
    assert frames[0].noise_window == pytest.approx(0.01)


@pytest.mark.parametrize(
    "call",
    [
        lambda: _make_concatenator(matching_strategy="advanced"),
        lambda: _make_concatenator(matching_strategy="no_such_strategy"),
        lambda: _make_concatenator().process_cloud(
            "unknown_topic", _make_cloud(10, 0), arrival_time=100.0
        ),
    ],
    ids=["advanced_without_offsets", "unknown_strategy", "unknown_topic"],
)
def test_invalid_input_raises_value_error(call):
    with pytest.raises(ValueError):
        call()


def test_build_diagnostics_forwards_its_options():
    concatenator = _make_concatenator()
    frames = _feed_one_frame(concatenator, {topic: _make_cloud(10, 0) for topic in _INPUT_TOPICS})

    default = concatenator.build_diagnostics(frames[0])
    custom = concatenator.build_diagnostics(
        frames[0],
        node_name="offline",
        diagnostic_name="concat",
        processing_time_ms=2.5,
        now=10.1,
        drop_previous_but_late=True,
    )

    default_values = {kv.key: kv.value for kv in default.values}
    custom_values = {kv.key: kv.value for kv in custom.values}
    assert default.name == "concatenate_data_synchronizer"
    assert default.level == DiagnosticStatus.OK
    assert "Processing time (ms)" not in default_values
    assert "Pipeline latency (ms)" not in default_values
    assert custom.name == "offline: concat"
    assert custom.level == DiagnosticStatus.ERROR
    assert custom_values["Processing time (ms)"] == "2.500000"
    assert custom_values["Pipeline latency (ms)"] == "100.000000"


def test_frames_outlive_the_concatenator():
    frames = _feed_one_frame(
        _make_concatenator(), {topic: _make_cloud(10, 0) for topic in _INPUT_TOPICS}
    )
    gc.collect()

    status = _make_concatenator().build_diagnostics(frames[0])

    assert frames[0].result.concatenated_cloud.width == len(_INPUT_TOPICS) * len(_POINTS)
    assert status.level == DiagnosticStatus.OK


def test_replaying_the_same_input_gives_identical_frames():
    def replay():
        concatenator = _make_concatenator(
            ["lidar_x", "lidar_y"],
            is_motion_compensated=True,
            publish_synchronized_pointcloud=True,
        )
        concatenator.set_transform(_make_transform("lidar_x"))
        for ms in range(9900, 10250, 50):
            concatenator.process_twist(_make_twist(ms // 1000, (ms % 1000) * 1_000_000, vx=1.0))
        frames = _feed_one_frame(
            concatenator,
            {
                "lidar_x": _make_cloud(10, 0, _UNIT_POINTS, "lidar_x"),
                "lidar_y": _make_cloud(10, 50_000_000),
            },
        )
        result = frames[0].result
        return result.concatenated_cloud, result.concatenation_info, result.transformed_clouds

    # Compare the messages, not their CDR bytes: the serializer leaves alignment padding
    # uninitialized, so equal messages can serialize differently.
    assert replay() == replay()
