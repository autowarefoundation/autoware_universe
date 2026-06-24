# Copyright 2025 TIER IV, Inc.
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

import math
import struct

from autoware_pointcloud_preprocessor.concatenate_pointclouds import CollectorStatus
from autoware_pointcloud_preprocessor.concatenate_pointclouds import CombineCloudHandler
from autoware_pointcloud_preprocessor.concatenate_pointclouds import Concatenator
from autoware_pointcloud_preprocessor.concatenate_pointclouds import build_diagnostics
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

_INPUT_TOPICS = ["lidar_top", "lidar_left", "lidar_right"]
_OUTPUT_FRAME = "base_link"

# Minimal x/y/z FLOAT32 input layout (point_step = 12). convert_to_xyzirc_cloud upgrades it to the
# 16-byte PointXYZIRC layout internally, so the concatenated output has point_step 16.
_INPUT_POINT_STEP = 12
_XYZIRC_POINT_STEP = 16
_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]

_POINTS = [(10.0, 0.0, 1.0), (0.0, 10.0, 2.0), (0.0, 0.0, 10.0)]

# Advanced strategy config (mirrors the node parameters), one entry per input topic.
_OFFSETS = [0.0, 0.04, 0.08]
_NOISE = [0.01, 0.01, 0.01]


def _make_cloud(sec, nanosec, points=_POINTS, frame_id=_OUTPUT_FRAME):
    msg = PointCloud2()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _INPUT_POINT_STEP
    msg.row_step = _INPUT_POINT_STEP * len(points)
    msg.is_dense = True
    data = bytearray()
    for x, y, z in points:
        data += struct.pack("<fff", x, y, z)
    msg.data = bytes(data)
    return msg


def _read_xyz(msg):
    return [struct.unpack_from("<fff", msg.data, i * msg.point_step) for i in range(msg.width)]


def _quat_z(angle_rad):
    """Return the unit quaternion (x, y, z, w) for a rotation of angle_rad about +Z."""
    return (0.0, 0.0, math.sin(angle_rad / 2.0), math.cos(angle_rad / 2.0))


def _make_transform(child_frame, translation, quaternion, parent_frame=_OUTPUT_FRAME):
    t = TransformStamped()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
    (
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
    ) = quaternion
    return t


def _make_twist(sec, nanosec, vx=0.0, wz=0.0):
    tw = TwistWithCovarianceStamped()
    tw.header.stamp.sec = sec
    tw.header.stamp.nanosec = nanosec
    tw.twist.twist.linear.x = vx
    tw.twist.twist.angular.z = wz
    return tw


def _approx_point(p, q, tol=1e-4):
    return all(abs(a - b) <= tol for a, b in zip(p, q))


def _contains_point(points, q, tol=1e-4):
    return any(_approx_point(p, q, tol) for p in points)


# --------------------------- CombineCloudHandler (single group) ---------------------------


def test_combine_three_clouds_in_output_frame():
    handler = CombineCloudHandler(_INPUT_TOPICS, _OUTPUT_FRAME, is_motion_compensated=False)
    clouds = {topic: _make_cloud(10, 0) for topic in _INPUT_TOPICS}

    result = handler.combine_pointclouds(clouds)

    assert result.concatenated_cloud is not None
    assert result.concatenated_cloud.width == len(_INPUT_TOPICS) * len(_POINTS)
    assert result.concatenated_cloud.point_step == _XYZIRC_POINT_STEP
    assert result.concatenated_cloud.header.frame_id == _OUTPUT_FRAME
    assert _read_xyz(result.concatenated_cloud) == _POINTS * len(_INPUT_TOPICS)


def test_combine_reports_original_stamps():
    handler = CombineCloudHandler(_INPUT_TOPICS, _OUTPUT_FRAME, is_motion_compensated=False)
    clouds = {topic: _make_cloud(10, 0) for topic in _INPUT_TOPICS}

    result = handler.combine_pointclouds(clouds)

    assert set(result.topic_to_original_stamp) == set(_INPUT_TOPICS)
    for stamp in result.topic_to_original_stamp.values():
        assert stamp == 10.0


def test_non_identity_transform_is_applied():
    # A single sensor-frame cloud with a real rotation + translation extrinsic, to exercise
    # to_eigen_matrix (quaternion -> matrix) and transform_pointcloud (the hand-written replacement
    # for pcl_ros::transformPointCloud) numerically -- the rest of the suite only hits identity.
    handler = CombineCloudHandler(["lidar_x"], _OUTPUT_FRAME, is_motion_compensated=False)
    # 90 deg about +Z, then translate by (10, 20, 30).
    handler.set_transform(_make_transform("lidar_x", (10.0, 20.0, 30.0), _quat_z(math.pi / 2.0)))
    points = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    cloud = _make_cloud(10, 0, points=points, frame_id="lidar_x")

    result = handler.combine_pointclouds({"lidar_x": cloud})

    # R_z(90): (x, y, z) -> (-y, x, z); then + (10, 20, 30). Point order is preserved.
    expected = [(10.0, 21.0, 30.0), (9.0, 20.0, 30.0), (10.0, 20.0, 31.0)]
    out = _read_xyz(result.concatenated_cloud)
    assert len(out) == len(expected)
    for got, want in zip(out, expected):
        assert _approx_point(got, want), f"{got} != {want}"
    assert result.dropped_frames_missing_transform == []


def test_keep_input_frame_returns_points_in_sensor_frame():
    # keep_input_frame maps the output-frame cloud back via inverse(extrinsic); exercises the
    # ->inverse() path. The round trip must recover the original sensor-frame points.
    handler = CombineCloudHandler(
        ["lidar_x"],
        _OUTPUT_FRAME,
        is_motion_compensated=False,
        publish_synchronized_pointcloud=True,
        keep_input_frame_in_synchronized_pointcloud=True,
    )
    handler.set_transform(_make_transform("lidar_x", (10.0, 20.0, 30.0), _quat_z(math.pi / 2.0)))
    points = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    cloud = _make_cloud(10, 0, points=points, frame_id="lidar_x")

    result = handler.combine_pointclouds({"lidar_x": cloud})

    assert result.transformed_clouds is not None
    sensor_cloud = result.transformed_clouds["lidar_x"]
    assert sensor_cloud.header.frame_id == "lidar_x"
    out = _read_xyz(sensor_cloud)
    for got, want in zip(out, points):
        assert _approx_point(got, want), f"{got} != {want}"


def test_motion_compensation_shifts_newer_cloud():
    # Two clouds 0.1 s apart with a constant 1.0 m/s twist along +x: the oldest is the reference and
    # stays put; the newer cloud is shifted by 0.1 s * 1.0 m/s = 0.1 m. Exercises
    # correct_pointcloud_motion + compute_transform_to_adjust_for_old_timestamp with real twist.
    handler = CombineCloudHandler(["lidar_a", "lidar_b"], _OUTPUT_FRAME, is_motion_compensated=True)
    for ms in range(
        9900, 10250, 50
    ):  # twists at 9.90 .. 10.20 s, 50 ms apart, bracketing the clouds
        handler.process_twist(_make_twist(ms // 1000, (ms % 1000) * 1_000_000, vx=1.0))

    cloud_a = _make_cloud(
        10, 0, points=[(1.0, 0.0, 0.0)], frame_id=_OUTPUT_FRAME
    )  # oldest (10.0 s)
    cloud_b = _make_cloud(
        10, 100_000_000, points=[(2.0, 0.0, 0.0)], frame_id=_OUTPUT_FRAME
    )  # 10.1 s

    result = handler.combine_pointclouds({"lidar_a": cloud_a, "lidar_b": cloud_b})

    out = _read_xyz(result.concatenated_cloud)
    assert len(out) == 2
    assert not result.no_twist_available
    assert _contains_point(out, (1.0, 0.0, 0.0))  # oldest unchanged
    assert _contains_point(out, (2.1, 0.0, 0.0))  # newest shifted +0.1 m in x


def test_missing_transform_drops_frame_and_reports_it():
    # An extrinsic is provided for lidar_x only; lidar_y must be dropped (not silently mis-placed),
    # and the dropped frame must be reported in the result (the node turns this into a warning).
    handler = CombineCloudHandler(
        ["lidar_x", "lidar_y"], _OUTPUT_FRAME, is_motion_compensated=False
    )
    handler.set_transform(_make_transform("lidar_x", (0.0, 0.0, 0.0), _quat_z(0.0)))
    cloud_x = _make_cloud(10, 0, frame_id="lidar_x")
    cloud_y = _make_cloud(10, 0, frame_id="lidar_y")

    result = handler.combine_pointclouds({"lidar_x": cloud_x, "lidar_y": cloud_y})

    assert result.concatenated_cloud.width == len(_POINTS)  # only lidar_x survived
    assert result.dropped_frames_missing_transform == ["lidar_y"]


# --------------------------- Concatenator: naive strategy ---------------------------


def test_naive_completes_when_all_topics_arrive():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )

    # All three arrive within 20 ms (< 0.2 s timeout) -> one complete group.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00) == []
    assert (
        concatenator.process_cloud("lidar_left", _make_cloud(10, 1_000_000), arrival_time=100.01)
        == []
    )
    frames = concatenator.process_cloud(
        "lidar_right", _make_cloud(10, 2_000_000), arrival_time=100.02
    )

    assert len(frames) == 1
    assert frames[0].status == CollectorStatus.COMPLETE
    assert frames[0].result.concatenated_cloud.width == len(_INPUT_TOPICS) * len(_POINTS)


def test_naive_times_out_incomplete_group():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )

    # Only lidar_top; then a cloud arriving 0.5 s later (> 0.2 s timeout) closes the stale group.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.0) == []
    frames = concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.5)

    assert len(frames) == 1
    assert frames[0].status == CollectorStatus.TIMEOUT
    assert frames[0].result.concatenated_cloud.width == len(_POINTS)  # only lidar_top


def test_arrival_time_drives_timeout_independent_of_stamp():
    """The timeout follows the receive time, not the header stamp."""
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )

    # Header stamps are identical, so grouping by stamp alone would keep the group open. But the
    # clouds *arrive* 0.5 s apart, so the arrival-driven timer closes the first group before the
    # second cloud is grouped.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.0) == []
    frames = concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.5)

    assert len(frames) == 1
    assert frames[0].status == CollectorStatus.TIMEOUT
    assert frames[0].result.concatenated_cloud.width == len(_POINTS)  # only lidar_top


def test_arrival_time_groups_clouds_with_far_apart_stamps():
    """Clouds that arrive together are grouped even if their header stamps are far apart."""
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )

    # Stamps span 1.0 s, but all three arrive within 20 ms, so the arrival-driven path groups them
    # as COMPLETE.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00) == []
    assert (
        concatenator.process_cloud("lidar_left", _make_cloud(10, 500_000_000), arrival_time=100.01)
        == []
    )
    frames = concatenator.process_cloud("lidar_right", _make_cloud(11, 0), arrival_time=100.02)

    assert len(frames) == 1
    assert frames[0].status == CollectorStatus.COMPLETE
    assert frames[0].result.concatenated_cloud.width == len(_INPUT_TOPICS) * len(_POINTS)


# --------------------------- Concatenator: advanced strategy ---------------------------


def _advanced_concatenator(timeout_sec=0.2):
    return Concatenator(
        _INPUT_TOPICS,
        _OUTPUT_FRAME,
        tf_static={},
        timeout_sec=timeout_sec,
        is_motion_compensated=False,
        matching_strategy="advanced",
        lidar_timestamp_offsets=_OFFSETS,
        lidar_timestamp_noise_window=_NOISE,
    )


def test_advanced_requires_offsets():
    try:
        Concatenator(
            _INPUT_TOPICS,
            _OUTPUT_FRAME,
            tf_static={},
            timeout_sec=0.2,
            matching_strategy="advanced",
        )
    except ValueError:
        pass
    else:
        raise AssertionError("advanced strategy without offsets/noise should raise ValueError")


def test_advanced_groups_offset_corrected_timestamps():
    concatenator = _advanced_concatenator()

    # One scan: per-topic stamps are staggered by exactly the offsets, so the offset-corrected
    # timestamps all map to 10.00 s; arriving within 20 ms they form a single complete group.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00) == []
    assert (
        concatenator.process_cloud("lidar_left", _make_cloud(10, 40_000_000), arrival_time=100.01)
        == []
    )
    frames = concatenator.process_cloud(
        "lidar_right", _make_cloud(10, 80_000_000), arrival_time=100.02
    )

    assert len(frames) == 1
    assert frames[0].status == CollectorStatus.COMPLETE
    assert frames[0].result.concatenated_cloud.width == len(_INPUT_TOPICS) * len(_POINTS)
    # The advanced strategy records the matching window in the info message.
    assert frames[0].result.concatenation_info is not None


def test_advanced_times_out_when_clock_advances():
    concatenator = _advanced_concatenator(timeout_sec=0.2)

    # lidar_top for one scan, then lidar_top arriving 0.5 s later: the first group can no longer be
    # completed, so the advancing arrival clock closes it as a timeout.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.0) == []
    frames = concatenator.process_cloud(
        "lidar_top", _make_cloud(10, 500_000_000), arrival_time=100.5
    )

    assert len(frames) == 1
    assert frames[0].status == CollectorStatus.TIMEOUT
    assert frames[0].result.concatenated_cloud.width == len(_POINTS)


def test_advanced_does_not_merge_out_of_window_cloud():
    concatenator = _advanced_concatenator(timeout_sec=0.2)

    # lidar_left's stamp is far outside lidar_top's window (corrected 10.16 vs window ~[9.99, 10.01]):
    # it must start its own group, not join. Both arrive close together (within the timeout), so
    # neither is closed during processing and both remain open.
    assert concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00) == []
    assert (
        concatenator.process_cloud("lidar_left", _make_cloud(10, 200_000_000), arrival_time=100.05)
        == []
    )

    # Flushing the stream emits both still-open (incomplete) groups as timeouts.
    frames = concatenator.flush()
    assert len(frames) == 2
    assert all(frame.status == CollectorStatus.TIMEOUT for frame in frames)
    assert all(frame.result.concatenated_cloud.width == len(_POINTS) for frame in frames)


def test_process_cloud_rejects_unknown_topic():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    try:
        concatenator.process_cloud("not_a_configured_topic", _make_cloud(10, 0), arrival_time=100.0)
    except ValueError:
        pass
    else:
        raise AssertionError("process_cloud should reject a topic not in input_topics")


# --------------------------- Diagnostics (mirrors the node's check_concat_status) ---------------


def _values(status):
    """Flatten a DiagnosticStatus's key/value pairs into a dict for assertions."""
    return {kv.key: kv.value for kv in status.values}


def test_build_diagnostics_complete_naive_is_ok():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00)
    concatenator.process_cloud("lidar_left", _make_cloud(10, 1_000_000), arrival_time=100.01)
    frames = concatenator.process_cloud(
        "lidar_right", _make_cloud(10, 2_000_000), arrival_time=100.02
    )
    assert frames[0].status == CollectorStatus.COMPLETE

    status = concatenator.build_diagnostics(frames[0])
    values = _values(status)

    # A complete group is OK; the published message is "OK" (matching the node's DiagnosticsInterface).
    assert status.level == DiagnosticStatus.OK
    assert status.message == "OK"
    assert values["Pointcloud concatenation succeeded"] == "True"
    # Every input topic contributed; "Concatenated: <topic>" is "True" with a 9-decimal timestamp.
    for topic in _INPUT_TOPICS:
        assert values["Concatenated: " + topic] == "True"
        assert "Timestamp: " + topic in values
    assert values["Concatenated pointcloud timestamp"] == "10.000000000"
    # Naive records the first arrival timestamp, not an advanced reference window.
    assert "First pointcloud arrival timestamp" in values
    assert "Minimum reference timestamp" not in values


def test_build_diagnostics_timeout_reports_missing_topics_as_error():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    # Only lidar_top arrives; a later cloud advances the clock past the timeout and closes the group.
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.0)
    frames = concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.5)
    assert frames[0].status == CollectorStatus.TIMEOUT

    status = concatenator.build_diagnostics(frames[0])
    values = _values(status)

    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "Concatenated pointcloud is published but misses some topics"
    assert values["Pointcloud concatenation succeeded"] == "False"
    assert values["Concatenated: lidar_top"] == "True"
    assert values["Concatenated: lidar_left"] == "False"
    # A topic that never arrived has no "Timestamp:" entry.
    assert "Timestamp: lidar_left" not in values


def test_build_diagnostics_advanced_records_reference_window():
    concatenator = _advanced_concatenator()
    # Per-topic stamps staggered by the offsets, so all correct to 10.00 s -> one complete group
    # whose reference is 10.00 s with a 0.01 s noise window.
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00)
    concatenator.process_cloud("lidar_left", _make_cloud(10, 40_000_000), arrival_time=100.01)
    frames = concatenator.process_cloud(
        "lidar_right", _make_cloud(10, 80_000_000), arrival_time=100.02
    )
    assert frames[0].status == CollectorStatus.COMPLETE

    values = _values(concatenator.build_diagnostics(frames[0]))

    assert values["Minimum reference timestamp"] == "9.990000000"
    assert values["Maximum reference timestamp"] == "10.010000000"
    # Advanced does not emit the naive first-arrival entry.
    assert "First pointcloud arrival timestamp" not in values


def test_build_diagnostics_optional_latency_and_processing_time():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00)
    concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.01)
    frames = concatenator.process_cloud("lidar_right", _make_cloud(10, 0), arrival_time=100.02)

    # now=10.1 s is 0.1 s after the 10.0 s stamps -> 100 ms latency; processing time is passed through.
    values = _values(concatenator.build_diagnostics(frames[0], now=10.1, processing_time_ms=2.5))

    assert values["Processing time (ms)"] == "2.500000"
    assert values["Pipeline latency (ms)"] == "100.000000"
    for topic in _INPUT_TOPICS:
        assert values["Latency (ms): " + topic] == "100.000000"


def test_build_diagnostics_omits_runtime_fields_by_default():
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00)
    concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.01)
    frames = concatenator.process_cloud("lidar_right", _make_cloud(10, 0), arrival_time=100.02)

    values = _values(concatenator.build_diagnostics(frames[0]))

    # Wall-clock-only fields are not fabricated when no data is supplied.
    assert "Processing time (ms)" not in values
    assert "Pipeline latency (ms)" not in values
    assert "Latency (ms): lidar_top" not in values


def test_build_diagnostics_module_function_matches_method():
    # The module-level function and the Concatenator.build_diagnostics convenience wrapper must
    # produce the same DiagnosticStatus (the wrapper only injects the configured input_topics).
    concatenator = Concatenator(
        _INPUT_TOPICS, _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    concatenator.process_cloud("lidar_top", _make_cloud(10, 0), arrival_time=100.00)
    concatenator.process_cloud("lidar_left", _make_cloud(10, 0), arrival_time=100.01)
    frames = concatenator.process_cloud("lidar_right", _make_cloud(10, 0), arrival_time=100.02)

    via_function = build_diagnostics(frames[0], _INPUT_TOPICS)
    via_method = concatenator.build_diagnostics(frames[0])

    assert via_function.level == via_method.level
    assert via_function.message == via_method.message
    assert _values(via_function) == _values(via_method)


def test_build_diagnostics_empty_cloud_is_error():
    # A single source whose extrinsic is missing is dropped, so the concatenated cloud is empty.
    # The topic still "contributed" (it is in topic_to_original_stamp), so the failure surfaces as
    # the empty-cloud error rather than a missing-topic one.
    concatenator = Concatenator(
        ["lidar_x"], _OUTPUT_FRAME, tf_static={}, timeout_sec=0.2, is_motion_compensated=False
    )
    frames = concatenator.process_cloud(
        "lidar_x", _make_cloud(10, 0, frame_id="lidar_x"), arrival_time=100.0
    )
    assert frames[0].status == CollectorStatus.COMPLETE
    assert frames[0].result.concatenated_cloud.width == 0

    status = concatenator.build_diagnostics(frames[0])
    values = _values(status)

    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "Concatenated pointcloud is empty"
    assert values["Concatenated: lidar_x"] == "True"  # contributed, even though it was dropped
