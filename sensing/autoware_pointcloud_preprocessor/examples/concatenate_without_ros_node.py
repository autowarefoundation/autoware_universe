#!/usr/bin/env python3
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

"""Concatenate three point clouds from Python, without creating or spinning a ROS node.

The Python counterpart of concatenate_without_ros_node.cpp. Clouds, transforms and twist are handed
to the same C++ core the concatenation node runs, so the run is deterministic and every input is
processed. rclpy is used for message serialization only: no rclpy.init(), no executor, no
middleware.

Run it from the workspace root after building the package:
  source install/setup.bash
  python3 src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor/examples/\
concatenate_without_ros_node.py
"""

import sys

from autoware_pointcloud_preprocessor.concatenate_pointclouds import Concatenator
from autoware_pointcloud_preprocessor.concatenate_pointclouds import FrameStatus
from autoware_pointcloud_preprocessor.concatenate_pointclouds import MotionCompensationStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

INPUT_TOPICS = ["/sensor0/points", "/sensor1/points", "/sensor2/points"]
SENSOR_FRAMES = ["sensor0", "sensor1", "sensor2"]
OUTPUT_FRAME = "base_link"

# One cloud per sensor, 10 ms apart, so motion compensation has something to correct.
BASE_STAMP_SEC = 1000.0
STAMP_STEP_SEC = 0.01

POINTS_PER_CLOUD = 40000

# The PointXYZIRC layout (16 bytes per point), the fields the default output requires.
XYZIRC_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
    PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
    PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
]


def to_stamp(seconds):
    sec = int(seconds)
    return Time(sec=sec, nanosec=round((seconds - sec) * 1e9))


def to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def make_cloud(sensor_index, stamp_sec, num_points):
    """Return a PointXYZIRC cloud of `num_points` points, offset per sensor to tell them apart."""
    points = np.zeros(num_points, dtype=point_cloud2.dtype_from_fields(XYZIRC_FIELDS))
    indices = np.arange(num_points)
    points["x"] = sensor_index * 10 + indices
    points["y"] = sensor_index
    points["intensity"] = 100 + sensor_index
    points["return_type"] = 1
    points["channel"] = indices % 64
    header = Header(stamp=to_stamp(stamp_sec), frame_id=SENSOR_FRAMES[sensor_index])
    return point_cloud2.create_cloud(header, XYZIRC_FIELDS, points)


def make_transform(sensor_index):
    """Return an identity rotation, sensor_index metres along y: each source gets its own lane."""
    transform = TransformStamped()
    transform.header.frame_id = OUTPUT_FRAME
    transform.child_frame_id = SENSOR_FRAMES[sensor_index]
    transform.transform.translation.y = float(sensor_index)
    transform.transform.rotation.w = 1.0
    return transform


def make_twist(stamp_sec):
    twist = TwistWithCovarianceStamped()
    twist.header.stamp = to_stamp(stamp_sec)
    twist.header.frame_id = OUTPUT_FRAME
    twist.twist.twist.linear.x = 5.0  # m/s
    return twist


def check(condition, what):
    print(f"  {'ok  ' if condition else 'FAIL'} {what}")
    return condition


def main():
    # ---------------------------------------------------------------------------------------
    # 1. Build the core. No node, no parameter server - the configuration is just arguments.
    #    This is the same object the concatenation node drives from its subscriptions.
    # ---------------------------------------------------------------------------------------
    concatenator = Concatenator(
        INPUT_TOPICS,
        OUTPUT_FRAME,
        timeout_sec=0.2,
        is_motion_compensated=True,
        matching_strategy="advanced",
        lidar_timestamp_offsets=[0.0] * len(INPUT_TOPICS),
        lidar_timestamp_noise_window=[0.02] * len(INPUT_TOPICS),
    )

    # Transforms the node would look up from tf2, handed over directly: no TF listener, no
    # waiting, no lookup failures. set_transform() raises ValueError for one it cannot use.
    for sensor_index in range(len(SENSOR_FRAMES)):
        concatenator.set_transform(make_transform(sensor_index))

    # Ego motion the node would take from /twist. Without this the frame reports
    # NO_TWIST_AVAILABLE and the clouds are not motion compensated.
    concatenator.process_twist(make_twist(BASE_STAMP_SEC))

    # ---------------------------------------------------------------------------------------
    # 2. Feed the clouds in arrival order. Matching, grouping and the timeout are the core's job;
    #    the node uses now() as the arrival time, offline it comes from the data (e.g. the
    #    rosbag record time), so there is no clock and no timer.
    # ---------------------------------------------------------------------------------------
    frames = []
    for sensor_index, topic in enumerate(INPUT_TOPICS):
        stamp_sec = BASE_STAMP_SEC + sensor_index * STAMP_STEP_SEC
        cloud = make_cloud(sensor_index, stamp_sec, POINTS_PER_CLOUD)
        frames += concatenator.process_cloud(topic, cloud, arrival_time=stamp_sec)
    # The three clouds completed one collector. A stream that ends with an incomplete collector
    # would call concatenator.flush() here to close it as a timeout.
    if len(frames) != 1:
        print(f"expected one concatenated frame, got {len(frames)}", file=sys.stderr)
        return 1
    frame = frames[0]
    result = frame.result

    # ---------------------------------------------------------------------------------------
    # 3. Report, and check what the node would have published on its topics.
    # ---------------------------------------------------------------------------------------
    cloud = result.concatenated_cloud
    info = result.concatenation_info
    point_count = cloud.width * cloud.height

    print("concatenated cloud")
    print(f"  frame_id   : {cloud.header.frame_id}")
    print(f"  stamp      : {to_seconds(cloud.header.stamp):g}")
    print(f"  points     : {point_count}")
    print(f"  point_step : {cloud.point_step}")
    print(f"  sources    : {len(info.source_info)}")
    print(f"  frames     : {len(frames)}")
    print()

    all_ok = True
    all_ok &= check(cloud.header.frame_id == OUTPUT_FRAME, "cloud is in the output frame")
    all_ok &= check(
        point_count == POINTS_PER_CLOUD * len(INPUT_TOPICS), "every source contributed its points"
    )
    all_ok &= check(
        to_seconds(cloud.header.stamp) == BASE_STAMP_SEC, "stamp is the oldest input stamp"
    )
    all_ok &= check(
        frame.status is FrameStatus.COMPLETE,
        "the collector closed on completion, not on its timeout",
    )
    all_ok &= check(concatenator.flush() == [], "no collector is left open")
    all_ok &= check(len(info.source_info) == len(INPUT_TOPICS), "info describes every source")
    all_ok &= check(
        result.motion_compensation_status is MotionCompensationStatus.VALID,
        "motion compensation ran against the supplied twist",
    )
    all_ok &= check(
        not result.dropped_sources_missing_transform, "no source was dropped for a missing TF"
    )

    # The diagnostics the node publishes are built by the core too, straight from the frame, so
    # they can be inspected offline without a diagnostic_updater.
    status = frame.build_diagnostics(node_name="concatenate_without_ros_node")

    print()
    print("diagnostics")
    print(f"  level   : {int.from_bytes(status.level, 'little')}")
    print(f"  message : {status.message}")
    for key_value in status.values:
        print(f"  {key_value.key} = {key_value.value}")
    print()
    all_ok &= check(
        "includes all topics" in status.message, "diagnostics report a complete concatenation"
    )

    print()
    print(f"{'PASS' if all_ok else 'FAIL'}: concatenation ran without a ROS node")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
