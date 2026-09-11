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

"""Characterization test for PointCloudConcatenateDataSynchronizerComponent.

Purpose
-------
This file is a *safety net* for refactoring the ``concatenate_data`` module. It does not
express what the node *should* do; it records what the node *currently does*, as seen from
outside the process. As long as every assertion here keeps passing, an arbitrary rewrite of
the internals is invisible to any other node in the system.

Method
------
Pure black box. The node is brought up as a real composable node, driven only by published
topics (point clouds, twist, odometry, TF), and observed only through the topics it
publishes:

* ``output``                     - the concatenated point cloud
* ``output_info``                - ConcatenatedPointCloudInfo metadata
* ``<input>/<postfix>``          - per-source synchronized point clouds
* ``/diagnostics``               - DiagnosticStatus produced by check_concat_status()
* ``<ns>/concatenate_data_synchronizer/debug/*`` - processing time / latency

No production header is included, no private member is reached into, and no production file
is modified. The only non-test change required is the ``add_ros_test()`` registration in
CMakeLists.txt.

Four node instances are launched side by side, each in its own namespace, so that
configurations which cannot coexist in one node (naive vs. advanced matching, twist vs.
odometry, drop-late vs. publish-late) are all covered by a single launch:

  A ``/char/adv``    advanced matching, motion compensation from twist,
                     synchronized clouds kept in the sensor frame
  B ``/char/naive``  naive matching, no motion compensation,
                     synchronized clouds in the output frame, postfix name collision
  C ``/char/strict`` advanced matching, publish_previous_but_late_pointcloud = False
  D ``/char/odom``   advanced matching, motion compensation from odometry

Reading the assertions
----------------------
Assertions marked ``CHARACTERIZED QUIRK`` capture behavior that is surprising but real. They
are recorded deliberately: if a refactor changes them, that is a behavior change and the
reviewer must decide whether it is intended, not a test that "just needs fixing".
"""

import struct
import time
import unittest

from autoware_internal_debug_msgs.msg import Float64Stamped
from autoware_sensing_msgs.msg import ConcatenatedPointCloudInfo
from autoware_sensing_msgs.msg import SourcePointCloudInfo
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
import launch
import launch.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# --------------------------------------------------------------------------------------
# Fixed world description
# --------------------------------------------------------------------------------------

OUTPUT_FRAME = "base_link"

# Sensor frames are prefixed so this test can never collide with the TF tree published by
# test_concatenate_node_component.py if both run in the same ROS domain.
SENSOR_FRAMES = ["char_left_lidar", "char_right_lidar", "char_top_lidar"]

# base_link -> sensor translations. Chosen to be integral and distinct per sensor so that
# every expected point below can be verified by inspection.
SENSOR_TRANSLATIONS = [
    (0.0, 1.0, 0.0),
    (0.0, -2.0, 0.0),
    (0.0, 0.0, 3.0),
]

# The same three points are published by every sensor, in its own frame.
SENSOR_POINTS = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
NUM_POINTS = len(SENSOR_POINTS)

# Per-sensor marker values, so that a point can be traced back to its source topic after
# concatenation has thrown the frame_id away.
SENSOR_INTENSITIES = [10, 20, 30]
SENSOR_RETURN_TYPES = [1, 2, 3]
SENSOR_CHANNELS = [100, 200, 300]

# Node configuration shared by every instance.
TIMEOUT_SEC = 0.2
NOISE_WINDOW = 0.01
TIMESTAMP_OFFSETS = [0.0, 0.04, 0.08]
VELOCITY_MPS = 1.0

# The concatenated point cloud is always PointXYZIRC, regardless of the input layout.
XYZIRC_POINT_STEP = 16
XYZIRC_FIELDS = [
    ("x", 0, PointField.FLOAT32),
    ("y", 4, PointField.FLOAT32),
    ("z", 8, PointField.FLOAT32),
    ("intensity", 12, PointField.UINT8),
    ("return_type", 13, PointField.UINT8),
    ("channel", 14, PointField.UINT16),
]

# Message timestamps are handed out from here so that no test ever reuses a stamp. Instance
# C in particular keeps the timestamp of the last published cloud forever, so overlapping
# stamps between tests would make results depend on execution order.
_BASE_SECONDS = 100


def next_base_stamp() -> float:
    """Return a fresh, strictly increasing base timestamp in seconds."""
    global _BASE_SECONDS
    _BASE_SECONDS += 1
    return float(_BASE_SECONDS)


# --------------------------------------------------------------------------------------
# Node instances under characterization
# --------------------------------------------------------------------------------------


class Instance:
    """Everything the test needs to know about one launched node instance."""

    def __init__(self, key, namespace, node_name, postfix, sync_topics, queue_size):
        self.key = key
        self.namespace = namespace
        self.node_name = node_name
        self.postfix = postfix
        self.inputs = [f"{namespace}/lidar/{name}/pointcloud" for name in ("left", "right", "top")]
        self.sync_topics = sync_topics
        self.queue_size = queue_size
        self.output = f"{namespace}/output"
        self.output_info = f"{namespace}/output_info"
        # ~/input/twist and ~/input/odom are left unremapped on purpose: the default
        # relative names are part of the interface being characterized.
        self.twist = f"{namespace}/{node_name}/input/twist"
        self.odom = f"{namespace}/{node_name}/input/odom"
        self.debug_ns = f"{namespace}/concatenate_data_synchronizer/debug"


# A: advanced matching, twist-driven motion compensation, synchronized clouds in sensor frame.
ADV = Instance(
    key="adv",
    namespace="/char/adv",
    node_name="char_concat_adv",
    postfix="pointcloud_sync",
    sync_topics=[
        "/char/adv/lidar/left/pointcloud_sync",
        "/char/adv/lidar/right/pointcloud_sync",
        "/char/adv/lidar/top/pointcloud_sync",
    ],
    queue_size=5,
)

# B: naive matching, no motion compensation, synchronized clouds in the output frame.
# The postfix "pointcloud" collides with the last element of every input topic name, which
# exercises the fallback branch of replace_sync_topic_name_postfix().
NAIVE = Instance(
    key="naive",
    namespace="/char/naive",
    node_name="char_concat_naive",
    postfix="pointcloud",
    sync_topics=[
        "/char/naive/lidar/left/pointcloud_synchronized",
        "/char/naive/lidar/right/pointcloud_synchronized",
        "/char/naive/lidar/top/pointcloud_synchronized",
    ],
    queue_size=3,
)

# C: drops concatenated clouds whose timestamp went backwards.
STRICT = Instance(
    key="strict",
    namespace="/char/strict",
    node_name="char_concat_strict",
    postfix="pointcloud_sync",
    sync_topics=[],
    queue_size=5,
)

# D: odometry-driven motion compensation.
ODOM = Instance(
    key="odom",
    namespace="/char/odom",
    node_name="char_concat_odom",
    postfix="pointcloud_sync",
    sync_topics=[],
    queue_size=5,
)

INSTANCES = [ADV, NAIVE, STRICT, ODOM]

ROSBAG_LENGTH_STRICT = 10.0


def _composable(instance, parameters):
    return ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::"
        "PointCloudConcatenateDataSynchronizerComponent",
        name=instance.node_name,
        namespace=instance.namespace,
        parameters=[parameters],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


@pytest.mark.launch_test
def generate_test_description():
    advanced_matching = {
        "matching_strategy.type": "advanced",
        "matching_strategy.lidar_timestamp_offsets": TIMESTAMP_OFFSETS,
        "matching_strategy.lidar_timestamp_noise_window": [NOISE_WINDOW] * 3,
    }

    nodes = [
        _composable(
            ADV,
            {
                "debug_mode": False,
                "rosbag_length": 0.0,
                "maximum_queue_size": ADV.queue_size,
                "timeout_sec": TIMEOUT_SEC,
                "is_motion_compensated": True,
                "publish_synchronized_pointcloud": True,
                "keep_input_frame_in_synchronized_pointcloud": True,
                "publish_previous_but_late_pointcloud": True,
                "synchronized_pointcloud_postfix": ADV.postfix,
                "input_twist_topic_type": "twist",
                "input_topics": ADV.inputs,
                "output_frame": OUTPUT_FRAME,
                **advanced_matching,
            },
        ),
        _composable(
            NAIVE,
            {
                "debug_mode": False,
                "rosbag_length": 0.0,
                "maximum_queue_size": NAIVE.queue_size,
                "timeout_sec": TIMEOUT_SEC,
                "is_motion_compensated": False,
                "publish_synchronized_pointcloud": True,
                "keep_input_frame_in_synchronized_pointcloud": False,
                "publish_previous_but_late_pointcloud": True,
                "synchronized_pointcloud_postfix": NAIVE.postfix,
                "input_twist_topic_type": "twist",
                "input_topics": NAIVE.inputs,
                "output_frame": OUTPUT_FRAME,
                "matching_strategy.type": "naive",
            },
        ),
        _composable(
            STRICT,
            {
                "debug_mode": False,
                "rosbag_length": ROSBAG_LENGTH_STRICT,
                "maximum_queue_size": STRICT.queue_size,
                "timeout_sec": TIMEOUT_SEC,
                "is_motion_compensated": False,
                "publish_synchronized_pointcloud": False,
                "keep_input_frame_in_synchronized_pointcloud": False,
                "publish_previous_but_late_pointcloud": False,
                "synchronized_pointcloud_postfix": STRICT.postfix,
                "input_twist_topic_type": "twist",
                "input_topics": STRICT.inputs,
                "output_frame": OUTPUT_FRAME,
                **advanced_matching,
            },
        ),
        _composable(
            ODOM,
            {
                "debug_mode": False,
                "rosbag_length": 0.0,
                "maximum_queue_size": ODOM.queue_size,
                "timeout_sec": TIMEOUT_SEC,
                "is_motion_compensated": True,
                "publish_synchronized_pointcloud": False,
                "keep_input_frame_in_synchronized_pointcloud": False,
                "publish_previous_but_late_pointcloud": True,
                "synchronized_pointcloud_postfix": ODOM.postfix,
                "input_twist_topic_type": "odom",
                "input_topics": ODOM.inputs,
                "output_frame": OUTPUT_FRAME,
                **advanced_matching,
            },
        ),
    ]

    container = ComposableNodeContainer(
        name="char_concatenate_container",
        namespace="char",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return launch.LaunchDescription([container, launch_testing.actions.ReadyToTest()])


# --------------------------------------------------------------------------------------
# Message construction helpers
# --------------------------------------------------------------------------------------

XYZIRCAEDT_FIELDS = [
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

XYZIRC_FIELD_MSGS = [
    PointField(name=name, offset=offset, datatype=datatype, count=1)
    for name, offset, datatype in XYZIRC_FIELDS
]

# Layout that the node rejects: 4 fields only, and intensity is FLOAT32 instead of UINT8.
XYZI_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def seconds_to_stamp(seconds: float):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    return Time(seconds=sec, nanoseconds=nanosec).to_msg()


def make_cloud(seconds: float, sensor_idx: int, layout: str = "xyzircaedt", empty: bool = False):
    """Build one input point cloud in the frame of sensor ``sensor_idx``."""
    header = Header()
    header.stamp = seconds_to_stamp(seconds)
    header.frame_id = SENSOR_FRAMES[sensor_idx]

    data = bytearray()
    if not empty:
        for x, y, z in SENSOR_POINTS:
            if layout == "xyzi":
                data += struct.pack("<ffff", x, y, z, 1.0)
                continue
            data += struct.pack("<fff", x, y, z)
            data += struct.pack(
                "<BBH",
                *(
                    SENSOR_INTENSITIES[sensor_idx],
                    SENSOR_RETURN_TYPES[sensor_idx],
                    SENSOR_CHANNELS[sensor_idx],
                ),
            )
            if layout == "xyzircaedt":
                data += struct.pack("<fffI", 0.0, 0.0, 1.0, 0)

    fields, point_step = {
        "xyzircaedt": (XYZIRCAEDT_FIELDS, 32),
        "xyzirc": (XYZIRC_FIELD_MSGS, 16),
        "xyzi": (XYZI_FIELDS, 16),
    }[layout]

    width = 0 if empty else NUM_POINTS
    return PointCloud2(
        header=header,
        height=1,
        width=width,
        is_dense=True,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * width,
        fields=fields,
        data=data,
    )


def make_static_transforms():
    transforms = []
    for frame, (x, y, z) in zip(SENSOR_FRAMES, SENSOR_TRANSLATIONS):
        tf = TransformStamped()
        tf.header.stamp = seconds_to_stamp(0.0)
        tf.header.frame_id = OUTPUT_FRAME
        tf.child_frame_id = frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        transforms.append(tf)
    return transforms


def make_twist(seconds: float) -> TwistWithCovarianceStamped:
    msg = TwistWithCovarianceStamped()
    msg.header.stamp = seconds_to_stamp(seconds)
    msg.header.frame_id = OUTPUT_FRAME
    msg.twist.twist.linear.x = VELOCITY_MPS
    return msg


def make_odometry(seconds: float) -> Odometry:
    msg = Odometry()
    msg.header.stamp = seconds_to_stamp(seconds)
    msg.header.frame_id = OUTPUT_FRAME
    msg.twist.twist.linear.x = VELOCITY_MPS
    return msg


# --------------------------------------------------------------------------------------
# Expected geometry
# --------------------------------------------------------------------------------------


def transformed_points(sensor_idx: int, motion_shift_x: float = 0.0):
    """Points of sensor ``sensor_idx`` expressed in base_link, optionally motion compensated."""
    tx, ty, tz = SENSOR_TRANSLATIONS[sensor_idx]
    return [(x + tx + motion_shift_x, y + ty, z + tz) for x, y, z in SENSOR_POINTS]


def motion_shift_for(sensor_idx: int) -> float:
    """Motion compensation applied to a sensor when all three clouds are present.

    correct_pointcloud_motion() walks every stamp older than the cloud's own stamp and
    accumulates ``linear.x * dt`` for each hop, so sensor i is shifted forward by the whole
    span between its stamp and the oldest stamp in the collector.
    """
    return VELOCITY_MPS * TIMESTAMP_OFFSETS[sensor_idx]


# --------------------------------------------------------------------------------------
# Observation helpers
# --------------------------------------------------------------------------------------

SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def read_xyzirc(cloud: PointCloud2):
    """Return the cloud as a list of (x, y, z, intensity, return_type, channel) tuples."""
    names = ("x", "y", "z", "intensity", "return_type", "channel")
    return [
        (
            float(p[0]),
            float(p[1]),
            float(p[2]),
            int(p[3]),
            int(p[4]),
            int(p[5]),
        )
        for p in point_cloud2.read_points(cloud, field_names=names)
    ]


def xyz_close(actual, expected, tol=1e-3):
    return all(abs(a - e) <= tol for a, e in zip(actual[:3], expected[:3]))


def diag_values(status: DiagnosticStatus):
    return {kv.key: kv.value for kv in status.values}


def diag_keys(status: DiagnosticStatus):
    return [kv.key for kv in status.values]


class Bus:
    """A single rclpy node that publishes into and records everything out of the node under test."""

    def __init__(self, node):
        self.node = node
        self.received = {}
        self._subs = []

    def subscribe(self, msg_type, topic, qos=None):
        self.received.setdefault(topic, [])
        self._subs.append(
            self.node.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.received[t].append(msg),
                qos if qos is not None else SENSOR_QOS,
            )
        )

    def clear(self):
        for buffer in self.received.values():
            del buffer[:]

    def spin(self, duration: float):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait_for(self, topic: str, count: int = 1, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(self.received[topic]) >= count:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return len(self.received[topic]) >= count

    def one(self, topic: str, timeout: float = 3.0):
        assert self.wait_for(topic, 1, timeout), f"no message on {topic} within {timeout}s"
        # Let a duplicate arrive if the node would publish one, so that callers can assert
        # on the exact count.
        self.spin(0.2)
        return self.received[topic][0]


# --------------------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------------------


# Where the assertions land. One input cloud travels through these functions, in order:
#
#   cloud_callback()            concatenate_and_time_sync_node.ipp - checks the point layout,
#                               then picks the collector this cloud belongs to
#   match_cloud_to_collector()  collector_matcher.ipp - naive or advanced matching
#   process_pointcloud()        cloud_collector.ipp - calls concatenate_callback() once every
#                               input has arrived, or when timeout_sec expires
#   combine_pointclouds()       combine_cloud_handler.cpp - converts to XYZIRC, transforms into
#                               output_frame, motion compensates, concatenates
#   publish_clouds()            concatenate_and_time_sync_node.ipp - publishes output,
#                               output_info and the synchronized clouds, then calls
#                               check_concat_status() and publish_debug_message()
#
# Every test below starts with a "Covers:" line naming the functions it pins down.


class TestConcatenateNodeCharacterization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_concat_characterization")

        # Static TF is transient_local, so the nodes pick it up whenever they start.
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)
        cls.tf_broadcaster.sendTransform(make_static_transforms())

        cls.bus = Bus(cls.node)
        cls.cloud_pubs = {}
        cls.twist_pub = cls.node.create_publisher(TwistWithCovarianceStamped, ADV.twist, 10)
        cls.odom_pub = cls.node.create_publisher(Odometry, ODOM.odom, 10)

        for instance in INSTANCES:
            cls.cloud_pubs[instance.key] = [
                cls.node.create_publisher(PointCloud2, topic, qos_profile=SENSOR_QOS)
                for topic in instance.inputs
            ]
            cls.bus.subscribe(PointCloud2, instance.output)
            cls.bus.subscribe(ConcatenatedPointCloudInfo, instance.output_info)
            for topic in instance.sync_topics:
                cls.bus.subscribe(PointCloud2, topic)

        cls.bus.subscribe(DiagnosticArray, "/diagnostics", qos=QoSProfile(depth=100))
        for topic in ("processing_time_ms", "cyclic_time_ms"):
            cls.bus.subscribe(Float64Stamped, f"{ADV.debug_ns}/{topic}", qos=QoSProfile(depth=10))
        for topic in ADV.inputs:
            # CHARACTERIZED QUIRK: the per-topic latency topic is built as
            # "debug" + <input topic>, and input topics are absolute, so the whole input
            # topic path is spliced into the debug namespace.
            cls.bus.subscribe(
                Float64Stamped,
                f"{ADV.debug_ns}{topic}/pipeline_latency_ms",
                qos=QoSProfile(depth=10),
            )

        # Give discovery time to connect every endpoint before the first test publishes.
        cls.bus.spin(2.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.bus.spin(0.3)
        self.bus.clear()

    # -- shared driving helpers ---------------------------------------------------------

    def publish_set(
        self,
        instance,
        base_seconds,
        stamps=None,
        layout="xyzircaedt",
        empty_indices=(),
        only=None,
        gap=0.01,
    ):
        """Publish one point cloud per sensor and return the stamps used, in sensor order."""
        if stamps is None:
            stamps = [base_seconds + offset for offset in TIMESTAMP_OFFSETS]
        indices = range(len(instance.inputs)) if only is None else only
        for idx in indices:
            cloud = make_cloud(stamps[idx], idx, layout=layout, empty=idx in empty_indices)
            self.cloud_pubs[instance.key][idx].publish(cloud)
            self.bus.spin(gap)
        return stamps

    def diagnostics_of(self, instance):
        """Every DiagnosticStatus published by ``instance`` since the last clear()."""
        statuses = []
        for array in self.bus.received["/diagnostics"]:
            for status in array.status:
                if status.hardware_id == instance.node_name:
                    statuses.append(status)
        return statuses

    def wait_for_diagnostic(self, instance, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            statuses = self.diagnostics_of(instance)
            if statuses:
                self.bus.spin(0.1)
                return self.diagnostics_of(instance)[-1]
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.fail(f"no diagnostics from {instance.node_name} within {timeout}s")

    def segment_of(self, info, cloud_points, topic):
        """Slice out the points that ``info`` attributes to ``topic``."""
        for source in info.source_info:
            if source.topic == topic:
                return source, cloud_points[source.idx_begin : source.idx_begin + source.length]
        self.fail(f"{topic} missing from source_info")

    # -- 01/02: static interface --------------------------------------------------------

    def test_01_published_topic_interface(self):
        """The set of published topics, their types and their QoS are part of the contract."""
        # Covers: initialize_pub_sub(), replace_sync_topic_name_postfix()
        for instance in INSTANCES:
            expected = {
                instance.output: "sensor_msgs/msg/PointCloud2",
                instance.output_info: "autoware_sensing_msgs/msg/ConcatenatedPointCloudInfo",
            }
            for topic in instance.sync_topics:
                expected[topic] = "sensor_msgs/msg/PointCloud2"

            for topic, msg_type in expected.items():
                endpoints = self.node.get_publishers_info_by_topic(topic)
                endpoints = [e for e in endpoints if e.node_name == instance.node_name]
                self.assertEqual(
                    len(endpoints), 1, f"{instance.node_name} should publish {topic} exactly once"
                )
                endpoint = endpoints[0]
                self.assertEqual(endpoint.topic_type, msg_type, topic)
                # Publishers use SensorDataQoS with depth overridden by maximum_queue_size.
                self.assertEqual(
                    endpoint.qos_profile.reliability,
                    QoSReliabilityPolicy.BEST_EFFORT,
                    topic,
                )
                self.assertEqual(
                    endpoint.qos_profile.durability, QoSDurabilityPolicy.VOLATILE, topic
                )
                self.assertEqual(endpoint.qos_profile.depth, instance.queue_size, topic)

        # Instances with publish_synchronized_pointcloud = False publish no sync topic.
        for instance in (STRICT, ODOM):
            for name in ("left", "right", "top"):
                topic = f"{instance.namespace}/lidar/{name}/pointcloud_sync"
                self.assertEqual(
                    self.node.get_publishers_info_by_topic(topic), [], f"{topic} must not exist"
                )

        # CHARACTERIZED QUIRK: NAIVE uses the postfix "pointcloud", which would rewrite
        # ".../lidar/left/pointcloud" to itself. replace_sync_topic_name_postfix() detects
        # the collision and appends the hard-coded "_synchronized" instead.
        for topic in NAIVE.sync_topics:
            self.assertTrue(topic.endswith("_synchronized"))

    def test_02_subscribed_topic_interface(self):
        """What the node listens to. Publishes nothing; only reads ROS 2 discovery."""
        # Covers: initialize_pub_sub(), and the twist/odom branch of the constructor.
        # One PointCloud2 subscription per input_topics entry. Names are used verbatim (not
        # namespace-resolved); QoS is SensorDataQoS with depth = maximum_queue_size.
        for instance in INSTANCES:
            for topic in instance.inputs:
                endpoints = [
                    e
                    for e in self.node.get_subscriptions_info_by_topic(topic)
                    if e.node_name == instance.node_name
                ]
                self.assertEqual(len(endpoints), 1, topic)
                self.assertEqual(endpoints[0].topic_type, "sensor_msgs/msg/PointCloud2")
                self.assertEqual(
                    endpoints[0].qos_profile.reliability, QoSReliabilityPolicy.BEST_EFFORT
                )
                self.assertEqual(endpoints[0].qos_profile.depth, instance.queue_size)

        def subscriber_names(topic):
            return {e.node_name for e in self.node.get_subscriptions_info_by_topic(topic)}

        # is_motion_compensated + input_twist_topic_type pick at most one velocity input:
        #   True/"twist" -> ~/input/twist    True/"odom" -> ~/input/odom    False -> neither
        # Only existence is checked; test_03 and test_17 prove the callbacks actually work.
        self.assertIn(ADV.node_name, subscriber_names(ADV.twist))
        self.assertNotIn(ADV.node_name, subscriber_names(ADV.odom))
        self.assertIn(ODOM.node_name, subscriber_names(ODOM.odom))
        self.assertNotIn(ODOM.node_name, subscriber_names(ODOM.twist))
        self.assertNotIn(NAIVE.node_name, subscriber_names(NAIVE.twist))
        self.assertNotIn(NAIVE.node_name, subscriber_names(NAIVE.odom))
        self.assertNotIn(STRICT.node_name, subscriber_names(STRICT.twist))
        self.assertNotIn(STRICT.node_name, subscriber_names(STRICT.odom))

    # -- 03-08: the happy path of the advanced instance ---------------------------------

    def test_03_advanced_concatenated_cloud(self):
        """Layout, header and payload of the concatenated cloud when all sources arrive."""
        # Covers: the whole chain, cloud_callback() through publish_clouds().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        stamps = self.publish_set(ADV, base)

        cloud = self.bus.one(ADV.output)
        self.assertEqual(len(self.bus.received[ADV.output]), 1)

        # combine_pointclouds(): the cloud carries the OLDEST input stamp, not the newest.
        self.assertEqual(cloud.header.stamp, seconds_to_stamp(min(stamps)))
        self.assertEqual(cloud.header.frame_id, OUTPUT_FRAME)

        # convert_to_xyzirc_cloud(): always unorganized PointXYZIRC, whatever came in.
        self.assertEqual(cloud.height, 1)
        self.assertEqual(cloud.width, NUM_POINTS * len(SENSOR_FRAMES))
        self.assertEqual(cloud.point_step, XYZIRC_POINT_STEP)
        self.assertEqual(cloud.row_step, XYZIRC_POINT_STEP * cloud.width)
        self.assertTrue(cloud.is_dense)
        self.assertFalse(cloud.is_bigendian)
        self.assertEqual([(f.name, f.offset, f.datatype) for f in cloud.fields], XYZIRC_FIELDS)
        self.assertEqual([f.count for f in cloud.fields], [1] * len(XYZIRC_FIELDS))

        # transformPointcloud() then correct_pointcloud_motion(). output_info tells us which
        # slice of the output belongs to which source, so we do not depend on their order.
        info = self.bus.one(ADV.output_info)
        points = read_xyzirc(cloud)
        for idx, topic in enumerate(ADV.inputs):
            source, segment = self.segment_of(info, points, topic)
            self.assertEqual(source.status, SourcePointCloudInfo.STATUS_OK, topic)
            self.assertEqual(source.length, NUM_POINTS, topic)
            expected = transformed_points(idx, motion_shift_for(idx))
            for actual, want in zip(segment, expected):
                self.assertTrue(xyz_close(actual, want), f"{topic}: {actual} != {want}")
                self.assertEqual(actual[3], SENSOR_INTENSITIES[idx], topic)
                self.assertEqual(actual[4], SENSOR_RETURN_TYPES[idx], topic)
                self.assertEqual(actual[5], SENSOR_CHANNELS[idx], topic)

    def test_04_advanced_concatenation_info(self):
        """Every field of ConcatenatedPointCloudInfo on the happy path."""
        # Covers: ConcatenationInfoManager, driven by combine_pointclouds().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        stamps = self.publish_set(ADV, base)

        cloud = self.bus.one(ADV.output)
        info = self.bus.one(ADV.output_info)
        self.assertEqual(len(self.bus.received[ADV.output_info]), 1)

        # The info header mirrors the concatenated cloud header exactly.
        self.assertEqual(info.header.stamp, cloud.header.stamp)
        self.assertEqual(info.header.frame_id, cloud.header.frame_id)
        self.assertTrue(info.concatenation_success)
        self.assertEqual(info.matching_strategy, ConcatenatedPointCloudInfo.STRATEGY_ADVANCED)

        # ConcatenationInfoManager::set_config(): the raw memcpy of two Time values,
        # i.e. the window [timestamp - noise_window, timestamp + noise_window].
        config = bytes(info.matching_strategy_config)
        self.assertEqual(len(config), 16)
        min_sec, min_nsec, max_sec, max_nsec = struct.unpack("<iIiI", config)
        window_min = min_sec + min_nsec * 1e-9
        window_max = max_sec + max_nsec * 1e-9
        self.assertAlmostEqual(window_min, base - NOISE_WINDOW, places=6)
        self.assertAlmostEqual(window_max, base + NOISE_WINDOW, places=6)

        # update_source_from_point_cloud(): source_info keeps input_topics order, not
        # arrival order.
        self.assertEqual([s.topic for s in info.source_info], ADV.inputs)

        offsets = sorted((s.idx_begin, s.length) for s in info.source_info)
        self.assertEqual(
            offsets, [(0, NUM_POINTS), (NUM_POINTS, NUM_POINTS), (2 * NUM_POINTS, NUM_POINTS)]
        )

        for idx, source in enumerate(info.source_info):
            self.assertEqual(source.status, SourcePointCloudInfo.STATUS_OK)
            # CHARACTERIZED QUIRK: the per-source header is snapshotted from the transformed
            # cloud, so its stamp is the ORIGINAL per-sensor stamp while its frame_id has
            # already become the output frame.
            self.assertEqual(source.header.stamp, seconds_to_stamp(stamps[idx]))
            self.assertEqual(source.header.frame_id, OUTPUT_FRAME)

    def test_05_advanced_synchronized_clouds_keep_input_frame(self):
        """keep_input_frame_in_synchronized_pointcloud = True publishes in the sensor frame."""
        # Covers: the synchronized-cloud branch of combine_pointclouds(), and publish_clouds().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        stamps = self.publish_set(ADV, base)

        self.bus.one(ADV.output)
        oldest = seconds_to_stamp(min(stamps))

        for idx, topic in enumerate(ADV.sync_topics):
            cloud = self.bus.one(topic)
            self.assertEqual(len(self.bus.received[topic]), 1, topic)
            # combine_pointclouds() overwrites the stamp with the concatenated (oldest)
            # one, NOT the stamp of the cloud this came from.
            self.assertEqual(cloud.header.stamp, oldest, topic)
            self.assertEqual(cloud.header.frame_id, SENSOR_FRAMES[idx], topic)
            self.assertEqual(cloud.width, NUM_POINTS, topic)
            self.assertEqual(cloud.point_step, XYZIRC_POINT_STEP, topic)
            self.assertEqual(
                [(f.name, f.offset, f.datatype) for f in cloud.fields], XYZIRC_FIELDS, topic
            )

            # Motion compensation is applied in base_link and then undone by the transform
            # back into the sensor frame, so only the compensation offset survives.
            shift = motion_shift_for(idx)
            expected = [(x + shift, y, z) for x, y, z in SENSOR_POINTS]
            for actual, want in zip(read_xyzirc(cloud), expected):
                self.assertTrue(xyz_close(actual, want), f"{topic}: {actual} != {want}")
                self.assertEqual(actual[3], SENSOR_INTENSITIES[idx], topic)
                self.assertEqual(actual[5], SENSOR_CHANNELS[idx], topic)

    def test_06_advanced_diagnostics_all_sources_present(self):
        """Diagnostic key order, values and message text when nothing is missing."""
        # Covers: check_concat_status().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        stamps = self.publish_set(ADV, base)
        self.bus.one(ADV.output)

        status = self.wait_for_diagnostic(ADV)
        self.assertEqual(status.hardware_id, ADV.node_name)
        self.assertEqual(status.name, f"{ADV.node_name}: {ADV.namespace}/{ADV.node_name}")
        self.assertEqual(status.level, DiagnosticStatus.OK)
        # CHARACTERIZED QUIRK: check_concat_status() passes the message "Concatenated
        # pointcloud is published and includes all topics", but DiagnosticsInterface
        # replaces the message of any OK status with the literal "OK" before publishing, so
        # that sentence is never observable. Only the non-OK messages below reach a
        # subscriber.
        self.assertEqual(status.message, "OK")

        expected_keys = ["Concatenated pointcloud timestamp"]
        # The advanced strategy reports the reference window instead of an arrival time.
        expected_keys += ["Minimum reference timestamp", "Maximum reference timestamp"]
        expected_keys += ["Processing time (ms)", "Pipeline latency (ms)"]
        for topic in ADV.inputs:
            expected_keys += [
                f"Concatenated: {topic}",
                f"Timestamp: {topic}",
                f"Latency (ms): {topic}",
            ]
        expected_keys += ["Pointcloud concatenation succeeded"]
        self.assertEqual(diag_keys(status), expected_keys)

        values = diag_values(status)
        self.assertEqual(values["Concatenated pointcloud timestamp"], f"{min(stamps):.9f}")
        self.assertEqual(values["Minimum reference timestamp"], f"{base - NOISE_WINDOW:.9f}")
        self.assertEqual(values["Maximum reference timestamp"], f"{base + NOISE_WINDOW:.9f}")
        self.assertEqual(values["Pointcloud concatenation succeeded"], "True")
        for idx, topic in enumerate(ADV.inputs):
            self.assertEqual(values[f"Concatenated: {topic}"], "True")
            self.assertEqual(values[f"Timestamp: {topic}"], f"{stamps[idx]:.9f}")

    def test_07_advanced_missing_source_times_out(self):
        """One source never arrives: the rest are still published after timeout_sec."""
        # Covers: the CloudCollector timeout timer, and check_concat_status() on a topic miss.
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        stamps = self.publish_set(ADV, base, only=[0, 1])

        cloud = self.bus.one(ADV.output, timeout=3.0)
        info = self.bus.one(ADV.output_info)
        self.assertEqual(len(self.bus.received[ADV.output]), 1)

        self.assertEqual(cloud.width, 2 * NUM_POINTS)
        self.assertEqual(cloud.header.stamp, seconds_to_stamp(stamps[0]))
        self.assertFalse(info.concatenation_success)

        missing = info.source_info[2]
        self.assertEqual(missing.topic, ADV.inputs[2])
        self.assertEqual(missing.status, SourcePointCloudInfo.STATUS_TIMEOUT)
        self.assertEqual(missing.idx_begin, 0)
        self.assertEqual(missing.length, 0)
        self.assertEqual(missing.header.frame_id, "")
        self.assertEqual(missing.header.stamp, seconds_to_stamp(0.0))

        # correct_pointcloud_motion(): only the two arrived sources, compensated relative to
        # their own oldest stamp.
        points = read_xyzirc(cloud)
        for idx in (0, 1):
            _, segment = self.segment_of(info, points, ADV.inputs[idx])
            expected = transformed_points(idx, motion_shift_for(idx))
            for actual, want in zip(segment, expected):
                self.assertTrue(xyz_close(actual, want), f"{ADV.inputs[idx]}: {actual} != {want}")

        status = self.wait_for_diagnostic(ADV)
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(
            status.message, "Concatenated pointcloud is published but misses some topics"
        )
        values = diag_values(status)
        self.assertEqual(values[f"Concatenated: {ADV.inputs[2]}"], "False")
        self.assertEqual(values["Pointcloud concatenation succeeded"], "False")
        # check_concat_status(): a missing topic adds no "Timestamp:"/"Latency (ms):" entry.
        self.assertNotIn(f"Timestamp: {ADV.inputs[2]}", values)
        self.assertNotIn(f"Latency (ms): {ADV.inputs[2]}", values)

    def test_08_advanced_debug_topics(self):
        """The debug publisher emits one Float64Stamped per metric and per input topic."""
        # Covers: publish_debug_message().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        self.publish_set(ADV, base)
        self.bus.one(ADV.output)

        for metric in ("processing_time_ms", "cyclic_time_ms"):
            topic = f"{ADV.debug_ns}/{metric}"
            msg = self.bus.one(topic)
            self.assertGreaterEqual(msg.data, 0.0, topic)

        for topic in ADV.inputs:
            latency_topic = f"{ADV.debug_ns}{topic}/pipeline_latency_ms"
            msg = self.bus.one(latency_topic)
            self.assertIsInstance(msg.data, float)

    # -- 09/10: degenerate inputs -------------------------------------------------------

    def test_09_incompatible_layout_is_dropped_silently(self):
        """A PointXYZI-layout cloud is rejected in the callback: nothing is ever published."""
        # Covers: the is_data_layout_compatible_with_point_xyzirc() guard in cloud_callback().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        self.publish_set(ADV, base, layout="xyzi")

        # cloud_callback() returns early, so process_pointcloud() is never reached: no
        # collector starts, and no timeout fires either. Wait well past timeout_sec.
        self.bus.spin(TIMEOUT_SEC * 4)
        self.assertEqual(self.bus.received[ADV.output], [])
        self.assertEqual(self.bus.received[ADV.output_info], [])
        for topic in ADV.sync_topics:
            self.assertEqual(self.bus.received[topic], [])
        self.assertEqual(self.diagnostics_of(ADV), [])

    def test_10_all_sources_empty(self):
        """Empty inputs still produce a published (empty) cloud, flagged in diagnostics."""
        # Covers: the empty-cloud paths of combine_pointclouds() and publish_clouds().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        stamps = self.publish_set(ADV, base, empty_indices=(0, 1, 2))

        cloud = self.bus.one(ADV.output)
        info = self.bus.one(ADV.output_info)

        self.assertEqual(cloud.width, 0)
        self.assertEqual(cloud.row_step, 0)
        self.assertEqual(cloud.height, 1)
        # combine_pointclouds() forces the XYZIRC layout even with nothing to concatenate.
        self.assertEqual(cloud.point_step, XYZIRC_POINT_STEP)
        self.assertEqual([(f.name, f.offset, f.datatype) for f in cloud.fields], XYZIRC_FIELDS)
        self.assertEqual(cloud.header.stamp, seconds_to_stamp(min(stamps)))
        self.assertEqual(cloud.header.frame_id, OUTPUT_FRAME)

        # CHARACTERIZED QUIRK: an all-empty concatenation still counts as successful,
        # because every source reported STATUS_OK - with length 0.
        self.assertTrue(info.concatenation_success)
        for source in info.source_info:
            self.assertEqual(source.status, SourcePointCloudInfo.STATUS_OK)
            self.assertEqual(source.length, 0)

        status = self.wait_for_diagnostic(ADV)
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(status.message, "Concatenated pointcloud is empty")

    def test_11_plain_xyzirc_input_is_accepted(self):
        """A 16-byte PointXYZIRC input is accepted exactly like the 32-byte XYZIRCAEDT one."""
        # Covers: convert_to_xyzirc_cloud().
        base = next_base_stamp()
        self.twist_pub.publish(make_twist(base))
        self.bus.spin(0.1)
        self.publish_set(ADV, base, layout="xyzirc")

        cloud = self.bus.one(ADV.output)
        info = self.bus.one(ADV.output_info)
        self.assertEqual(cloud.width, NUM_POINTS * len(SENSOR_FRAMES))
        self.assertTrue(info.concatenation_success)

        points = read_xyzirc(cloud)
        for idx, topic in enumerate(ADV.inputs):
            _, segment = self.segment_of(info, points, topic)
            expected = transformed_points(idx, motion_shift_for(idx))
            for actual, want in zip(segment, expected):
                self.assertTrue(xyz_close(actual, want), f"{topic}: {actual} != {want}")
                self.assertEqual(actual[3], SENSOR_INTENSITIES[idx])

    # -- 12-14: naive matching, no motion compensation ----------------------------------

    def test_12_naive_matches_on_arrival_time(self):
        """Naive matching ignores message stamps entirely; only arrival order matters."""
        # Covers: NaiveMatchingPolicy::match(), via NaiveCollectorMatcher.
        base = next_base_stamp()
        # Stamps far outside any advanced noise window - the advanced strategy would put
        # each of these in its own collector.
        stamps = [base, base + 5.0, base + 11.0]
        self.publish_set(NAIVE, base, stamps=stamps)

        cloud = self.bus.one(NAIVE.output)
        info = self.bus.one(NAIVE.output_info)
        self.assertEqual(len(self.bus.received[NAIVE.output]), 1)

        self.assertEqual(cloud.width, NUM_POINTS * len(SENSOR_FRAMES))
        self.assertEqual(cloud.header.stamp, seconds_to_stamp(min(stamps)))
        self.assertTrue(info.concatenation_success)
        self.assertEqual(info.matching_strategy, ConcatenatedPointCloudInfo.STRATEGY_NAIVE)
        # set_config() is only called for the advanced strategy, so this stays empty.
        self.assertEqual(len(info.matching_strategy_config), 0)

    def test_13_naive_without_motion_compensation(self):
        """is_motion_compensated = False leaves points at their pure TF-transformed position."""
        # Covers: combine_pointclouds() with motion compensation off, and the naive branch of
        # check_concat_status().
        base = next_base_stamp()
        stamps = self.publish_set(NAIVE, base)

        cloud = self.bus.one(NAIVE.output)
        info = self.bus.one(NAIVE.output_info)
        points = read_xyzirc(cloud)

        for idx, topic in enumerate(NAIVE.inputs):
            _, segment = self.segment_of(info, points, topic)
            expected = transformed_points(idx, motion_shift_x=0.0)
            for actual, want in zip(segment, expected):
                self.assertTrue(xyz_close(actual, want), f"{topic}: {actual} != {want}")
        self.assertEqual(cloud.header.stamp, seconds_to_stamp(min(stamps)))

        status = self.wait_for_diagnostic(NAIVE)
        self.assertEqual(status.level, DiagnosticStatus.OK)
        # check_concat_status() reports an arrival timestamp for naive, a window for advanced.
        self.assertIn("First pointcloud arrival timestamp", diag_keys(status))
        self.assertNotIn("Minimum reference timestamp", diag_keys(status))

    def test_14_naive_synchronized_clouds_in_output_frame(self):
        """keep_input_frame_in_synchronized_pointcloud = False publishes in the output frame."""
        # Covers: the synchronized-cloud branch of combine_pointclouds(), keep_input_frame off.
        base = next_base_stamp()
        stamps = self.publish_set(NAIVE, base)
        self.bus.one(NAIVE.output)
        oldest = seconds_to_stamp(min(stamps))

        for idx, topic in enumerate(NAIVE.sync_topics):
            cloud = self.bus.one(topic)
            self.assertEqual(cloud.header.frame_id, OUTPUT_FRAME, topic)
            self.assertEqual(cloud.header.stamp, oldest, topic)
            self.assertEqual(cloud.width, NUM_POINTS, topic)
            expected = transformed_points(idx, motion_shift_x=0.0)
            for actual, want in zip(read_xyzirc(cloud), expected):
                self.assertTrue(xyz_close(actual, want), f"{topic}: {actual} != {want}")

    # -- 15/16: publish_previous_but_late_pointcloud = False -----------------------------

    def test_15_late_cloud_is_dropped_but_info_is_still_published(self):
        """A cloud older than the last published one is withheld from ``output`` only."""
        # Covers: the drop-late branch of publish_clouds(), and check_concat_status().
        first = next_base_stamp()
        self.publish_set(STRICT, first)
        self.bus.one(STRICT.output)
        self.assertEqual(len(self.bus.received[STRICT.output]), 1)

        self.bus.clear()

        # One second earlier, i.e. inside rosbag_length (10 s), so this is treated as a
        # genuinely late cloud rather than a rosbag loop.
        late = first - 1.0
        self.publish_set(STRICT, late)

        info = self.bus.one(STRICT.output_info, timeout=3.0)
        # CHARACTERIZED QUIRK: output_info is published unconditionally, so a consumer sees
        # metadata for a concatenated cloud that never appeared on ``output``.
        self.assertEqual(info.header.stamp, seconds_to_stamp(late))
        self.assertTrue(info.concatenation_success)
        self.assertEqual(self.bus.received[STRICT.output], [])

        status = self.wait_for_diagnostic(STRICT)
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertEqual(
            status.message,
            "Concatenated pointcloud was dropped due to its timestamp is earlier "
            "than the latest published one",
        )
        # The diagnostic still describes the cloud that was dropped.
        self.assertEqual(diag_values(status)["Concatenated pointcloud timestamp"], f"{late:.9f}")

    def test_16_rosbag_loop_forces_publication(self):
        """A jump backwards larger than rosbag_length is treated as a replay, not a delay."""
        # Covers: the rosbag_length branch of publish_clouds().
        # test_15 left the instance's latest timestamp at its own `first`; go far enough
        # back that the gap exceeds rosbag_length.
        base = next_base_stamp()
        self.publish_set(STRICT, base)
        self.bus.one(STRICT.output)
        self.bus.clear()

        looped = base - (ROSBAG_LENGTH_STRICT + 5.0)
        self.publish_set(STRICT, looped)

        cloud = self.bus.one(STRICT.output, timeout=3.0)
        self.assertEqual(cloud.header.stamp, seconds_to_stamp(looped))
        self.assertEqual(cloud.width, NUM_POINTS * len(SENSOR_FRAMES))

        status = self.wait_for_diagnostic(STRICT)
        self.assertEqual(status.level, DiagnosticStatus.OK)
        # See test_06: an OK status always reports the literal "OK".
        self.assertEqual(status.message, "OK")

    # -- 17: odometry-driven motion compensation ----------------------------------------

    def test_17_odometry_motion_compensation(self):
        """input_twist_topic_type = "odom" compensates exactly like the twist input."""
        # Covers: odom_callback() -> process_odometry(), then correct_pointcloud_motion().
        base = next_base_stamp()
        self.odom_pub.publish(make_odometry(base))
        self.bus.spin(0.1)
        self.publish_set(ODOM, base)

        cloud = self.bus.one(ODOM.output)
        info = self.bus.one(ODOM.output_info)
        points = read_xyzirc(cloud)
        for idx, topic in enumerate(ODOM.inputs):
            _, segment = self.segment_of(info, points, topic)
            expected = transformed_points(idx, motion_shift_for(idx))
            for actual, want in zip(segment, expected):
                self.assertTrue(xyz_close(actual, want), f"{topic}: {actual} != {want}")


@launch_testing.post_shutdown_test()
class TestConcatenateNodeCharacterizationShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
