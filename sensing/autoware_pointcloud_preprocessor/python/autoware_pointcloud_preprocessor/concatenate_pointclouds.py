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

"""Deterministic, offline Python interface to the point cloud concatenation core.

Two layers are provided:

* :class:`CombineCloudHandler` -- a thin wrapper over the C++ core. It combines a *group* of clouds
  (one per input topic) that have already been collected, transforms them into the output frame
  using the injected extrinsics, optionally motion-compensates them, and concatenates them. ROS
  messages are passed as their rclpy objects (CDR-serialized under the hood, so no wall clock is
  used and results are reproducible).

* :class:`Concatenator` -- a stateful collector on top of the handler. Feed it clouds one at a time
  with :meth:`Concatenator.process_cloud`; it groups them and emits concatenated frames when a group
  is complete or times out. It supports the same ``naive`` and ``advanced`` matching strategies as
  the concatenation node, but is driven entirely by message timestamps (no ROS timer), so it runs
  offline and deterministically.
"""

from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional

from autoware_sensing_msgs.msg import ConcatenatedPointCloudInfo
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

from . import _concatenate_pointclouds_pybind as _ext

__all__ = [
    "CombineCloudHandler",
    "Concatenator",
    "ConcatenationResult",
    "ConcatenatedFrame",
    "CollectorStatus",
]


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CollectorStatus:
    """Why a frame was emitted by :meth:`Concatenator.process_cloud` (mirrors the node's states)."""

    COMPLETE = "complete"  # every input topic contributed a cloud
    TIMEOUT = "timeout"  # the group was closed before all topics arrived


class ConcatenationResult(NamedTuple):
    """A concatenated point cloud and its metadata (the output of a single combine).

    ``transformed_clouds`` is the per-topic, output-/sensor-frame cloud map (only populated when
    ``publish_synchronized_pointcloud`` is enabled), otherwise ``None``.
    """

    concatenated_cloud: PointCloud2
    concatenation_info: Optional[ConcatenatedPointCloudInfo]
    transformed_clouds: Optional[Dict[str, PointCloud2]]
    no_twist_available: bool
    twist_time_gap_too_large: bool
    topic_to_original_stamp: Dict[str, float]
    # Sensor frames dropped from the concatenation because no extrinsic to the output frame was
    # provided for them (empty in the normal case where every frame has a transform).
    dropped_frames_missing_transform: List[str]


class ConcatenatedFrame(NamedTuple):
    """A concatenated group emitted by the collector, tagged with why it was emitted."""

    status: str  # one of CollectorStatus
    result: ConcatenationResult


class CombineCloudHandler:
    """Combine an already-collected group of clouds into one concatenated cloud.

    Provide the sensor extrinsics once via :meth:`set_transform` (``header.frame_id`` = output
    frame, ``child_frame_id`` = sensor frame; rotations must be unit quaternions, as TF provides).
    Feed twist/odometry in timestamp order for motion compensation.
    """

    def __init__(
        self,
        input_topics: List[str],
        output_frame: str,
        is_motion_compensated: bool = True,
        publish_synchronized_pointcloud: bool = False,
        keep_input_frame_in_synchronized_pointcloud: bool = False,
        matching_strategy: str = "naive",
    ) -> None:
        self._impl = _ext.CombineCloudHandler(
            input_topics,
            output_frame,
            is_motion_compensated,
            publish_synchronized_pointcloud,
            keep_input_frame_in_synchronized_pointcloud,
            matching_strategy,
        )

    def set_transform(self, sensor_to_output_frame: TransformStamped) -> None:
        self._impl.set_transform(serialize_message(sensor_to_output_frame))

    def process_twist(self, twist: TwistWithCovarianceStamped) -> None:
        self._impl.process_twist(serialize_message(twist))

    def process_odometry(self, odometry: Odometry) -> None:
        self._impl.process_odometry(serialize_message(odometry))

    def combine_pointclouds(
        self,
        topic_to_cloud: Dict[str, PointCloud2],
        reference_timestamp: Optional[float] = None,
        noise_window: Optional[float] = None,
    ) -> ConcatenationResult:
        """Concatenate ``topic_to_cloud`` (topic -> :class:`PointCloud2`) and return the result.

        When ``reference_timestamp`` is given (advanced strategy), the matching window is recorded in
        the ``ConcatenatedPointCloudInfo``.
        """
        topic_to_cloud_bytes = {
            topic: serialize_message(cloud) for topic, cloud in topic_to_cloud.items()
        }
        (
            concatenated_bytes,
            info_bytes,
            transformed_bytes,
            status,
            topic_to_original_stamp,
            dropped_frames_missing_transform,
        ) = self._impl.combine_pointclouds(
            topic_to_cloud_bytes, reference_timestamp, noise_window
        )

        concatenated_cloud = (
            deserialize_message(concatenated_bytes, PointCloud2)
            if concatenated_bytes is not None
            else None
        )
        concatenation_info = (
            deserialize_message(info_bytes, ConcatenatedPointCloudInfo)
            if info_bytes is not None
            else None
        )
        transformed_clouds = (
            {topic: deserialize_message(b, PointCloud2) for topic, b in transformed_bytes.items()}
            if transformed_bytes is not None
            else None
        )
        return ConcatenationResult(
            concatenated_cloud=concatenated_cloud,
            concatenation_info=concatenation_info,
            transformed_clouds=transformed_clouds,
            no_twist_available=status.no_twist_available,
            twist_time_gap_too_large=status.twist_time_gap_too_large,
            topic_to_original_stamp=dict(topic_to_original_stamp),
            dropped_frames_missing_transform=list(dropped_frames_missing_transform),
        )


class _Collector:
    """One in-progress group of clouds (the offline counterpart of CloudCollector)."""

    def __init__(self, reference_time: float, noise_window: float, creation_arrival: float) -> None:
        self.topic_to_cloud: Dict[str, PointCloud2] = {}
        # reference_time is the (offset-corrected, for advanced) timestamp of the cloud that created
        # this collector; it is the reference for the matching window.
        self.reference_time = reference_time
        self.noise_window = noise_window
        # creation_arrival is the arrival time of the cloud that created this collector; it starts
        # the timeout, mirroring the node's per-collector wall-clock timer (which starts on the first
        # cloud and fires timeout_sec later in arrival time).
        self.creation_arrival = creation_arrival


class Concatenator:
    """Stateful, arrival-driven offline concatenator.

    Feed clouds with :meth:`process_cloud` in arrival order, passing each cloud's ``arrival_time``
    (e.g. the rosbag record timestamp). Clouds are grouped using the selected matching strategy
    (``naive`` or ``advanced``); the matching itself is delegated to the **same C++ core the ROS node
    uses** (via pybind), so the online and offline paths share one implementation. A group is emitted
    when every input topic has contributed (``COMPLETE``), or when the arrival clock advances more
    than ``timeout_sec`` past the group's creation time (``TIMEOUT``) -- the offline, ROS-timer-free
    replacement for the node's per-collector wall-clock timeout. The heavy combine work (transform,
    motion compensation, concatenation, info) is done by the C++ core too.

    For the ``advanced`` strategy, provide ``lidar_timestamp_offsets`` and
    ``lidar_timestamp_noise_window`` (one per input topic, in seconds, in the same order as
    ``input_topics``) -- these are the same values the node reads from its parameters.
    """

    def __init__(
        self,
        input_topics: List[str],
        output_frame: str,
        tf_static: Dict[str, TransformStamped],
        timeout_sec: float,
        is_motion_compensated: bool = True,
        publish_synchronized_pointcloud: bool = False,
        keep_input_frame_in_synchronized_pointcloud: bool = False,
        matching_strategy: str = "naive",
        lidar_timestamp_offsets: Optional[List[float]] = None,
        lidar_timestamp_noise_window: Optional[List[float]] = None,
    ) -> None:
        self._handler = CombineCloudHandler(
            input_topics,
            output_frame,
            is_motion_compensated,
            publish_synchronized_pointcloud,
            keep_input_frame_in_synchronized_pointcloud,
            matching_strategy,
        )
        for transform in tf_static.values():
            self._handler.set_transform(transform)

        self._input_topics = list(input_topics)
        self._timeout_sec = timeout_sec
        self._collectors: List[_Collector] = []
        self._is_advanced = matching_strategy == "advanced"

        if matching_strategy == "advanced":
            if lidar_timestamp_offsets is None or lidar_timestamp_noise_window is None:
                raise ValueError(
                    "advanced matching requires lidar_timestamp_offsets and "
                    "lidar_timestamp_noise_window (one per input topic)"
                )
            # The C++ core validates the sizes against the topics.
            self._strategy = _ext.MatchingPolicy.make_advanced(
                self._input_topics, list(lidar_timestamp_offsets), list(lidar_timestamp_noise_window)
            )
        elif matching_strategy == "naive":
            self._strategy = _ext.MatchingPolicy.make_naive()
        else:
            raise ValueError(f"unknown matching_strategy: {matching_strategy}")

    def process_twist(self, twist: TwistWithCovarianceStamped) -> None:
        self._handler.process_twist(twist)

    def process_odometry(self, odometry: Odometry) -> None:
        self._handler.process_odometry(odometry)

    def _emit(self, collector: _Collector, status: str) -> ConcatenatedFrame:
        # Only the advanced strategy records a reference window in the info message.
        reference_timestamp = collector.reference_time if self._is_advanced else None
        noise_window = collector.noise_window if self._is_advanced else None
        result = self._handler.combine_pointclouds(
            collector.topic_to_cloud, reference_timestamp, noise_window
        )
        return ConcatenatedFrame(status=status, result=result)

    def process_cloud(
        self, topic: str, cloud: PointCloud2, arrival_time: float
    ) -> List[ConcatenatedFrame]:
        """Add ``cloud`` for ``topic`` (received at ``arrival_time``) and return any emitted frames.

        The list is usually empty (still collecting) or has one element (a group completed or an old
        group timed out), but may contain several when more than one group resolves at once. Each
        entry's ``status`` is :data:`CollectorStatus.COMPLETE` or :data:`CollectorStatus.TIMEOUT`.

        ``arrival_time`` (seconds) is the time the cloud was *received* -- e.g. the rosbag record
        timestamp (``messages.timestamp``), the offline analog of the wall-clock arrival the online
        node sees. It drives both the ``naive`` matching window and the per-collector timeout, so the
        offline grouping reproduces the node's wall-clock-timer behavior. **Feed clouds in arrival
        order.**
        """
        if topic not in self._input_topics:
            raise ValueError(
                f"unknown topic {topic!r}; expected one of {self._input_topics}"
            )
        stamp = _stamp_to_seconds(cloud.header.stamp)
        arrival = float(arrival_time)
        params = _ext.IncomingCloudInfo(topic, stamp, arrival)
        outputs: List[ConcatenatedFrame] = []

        # The reference the matching core would assign a new collector for this cloud. For advanced it
        # is stamp - offset[topic]; for naive it is the arrival time. It defines the matching window.
        new_reference = self._strategy.reference_for(params)

        # (1) Timeout: close any group whose timer has expired in arrival time (it can no longer
        # receive more clouds). This replaces the node's per-collector wall-clock timer.
        stale = [
            c for c in self._collectors if c.creation_arrival < arrival - self._timeout_sec
        ]
        self._collectors = [c for c in self._collectors if c not in stale]
        for collector in sorted(stale, key=lambda c: c.creation_arrival):
            outputs.append(self._emit(collector, CollectorStatus.TIMEOUT))

        # (2) Match against the open collectors (delegated to the shared C++ matching core).
        states = [
            _ext.CandidateCollectorState(c.reference_time, c.noise_window, topic in c.topic_to_cloud)
            for c in self._collectors
        ]
        matched_index = self._strategy.match(states, params)
        collector = self._collectors[matched_index] if matched_index is not None else None

        # (3) No match -> start a new collector (matching window from the core-provided reference,
        # timeout starting at this cloud's arrival).
        if collector is None:
            collector = _Collector(
                new_reference.reference_time, new_reference.noise_window, arrival
            )
            self._collectors.append(collector)

        # (4) Add the cloud; emit immediately if the group is now complete.
        collector.topic_to_cloud[topic] = cloud
        if len(collector.topic_to_cloud) == len(self._input_topics):
            self._collectors.remove(collector)
            outputs.append(self._emit(collector, CollectorStatus.COMPLETE))

        return outputs

    def flush(self) -> List[ConcatenatedFrame]:
        """Emit every still-open group as a timeout (call once the stream is exhausted)."""
        outputs = [
            self._emit(collector, CollectorStatus.TIMEOUT)
            for collector in sorted(self._collectors, key=lambda c: c.creation_arrival)
        ]
        self._collectors = []
        return outputs
