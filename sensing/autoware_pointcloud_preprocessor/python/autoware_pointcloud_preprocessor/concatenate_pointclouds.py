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

The public API is :class:`Concatenator`: feed it clouds one at a time with
:meth:`Concatenator.process_cloud` (in arrival order) and it emits concatenated frames when a
collector is complete or times out. All concatenation logic -- matching, collector lifecycle,
timeout, combining, and diagnostics -- runs in the same C++ core the ROS node builds on; this
module only converts between rclpy message objects and the CDR bytes the binding exchanges, so
results are reproducible and no wall clock is involved.
"""

from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional

from autoware_sensing_msgs.msg import ConcatenatedPointCloudInfo
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

from . import _concatenate_pointclouds_pybind as _ext

__all__ = [
    "FrameStatus",
    "ConcatenationResult",
    "ConcatenatedFrame",
    "Concatenator",
    "build_diagnostics",
]

# Default name used for the offline DiagnosticStatus (the online node uses its fully-qualified
# name).
_DEFAULT_DIAGNOSTIC_NODE_NAME = "concatenate_data_synchronizer"


class FrameStatus:
    """Why a frame was emitted by :meth:`Concatenator.process_cloud` (mirrors the node's states)."""

    COMPLETE = "complete"  # every input topic contributed a cloud
    TIMEOUT = "timeout"  # the collector was closed before all topics arrived


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


@dataclass(frozen=True)
class ConcatenatedFrame:
    """One concatenated frame emitted by the concatenator, tagged with why it was emitted.

    ``reference_time`` / ``noise_window`` / ``first_arrival_time`` describe the matching context of
    the collector that produced this frame: for ``advanced``, ``reference_time +/- noise_window`` is
    the matching window; ``first_arrival_time`` is the arrival time of the cloud that opened the
    collector (the node's naive "First pointcloud arrival timestamp").
    """

    status: str  # one of FrameStatus
    result: ConcatenationResult
    reference_time: float
    noise_window: float
    first_arrival_time: float
    # The C++ frame this wraps; diagnostics are built from it on the C++ side.
    _core: object = field(default=None, repr=False, compare=False)


def _wrap_frame(core) -> ConcatenatedFrame:
    """Package one C++ frame (CDR-bytes fields) as rclpy message objects."""
    concatenated_cloud = (
        deserialize_message(core.concatenated_cloud, PointCloud2)
        if core.concatenated_cloud is not None
        else None
    )
    concatenation_info = (
        deserialize_message(core.concatenation_info, ConcatenatedPointCloudInfo)
        if core.concatenation_info is not None
        else None
    )
    transformed_clouds = (
        {
            topic: deserialize_message(data, PointCloud2)
            for topic, data in core.transformed_clouds.items()
        }
        if core.transformed_clouds is not None
        else None
    )
    result = ConcatenationResult(
        concatenated_cloud=concatenated_cloud,
        concatenation_info=concatenation_info,
        transformed_clouds=transformed_clouds,
        no_twist_available=core.no_twist_available,
        twist_time_gap_too_large=core.twist_time_gap_too_large,
        topic_to_original_stamp=dict(core.topic_to_original_stamp),
        dropped_frames_missing_transform=list(core.dropped_frames_missing_transform),
    )
    return ConcatenatedFrame(
        status=core.status,
        result=result,
        reference_time=core.reference_time,
        noise_window=core.noise_window,
        first_arrival_time=core.first_arrival_time,
        _core=core,
    )


class Concatenator:
    """Stateful, arrival-driven offline concatenator.

    Feed clouds with :meth:`process_cloud` in arrival order, passing each cloud's ``arrival_time``
    (e.g. the rosbag record timestamp). Matching, timeout handling, combining, and diagnostics all
    run in the **same C++ core the ROS node uses**; a frame is emitted when every input topic has
    contributed to a collector (``COMPLETE``), or when the arrival clock advances more than
    ``timeout_sec`` past the collector's creation time (``TIMEOUT``) -- the offline,
    ROS-timer-free replacement for the node's per-collector wall-clock timeout.

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
        self._impl = _ext.Concatenator(
            input_topics,
            output_frame,
            float(timeout_sec),
            is_motion_compensated,
            publish_synchronized_pointcloud,
            keep_input_frame_in_synchronized_pointcloud,
            matching_strategy,
            lidar_timestamp_offsets,
            lidar_timestamp_noise_window,
        )
        for transform in tf_static.values():
            self._impl.set_transform(serialize_message(transform))
        self._input_topics = list(input_topics)

    def process_twist(self, twist: TwistWithCovarianceStamped) -> None:
        self._impl.process_twist(serialize_message(twist))

    def process_odometry(self, odometry: Odometry) -> None:
        self._impl.process_odometry(serialize_message(odometry))

    def process_cloud(
        self, topic: str, cloud: PointCloud2, arrival_time: float
    ) -> List[ConcatenatedFrame]:
        """Add ``cloud`` for ``topic`` (received at ``arrival_time``) and return emitted frames.

        The list is usually empty (still collecting) or has one element (a collector completed or
        an old collector timed out), but may contain several when more than one resolves at once.
        Each entry's ``status`` is :data:`FrameStatus.COMPLETE` or
        :data:`FrameStatus.TIMEOUT`.

        ``arrival_time`` (seconds) is the time the cloud was *received* -- e.g. the rosbag record
        timestamp (``messages.timestamp``), the offline analog of the wall-clock arrival the online
        node sees. It drives both the ``naive`` matching window and the per-collector timeout, so
        the offline collector lifecycle reproduces the node's wall-clock-timer behavior. **Feed
        clouds in arrival order.**
        """
        frames = self._impl.process_cloud(topic, serialize_message(cloud), float(arrival_time))
        return [_wrap_frame(frame) for frame in frames]

    def close_expired_collectors(self, now: float) -> List[ConcatenatedFrame]:
        """Close every collector whose timeout expired at ``now``, without adding a cloud."""
        return [_wrap_frame(frame) for frame in self._impl.close_expired_collectors(float(now))]

    def flush(self) -> List[ConcatenatedFrame]:
        """Emit every still-open collector as a timeout (call once the stream is exhausted)."""
        return [_wrap_frame(frame) for frame in self._impl.flush()]

    def build_diagnostics(self, frame: ConcatenatedFrame, **kwargs) -> DiagnosticStatus:
        """Build the node-equivalent :class:`DiagnosticStatus` for ``frame``.

        Convenience wrapper around :func:`build_diagnostics` that supplies this concatenator's
        ``input_topics`` (so the per-topic entries appear in the configured order). Extra keyword
        arguments (``node_name``, ``diagnostic_name``, ``processing_time_ms``, ``now``,
        ``drop_previous_but_late``) are forwarded unchanged.
        """
        return build_diagnostics(frame, self._input_topics, **kwargs)


def build_diagnostics(
    frame: ConcatenatedFrame,
    input_topics: List[str],
    *,
    node_name: str = _DEFAULT_DIAGNOSTIC_NODE_NAME,
    diagnostic_name: Optional[str] = None,
    processing_time_ms: Optional[float] = None,
    now: Optional[float] = None,
    drop_previous_but_late: bool = False,
) -> DiagnosticStatus:
    """Build a :class:`DiagnosticStatus` mirroring the concatenate node's ``check_concat_status``.

    The status is built by the same C++ code path offline and online-equivalent: the same key/value
    entries (same order, same string formatting), level, and message the node publishes on
    ``/diagnostics`` for a single concatenated frame. Pass a frame returned by
    :meth:`Concatenator.process_cloud` or :meth:`Concatenator.flush`; ``input_topics`` fixes the
    order of the per-topic entries (use the node's ``input_topics``).

    Two sets of entries have no offline meaning unless you supply the data, so they are opt-in:

    * ``processing_time_ms`` -- if given, added as ``"Processing time (ms)"``.
    * ``now`` (seconds) -- a publish-time analog (e.g. the frame's arrival/record timestamp). If
      given, ``"Pipeline latency (ms)"`` and the per-topic ``"Latency (ms): <topic>"`` are computed
      as ``(now - original_stamp) * 1000``, exactly as the node does against its wall clock.

    ``drop_previous_but_late`` reproduces the node's out-of-order-republish guard; offline batches
    normally leave it ``False``.

    Note: like the node, ``"Concatenated: <topic>"`` reflects whether the topic *contributed a
    cloud* (it is in ``topic_to_original_stamp``), not whether that cloud survived -- a cloud
    dropped for a missing transform still shows ``True`` here. The finer per-source verdict
    (OK / TIMEOUT / INVALID) lives in ``frame.result.concatenation_info.source_info``.
    """
    if frame._core is None:
        raise ValueError("frame was not produced by a Concatenator")
    status_bytes = frame._core.build_diagnostics(
        input_topics,
        node_name,
        diagnostic_name,
        processing_time_ms,
        now,
        drop_previous_but_late,
    )
    return deserialize_message(status_bytes, DiagnosticStatus)
