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

"""Offline Python interface to the point cloud concatenation core.

:class:`Concatenator` runs the same C++ ``CloudConcatenator`` as the concatenation node.

Time is supplied by the caller: every cloud comes with its arrival time (e.g. the rosbag record
timestamp), which drives the collector timeouts and, for the naive strategy, the matching. The
same clouds with the same arrival times always give the same frames.
"""

from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import List
from typing import NamedTuple
from typing import Optional
from typing import Sequence

from autoware_sensing_msgs.msg import ConcatenatedPointCloudInfo
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

from ._concatenate_pointclouds_pybind import FrameStatus
from ._concatenate_pointclouds_pybind import MotionCompensationStatus
from . import _concatenate_pointclouds_pybind as _ext

__all__ = [
    "Concatenator",
    "ConcatenatedFrame",
    "ConcatenationResult",
    "FrameStatus",
    "MotionCompensationStatus",
]


class ConcatenationResult(NamedTuple):
    """The concatenated cloud of one frame and its metadata.

    Attributes:
        concatenated_cloud: The concatenated cloud, in the output frame.
        concatenation_info: The ``ConcatenatedPointCloudInfo`` the node publishes with the cloud.
        transformed_clouds: The input clouds by topic, or ``None`` unless
            ``publish_synchronized_pointcloud`` is set.
        motion_compensation_status: Whether motion compensation ran, and if not, why.
        topic_to_original_stamp: Header stamp (seconds) of each contributing cloud, by topic.
        dropped_sources_missing_transform: Topic to sensor frame of the clouds left out because
            no transform to the output frame was set.
    """

    concatenated_cloud: PointCloud2
    concatenation_info: ConcatenatedPointCloudInfo
    transformed_clouds: Optional[Dict[str, PointCloud2]]
    motion_compensation_status: MotionCompensationStatus
    topic_to_original_stamp: Dict[str, float]
    dropped_sources_missing_transform: Dict[str, str]


@dataclass(frozen=True)
class ConcatenatedFrame:
    """One frame emitted by :class:`Concatenator`.

    Attributes:
        status: Why the collector closed: ``COMPLETE``, ``TIMEOUT`` or ``LATE``.
        result: The concatenated cloud and its metadata.
        reference_time: Reference timestamp (seconds) of the collector that produced the frame.
        noise_window: Half-width (seconds) of the collector's matching window around
            ``reference_time``; 0 for the ``naive`` strategy.
        first_arrival_time: Arrival time (seconds) of the cloud that opened the collector.
    """

    status: FrameStatus
    result: ConcatenationResult
    reference_time: float
    noise_window: float
    first_arrival_time: float
    # What build_diagnostics() needs (a few scalars and the input topics), not the C++ clouds.
    _diagnostics: _ext.FrameDiagnostics = field(repr=False, compare=False)

    def build_diagnostics(
        self,
        *,
        node_name: Optional[str] = None,
        diagnostic_name: Optional[str] = None,
        processing_time_ms: Optional[float] = None,
        now: Optional[float] = None,
        drop_previous_but_late: bool = False,
    ) -> DiagnosticStatus:
        """Build the ``DiagnosticStatus`` the node publishes for this frame.

        Args:
            node_name: Node name used in the status name. Defaults to the node's.
            diagnostic_name: Name of the diagnostic task. Defaults to the node's.
            processing_time_ms: Value of the ``"Processing time (ms)"`` entry, which is left out
                if ``None``.
            now: Current time (seconds) the latency entries are measured against. They are left
                out if ``None``.
            drop_previous_but_late: Report the frame as dropped for being older than the last
                published one, as the node does when ``publish_previous_but_late_pointcloud`` is
                off. Offline, the caller makes that comparison.

        Returns:
            The diagnostic status of this frame.
        """
        status = self._diagnostics.build(
            node_name, diagnostic_name, processing_time_ms, now, drop_previous_but_late
        )
        return deserialize_message(status, DiagnosticStatus)


def _registered(member):
    # pybind11 returns a new enum object for every value; hand out the registered member instead,
    # so that `frame.status is FrameStatus.COMPLETE` holds as it does for a Python enum.
    return getattr(type(member), member.name)


def _check_type(message, expected):
    # The CDR bytes carry no type, so a wrong message type would be silently reinterpreted.
    if not isinstance(message, expected):
        raise TypeError(f"expected {expected.__name__}, got {type(message).__name__}")


def _wrap_frame(core: _ext.CoreFrame) -> ConcatenatedFrame:
    transformed_clouds = core.transformed_clouds
    result = ConcatenationResult(
        concatenated_cloud=deserialize_message(core.concatenated_cloud, PointCloud2),
        concatenation_info=deserialize_message(core.concatenation_info, ConcatenatedPointCloudInfo),
        transformed_clouds=(
            None
            if transformed_clouds is None
            else {
                topic: deserialize_message(data, PointCloud2)
                for topic, data in transformed_clouds.items()
            }
        ),
        motion_compensation_status=_registered(core.motion_compensation_status),
        topic_to_original_stamp=core.topic_to_original_stamp,
        dropped_sources_missing_transform=core.dropped_sources_missing_transform,
    )
    return ConcatenatedFrame(
        status=_registered(core.status),
        result=result,
        reference_time=core.reference_time,
        noise_window=core.noise_window,
        first_arrival_time=core.first_arrival_time,
        _diagnostics=core.diagnostics,
    )


class Concatenator:
    """Concatenates point clouds offline, with the node's matching, timeout and combine logic.

    Feed clouds with :meth:`process_cloud` in arrival order. A frame is emitted when every input
    topic has contributed to a collector (``COMPLETE``), or once the arrival clock reaches the
    collector's creation plus ``timeout_sec`` (``TIMEOUT``). Call :meth:`flush` at the end of the
    stream for the collectors still open.

    The defaults keep every collector open until it completes or times out. To reproduce the node
    exactly, pass its ``max_open_collectors`` and, for
    ``publish_previous_but_late_pointcloud: false``, ``reject_late_clouds=True``.
    """

    def __init__(
        self,
        input_topics: Sequence[str],
        output_frame: str,
        timeout_sec: float,
        is_motion_compensated: bool = True,
        publish_synchronized_pointcloud: bool = False,
        keep_input_frame_in_synchronized_pointcloud: bool = False,
        matching_strategy: str = "naive",
        lidar_timestamp_offsets: Optional[Sequence[float]] = None,
        lidar_timestamp_noise_window: Optional[Sequence[float]] = None,
        max_open_collectors: int = 0,
        reject_late_clouds: bool = False,
        replay_jump_sec: float = 10.0,
    ) -> None:
        """Create a concatenator with no open collector, no transform and no velocity.

        Args:
            input_topics: Topics of the clouds to concatenate, without duplicates.
            output_frame: Frame of the concatenated cloud.
            timeout_sec: Time after its first cloud's arrival at which an incomplete collector is
                emitted as ``TIMEOUT``.
            is_motion_compensated: Compensate each cloud for the ego motion between its stamp and
                the oldest stamp of the frame.
            publish_synchronized_pointcloud: Also return the input clouds, in
                ``transformed_clouds``.
            keep_input_frame_in_synchronized_pointcloud: Keep those clouds in their sensor frame
                instead of the output frame.
            matching_strategy: ``"naive"`` or ``"advanced"``, as in the node.
            lidar_timestamp_offsets: Timestamp offset (seconds) of each input topic, in
                ``input_topics`` order, as in the node's parameter. Required for ``"advanced"``.
            lidar_timestamp_noise_window: Timestamp noise window (seconds) of each input topic,
                in ``input_topics`` order, as in the node's parameter. Required for
                ``"advanced"``.
            max_open_collectors: Number of collectors that may be open at once; opening another
                one discards the oldest without emitting it. 0 means no limit.
            reject_late_clouds: Return a cloud that arrives after its frame was emitted as a
                one-cloud ``LATE`` frame, instead of opening a collector for it.
            replay_jump_sec: With ``reject_late_clouds``, a cloud older than the last emitted
                frame by more than this is a replay restart, not a late cloud (the node's
                ``rosbag_length``).

        Raises:
            ValueError: If ``input_topics`` has duplicates, ``matching_strategy`` is unknown, or
                the ``"advanced"`` offsets and noise windows are missing or do not have one value
                per input topic.
        """
        self._impl = _ext.Concatenator(
            list(input_topics),
            output_frame,
            timeout_sec,
            is_motion_compensated,
            publish_synchronized_pointcloud,
            keep_input_frame_in_synchronized_pointcloud,
            matching_strategy,
            lidar_timestamp_offsets,
            lidar_timestamp_noise_window,
            max_open_collectors,
            reject_late_clouds,
            replay_jump_sec,
        )

    def set_transform(self, sensor_to_output_frame: TransformStamped) -> None:
        """Set the transform from one sensor frame into the output frame.

        Clouds in a sensor frame without a transform are left out of the frame and reported in
        ``dropped_sources_missing_transform``.

        Args:
            sensor_to_output_frame: Transform with ``header.frame_id`` = output frame and
                ``child_frame_id`` = sensor frame, as tf2 returns for
                ``lookup_transform(output_frame, sensor_frame, ...)``.

        Raises:
            TypeError: If ``sensor_to_output_frame`` is not a ``TransformStamped``.
            ValueError: If the transform does not have that form.
        """
        _check_type(sensor_to_output_frame, TransformStamped)
        self._impl.set_transform(serialize_message(sensor_to_output_frame))

    def process_twist(self, twist: TwistWithCovarianceStamped) -> None:
        """Queue a velocity sample for motion compensation.

        Args:
            twist: Vehicle twist. Only the linear x and angular z velocities are used.

        Raises:
            TypeError: If ``twist`` is not a ``TwistWithCovarianceStamped``.
        """
        _check_type(twist, TwistWithCovarianceStamped)
        self._impl.process_twist(serialize_message(twist))

    def process_odometry(self, odometry: Odometry) -> None:
        """Queue a velocity sample for motion compensation, from odometry.

        Takes the place of :meth:`process_twist` for an ``input_twist_topic_type: odom`` setup.

        Args:
            odometry: Vehicle odometry. Only its twist's linear x and angular z velocities are
                used.

        Raises:
            TypeError: If ``odometry`` is not an ``Odometry``.
        """
        _check_type(odometry, Odometry)
        self._impl.process_odometry(serialize_message(odometry))

    def process_cloud(
        self, topic: str, cloud: PointCloud2, arrival_time: float
    ) -> List[ConcatenatedFrame]:
        """Add a cloud to its collector and return the frames this resolves.

        Args:
            topic: Input topic the cloud was received on.
            cloud: Cloud with the PointXYZIRC fields.
            arrival_time: Time (seconds) the cloud was received, e.g. its rosbag record time. It
                takes the place of the node's clock.

        Returns:
            The frames resolved by this cloud: usually none or one, but several collectors can
            resolve at once.

        Raises:
            TypeError: If ``cloud`` is not a ``PointCloud2``.
            ValueError: If ``topic`` is not one of the input topics, or the cloud lacks a field
                of the output point type (``x``, ``y``, ``z``, ``intensity``, ``return_type``,
                ``channel``).
        """
        _check_type(cloud, PointCloud2)
        frames = self._impl.process_cloud(topic, serialize_message(cloud), arrival_time)
        return [_wrap_frame(frame) for frame in frames]

    def close_expired_collectors(self, now: float) -> List[ConcatenatedFrame]:
        """Emit the collectors whose timeout has expired, without adding a cloud.

        Args:
            now: Current time (seconds), on the same clock as the arrival times.

        Returns:
            One ``TIMEOUT`` frame per expired collector.
        """
        return [_wrap_frame(frame) for frame in self._impl.close_expired_collectors(now)]

    def flush(self) -> List[ConcatenatedFrame]:
        """Emit every collector still open. Call it at the end of the stream.

        Returns:
            One ``TIMEOUT`` frame per open collector.
        """
        return [_wrap_frame(frame) for frame in self._impl.flush()]
