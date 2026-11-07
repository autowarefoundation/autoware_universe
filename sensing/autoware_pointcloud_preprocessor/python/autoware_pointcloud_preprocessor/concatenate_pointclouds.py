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

:class:`Concatenator` runs the same C++ ``CloudConcatenator`` as the concatenation node; this module
only converts between rclpy messages and the CDR bytes the extension exchanges.

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
    """The concatenated cloud of one frame and its metadata."""

    concatenated_cloud: PointCloud2
    concatenation_info: ConcatenatedPointCloudInfo
    # Per-topic clouds; None unless publish_synchronized_pointcloud is set.
    transformed_clouds: Optional[Dict[str, PointCloud2]]
    motion_compensation_status: MotionCompensationStatus
    topic_to_original_stamp: Dict[str, float]
    # Topic -> sensor frame of the clouds left out because no transform to the output frame was set.
    dropped_sources_missing_transform: Dict[str, str]


@dataclass(frozen=True)
class ConcatenatedFrame:
    """One frame emitted by :class:`Concatenator`.

    ``reference_time`` / ``noise_window`` / ``first_arrival_time`` describe the collector that
    produced it: for ``advanced``, ``reference_time +/- noise_window`` is its matching window;
    ``first_arrival_time`` is the arrival time of the cloud that opened it.
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
        """Build the :class:`DiagnosticStatus` the node publishes for this frame.

        ``node_name`` defaults to the node's. ``"Processing time (ms)"`` is added only when
        ``processing_time_ms`` is given, and the latency entries only when ``now`` (seconds) is
        given, since neither has an offline meaning otherwise. ``drop_previous_but_late`` reports
        the frame as dropped for being older than the last published one, as the node does when
        ``publish_previous_but_late_pointcloud`` is off (that check is the caller's offline).
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
    collector's creation plus ``timeout_sec`` (``TIMEOUT``); call :meth:`flush` at the end of the
    stream for the collectors still open.

    For the ``advanced`` strategy, give ``lidar_timestamp_offsets`` and
    ``lidar_timestamp_noise_window`` (seconds, one per input topic, in ``input_topics`` order),
    the same values the node reads from its parameters.

    The defaults keep every collector open until it completes or times out, as the core does
    offline. To reproduce the node exactly, also pass its ``max_open_collectors`` and, for
    ``publish_previous_but_late_pointcloud: false``, ``reject_late_clouds=True``: a cloud arriving
    after its frame was emitted is then returned at once as a one-cloud ``LATE`` frame (which the
    node drops) instead of opening a collector of its own. A cloud more than ``replay_jump_sec``
    (the node's ``rosbag_length``, default 10 s) behind the last emitted frame is a replay restart,
    not a late cloud.
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
        """Set the extrinsic of one sensor frame.

        ``header.frame_id`` must be the output frame and ``child_frame_id`` the sensor frame, i.e.
        the transform tf2 returns for ``lookup_transform(output_frame, sensor_frame, ...)``. Clouds
        in a sensor frame without a transform are left out of the frame and reported in
        ``dropped_sources_missing_transform``.

        :raises ValueError: if the transform does not have that form.
        """
        _check_type(sensor_to_output_frame, TransformStamped)
        self._impl.set_transform(serialize_message(sensor_to_output_frame))

    def process_twist(self, twist: TwistWithCovarianceStamped) -> None:
        _check_type(twist, TwistWithCovarianceStamped)
        self._impl.process_twist(serialize_message(twist))

    def process_odometry(self, odometry: Odometry) -> None:
        _check_type(odometry, Odometry)
        self._impl.process_odometry(serialize_message(odometry))

    def process_cloud(
        self, topic: str, cloud: PointCloud2, arrival_time: float
    ) -> List[ConcatenatedFrame]:
        """Add ``cloud``, received on ``topic`` at ``arrival_time`` (seconds).

        Returns the frames this resolves: usually none or one, but several collectors can resolve
        at once.

        :raises ValueError: if ``topic`` is not one of the input topics, or if the cloud's layout is
            inconsistent (``data`` size vs ``width``/``height``/``point_step``, field offsets,
            missing or non-FLOAT32 ``x``/``y``/``z``).
        """
        _check_type(cloud, PointCloud2)
        frames = self._impl.process_cloud(topic, serialize_message(cloud), arrival_time)
        return [_wrap_frame(frame) for frame in frames]

    def close_expired_collectors(self, now: float) -> List[ConcatenatedFrame]:
        """Return the collectors whose timeout has expired at ``now``, without adding a cloud."""
        return [_wrap_frame(frame) for frame in self._impl.close_expired_collectors(now)]

    def flush(self) -> List[ConcatenatedFrame]:
        """Return every collector still open, as timed out (call at the end of the stream)."""
        return [_wrap_frame(frame) for frame in self._impl.flush()]
