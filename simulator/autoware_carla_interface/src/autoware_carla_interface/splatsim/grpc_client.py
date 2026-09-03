# Copyright 2024 Tier IV, Inc.
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

"""gRPC client wrapper for the splatsim RenderingService."""

from __future__ import annotations

import threading
from typing import Callable
from typing import Iterator

from autoware_carla_interface.splatsim.proto import rendering_service_pb2 as pb2
from autoware_carla_interface.splatsim.proto import rendering_service_pb2_grpc as pb2_grpc
import grpc
import rclpy

_rlog = rclpy.logging.get_logger("splatsim_grpc_client")


class _PoseStream:
    """Single-slot latest-pose client stream shared by the camera and LiDAR paths.

    No client-side buffering — only the latest pose is kept.  The
    generator yields only when a new pose arrives, so the gRPC
    transport never accumulates a stale backlog.
    """

    def __init__(self, name: str, rpc: Callable) -> None:
        self._name = name
        self._rpc = rpc
        self._latest_pose = None
        self._pose_lock = threading.Lock()
        self._new_pose = threading.Event()
        self._closed = False
        self._thread: threading.Thread | None = None
        self._result: pb2.StreamSummary | None = None
        self._error: Exception | None = None
        self._send_count: int = 0

    def start(self) -> None:
        """Open the client-streaming RPC in a background thread."""
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def send(self, msg) -> None:
        """Store the latest pose message for the background stream."""
        if self._thread is not None and not self._thread.is_alive():
            if self._error is not None:
                _rlog.error(f"{self._name} gRPC stream died: {self._error}")
            else:
                _rlog.error(f"{self._name} gRPC stream thread ended unexpectedly")
            return

        with self._pose_lock:
            self._latest_pose = msg
        self._new_pose.set()

        self._send_count += 1
        if self._send_count <= 10 or self._send_count % 50 == 0:
            p = msg.pose.position
            _rlog.warn(
                f"{self._name} gRPC send #{self._send_count}: "
                f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f})"
            )

    def close(self) -> pb2.StreamSummary | None:
        """Signal end-of-stream and wait for the background thread."""
        self._closed = True
        self._new_pose.set()  # wake generator
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        if self._error is not None:
            _rlog.error(f"{self._name} stream ended with error: {self._error}")
        return self._result

    def _pose_generator(self) -> Iterator:
        """Yield only the latest pose, blocking until a new one arrives."""
        yield_count = 0
        while True:
            self._new_pose.wait()
            if self._closed:
                _rlog.warn(f"{self._name} generator done after {yield_count} yields")
                return
            self._new_pose.clear()

            with self._pose_lock:
                msg = self._latest_pose
            if msg is None:
                continue

            yield_count += 1
            if yield_count <= 10 or yield_count % 50 == 0:
                p = msg.pose.position
                _rlog.warn(
                    f"{self._name} generator yield #{yield_count}: "
                    f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                    f"t={msg.stamp.sec}.{msg.stamp.nanosec:09d}"
                )
            yield msg

    def _worker(self) -> None:
        """Background thread that drives the client-streaming RPC."""
        try:
            self._result = self._rpc(self._pose_generator())
            _rlog.warn(
                f"{self._name} stream finished: "
                f"rendered={self._result.frames_rendered}, "
                f"received={self._result.poses_received}"
            )
        except Exception as exc:
            self._error = exc
            _rlog.error(f"{self._name} stream RPC failed: {exc}")


class SplatSimGrpcClient:
    """Thread-safe gRPC client for the splatsim ``RenderingService``.

    The camera and LiDAR paths each own an independent single-slot
    latest-pose stream (:class:`_PoseStream`).
    """

    def __init__(self, address: str = "localhost:50051") -> None:
        self._address = address
        self._channel = grpc.insecure_channel(address)
        self._stub = pb2_grpc.RenderingServiceStub(self._channel)
        self._camera_stream = _PoseStream("camera", self._stub.StreamCameraData)
        self._lidar_stream = _PoseStream("LiDAR", self._stub.StreamLidarData)

    def _call_initialize(self, name: str, rpc: Callable, request) -> pb2.InitializeResponse:
        """Send an initialize-style RPC.  Blocks until the server finishes loading."""
        _rlog.warn(f"Sending {name} to {self._address} ...")
        response = rpc(request)
        if response.success:
            _rlog.warn(f"{name} succeeded")
        else:
            _rlog.error(f"{name} failed: {response.message}")
        return response

    def initialize(self, request: pb2.InitializeRequest) -> pb2.InitializeResponse:
        """Send ``Initialize`` RPC.  Blocks until the server finishes loading."""
        return self._call_initialize("Initialize", self._stub.Initialize, request)

    @staticmethod
    def _make_pose(position, rotation_wxyz) -> pb2.Pose:
        """Build a gRPC ``Pose`` from a position and a wxyz quaternion."""
        return pb2.Pose(
            position=pb2.Vector3(x=position[0], y=position[1], z=position[2]),
            rotation=pb2.Quaternion(
                w=rotation_wxyz[0],
                x=rotation_wxyz[1],
                y=rotation_wxyz[2],
                z=rotation_wxyz[3],
            ),
        )

    def _send_pose(
        self,
        stream: _PoseStream,
        msg_cls,
        sec: int,
        nanosec: int,
        position: tuple[float, float, float],
        rotation_wxyz: tuple[float, float, float, float],
    ) -> None:
        """Wrap a timestamped pose in ``msg_cls`` and push it onto ``stream``."""
        stream.send(
            msg_cls(
                stamp=pb2.Timestamp(sec=sec, nanosec=nanosec),
                pose=self._make_pose(position, rotation_wxyz),
            )
        )

    # ── camera streaming ──────────────────────────────────────────────

    def start_stream(self) -> None:
        """Open the ``StreamCameraData`` client-streaming RPC in a background thread."""
        self._camera_stream.start()

    def send_camera_data(
        self,
        sec: int,
        nanosec: int,
        position: tuple[float, float, float],
        rotation_wxyz: tuple[float, float, float, float],
    ) -> None:
        """Store the latest camera pose for the background stream."""
        self._send_pose(self._camera_stream, pb2.CameraData, sec, nanosec, position, rotation_wxyz)

    def close_stream(self) -> pb2.StreamSummary | None:
        """Signal end-of-stream and wait for the background thread."""
        return self._camera_stream.close()

    def close(self) -> None:
        """Close the gRPC channel."""
        self._channel.close()

    # ── LiDAR streaming ───────────────────────────────────────────────

    def initialize_lidar(self, request: pb2.InitializeLidarRequest) -> pb2.InitializeResponse:
        """Send ``InitializeLidar`` RPC.  ``Initialize`` must have run first."""
        return self._call_initialize("InitializeLidar", self._stub.InitializeLidar, request)

    def start_lidar_stream(self) -> None:
        """Open the ``StreamLidarData`` client-streaming RPC in a background thread."""
        self._lidar_stream.start()

    def send_lidar_data(
        self,
        sec: int,
        nanosec: int,
        position: tuple[float, float, float],
        rotation_wxyz: tuple[float, float, float, float],
    ) -> None:
        """Store the latest base_link pose for the background LiDAR stream."""
        self._send_pose(self._lidar_stream, pb2.LidarData, sec, nanosec, position, rotation_wxyz)

    def close_lidar_stream(self) -> pb2.StreamSummary | None:
        """Signal end-of-stream and wait for the background LiDAR thread."""
        return self._lidar_stream.close()
