"""gRPC client wrapper for the splatsim RenderingService."""

from __future__ import annotations

import threading
from typing import Iterator

import grpc
import rclpy

from autoware_carla_interface.splatsim.proto import (
    rendering_service_pb2 as pb2,
    rendering_service_pb2_grpc as pb2_grpc,
)

_rlog = rclpy.logging.get_logger("splatsim_grpc_client")


class SplatSimGrpcClient:
    """Thread-safe gRPC client for the splatsim ``RenderingService``.

    No client-side buffering — only the latest pose is kept.  The
    generator yields only when a new pose arrives, so the gRPC
    transport never accumulates a stale backlog.
    """

    def __init__(self, address: str = "localhost:50051") -> None:
        self._address = address
        self._channel = grpc.insecure_channel(address)
        self._stub = pb2_grpc.RenderingServiceStub(self._channel)

        # Single-slot latest pose (no queue)
        self._latest_pose: pb2.CameraData | None = None
        self._pose_lock = threading.Lock()
        self._new_pose = threading.Event()
        self._closed = False

        self._stream_thread: threading.Thread | None = None
        self._stream_result: pb2.StreamSummary | None = None
        self._stream_error: Exception | None = None
        self._send_count: int = 0

    def initialize(self, request: pb2.InitializeRequest) -> pb2.InitializeResponse:
        """Send ``Initialize`` RPC.  Blocks until the server finishes loading."""
        _rlog.warn(f"Sending Initialize to {self._address} ...")
        response = self._stub.Initialize(request)
        if response.success:
            _rlog.warn("Initialize succeeded")
        else:
            _rlog.error(f"Initialize failed: {response.message}")
        return response

    # ── streaming ─────────────────────────────────────────────────────

    def start_stream(self) -> None:
        """Open the ``StreamCameraData`` client-streaming RPC in a background thread."""
        self._stream_thread = threading.Thread(
            target=self._stream_worker, daemon=True
        )
        self._stream_thread.start()

    def send_camera_data(
        self,
        sec: int,
        nanosec: int,
        position: tuple[float, float, float],
        rotation_wxyz: tuple[float, float, float, float],
    ) -> None:
        """Store the latest camera pose for the background stream."""
        if self._stream_thread is not None and not self._stream_thread.is_alive():
            if self._stream_error is not None:
                _rlog.error(f"gRPC stream died: {self._stream_error}")
            else:
                _rlog.error("gRPC stream thread ended unexpectedly")
            return

        msg = pb2.CameraData(
            stamp=pb2.Timestamp(sec=sec, nanosec=nanosec),
            pose=pb2.Pose(
                position=pb2.Vector3(x=position[0], y=position[1], z=position[2]),
                rotation=pb2.Quaternion(
                    w=rotation_wxyz[0],
                    x=rotation_wxyz[1],
                    y=rotation_wxyz[2],
                    z=rotation_wxyz[3],
                ),
            ),
        )
        with self._pose_lock:
            self._latest_pose = msg
        self._new_pose.set()

        self._send_count += 1
        if self._send_count <= 10 or self._send_count % 50 == 0:
            _rlog.warn(
                f"gRPC send #{self._send_count}: "
                f"pos=({position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f})"
            )

    def close_stream(self) -> pb2.StreamSummary | None:
        """Signal end-of-stream and wait for the background thread."""
        self._closed = True
        self._new_pose.set()  # wake generator
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=10.0)
        if self._stream_error is not None:
            _rlog.error(f"Stream ended with error: {self._stream_error}")
        return self._stream_result

    def close(self) -> None:
        """Close the gRPC channel."""
        self._channel.close()

    # ── internals ─────────────────────────────────────────────────────

    def _pose_generator(self) -> Iterator[pb2.CameraData]:
        """Yield only the latest pose, blocking until a new one arrives."""
        yield_count = 0
        while True:
            self._new_pose.wait()
            if self._closed:
                _rlog.warn(f"Generator done after {yield_count} yields")
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
                    f"Generator yield #{yield_count}: "
                    f"pos=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) "
                    f"t={msg.stamp.sec}.{msg.stamp.nanosec:09d}"
                )
            yield msg

    def _stream_worker(self) -> None:
        """Background thread that drives the StreamCameraData RPC."""
        try:
            self._stream_result = self._stub.StreamCameraData(
                self._pose_generator()
            )
            _rlog.warn(
                f"Stream finished: rendered={self._stream_result.frames_rendered}, "
                f"received={self._stream_result.poses_received}"
            )
        except Exception as exc:
            self._stream_error = exc
            _rlog.error(f"StreamCameraData failed: {exc}")
