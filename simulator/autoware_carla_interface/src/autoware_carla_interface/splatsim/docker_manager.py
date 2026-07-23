"""Manage the splatsim Docker container lifecycle."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import threading
import time

import docker
from docker.types import DeviceRequest
import grpc


def _log(msg: str) -> None:
    """Print to stderr so it always appears in the ROS 2 launch terminal."""
    print(f"[splatsim-docker] {msg}", file=sys.stderr, flush=True)


class SplatSimDockerManager:
    """Start / stop the ``splatsim:latest`` Docker container.

    The container is launched with ``--network=host`` so that DDS
    multicast discovery works between the container and the host, and
    with full GPU access via the NVIDIA Container Toolkit.
    """

    def __init__(
        self,
        image: str = "splatsim:latest",
        grpc_port: int = 50051,
        container_name: str | None = None,
        force_restart: bool = False,
    ) -> None:
        self._image = image
        self._grpc_port = grpc_port
        self._container_name = container_name
        self._force_restart = force_restart
        self._client = docker.from_env()
        self._container = None
        self._container_dead = False
        self._reused = False

    @property
    def grpc_address(self) -> str:
        return f"localhost:{self._grpc_port}"

    def start(self, tileset_host_path: str) -> str:
        """Start the container and return the gRPC address.

        If a container with the configured name already exists and is
        running, it will be reused instead of launching a new one.

        Parameters
        ----------
        tileset_host_path:
            Absolute path to the tileset directory on the host.
            Mounted as ``/data`` inside the container.
        """
        # Check for an existing container with the same name.
        if self._container_name:
            try:
                existing = self._client.containers.get(self._container_name)
                existing.reload()
                if existing.status == "running" and not self._force_restart:
                    _log(
                        f"Container '{self._container_name}' is already running "
                        f"({existing.short_id}), reusing it"
                    )
                    self._container = existing
                    self._reused = True
                    self._log_thread = threading.Thread(
                        target=self._stream_logs,
                        daemon=True,
                    )
                    self._log_thread.start()
                    return self.grpc_address
                else:
                    reason = (
                        "force_restart requested"
                        if self._force_restart
                        else f"status={existing.status}"
                    )
                    _log(f"Container '{self._container_name}' exists ({reason}), " f"removing it")
                    existing.stop(timeout=10)
                    existing.remove(force=True)
            except docker.errors.NotFound:
                pass

        tileset_dir = str(Path(tileset_host_path).resolve().parent)

        _log(
            f"Starting container (image={self._image}, name={self._container_name}, mount={tileset_dir} -> /data)"
        )
        env = {"GRPC_PORT": str(self._grpc_port)}
        splatsim_log_level = os.environ.get("SPLATSIM_LOG_LEVEL")
        if splatsim_log_level:
            env["SPLATSIM_LOG_LEVEL"] = splatsim_log_level
        run_kwargs = {
            "image": self._image,
            "command": f"splatsim-grpc-server --port {self._grpc_port}",
            "detach": True,
            "network_mode": "host",
            "device_requests": [DeviceRequest(count=-1, capabilities=[["gpu"]])],
            "volumes": {tileset_dir: {"bind": "/data", "mode": "ro"}},
            "environment": env,
        }
        if self._container_name:
            run_kwargs["name"] = self._container_name
        self._container = self._client.containers.run(**run_kwargs)
        _log(f"Container started: {self._container.short_id}")
        self._log_thread = threading.Thread(
            target=self._stream_logs,
            daemon=True,
        )
        self._log_thread.start()
        return self.grpc_address

    def wait_for_ready(self, timeout: float = 60.0) -> None:
        """Block until the gRPC server is reachable or *timeout* expires."""
        deadline = time.monotonic() + timeout
        address = self.grpc_address
        while time.monotonic() < deadline:
            if self._container_dead:
                raise RuntimeError(
                    "splatsim container exited before gRPC became ready. "
                    "Check the [splatsim] log lines above for details."
                )
            try:
                channel = grpc.insecure_channel(address)
                grpc.channel_ready_future(channel).result(timeout=2.0)
                channel.close()
                _log(f"gRPC server is ready at {address}")
                return
            except grpc.FutureTimeoutError:
                pass
            except Exception:
                pass
            time.sleep(1.0)
        raise TimeoutError(f"splatsim gRPC server at {address} not ready within {timeout}s")

    def _stream_logs(self) -> None:
        """Stream container logs to stderr in a background thread."""
        try:
            for chunk in self._container.logs(stream=True, follow=True):
                for line in chunk.decode("utf-8", errors="replace").splitlines():
                    _log(line)
        except Exception as exc:
            _log(f"Log stream ended: {exc}")
        # If we reach here, the container has stopped producing logs.
        try:
            self._container.reload()
            status = self._container.status
        except Exception:
            status = "removed"
        if status != "running":
            _log(f"Container is no longer running (status={status})")
            self._container_dead = True

    def stop(self) -> None:
        """Stop and remove the container (idempotent)."""
        if self._container is not None:
            try:
                self._container.stop(timeout=10)
                _log(f"Container stopped: {self._container.short_id}")
            except Exception as exc:
                _log(f"Error stopping container: {exc}")
            try:
                self._container.remove(force=True)
            except Exception:
                pass
            self._container = None
