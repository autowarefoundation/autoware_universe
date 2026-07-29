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

"""Manage the splatsim Docker container lifecycle."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import docker
from docker.types import DeviceRequest
import grpc


def _log(msg: str) -> None:
    """Print to stderr so it always appears in the ROS 2 launch terminal."""
    print(f"[splatsim-docker] {msg}", file=sys.stderr, flush=True)


def _detect_cuda_arch(default: str = "89") -> str:
    """Host GPU compute capability as an sm string (e.g. '8.9' -> '89', '12.0' -> '120').

    Used to auto-select the matching GHCR splatsim image tag (``latest-sm<arch>``),
    so the pulled image follows the host GPU. Falls back to *default* (sm_89, Ada)
    when ``nvidia-smi`` is unavailable.
    """
    try:
        out = (
            subprocess.run(
                ["nvidia-smi", "--query-gpu=compute_cap", "--format=csv,noheader"],
                capture_output=True,
                text=True,
                timeout=10,
                check=True,
            )
            .stdout.strip()
            .splitlines()[0]
            .strip()
        )
        major, _, minor = out.partition(".")
        return f"{int(major)}{int(minor)}"
    except Exception as exc:  # noqa: BLE001
        _log(f"WARNING: GPU compute-cap detection failed ({exc}); defaulting to sm_{default}")
        return default


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

        # Auto-resolve an "{arch}" placeholder in the image tag from the host GPU's
        # compute capability so the matching GHCR build is pulled automatically
        # (e.g. ghcr.io/tier4/splatsim:latest-sm{arch} -> ...-sm86 / ...-sm89 / ...-sm120).
        if "{arch}" in self._image:
            self._image = self._image.replace("{arch}", _detect_cuda_arch())
            _log(f"Resolved GPU-specific splatsim image: {self._image}")

        _log(
            f"Starting container (image={self._image}, name={self._container_name}, mount={tileset_dir} -> /data)"
        )
        env = {"GRPC_PORT": str(self._grpc_port)}
        splatsim_log_level = os.environ.get("SPLATSIM_LOG_LEVEL")
        if splatsim_log_level:
            env["SPLATSIM_LOG_LEVEL"] = splatsim_log_level
        # Propagate ROS 2 DDS settings so the container's CycloneDDS PointCloud2
        # publisher joins the same graph as the Autoware side.  Both run with
        # --network=host, so pinning CycloneDDS to the loopback interface matches
        # the Autoware devcontainer's lo-only config and lets discovery succeed
        # over the shared host lo (default all-interface discovery does not reach
        # the lo-restricted subscriber).  ROS_DOMAIN_ID / RMW must match too.
        env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")
        env["RMW_IMPLEMENTATION"] = os.environ.get(
            "RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"
        )
        env["CYCLONEDDS_URI"] = os.environ.get(
            "SPLATSIM_CYCLONEDDS_URI",
            '<CycloneDDS><Domain Id="any"><General>'
            '<Interfaces><NetworkInterface name="lo" priority="default" multicast="default"/></Interfaces>'
            "<AllowMulticast>default</AllowMulticast></General></Domain></CycloneDDS>",
        )
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
        # TEMPORARY dev-only escape hatch: shadow-mount a host copy of the
        # container's PointCloud2 publisher, e.g. to switch its DDS QoS to
        # BEST_EFFORT so it matches Autoware's SensorDataQoS (a RELIABLE writer
        # stalls per-frame on ~2 MB LiDAR messages and caps FPS). The real fix
        # is a one-line QoS change in tier4/splatsim's publisher shipped in the
        # GHCR image; drop this hatch once that lands. Off unless the env var is
        # set. The bind source is a HOST path (resolved by the host daemon) and
        # the target is splatsim's private layout, so it is inherently coupled to
        # the image internals -- keep it opt-in, never a default.
        pc_patch = os.environ.get("SPLATSIM_PC_PUBLISHER_PATCH")
        if pc_patch:
            run_kwargs["volumes"][pc_patch] = {
                "bind": "/workspace/splatsim/src/splatsim/cyclonedds/pointcloud2_publisher.py",
                "mode": "ro",
            }
        # Refresh registry-qualified images (e.g. ghcr.io/tier4/splatsim:latest-sm89)
        # so the newest GHCR build is used instead of a stale local tag. Bare local
        # tags (e.g. "splatsim:latest") are left untouched. Best-effort: fall back to
        # whatever is available locally if the pull fails (offline / auth).
        if "/" in self._image:
            repo, _, tag = self._image.rpartition(":")
            if not repo:
                repo, tag = self._image, "latest"
            try:
                _log(f"Pulling {self._image} from registry ...")
                self._client.images.pull(repo, tag=tag or "latest")
                _log(f"Pull complete: {self._image}")
            except docker.errors.APIError as exc:
                _log(f"WARNING: pull failed ({exc}); using local image if present")
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
        """Stop and remove the container (idempotent).

        A container that was reused (i.e. started externally before the
        bridge) is left running: this manager did not create it, so it must
        not tear it down on shutdown.
        """
        if self._container is None:
            return
        if self._reused:
            _log(
                f"Container '{self._container_name}' was reused, " f"leaving it running on shutdown"
            )
            self._container = None
            return
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
