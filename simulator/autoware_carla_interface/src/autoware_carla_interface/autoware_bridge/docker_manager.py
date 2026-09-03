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

"""Splatsim-style Docker launch for the scenario-library gRPC server.

The interface node launches the scenario-library container (which hosts the
``AutowareBridge`` server) and then dials it with :class:`AutowareBridgeClient`,
mirroring ``splatsim/docker_manager.py`` (which launches the renderer server the
splatsim client connects to).  ``--network=host`` keeps the gRPC endpoint on
``localhost`` and lets the container share the CARLA world / DDS with the host.

This is optional: when the scenario server is started out of band (e.g. in CI or
by hand), set ``launch_scenario_container:=false`` and point ``bridge_address``
at it directly.  ``docker`` is imported lazily so the node does not hard-depend
on the SDK when the container launch is disabled.
"""

from __future__ import annotations

import logging

import grpc

logger = logging.getLogger(__name__)

__all__ = ["ScenarioContainerManager"]


class ScenarioContainerManager:
    """Runs (and tears down) the scenario-library gRPC-server container.

    Args:
        image: Container image that runs the scenario server.
        command: Command the container runs to start the server.
        grpc_port: Port the server listens on (published via host networking).
        container_name: Name to reuse an already-running container by.
    """

    def __init__(
        self,
        image: str,
        *,
        command: str,
        grpc_port: int = 50052,
        container_name: str = "autoware_scenario_bridge",
    ) -> None:
        self._image = image
        self._command = command
        self._grpc_port = grpc_port
        self._container_name = container_name
        self._container = None  # type: ignore[assignment]

    @property
    def grpc_address(self) -> str:
        """Return the ``host:port`` the client should dial."""
        return f"localhost:{self._grpc_port}"

    def start(self) -> str:
        """Start the container (reusing one by name if already running).

        Returns:
            The ``host:port`` address of the started server.

        Raises:
            ImportError: If the ``docker`` SDK is not installed.
        """
        import docker  # noqa: PLC0415 - optional dependency, imported lazily

        client = docker.from_env()
        existing = client.containers.list(all=True, filters={"name": self._container_name})
        if existing:
            container = existing[0]
            if container.status != "running":
                container.start()
            self._container = container
            logger.info("Reusing scenario container %s", self._container_name)
            return self.grpc_address

        logger.info(
            "Starting scenario container (image=%s, name=%s, port=%d)",
            self._image,
            self._container_name,
            self._grpc_port,
        )
        self._container = client.containers.run(
            image=self._image,
            command=self._command,
            name=self._container_name,
            detach=True,
            # Host networking keeps the gRPC endpoint on localhost and shares the
            # CARLA world / DDS with the host, matching the splatsim launch.
            network_mode="host",
            remove=True,
        )
        return self.grpc_address

    def wait_for_ready(self, timeout: float = 60.0) -> bool:
        """Block until the server accepts connections, returning success."""
        try:
            grpc.channel_ready_future(grpc.insecure_channel(self.grpc_address)).result(
                timeout=timeout
            )
            return True
        except grpc.FutureTimeoutError:
            logger.error(
                "Scenario server at %s not ready within %.1fs",
                self.grpc_address,
                timeout,
            )
            return False

    def stop(self) -> None:
        """Stop the container if this manager started it.  Idempotent."""
        container = self._container
        self._container = None
        if container is None:
            return
        try:
            container.stop()
        except Exception:  # noqa: BLE001 - teardown must not raise
            logger.warning("Failed to stop scenario container", exc_info=True)
