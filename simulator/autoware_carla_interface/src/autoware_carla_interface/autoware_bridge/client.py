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

"""gRPC client for the scenario library's ``AutowareBridge`` server.

This is the Autoware side of the (deliberately tiny) contract: pull the mission
once it is available (``GetMission``) and push a single readiness flag
(``ReportReadiness``).  It mirrors ``splatsim/grpc_client.py`` -- an
``insecure_channel`` to a server the interface node launches/dials -- but is kept
free of ``rclpy`` so it can be unit tested against an in-process server.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Optional

from autoware_carla_interface.autoware_bridge.proto import autoware_bridge_pb2 as pb2
from autoware_carla_interface.autoware_bridge.proto import autoware_bridge_pb2_grpc as pb2_grpc
import grpc

logger = logging.getLogger(__name__)

__all__ = ["AutowareBridgeClient", "Mission"]


@dataclass(frozen=True)
class Mission:
    """The scenario's mission, as map-frame ``Pose`` wire messages.

    Attributes:
        initial_pose: Where Autoware initializes localization.
        goal: Where Autoware plans the route to.
    """

    initial_pose: pb2.Pose
    goal: pb2.Pose


class AutowareBridgeClient:
    """Talks to the scenario library's ``AutowareBridge`` server.

    Args:
        address: ``host:port`` of the scenario-library server to dial.
        channel: Pre-built channel to use instead of dialling *address*.
            Intended for tests, which run the server in-process.
    """

    def __init__(
        self,
        address: str = "localhost:50052",
        *,
        channel: Optional[grpc.Channel] = None,
    ) -> None:
        self._address = address
        self._owns_channel = channel is None
        self._channel = channel or grpc.insecure_channel(address)
        self._stub = pb2_grpc.AutowareBridgeStub(self._channel)

    def wait_for_ready(self, timeout: float) -> bool:
        """Block until the channel connects, returning ``True`` on success.

        Args:
            timeout: Seconds to wait for the server to become reachable.
        """
        try:
            grpc.channel_ready_future(self._channel).result(timeout=timeout)
            return True
        except grpc.FutureTimeoutError:
            logger.warning(
                "AutowareBridge server at %s not reachable within %.1fs",
                self._address,
                timeout,
            )
            return False

    def get_mission(self, timeout: Optional[float] = None) -> Optional[Mission]:
        """Return the mission once the scenario has one, else ``None``.

        Non-fatal: while the scenario has not configured a mission yet the server
        answers ``available=False`` and this returns ``None``, so the caller can
        keep polling.

        Args:
            timeout: Per-RPC timeout in seconds.

        Raises:
            grpc.RpcError: If the server cannot be reached.
        """
        response = self._stub.GetMission(pb2.GetMissionRequest(), timeout=timeout)
        if not response.available:
            return None
        return Mission(initial_pose=response.initial_pose, goal=response.goal)

    def report_readiness(self, ready: bool, timeout: Optional[float] = None) -> None:
        """Push the readiness flag to the scenario framework.

        Args:
            ready: ``True`` once Autoware is initialized, routed, engaged, driving.
            timeout: Per-RPC timeout in seconds.

        Raises:
            grpc.RpcError: If the server cannot be reached.
        """
        self._stub.ReportReadiness(pb2.ReportReadinessRequest(ready=ready), timeout=timeout)

    def close(self) -> None:
        """Close the channel.  Idempotent; a no-op for an injected channel."""
        if self._owns_channel and self._channel is not None:
            self._channel.close()
            self._channel = None  # type: ignore[assignment]
