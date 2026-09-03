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

"""Round-trip tests for AutowareBridgeClient against an in-process server.

ROS 2-free: a stub server plays the scenario-library role on a real loopback
port, so these exercise the generated stubs and the client end to end without a
running Autoware stack.
"""

from concurrent import futures

from autoware_carla_interface.autoware_bridge.client import AutowareBridgeClient
from autoware_carla_interface.autoware_bridge.proto import autoware_bridge_pb2 as pb2
from autoware_carla_interface.autoware_bridge.proto import autoware_bridge_pb2_grpc as pb2_grpc
import grpc
import pytest


class _StubServer(pb2_grpc.AutowareBridgeServicer):
    """Records readiness reports and hands out a mission once one is set."""

    def __init__(self):
        self.mission = None  # (initial_pose, goal) or None
        self.reported = []

    def GetMission(self, request, context):  # noqa: N802 - gRPC signature
        if self.mission is None:
            return pb2.GetMissionResponse(available=False)
        initial, goal = self.mission
        return pb2.GetMissionResponse(available=True, initial_pose=initial, goal=goal)

    def ReportReadiness(self, request, context):  # noqa: N802 - gRPC signature
        self.reported.append(request.ready)
        return pb2.ReportReadinessResponse()


@pytest.fixture
def server():
    stub = _StubServer()
    grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    pb2_grpc.add_AutowareBridgeServicer_to_server(stub, grpc_server)
    stub.port = grpc_server.add_insecure_port("localhost:0")
    grpc_server.start()
    try:
        yield stub
    finally:
        grpc_server.stop(grace=None)


def _client(server):
    channel = grpc.insecure_channel(f"localhost:{server.port}")
    return AutowareBridgeClient(channel=channel)


def _pose(x, y, z):
    return pb2.Pose(
        position=pb2.Vector3(x=x, y=y, z=z),
        rotation=pb2.Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
    )


def test_get_mission_is_none_until_available(server):
    client = _client(server)
    assert client.get_mission(timeout=5.0) is None


def test_get_mission_returns_configured_mission(server):
    server.mission = (_pose(1.0, 2.0, 3.0), _pose(10.0, 20.0, 30.0))
    client = _client(server)

    mission = client.get_mission(timeout=5.0)

    assert mission is not None
    assert mission.initial_pose.position.x == pytest.approx(1.0)
    assert mission.goal.position.x == pytest.approx(10.0)
    assert mission.goal.position.z == pytest.approx(30.0)


def test_report_readiness_reaches_the_server(server):
    client = _client(server)
    client.report_readiness(True, timeout=5.0)
    assert server.reported == [True]


def test_close_is_a_no_op_for_injected_channel(server):
    client = _client(server)
    client.close()  # must not raise
    client.close()
