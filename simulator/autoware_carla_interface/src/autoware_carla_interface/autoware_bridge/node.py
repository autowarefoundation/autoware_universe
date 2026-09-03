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

"""ROS 2 node: drive Autoware's startup from a scenario mission and report readiness.

This is the Autoware counterpart of the scenario library's ``AutowareBridge``
server (issue #9).  It owns the whole startup sequence and exposes only a single
readiness flag back to the framework:

1. poll ``GetMission`` until the scenario hands over an initial pose + goal;
2. initialize localization at the initial pose (``/api/localization/initialize``);
3. set the route to the goal (``/api/routing/set_route_points``);
4. once localization is INITIALIZED, the route is SET, and autonomous mode is
   available, engage once (``/api/operation_mode/change_to_autonomous``), gated
   behind ``auto_engage`` (scenario/sim only);
5. aggregate localization/routing/operation-mode state into a single readiness
   flag and push it back with ``ReportReadiness``.

High-bandwidth data (sensors, control, ``/clock``, tf) never flows over the
bridge; it stays on ROS 2 topics and direct CARLA control in the interface node.
"""

from __future__ import annotations

from typing import Optional

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_carla_interface.autoware_bridge.ad_api import (
    OPERATION_MODE_CHANGE_TO_AUTONOMOUS_SERVICE,
)
from autoware_carla_interface.autoware_bridge.ad_api import LOCALIZATION_INITIALIZATION_STATE_TOPIC
from autoware_carla_interface.autoware_bridge.ad_api import LOCALIZATION_INITIALIZE_SERVICE
from autoware_carla_interface.autoware_bridge.ad_api import OPERATION_MODE_STATE_TOPIC
from autoware_carla_interface.autoware_bridge.ad_api import ROUTING_SET_ROUTE_POINTS_SERVICE
from autoware_carla_interface.autoware_bridge.ad_api import ROUTING_STATE_TOPIC
from autoware_carla_interface.autoware_bridge.ad_api import ReadinessAggregator
from autoware_carla_interface.autoware_bridge.client import AutowareBridgeClient
from autoware_carla_interface.autoware_bridge.proto import autoware_bridge_pb2 as pb2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import grpc
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

#: Initial-pose covariance (row-major 6x6), matching the RViz "2D Pose Estimate"
#: default so NDT converges from roughly the same basin.
_INITIAL_POSE_COVARIANCE = [0.0] * 36
_INITIAL_POSE_COVARIANCE[0] = 0.25  # x
_INITIAL_POSE_COVARIANCE[7] = 0.25  # y
_INITIAL_POSE_COVARIANCE[35] = 0.06853891909122467  # yaw


def _to_ros_pose(pose: pb2.Pose) -> Pose:
    """Convert a wire ``Pose`` (position + wxyz quaternion) to ``geometry_msgs/Pose``."""
    ros = Pose()
    ros.position.x = pose.position.x
    ros.position.y = pose.position.y
    ros.position.z = pose.position.z
    ros.orientation.w = pose.rotation.w
    ros.orientation.x = pose.rotation.x
    ros.orientation.y = pose.rotation.y
    ros.orientation.z = pose.rotation.z
    return ros


def _latched_state_qos() -> QoSProfile:
    """Return the QoS matching the AD API state topics (reliable, transient-local, depth 1)."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class AutowareBridgeNode(Node):
    """Drives Autoware from a scenario mission and reports readiness over gRPC."""

    def __init__(self) -> None:
        super().__init__("autoware_bridge")

        self._bridge_address = (
            self.declare_parameter("bridge_address", "localhost:50052")
            .get_parameter_value()
            .string_value
        )
        self._auto_engage = (
            self.declare_parameter("auto_engage", True).get_parameter_value().bool_value
        )
        self._map_frame = (
            self.declare_parameter("map_frame", "map").get_parameter_value().string_value
        )
        self._rpc_timeout_s = (
            self.declare_parameter("rpc_timeout_s", 5.0).get_parameter_value().double_value
        )
        mission_poll_period_s = (
            self.declare_parameter("mission_poll_period_s", 0.5).get_parameter_value().double_value
        )

        self._client = AutowareBridgeClient(self._bridge_address)
        self._aggregator = ReadinessAggregator()

        self._mission: Optional[pb2.GetMissionResponse] = None
        self._engage_requested = False
        self._readiness_reported = False

        group = ReentrantCallbackGroup()

        self._init_cli = self.create_client(
            InitializeLocalization,
            LOCALIZATION_INITIALIZE_SERVICE,
            callback_group=group,
        )
        self._route_cli = self.create_client(
            SetRoutePoints, ROUTING_SET_ROUTE_POINTS_SERVICE, callback_group=group
        )
        self._engage_cli = self.create_client(
            ChangeOperationMode,
            OPERATION_MODE_CHANGE_TO_AUTONOMOUS_SERVICE,
            callback_group=group,
        )

        qos = _latched_state_qos()
        self.create_subscription(
            LocalizationInitializationState,
            LOCALIZATION_INITIALIZATION_STATE_TOPIC,
            self._on_localization_state,
            qos,
            callback_group=group,
        )
        self.create_subscription(
            RouteState,
            ROUTING_STATE_TOPIC,
            self._on_route_state,
            qos,
            callback_group=group,
        )
        self.create_subscription(
            OperationModeState,
            OPERATION_MODE_STATE_TOPIC,
            self._on_operation_mode_state,
            qos,
            callback_group=group,
        )

        # Poll GetMission until the scenario hands one over; this timer cancels
        # itself once the mission has been received and the startup kicked off.
        self._mission_timer = self.create_timer(
            mission_poll_period_s, self._poll_mission, callback_group=group
        )

        self.get_logger().info(
            f"autoware_bridge dialling scenario server at {self._bridge_address} "
            f"(auto_engage={self._auto_engage})"
        )

    # ------------------------------------------------------------------
    # Mission acquisition
    # ------------------------------------------------------------------

    def _poll_mission(self) -> None:
        """Poll the scenario server; on the first mission, start Autoware up."""
        if self._mission is not None:
            return
        try:
            mission = self._client.get_mission(timeout=self._rpc_timeout_s)
        except grpc.RpcError as error:
            # The scenario server may not be up yet; keep polling.
            self.get_logger().debug(f"GetMission not ready: {error}")
            return
        if mission is None:
            return

        self._mission = mission
        self._mission_timer.cancel()
        self.get_logger().info("Received scenario mission; initializing Autoware")
        self._initialize_localization(mission.initial_pose)
        self._set_route(mission.goal)

    def _initialize_localization(self, initial_pose: pb2.Pose) -> None:
        """Call ``/api/localization/initialize`` at *initial_pose*."""
        if not self._init_cli.wait_for_service(timeout_sec=self._rpc_timeout_s):
            self.get_logger().error(
                f"{LOCALIZATION_INITIALIZE_SERVICE} unavailable; localization "
                "will not auto-initialize"
            )
            return
        request = InitializeLocalization.Request()
        stamped = PoseWithCovarianceStamped()
        stamped.header.frame_id = self._map_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.pose = _to_ros_pose(initial_pose)
        stamped.pose.covariance = _INITIAL_POSE_COVARIANCE
        request.pose = [stamped]
        future = self._init_cli.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info("localization initialize requested")
        )

    def _set_route(self, goal: pb2.Pose) -> None:
        """Call ``/api/routing/set_route_points`` to *goal*."""
        if not self._route_cli.wait_for_service(timeout_sec=self._rpc_timeout_s):
            self.get_logger().error(
                f"{ROUTING_SET_ROUTE_POINTS_SERVICE} unavailable; route will not be set"
            )
            return
        request = SetRoutePoints.Request()
        request.header.frame_id = self._map_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.option.allow_goal_modification = True
        request.goal = _to_ros_pose(goal)
        future = self._route_cli.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info("route set requested"))

    # ------------------------------------------------------------------
    # AD API state -> engage + readiness
    # ------------------------------------------------------------------

    def _on_localization_state(self, msg: LocalizationInitializationState) -> None:
        self._aggregator.update_localization(msg.state)
        self._advance()

    def _on_route_state(self, msg: RouteState) -> None:
        self._aggregator.update_routing(msg.state)
        self._advance()

    def _on_operation_mode_state(self, msg: OperationModeState) -> None:
        self._aggregator.update_operation_mode(
            msg.mode, msg.is_autoware_control_enabled, msg.is_autonomous_mode_available
        )
        self._advance()

    def _advance(self) -> None:
        """React to a state change: engage when possible, report when ready."""
        if self._mission is None:
            return
        self._maybe_engage()
        self._maybe_report_ready()

    def _maybe_engage(self) -> None:
        """Call ``change_to_autonomous`` once, when the preconditions hold."""
        if self._engage_requested or not self._auto_engage:
            return
        if not self._aggregator.can_engage:
            return
        if not self._engage_cli.service_is_ready():
            return
        self._engage_requested = True
        future = self._engage_cli.call_async(ChangeOperationMode.Request())
        future.add_done_callback(lambda f: self.get_logger().info("change_to_autonomous requested"))

    def _maybe_report_ready(self) -> None:
        """Push ``ReportReadiness(True)`` once Autoware is ready."""
        if self._readiness_reported or not self._aggregator.ready:
            return
        try:
            self._client.report_readiness(True, timeout=self._rpc_timeout_s)
        except grpc.RpcError as error:
            self.get_logger().warning(f"ReportReadiness failed: {error}")
            return
        self._readiness_reported = True
        self.get_logger().info("Autoware ready; reported readiness to the scenario")

    def destroy_node(self) -> bool:
        """Close the gRPC channel on shutdown."""
        self._client.close()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Entry point for the ``autoware_bridge`` node."""
    rclpy.init(args=args)
    node = AutowareBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
