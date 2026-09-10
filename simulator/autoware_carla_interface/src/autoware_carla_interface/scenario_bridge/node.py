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

This is the Autoware side of the CARLA scenario contract (the remote gRPC service
is named ``AutowareBridge``; this node is its client).  It owns the whole startup
sequence and exposes only a single readiness flag back to the scenario runner:

1. poll ``GetMission`` until the scenario hands over an initial pose + goal;
2. initialize localization at the initial pose (``/api/localization/initialize``) --
   skipped when ``initialize_localization`` is False (ground-truth / E2E stacks);
3. set the route to the goal (``/api/routing/set_route_points``), but only once
   localization has reached INITIALIZED (the mission planner starts the route from
   the current odometry pose and rejects requests before that);
4. once localization is satisfied, the route is SET, and autonomous mode is
   available, engage once (``/api/operation_mode/change_to_autonomous``), gated
   behind ``auto_engage`` (scenario/sim only);
5. aggregate localization/routing/operation-mode state into a single readiness
   flag and push it back with ``ReportReadiness``.  ``require_localization_initialized``
   controls whether localization is part of "ready".

The startup runs as an **idempotent reconciliation loop**: a periodic tick (and
every AD API state change) re-drives whichever step is still outstanding, so a
step that cannot complete yet -- the AD API services are not up, the mission
planner is not ready, or a gRPC call fails transiently -- is retried on the next
pass instead of being silently dropped.  The tick keeps running until readiness
has been reported.

This replaces the manual ``ros2 service call`` goal + engage flow (e.g.
``run_odaiba_outbound.sh``) with a scenario-driven one.

High-bandwidth data (sensors, control, ``/clock``, tf) never flows over the
bridge; it stays on ROS 2 topics and direct CARLA control in the interface node.
"""

from __future__ import annotations

from functools import partial
import threading
from typing import Optional

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_carla_interface.scenario_bridge.ad_api import (
    OPERATION_MODE_CHANGE_TO_AUTONOMOUS_SERVICE,
)
from autoware_carla_interface.scenario_bridge.ad_api import LOCALIZATION_INITIALIZATION_STATE_TOPIC
from autoware_carla_interface.scenario_bridge.ad_api import LOCALIZATION_INITIALIZE_SERVICE
from autoware_carla_interface.scenario_bridge.ad_api import OPERATION_MODE_STATE_TOPIC
from autoware_carla_interface.scenario_bridge.ad_api import ROUTING_SET_ROUTE_POINTS_SERVICE
from autoware_carla_interface.scenario_bridge.ad_api import ROUTING_STATE_TOPIC
from autoware_carla_interface.scenario_bridge.ad_api import ReadinessAggregator
from autoware_carla_interface.scenario_bridge.client import ScenarioBridgeClient
from autoware_carla_interface.scenario_bridge.proto import autoware_bridge_pb2 as pb2
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


def _response_ok(future) -> tuple[bool, str]:
    """Return ``(accepted, detail)`` for a finished AD API service call.

    AD API services answer with an ``autoware_adapi_v1_msgs/ResponseStatus``; a
    request is only accepted when ``status.success``.  A failed ``future`` (the
    call itself raised) or an unsuccessful status both count as "not accepted", so
    the caller can retry.
    """
    try:
        response = future.result()
    except Exception as error:  # noqa: BLE001 - surfaced as a retryable failure
        return False, f"call failed: {error}"
    status = getattr(response, "status", None)
    if status is None:
        return True, ""
    if status.success:
        return True, ""
    return False, f"code={status.code} {status.message}".strip()


def _latched_state_qos() -> QoSProfile:
    """Return the QoS matching the AD API state topics (reliable, transient-local, depth 1)."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class ScenarioBridgeNode(Node):
    """Drives Autoware from a scenario mission and reports readiness over gRPC."""

    def __init__(self) -> None:
        super().__init__("scenario_bridge")

        self._bridge_address = (
            self.declare_parameter("bridge_address", "localhost:50052")
            .get_parameter_value()
            .string_value
        )
        self._auto_engage = (
            self.declare_parameter("auto_engage", True).get_parameter_value().bool_value
        )
        # Localization is a configurable step so the node fits both the mainline
        # AD API flow and stacks that localize outside it (CARLA ground-truth /
        # E2E, localization:=false).  Defaults match the mainline; set both False
        # to skip the /api/localization/initialize call and drop the localization
        # requirement from readiness.
        self._init_localization = (
            self.declare_parameter("initialize_localization", True).get_parameter_value().bool_value
        )
        require_localization = (
            self.declare_parameter("require_localization_initialized", True)
            .get_parameter_value()
            .bool_value
        )
        self._map_frame = (
            self.declare_parameter("map_frame", "map").get_parameter_value().string_value
        )
        self._rpc_timeout_s = (
            self.declare_parameter("rpc_timeout_s", 5.0).get_parameter_value().double_value
        )
        tick_period_s = (
            self.declare_parameter("tick_period_s", 0.5).get_parameter_value().double_value
        )
        # GetMission is polled from the tick with its own short timeout so an
        # unreachable scenario server can't pin a worker thread for the full
        # rpc_timeout_s each poll; the retry just comes on the next tick.
        self._mission_poll_timeout_s = (
            self.declare_parameter("mission_poll_timeout_s", 1.0).get_parameter_value().double_value
        )
        self._client = ScenarioBridgeClient(self._bridge_address)
        self._aggregator = ReadinessAggregator(require_localization=require_localization)

        # Startup state, guarded by ``_lock`` (the tick timer and the three AD API
        # state callbacks all advance concurrently under a MultiThreadedExecutor).
        # One latch per step, set before its request is issued and cleared again
        # only if the request is rejected: an accepted step is never re-sent, a
        # failed one retries on the next pass.
        self._lock = threading.RLock()
        self._mission: Optional[pb2.GetMissionResponse] = None
        self._localization_requested = False
        self._route_requested = False
        self._engage_requested = False
        self._readiness_inflight = False
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

        # Reconciliation tick: polls GetMission until one arrives, then re-drives
        # the outstanding startup step on every pass.  It keeps ticking (rather
        # than cancelling once the mission arrives) so any step whose service is
        # not up yet is retried; it stops only after readiness is reported.
        self._tick_timer = self.create_timer(tick_period_s, self._tick, callback_group=group)

        self.get_logger().info(
            f"scenario_bridge dialling scenario server at {self._bridge_address} "
            f"(auto_engage={self._auto_engage})"
        )

    # ------------------------------------------------------------------
    # Reconciliation loop
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """Periodic driver: poll for the mission, advance the startup, retry readiness.

        Keeps ticking (rather than cancelling once the mission arrives) so any step
        whose AD API service is not up yet is retried, and so ReportReadiness is
        retried after a transient failure even though the AD API states go
        quiescent once Autoware is ready.  Stops once readiness has been reported.
        """
        if self._mission is None:
            self._poll_mission()
            if self._mission is None:
                return
        self._advance()
        # ReportReadiness makes a blocking gRPC call, so it runs only here on the
        # tick -- never on the AD API state-callback path, where it would stall the
        # executor worker servicing that subscription.
        self._maybe_report_ready()
        if self._readiness_reported:
            self._stop_tick()

    def _advance(self) -> None:
        """Re-issue whichever startup step is still outstanding (idempotent).

        Runs on the tick and on every AD API state change.  Each step guards
        itself, so calling this repeatedly only advances the startup: it (re)issues
        localization init, then -- once localization is INITIALIZED -- the route,
        then engages.  Every AD API call is asynchronous, so this never blocks.
        """
        if self._mission is None:
            return
        with self._lock:
            self._ensure_localization()
            if self._localization_ready_for_routing():
                self._ensure_route()
                self._maybe_engage()

    def _stop_tick(self) -> None:
        """Cancel the reconciliation tick once startup is complete.  Idempotent."""
        with self._lock:
            if self._tick_timer is not None:
                self._tick_timer.cancel()
                self._tick_timer = None

    # ------------------------------------------------------------------
    # Mission acquisition
    # ------------------------------------------------------------------

    def _poll_mission(self) -> None:
        """Poll the scenario server once; store the mission when it is available."""
        try:
            mission = self._client.get_mission(timeout=self._mission_poll_timeout_s)
        except grpc.RpcError as error:
            # The scenario server may not be up yet; keep polling.
            self.get_logger().debug(f"GetMission not ready: {error}")
            return
        if mission is None:
            return
        with self._lock:
            if self._mission is None:
                self._mission = mission
                self.get_logger().info("Received scenario mission; driving Autoware startup")

    # ------------------------------------------------------------------
    # Startup steps (each called under _lock, each idempotent + retrying)
    # ------------------------------------------------------------------

    def _issue(self, client, request, latch: str, label: str) -> None:
        """Set *latch*, fire *request*; its response clears *latch* iff rejected.

        Shared by all three startup steps.  The latch is set before the call so a
        concurrent pass won't re-issue the same request; :meth:`_on_step_response`
        clears it again only on rejection, so an accepted step stays latched (never
        re-sent) while a failed one is retried on the next pass.  Called under
        ``_lock``; ``call_async`` never completes synchronously, so the callback
        runs later.
        """
        setattr(self, latch, True)
        future = client.call_async(request)
        future.add_done_callback(partial(self._on_step_response, latch=latch, label=label))

    def _on_step_response(self, future, *, latch: str, label: str) -> None:
        ok, detail = _response_ok(future)
        if ok:
            self.get_logger().info(f"{label} accepted")
            return
        # Rejected (e.g. service not settled yet / ERROR_PLANNER_UNREADY): clear
        # the latch so the next pass retries.
        with self._lock:
            setattr(self, latch, False)
        self.get_logger().warning(f"{label} rejected ({detail}); will retry")

    def _ensure_localization(self) -> None:
        """(Re)issue ``/api/localization/initialize`` until the AD API accepts it.

        No-op when ``initialize_localization`` is ``False`` (stacks that localize
        outside the AD API).  Retries only until the request is accepted; once
        accepted it waits for ``LocalizationInitializationState`` to reach
        INITIALIZED rather than re-sending the pose (which would reset NDT).
        """
        if not self._init_localization:
            return
        if self._localization_requested or self._aggregator.localization_initialized:
            return
        if not self._init_cli.service_is_ready():
            # AD API node not up yet; retry on the next pass.
            return
        request = InitializeLocalization.Request()
        stamped = PoseWithCovarianceStamped()
        stamped.header.frame_id = self._map_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.pose = _to_ros_pose(self._mission.initial_pose)
        stamped.pose.covariance = _INITIAL_POSE_COVARIANCE
        request.pose = [stamped]
        self._issue(self._init_cli, request, "_localization_requested", "localization initialize")

    def _localization_ready_for_routing(self) -> bool:
        """Whether the route may be submitted now.

        ``MissionPlanner::create_route`` starts the route from the current
        ``odometry_->pose.pose`` and rejects requests while its map/odometry
        initialization is not ready, so the route must wait until localization has
        reached INITIALIZED.  Stacks that localize outside the AD API
        (``initialize_localization=false``) publish odometry directly and never
        drive that state, so they skip the gate.
        """
        if not self._init_localization:
            return True
        return self._aggregator.localization_initialized

    def _ensure_route(self) -> None:
        """(Re)issue ``/api/routing/set_route_points`` until the planner accepts it.

        The mission planner can bounce the request (e.g. ``ERROR_PLANNER_UNREADY``)
        while it is still settling; an unsuccessful response leaves ``_route_requested``
        clear so the next pass retries.
        """
        if self._route_requested or self._aggregator.route_set:
            return
        if not self._route_cli.service_is_ready():
            return
        request = SetRoutePoints.Request()
        request.header.frame_id = self._map_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.option.allow_goal_modification = True
        request.goal = _to_ros_pose(self._mission.goal)
        self._issue(self._route_cli, request, "_route_requested", "route set")

    def _maybe_engage(self) -> None:
        """Call ``change_to_autonomous`` once the preconditions hold."""
        if self._engage_requested or not self._auto_engage:
            return
        if not self._aggregator.can_engage:
            return
        if not self._engage_cli.service_is_ready():
            return
        self._issue(
            self._engage_cli,
            ChangeOperationMode.Request(),
            "_engage_requested",
            "change_to_autonomous",
        )

    def _maybe_report_ready(self) -> None:
        """Push ``ReportReadiness(True)`` once Autoware is ready, retrying on failure.

        Called only from the tick; the ``_readiness_inflight`` latch (set under the
        lock before the blocking call) stops two overlapping ticks from both firing
        the RPC.  The tick retries after a transient failure independently of AD API
        state changes, which go quiescent once Autoware is ready.
        """
        with self._lock:
            if self._readiness_reported or self._readiness_inflight or not self._aggregator.ready:
                return
            self._readiness_inflight = True
        try:
            self._client.report_readiness(True, timeout=self._rpc_timeout_s)
        except grpc.RpcError as error:
            with self._lock:
                self._readiness_inflight = False
            self.get_logger().warning(f"ReportReadiness failed ({error.code()}); will retry")
            return
        with self._lock:
            self._readiness_reported = True
            self._readiness_inflight = False
        self.get_logger().info("Autoware ready; reported readiness to the scenario")

    # ------------------------------------------------------------------
    # AD API state callbacks
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

    def destroy_node(self) -> bool:
        """Close the gRPC channel on shutdown."""
        self._client.close()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Entry point for the ``scenario_bridge`` node."""
    rclpy.init(args=args)
    node = ScenarioBridgeNode()
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
