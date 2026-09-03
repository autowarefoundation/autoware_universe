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

"""Autoware AD API names and the readiness aggregation logic.

The topic/service names are the stable AD API contract (``autoware_adapi_specs``).
:class:`ReadinessAggregator` is kept ``rclpy``-free -- it operates on the plain
enum values from the AD API messages -- so the readiness/engage logic can be unit
tested without a running Autoware stack.
"""

from __future__ import annotations

from dataclasses import dataclass

# -- AD API interface names (see autoware_adapi_specs) -----------------------

#: Service: initialize localization at a pose (empty pose = GNSS auto-init).
LOCALIZATION_INITIALIZE_SERVICE = "/api/localization/initialize"
#: Topic: LocalizationInitializationState.
LOCALIZATION_INITIALIZATION_STATE_TOPIC = "/api/localization/initialization_state"
#: Service: set a route from a goal pose (+ optional waypoints).
ROUTING_SET_ROUTE_POINTS_SERVICE = "/api/routing/set_route_points"
#: Topic: RouteState.
ROUTING_STATE_TOPIC = "/api/routing/state"
#: Topic: OperationModeState.
OPERATION_MODE_STATE_TOPIC = "/api/operation_mode/state"
#: Service: change the operation mode to autonomous.
OPERATION_MODE_CHANGE_TO_AUTONOMOUS_SERVICE = "/api/operation_mode/change_to_autonomous"

# -- Enum values (mirror the autoware_adapi_v1_msgs message constants) --------

#: LocalizationInitializationState.INITIALIZED.
LOCALIZATION_STATE_INITIALIZED = 3
#: RouteState.SET.
ROUTE_STATE_SET = 2
#: OperationModeState.AUTONOMOUS.
OPERATION_MODE_AUTONOMOUS = 2


@dataclass
class ReadinessAggregator:
    """Folds the three AD API states into the engage decision and readiness flag.

    Feed it the plain enum values from the AD API message callbacks; it exposes:

    * :attr:`can_engage` - localization initialized, route set, and autonomous
      mode available: the precondition for calling ``change_to_autonomous``.
    * :attr:`ready` - localization initialized, route set, and Autoware now in
      AUTONOMOUS with control enabled: the flag pushed to the scenario framework.
    """

    localization_initialized: bool = False
    route_set: bool = False
    autonomous_available: bool = False
    autonomous_engaged: bool = False

    def update_localization(self, state: int) -> None:
        """Update from ``LocalizationInitializationState.state``."""
        self.localization_initialized = state == LOCALIZATION_STATE_INITIALIZED

    def update_routing(self, state: int) -> None:
        """Update from ``RouteState.state``."""
        self.route_set = state == ROUTE_STATE_SET

    def update_operation_mode(
        self, mode: int, is_control_enabled: bool, is_autonomous_available: bool
    ) -> None:
        """Update from ``OperationModeState`` fields."""
        self.autonomous_available = is_autonomous_available
        self.autonomous_engaged = mode == OPERATION_MODE_AUTONOMOUS and is_control_enabled

    @property
    def can_engage(self) -> bool:
        """Whether ``change_to_autonomous`` may be called now."""
        return self.localization_initialized and self.route_set and self.autonomous_available

    @property
    def ready(self) -> bool:
        """Whether Autoware is initialized, routed, engaged, and driving."""
        return self.localization_initialized and self.route_set and self.autonomous_engaged
