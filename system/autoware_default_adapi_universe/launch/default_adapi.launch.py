# Copyright 2022 TIER IV, Inc.
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

import pathlib

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

CORE = "autoware_default_adapi"
UNIVERSE = "autoware_default_adapi_universe"

# Nodes derived from autoware::agnocast_wrapper::Node, as (package, node name, class name,
# executable). Composed like any other node under ENABLE_AGNOCAST=0, where that base is backed by
# rclcpp; run as their own process under =1, where they need an AgnocastOnly executor that a shared
# component container cannot provide.
AGNOCAST_WRAPPER_NODES = [
    (CORE, "interface", "InterfaceNode", "interface_node"),
    (CORE, "localization", "LocalizationNode", "localization_node"),
    (CORE, "routing", "RoutingNode", "routing_node"),
    (UNIVERSE, "autoware_state", "AutowareStateNode", "autoware_state_node"),
    (UNIVERSE, "diagnostics", "DiagnosticsNode", "diagnostics_node"),
    (UNIVERSE, "fail_safe", "FailSafeNode", "fail_safe_node"),
    (UNIVERSE, "heartbeat", "HeartbeatNode", "heartbeat_node"),
    (UNIVERSE, "manual/local", "ManualControlNode", "manual_control_node"),
    (UNIVERSE, "manual/remote", "ManualControlNode", "manual_control_node"),
    (UNIVERSE, "motion", "MotionNode", "motion_node"),
    (UNIVERSE, "mrm_request", "MrmRequestNode", "mrm_request_node"),
    (UNIVERSE, "operation_mode", "OperationModeNode", "operation_mode_node"),
    (UNIVERSE, "perception", "PerceptionNode", "perception_node"),
    (UNIVERSE, "planning", "PlanningNode", "planning_node"),
    (UNIVERSE, "vehicle_command", "VehicleCommandNode", "vehicle_command_node"),
    (UNIVERSE, "vehicle_door", "VehicleDoorNode", "vehicle_door_node"),
    (UNIVERSE, "vehicle_info", "VehicleInfoNode", "vehicle_info_node"),
    (UNIVERSE, "vehicle_metrics", "VehicleMetricsNode", "vehicle_metrics_node"),
    (UNIVERSE, "vehicle_status", "VehicleStatusNode", "vehicle_status_node"),
]


def create_api_node(package_name, node_name, class_name):
    fullname = pathlib.Path("adapi/node") / node_name
    return ComposableNode(
        namespace=str(fullname.parent),
        name=str(fullname.name),
        package=package_name,
        plugin="autoware::default_adapi::" + class_name,
        parameters=[ParameterFile(LaunchConfiguration("config"))],
    )


def create_standalone_api_node(package_name, node_name, executable):
    """Launch one AGNOCAST_WRAPPER_NODES entry as its own process.

    LD_PRELOAD goes on the node process alone: the heaphook has to be in place before the node
    allocates, and preloading it into the launch process would register a second Agnocast process.
    """
    fullname = pathlib.Path("adapi/node") / node_name
    return Node(
        namespace=str(fullname.parent),
        name=str(fullname.name),
        package=package_name,
        executable=executable,
        parameters=[ParameterFile(LaunchConfiguration("config"))],
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
        output="screen",
    )


def get_agnocast_env():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("autoware_agnocast_wrapper"),
                    "launch",
                    "agnocast_env.launch.py",
                ]
            )
        )
    )


def get_default_config():
    path = FindPackageShare("autoware_default_adapi_universe")
    path = PathJoinSubstitution([path, "config/default_adapi.param.yaml"])
    return path


def launch_setup(context, *args, **kwargs):
    use_agnocast = context.perform_substitution(LaunchConfiguration("use_agnocast")) == "1"

    if use_agnocast:
        return [
            create_standalone_api_node(package_name, node_name, executable)
            for package_name, node_name, _, executable in AGNOCAST_WRAPPER_NODES
        ]

    components = [
        create_api_node(package_name, node_name, class_name)
        for package_name, node_name, class_name, _ in AGNOCAST_WRAPPER_NODES
    ]
    container = ComposableNodeContainer(
        namespace="adapi",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        ros_arguments=["--log-level", "adapi.container:=WARN"],
        composable_node_descriptions=components,
    )
    return [container]


def generate_launch_description():
    argument = DeclareLaunchArgument("config", default_value=get_default_config())
    return launch.LaunchDescription(
        [argument, get_agnocast_env(), OpaqueFunction(function=launch_setup)]
    )
