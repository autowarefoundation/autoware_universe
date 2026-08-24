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

UNIVERSE = "autoware_default_adapi_universe"

# Nodes derived from autoware::agnocast_wrapper::Node, as (node name, class name, executable).
# Composed like any other node under ENABLE_AGNOCAST=0, where that base is backed by rclcpp;
# run as their own process under =1, where they need an AgnocastOnly executor that a shared
# component container cannot provide.
AGNOCAST_WRAPPER_NODES = [
    ("diagnostics", "DiagnosticsNode", "diagnostics_node"),
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

    components = [
        create_api_node("autoware_default_adapi", "interface", "InterfaceNode"),
        create_api_node("autoware_default_adapi", "localization", "LocalizationNode"),
        create_api_node("autoware_default_adapi", "routing", "RoutingNode"),
        create_api_node("autoware_default_adapi_universe", "autoware_state", "AutowareStateNode"),
        create_api_node("autoware_default_adapi_universe", "fail_safe", "FailSafeNode"),
        create_api_node("autoware_default_adapi_universe", "heartbeat", "HeartbeatNode"),
        create_api_node("autoware_default_adapi_universe", "manual/local", "ManualControlNode"),
        create_api_node("autoware_default_adapi_universe", "manual/remote", "ManualControlNode"),
        create_api_node("autoware_default_adapi_universe", "motion", "MotionNode"),
        create_api_node("autoware_default_adapi_universe", "mrm_request", "MrmRequestNode"),
        create_api_node("autoware_default_adapi_universe", "operation_mode", "OperationModeNode"),
        create_api_node("autoware_default_adapi_universe", "perception", "PerceptionNode"),
        create_api_node("autoware_default_adapi_universe", "planning", "PlanningNode"),
        create_api_node("autoware_default_adapi_universe", "vehicle_status", "VehicleStatusNode"),
        create_api_node("autoware_default_adapi_universe", "vehicle_command", "VehicleCommandNode"),
        create_api_node("autoware_default_adapi_universe", "vehicle_metrics", "VehicleMetricsNode"),
        create_api_node("autoware_default_adapi_universe", "vehicle_info", "VehicleInfoNode"),
        create_api_node("autoware_default_adapi_universe", "vehicle_door", "VehicleDoorNode"),
    ]
    nodes = []
    for node_name, class_name, executable in AGNOCAST_WRAPPER_NODES:
        if use_agnocast:
            nodes.append(create_standalone_api_node(UNIVERSE, node_name, executable))
        else:
            components.append(create_api_node(UNIVERSE, node_name, class_name))

    container = ComposableNodeContainer(
        namespace="adapi",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        ros_arguments=["--log-level", "adapi.container:=WARN"],
        composable_node_descriptions=components,
    )
    return [container, *nodes]


def generate_launch_description():
    argument = DeclareLaunchArgument("config", default_value=get_default_config())
    return launch.LaunchDescription(
        [argument, get_agnocast_env(), OpaqueFunction(function=launch_setup)]
    )
