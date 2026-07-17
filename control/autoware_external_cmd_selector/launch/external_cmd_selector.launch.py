# Copyright 2021 Tier IV, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import yaml


def _create_mapping_tuple(name):
    return ("~/" + name, LaunchConfiguration(name))


def launch_setup(context, *args, **kwargs):
    with open(LaunchConfiguration("external_cmd_selector_param_path").perform(context), "r") as f:
        external_cmd_selector_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    remappings = [
        _create_mapping_tuple("service/select_external_command"),
        _create_mapping_tuple("output/current_selector_mode"),
        _create_mapping_tuple("input/local/pedals_cmd"),
        _create_mapping_tuple("input/local/steering_cmd"),
        _create_mapping_tuple("input/local/heartbeat"),
        _create_mapping_tuple("input/local/gear_cmd"),
        _create_mapping_tuple("input/local/turn_indicators_cmd"),
        _create_mapping_tuple("input/local/hazard_lights_cmd"),
        _create_mapping_tuple("input/remote/pedals_cmd"),
        _create_mapping_tuple("input/remote/steering_cmd"),
        _create_mapping_tuple("input/remote/heartbeat"),
        _create_mapping_tuple("input/remote/gear_cmd"),
        _create_mapping_tuple("input/remote/turn_indicators_cmd"),
        _create_mapping_tuple("input/remote/hazard_lights_cmd"),
        _create_mapping_tuple("output/pedals_cmd"),
        _create_mapping_tuple("output/steering_cmd"),
        _create_mapping_tuple("output/heartbeat"),
        _create_mapping_tuple("output/gear_cmd"),
        _create_mapping_tuple("output/turn_indicators_cmd"),
        _create_mapping_tuple("output/hazard_lights_cmd"),
    ]

    node = Node(
        package="autoware_external_cmd_selector",
        executable="autoware_external_cmd_selector",
        name="external_cmd_selector",
        remappings=remappings,
        parameters=[
            external_cmd_selector_param,
        ],
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
        output="screen",
    )

    group = GroupAction(
        [
            PushRosNamespace(""),
            node,
        ]
    )

    return [group]


def generate_launch_description():
    # fmt: off
    arguments = [
        # mode select
        DeclareLaunchArgument("service/select_external_command", default_value="~/select_external_command"),
        DeclareLaunchArgument("output/current_selector_mode", default_value="~/current_selector_mode"),
        # local input
        DeclareLaunchArgument("input/local/pedals_cmd", default_value="/external/local/pedals_cmd"),
        DeclareLaunchArgument("input/local/steering_cmd", default_value="/external/local/steering_cmd"),
        DeclareLaunchArgument("input/local/heartbeat", default_value="/external/local/heartbeat"),
        DeclareLaunchArgument("input/local/gear_cmd", default_value="/external/local/gear_cmd"),
        DeclareLaunchArgument("input/local/turn_indicators_cmd", default_value="/external/local/turn_indicators_cmd"),
        DeclareLaunchArgument("input/local/hazard_lights_cmd", default_value="/external/local/hazard_lights_cmd"),
        # remote input
        DeclareLaunchArgument("input/remote/pedals_cmd", default_value="/external/remote/pedals_cmd"),
        DeclareLaunchArgument("input/remote/steering_cmd", default_value="/external/remote/steering_cmd"),
        DeclareLaunchArgument("input/remote/heartbeat", default_value="/external/remote/heartbeat"),
        DeclareLaunchArgument("input/remote/gear_cmd", default_value="/external/remote/gear_cmd"),
        DeclareLaunchArgument("input/remote/turn_indicators_cmd", default_value="/external/remote/turn_indicators_cmd"),
        DeclareLaunchArgument("input/remote/hazard_lights_cmd", default_value="/external/remote/hazard_lights_cmd"),
        # output
        DeclareLaunchArgument("output/pedals_cmd", default_value="/external/selected/pedals_cmd"),
        DeclareLaunchArgument("output/steering_cmd", default_value="/external/selected/steering_cmd"),
        DeclareLaunchArgument("output/heartbeat", default_value="/external/selected/heartbeat"),
        DeclareLaunchArgument("output/gear_cmd", default_value="/external/selected/gear_cmd"),
        DeclareLaunchArgument("output/turn_indicators_cmd", default_value="/external/selected/turn_indicators_cmd"),
        DeclareLaunchArgument("output/hazard_lights_cmd", default_value="/external/selected/hazard_lights_cmd"),
    ]
    # fmt: on

    agnocast_env = IncludeLaunchDescription(
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

    return LaunchDescription(arguments + [agnocast_env, OpaqueFunction(function=launch_setup)])
