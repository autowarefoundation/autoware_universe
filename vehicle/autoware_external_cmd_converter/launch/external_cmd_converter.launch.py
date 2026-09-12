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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _create_mapping_tuple(name):
    return (name, LaunchConfiguration(name))


def generate_launch_description():
    arguments = [
        # map file
        DeclareLaunchArgument(
            "csv_path_accel_map",
            default_value=[
                FindPackageShare("autoware_raw_vehicle_cmd_converter"),
                "/data/default/accel_map.csv",
            ],  # noqa: E501
            description="csv file path for accel map",
        ),
        DeclareLaunchArgument(
            "csv_path_brake_map",
            default_value=[
                FindPackageShare("autoware_raw_vehicle_cmd_converter"),
                "/data/default/brake_map.csv",
            ],  # noqa: E501
            description="csv file path for brake map",
        ),
        # settings
        DeclareLaunchArgument(
            "ref_vel_gain",
            default_value="3.0",
            description="gain for external command accel",
        ),
        DeclareLaunchArgument(
            "timer_rate",
            default_value="10.0",
            description="timer's update rate",
        ),
        DeclareLaunchArgument(
            "wait_for_first_topic",
            default_value="true",
            description="disable topic disruption detection until subscribing first topics",
        ),
        DeclareLaunchArgument(
            "control_command_timeout",
            default_value="1.0",
            description="external control command timeout",
        ),
        DeclareLaunchArgument(
            "emergency_stop_timeout",
            default_value="3.0",
            description="emergency stop timeout for external heartbeat",
        ),
        # input
        DeclareLaunchArgument(
            "in/pedals_cmd",
            default_value="/external/selected/pedals_cmd",
        ),
        DeclareLaunchArgument(
            "in/steering_cmd",
            default_value="/external/selected/steering_cmd",
        ),
        DeclareLaunchArgument(
            "in/gear_cmd",
            default_value="/external/selected/gear_cmd",
        ),
        DeclareLaunchArgument(
            "in/heartbeat",
            default_value="/external/selected/heartbeat",
        ),
        DeclareLaunchArgument(
            "in/current_gate_mode",
            default_value="/control/current_gate_mode",
        ),
        DeclareLaunchArgument(
            "in/odometry",
            default_value="/localization/kinematic_state",
        ),
        # output
        DeclareLaunchArgument(
            "out/control_cmd",
            default_value="/external/selected/control_cmd",
        ),
    ]

    agnocast_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("autoware_agnocast_wrapper"),
                    "launch",
                    "agnocast_env.launch.py",
                ]
            )
        ),
    )

    remappings = [
        _create_mapping_tuple("in/pedals_cmd"),
        _create_mapping_tuple("in/steering_cmd"),
        _create_mapping_tuple("in/gear_cmd"),
        _create_mapping_tuple("in/heartbeat"),
        _create_mapping_tuple("in/current_gate_mode"),
        _create_mapping_tuple("in/odometry"),
        _create_mapping_tuple("out/control_cmd"),
    ]

    parameters = [
        dict(  # noqa: C406 for using helper function
            [
                _create_mapping_tuple("csv_path_accel_map"),
                _create_mapping_tuple("csv_path_brake_map"),
                _create_mapping_tuple("ref_vel_gain"),
                _create_mapping_tuple("timer_rate"),
                _create_mapping_tuple("wait_for_first_topic"),
                _create_mapping_tuple("control_command_timeout"),
                _create_mapping_tuple("emergency_stop_timeout"),
            ]
        )
    ]

    node = Node(
        package="autoware_external_cmd_converter",
        executable="external_cmd_converter_node",
        name="external_cmd_converter",
        remappings=remappings,
        parameters=parameters,
        output="screen",
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
    )

    return LaunchDescription(arguments + [agnocast_env, node])
