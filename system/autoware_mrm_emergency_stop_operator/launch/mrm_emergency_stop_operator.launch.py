# Copyright 2022 The Autoware Contributors
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    launch_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=[
                FindPackageShare("autoware_mrm_emergency_stop_operator"),
                "/config/mrm_emergency_stop_operator.param.yaml",
            ],
            description="path to the parameter file of mrm_emergency_stop_operator",
        )
    ]

    node = Node(
        package="autoware_mrm_emergency_stop_operator",
        executable="autoware_mrm_emergency_stop_operator_node",
        name="mrm_emergency_stop_operator",
        parameters=[
            LaunchConfiguration("config_file"),
        ],
        remappings=[
            ("~/input/mrm/emergency_stop/operate", "/system/mrm/emergency_stop/operate"),
            ("~/input/control/control_cmd", "/control/command/control_cmd"),
            ("~/input/driving_mode_request", "/system/driving_mode/request"),
            ("~/input/driving_mode_info", "/system/driving_mode/info"),
            ("~/output/mrm_state", "/system/driving_mode/mrm_state"),
            ("~/output/mrm/emergency_stop/status", "/system/mrm/emergency_stop/status"),
            ("~/output/mrm/emergency_stop/control_cmd", "/system/emergency/control_cmd"),
        ],
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
        output="screen",
    )

    return launch.LaunchDescription(launch_arguments + [agnocast_env, node])
