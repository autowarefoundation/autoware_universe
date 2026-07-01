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
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    config_file_path = LaunchConfiguration("config_file").perform(context)
    with open(config_file_path, "r") as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]

    # An agnocast_wrapper::Node registered with an AgnocastOnly* executor must run as its own
    # standalone executable: the agnocast runtime (signal handler / shutdown eventfd) is set up
    # only by the generated node_main, so loading it as a composable node into a container
    # SIGSEGVs on construction. LD_PRELOAD supplies the heaphook, required by AgnocastOnly.
    node = Node(
        package="autoware_mrm_comfortable_stop_operator",
        executable="autoware_mrm_comfortable_stop_operator_node",
        name="mrm_comfortable_stop_operator",
        parameters=[
            params,
        ],
        remappings=[
            ("~/input/mrm/comfortable_stop/operate", "/system/mrm/comfortable_stop/operate"),
            ("~/output/mrm/comfortable_stop/status", "/system/mrm/comfortable_stop/status"),
            ("~/output/velocity_limit", "/planning/scenario_planning/max_velocity_candidates"),
            ("~/output/velocity_limit/clear", "/planning/scenario_planning/clear_velocity_limit"),
            ("~/input/driving_mode_request", "/system/driving_mode/request"),
            ("~/input/driving_mode_info", "/system/driving_mode/info"),
            ("~/output/mrm_state", "/system/driving_mode/mrm_state"),
        ],
        additional_env={"LD_PRELOAD": LaunchConfiguration("ld_preload_value")},
        output="screen",
    )

    return [node]


def generate_launch_description():
    # Resolves the LD_PRELOAD value (agnocast heaphook) based on the ENABLE_AGNOCAST environment
    # variable. Stays backward compatible when Agnocast is disabled.
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
                FindPackageShare("autoware_mrm_comfortable_stop_operator"),
                "/config/mrm_comfortable_stop_operator.param.yaml",
            ],
            description="path to the parameter file of mrm_comfortable_stop_operator",
        )
    ]

    return launch.LaunchDescription(
        launch_arguments + [agnocast_env, OpaqueFunction(function=launch_setup)]
    )
