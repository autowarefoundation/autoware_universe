# Copyright 2025 The Autoware Foundation.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import yaml

# This launch file is intended for simulation usage.


def _should_launch_converter(simulator_model_param_path: str) -> bool:
    if not simulator_model_param_path:
        return False

    try:
        with open(simulator_model_param_path, "r") as f:
            simulator_model_param_yaml = yaml.safe_load(f) or {}
        params = simulator_model_param_yaml.get("/**", {}).get("ros__parameters", {})
        vehicle_model_type = params.get("vehicle_model_type", "")
        return "ACTUATION_CMD" in vehicle_model_type
    except Exception:
        return False


def launch_setup(context, *args, **kwargs):
    simulator_model_param_path = LaunchConfiguration("simulator_model_param_file").perform(
        context
    )
    if not _should_launch_converter(simulator_model_param_path):
        return []

    raw_vehicle_converter_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [
                FindPackageShare("autoware_raw_vehicle_cmd_converter"),
                "/launch/raw_vehicle_converter.launch.xml",
            ]
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("raw_vehicle_cmd_converter_param_path").perform(
                context
            ),
        }.items(),
    )

    return [raw_vehicle_converter_node]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "simulator_model_param_file",
        "",
        "path to config file for simulator_model",
    )

    add_launch_arg(
        "raw_vehicle_cmd_converter_param_path",
        [
            FindPackageShare("autoware_raw_vehicle_cmd_converter"),
            "/config/raw_vehicle_cmd_converter.param.yaml",
        ],
        "path to config file for raw_vehicle_cmd_converter",
    )

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
