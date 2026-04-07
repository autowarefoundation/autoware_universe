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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
import yaml

# This launch file is intended for simulation usage.


def use_actuation_command(context, *args, **kwargs) -> list:
    simulator_model_param_path = LaunchConfiguration("simulator_model_param_file").perform(context)
    use_actuation_command = "false"

    if simulator_model_param_path:
        try:
            with open(simulator_model_param_path, "r") as f:
                simulator_model_param_yaml = yaml.safe_load(f) or {}
            params = simulator_model_param_yaml.get("/**", {}).get("ros__parameters", {})
            vehicle_model_type = params.get("vehicle_model_type", "")
            if vehicle_model_type.startswith("ACTUATION_CMD"):
                use_actuation_command = "true"
        except Exception:
            use_actuation_command = "false"

    return [
        SetLaunchConfiguration(name="use_actuation_command", value=use_actuation_command),
    ]


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

    return LaunchDescription(launch_arguments + [OpaqueFunction(function=use_actuation_command)])
