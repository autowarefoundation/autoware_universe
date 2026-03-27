# Copyright 2025 TIER IV, Inc.
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
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration


def current_ros_namespace(context):
    namespace = context.launch_configurations.get("ros_namespace", "")
    return [SetLaunchConfiguration("current_ros_namespace", namespace)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=current_ros_namespace)])
