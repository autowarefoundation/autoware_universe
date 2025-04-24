# Copyright 2024 TIER IV, Inc.
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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


def create_traffic_light_camera_info_relay(namespace):
    relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="traffic_light_camera_info_relay",
        arguments=[f"/sensing/camera/{namespace}/camera_info", "camera_info"],
    )

    group = GroupAction(
        [
            PushRosNamespace(namespace),
            relay_node,
        ]
    )

    return group


def launch_setup(context, *args, **kwargs):
    # Load camera namespaces
    camera_namespaces = LaunchConfiguration("camera_namespaces").perform(context)

    # Convert string to list
    camera_namespaces = yaml.load(camera_namespaces, Loader=yaml.FullLoader)
    if not isinstance(camera_namespaces, list):
        raise ValueError(
            "camera_namespaces is not a list. You should declare it like `['camera6', 'camera7']`."
        )
    if not all((isinstance(v, str) for v in camera_namespaces)):
        raise ValueError(
            "camera_namespaces is not a list of strings. You should declare it like `['camera6', 'camera7']`."
        )

    # Create containers for all cameras
    traffic_light_recognition_containers = [
        create_traffic_light_camera_info_relay(namespace) for namespace in camera_namespaces
    ]
    return traffic_light_recognition_containers


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("camera_namespaces"),
            OpaqueFunction(function=launch_setup),
        ]
    )
