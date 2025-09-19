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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import yaml


def create_traffic_light_occlusion_predictor(namespace):
    package = FindPackageShare("autoware_traffic_light_occlusion_predictor")
    include = PathJoinSubstitution([package, "launch/traffic_light_occlusion_predictor.launch.xml"])

    arguments = {
        "input/cloud": LaunchConfiguration("input/cloud"),
        "input/vector_map": LaunchConfiguration("input/vector_map"),
        "input/camera_info": f"/sensing/camera/{namespace}/camera_info",
        "input/rois": f"/perception/traffic_light_recognition/{namespace}/detection/rois",
        "input/car/traffic_signals": "car/traffic_signals",
        "input/pedestrian/traffic_signals": "pedestrian/traffic_signals",
        "output/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/traffic_signals",
        "param_path": LaunchConfiguration("param_path"),
    }.items()

    group = GroupAction(
        [
            PushRosNamespace(namespace),
            PushRosNamespace("classification"),
            IncludeLaunchDescription(include, launch_arguments=arguments),
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
        create_traffic_light_occlusion_predictor(namespace) for namespace in camera_namespaces
    ]
    return traffic_light_recognition_containers


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("camera_namespaces")
    add_launch_arg("input/cloud")
    add_launch_arg("input/vector_map")
    add_launch_arg("param_path")

    return launch.LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
