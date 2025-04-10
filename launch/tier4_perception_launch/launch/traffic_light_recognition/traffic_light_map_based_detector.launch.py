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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import yaml


def create_traffic_light_map_based_detector(namespace, context):
    package = FindPackageShare("autoware_traffic_light_map_based_detector")
    include = PathJoinSubstitution([package, "launch/traffic_light_map_based_detector.launch.xml"])

    output_rois = (
        "rough/rois"
        if IfCondition(LaunchConfiguration("use_high_accuracy_detection")).evaluate(context)
        else f"/perception/traffic_light_recognition/{namespace}/detection/rois"
    )

    each_namespace_param_path = LaunchConfiguration("param_path").perform(context)
    each_namespace_param_path = each_namespace_param_path.replace(
        "TRAFFIC_LIGHT_RECOGNITION_CAMERA_NAMESPACE", namespace
    )
    arguments = {
        "input/vector_map": LaunchConfiguration("input/vector_map"),
        "input/camera_info": f"/sensing/camera/{namespace}/camera_info",
        "input/route": LaunchConfiguration("input/route"),
        "expect/rois": "expect/rois",
        "output/rois": output_rois,
        "param_path": each_namespace_param_path,
    }.items()

    group = GroupAction(
        [
            PushRosNamespace(namespace),
            PushRosNamespace("detection"),
            IncludeLaunchDescription(include, launch_arguments=arguments),
        ]
    )

    return group


def launch_setup(context, *args, **kwargs):
    # Load all camera namespaces
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
        create_traffic_light_map_based_detector(namespace, context)
        for namespace in camera_namespaces
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
    add_launch_arg("input/vector_map")
    add_launch_arg("input/route")
    add_launch_arg("use_high_accuracy_detection")
    add_launch_arg("param_path")

    return launch.LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
