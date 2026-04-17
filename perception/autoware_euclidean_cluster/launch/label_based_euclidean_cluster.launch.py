# Copyright 2026 TIER IV, Inc.
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
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import AnonName
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    ns = ""
    component = ComposableNode(
        package="autoware_euclidean_cluster",
        namespace=ns,
        plugin="autoware::euclidean_cluster::LabelBasedEuclideanClusterNode",
        name=AnonName("label_based_euclidean_cluster"),
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", LaunchConfiguration("output_objects")),
        ],
        parameters=[load_composable_node_param("param_path")],
    )

    container = ComposableNodeContainer(
        name="label_based_euclidean_cluster_container",
        namespace=ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    target_container = (
        LaunchConfiguration("pointcloud_container_name")
        if IfCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else container
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[component],
        target_container=target_container,
    )

    return [container, loader]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("input_pointcloud", "/perception/ptv3/segmented/pointcloud"),
            add_launch_arg("output_objects", "objects"),
            add_launch_arg("use_pointcloud_container", "false"),
            add_launch_arg("pointcloud_container_name", "pointcloud_container"),
            add_launch_arg(
                "param_path",
                [
                    FindPackageShare("autoware_euclidean_cluster"),
                    "/config/label_based_euclidean_cluster.param.yaml",
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )