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

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch the pointcloud densifier node."""
    pkg_prefix = get_package_share_directory("autoware_pointcloud_preprocessor")
    config_file = os.path.join(pkg_prefix, "config/pointcloud_densifier.param.yaml")

    input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/sensing/lidar/concatenated/pointcloud",
        description="Input pointcloud topic",
    )

    output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/sensing/lidar/densified/pointcloud",
        description="Output pointcloud topic",
    )

    # Create the composable node
    component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::PointCloudDensifierNode",
        name="pointcloud_densifier",
        remappings=[
            ("input", LaunchConfiguration("input_topic")),
            ("output", LaunchConfiguration("output_topic")),
        ],
        parameters=[config_file],
    )

    # Create container
    container = ComposableNodeContainer(
        name="pointcloud_densifier_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[component],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            # Launch arguments
            input_topic,
            output_topic,
            # Nodes
            container,
        ]
    )
