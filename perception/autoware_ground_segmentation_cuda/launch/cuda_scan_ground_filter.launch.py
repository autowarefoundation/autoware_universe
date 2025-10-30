# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    ground_segmentation_node_param = ParameterFile(
        param_file=LaunchConfiguration("cuda_ground_segmentation_node_param_path").perform(context),
        allow_substs=True,
    )

    nodes = [
        ComposableNode(
            package="autoware_ground_segmentation_cuda",
            plugin="autoware::cuda_ground_segmentation::CudaScanGroundSegmentationFilterNode",
            name="cuda_scan_ground_segmentation_filter",
            remappings=[
                ("~/input/pointcloud", LaunchConfiguration("input/pointcloud")),
                ("~/output/pointcloud", LaunchConfiguration("output/pointcloud")),
            ],
            parameters=[ground_segmentation_node_param],
            extra_arguments=[],
        ),
    ]

    loader = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals("container", ""),
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container"),
    )

    container = ComposableNodeContainer(
        name="scan_ground_filter_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
        condition=LaunchConfigurationEquals("container", ""),
    )

    group = GroupAction(
        [
            container,
            loader,
        ]
    )

    return [group]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("container", ""),
            add_launch_arg("input/pointcloud", "/sensing/lidar/concatenated/pointcloud"),
            add_launch_arg("output/pointcloud", "/perception/obstacle_segmentation/pointcloud"),
            add_launch_arg(
                "cuda_ground_segmentation_node_param_path",
                [
                    FindPackageShare("autoware_ground_segmentation_cuda"),
                    "/config/cuda_scan_ground_segmentation_filter.param.yaml",
                ],
            ),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
