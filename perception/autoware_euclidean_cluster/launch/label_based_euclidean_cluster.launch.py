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

from dataclasses import dataclass
from dataclasses import field

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


@dataclass(frozen=True)
class LaunchArgument:
    """Represents a topic argument for the label based euclidean cluster node.

    Attributes:
        name (str): The name of the ROS parameter.
        default (str | list): The default value of the parameter.
    """

    name: str
    default: str | list
    config: LaunchConfiguration = field(init=False)
    remapping: tuple[str, LaunchConfiguration] = field(init=False)

    def __post_init__(self):
        object.__setattr__(self, "config", LaunchConfiguration(self.name, default=self.default))
        object.__setattr__(self, "remapping", (self.name, self.config))

    def declare(self) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(self.name, default_value=self.default)


# === Node information ===
PACKAGE_NAME = "autoware_euclidean_cluster"
PLUGIN_NAME = "autoware::euclidean_cluster::LabelBasedEuclideanClusterNode"
NODE_NAME = "label_based_euclidean_cluster"

# === Launch arguments ===
INPUT_POINTCLOUD = LaunchArgument("input/pointcloud", "~/input/segmented/pointcloud")
OUTPUT_OBJECTS = LaunchArgument("output/objects", "~/output/objects")
OUTPUT_POINTCLOUD = LaunchArgument("output/pointcloud", "~/output/pointcloud")

PARAM_PATH = LaunchArgument(
    "param_path",
    [
        FindPackageShare(PACKAGE_NAME),
        "/config/label_based_euclidean_cluster.param.yaml",
    ],
)

USE_POINTCLOUD_CONTAINER = LaunchArgument("use_pointcloud_container", "false")
POINTCLOUD_CONTAINER_NAME = LaunchArgument("pointcloud_container_name", "pointcloud_container")


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    ns = ""
    component = ComposableNode(
        package=PACKAGE_NAME,
        namespace=ns,
        plugin=PLUGIN_NAME,
        name=NODE_NAME,
        remappings=[
            INPUT_POINTCLOUD.remapping,
            OUTPUT_OBJECTS.remapping,
            OUTPUT_POINTCLOUD.remapping,
        ],
        parameters=[load_composable_node_param(PARAM_PATH.name)],
    )

    container = ComposableNodeContainer(
        name=NODE_NAME + "_container",
        namespace=ns,
        package=LaunchConfiguration("container_package"),
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        output="screen",
        condition=UnlessCondition(USE_POINTCLOUD_CONTAINER.config),
        additional_env={
            "LD_PRELOAD": LaunchConfiguration("ld_preload_value"),
        },
    )

    target_container = (
        POINTCLOUD_CONTAINER_NAME.config
        if IfCondition(USE_POINTCLOUD_CONTAINER.config).evaluate(context)
        else container
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[component],
        target_container=target_container,
    )

    return [container, loader]


def generate_launch_description():
    # Resolve LD_PRELOAD / container package / container executable based on ENABLE_AGNOCAST.
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

    return launch.LaunchDescription(
        [
            agnocast_env,
            # I/O topics
            INPUT_POINTCLOUD.declare(),
            OUTPUT_OBJECTS.declare(),
            OUTPUT_POINTCLOUD.declare(),
            # Parameters
            PARAM_PATH.declare(),
            # Container
            USE_POINTCLOUD_CONTAINER.declare(),
            POINTCLOUD_CONTAINER_NAME.declare(),
            OpaqueFunction(function=launch_setup),
        ]
    )
