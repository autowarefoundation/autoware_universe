# Copyright 2025 Tier IV, Inc. All rights reserved.
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
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


class SegmentationPointcloudFusion:
    def __init__(self, context):
        self.context = context
        with open(LaunchConfiguration("sync_param_path").perform(context), "r") as f:
            self.segmentation_pointcloud_fusion_sync_param = yaml.safe_load(f)["/**"][
                "ros__parameters"
            ]

        with open(
            LaunchConfiguration("segmentation_pointcloud_fusion_param_path").perform(context), "r"
        ) as f:
            self.segmentation_pointcloud_fusion_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        self.camera_ids = LaunchConfiguration("fusion_camera_ids").perform(context)
        # convert string to list
        self.camera_ids = yaml.load(self.camera_ids, Loader=yaml.FullLoader)
        self.segmentation_pointcloud_fusion_param["rois_number"] = len(self.camera_ids)
        mask_timestamp_offsets = []
        approximate_camera_projection = []
        rois_timestamp_noise_window = []
        point_project_to_unrectified_image = []
        image_topic_name = LaunchConfiguration("image_topic_name").perform(context)

        for index, camera_id in enumerate(self.camera_ids):
            mask_timestamp_offsets.append(
                self.segmentation_pointcloud_fusion_sync_param["rois_timestamp_offsets"][camera_id]
            )
            rois_timestamp_noise_window.append(
                self.segmentation_pointcloud_fusion_sync_param["matching_strategy"][
                    "rois_timestamp_noise_window"
                ][camera_id]
            )
            approximate_camera_projection.append(
                self.segmentation_pointcloud_fusion_sync_param["approximate_camera_projection"][
                    camera_id
                ]
            )
            point_project_to_unrectified_image.append(
                self.segmentation_pointcloud_fusion_sync_param[
                    "point_project_to_unrectified_image"
                ][camera_id]
            )
            self.segmentation_pointcloud_fusion_param[f"input/rois{index}"] = (
                f"/perception/object_recognition/detection/mask{camera_id}"
            )
            self.segmentation_pointcloud_fusion_param[f"input/camera_info{index}"] = (
                f"/sensing/camera/camera{camera_id}/camera_info"
            )
            self.segmentation_pointcloud_fusion_param[f"input/image{index}"] = (
                f"/sensing/camera/camera{camera_id}/{image_topic_name}"
            )

        self.segmentation_pointcloud_fusion_sync_param["rois_timestamp_offsets"] = (
            mask_timestamp_offsets
        )
        self.segmentation_pointcloud_fusion_sync_param["approximate_camera_projection"] = (
            approximate_camera_projection
        )
        self.segmentation_pointcloud_fusion_sync_param["matching_strategy"][
            "rois_timestamp_noise_window"
        ] = rois_timestamp_noise_window
        self.segmentation_pointcloud_fusion_sync_param["approximate_camera_projection"] = (
            approximate_camera_projection
        )
        self.segmentation_pointcloud_fusion_sync_param["point_project_to_unrectified_image"] = (
            point_project_to_unrectified_image
        )

    def create_segmentation_pointcloud_fusion_node(self, input_topic, output_topic):
        node = Node(
            package="autoware_image_projection_based_fusion",
            executable="segmentation_pointcloud_fusion_node",
            name="segmentation_pointcloud_fusion",
            remappings=[
                ("input", input_topic),
                ("output", output_topic),
            ],
            parameters=[
                self.segmentation_pointcloud_fusion_sync_param,
                self.segmentation_pointcloud_fusion_param,
            ],
        )
        return node


def launch_setup(context, *args, **kwargs):
    pipeline = SegmentationPointcloudFusion(context)
    segmentation_pointcloud_fusion_node = pipeline.create_segmentation_pointcloud_fusion_node(
        LaunchConfiguration("input/pointcloud"), LaunchConfiguration("output/pointcloud")
    )
    # TODO(badai-nguyen): add option of using container
    return [segmentation_pointcloud_fusion_node]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("input/pointcloud", "pointcloud_map_filtered/pointcloud")
    add_launch_arg("output/pointcloud", "segmentation_based_filtered/pointcloud")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_multithread", "True")
    add_launch_arg("fusion_camera_ids", "[0]")
    add_launch_arg("image_topic_name", "image_raw")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("use_pointcloud_container", "True")
    add_launch_arg(
        "segmentation_pointcloud_fusion_param_path",
        [
            FindPackageShare("autoware_image_projection_based_fusion"),
            "/config/segmentation_pointcloud_fusion.param.yaml",
        ],
    )
    add_launch_arg(
        "sync_param_path",
        [
            FindPackageShare("autoware_launch"),
            "/config/perception/object_recognition/detection/image_projection_based_fusion/fusion_common.param.yaml",
        ],
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )
    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
