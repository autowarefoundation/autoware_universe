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
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


class SmallUnknownPipeline:
    def __init__(self, context):
        self.context = context
        self.vehicle_info = self.get_vehicle_info()
        with open(
            LaunchConfiguration("irregular_object_detector_param_path").perform(context), "r"
        ) as f:
            self.irregular_object_detector_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        with open(LaunchConfiguration("sync_param_path").perform(context), "r") as f:
            self.roi_pointcloud_fusion_sync_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        self.roi_pointcloud_fusion_param = self.irregular_object_detector_param[
            "roi_pointcloud_fusion"
        ]["parameters"]

        self.camera_ids = LaunchConfiguration("fusion_camera_ids").perform(context)
        # convert string to list
        self.camera_ids = yaml.load(self.camera_ids, Loader=yaml.FullLoader)
        self.roi_pointcloud_fusion_param["rois_number"] = len(self.camera_ids)
        rois_timestamp_offsets = []
        approximate_camera_projection = []
        rois_timestamp_noise_window = []
        approximate_camera_projection = []
        point_project_to_unrectified_image = []

        for index, camera_id in enumerate(self.camera_ids):
            rois_timestamp_offsets.append(
                self.roi_pointcloud_fusion_sync_param["rois_timestamp_offsets"][camera_id]
            )
            rois_timestamp_noise_window.append(
                self.roi_pointcloud_fusion_sync_param["matching_strategy"][
                    "rois_timestamp_noise_window"
                ][camera_id]
            )
            approximate_camera_projection.append(
                self.roi_pointcloud_fusion_sync_param["approximate_camera_projection"][camera_id]
            )
            point_project_to_unrectified_image.append(
                self.roi_pointcloud_fusion_sync_param["point_project_to_unrectified_image"][
                    camera_id
                ]
            )
            self.roi_pointcloud_fusion_param[f"input/rois{index}"] = (
                f"/perception/object_recognition/detection/rois{camera_id}"
            )
            self.roi_pointcloud_fusion_param[f"input/camera_info{index}"] = (
                f"/sensing/camera/camera{camera_id}/camera_info"
            )
            self.roi_pointcloud_fusion_param[f"input/image{index}"] = (
                f"/sensing/camera/camera{camera_id}/image_raw"
            )

        self.roi_pointcloud_fusion_sync_param["rois_timestamp_offsets"] = rois_timestamp_offsets
        self.roi_pointcloud_fusion_sync_param["approximate_camera_projection"] = (
            approximate_camera_projection
        )
        self.roi_pointcloud_fusion_sync_param["matching_strategy"][
            "rois_timestamp_noise_window"
        ] = rois_timestamp_noise_window
        self.roi_pointcloud_fusion_sync_param["approximate_camera_projection"] = (
            approximate_camera_projection
        )
        self.roi_pointcloud_fusion_sync_param["point_project_to_unrectified_image"] = (
            point_project_to_unrectified_image
        )

    def get_vehicle_info(self):
        # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
        # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
        gp = self.context.launch_configurations.get("ros_params", {})
        if not gp:
            gp = dict(self.context.launch_configurations.get("global_params", {}))
        p = {}
        p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
        p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
        p["min_longitudinal_offset"] = -gp["rear_overhang"]
        p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
        p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
        p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
        p["min_height_offset"] = 0.0
        p["max_height_offset"] = gp["vehicle_height"]
        return p

    def create_irregular_object_pipeline(self, input_topic, output_topic):
        components = []
        # create cropbox filter
        components.append(
            ComposableNode(
                package="autoware_pointcloud_preprocessor",
                plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
                name="crop_box_filter",
                remappings=[("input", input_topic), ("output", "cropped_range/pointcloud")],
                parameters=[
                    {
                        "input_frame": LaunchConfiguration("base_frame"),
                        "output_frame": LaunchConfiguration("base_frame"),
                    },
                    self.irregular_object_detector_param["crop_box_filter"]["parameters"],
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        # create ground_segmentation
        components.append(
            ComposableNode(
                package="autoware_ground_segmentation",
                plugin="autoware::ground_segmentation::ScanGroundFilterComponent",
                name="ground_filter",
                remappings=[
                    ("input", "cropped_range/pointcloud"),
                    ("output", "obstacle_segmentation/pointcloud"),
                ],
                parameters=[
                    self.irregular_object_detector_param["ground_segmentation"]["parameters"]
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )
        return components

    def create_roi_pointcloud_fusion_node(self, input_topic, output_topic):
        node = Node(
            package="autoware_image_projection_based_fusion",
            executable="roi_pointcloud_fusion_node",
            name="roi_pointcloud_fusion",
            remappings=[
                ("input", input_topic),
                ("output", output_topic),
            ],
            parameters=[
                self.roi_pointcloud_fusion_sync_param,
                self.roi_pointcloud_fusion_param,
            ],
        )
        return node


def launch_setup(context, *args, **kwargs):
    obstacle_pointcloud_topic = "obstacle_segmentation/pointcloud"
    pipeline = SmallUnknownPipeline(context)
    components = []
    components.extend(
        pipeline.create_irregular_object_pipeline(
            LaunchConfiguration("input/pointcloud"), obstacle_pointcloud_topic
        )
    )
    loader = LoadComposableNodes(
        composable_node_descriptions=components,
        target_container=LaunchConfiguration("pointcloud_container_name"),
    )
    roi_pointcloud_fusion_node = pipeline.create_roi_pointcloud_fusion_node(
        obstacle_pointcloud_topic, LaunchConfiguration("output_topic")
    )
    return [loader, roi_pointcloud_fusion_node]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("input/pointcloud", "/sensing/lidar/concatenated/pointcloud")
    add_launch_arg(
        "output_topic", "/perception/object_recognition/detection/irregular_object/clusters"
    )
    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_multithread", "True")
    add_launch_arg("fusion_camera_ids", "[3,5]")
    add_launch_arg("image_topic_name", "image_raw")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg("use_pointcloud_container", "True")
    add_launch_arg(
        "irregular_object_detector_param_path",
        [
            FindPackageShare("autoware_launch"),
            "/config/perception/object_recognition/detection/irregular_object_detection/irregular_object_detector.param.yaml",
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
