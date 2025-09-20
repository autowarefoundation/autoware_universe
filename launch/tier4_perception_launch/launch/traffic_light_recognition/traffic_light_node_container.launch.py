# Copyright 2020 Tier IV, Inc. All rights reserved.
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

from itertools import chain

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
import yaml


def launch_setup(context, *args, **kwargs):
    high_accuracy_detection_type = LaunchConfiguration("high_accuracy_detection_type").perform(
        context
    )
    assert high_accuracy_detection_type in [
        "whole_image_detection",
        "fine_detection",
    ], "high_accuracy_detection_type must be either 'whole_image_detection' or 'fine_detection'."

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
        create_traffic_light_node_container(namespace, context, *args, **kwargs)
        for namespace in camera_namespaces
    ]
    traffic_light_recognition_containers = list(chain(*traffic_light_recognition_containers))

    return traffic_light_recognition_containers


def create_traffic_light_node_container(namespace, context, *args, **kwargs):
    camera_arguments = {
        "input/camera_info": f"/sensing/camera/{namespace}/camera_info",
        "input/image": f"/sensing/camera/{namespace}/image_raw",
        "output/rois": f"/perception/traffic_light_recognition/{namespace}/detection/rois",
        "output/car/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/car/traffic_signals",
        "output/pedestrian/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/pedestrian/traffic_signals",
        "output/traffic_signals": f"/perception/traffic_light_recognition/{namespace}/classification/traffic_signals",
    }

    # parameter files
    traffic_light_whole_image_detector_param = ParameterFile(
        param_file=LaunchConfiguration("yolox_traffic_light_detector_param_path").perform(context),
        allow_substs=True,
    )
    traffic_light_fine_detector_param = ParameterFile(
        param_file=LaunchConfiguration("traffic_light_fine_detector_param_path").perform(context),
        allow_substs=True,
    )
    car_traffic_light_classifier_param = ParameterFile(
        param_file=LaunchConfiguration("car_traffic_light_classifier_param_path").perform(context),
        allow_substs=True,
    )
    pedestrian_traffic_light_classifier_param = ParameterFile(
        param_file=LaunchConfiguration("pedestrian_traffic_light_classifier_param_path").perform(
            context
        ),
        allow_substs=True,
    )
    traffic_light_roi_visualizer_param = ParameterFile(
        param_file=LaunchConfiguration("traffic_light_roi_visualizer_param_path").perform(context),
        allow_substs=True,
    )

    container = ComposableNodeContainer(
        name="traffic_light_node_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_traffic_light_classifier",
                plugin="autoware::traffic_light::TrafficLightClassifierNodelet",
                name="car_traffic_light_classifier",
                namespace="classification",
                parameters=[
                    car_traffic_light_classifier_param,
                    {
                        "build_only": False,
                        "label_path": LaunchConfiguration("classification/car/label_path"),
                        "model_path": LaunchConfiguration("classification/car/model_path"),
                    },
                ],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", camera_arguments["output/rois"]),
                    ("~/output/traffic_signals", "car/traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_classifier",
                plugin="autoware::traffic_light::TrafficLightClassifierNodelet",
                name="pedestrian_traffic_light_classifier",
                namespace="classification",
                parameters=[
                    pedestrian_traffic_light_classifier_param,
                    {
                        "build_only": False,
                        "label_path": LaunchConfiguration("classification/pedestrian/label_path"),
                        "model_path": LaunchConfiguration("classification/pedestrian/model_path"),
                    },
                ],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", camera_arguments["output/rois"]),
                    ("~/output/traffic_signals", "pedestrian/traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_visualization",
                plugin="autoware::traffic_light::TrafficLightRoiVisualizerNode",
                name="traffic_light_roi_visualizer",
                parameters=[
                    traffic_light_roi_visualizer_param,
                    {
                        "use_high_accuracy_detection": LaunchConfiguration(
                            "use_high_accuracy_detection"
                        )
                    },
                ],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", camera_arguments["output/rois"]),
                    ("~/input/rough/rois", "detection/rough/rois"),
                    (
                        "~/input/traffic_signals",
                        camera_arguments["output/traffic_signals"],
                    ),
                    ("~/output/image", "debug/rois"),
                    ("~/output/image/compressed", "debug/rois/compressed"),
                    ("~/output/image/compressedDepth", "debug/rois/compressedDepth"),
                    ("~/output/image/theora", "debug/rois/theora"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        output="both",
    )

    decompressor_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_image_transport_decompressor",
                plugin="autoware::image_preprocessor::ImageTransportDecompressor",
                name="traffic_light_image_decompressor",
                namespace=namespace,
                parameters=[{"encoding": "rgb8"}],
                remappings=[
                    (
                        "~/input/compressed_image",
                        [camera_arguments["input/image"], "/compressed"],
                    ),
                    ("~/output/raw_image", camera_arguments["input/image"]),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_image_decompressor")),
    )

    fine_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_traffic_light_fine_detector",
                plugin="autoware::traffic_light::TrafficLightFineDetectorNode",
                name="traffic_light_fine_detector",
                namespace=f"{namespace}/detection",
                parameters=[
                    traffic_light_fine_detector_param,
                    {
                        "build_only": False,
                        "label_path": LaunchConfiguration("fine_detection/label_path"),
                        "model_path": LaunchConfiguration("fine_detection/model_path"),
                    },
                ],
                remappings=[
                    ("~/input/image", camera_arguments["input/image"]),
                    ("~/input/rois", "rough/rois"),
                    ("~/expect/rois", "expect/rois"),
                    ("~/output/rois", camera_arguments["output/rois"]),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("high_accuracy_detection_type"),
                    "' == 'fine_detection' ",
                ]
            )
        ),
    )

    internal_node_name = "traffic_light_whole_image_detector"
    whole_img_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="autoware_tensorrt_yolox",
                plugin="autoware::tensorrt_yolox::TrtYoloXNode",
                name=internal_node_name,
                namespace=f"{namespace}/detection",
                parameters=[
                    traffic_light_whole_image_detector_param,
                    {
                        "build_only": False,
                        "label_path": LaunchConfiguration("whole_image_detection/label_path"),
                        "model_path": LaunchConfiguration("whole_image_detection/model_path"),
                        "color_map_path": "",  # not used
                    },
                ],
                remappings=[
                    ("~/in/image", camera_arguments["input/image"]),
                    ("~/out/objects", internal_node_name + "/rois"),
                    ("~/out/image", internal_node_name + "/debug/image"),
                    (
                        "~/out/image/compressed",
                        internal_node_name + "/debug/image/compressed",
                    ),
                    (
                        "~/out/image/compressedDepth",
                        internal_node_name + "/debug/image/compressedDepth",
                    ),
                    ("~/out/image/theora", internal_node_name + "/debug/image/theora"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_selector",
                plugin="autoware::traffic_light::TrafficLightSelectorNode",
                name="traffic_light_selector",
                namespace=f"{namespace}/detection",
                parameters=[],
                remappings=[
                    ("input/detected_rois", internal_node_name + "/rois"),
                    ("input/rough_rois", "rough/rois"),
                    ("input/expect_rois", "expect/rois"),
                    ("input/camera_info", camera_arguments["input/camera_info"]),
                    ("output/traffic_rois", camera_arguments["output/rois"]),
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_category_merger",
                plugin="autoware::traffic_light::TrafficLightCategoryMergerNode",
                name="traffic_light_category_merger",
                namespace=f"{namespace}/classification",
                parameters=[],
                remappings=[
                    ("input/car_signals", "car/traffic_signals"),
                    ("input/pedestrian_signals", "pedestrian/traffic_signals"),
                    ("output/traffic_signals", camera_arguments["output/traffic_signals"]),
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("high_accuracy_detection_type"),
                    "' == 'whole_image_detection' ",
                ]
            )
        ),
    )

    return [
        GroupAction([PushRosNamespace(namespace), container]),
        decompressor_loader,
        fine_detector_loader,
        whole_img_detector_loader,
    ]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("enable_image_decompressor")
    add_launch_arg("camera_namespaces")
    add_launch_arg("use_high_accuracy_detection")
    add_launch_arg("high_accuracy_detection_type")

    # whole image detector by yolox
    add_launch_arg("whole_image_detection/model_path")
    add_launch_arg("whole_image_detection/label_path")
    add_launch_arg("yolox_traffic_light_detector_param_path")

    # traffic_light_fine_detector
    add_launch_arg("fine_detection/model_path")
    add_launch_arg("fine_detection/label_path")
    add_launch_arg("traffic_light_fine_detector_param_path")

    # traffic_light_classifier
    add_launch_arg("classification/car/model_path")
    add_launch_arg("classification/car/label_path")
    add_launch_arg("classification/pedestrian/model_path")
    add_launch_arg("classification/pedestrian/label_path")
    add_launch_arg("car_traffic_light_classifier_param_path")
    add_launch_arg("pedestrian_traffic_light_classifier_param_path")

    # traffic_light_roi_visualizer
    add_launch_arg("traffic_light_roi_visualizer_param_path")

    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_multithread", "False")

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

    return LaunchDescription(
        [
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            OpaqueFunction(function=launch_setup),
        ]
    )
