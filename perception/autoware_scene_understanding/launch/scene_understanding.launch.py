# Copyright 2025 TIER IV, Inc.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    onnx_model_path_arg = DeclareLaunchArgument(
        "onnx_model_path",
        default_value="/opt/autoware/mlmodels/scene_understanding_model.onnx",
        description="Path to ONNX model file",
    )

    input_image_topic_arg = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/sensing/camera/camera0/image_raw",
        description="Input image topic",
    )

    scene_topic_arg = DeclareLaunchArgument(
        "scene_topic",
        default_value="/scene_understanding",
        description="Output scene understanding topic",
    )

    output_image_topic_arg = DeclareLaunchArgument(
        "output_image_topic",
        default_value="/scene_visualization",
        description="Output visualization topic",
    )

    use_fp16_arg = DeclareLaunchArgument(
        "use_fp16", default_value="true", description="Enable FP16 precision"
    )

    use_int8_arg = DeclareLaunchArgument(
        "use_int8", default_value="false", description="Enable INT8 precision"
    )

    # Node
    scene_understanding_node = Node(
        package="scene_understanding_tensorrt",
        executable="scene_understanding_node",
        name="scene_understanding_node",
        output="screen",
        parameters=[
            {
                "onnx_model_path": LaunchConfiguration("onnx_model_path"),
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "output_topic": LaunchConfiguration("scene_topic"),
                "use_fp16": LaunchConfiguration("use_fp16"),
                "use_int8": LaunchConfiguration("use_int8"),
                "max_batch_size": 1,
            }
        ],
        remappings=[
            ("image_raw", LaunchConfiguration("input_image_topic")),
            ("scene_understanding", LaunchConfiguration("scene_topic")),
        ],
    )

    visualization_node = Node(
        package="scene_understanding_tensorrt",
        executable="scene_visualization_node",
        name="scene_visualization_node",
        output="screen",
        parameters=[
            {
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "input_scene_topic": LaunchConfiguration("scene_topic"),
                "output_image_topic": LaunchConfiguration("output_image_topic"),
                "font_scale": 1.5,
                "font_thickness": 3,
                "line_spacing": 60,
            }
        ],
    )

    return LaunchDescription(
        [
            onnx_model_path_arg,
            input_image_topic_arg,
            scene_topic_arg,
            output_image_topic_arg,
            use_fp16_arg,
            use_int8_arg,
            scene_understanding_node,
            visualization_node,
        ]
    )
