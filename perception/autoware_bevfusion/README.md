# autoware_bevfusion

## Purpose

The `autoware_bevfusion` package is used for 3D object detection based on camera-lidar fusion.

## Inner-workings / Algorithms

This package implements a TensorRT powered inference node for BEVFusion [1].
The sparse convolution backend corresponds to [spconv](https://github.com/traveller59/spconv).
Autoware installs it automatically in its setup script. If needed, the user can also build it and install it following the [following instructions](https://github.com/autowarefoundation/spconv_cpp).

## Inputs / Outputs

### Input

| Name                   | Type                            | Description               |
| ---------------------- | ------------------------------- | ------------------------- |
| `~/input/pointcloud`   | `sensor_msgs::msg::PointCloud2` | Input pointcloud topics.  |
| `~/input/image*`       | `sensor_msgs::msg::Image`       | Input image topics.       |
| `~/input/camera_info*` | `sensor_msgs::msg::CameraInfo`  | Input camera info topics. |

### Output

| Name                                   | Type                                             | Description                 |
| -------------------------------------- | ------------------------------------------------ | --------------------------- |
| `~/output/objects`                     | `autoware_perception_msgs::msg::DetectedObjects` | Detected objects.           |
| `debug/cyclic_time_ms`                 | `tier4_debug_msgs::msg::Float64Stamped`          | Cyclic time (ms).           |
| `debug/pipeline_latency_ms`            | `tier4_debug_msgs::msg::Float64Stamped`          | Pipeline latency time (ms). |
| `debug/processing_time/preprocess_ms`  | `tier4_debug_msgs::msg::Float64Stamped`          | Preprocess (ms).            |
| `debug/processing_time/inference_ms`   | `tier4_debug_msgs::msg::Float64Stamped`          | Inference time (ms).        |
| `debug/processing_time/postprocess_ms` | `tier4_debug_msgs::msg::Float64Stamped`          | Postprocess time (ms).      |
| `debug/processing_time/total_ms`       | `tier4_debug_msgs::msg::Float64Stamped`          | Total processing time (ms). |

## Parameters

### BEVFusion node

{{ json_to_markdown("perception/autoware_bevfusion/schema/bevfusion.schema.dummy.json") }}

### BEVFusion model

{{ json_to_markdown("perception/autoware_bevfusion/schema/bevfusion_ml_package.schema.json") }}

### Detection class remapper

{{ json_to_markdown("perception/autoware_bevfusion/schema/detection_class_remapper.schema.json") }}

### The `build_only` option

The `autoware_bevfusion` node has a `build_only` option to build the TensorRT engine file from the specified ONNX file, after which the program exits.

```bash
ros2 launch autoware_bevfusion bevfusion.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `autoware_bevfusion` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch autoware_bevfusion bevfusion.launch.xml log_level:=debug
```

## Assumptions / Known limits

This node assumes that the input pointcloud follows the `PointXYZIRC` layout defined in `autoware_point_types`.

## Trained Models

TODO

### Changelog

## References/External links

[1] Zhijian Liu, Haotian Tang, Alexander Amini, Xinyu Yang, Huizi Mao, Daniela Rus, and Song Han. "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation." 2023 International Conference on Robotics and Automation. <!-- cspell:disable-line -->

## (Optional) Future extensions / Unimplemented parts

Although this node can perform camera-lidar fusion, as is the first method in autoware to actually use images and lidars for inference, the package structure and its full integration in the autoware pipeline are left for future work. In the current structure, it can be employed without any changes as a lidar-based detector.
