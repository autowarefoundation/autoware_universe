# autoware_tensorrt_rtmdet

## Purpose

RTMDet is a real-time instance segmentation model which can be used for detecting objects like cars, pedestrians,
bicycles, etc. in a scene. This package provides a ROS2 interface for RTMDet using TensorRT.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name       | Type                | Description     |
| ---------- | ------------------- | --------------- |
| `in/image` | `sensor_msgs/Image` | The input image |

### Output

| Name              | Type                                               | Description                                                         |
| ----------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| `out/objects`     | `tier4_perception_msgs/DetectedObjectsWithFeature` | The detected objects with 2D bounding boxes and scores              |
| `out/mask`        | `sensor_msgs/Image`                                | The instance segmentation mask                                      |
| `out/color_mask`  | `sensor_msgs/Image`                                | The colorized image of instance segmentation mask for visualization |
| `out/debug_image` | `sensor_msgs/Image`                                | The image with 2D bounding boxes for visualization                  |

## Parameters

{{ json_to_markdown("perception/autoware_tensorrt_rtmdet/schema/rtmdet.schema.json") }}

## Assumptions / Known limits

## Onnx model

## Label file

## Reference repositories

- <https://github.com/open-mmlab/mmdetection/tree/3.x/configs/rtmdet#rtmdet-an-empirical-study-of-designing-real-time-object-detectors>
