# autoware_camera_streampetr

## Purpose

The `autoware_camera_streampetr` package is used for 3D object detection based on images only.

## Inner-workings / Algorithms

This package implements a TensorRT powered inference node for StreamPETR [1]. This is the first camera-only 3D object detection node in autoware.


## Inputs / Outputs

### Input

| Name                           | Type                                                             | Description                                                |
|--------------------------------|------------------------------------------------------------------|------------------------------------------------------------|
| `~/input/kinematic_state`      | `nav_msgs::msg::Odometry`                                        | Vehicle kinematic state for ego motion tracking.           |
| `~/input/camera*/image`        | `sensor_msgs::msg::Image` or `sensor_msgs::msg::CompressedImage` | Input image topics (supports both compressed and uncompressed). |
| `~/input/camera*/camera_info`  | `sensor_msgs::msg::CameraInfo`                                   | Input camera info topics.                                  |

### Output

| Name                              | Type                                                          | Description                                                                 |
|-----------------------------------|---------------------------------------------------------------|-----------------------------------------------------------------------------|
| `~/output/objects`                | `autoware_perception_msgs::msg::DetectedObjects`              | Detected objects.                                                           |
| `latency/preprocess`              | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Preprocessing time (ms).                                                    |
| `latency/total`                   | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Total processing time (ms): preprocessing + inference + postprocessing.     |
| `latency/inference`               | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Total inference time (ms).                                                  |
| `latency/inference/backbone`      | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Backbone inference time (ms).                                               |
| `latency/inference/ptshead`       | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Points head inference time (ms).                                            |
| `latency/inference/pos_embed`     | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Position embedding inference time (ms).                                     |
| `latency/inference/postprocess`   | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Postprocessing time (ms): network predictions → Autoware topics.            |
| `latency/cycle_time_ms`           | `autoware_internal_debug_msgs::msg::Float64Stamped`           | Cycle time (ms): from receiving first camera topic to publishing results.   |


## Parameters

### StreamPETR node

The `autoware_camera_streampetr` node has various parameters for configuration:

#### Model Parameters
- `model_params.backbone_path`: Path to the backbone ONNX model
- `model_params.head_path`: Path to the head ONNX model  
- `model_params.position_embedding_path`: Path to the position embedding ONNX model
- `model_params.fp16_mode`: Enable FP16 inference mode
- `model_params.use_temporal`: Enable temporal modeling
- `model_params.input_image_height`: Input image height for preprocessing
- `model_params.input_image_width`: Input image width for preprocessing
- `model_params.max_camera_time_diff`: Maximum allowed time difference between cameras (seconds)
- `model_params.class_names`: List of detection class names
- `model_params.num_proposals`: Number of object proposals
- `model_params.detection_range`: Detection range for filtering objects

#### Post-processing Parameters
- `post_process_params.iou_nms_search_distance_2d`: 2D search distance for IoU NMS
- `post_process_params.circle_nms_dist_threshold`: Distance threshold for circle NMS
- `post_process_params.iou_nms_threshold`: IoU threshold for NMS
- `post_process_params.confidence_threshold`: Confidence threshold for detections
- `post_process_params.yaw_norm_thresholds`: Yaw normalization thresholds

#### Node Parameters  
- `rois_number`: Number of camera ROIs/cameras (default: 6)
- `is_compressed_image`: Whether input images are compressed
- `anchor_camera_id`: ID of the anchor camera for synchronization (default: 0)
- `debug_mode`: Enable debug mode for timing measurements
- `decompression_downsample`: Downsampling factor for image decompression
- `build_only`: Build TensorRT engines and exit without running inference

### The `build_only` option

The `autoware_camera_streampetr` node has a `build_only` option to build the TensorRT engine files from the specified ONNX files, after which the program exits.

```bash
ros2 launch autoware_camera_streampetr tensorrt_stream_petr.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `autoware_camera_streampetr` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch autoware_camera_streampetr tensorrt_stream_petr.launch.xml log_level:=debug
```

## Assumptions / Known limits

This node is camera-only and does not require pointcloud input. It assumes:
- All cameras are synchronized within the specified `max_camera_time_diff`
- Camera calibration information is available and accurate
- The anchor camera (specified by `anchor_camera_id`) triggers the inference cycle
- Vehicle odometry is available for ego motion compensation
- Transform information between camera frames and base_link is available

## Trained Models

You can download the ONNX model files for StreamPETR. The files should be placed in the appropriate model directory as specified in the launch configuration.

Required model files:
- Backbone ONNX model
- Head ONNX model  
- Position embedding ONNX model

### Changelog

## References/External links

[1] Wang, Shihao and Liu, Yingfei and Wang, Tiancai and Li, Ying and Zhang, Xiangyu. "Exploring Object-Centric Temporal Modeling for Efficient Multi-View 3D Object Detection." 2023

## (Optional) Future extensions / Unimplemented parts

- Implement int8 quantization for the backbone to further reduce inference latency
- Support for dynamic number of cameras
- Enhanced temporal modeling capabilities
- Integration with additional sensor modalities
