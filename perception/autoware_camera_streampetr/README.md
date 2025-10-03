# autoware_camera_streampetr

## Purpose

The `autoware_camera_streampetr` package is used for 3D object detection based on images only.

## Inner-workings / Algorithms

This package implements a TensorRT powered inference node for StreamPETR [1]. This is the first camera-only 3D object detection node in autoware.

This node has been optimized for multi-camera systems where the camera topics are published in a sequential manner, not all at once. The node takes
advantage of this by preprocessing (resize, crop, normalize) the images and storing them appropriately on GPU, so that delay due to preprocessing can be minimized.

```pgsql

Topic for image_i arrived                                     -------------------------
  |                                                                                   |
  |                                                                                   |
  |                                                                                   |
  v                                                                                   |
Is image distorted?                                                                   |
  |              \                                                                    |
  |               \                                                                   |
Yes               No                                                                  |Image Updates
  |                |                                                                  |done in parallel, if multitheading is on
  v                |                                                                  |otherwise done sequentially in FIFO order
Undistort          |                                                                  |
  |                |                                                                  |
  v                v                                                                  |
Load image into GPU memory                                                            |
  |                                                                                   |
  v                                                                                   |
Preprocess image (scale & crop ROI & normalize)                                       |
  |                                                                                   |
  v                                                                                   |
Store in GPU memory binding location for model input                                  |
  |                                                          -------------------------|
  v                                                                                   |
Is image the `anchor_image`?                                                          |
  |                \                                                                  |
  |                 \                                                                 |
No                  Yes                                                               |
  |                  |                                                                |
  v                  v                                                                | If multithreading is on
(Wait)     Are all images synced within `max_time_difference`?                        | image Updates are temporarily frozen
                      |                           \                                   | until this part completes.
                      |                            \                                  |
                    Yes                             No                                |
                      |                             |                                 |
                      v                             v                                 |
         Perform model forward pass            (Sync failed! Skip prediction)         |
                      |                                                               |
                      v                                                               |
         Postprocess (NMS + ROS2 format)                                              |
                      |                                                               |
                      v                                                               |
             Publish predictions                             -------------------------|

```

## Inputs / Outputs

### Input

| Name                          | Type                                                             | Description                                                     |
| ----------------------------- | ---------------------------------------------------------------- | --------------------------------------------------------------- |
| `~/input/camera*/image`       | `sensor_msgs::msg::Image` or `sensor_msgs::msg::CompressedImage` | Input image topics (supports both compressed and uncompressed). |
| `~/input/camera*/camera_info` | `sensor_msgs::msg::CameraInfo`                                   | Input camera info topics, for camera parameters.                |

### Output

| Name                            | Type                                                | Description                                                               | RTX 3090 Latency (ms) |
| ------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------- | --------------------- |
| `~/output/objects`              | `autoware_perception_msgs::msg::DetectedObjects`    | Detected objects.                                                         | —                     |
| `latency/preprocess`            | `autoware_internal_debug_msgs::msg::Float64Stamped` | Preprocessing time per image(ms).                                         | 3.25                  |
| `latency/total`                 | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total processing time (ms): preprocessing + inference + postprocessing.   | 26.04                 |
| `latency/inference`             | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total inference time (ms).                                                | 22.13                 |
| `latency/inference/backbone`    | `autoware_internal_debug_msgs::msg::Float64Stamped` | Backbone inference time (ms).                                             | 16.21                 |
| `latency/inference/ptshead`     | `autoware_internal_debug_msgs::msg::Float64Stamped` | Points head inference time (ms).                                          | 5.45                  |
| `latency/inference/pos_embed`   | `autoware_internal_debug_msgs::msg::Float64Stamped` | Position embedding inference time (ms).                                   | 0.40                  |
| `latency/inference/postprocess` | `autoware_internal_debug_msgs::msg::Float64Stamped` | nms + filtering + converting network predictions to autoware format (ms). | 0.40                  |
| `latency/cycle_time_ms`         | `autoware_internal_debug_msgs::msg::Float64Stamped` | Time between two consecutive predictions (ms).                            | 110.65                |

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

- `max_camera_time_diff`: Maximum allowed time difference between cameras (seconds)
- `rois_number`: Number of camera ROIs/cameras (default: 6)
- `is_compressed_image`: Whether input images are compressed
- `is_distorted_image`: Whether input images are distorted
- `multithreading`: Whether to use multithreading for handling image callbacks
- `anchor_camera_id`: ID of the anchor camera for synchronization (default: 0)
- `debug_mode`: Enable debug mode for timing measurements
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
- Transform information between camera frames and base_link is available via tf
- Transform information between map and base_link is available via tf for ego motion compensation
- **The input images are undistorted**

## Trained Models

You can download the ONNX model files for StreamPETR. The files should be placed in the appropriate model directory as specified in the launch configuration.

Required model files:

- Backbone ONNX model: TODO
- Head ONNX model: TODO
- Position embedding ONNX model: TODO

If you want to train and deploy your own model, you can find the source code for that in [AWML](https://github.com/tier4/AWML/tree/main/projects/StreamPETR).

## Changelog

## References/External links

[1] Wang, Shihao and Liu, Yingfei and Wang, Tiancai and Li, Ying and Zhang, Xiangyu. "Exploring Object-Centric Temporal Modeling for Efficient Multi-View 3D Object Detection." 2023 <!-- cspell:disable-line -->

## (Optional) Future extensions / Unimplemented parts

- Enable 2d object detection. Because 2d object detection is used as an auxiliary loss during training, the same node can easily support 2d object detection with minor updates.
- Implement int8 quantization for the backbone to further reduce inference latency
- Execute the image backbone for each image as they arrive, to further reduce latency.
- Add velocity to predictions.
