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

| Name                          | Type                           | Description                                                                                                                |
| ----------------------------- | ------------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| `~/input/camera*/image`       | `sensor_msgs::msg::Image`      | Input image topics, subscribed via `image_transport` (`raw` or `compressed` transport, selected by `is_compressed_image`). |
| `~/input/camera*/camera_info` | `sensor_msgs::msg::CameraInfo` | Input camera info topics, for camera parameters.                                                                           |

### Output

| Name                          | Type                                                | Description                                                                                                    | RTX 3090 Latency (ms) |
| ----------------------------- | --------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- | --------------------- |
| `~/output/objects`            | `autoware_perception_msgs::msg::DetectedObjects`    | Detected objects.                                                                                              | —                     |
| `latency/preprocess`          | `autoware_internal_debug_msgs::msg::Float64Stamped` | Preprocessing time per image(ms).                                                                              | 3.25                  |
| `latency/total`               | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total processing time (ms): preprocessing + inference + postprocessing.                                        | 26.04                 |
| `latency/inference`           | `autoware_internal_debug_msgs::msg::Float64Stamped` | Model forward pass (ms), stream-synchronized before returning.                                                 | 22.13                 |
| `latency/inference/backbone`  | `autoware_internal_debug_msgs::msg::Float64Stamped` | Backbone inference time (ms).                                                                                  | 16.21                 |
| `latency/inference/ptshead`   | `autoware_internal_debug_msgs::msg::Float64Stamped` | Points head inference time (ms).                                                                               | 5.45                  |
| `latency/inference/pos_embed` | `autoware_internal_debug_msgs::msg::Float64Stamped` | Position embedding inference time (ms).                                                                        | 0.40                  |
| `latency/postprocess`         | `autoware_internal_debug_msgs::msg::Float64Stamped` | bbox decode + nms + converting network predictions to autoware format (ms); disjoint from `latency/inference`. | 0.40                  |
| `latency/cycle_time_ms`       | `autoware_internal_debug_msgs::msg::Float64Stamped` | Time between two consecutive predictions (ms).                                                                 | 110.65                |

All `latency/*` topics are published only when `debug_mode` is enabled.

### Diagnostics

Published on `/diagnostics` regardless of `debug_mode`. Both statuses are tasks of one
timer-driven updater (period `diagnostics.validation_callback_interval_ms`, hardware_id = the
node name), so they keep reporting when the cameras or the inference stop — precisely the
conditions they exist for — and always arrive in the same `/diagnostics` message. One status per
failure mode, so a diagnostic graph can route input trouble and processing-time trouble to
different paths.

| Name                     | Description                                           |
| ------------------------ | ----------------------------------------------------- |
| `camera_status`          | Per-camera input availability, staleness and validity |
| `processing_time_status` | Per-cycle processing-time watchdog                    |

`camera_status` reports three key/values per camera — the resolved input `cameraN/topic` (the
model input index and the physical camera differ on every deployment), `cameraN/image_age_ms`
(node clock minus newest header stamp; `n/a` until the first frame) and `cameraN/state`, which
collapses the camera into one state, most-specific-cause first:

| `cameraN/state`       | Meaning                                                     | Level   |
| --------------------- | ----------------------------------------------------------- | ------- |
| `rejected`            | Newest frame dropped by the input validation                | `ERROR` |
| `waiting_camera_info` | No `camera_info` yet (normal during start-up)               | `WARN`  |
| `waiting_image`       | No first frame yet (normal during start-up)                 | `WARN`  |
| `stale`               | Used to publish; newest frame older than `max_image_age_ms` | `ERROR` |
| `active`              | Healthy                                                     | `OK`    |

plus summary key/values:

| Key                                              | Meaning                                                                        |
| ------------------------------------------------ | ------------------------------------------------------------------------------ |
| `num_cameras`                                    | Expected camera count (`rois_number`)                                          |
| `num_waiting` / `num_stale` / `num_rejected`     | State counts; the summary level falls out of these                             |
| `oldest_image_camera_id` / `oldest_image_age_ms` | The camera that has gone longest without a new frame                           |
| `max_inter_camera_time_diff_ms`                  | Subscriber-side stamp spread between cameras (`n/a` while cameras are missing) |

Timestamps and ages are formatted as fixed-point strings (never scientific notation), like the
pointcloud concatenation node's diagnostics.

The summary level is the worst camera state; additionally an inter-camera stamp spread above
`max_camera_time_diff` raises a `WARN` (prediction cycles are being skipped by the sync check).
`rejected` and `stale` are `ERROR` rather than `WARN` because neither recovers until the
publisher is fixed: a rejected camera publishes an encoding other than `rgb8`/`bgr8`, or a buffer
that is not densely packed, and those frames never reach the model.

`processing_time_status` reports `WARN` once a cycle exceeds
`diagnostics.max_allowed_processing_time_ms`, escalating to `ERROR` if it stays over budget for
longer than `diagnostics.max_acceptable_consecutive_delay_ms`. It reports "waiting" until the
first inference completes, and carries a per-stage breakdown of the measured cycle —
`preprocess_time_ms`, `inference_time_ms` and `postprocess_time_ms`, mirroring the
`latency/preprocess`, `latency/inference` and `latency/postprocess` debug topics — so an
over-budget `processing_time_ms` can be localized without enabling `debug_mode`.

`inference_time_ms` (the model forward pass) and `postprocess_time_ms` (bbox decode and NMS)
are disjoint stages. `preprocess_time_ms` however is the cost of a single image rather than the
sum over the cameras, and the total is started from the anchor camera's callback, so
preprocessing done in the other cameras' callbacks falls outside it — the stages do not add up
to `processing_time_ms`; compare `preprocess_time_ms` against its own history instead.

The newest published detection is reported under both clocks, because `processing_time_ms` only
covers the work done inside the node:

| Key                          | Description                                                                                               |
| ---------------------------- | --------------------------------------------------------------------------------------------------------- |
| `last_frame_timestamp`       | Header stamp the detections were computed for (the sensing instant)                                       |
| `last_published_timestamp`   | Node-clock instant at which they left the node                                                            |
| `output_latency_ms`          | The difference: end-to-end latency, including the camera transport and the decode queue ahead of the node |
| `time_since_last_publish_ms` | Age of `last_published_timestamp`, i.e. how long since the node last produced objects                     |

These are observational only — the level is still decided by the processing-time thresholds
alone. Comparing a node clock against a header stamp requires both to share a time base, so a
rosbag replay has to publish `/clock` and the node has to run with `use_sim_time` for
`output_latency_ms` and `time_since_last_publish_ms` to be meaningful.

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

#### Ego vehicle mask

Masking the area of the ego vehicle in order to reduce FP caused by reflection. Configure via **launch** or `camera_streampetr.param.yaml`, not `tensorrt_stream_petr.param.yaml` (model/post-process only).

- `ego_mask.enabled`: Enable masking (default: `false`)
- `ego_mask.fill_value_rgb`: RGB fill inside polygons, 0–255 (default: `[0, 0, 0]`)
- `ego_mask.roi_polygons_yaml`: One YAML path per model ROI index; empty string disables that ROI.

Example polygon files: `config/camera9_polygons.yaml`, `config/camera10_polygons.yaml`.

**X2 five-camera layout** (`tensorrt_stream_petr.x2.launch.xml`): ROI 2 → camera10 (left strip), ROI 4 → camera9 (right strip). Ego mask params are set in that launch file.

#### Node Parameters

- `max_camera_time_diff`: Maximum allowed time difference between cameras (seconds)
- `rois_number`: Number of camera ROIs/cameras (default: 6)
- `is_compressed_image`: Whether to subscribe with the `compressed` image transport instead of `raw`
- `is_distorted_image`: Whether input images are distorted
- `multithreading`: Whether to use multithreading for handling image callbacks
- `anchor_camera_id`: ID of the anchor camera for synchronization (default: 0)
- `debug_mode`: Enable debug mode for timing measurements
- `build_only`: Build TensorRT engines and exit without running inference
- `diagnostics.max_allowed_processing_time_ms`: Per-cycle processing-time budget; exceeding it raises a `WARN` (default: 200.0)
- `diagnostics.max_acceptable_consecutive_delay_ms`: How long the processing time may stay over budget before the `WARN` escalates to an `ERROR` (default: 1000.0)
- `diagnostics.max_image_age_ms`: A camera whose newest frame is older than this is reported as stalled at `ERROR` level (default: 300.0)
- `diagnostics.validation_callback_interval_ms`: Interval of the diagnostic callbacks (default: 100.0)

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

The model bundle is hosted on [Hugging Face](https://huggingface.co/AutowareFoundation/camera_streampetr/tree/v1.0). The [ansible artifacts role](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts) downloads it into `~/autoware_data/ml_models/camera_streampetr`.

Required model files:

- `simplify_extract_img_feat.onnx` (backbone)
- `simplify_pts_head_memory.onnx` (head)
- `simplify_position_embedding.onnx` (position embedding)
- `ml_package_camera_streampetr.param.yaml` (ML package config)

If you want to train and deploy your own model, you can find the source code for that in [AWML](https://github.com/tier4/AWML/tree/main/projects/StreamPETR).

## Changelog

## References/External links

[1] Wang, Shihao and Liu, Yingfei and Wang, Tiancai and Li, Ying and Zhang, Xiangyu. "Exploring Object-Centric Temporal Modeling for Efficient Multi-View 3D Object Detection." 2023 <!-- cspell:disable-line -->

## (Optional) Future extensions / Unimplemented parts

- Enable 2d object detection. Because 2d object detection is used as an auxiliary loss during training, the same node can easily support 2d object detection with minor updates.
- Implement int8 quantization for the backbone to further reduce inference latency
- Execute the image backbone for each image as they arrive, to further reduce latency.
- Add velocity to predictions.
