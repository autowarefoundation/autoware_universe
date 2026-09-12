# yabloc_pose_initializer

This package contains a node related to initial pose estimation.

- [camera_pose_initializer](#camera_pose_initializer)

This package requires the pre-trained semantic segmentation model for runtime. This model is usually downloaded by the [ansible artifacts role](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts) during the installation.
It is also possible to download it manually. Even if the model is not downloaded, initialization will still complete, but the accuracy may be compromised.

The model is hosted on [Hugging Face](https://huggingface.co/AutowareFoundation/yabloc_pose_initializer/tree/v1.0). To download it manually with the [hf CLI](https://huggingface.co/docs/huggingface_hub/guides/cli):

```bash
hf download AutowareFoundation/yabloc_pose_initializer --revision v1.0 --local-dir ~/autoware_data/ml_models/yabloc_pose_initializer
```

## Note

This package makes use of external code. The trained files are provided by apollo. The trained files are automatically downloaded during env preparation.

Original model URL

<https://github.com/openvinotoolkit/open_model_zoo/tree/master/models/intel/road-segmentation-adas-0001>

> Open Model Zoo is licensed under Apache License Version 2.0.

Converted model URL

<https://github.com/PINTO0309/PINTO_model_zoo/tree/main/136_road-segmentation-adas-0001>

> model conversion scripts are released under the MIT license

## Special thanks

- [openvinotoolkit/open_model_zoo](https://github.com/openvinotoolkit/open_model_zoo)
- [PINTO0309](https://github.com/PINTO0309)

## camera_pose_initializer

### Purpose

- This node estimates the initial position using the camera at the request of ADAPI.

#### Input

| Name                | Type                                    | Description              |
| ------------------- | --------------------------------------- | ------------------------ |
| `input/camera_info` | `sensor_msgs::msg::CameraInfo`          | undistorted camera info  |
| `input/image_raw`   | `sensor_msgs::msg::Image`               | undistorted camera image |
| `input/vector_map`  | `autoware_map_msgs::msg::LaneletMapBin` | vector map               |

#### Output

| Name                    | Type                                   | Description             |
| ----------------------- | -------------------------------------- | ----------------------- |
| `debug/init_candidates` | `visualization_msgs::msg::MarkerArray` | initial pose candidates |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_pose_initializer/schema/camera_pose_initializer.schema.json") }}

### Services

| Name               | Type                                                                  | Description                     |
| ------------------ | --------------------------------------------------------------------- | ------------------------------- |
| `yabloc_align_srv` | `autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped` | initial pose estimation request |
