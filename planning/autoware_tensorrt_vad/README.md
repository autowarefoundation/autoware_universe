# autoware_tensorrt_vad

## Overview

The `autoware_tensorrt_vad` is a ROS 2 component that implements end-to-end autonomous driving using the TensorRT-optimized Vectorized Autonomous Driving (VAD) model. It leverages the [VAD model](https://github.com/hustvl/VAD) (Jiang et al., 2023), optimized for deployment using NVIDIA's [DL4AGX](https://developer.nvidia.com/drive/drive-agx) TensorRT framework. <!-- cSpell:ignore Jiang Shaoyu Bencheng Liao Jiajie Helong Wenyu Xinggang -->

This module replaces traditional localization, perception, and planning modules with a single neural network, trained on the [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) benchmark (Jia et al., 2024) using CARLA simulation data. It integrates seamlessly with [Autoware](https://autoware.org/) and is designed to work within the Autoware planning framework.

Key capabilities:

- End-to-end driving from camera inputs to trajectory output
- Vectorized scene representation for efficient processing
- Multi-camera perception (6 surround-view cameras)
- Real-time inference using TensorRT optimization
- Integrated object prediction and map generation

---

## Features

- **End-to-End Autonomous Driving**: Directly generates trajectories from camera inputs without traditional modular pipeline
- **Multi-Camera Perception**: Processes 6 surround-view cameras simultaneously for 360° awareness
- **Vectorized Scene Representation**: Efficient scene encoding using vector maps for reduced computational overhead
- **Real-time TensorRT Inference**: Optimized for embedded deployment with ~20ms inference time

---

## Visualization

### Lane Following Demo

![Lane Following](media/lane_follow_demo.gif)

The module demonstrates robust lane following capabilities, maintaining the vehicle within lane boundaries while tracking the desired trajectory.

### Turn Right Demo

![Turn Right](media/turn_right_demo.gif)

The system successfully handles right turn maneuvers, generating smooth trajectories that follow road geometry and traffic rules.

---

## Parameters

{{ json_to_markdown("planning/autoware_tensorrt_vad/schema/vad_tiny.schema.json") }}

Parameters can be set via YAML configuration files:

- Node and interface parameters: `config/vad_carla_tier4.param.yaml`
- Model and network parameters: `config/ml_package_vad_carla_tier4.param.yaml`

---

## Inputs

| Topic                   | Message Type                                 | Description                                                                    |
| ----------------------- | -------------------------------------------- | ------------------------------------------------------------------------------ |
| ~/input/image\*         | sensor_msgs/msg/Image\*                      | Camera images 0-5: FRONT, BACK, FRONT_LEFT, BACK_LEFT, FRONT_RIGHT, BACK_RIGHT |
| ~/input/camera_info\*   | sensor_msgs/msg/CameraInfo                   | Camera calibration for cameras 0-5                                             |
| ~/input/kinematic_state | nav_msgs/msg/Odometry                        | Vehicle odometry                                                               |
| ~/input/acceleration    | geometry_msgs/msg/AccelWithCovarianceStamped | Vehicle acceleration                                                           |

\*Image transport supports both raw and compressed formats. Configure per-camera via `use_raw` parameter (default: compressed).

---

## Outputs

| Topic                 | Message Type                                              | Description                         |
| --------------------- | --------------------------------------------------------- | ----------------------------------- |
| ~/output/trajectory   | autoware_planning_msgs/msg/Trajectory                     | Selected ego trajectory             |
| ~/output/trajectories | autoware_internal_planning_msgs/msg/CandidateTrajectories | All 6 candidate trajectories        |
| ~/output/objects      | autoware_perception_msgs/msg/PredictedObjects             | Predicted objects with trajectories |
| ~/output/map          | visualization_msgs/msg/MarkerArray                        | Predicted map elements              |

---

## Building

Build the package with colcon:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_tensorrt_vad
```

---

## Testing

### Unit Tests

Unit tests are provided and can be run with:

```bash
colcon test --packages-select autoware_tensorrt_vad
colcon test-result --all
```

For verbose output:

```bash
colcon test --packages-select autoware_tensorrt_vad --event-handlers console_cohesion+
```

### CARLA Simulator Testing

First, setup CARLA following the [autoware_carla_interface](https://github.com/autowarefoundation/autoware.universe/tree/main/simulator/autoware_carla_interface) instructions.

Then launch the E2E VAD system:

```bash
ros2 launch autoware_launch e2e_vad_simulator.launch.xml \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  carla_map:=Town01 \
  use_autoware_sensor_kit:=true
```

---

## Model Setup and Versioning

### Model Preparation

> :warning: **Note**: The node automatically builds TensorRT engines from ONNX models on first run. Pre-built engines are cached for subsequent runs and are hardware-specific.

1. **Download the pre-trained ONNX models** trained on Bench2Drive CARLA dataset:
   - `sim_vadv1.extract_img_feat.onnx` - Image feature extraction backbone
   - `sim_vadv1.pts_bbox_head.forward.onnx` - Planning head (first frame)
   - `sim_vadv1_prev.pts_bbox_head.forward.onnx` - Temporal planning head

2. **Place the models** in your designated model directory (e.g., `~/autoware_data/vad/`) and update the paths in `ml_package_vad_carla_tier4.param.yaml`:

```yaml
model_params:
  nets:
    backbone:
      onnx_path: "$(var model_path)/sim_vadv1.extract_img_feat.onnx"
      engine_path: "$(var model_path)/vad-carla-tier4_backbone.engine"
    head:
      onnx_path: "$(var model_path)/sim_vadv1_prev.pts_bbox_head.forward.onnx"
      engine_path: "$(var model_path)/vad-carla-tier4_head.engine"
    head_no_prev:
      onnx_path: "$(var model_path)/sim_vadv1.pts_bbox_head.forward.onnx"
      engine_path: "$(var model_path)/vad-carla-tier4_head_no_prev.engine"
```

3. **Launch the node**: On first run, the node will automatically:
   - Build TensorRT engines from ONNX models
   - Optimize for your specific GPU
   - Cache engines at the specified `engine_path` locations
   - Use FP16 precision for backbone and FP32 for heads (configurable)

### Model Versions

| Model Version | Training Dataset  | Release Date | Notes                            | Node Compatibility |
| ------------- | ----------------- | ------------ | -------------------------------- | ------------------ |
| v1.0.0        | Bench2Drive CARLA | 2024-10      | Initial release, 6-camera config | >= 0.1.0           |

---

## ❗ Limitations

While the VAD shows promising capabilities, there are several limitations to be aware of:

- **Training Dataset Domain Gap**:
  The provided VAD model checkpoint was trained on datasets using a nuScenes.

---

## Development & Contribution

- Follow the [Autoware coding guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
- Contributions, bug reports, and feature requests are welcome via GitHub issues and pull requests.

---

## References

### Core Model

- **VAD Repository**: [hustvl/VAD](https://github.com/hustvl/VAD/tree/main)
- **VAD Paper**: Jiang et al., ["VAD: Vectorized Scene Representation for Efficient Autonomous Driving"](https://arxiv.org/abs/2303.12077), arXiv:2303.12077, 2023

### Training and Datasets

- **Bench2Drive**: Jia et al., ["Bench2Drive: Towards Multi-Ability Benchmarking of Closed-Loop End-To-End Autonomous Driving"](https://arxiv.org/abs/2406.03877), arXiv:2406.03877, 2024
  - Repository: [Thinklab-SJTU/Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)
  - CARLA-based benchmark for end-to-end autonomous driving evaluation

### Deployment and Optimization

- **NVIDIA DL4AGX**: [Deep Learning for Autonomous Vehicles](https://developer.nvidia.com/drive/drive-agx)
  - TensorRT optimization for autonomous driving workloads
  - Embedded GPU deployment strategies

### Related Work

- **nuScenes**: Caesar et al., ["nuScenes: A multimodal dataset for autonomous driving"](https://arxiv.org/abs/1903.11027), CVPR 2020
- **BEVFormer**: Li et al., ["BEVFormer: Learning Bird's-Eye-View Representation from Multi-Camera Images via Spatiotemporal Transformers"](https://arxiv.org/abs/2203.17270), ECCV 2022

---

## License

This package is released under the Apache 2.0 License.
