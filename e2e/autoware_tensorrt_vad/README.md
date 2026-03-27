# autoware_tensorrt_vad

## Overview

The `autoware_tensorrt_vad` is a ROS 2 component that implements end-to-end autonomous driving using the TensorRT-optimized Vectorized Autonomous Driving (VAD) model. It leverages the [VAD model](https://github.com/hustvl/VAD) (Jiang et al., 2023), optimized for deployment using NVIDIA's [DL4AGX](https://github.com/NVIDIA/DL4AGX) TensorRT framework. <!-- cSpell:ignore Jiang Shaoyu Bencheng Liao Jiajie Helong Wenyu Xinggang -->

This module replaces traditional localization, perception, and planning modules with a single neural network, trained on the [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) benchmark (Jia et al., 2024) using CARLA simulation data. It integrates seamlessly with [Autoware](https://autowarefoundation.github.io/autoware-documentation/main/) and is designed to work within the Autoware framework.

---

## Features

- **Monolithic End-to-End Architecture**: Single neural network directly maps camera inputs to trajectories, replacing the entire traditional perception-planning pipeline with one unified model - no separate detection, tracking, prediction, or planning modules
- **Multi-Camera Perception**: Processes 6 surround-view cameras simultaneously for 360° awareness
- **Vectorized Scene Representation**: Efficient scene encoding using vector maps for reduced computational overhead
- **Real-time TensorRT Inference**: Optimized for embedded deployment with ~20ms inference time
- **Integrated Perception Outputs**: Generates both object predictions (with future trajectories) and map elements as auxiliary outputs
- **Temporal Modeling**: Leverages historical features for improved temporal consistency and prediction accuracy

---

## Visualization

### Lane Following Demo

![Lane Following](media/lane_follow_demo.jpg)

### Turn Right Demo

![Turn Right](media/turn_right_demo.jpg)

---

## Parameters

Parameters can be set via configuration files:

- Deployment configuration (node and interface parameters): `config/vad_carla_tiny.param.yaml`
- Model architecture parameters: `vad-carla-tiny.param.json` (downloaded with model to `~/autoware_data/vad/v0.1/`)

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
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=$HOME/autoware_map/Town01 \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  use_e2e_planning:=true
```

---

## Model Setup and Versioning

### Model Download

The VAD model files are automatically downloaded when setting up the Autoware development environment.

To download the latest models, simply run the provided setup script:
[How to set up a development environment](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-set-up-a-development-environment)

The models will be downloaded to `~/autoware_data/vad/` by default.

**Manual Download** (if needed):
Models are hosted at: <https://awf.ml.dev.web.auto/planning/models/tensorrt_vad/carla_tiny/v0.1/>

### Model Preparation

> :warning: **Note**: The node automatically builds TensorRT engines from ONNX models on first run. Pre-built engines are cached for subsequent runs and are hardware-specific.

**Model Components** (trained on Bench2Drive CARLA dataset):

- `vad-carla-tiny_backbone.onnx` - Image feature extraction backbone
- `vad-carla-tiny_head_no_prev.onnx` - Planning head (first frame)
- `vad-carla-tiny_head.onnx` - Temporal planning head

If you need to manually configure the model paths, update them in `config/vad_carla_tiny.param.yaml`:

```yaml
model_params:
  nets:
    backbone:
      onnx_path: "$(var model_path)/v0.1/vad-carla-tiny_backbone.onnx"
      engine_path: "$(var model_path)/v0.1/vad-carla-tiny_backbone.engine"
    head:
      onnx_path: "$(var model_path)/v0.1/vad-carla-tiny_head.onnx"
      engine_path: "$(var model_path)/v0.1/vad-carla-tiny_head.engine"
    head_no_prev:
      onnx_path: "$(var model_path)/v0.1/vad-carla-tiny_head_no_prev.onnx"
      engine_path: "$(var model_path)/v0.1/vad-carla-tiny_head_no_prev.engine"
```

1. **Launch the node**: On first run, the node will automatically:
   - Build TensorRT engines from ONNX models
   - Optimize for your specific GPU
   - Cache engines at the specified `engine_path` locations
   - Use FP16 precision for backbone and FP32 for heads (configurable)

### Model Version History

| Version | Training Dataset  | Release Date | Notes                                                                                                    | Node Compatibility |
| ------- | ----------------- | ------------ | -------------------------------------------------------------------------------------------------------- | ------------------ |
| **0.1** | Bench2Drive CARLA | 2025-11-04   | - Initial release<br>- 6-camera surround view<br>- Trained on CARLA Towns<br>- FP16/FP32 mixed precision | >= 0.1.0           |

---

## ❗ Limitations

While VAD demonstrates promising end-to-end driving capabilities, users should be aware of the following limitations:

### Training Data Constraints

- **Simulation-Only Training**: The model is trained exclusively on CARLA simulator data, which may not capture the full complexity and variability of real-world driving scenarios

### Lack of High-Level Command Interface

- **No Dynamic Mission Control**: The current implementation lacks a high-level command interface, meaning the model cannot dynamically switch between driving behaviors (e.g., "follow lane" → "turn right at next intersection") during runtime

---

## Development & Contribution

- Follow the [Autoware coding guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
- Contributions, bug reports, and feature requests are welcome via GitHub issues and pull requests.

---

## References

### Core Model

1. VAD: Vectorized Scene Representation for Efficient Autonomous Driving (2023)
   - Paper: [arXiv:2303.12077](https://arxiv.org/abs/2303.12077)
   - Code: [github.com/hustvl/VAD](https://github.com/hustvl/VAD)

### Training and Datasets

1. Bench2Drive: Towards Multi-Ability Benchmarking of Closed-Loop End-To-End Autonomous Driving (2024)
   - Paper: [arXiv:2406.03877](https://arxiv.org/abs/2406.03877)
   - Code: [github.com/Thinklab-SJTU/Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive)
   - Description: CARLA-based benchmark for end-to-end autonomous driving evaluation

### Deployment and Optimization

1. DL4AGX (2024)
   - Resource: [github.com/NVIDIA/DL4AGX](https://github.com/NVIDIA/DL4AGX)
   - Description: TensorRT optimization for autonomous driving workloads and embedded GPU deployment strategies

### Related Work

1. nuScenes: A Multimodal Dataset for Autonomous Driving (2020)
   - Paper: [arXiv:1903.11027](https://arxiv.org/abs/1903.11027)
   - Dataset: [nuscenes.org](https://www.nuscenes.org)

2. BEVFormer: Learning Bird's-Eye-View Representation from Multi-Camera Images via Spatiotemporal Transformers (2022)
   - Paper: [arXiv:2203.17270](https://arxiv.org/abs/2203.17270)

---

## License

This package is released under the Apache 2.0 License.
