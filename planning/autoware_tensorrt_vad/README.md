# Autoware VAD

## Overview

The **Autoware VAD** is a trajectory generation module for autonomous vehicles, designed to work within the [Autoware](https://autoware.org/) ecosystem. It leverages the [VAD](https://github.com/hustvl/VAD/tree/main) model, as described in the paper [VAD: Vectorized Scene Representation for Efficient Autonomous Driving](https://arxiv.org/abs/2303.12077) by Jiang et al. <!-- cSpell:ignore Jiang -->

It is implemented as a ROS 2 component node, making it easy to integrate into Autoware-based stacks. The node is aimed at working within the proposed [Autoware new planning framework](https://github.com/tier4/new_planning_framework).

---

## Features

- **TensorRT Runtime** inference for fast neural network execution
- **ROS 2 publishers** for planned trajectories, predicted objects, and debug markers

---

## Parameters

{{ json_to_markdown("planning/autoware_tensorrt_vad/schema/vad_tiny.schema.json") }}

Parameters can be set via YAML (see `config/vad_tiny.param.yaml` and `config/ml_package_vad_tiny.param.yaml`).

---

## Inputs

| Topic                  | Message Type                                             | Description                                                     |
| ---------------------- | -------------------------------------------------------- | --------------------------------------------------------------- |
| `~/input/image*`       | sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage | Input image topics (supports both compressed and uncompressed). |
| `~/input/camera_info*` | sensor_msgs/msg/CameraInfo                               | Input camera info topics, for camera parameters.                |
| `~/input/odometry`     | nav_msgs/msg/Odometry                                    | Ego vehicle odometry                                            |
| `~/input/acceleration` | geometry_msgs/msg/AccelWithCovarianceStamped             | Ego acceleration                                                |

## Outputs

| Topic                   | Message Type                                              | Description                                |
| ----------------------- | --------------------------------------------------------- | ------------------------------------------ |
| `~/output/trajectory`   | autoware_planning_msgs/msg/Trajectory                     | Planned trajectory for the ego vehicle     |
| `~/output/trajectories` | autoware_internal_planning_msgs/msg/CandidateTrajectories | Multiple candidate trajectories            |
| `~/output/objects`      | autoware_perception_msgs/msg/PredictedObjects             | Predicted future states of dynamic objects |
| `~/debug/lane_marker`   | visualization_msgs/msg/MarkerArray                        | Lane debug markers                         |

---

## How to use

### Step1. Download onnx

- Please download onnx file from [this link](https://tier4inc-my.sharepoint.com/:f:/g/personal/taiki_tanaka_tier4_jp/EvQZY6sIudNKnFJSAnuyS9ABpodIW_FSYk57BrenzhCtXg?e=T4RLVw).

- Please set onnx directory under `~/autoware_data`

```sh
❯ tree ~/autoware_data/vad
/home/user_name/autoware_data/vad
├── sim_vadv1.extract_img_feat.onnx
├── sim_vadv1.pts_bbox_head.forward.onnx
└── sim_vadv1_prev.pts_bbox_head.forward.onnx
```

### Step2. Build `autoware_tensorrt_vad`

```sh
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_tensorrt_vad
```

### Step3. Launch `autoware_tensorrt_vad`

```sh
ros2 launch autoware_tensorrt_vad vad.launch.xml use_sim_time:=true
```

## Testing

Unit tests are provided and can be run with:

```bash
colcon test --packages-select autoware_tensorrt_vad
colcon test-result --all
```

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

- [VAD (original repo)](https://github.com/hustvl/VAD/tree/main)
- [VAD: Vectorized Scene Representation for Efficient Autonomous Driving](https://arxiv.org/abs/2303.12077)

---

## License

This package is released under the Apache 2.0 License.
