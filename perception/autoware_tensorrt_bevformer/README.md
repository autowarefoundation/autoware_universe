# tensorrt_bevformer <!-- cspell:ignore Zhicheng, canbus, trainval, ROS2, bevformer, Multicoreware's, Bevformer -->

## Purpose

The core algorithm, named `BEVFormer`, unifies multi-view images into the BEV perspective for 3D object detection tasks with temporal fusion.

## Inner-workings / Algorithms

### Cite

- Zhicheng Wang, et al., "BEVFormer: Incorporating Transformers for Multi-Camera 3D Detection" [[ref](https://arxiv.org/abs/2203.17270)]
- This node is ported and adapted for Autoware from [Multicoreware's](https://multicorewareinc.com/) BEVFormer ROS2 C++ repository.

## Inputs / Outputs

### Inputs

| Name                                        | Type                                              | Description                         |
| ------------------------------------------- | ------------------------------------------------- | ----------------------------------- |
| `~/input/topic_img_front_left`              | `sensor_msgs::msg::Image`                         | input front_left camera image       |
| `~/input/topic_img_front`                   | `sensor_msgs::msg::Image`                         | input front camera image            |
| `~/input/topic_img_front_right`             | `sensor_msgs::msg::Image`                         | input front_right camera image      |
| `~/input/topic_img_back_left`               | `sensor_msgs::msg::Image`                         | input back_left camera image        |
| `~/input/topic_img_back`                    | `sensor_msgs::msg::Image`                         | input back camera image             |
| `~/input/topic_img_back_right`              | `sensor_msgs::msg::Image`                         | input back_right camera image       |
| `~/input/topic_img_front_left/camera_info`  | `sensor_msgs::msg::CameraInfo`                    | input front_left camera parameters  |
| `~/input/topic_img_front/camera_info`       | `sensor_msgs::msg::CameraInfo`                    | input front camera parameters       |
| `~/input/topic_img_front_right/camera_info` | `sensor_msgs::msg::CameraInfo`                    | input front_right camera parameters |
| `~/input/topic_img_back_left/camera_info`   | `sensor_msgs::msg::CameraInfo`                    | input back_left camera parameters   |
| `~/input/topic_img_back/camera_info`        | `sensor_msgs::msg::CameraInfo`                    | input back camera parameters        |
| `~/input/topic_img_back_right/camera_info`  | `sensor_msgs::msg::CameraInfo`                    | input back_right camera parameters  |
| `~/input/can_bus`                           | `autoware_localization_msgs::msg::KinematicState` | CAN bus data for ego-motion         |

### Outputs

| Name              | Type                                             | Description                                 |
| ----------------- | ------------------------------------------------ | ------------------------------------------- |
| `~/output_boxes`  | `autoware_perception_msgs::msg::DetectedObjects` | detected objects                            |
| `~/output_bboxes` | `visualization_msgs::msg::MarkerArray`           | detected objects for nuScenes visualization |

## How to Use Tensorrt BEVFormer Node

### Prerequisites

- **TensorRT** 10.8.0.43
- **CUDA** 12.4
- **cuDNN** 8.9.2

### Trained Model

Download the [`bevformer_small.onnx`](https://multicorewareinc1-my.sharepoint.com/:u:/g/personal/naveen_sathiyaseelan_multicorewareinc_com/ERQSpS-BoAZGh4R4zNZhITcB58aqDW_tu9aKHLpit6aLAg?e=IZ5nZN) model to:

```bash
$HOME/autoware_data/tensorrt_bevformer
```

> **Note:** The **BEVFormer** model was trained on the **nuScenes** dataset for 24 epochs with temporal fusion enabled.

### Test TensorRT BEVFormer Node with nuScenes

1. Integrate this package into your **autoware_universe/perception** directory.

2. To play ROS 2 bag of nuScenes data:

   ```bash
   cd autoware/src
   git clone -b feature/bevformer-integration https://github.com/naveen-mcw/ros2_dataset_bridge.git
   cd ..
   ```

   > **Note:** The `feature/bevformer-integration` branch provides required data for the BEVFormer.

   Download nuScenes dataset and canbus data [here](https://www.nuscenes.org/nuscenes#).

   Open and edit the launch file to set dataset paths/configs:

   ```bash
   nano src/ros2_dataset_bridge/launch/nuscenes_launch.xml
   ```

   Update as needed:

   ```xml
   <arg name="NUSCENES_DIR" default="<nuScenes_dataset_path>"/>
   <arg name="NUSCENES_CAN_BUS_DIR" default="<can_bus_path>"/>
   <arg name="NUSCENES_VER" default="v1.0-trainval"/>
   <arg name="UPDATE_FREQUENCY" default="10.0"/>
   ```

3. Build the autoware_tensorrt_bevformer and ros2_dataset_bridge packages

   ```bash
   # Build ros2_dataset_bridge

   colcon build --packages-up-to ros2_dataset_bridge

   # Build autoware_tensorrt_bevformer

   colcon build --packages-up-to autoware_tensorrt_bevformer

   ```

   Source environments:

   ```bash
   source install/setup.bash
   source /opt/ros/humble/setup.bash
   ```

4. Launch dataset publisher and GUI tools:

   ```bash
   ros2 launch ros2_dataset_bridge nuscenes_launch.xml
   ```

   > **Tip:** If nuScenes boxes aren't visible in RViz, uncheck **Stop** in the GUI controller, then click **OK**.

5. Launch TensorRT BEVFormer Node

   ```bash
   # Default mode (FP16)
   ros2 launch autoware_tensorrt_bevformer bevformer.launch.xml

   # FP32 precision
   ros2 launch autoware_tensorrt_bevformer bevformer.launch.xml precision:=fp32

   # With visualization
   ros2 launch autoware_tensorrt_bevformer bevformer.launch.xml debug_mode:=true

   # FP32 + visualization
   ros2 launch autoware_tensorrt_bevformer bevformer.launch.xml precision:=fp32 debug_mode:=true
   ```

### Configuration

The configuration file in `config/bevformer.param.yaml` can be modified to suit your needs:

- Modify `precision` to `fp16` or `fp32`
- Set `debug_mode` to `true` to enable publishing bounding box markers.

## References/External links

[1] [BEVFormer (arXiv)](https://arxiv.org/abs/2203.17270)  
[2] [Original Python BEVFormer TensorRT](https://github.com/DerryHub/BEVFormer_tensorrt.git)  
[3] [nuScenes Dataset](https://www.nuscenes.org/)

---
