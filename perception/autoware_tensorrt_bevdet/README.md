# tensorrt_bevdet <!-- cspell: ignore bevdet -->

## Purpose

The core algorithm, named `BEVDet`, it unifies multi-view images into the perspective of BEV for 3D object detection task.

## Inner-workings / Algorithms

### Cite

<!-- cspell: ignore Junjie Huang, Guan Huang, BEVPoolv2 -->

- Junjie Huang, Guan Huang, "BEVPoolv2: A Cutting-edge Implementation of BEVDet Toward Deployment", [[ref](https://arxiv.org/pdf/2211.17111)]
- [bevdet_vendor](https://github.com/autowarefoundation/bevdet_vendor) package are copied from the [original codes](https://github.com/LCH1238/bevdet-tensorrt-cpp/tree/one) (The TensorRT, C++ implementation by LCH1238) and modified.
- This package is ported version toward Autoware from [bevdet_vendor](https://github.com/autowarefoundation/bevdet_vendor).

## Inputs / Outputs

### Inputs

| Name                                        | Type                           | Description                         |
| ------------------------------------------- | ------------------------------ | ----------------------------------- |
| `~/input/topic_img_front_left`              | `sensor_msgs::msg::Image`      | input front_left camera image       |
| `~/input/topic_img_front`                   | `sensor_msgs::msg::Image`      | input front camera image            |
| `~/input/topic_img_front_right`             | `sensor_msgs::msg::Image`      | input front_right camera image      |
| `~/input/topic_img_back_left`               | `sensor_msgs::msg::Image`      | input back_left camera image        |
| `~/input/topic_img_back`                    | `sensor_msgs::msg::Image`      | input back camera image             |
| `~/input/topic_img_back_right`              | `sensor_msgs::msg::Image`      | input back_right camera image       |
| `~/input/topic_img_front_left/camera_info`  | `sensor_msgs::msg::CameraInfo` | input front_left camera parameters  |
| `~/input/topic_img_front/camera_info`       | `sensor_msgs::msg::CameraInfo` | input front camera parameters       |
| `~/input/topic_img_front_right/camera_info` | `sensor_msgs::msg::CameraInfo` | input front_right camera parameters |
| `~/input/topic_img_back_left/camera_info`   | `sensor_msgs::msg::CameraInfo` | input back_left camera parameters   |
| `~/input/topic_img_back/camera_info`        | `sensor_msgs::msg::CameraInfo` | input back camera parameters        |
| `~/input/topic_img_back_right/camera_info`  | `sensor_msgs::msg::CameraInfo` | input back_right camera parameters  |

### Outputs

| Name              | Type                                             | Description                                 |
| ----------------- | ------------------------------------------------ | ------------------------------------------- |
| `~/output/boxes`  | `autoware_perception_msgs::msg::DetectedObjects` | detected objects                            |
| `~/output_bboxes` | `visualization_msgs::msg::MarkerArray`           | detected objects for nuScenes visualization |

## How to Use Tensorrt BEVDet Node

### Prerequisites

- Tensorrt 10.8.0.43
- CUDA 12.4
- cuDNN 8.9.2

### Trained Models

Download the trained models with the instructions in [Autoware artifacts](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts#readme).

The `BEVDet` model was trained in `NuScenes` dataset for 20 epochs.

### Test Tensorrt BEVDet Node with Nuscenes

1. Integrate this branch changes in your **autoware_universe/perception** directory

2. Include this [bevdet_vendor pr](https://github.com/autowarefoundation/bevdet_vendor/pull/1) in **src/universe/external/bevdet_vendor** as this supports fp16 precision and api support for Tensorrt 10.x.x

3. To play ros2 bag of nuScenes data
   <!-- cspell: ignore trainval -->

   ```bash

   cd autoware/src
   git clone https://github.com/Owen-Liuyuxuan/ros2_dataset_bridge
   cd ..

   # Open the launch file to configure dataset settings:

   nano src/ros2_dataset_bridge/launch/nuscenes_launch.xml

   # Update the following lines with the correct NuScenes dataset path and set the publishing frequency to 10 Hz for optimal data streaming:

   <arg name="NUSCENES_DIR" default="<nuscenes_dataset_path>"/>
   <arg name="NUSCENES_VER" default="v1.0-trainval"/>
   <arg name="UPDATE_FREQUENCY" default="10.0"/>

   # Open the ros_utils script:

   nano src/ros2_dataset_bridge/ros2_dataset_bridge/utils/ros_util.py

   # Modify the encoding to bgr8 at line 92:

   image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")

   ```

4. Build the `autoware_tensorrt_bevdet` and `ros2_dataset_bridge` packages

   ```bash

   # Build autoware_tensorrt_bevdet

   colcon build --packages-up-to autoware_tensorrt_bevdet

   # Build ros2_dataset_bridge

   colcon build --packages-select=ros2_dataset_bridge

   # Source the environment

   source install/setup.bash # install/setup.zsh or install/setup.sh for your own need.
   source /opt/ros/humble/setup.bash

   ```

5. Launch `ros2_dataset_bridge` that publishes nuScenes dataset

   ```bash

   # Launch the data publisher, RViz, and GUI controller:

   ros2 launch ros2_dataset_bridge nuscenes_launch.xml

   # Tip: If NuScenes boxes are not visible in RViz, ensure the "Stop" checkbox in the GUI controller is unchecked, then click "OK".

   # Note: ROS bag playback is limited to 10 Hz, which constrains the BEVDet node to the same rate. However, based on callback execution time, BEVDet can run at up to 35 FPS with FP16 and 17 FPS with FP32.
   ```

6. Launch `tensorrt_bevdet_node`

   ```bash
   ros2 launch autoware_tensorrt_bevdet tensorrt_bevdet.launch.xml
   ```

### Configuration

The configuration file in `config/bevdet.param.yaml` can be modified to suit your needs:

- Modify `precision` to `fp16` or `fp32`
- Set `debug_mode` to `true` to enable publishing bounding box markers.

## Limitation

The model is trained on open-source dataset `NuScenes` and has poor generalization on its own dataset, If you want to use this model to infer your data, you need to retrain it.

## Training BEVDet Model

If you want to train model using the [TIER IV's internal database(~2600 key frames)](https://drive.google.com/file/d/1UaarK88HZu09sf7Ix-bEVl9zGNGFwTVL/view?usp=sharing), please refer to the following repositories:[BEVDet adapted to TIER IV dataset](https://github.com/cyn-liu/BEVDet/tree/train_export).

## References/External links

[1] <https://github.com/HuangJunJie2017/BEVDet/tree/dev2.1>

[2] <https://github.com/LCH1238/BEVDet/tree/export>

[3] <https://github.com/LCH1238/bevdet-tensorrt-cpp/tree/one>
