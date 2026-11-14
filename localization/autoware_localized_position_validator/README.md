# Localized Position Validator
This package will provide a validation feature for current localized position. Validation will be done by a ML model.

# Purpose
This package will validate the current localized position is valid or not. In some cases, NDT's score might not match with the actual state.  
This validator will validate the state independently by a machine learning model and can detect the failure of NDT scoring in some cases.


# Installation

## Requirements

### TensorRT and LibTorch
This package uses TensorRT and LibTorch.  

LibTorch will automatically get donwloaded and built in the compile time.  

> [!NOTE]
> Building the LibTorch will take time. Also it is compiling with `-j$(nproc)` which may consume a large memory space.
> If you encounter with some out-of-memory issue, change the option in the script to use less more CPU threads by `-j 4` or something.

For TensorRT you need to install by yourself. Usually Autoware will install the dependency, but current Autoware does not include some required packages.

The easiest way is to modified the [Autoware's ansible file](https://github.com/autowarefoundation/autoware/tree/main/ansible).

Add following packages to `ansible/roles/cuda/tasks/main.yaml` in the [Autoware](https://github.com/autowarefoundation/autoware/blob/8e44ac3a4eac164cee6fe789c3cb9670dd778a16/ansible/roles/cuda/tasks/main.yaml#L28-L40):
```text
libcufft-dev-{{ cuda__dash_case_cuda_version.stdout }}
libcusolver-dev-{{ cuda__dash_case_cuda_version.stdout }}
libnvjitlink-dev-{{ cuda__dash_case_cuda_version.stdout }}
```
, then run the `setup-dev-env.sh`.  
Say `yes` to the installation of CUDA related packages.


## Model
You need to download the ONNX file and need to place in the `autoware_data/autoware_localized_position_validator`.


## Building
```sh
# By default, this package will not be built
# this might take time, be patient
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DSKIP_THIS_PACKAGE=OFF --packages-select autoware_localized_position_validator
```

> [!NOTE]
> This package will not get built with only `colcon build` command.  
> You need to set `--cmake-args -DSKIP_THIS_PACKAGE=OFF`.  
> This is to avoid the long compilation time (mainly due to LibTorch) for this package.


# Launch

Before launching, you need to add following launch settings to a launch file.
```xml
<include file="$(find-pkg-share autoware_localized_position_validator)/launch/localized_position_validator.launch.xml">
  <arg name="input/odometry" value="/localization/kinematic_state"/>
  <arg name="input/pointcloud" value="/localization/pose_estimator/points_aligned"/>
  <arg name="input/pointcloud_map" value="/map/pointcloud_map"/>
  <arg name="input/partial_pointcloud_map" value="/map/get_partial_pointcloud_map"/>
</include>
```
For example, you can add to [localization_error_monitor.launch.xml](https://github.com/autowarefoundation/autoware_universe/blob/main/launch/tier4_localization_launch/launch/localization_error_monitor/localization_error_monitor.launch.xml).  
Or manually launch the node by
```sh
ros2 launch autoware_localized_position_validator localized_position_validator.launch.xml
```


# Inputs / Outputs

## Inputs
| Name                         | Type                                            | Description                                    |
| ---------------------------- | ----------------------------------------------- | ---------------------------------------------- |
| input/odometry               | nav_msgs::msg::Odometry                         | Ego vehicle odometry topic.                    |
| input/pointcloud_map         | sensor_msgs::msg::PointCloud2                   | Map (when partial map loading option is false) |
| input/partial_pointcloud_map | autoware_map_msgs::srv::GetPartialPointCloudMap | Map (when partial map loading option is true)  |
| input/pointcloud             | sensor_msgs::msg::PointCloud2                   | Topic from NDT (points_aligned)                |

## Output
| Name              | Type                                                               | Description       |
| ----------------- | ------------------------------------------------------------------ | ----------------- |
| validation_result | tier4_localization_msgs::msg::LocalizedPositionValidatorPrediction | Prediction result |


# NOTICES

This package contains a modified code from https://github.com/zhulf0804/PointPillars (MIT License).
See [ThirdPartyNotices](ThirdPartyNotices) for more details.


# Reference

A. H. Lang, S. Vora, H. Caesar, L. Zhou, J. Yang and O. Beijbom, "PointPillars: Fast Encoders for Object Detection From Point Clouds," 2019 IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), Long Beach, CA, USA, 2019, pp. 12689-12697, doi: 10.1109/CVPR.2019.01298.
