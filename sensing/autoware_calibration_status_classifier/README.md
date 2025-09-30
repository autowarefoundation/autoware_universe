# autoware_calibration_status_classifier

## Purpose

The `autoware_calibration_status_classifier` package provides real-time LiDAR-camera calibration validation using deep learning inference. It detects miscalibration between LiDAR and camera sensors by analyzing projected point clouds overlaid on camera images through a neural network-based approach.

## Inner-workings / Algorithms

The calibration status detection system operates through the following pipeline:

### 1. Data Preprocessing (CUDA-accelerated)

- **Image Undistortion**: Corrects camera distortion
- **Point Cloud Projection**: Projects 3D LiDAR points onto undistorted 2D image plane - adds depth and intensity information
- **Morphological Dilation**: Enhances point visibility for neural network input

### 2. Neural Network Inference (TensorRT)

- **Input Format**: 5-channel normalized data (RGB + depth + intensity)
- **Architecture**: Deep neural network trained on calibrated/miscalibrated data
- **Output**: Binary classification with confidence scores for calibration status

### 3. Runtime Modes

- **MANUAL**: On-demand validation via service calls
- **PERIODIC**: Regular validation at configurable intervals
- **ACTIVE**: Continuous monitoring with synchronized sensor data

## Inputs / Outputs

### Input

| Name                 | Type                                     | Description                                             |
| -------------------- | ---------------------------------------- | ------------------------------------------------------- |
| `~/input/velocity`   | `prerequisite.velocity_source` parameter | Vehicle velocity (multiple message types supported)     |
| `input.cloud_topics` | `sensor_msgs::msg::PointCloud2`          | LiDAR point cloud data                                  |
| `input.image_topics` | `sensor_msgs::msg::Image`                | Camera image data (BGR8 format)                         |
| Camera info topics   | `sensor_msgs::msg::CameraInfo`           | Camera intrinsic parameters and distortion coefficients |

### Output

| Name                         | Type                                    | Description                                |
| ---------------------------- | --------------------------------------- | ------------------------------------------ |
| `/diagnostics`               | `diagnostic_msgs::msg::DiagnosticArray` | ROS diagnostics with calibration status    |
| `~/validate_calibration_srv` | `std_srvs::srv::Trigger`                | Manual validation service (MANUAL mode)    |
| Preview image topics         | `sensor_msgs::msg::Image`               | Visualization images with projected points |

### Services

| Name                               | Type                     | Description                           |
| ---------------------------------- | ------------------------ | ------------------------------------- |
| `~/input/validate_calibration_srv` | `std_srvs::srv::Trigger` | Manual calibration validation request |

## Parameters

### Node Parameters

{{ json_to_markdown("sensing/autoware_calibration_status_classifier/schema/calibration_status_classifier.schema.json") }}

### Network Parameters

{{ json_to_markdown("sensing/autoware_calibration_status_classifier/schema/ml_package_calibration_status_classifier.schema.json") }}

## Assumptions / Known Limits

- Input images must be in BGR8 format (8-bit per channel)
- Input point clouds should contain intensity information (XYZIRC format)

## Usage Example

```bash
ros2 launch autoware_calibration_status_classifier calibration_status_classifier.launch.xml
```

## Future Extensions / Unimplemented Parts

- Manual runtime mode with detailed response (custom srv)
- Replace filter for objects on the scene counter to objects within the camera FOV counter (raytracing)
- Multithreading for multiple camera-LiDAR pairs
- More filters (e.g. yaw rate)
- cuda_blackboard support
- Replace custom kernels with NPP functions where applicable

## References

- [AWML - Calibration Status Classification](https://github.com/tier4/AWML/tree/main/projects/CalibrationStatusClassification)
