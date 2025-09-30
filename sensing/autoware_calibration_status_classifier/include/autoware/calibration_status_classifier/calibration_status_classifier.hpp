// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_HPP_

#include "autoware/calibration_status_classifier/config.hpp"
#include "autoware/calibration_status_classifier/data_type.hpp"
#include "autoware/calibration_status_classifier/data_type_eigen.hpp"
#include "autoware/calibration_status_classifier/preprocess_cuda.hpp"
#include "autoware/calibration_status_classifier/visibility_control.hpp"

#include <Eigen/Geometry>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

namespace autoware::calibration_status_classifier
{

/**
 * @brief Core calibration status detection class using CUDA and TensorRT
 *
 * This class implements deep learning-based LiDAR-camera calibration validation.
 * It processes synchronized point cloud and image data through:
 * 1. CUDA-accelerated image undistortion
 * 2. Point cloud projection onto undistorted images
 * 3. TensorRT neural network inference for miscalibration detection
 *
 * The neural network analyzes 5-channel input data (RGB + depth + intensity)
 * to classify calibration status as calibrated or miscalibrated.
 */
class CALIBRATION_STATUS_PUBLIC CalibrationStatusClassifier
{
public:
  /**
   * @brief Constructor for CalibrationStatusClassifier
   * @param onnx_path Path to the ONNX model file for TensorRT engine creation
   * @param trt_precision TensorRT precision mode
   * @param cloud_capacity Maximum number of LiDAR points to process
   * @param config Configuration parameters for processing
   * @throws std::runtime_error if TensorRT engine setup fails
   */
  explicit CalibrationStatusClassifier(
    const std::string & onnx_path, const std::string & trt_precision, int64_t cloud_capacity,
    const CalibrationStatusClassifierConfig & config);

  /**
   * @brief Destructor
   */
  ~CalibrationStatusClassifier() = default;

  /**
   * @brief Process synchronized sensor data to determine calibration status
   *
   * This method performs the complete processing pipeline:
   * 1. Copy input data to GPU memory
   * 2. Undistort camera image using intrinsic parameters
   * 3. Project LiDAR points onto undistorted image plane
   * 4. Run neural network inference on 5-channel data
   * 5. Return calibration status and confidence scores
   *
   * @param cloud_msg Point cloud data from LiDAR sensor
   * @param image_msg Raw camera image (BGR8 format)
   * @param camera_lidar_info Camera and LiDAR intrinsic/extrinsic parameters
   * @param preview_img_data Output buffer for visualization image with projected points
   * @return CalibrationStatusClassifierResult containing validation results and timing information
   */
  CalibrationStatusClassifierResult process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const CameraLidarInfo & camera_lidar_info, uint8_t * preview_img_data);

private:
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_;
  std::unique_ptr<PreprocessCuda> preprocess_ptr_;
  autoware::cuda_utils::CudaUniquePtr<float[]> in_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> out_d_;
  autoware::cuda_utils::CudaUniquePtr<InputPointType[]> cloud_d_;
  autoware::cuda_utils::CudaUniquePtr<InputImageBGR8Type[]> image_d_;
  autoware::cuda_utils::CudaUniquePtr<InputImageBGR8Type[]> image_undistorted_d_;
  autoware::cuda_utils::CudaUniquePtr<double[]> dist_coeffs_d_;
  autoware::cuda_utils::CudaUniquePtr<double[]> camera_matrix_d_;
  autoware::cuda_utils::CudaUniquePtr<double[]> projection_matrix_d_;
  autoware::cuda_utils::CudaUniquePtr<double[]> tf_matrix_d_;
  autoware::cuda_utils::CudaUniquePtr<uint32_t> num_points_projected_d_;
  cudaStream_t stream_{nullptr};
  size_t cloud_capacity_;
  const CalibrationStatusClassifierConfig config_;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_HPP_
