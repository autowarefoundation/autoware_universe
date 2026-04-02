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

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
   * @param ego_box Ego vehicle bounding box in lidar frame
   * @param config Configuration parameters for processing
   * @throws std::runtime_error if TensorRT engine setup fails
   */
  explicit CalibrationStatusClassifier(
    const std::string & onnx_path, const std::string & trt_precision, int64_t cloud_capacity,
    const std::vector<double> & ego_box, const CalibrationStatusClassifierConfig & config);

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
  /**
   * @brief Generate ego vehicle occlusion mask for a camera-lidar pair
   *
   * Prevents LiDAR points that physically pass through the ego vehicle chassis from being
   * projected onto the camera image. Without this mask, points reflected off the ground
   * beneath the vehicle (or the chassis itself) would create false projections, polluting
   * both the preview image and the neural network input.
   *
   * The mask is computed once per camera-lidar pair (static geometry) then cached on
   * GPU for zero-copy reuse on subsequent frames.
   *
   * ## Algorithm
   *
   * For every pixel (u, v) in the rectified image:
   * 1. Back-project via P_3x3^{-1} to get a ray direction in the camera frame.
   * 2. Transform the ray direction to the lidar frame using R^T (rotation transpose).
   * 3. Perform a slab intersection test between the ray (originating at the camera
   *    center in lidar frame, c = -R^T * t) and the axis-aligned ego bounding box.
   * 4. If the ray intersects the box in front of the camera (t_exit >= 0), the pixel
   *    is marked as occluded (0); otherwise it remains valid (1).
   *
   * ## Ego box adjustment when camera is inside the box
   *
   * Cameras mounted on the vehicle roof may fall geometrically inside the configured
   * ego bounding box. In that case, every ray would trivially intersect the box,
   * blanking the entire image. To handle this, the closest X/Y wall is pushed past
   * the camera center (by 0.01 m), effectively "opening" the box on the camera side
   * so only the far walls produce occlusion:
   *
   *   Top-down view (lidar frame, X forward, Y left):
   *
   *             new_x_max
   *                 ^
   *        y_min ───:─────────────────── y_min
   *            │    :                   │
   *            │    :                   │
   *            │    :        base_link  │
   *            │    :                   │         All rays from C toward the far
   *            │    :                   │         (bottom) wall are occluded. The
   *       x_max│....:...................│x_min    near (top) wall is pushed past C
   *            │    C           ^       │         (camera) so it does not block
   *            │                │       │         anything.
   *       y_max ────────────────│─────── y_max
   *                         new_y_min
   *
   *   Result on image with projected points:
   *
   *   ┌────────────────────────────────────┐
   *   │\     RGB image fully visible       │
   *   │ \                                  │
   *   │  \      Projected   o o o o        │
   *   │   \     points ---> o o o o o      │
   *   │    |                o o o o o o    │
   *   │    |                               │
   *   │    | <--- No projected             │
   *   │    |      points here              │
   *   │    |      as this is               │
   *   │    |      ego box                  │
   *   └────────────────────────────────────┘
   *
   * @param camera_lidar_info Camera and LiDAR calibration parameters
   * @param image_width Image width in pixels
   * @param image_height Image height in pixels
   * @return Binary mask (H*W) where 1=valid, 0=occluded by ego chassis
   */
  std::vector<uint8_t> generate_ego_occlusion_mask(
    const CameraLidarInfo & camera_lidar_info, std::size_t image_width,
    std::size_t image_height) const;

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
  std::map<std::string, autoware::cuda_utils::CudaUniquePtr<uint8_t[]>> ego_masks_d_;
  cudaStream_t stream_{nullptr};
  size_t cloud_capacity_;
  std::vector<double> ego_box_;
  const CalibrationStatusClassifierConfig config_;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_HPP_
