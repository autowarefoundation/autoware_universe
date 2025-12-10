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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTER_INTERFACE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTER_INTERFACE_HPP_

#include "processing_state.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cuda_runtime.h>
#include <tf2_ros/buffer.h>

#include <any>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Forward declaration to avoid circular dependency
namespace autoware::cuda_pointcloud_preprocessor
{
class CudaPointcloudPreprocessor;
}

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Typed inputs provided to filters
 * The DAG executor identifies and casts types before passing to filters
 * NOTE: Now uses PointcloudProcessingState for zero-copy between filters
 */
struct TypedInputs
{
  // Processing state inputs (lightweight metadata + non-owning device pointer)
  // Zero-copy: filters work directly on GPU memory without allocations
  std::map<std::string, std::shared_ptr<PointcloudProcessingState>> processing_states;

  // Named inputs for special data types (twist, imu)
  // These are marker inputs in YAML - actual data comes from context queues
  std::vector<std::string> special_inputs;  // e.g., "twist", "imu"
};

/**
 * @brief Execution context for filters
 * Provides shared resources like CUDA streams, memory pools, and ROS interfaces
 */
struct FilterContext
{
  cudaStream_t stream{};
  cudaMemPool_t memory_pool{};
  tf2_ros::Buffer * tf_buffer{nullptr};
  rclcpp::Clock * clock{nullptr};
  rclcpp::Logger * logger{nullptr};  // Pointer instead of value (Logger has private constructor)

  // SHARED preprocessor instance (one instance reused by all filters)
  // This allows filters to delegate to existing implementations without code duplication
  autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessor * shared_preprocessor{nullptr};

  // Additional context data for distortion correction
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> * twist_queue{nullptr};
  std::deque<geometry_msgs::msg::Vector3Stamped> * angular_velocity_queue{nullptr};
};

/**
 * @brief Filter metadata for validation and introspection
 */
struct FilterMetadata
{
  std::string filter_type;
  std::vector<std::string> required_inputs;
  std::vector<std::string> optional_inputs;
  std::vector<std::string> outputs;
  std::map<std::string, std::string> input_types;  // name -> ROS message type
  std::map<std::string, std::string> output_types;
};

/**
 * @brief Base interface for all DAG filter nodes
 */
class IFilter
{
public:
  virtual ~IFilter() = default;

  /**
   * @brief Initialize the filter with parameters
   * @param params Map containing filter-specific parameters
   */
  virtual void initialize(const std::map<std::string, std::any> & params) = 0;

  /**
   * @brief Process input data and produce output
   * @param inputs Typed inputs (pointclouds already cast by DAG executor)
   * @param outputs Map of output names to data pointers (to be filled)
   * @param context Execution context (stream, memory pool, etc.)
   * @param output_names Expected output names from YAML configuration
   */
  virtual void process(
    const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context, const std::vector<std::string> & output_names) = 0;

  /**
   * @brief Get filter metadata
   */
  virtual FilterMetadata getMetadata() const = 0;

  /**
   * @brief Validate inputs before processing
   * @param inputs Typed inputs
   * @return true if inputs are valid, false otherwise
   */
  virtual bool validateInputs(const TypedInputs & inputs) const = 0;
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTER_INTERFACE_HPP_
