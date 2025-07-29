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

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cuda_polar_voxel_outlier_filter_node.hpp"

#include "autoware/pointcloud_preprocessor/filter.hpp"  // for get_param
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"  // for autoware::pointcloud_preprocessor::utils

namespace autoware::cuda_pointcloud_preprocessor
{
CudaPolarVoxelOutlierFilterNode::CudaPolarVoxelOutlierFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_polar_voxexl_outlier_filter", node_options)
{
  // set initial parameters
  {
    filter_params_.radius_resolution = declare_parameter<double>("radius_resolution");
    filter_params_.azimuth_resolution = declare_parameter<double>("azimuth_resolution");
    filter_params_.elevation_resolution = declare_parameter<double>("elevation_resolution");
    filter_params_.voxel_points_threshold = declare_parameter<int>("voxel_points_threshold");
    filter_params_.min_radius = declare_parameter<double>("min_radius");
    filter_params_.max_radius = declare_parameter<double>("max_radius");
    filter_params_.use_return_type_classification =
      declare_parameter<bool>("use_return_type_classification");
    filter_params_.filter_secondary_returns = declare_parameter<bool>("filter_secondary_returns");
    filter_params_.secondary_noise_threshold = declare_parameter<int>("secondary_noise_threshold");
    filter_params_.visibility_error_threshold =
      declare_parameter<double>("visibility_error_threshold", 0.5);
    filter_params_.visibility_warn_threshold =
      declare_parameter<double>("visibility_warn_threshold", 0.7);
    filter_params_.filter_ratio_error_threshold =
      declare_parameter<double>("filter_ratio_error_threshold", 0.5);
    filter_params_.filter_ratio_warn_threshold =
      declare_parameter<double>("filter_ratio_warn_threshold", 0.7);

    primary_return_types_ = declare_parameter<std::vector<int>>("primary_return_types");
    secondary_return_types_ = declare_parameter<std::vector<int>>("secondary_return_types");
  }

  cuda_polar_voxel_outlier_filter_ = std::make_unique<CudaPolarVoxelOutlierFilter>();
  cuda_polar_voxel_outlier_filter_->set_primary_return_types(primary_return_types_);
  cuda_polar_voxel_outlier_filter_->set_secondary_return_types(secondary_return_types_);

  std::string diagnostics_hardware_id =
    declare_parameter<std::string>("hardware_id", "cuda_polar_voxel_outlier_filter");

  // Initialize diagnostics
  visibility_ = -1.0;
  filter_ratio_ = -1.0;
  updater_.setHardwareID(diagnostics_hardware_id);
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &CudaPolarVoxelOutlierFilterNode::onVisibilityChecker);
  updater_.add(
    std::string(this->get_namespace()) + ": filter_ratio_validation", this,
    &CudaPolarVoxelOutlierFilterNode::onFilterRatioChecker);
  updater_.setPeriod(0.1);

  // Create visibility publisher
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/visibility", rclcpp::SensorDataQoS());

  // Create ratio publisher
  ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/filter_ratio", rclcpp::SensorDataQoS());

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaPolarVoxelOutlierFilterNode::pointcloud_callback, this, std::placeholders::_1));

  filtered_cloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  noise_cloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/debug/pointcloud_noise");
}

void CudaPolarVoxelOutlierFilterNode::pointcloud_callback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // Take mutex so that node configuration will not be
  // ovewritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filtered_cloud;
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> noise_cloud;
  if (autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(
        *msg)) {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZIRCAEDT format with pre-computed polar coordinates");
    auto filter_return =
      cuda_polar_voxel_outlier_filter_->filter_point_xyzircaedt(msg, filter_params_);
    filtered_cloud = std::move(filter_return.filtered_cloud);
    noise_cloud = std::move(filter_return.noise_cloud);
    filter_ratio_ = filter_return.filter_ratio;
    visibility_ = filter_return.visibility;
  } else {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZ format, computing polar coordinates");
    // TODO: filter_point_xyz(msg);
  }

  // Publish results
  filtered_cloud_pub_->publish(std::move(filtered_cloud));
  noise_cloud_pub_->publish(std::move(noise_cloud));

  autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
  ratio_msg.data = static_cast<float>(filter_ratio_);
  ratio_msg.stamp = now();
  ratio_pub_->publish(ratio_msg);

  if (filter_params_.use_return_type_classification) {
    autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.data = static_cast<float>(visibility_);
    visibility_msg.stamp = now();
    visibility_pub_->publish(visibility_msg);
  }
}

rcl_interfaces::msg::SetParametersResult CudaPolarVoxelOutlierFilterNode::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  using autoware::pointcloud_preprocessor::get_param;
  std::scoped_lock lock(param_mutex_);

  if (get_param(p, "radius_resolution", filter_params_.radius_resolution)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new radius resolution to: %f.", filter_params_.radius_resolution);
  }
  if (get_param(p, "azimuth_resolution", filter_params_.azimuth_resolution)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new azimuth resolution to: %f.", filter_params_.azimuth_resolution);
  }
  if (get_param(p, "elevation_resolution", filter_params_.elevation_resolution)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new elevation resolution to: %f.",
      filter_params_.elevation_resolution);
  }
  if (get_param(p, "voxel_points_threshold", filter_params_.voxel_points_threshold)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new voxel points threshold to: %d.",
      filter_params_.voxel_points_threshold);
  }
  if (get_param(p, "min_radius", filter_params_.min_radius)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min radius to: %f.", filter_params_.min_radius);
  }
  if (get_param(p, "max_radius", filter_params_.max_radius)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max radius to: %f.", filter_params_.max_radius);
  }
  if (get_param(
        p, "use_return_type_classification", filter_params_.use_return_type_classification)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting use return type classification to: %s.",
      filter_params_.use_return_type_classification ? "true" : "false");
  }
  if (get_param(p, "filter_secondary_returns", filter_params_.filter_secondary_returns)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting filter secondary returns to: %s.",
      filter_params_.filter_secondary_returns ? "true" : "false");
  }
  if (get_param(p, "secondary_noise_threshold", filter_params_.secondary_noise_threshold)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting secondary noise threshold to: %d.",
      filter_params_.secondary_noise_threshold);
  }
  if (get_param(p, "filter_ratio_error_threshold", filter_params_.filter_ratio_error_threshold)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new filter ratio error threshold to: %f.",
      filter_params_.filter_ratio_error_threshold);
  }
  if (get_param(p, "filter_ratio_warn_threshold", filter_params_.filter_ratio_warn_threshold)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new filter ratio warning threshold to: %f.",
      filter_params_.filter_ratio_warn_threshold);
  }
  if (get_param(p, "visibility_error_threshold", filter_params_.visibility_error_threshold)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new visibility error threshold to: %f.",
      filter_params_.visibility_error_threshold);
  }
  if (get_param(p, "visibility_warn_threshold", filter_params_.visibility_warn_threshold)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new visibility warning threshold to: %f.",
      filter_params_.visibility_warn_threshold);
  }

  if (get_param(p, "primary_return_types", primary_return_types_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new primary return types");
    cuda_polar_voxel_outlier_filter_->set_primary_return_types(primary_return_types_);
  }
  if (get_param(p, "secondary_return_types", secondary_return_types_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new secondary return types");
    cuda_polar_voxel_outlier_filter_->set_secondary_return_types(secondary_return_types_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void CudaPolarVoxelOutlierFilterNode::onVisibilityChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Take mutex so that node configuration will not be
  // ovewritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  using diagnostic_msgs::msg::DiagnosticStatus;

  // Add values
  stat.add("value", std::to_string(visibility_));

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (visibility_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (visibility_ < filter_params_.visibility_error_threshold) {
    level = DiagnosticStatus::ERROR;
    msg =
      "ERROR: critically low LiDAR visibility detected while filtering outliers in polar voxel "
      "filter";
  } else if (visibility_ < filter_params_.visibility_warn_threshold) {
    level = DiagnosticStatus::WARN;
    msg = "WARNING: low LiDAR visibility detected while filtering outliers in polar voxel filter";
  }
  stat.summary(level, msg);
}

void CudaPolarVoxelOutlierFilterNode::onFilterRatioChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Take mutex so that node configuration will not be
  // ovewritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  using diagnostic_msgs::msg::DiagnosticStatus;

  // Add values
  stat.add("value", std::to_string(filter_ratio_));

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (filter_ratio_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (filter_ratio_ < filter_params_.filter_ratio_error_threshold) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR: critically low filter ratio in polar voxel outlier filter";
  } else if (filter_ratio_ < filter_params_.filter_ratio_warn_threshold) {
    level = DiagnosticStatus::WARN;
    msg = "WARNING: low filter ratio in polar voxel outlier filter";
  }
  stat.summary(level, msg);
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPolarVoxelOutlierFilterNode)
