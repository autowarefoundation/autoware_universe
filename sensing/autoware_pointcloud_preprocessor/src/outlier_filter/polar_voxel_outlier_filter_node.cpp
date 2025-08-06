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

#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp"

#include <autoware/pointcloud_preprocessor/utility/memory.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

// Complete structure for point processing results
struct PointProcessingResult {
  std::vector<bool> valid_points;
  size_t input_points;
  size_t output_points;
  size_t voxels_passed_secondary_test = 0;
  size_t voxels_failed_secondary_test = 0;
  size_t populated_voxels = 0;
  size_t total_voxels = 0;
};

PolarVoxelOutlierFilterComponent::PolarVoxelOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PolarVoxelOutlierFilter", options)
{
  // set initial parameters
  {
    radial_resolution_m_ = declare_parameter<double>("radial_resolution_m");
    azimuth_resolution_rad_ = declare_parameter<double>("azimuth_resolution_rad");
    elevation_resolution_rad_ = declare_parameter<double>("elevation_resolution_rad");
    voxel_points_threshold_ = declare_parameter<int>("voxel_points_threshold");
    min_radius_m_ = declare_parameter<double>("min_radius_m");
    max_radius_m_ = declare_parameter<double>("max_radius_m");
    use_return_type_classification_ = declare_parameter<bool>("use_return_type_classification");
    filter_secondary_returns_ = declare_parameter<bool>("filter_secondary_returns");
    secondary_noise_threshold_ = declare_parameter<int>("secondary_noise_threshold");
    primary_return_types_ = declare_parameter<std::vector<int>>("primary_return_types");
    visibility_error_threshold_ = declare_parameter<double>("visibility_error_threshold", 0.5);
    visibility_warn_threshold_ = declare_parameter<double>("visibility_warn_threshold", 0.7);
    filter_ratio_error_threshold_ = declare_parameter<double>("filter_ratio_error_threshold", 0.5);
    filter_ratio_warn_threshold_ = declare_parameter<double>("filter_ratio_warn_threshold", 0.7);
  }

  // Initialize diagnostics
  visibility_ = -1.0;
  filter_ratio_ = -1.0;
  updater_.setHardwareID("polar_voxel_outlier_filter");
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &PolarVoxelOutlierFilterComponent::onVisibilityChecker);
  updater_.add(
    std::string(this->get_namespace()) + ": filter_ratio_validation", this,
    &PolarVoxelOutlierFilterComponent::onFilterRatioChecker);
  updater_.setPeriod(0.1);

  // Create visibility publisher
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "polar_voxel_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());

  // Create ratio publisher
  ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "polar_voxel_outlier_filter/debug/filter_ratio", rclcpp::SensorDataQoS());

  // Create noise points publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PolarVoxelOutlierFilterComponent::param_callback, this, _1));
}

PolarVoxelOutlierFilterComponent::PolarCoordinate 
PolarVoxelOutlierFilterComponent::cartesian_to_polar(const CartesianCoordinate& cartesian)
{
  double radius = std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z);
  double azimuth = std::atan2(cartesian.y, cartesian.x);
  double elevation = std::atan2(cartesian.z, std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y));
  
  return PolarCoordinate(radius, azimuth, elevation);
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex
PolarVoxelOutlierFilterComponent::cartesian_to_polar_voxel(const CartesianCoordinate& cartesian) const
{
  PolarCoordinate polar = cartesian_to_polar(cartesian);
  return polar_to_polar_voxel(polar);
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex
PolarVoxelOutlierFilterComponent::polar_to_polar_voxel(const PolarCoordinate& polar) const
{
  PolarVoxelIndex voxel_idx{};
  voxel_idx.radius_idx = static_cast<int32_t>(std::floor(polar.radius / radial_resolution_m_));
  voxel_idx.azimuth_idx = static_cast<int32_t>(std::floor(polar.azimuth / azimuth_resolution_rad_));
  voxel_idx.elevation_idx = static_cast<int32_t>(std::floor(polar.elevation / elevation_resolution_rad_));
  return voxel_idx;
}

bool PolarVoxelOutlierFilterComponent::is_primary_return_type(uint8_t return_type) const
{
  auto it = std::find(primary_return_types_.begin(), primary_return_types_.end(), 
                      static_cast<int64_t>(return_type));
  return it != primary_return_types_.end();
}

// Helper function for point validation
bool PolarVoxelOutlierFilterComponent::validate_point_basic(const PolarCoordinate& polar) const
{
  // Skip points with NaN or Inf values
  if (!std::isfinite(polar.radius) || !std::isfinite(polar.azimuth) || !std::isfinite(polar.elevation)) {
    return false;
  }

  // Skip points outside the configured radius range
  if (polar.radius < min_radius_m_ || polar.radius > max_radius_m_) {
    return false;
  }

  // Skip points with insufficient radius (degenerate points at origin)
  if (std::abs(polar.radius) < std::numeric_limits<double>::epsilon()) {
    return false;
  }

  return true;
}

// Helper function to publish diagnostics
void PolarVoxelOutlierFilterComponent::publish_diagnostics(
  const PointProcessingResult& result, bool has_return_type_classification)
{
  // Calculate and publish filter ratio
  filter_ratio_ = result.input_points > 0 ? 
    static_cast<double>(result.output_points) / static_cast<double>(result.input_points) : 0.0;

  // Publish ratio
  autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
  ratio_msg.data = static_cast<float>(filter_ratio_);
  ratio_msg.stamp = now();
  ratio_pub_->publish(ratio_msg);

  // Only publish visibility when return type classification is enabled
  if (has_return_type_classification && use_return_type_classification_) {
    size_t total_voxels_with_points = result.total_voxels;

    // Visibility is the percentage of voxels that passed the primary-to-secondary ratio test
    visibility_ = total_voxels_with_points > 0 ? 
      static_cast<double>(result.voxels_passed_secondary_test) / static_cast<double>(total_voxels_with_points) : 0.0;

    // Publish visibility
    autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.data = static_cast<float>(visibility_);
    visibility_msg.stamp = now();
    visibility_pub_->publish(visibility_msg);

    RCLCPP_DEBUG(
      get_logger(), "Visibility: %.3f (%zu/%zu voxels passed secondary test, %zu failed)",
      visibility_, result.voxels_passed_secondary_test, total_voxels_with_points,
      result.voxels_failed_secondary_test);
  }

  // Debug output for voxel statistics
  RCLCPP_DEBUG(
    get_logger(), "Filter ratio: %.3f (%zu/%zu points), Voxel stats: %zu total, %zu populated",
    filter_ratio_, result.output_points, result.input_points, result.total_voxels, result.populated_voxels);
}

void PolarVoxelOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  // Add input validation
  if (!input) {
    RCLCPP_ERROR(get_logger(), "Input point cloud is null");
    return;
  }

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  if (autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(
        *input)) {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZIRCAEDT format with pre-computed polar coordinates");
    filter_point_xyzircaedt(input, indices, output);
  } else {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZ format, computing polar coordinates");
    filter_point_xyz(input, indices, output);
  }
}

void PolarVoxelOutlierFilterComponent::filter_point_xyz(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);

  // Structure to store voxel information for each point
  struct VoxelInfo {
    PolarVoxelIndex voxel_idx;
  };

  // Phase 1: Collect valid point voxel information
  std::vector<std::optional<VoxelInfo>> point_voxel_info(pcl_input->points.size());
  
  for (size_t point_idx = 0; point_idx < pcl_input->points.size(); ++point_idx) {
    const auto & point = pcl_input->points[point_idx];

    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    CartesianCoordinate cartesian(
      static_cast<double>(point.x), 
      static_cast<double>(point.y), 
      static_cast<double>(point.z));

    PolarCoordinate polar = cartesian_to_polar(cartesian);

    if (validate_point_basic(polar)) {
      PolarVoxelIndex voxel_idx = polar_to_polar_voxel(polar);
      point_voxel_info[point_idx] = VoxelInfo{voxel_idx};
    }
  }

  // Phase 2: Count and validate voxels
  std::unordered_map<PolarVoxelIndex, int, PolarVoxelIndexHash> voxel_counts;
  for (const auto& info_opt : point_voxel_info) {
    if (info_opt.has_value()) {
      voxel_counts[info_opt.value().voxel_idx]++;
    }
  }

  std::unordered_set<PolarVoxelIndex, PolarVoxelIndexHash> valid_voxels;
  for (const auto& [voxel_idx, count] : voxel_counts) {
    if (count >= voxel_points_threshold_) {
      valid_voxels.insert(voxel_idx);
    }
  }

  // Phase 3: Mark valid points and create output
  std::vector<bool> valid_points(pcl_input->points.size(), false);
  for (size_t i = 0; i < point_voxel_info.size(); ++i) {
    if (point_voxel_info[i].has_value() && 
        valid_voxels.count(point_voxel_info[i].value().voxel_idx) > 0) {
      valid_points[i] = true;
    }
  }

  // Create filtered output
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr noise_output(new pcl::PointCloud<pcl::PointXYZ>);

  size_t valid_count = std::count(valid_points.begin(), valid_points.end(), true);
  pcl_output->points.reserve(valid_count);

  // Reserve space for noise points - only processed points that were filtered out
  size_t noise_count = 0;
  for (size_t i = 0; i < valid_points.size(); ++i) {
    if (!valid_points[i] && point_voxel_info[i].has_value()) noise_count++;
  }
  noise_output->points.reserve(noise_count);

  // Copy valid and noise points
  for (size_t i = 0; i < valid_points.size(); ++i) {
    if (valid_points[i]) {
      pcl_output->points.push_back(pcl_input->points[i]);
    } else if (point_voxel_info[i].has_value()) {
      noise_output->points.push_back(pcl_input->points[i]);
    }
  }

  // Convert to ROS messages and publish
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;

  // Publish noise points
  sensor_msgs::msg::PointCloud2 noise_output_msg;
  pcl::toROSMsg(*noise_output, noise_output_msg);
  noise_output_msg.header = input->header;
  noise_cloud_pub_->publish(noise_output_msg);

  // Publish diagnostics
  PointProcessingResult result;
  result.input_points = pcl_input->points.size();
  result.output_points = valid_count;
  result.total_voxels = voxel_counts.size();
  result.populated_voxels = valid_voxels.size();
  
  publish_diagnostics(result, false);
}

void PolarVoxelOutlierFilterComponent::filter_point_xyzircaedt(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning

  // Structure to store voxel information for each point
  struct VoxelInfo {
    PolarVoxelIndex voxel_idx;
    bool is_primary;
  };

  // Phase 1: Collect valid point voxel information
  std::vector<std::optional<VoxelInfo>> point_voxel_info(input->width * input->height);
  
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(*input, "distance");
  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(*input, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_elevation(*input, "elevation");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(*input, "return_type");

  size_t point_idx = 0;
  for (; iter_distance != iter_distance.end();
       ++iter_distance, ++iter_azimuth, ++iter_elevation, ++iter_return_type, ++point_idx) {
    
    auto radius = static_cast<double>(*iter_distance);
    auto azimuth = static_cast<double>(*iter_azimuth);
    auto elevation = static_cast<double>(*iter_elevation);
    uint8_t return_type = *iter_return_type;

    PolarCoordinate polar(radius, azimuth, elevation);

    if (validate_point_basic(polar)) {
      PolarVoxelIndex voxel_idx = polar_to_polar_voxel(polar);
      bool is_primary = is_primary_return_type(return_type);
      point_voxel_info[point_idx] = VoxelInfo{voxel_idx, is_primary};
    }
  }

  // Phase 2: Count and validate voxels with dual criteria
  std::unordered_map<PolarVoxelIndex, std::pair<int, int>, PolarVoxelIndexHash> voxel_counts;
  for (const auto& info_opt : point_voxel_info) {
    if (info_opt.has_value()) {
      const auto& info = info_opt.value();
      if (info.is_primary) {
        voxel_counts[info.voxel_idx].first++;
      } else {
        voxel_counts[info.voxel_idx].second++;
      }
    }
  }

  std::unordered_set<PolarVoxelIndex, PolarVoxelIndexHash> valid_voxels;
  size_t voxels_passed_secondary_test = 0;
  size_t voxels_failed_secondary_test = 0;
  
  for (const auto& [voxel_idx, counts] : voxel_counts) {
    bool primary_meets_threshold = counts.first >= voxel_points_threshold_;
    bool secondary_meets_threshold = counts.second <= secondary_noise_threshold_;

    if (secondary_meets_threshold) {
      voxels_passed_secondary_test++;
    } else {
      voxels_failed_secondary_test++;
    }

    if (primary_meets_threshold && secondary_meets_threshold) {
      valid_voxels.insert(voxel_idx);
    }
  }

  // Phase 3: Mark valid points and create output
  std::vector<bool> valid_points(input->width * input->height, false);
  for (size_t i = 0; i < point_voxel_info.size(); ++i) {
    if (point_voxel_info[i].has_value()) {
      const auto& info = point_voxel_info[i].value();
      if (valid_voxels.count(info.voxel_idx) > 0) {
        if (info.is_primary || !filter_secondary_returns_) {
          valid_points[i] = true;
        }
      }
    }
  }

  // Create filtered output
  sensor_msgs::msg::PointCloud2 filtered_cloud;
  filtered_cloud.header = input->header;
  filtered_cloud.fields = input->fields;
  filtered_cloud.is_bigendian = input->is_bigendian;
  filtered_cloud.point_step = input->point_step;
  filtered_cloud.is_dense = input->is_dense;

  size_t valid_count = std::count(valid_points.begin(), valid_points.end(), true);
  filtered_cloud.height = 1;
  filtered_cloud.width = valid_count;
  filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;
  filtered_cloud.data.resize(filtered_cloud.row_step);

  // Copy valid points
  size_t output_idx = 0;
  for (size_t i = 0; i < valid_points.size(); ++i) {
    if (valid_points[i]) {
      std::memcpy(
        &filtered_cloud.data[output_idx * filtered_cloud.point_step],
        &input->data[i * input->point_step], input->point_step);
      output_idx++;
    }
  }

  output = filtered_cloud;

  // Create and publish noise cloud
  sensor_msgs::msg::PointCloud2 noise_cloud;
  noise_cloud.header = input->header;
  noise_cloud.fields = input->fields;
  noise_cloud.is_bigendian = input->is_bigendian;
  noise_cloud.point_step = input->point_step;
  noise_cloud.is_dense = input->is_dense;

  size_t noise_count = input->width * input->height - valid_count;
  noise_cloud.height = 1;
  noise_cloud.width = noise_count;
  noise_cloud.row_step = noise_cloud.width * noise_cloud.point_step;
  noise_cloud.data.resize(noise_cloud.row_step);

  // Copy filtered-out points
  size_t noise_idx = 0;
  for (size_t i = 0; i < valid_points.size(); ++i) {
    if (!valid_points[i]) {
      std::memcpy(
        &noise_cloud.data[noise_idx * noise_cloud.point_step], 
        &input->data[i * input->point_step],
        input->point_step);
      noise_idx++;
    }
  }

  noise_cloud_pub_->publish(noise_cloud);

  // Publish diagnostics
  PointProcessingResult result;
  result.input_points = input->width * input->height;
  result.output_points = valid_count;
  result.voxels_passed_secondary_test = voxels_passed_secondary_test;
  result.voxels_failed_secondary_test = voxels_failed_secondary_test;
  result.populated_voxels = valid_voxels.size();
  result.total_voxels = voxel_counts.size();

  publish_diagnostics(result, true);
}

rcl_interfaces::msg::SetParametersResult PolarVoxelOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "radial_resolution_m", radial_resolution_m_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new radial resolution to: %f.", radial_resolution_m_);
  }
  if (get_param(p, "azimuth_resolution_rad", azimuth_resolution_rad_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new azimuth resolution to: %f.", azimuth_resolution_rad_);
  }
  if (get_param(p, "elevation_resolution_rad", elevation_resolution_rad_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new elevation resolution to: %f.", elevation_resolution_rad_);
  }
  if (get_param(p, "voxel_points_threshold", voxel_points_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new voxel points threshold to: %d.", voxel_points_threshold_);
  }
  if (get_param(p, "min_radius_m", min_radius_m_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min radius to: %f.", min_radius_m_);
  }
  if (get_param(p, "max_radius_m", max_radius_m_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max radius to: %f.", max_radius_m_);
  }
  if (get_param(p, "use_return_type_classification", use_return_type_classification_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting use return type classification to: %s.",
      use_return_type_classification_ ? "true" : "false");
  }
  if (get_param(p, "filter_secondary_returns", filter_secondary_returns_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting filter secondary returns to: %s.",
      filter_secondary_returns_ ? "true" : "false");
  }
  if (get_param(p, "secondary_noise_threshold", secondary_noise_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting secondary noise threshold to: %d.", secondary_noise_threshold_);
  }
  if (get_param(p, "primary_return_types", primary_return_types_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new primary return types");
  }
  if (get_param(p, "filter_ratio_error_threshold", filter_ratio_error_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new filter ratio error threshold to: %f.",
      filter_ratio_error_threshold_);
  }
  if (get_param(p, "filter_ratio_warn_threshold", filter_ratio_warn_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new filter ratio warning threshold to: %f.",
      filter_ratio_warn_threshold_);
  }
  if (get_param(p, "visibility_error_threshold", visibility_error_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new visibility error threshold to: %f.", visibility_error_threshold_);
  }
  if (get_param(p, "visibility_warn_threshold", visibility_warn_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new visibility warning threshold to: %f.", visibility_warn_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void PolarVoxelOutlierFilterComponent::onVisibilityChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  // Add values
  stat.add("value", std::to_string(visibility_));

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (visibility_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (visibility_ < visibility_error_threshold_) {
    level = DiagnosticStatus::ERROR;
    msg =
      "ERROR: critically low LiDAR visibility detected while filtering outliers in polar voxel "
      "filter";
  } else if (visibility_ < visibility_warn_threshold_) {
    level = DiagnosticStatus::WARN;
    msg = "WARNING: low LiDAR visibility detected while filtering outliers in polar voxel filter";
  }
  stat.summary(level, msg);
}

void PolarVoxelOutlierFilterComponent::onFilterRatioChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  // Add values
  stat.add("value", std::to_string(filter_ratio_));

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (filter_ratio_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (filter_ratio_ < filter_ratio_error_threshold_) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR: critically low filter ratio in polar voxel outlier filter";
  } else if (filter_ratio_ < filter_ratio_warn_threshold_) {
    level = DiagnosticStatus::WARN;
    msg = "WARNING: low filter ratio in polar voxel outlier filter";
  }
  stat.summary(level, msg);
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent)
