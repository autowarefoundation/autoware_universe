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
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

static constexpr double DIAGNOSTICS_UPDATE_PERIOD_SEC = 0.1;
static constexpr size_t POINT_CLOUD_HEIGHT_ORGANIZED = 1;

// Common voxel information structure
struct VoxelInfo
{
  PolarVoxelOutlierFilterComponent::PolarVoxelIndex voxel_idx;
  bool is_primary = true;  // Default for XYZ format

  explicit VoxelInfo(
    const PolarVoxelOutlierFilterComponent::PolarVoxelIndex & idx, bool primary = true)
  : voxel_idx(idx), is_primary(primary)
  {
  }
};

// Voxel counting structure
struct VoxelCounts
{
  size_t primary_count = 0;
  size_t secondary_count = 0;

  bool meets_primary_threshold(int threshold) const
  {
    return primary_count >= static_cast<size_t>(threshold);
  }

  bool meets_secondary_threshold(int threshold) const
  {
    return secondary_count <= static_cast<size_t>(threshold);
  }
};

// Complete structure for point processing results
struct PointProcessingResult
{
  size_t input_points;
  size_t output_points;
  size_t voxels_passed_secondary_test = 0;
  size_t populated_voxels = 0;
  size_t total_voxels = 0;
};

// Point count calculations structure
struct PointCounts
{
  size_t valid_count;
  size_t noise_count;
  size_t total_count;
};

PolarVoxelOutlierFilterComponent::PolarVoxelOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PolarVoxelOutlierFilter", options), updater_(this)
{
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

    // Noise cloud publishing control (default: false for performance)
    publish_noise_cloud_ = declare_parameter<bool>("publish_noise_cloud", false);

    auto primary_return_types_param =
      declare_parameter<std::vector<int64_t>>("primary_return_types");
    primary_return_types_.clear();
    primary_return_types_.reserve(primary_return_types_param.size());
    for (const auto & val : primary_return_types_param) {
      primary_return_types_.push_back(static_cast<int>(val));
    }

    visibility_error_threshold_ = declare_parameter<double>("visibility_error_threshold", 0.5);
    visibility_warn_threshold_ = declare_parameter<double>("visibility_warn_threshold", 0.7);
    filter_ratio_error_threshold_ = declare_parameter<double>("filter_ratio_error_threshold", 0.5);
    filter_ratio_warn_threshold_ = declare_parameter<double>("filter_ratio_warn_threshold", 0.7);
  }

  // Initialize diagnostics
  updater_.setHardwareID("polar_voxel_outlier_filter");
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &PolarVoxelOutlierFilterComponent::on_visibility_check);
  updater_.add(
    std::string(this->get_namespace()) + ": filter_ratio_validation", this,
    &PolarVoxelOutlierFilterComponent::on_filter_ratio_check);
  updater_.setPeriod(DIAGNOSTICS_UPDATE_PERIOD_SEC);

  // Create visibility publisher
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "polar_voxel_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());

  // Create ratio publisher
  ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "polar_voxel_outlier_filter/debug/filter_ratio", rclcpp::SensorDataQoS());

  // Only create noise cloud publisher if enabled
  if (publish_noise_cloud_) {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
    RCLCPP_INFO(get_logger(), "Noise cloud publishing enabled");
  } else {
    noise_cloud_pub_ = nullptr;
    RCLCPP_INFO(get_logger(), "Noise cloud publishing disabled for performance optimization");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return param_callback(p); });
}

PolarVoxelOutlierFilterComponent::PolarCoordinate
PolarVoxelOutlierFilterComponent::cartesian_to_polar(const CartesianCoordinate & cartesian)
{
  double radius =
    std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z);
  double azimuth = std::atan2(cartesian.y, cartesian.x);
  double elevation =
    std::atan2(cartesian.z, std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y));

  return PolarCoordinate(radius, azimuth, elevation);
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex
PolarVoxelOutlierFilterComponent::cartesian_to_polar_voxel(
  const CartesianCoordinate & cartesian) const
{
  PolarCoordinate polar = cartesian_to_polar(cartesian);
  return polar_to_polar_voxel(polar);
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex
PolarVoxelOutlierFilterComponent::polar_to_polar_voxel(const PolarCoordinate & polar) const
{
  PolarVoxelIndex voxel_idx{};
  voxel_idx.radius_idx = static_cast<int32_t>(std::floor(polar.radius / radial_resolution_m_));
  voxel_idx.azimuth_idx = static_cast<int32_t>(std::floor(polar.azimuth / azimuth_resolution_rad_));
  voxel_idx.elevation_idx =
    static_cast<int32_t>(std::floor(polar.elevation / elevation_resolution_rad_));
  return voxel_idx;
}

bool PolarVoxelOutlierFilterComponent::is_primary_return_type(uint8_t return_type) const
{
  auto it = std::find(
    primary_return_types_.begin(), primary_return_types_.end(), static_cast<int>(return_type));
  return it != primary_return_types_.end();
}

// Helper function for point validation
bool PolarVoxelOutlierFilterComponent::validate_point_basic(const PolarCoordinate & polar) const
{
  // Skip points with NaN or Inf values
  if (
    !std::isfinite(polar.radius) || !std::isfinite(polar.azimuth) ||
    !std::isfinite(polar.elevation)) {
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

// Helper method implementations
PointProcessingResult PolarVoxelOutlierFilterComponent::create_processing_result(
  const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask,
  const VoxelCountMap & voxel_counts, const VoxelIndexSet & valid_voxels,
  size_t voxels_passed_secondary_test) const
{
  PointProcessingResult result;
  result.input_points = input->width * input->height;
  result.output_points = std::count(valid_points_mask.begin(), valid_points_mask.end(), true);
  result.total_voxels = voxel_counts.size();
  result.populated_voxels = valid_voxels.size();
  result.voxels_passed_secondary_test = voxels_passed_secondary_test;
  return result;
}

sensor_msgs::msg::PointCloud2 PolarVoxelOutlierFilterComponent::create_noise_cloud(
  const PointCloud2ConstPtr & input, size_t noise_count) const
{
  sensor_msgs::msg::PointCloud2 noise_cloud;
  noise_cloud.header = input->header;
  noise_cloud.fields = input->fields;
  noise_cloud.is_bigendian = input->is_bigendian;
  noise_cloud.point_step = input->point_step;
  noise_cloud.is_dense = input->is_dense;
  noise_cloud.height = POINT_CLOUD_HEIGHT_ORGANIZED;
  noise_cloud.width = noise_count;
  noise_cloud.row_step = noise_cloud.width * noise_cloud.point_step;
  noise_cloud.data.resize(noise_cloud.row_step);
  return noise_cloud;
}

PointCounts PolarVoxelOutlierFilterComponent::calculate_point_counts(
  const ValidPointsMask & valid_points_mask) const
{
  size_t valid_count = std::count(valid_points_mask.begin(), valid_points_mask.end(), true);
  size_t total_count = valid_points_mask.size();
  size_t noise_count = total_count - valid_count;
  return {valid_count, noise_count, total_count};
}

void PolarVoxelOutlierFilterComponent::setup_output_header(
  PointCloud2 & output, const PointCloud2ConstPtr & input, size_t valid_count) const
{
  output.header = input->header;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.is_dense = input->is_dense;
  output.height = POINT_CLOUD_HEIGHT_ORGANIZED;
  output.width = valid_count;
  output.row_step = output.width * output.point_step;
  output.data.resize(output.row_step);
}

// Phase 1: Data collection
PolarVoxelOutlierFilterComponent::VoxelInfoVector
PolarVoxelOutlierFilterComponent::collect_voxel_info_xyz(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input) const
{
  VoxelInfoVector point_voxel_info(pcl_input->points.size());

  for (size_t point_idx = 0; point_idx < pcl_input->points.size(); ++point_idx) {
    const auto & point = pcl_input->points[point_idx];

    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    CartesianCoordinate cartesian(
      static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z));

    PolarCoordinate polar = cartesian_to_polar(cartesian);

    if (validate_point_basic(polar)) {
      PolarVoxelIndex voxel_idx = polar_to_polar_voxel(polar);
      point_voxel_info[point_idx] = VoxelInfo{voxel_idx, true};  // All points are primary for XYZ
    }
  }

  return point_voxel_info;
}

PolarVoxelOutlierFilterComponent::VoxelInfoVector
PolarVoxelOutlierFilterComponent::collect_voxel_info_xyzircaedt(
  const PointCloud2ConstPtr & input) const
{
  VoxelInfoVector point_voxel_info(input->width * input->height);

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

  return point_voxel_info;
}

// Phase 2: Voxel validation
PolarVoxelOutlierFilterComponent::VoxelCountMap PolarVoxelOutlierFilterComponent::count_voxels(
  const VoxelInfoVector & point_voxel_info) const
{
  VoxelCountMap voxel_counts;

  for (const auto & info_opt : point_voxel_info) {
    if (info_opt.has_value()) {
      const auto & info = info_opt.value();
      if (info.is_primary) {
        voxel_counts[info.voxel_idx].primary_count++;
      } else {
        voxel_counts[info.voxel_idx].secondary_count++;
      }
    }
  }

  return voxel_counts;
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_simple(
  const VoxelCountMap & voxel_counts) const
{
  VoxelIndexSet valid_voxels;

  for (const auto & [voxel_idx, counts] : voxel_counts) {
    if (counts.meets_primary_threshold(voxel_points_threshold_)) {
      valid_voxels.insert(voxel_idx);
    }
  }

  return valid_voxels;
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_with_return_types(
  const VoxelCountMap & voxel_counts, size_t & voxels_passed_secondary_test) const
{
  VoxelIndexSet valid_voxels;
  voxels_passed_secondary_test = 0;

  for (const auto & [voxel_idx, counts] : voxel_counts) {
    bool primary_meets_threshold = counts.meets_primary_threshold(voxel_points_threshold_);
    bool secondary_meets_threshold = counts.meets_secondary_threshold(secondary_noise_threshold_);

    if (secondary_meets_threshold) {
      voxels_passed_secondary_test++;
    }

    if (primary_meets_threshold && secondary_meets_threshold) {
      valid_voxels.insert(voxel_idx);
    }
  }

  return valid_voxels;
}

// Phase 3: Point filtering
PolarVoxelOutlierFilterComponent::ValidPointsMask
PolarVoxelOutlierFilterComponent::create_valid_points_mask(
  const VoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels,
  bool filter_secondary) const
{
  ValidPointsMask valid_points_mask(point_voxel_info.size(), false);

  for (size_t i = 0; i < point_voxel_info.size(); ++i) {
    if (point_voxel_info[i].has_value()) {
      const auto & info = point_voxel_info[i].value();
      if (valid_voxels.count(info.voxel_idx) > 0) {
        if (info.is_primary || !filter_secondary) {
          valid_points_mask[i] = true;
        }
      }
    }
  }

  return valid_points_mask;
}

// Phase 4: Output creation
void PolarVoxelOutlierFilterComponent::create_filtered_output_xyz(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input, const ValidPointsMask & valid_points_mask,
  PointCloud2 & output) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);

  auto point_counts = calculate_point_counts(valid_points_mask);
  pcl_output->points.reserve(point_counts.valid_count);

  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (valid_points_mask[i]) {
      pcl_output->points.push_back(pcl_input->points[i]);
    }
  }

  pcl::toROSMsg(*pcl_output, output);
}

void PolarVoxelOutlierFilterComponent::create_filtered_output_xyzircaedt(
  const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask,
  PointCloud2 & output) const
{
  auto point_counts = calculate_point_counts(valid_points_mask);
  setup_output_header(output, input, point_counts.valid_count);

  size_t output_idx = 0;
  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (valid_points_mask[i]) {
      std::memcpy(
        &output.data[output_idx * output.point_step], &input->data[i * input->point_step],
        input->point_step);
      output_idx++;
    }
  }
}

// Phase 5: Noise cloud publishing
void PolarVoxelOutlierFilterComponent::publish_noise_cloud_xyz(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input, const ValidPointsMask & valid_points_mask,
  const VoxelInfoVector & point_voxel_info,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input) const
{
  // Skip processing if noise cloud publishing is disabled
  if (!publish_noise_cloud_ || !noise_cloud_pub_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr noise_output(new pcl::PointCloud<pcl::PointXYZ>);

  size_t noise_count = 0;
  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (!valid_points_mask[i] && point_voxel_info[i].has_value()) {
      noise_count++;
    }
  }
  noise_output->points.reserve(noise_count);

  // Copy noise points (only processed points that were filtered out)
  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (!valid_points_mask[i] && point_voxel_info[i].has_value()) {
      noise_output->points.push_back(pcl_input->points[i]);
    }
  }

  // Publish noise points
  sensor_msgs::msg::PointCloud2 noise_output_msg;
  pcl::toROSMsg(*noise_output, noise_output_msg);
  noise_output_msg.header = input->header;
  noise_cloud_pub_->publish(noise_output_msg);
}

void PolarVoxelOutlierFilterComponent::publish_noise_cloud_xyzircaedt(
  const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask) const
{
  // Skip processing if noise cloud publishing is disabled
  if (!publish_noise_cloud_ || !noise_cloud_pub_) {
    return;
  }

  auto point_counts = calculate_point_counts(valid_points_mask);
  auto noise_cloud = create_noise_cloud(input, point_counts.noise_count);

  size_t noise_idx = 0;
  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (!valid_points_mask[i]) {
      std::memcpy(
        &noise_cloud.data[noise_idx * noise_cloud.point_step], &input->data[i * input->point_step],
        input->point_step);
      noise_idx++;
    }
  }

  noise_cloud_pub_->publish(noise_cloud);
}

// Add publish_noise_cloud parameter handling
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

  // Handle noise cloud publishing parameter with dynamic publisher management
  if (get_param(p, "publish_noise_cloud", publish_noise_cloud_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting publish noise cloud to: %s.", publish_noise_cloud_ ? "true" : "false");

    // Dynamic publisher management: Create/destroy publisher based on parameter
    if (publish_noise_cloud_ && !noise_cloud_pub_) {
      rclcpp::PublisherOptions pub_options;
      pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
      noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
      RCLCPP_INFO(get_logger(), "Noise cloud publisher created dynamically");
    } else if (!publish_noise_cloud_ && noise_cloud_pub_) {
      noise_cloud_pub_.reset();
      RCLCPP_INFO(get_logger(), "Noise cloud publisher destroyed for performance optimization");
    }
  }

  // Handle primary_return_types parameter conversion
  if (auto param_it = std::find_if(
        p.begin(), p.end(),
        [](const rclcpp::Parameter & param) { return param.get_name() == "primary_return_types"; });
      param_it != p.end()) {
    auto primary_return_types_param = param_it->as_integer_array();
    primary_return_types_.clear();
    primary_return_types_.reserve(primary_return_types_param.size());
    for (const auto & val : primary_return_types_param) {
      primary_return_types_.push_back(static_cast<int>(val));
    }
    RCLCPP_DEBUG(get_logger(), "Setting new primary return types");
  }

  // Handle diagnostic threshold parameters
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

// Main filter entry point
void PolarVoxelOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  if (!input) {
    RCLCPP_ERROR(get_logger(), "Input point cloud is null");
    return;
  }

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  if (autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(
        *input)) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Using PointXYZIRCAEDT format with pre-computed polar coordinates");
    filter_point_xyzircaedt(input, indices, output);
  } else {
    RCLCPP_DEBUG_ONCE(get_logger(), "Using PointXYZ format, computing polar coordinates");
    filter_point_xyz(input, indices, output);
  }
}

// Main filtering functions
void PolarVoxelOutlierFilterComponent::filter_point_xyz(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);

  // Phase 1: Collect voxel information
  auto point_voxel_info = collect_voxel_info_xyz(pcl_input);

  // Phase 2: Count and validate voxels
  auto voxel_counts = count_voxels(point_voxel_info);
  auto valid_voxels = determine_valid_voxels_simple(voxel_counts);

  // Phase 3: Create valid points mask
  auto valid_points_mask = create_valid_points_mask(point_voxel_info, valid_voxels, false);

  // Phase 4: Create filtered output
  create_filtered_output_xyz(pcl_input, valid_points_mask, output);
  output.header = input->header;

  // Phase 5: Conditionally publish noise cloud
  if (publish_noise_cloud_ && noise_cloud_pub_) {
    publish_noise_cloud_xyz(pcl_input, valid_points_mask, point_voxel_info, input);
  }

  // Phase 6: Publish diagnostics using ProcessingContext
  ProcessingContext context{input, valid_points_mask, voxel_counts, valid_voxels};
  publish_diagnostics_simple(context);
}

void PolarVoxelOutlierFilterComponent::filter_point_xyzircaedt(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning

  // Phase 1: Collect voxel information
  auto point_voxel_info = collect_voxel_info_xyzircaedt(input);

  // Phase 2: Count and validate voxels with return type logic
  auto voxel_counts = count_voxels(point_voxel_info);
  size_t voxels_passed_secondary_test = 0;
  auto valid_voxels =
    determine_valid_voxels_with_return_types(voxel_counts, voxels_passed_secondary_test);

  // Phase 3: Create valid points mask
  auto valid_points_mask =
    create_valid_points_mask(point_voxel_info, valid_voxels, filter_secondary_returns_);

  // Phase 4: Create filtered output
  create_filtered_output_xyzircaedt(input, valid_points_mask, output);

  // Phase 5: Conditionally publish noise cloud
  if (publish_noise_cloud_ && noise_cloud_pub_) {
    publish_noise_cloud_xyzircaedt(input, valid_points_mask);
  }

  // Phase 6: Publish diagnostics using ProcessingContext
  ProcessingContext context{input, valid_points_mask, voxel_counts, valid_voxels};
  publish_diagnostics_with_return_types(context, voxels_passed_secondary_test);
}

// Phase 6: Diagnostic publishing methods
void PolarVoxelOutlierFilterComponent::publish_diagnostics_simple(
  const ProcessingContext & context) const
{
  auto result = create_processing_result(
    context.input, context.valid_points_mask, context.voxel_counts, context.valid_voxels, 0);
  const_cast<PolarVoxelOutlierFilterComponent *>(this)->publish_diagnostics(result, false);
}

void PolarVoxelOutlierFilterComponent::publish_diagnostics_with_return_types(
  const ProcessingContext & context, size_t voxels_passed_secondary_test) const
{
  auto result = create_processing_result(
    context.input, context.valid_points_mask, context.voxel_counts, context.valid_voxels,
    voxels_passed_secondary_test);
  const_cast<PolarVoxelOutlierFilterComponent *>(this)->publish_diagnostics(result, true);
}

void PolarVoxelOutlierFilterComponent::publish_diagnostics(
  const PointProcessingResult & result, bool has_return_type_classification)
{
  // Calculate and store filter ratio
  filter_ratio_ = result.input_points > 0 ? static_cast<double>(result.output_points) /
                                              static_cast<double>(result.input_points)
                                          : 0.0;

  if (filter_ratio_.has_value()) {
    autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
    ratio_msg.data = static_cast<float>(*filter_ratio_);
    ratio_msg.stamp = now();
    ratio_pub_->publish(ratio_msg);
  }

  // Only calculate and publish visibility when return type classification is enabled
  if (has_return_type_classification && use_return_type_classification_) {
    size_t total_voxels_with_points = result.total_voxels;

    // Calculate and store visibility
    visibility_ = total_voxels_with_points > 0
                    ? static_cast<double>(result.voxels_passed_secondary_test) /
                        static_cast<double>(total_voxels_with_points)
                    : 0.0;

    if (visibility_.has_value()) {
      autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
      visibility_msg.data = static_cast<float>(*visibility_);
      visibility_msg.stamp = now();
      visibility_pub_->publish(visibility_msg);

      RCLCPP_DEBUG(
        get_logger(), "Visibility: %.3f (%zu/%zu voxels passed secondary test, %zu failed)",
        *visibility_, result.voxels_passed_secondary_test, total_voxels_with_points,
        total_voxels_with_points - result.voxels_passed_secondary_test);
    }
  }

  if (filter_ratio_.has_value()) {
    RCLCPP_DEBUG(
      get_logger(), "Filter ratio: %.3f (%zu/%zu points), Voxel stats: %zu total, %zu populated",
      *filter_ratio_, result.output_points, result.input_points, result.total_voxels,
      result.populated_voxels);
  }
}

void PolarVoxelOutlierFilterComponent::on_visibility_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::scoped_lock lock(mutex_);

  if (!visibility_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No visibility data available yet");
    stat.add("visibility", "uninitialized");
    stat.add("error_threshold", visibility_error_threshold_);
    stat.add("warn_threshold", visibility_warn_threshold_);
    return;
  }

  const double visibility_value = *visibility_;

  if (visibility_value < visibility_error_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Visibility too low - possible sensor degradation or environmental conditions");
  } else if (visibility_value < visibility_warn_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Visibility below warning threshold - monitor sensor performance");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Visibility within normal range");
  }

  stat.add("visibility", visibility_value);
  stat.add("error_threshold", visibility_error_threshold_);
  stat.add("warn_threshold", visibility_warn_threshold_);
}

void PolarVoxelOutlierFilterComponent::on_filter_ratio_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::scoped_lock lock(mutex_);

  if (!filter_ratio_.has_value()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "No filter ratio data available yet");
    stat.add("filter_ratio", "uninitialized");
    stat.add("error_threshold", filter_ratio_error_threshold_);
    stat.add("warn_threshold", filter_ratio_warn_threshold_);
    return;
  }

  const double filter_ratio_value = *filter_ratio_;

  if (filter_ratio_value < filter_ratio_error_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Filter ratio too low - excessive point filtering detected");
  } else if (filter_ratio_value < filter_ratio_warn_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Filter ratio below warning threshold - high filtering activity");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Filter ratio within normal range");
  }

  stat.add("filter_ratio", filter_ratio_value);
  stat.add("error_threshold", filter_ratio_error_threshold_);
  stat.add("warn_threshold", filter_ratio_warn_threshold_);
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent)
