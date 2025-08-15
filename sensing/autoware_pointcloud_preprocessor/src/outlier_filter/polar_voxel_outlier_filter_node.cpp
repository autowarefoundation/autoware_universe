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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

static constexpr double diagnostics_update_period_sec = 0.1;
static constexpr size_t point_cloud_height_organized = 1;

PolarVoxelOutlierFilterComponent::PolarVoxelOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PolarVoxelOutlierFilter", options), updater_(this)
{
  radial_resolution_m_ = declare_parameter<double>("radial_resolution_m");
  azimuth_resolution_rad_ = declare_parameter<double>("azimuth_resolution_rad");
  elevation_resolution_rad_ = declare_parameter<double>("elevation_resolution_rad");
  voxel_points_threshold_ = static_cast<int>(declare_parameter<int64_t>("voxel_points_threshold"));
  min_radius_m_ = declare_parameter<double>("min_radius_m");
  max_radius_m_ = declare_parameter<double>("max_radius_m");
  visibility_estimation_max_range_m_ =
    declare_parameter<double>("visibility_estimation_max_range_m", 20.0);
  use_return_type_classification_ = declare_parameter<bool>("use_return_type_classification", true);
  enable_secondary_return_filtering_ = declare_parameter<bool>("filter_secondary_returns");
  secondary_noise_threshold_ =
    static_cast<int>(declare_parameter<int64_t>("secondary_noise_threshold"));
  publish_noise_cloud_ = declare_parameter<bool>("publish_noise_cloud", false);

  auto primary_return_types_param = declare_parameter<std::vector<int64_t>>("primary_return_types");
  primary_return_types_.clear();
  primary_return_types_.reserve(primary_return_types_param.size());
  for (const auto & val : primary_return_types_param) {
    primary_return_types_.push_back(static_cast<int>(val));
    RCLCPP_DEBUG(get_logger(), "primary_return_types_ value: %d", static_cast<int>(val));
  }

  visibility_error_threshold_ = declare_parameter<double>("visibility_error_threshold", 0.5);
  visibility_warn_threshold_ = declare_parameter<double>("visibility_warn_threshold", 0.7);
  filter_ratio_error_threshold_ = declare_parameter<double>("filter_ratio_error_threshold", 0.5);
  filter_ratio_warn_threshold_ = declare_parameter<double>("filter_ratio_warn_threshold", 0.7);

  // Initialize diagnostics
  updater_.setHardwareID("polar_voxel_outlier_filter");
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &PolarVoxelOutlierFilterComponent::on_visibility_check);
  updater_.add(
    std::string(this->get_namespace()) + ": filter_ratio_validation", this,
    &PolarVoxelOutlierFilterComponent::on_filter_ratio_check);
  updater_.setPeriod(diagnostics_update_period_sec);

  // Create publishers
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "polar_voxel_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());
  ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "polar_voxel_outlier_filter/debug/filter_ratio", rclcpp::SensorDataQoS());

  // Create noise cloud publisher if enabled
  if (publish_noise_cloud_) {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
    RCLCPP_INFO(get_logger(), "Noise cloud publishing enabled");
  } else {
    RCLCPP_INFO(get_logger(), "Noise cloud publishing disabled for performance optimization");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return param_callback(p); });

  RCLCPP_INFO(
    get_logger(),
    "Polar Voxel Outlier Filter initialized - supports PointXYZIRC and PointXYZIRCAEDT with %s "
    "filtering",
    use_return_type_classification_ ? "advanced two-criteria" : "simple occupancy");
}

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

  if (
    use_return_type_classification_ &&
    !PolarVoxelOutlierFilterComponent::has_return_type_field(input)) {
    RCLCPP_ERROR(
      get_logger(),
      "Advanced mode (use_return_type_classification=true) requires 'return_type' field. "
      "Set use_return_type_classification=false for simple mode or ensure input has return_type "
      "field.");
    return;
  }

  // Check if we have pre-computed polar coordinates
  bool has_polar_coords = has_polar_coordinates(input);

  if (has_polar_coords) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRCAEDT format with pre-computed polar coordinates");
  } else {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRC format, computing azimuth and elevation");
  }

  // Phase 1: Collect voxel information (unified for both formats)
  auto point_voxel_info = collect_voxel_info(input);

  // Phase 2: Count and validate voxels (mode-dependent logic)
  auto voxel_counts = count_voxels(point_voxel_info);
  auto valid_voxels = determine_valid_voxels(voxel_counts);

  // Phase 3: Create valid points mask
  auto valid_points_mask = create_valid_points_mask(point_voxel_info, valid_voxels);

  // Phase 4: Create filtered output
  create_filtered_output(input, valid_points_mask, output);

  // Phase 5: Conditionally publish noise cloud
  if (publish_noise_cloud_ && noise_cloud_pub_) {
    publish_noise_cloud(input, valid_points_mask);
  }

  // Phase 6: Publish diagnostics
  publish_diagnostics(voxel_counts, valid_points_mask);
}

PolarVoxelOutlierFilterComponent::PointVoxelInfoVector
PolarVoxelOutlierFilterComponent::collect_voxel_info(const PointCloud2ConstPtr & input)
{
  PointVoxelInfoVector point_voxel_info;
  point_voxel_info.reserve(input->width * input->height);

  bool has_polar_coords = has_polar_coordinates(input);
  bool has_return_type = PolarVoxelOutlierFilterComponent::has_return_type_field(input);

  // Create iterators based on point cloud format
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_x;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_y;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_z;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_distance;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_azimuth;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_elevation;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_return_type;

  if (has_polar_coords) {
    iter_distance =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "distance");
    iter_azimuth =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "azimuth");
    iter_elevation =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "elevation");
  } else {
    iter_x = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "x");
    iter_y = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "y");
    iter_z = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "z");
  }

  if (has_return_type) {
    iter_return_type =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*input, "return_type");
  }

  size_t point_count = input->width * input->height;

  for (size_t i = 0; i < point_count; ++i) {
    point_voxel_info.emplace_back(process_point_for_voxel_info(
      has_polar_coords, has_return_type, iter_x.get(), iter_y.get(), iter_z.get(),
      iter_distance.get(), iter_azimuth.get(), iter_elevation.get(), iter_return_type.get()));
  }

  return point_voxel_info;
}

// Helper function to process a single point for voxel info
auto PolarVoxelOutlierFilterComponent::process_point_for_voxel_info(
  bool has_polar_coords, bool has_return_type,
  sensor_msgs::PointCloud2ConstIterator<float> * iter_x,
  sensor_msgs::PointCloud2ConstIterator<float> * iter_y,
  sensor_msgs::PointCloud2ConstIterator<float> * iter_z,
  sensor_msgs::PointCloud2ConstIterator<float> * iter_distance,
  sensor_msgs::PointCloud2ConstIterator<float> * iter_azimuth,
  sensor_msgs::PointCloud2ConstIterator<float> * iter_elevation,
  sensor_msgs::PointCloud2ConstIterator<uint8_t> * iter_return_type) const
  -> std::optional<PointVoxelInfo>
{
  bool valid_point = false;
  uint8_t current_return_type = 0;

  if (has_return_type && iter_return_type) {
    current_return_type = **iter_return_type;
    ++(*iter_return_type);
  }

  std::optional<PolarCoordinate> polar_opt;

  if (has_polar_coords) {
    if (
      std::isfinite(**iter_distance) && std::isfinite(**iter_azimuth) &&
      std::isfinite(**iter_elevation)) {
      PolarCoordinate polar(**iter_distance, **iter_azimuth, **iter_elevation);
      if (validate_point_polar(polar)) {
        polar_opt = polar;
        valid_point = true;
      }
    }
    ++(*iter_distance);
    ++(*iter_azimuth);
    ++(*iter_elevation);
  } else {
    if (std::isfinite(**iter_x) && std::isfinite(**iter_y) && std::isfinite(**iter_z)) {
      CartesianCoordinate cartesian(**iter_x, **iter_y, **iter_z);
      PolarCoordinate polar = cartesian_to_polar(cartesian);
      if (validate_point_polar(polar)) {
        polar_opt = polar;
        valid_point = true;
      }
    }
    ++(*iter_x);
    ++(*iter_y);
    ++(*iter_z);
  }

  if (!valid_point || !polar_opt.has_value()) {
    return std::nullopt;
  }

  PolarVoxelIndex voxel_idx = polar_to_polar_voxel(*polar_opt);
  bool is_primary = (has_return_type && use_return_type_classification_)
                      ? is_primary_return_type(current_return_type)
                      : true;

  return std::optional<PointVoxelInfo>{PointVoxelInfo{voxel_idx, is_primary}};
}

PolarVoxelOutlierFilterComponent::ValidPointsMask
PolarVoxelOutlierFilterComponent::create_valid_points_mask(
  const PointVoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels) const
{
  ValidPointsMask valid_points_mask(point_voxel_info.size(), false);

  for (size_t i = 0; i < point_voxel_info.size(); ++i) {
    if (const auto & optional_info = point_voxel_info[i]) {
      const auto & info = *optional_info;
      if (valid_voxels.count(info.voxel_idx) > 0) {
        if (info.is_primary || !enable_secondary_return_filtering_) {
          valid_points_mask[i] = true;
        }
      }
    }
  }
  return valid_points_mask;
}

void PolarVoxelOutlierFilterComponent::create_filtered_output(
  const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask,
  PointCloud2 & output)
{
  setup_output_header(
    output, input, std::count(valid_points_mask.begin(), valid_points_mask.end(), true));

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

void PolarVoxelOutlierFilterComponent::publish_noise_cloud(
  const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask) const
{
  if (!publish_noise_cloud_ || !noise_cloud_pub_) {
    return;
  }

  auto noise_cloud = create_noise_cloud(
    input, std::count(valid_points_mask.begin(), valid_points_mask.end(), false));

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

void PolarVoxelOutlierFilterComponent::publish_diagnostics(
  const VoxelPointCountMap & voxel_counts, const ValidPointsMask & valid_points_mask)
{
  if (use_return_type_classification_) {
    // Calculate visibility based on valid points mask
    uint32_t high_visibility_voxels = 0;
    uint32_t low_visibility_voxels = 0;
    for (const auto & [voxel_idx, counts] : voxel_counts) {
      if (counts.is_in_visibility_range) {
        if (
          counts.meets_primary_threshold(voxel_points_threshold_) &&
          counts.meets_secondary_threshold(secondary_noise_threshold_)) {
          high_visibility_voxels++;
        } else {
          low_visibility_voxels++;
        }
      }
    }
    visibility_ = ((high_visibility_voxels + low_visibility_voxels) > 0)
                    ? static_cast<double>(high_visibility_voxels) /
                        (high_visibility_voxels + low_visibility_voxels)
                    : 0.0;
  }
  filter_ratio_ =
    (!valid_points_mask.empty())
      ? static_cast<double>(std::count(valid_points_mask.begin(), valid_points_mask.end(), true)) /
          static_cast<double>(valid_points_mask.size())
      : 0.0;

  // Publish filter ratio
  if (ratio_pub_) {
    autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
    ratio_msg.stamp = this->now();
    ratio_msg.data = static_cast<float>(filter_ratio_.value_or(0.0));
    ratio_pub_->publish(ratio_msg);
  }

  // Publish visibility (only when available)
  if (visibility_pub_ && visibility_.has_value()) {
    autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.stamp = this->now();
    visibility_msg.data = static_cast<float>(visibility_.value());
    visibility_pub_->publish(visibility_msg);
  }

  // Update diagnostics
  updater_.force_update();
}

bool PolarVoxelOutlierFilterComponent::has_return_type_field(const PointCloud2ConstPtr & input)
{
  for (const auto & field : input->fields) {
    if (field.name == "return_type") {
      return true;
    }
  }
  return false;
}

bool PolarVoxelOutlierFilterComponent::has_polar_coordinates(const PointCloud2ConstPtr & input)
{
  return autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(
    *input);
}

PolarVoxelOutlierFilterComponent::PolarCoordinate
PolarVoxelOutlierFilterComponent::cartesian_to_polar(const CartesianCoordinate & cartesian)
{
  double radius =
    std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z);
  double azimuth = std::atan2(cartesian.y, cartesian.x);
  double elevation =
    std::atan2(cartesian.z, std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y));
  return {radius, azimuth, elevation};
}

PolarVoxelIndex PolarVoxelOutlierFilterComponent::polar_to_polar_voxel(
  const PolarCoordinate & polar) const
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

bool PolarVoxelOutlierFilterComponent::validate_point_polar(const PolarCoordinate & polar) const
{
  if (
    !std::isfinite(polar.radius) || !std::isfinite(polar.azimuth) ||
    !std::isfinite(polar.elevation)) {
    return false;
  }
  if (polar.radius < min_radius_m_ || polar.radius > max_radius_m_) {
    return false;
  }
  if (std::abs(polar.radius) < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  return true;
}

PolarVoxelOutlierFilterComponent::VoxelPointCountMap PolarVoxelOutlierFilterComponent::count_voxels(
  const PointVoxelInfoVector & point_voxel_info) const
{
  VoxelPointCountMap voxel_counts;
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
  // Add range information for visibility calculation
  for (const auto & [voxel_idx, counts] : voxel_counts) {
    // Calculate the maximum radius for this voxel
    double voxel_max_radius = (voxel_idx.radius_idx + 1) * radial_resolution_m_;
    voxel_counts[voxel_idx].is_in_visibility_range =
      voxel_max_radius <= visibility_estimation_max_range_m_;
  }
  return voxel_counts;
}

namespace
{
bool is_in_vector(const std::vector<std::string> & vec, const std::string & value)
{
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}
}  // namespace

void PolarVoxelOutlierFilterComponent::update_parameter(const rclcpp::Parameter & param)
{
  const auto & name = param.get_name();

  if (name == "radial_resolution_m") {
    radial_resolution_m_ = param.as_double();
  } else if (name == "azimuth_resolution_rad") {
    azimuth_resolution_rad_ = param.as_double();
  } else if (name == "elevation_resolution_rad") {
    elevation_resolution_rad_ = param.as_double();
  } else if (name == "voxel_points_threshold") {
    voxel_points_threshold_ = static_cast<int>(param.as_int());
  } else if (name == "secondary_noise_threshold") {
    secondary_noise_threshold_ = static_cast<int>(param.as_int());
  } else if (name == "min_radius_m") {
    min_radius_m_ = param.as_double();
  } else if (name == "max_radius_m") {
    max_radius_m_ = param.as_double();
  } else if (name == "visibility_estimation_max_range_m") {
    visibility_estimation_max_range_m_ = param.as_double();
  } else if (name == "visibility_error_threshold") {
    visibility_error_threshold_ = param.as_double();
  } else if (name == "visibility_warn_threshold") {
    visibility_warn_threshold_ = param.as_double();
  } else if (name == "filter_ratio_error_threshold") {
    filter_ratio_error_threshold_ = param.as_double();
  } else if (name == "filter_ratio_warn_threshold") {
    filter_ratio_warn_threshold_ = param.as_double();
  } else if (name == "use_return_type_classification") {
    use_return_type_classification_ = param.as_bool();
  } else if (name == "filter_secondary_returns") {
    enable_secondary_return_filtering_ = param.as_bool();
  } else if (name == "primary_return_types") {
    auto values = param.as_integer_array();
    primary_return_types_.clear();
    for (const auto & val : values) {
      primary_return_types_.push_back(static_cast<int>(val));
    }
  } else if (name == "publish_noise_cloud") {
    bool new_value = param.as_bool();
    if (new_value != publish_noise_cloud_) {
      publish_noise_cloud_ = new_value;
      // Recreate publisher if needed
      if (publish_noise_cloud_ && !noise_cloud_pub_) {
        rclcpp::PublisherOptions pub_options;
        pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
        noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(),
          pub_options);
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult PolarVoxelOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  // Parameters that can be updated at runtime
  static const std::vector<std::string> updatable_params = {
    "radial_resolution_m",
    "azimuth_resolution_rad",
    "elevation_resolution_rad",
    "voxel_points_threshold",
    "min_radius_m",
    "max_radius_m",
    "visibility_estimation_max_range_m",
    "use_return_type_classification",
    "filter_secondary_returns",
    "secondary_noise_threshold",
    "primary_return_types",
    "publish_noise_cloud",
    "visibility_error_threshold",
    "visibility_warn_threshold",
    "filter_ratio_error_threshold",
    "filter_ratio_warn_threshold"};

  auto result = rcl_interfaces::msg::SetParametersResult{};
  result.successful = true;

  for (const auto & param : p) {
    if (!is_in_vector(updatable_params, param.get_name())) {
      result.successful = false;
      result.reason = "Parameter " + param.get_name() + " is not updatable";
      return result;
    }

    try {
      update_parameter(param);

      RCLCPP_DEBUG(get_logger(), "Updated parameter: %s", param.get_name().c_str());
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = "Failed to update parameter " + param.get_name() + ": " + e.what();
      return result;
    }
  }

  return result;
}

void PolarVoxelOutlierFilterComponent::setup_output_header(
  PointCloud2 & output, const PointCloud2ConstPtr & input, size_t valid_count)
{
  output.header = input->header;
  output.height = point_cloud_height_organized;
  output.width = static_cast<uint32_t>(valid_count);
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.row_step = output.width * output.point_step;
  output.is_dense = input->is_dense;
  output.data.resize(output.row_step * output.height);
}

sensor_msgs::msg::PointCloud2 PolarVoxelOutlierFilterComponent::create_noise_cloud(
  const PointCloud2ConstPtr & input, size_t noise_count)
{
  sensor_msgs::msg::PointCloud2 noise_cloud;
  noise_cloud.header = input->header;
  noise_cloud.height = point_cloud_height_organized;
  noise_cloud.width = static_cast<uint32_t>(noise_count);
  noise_cloud.fields = input->fields;
  noise_cloud.is_bigendian = input->is_bigendian;
  noise_cloud.point_step = input->point_step;
  noise_cloud.row_step = noise_cloud.width * noise_cloud.point_step;
  noise_cloud.is_dense = input->is_dense;
  noise_cloud.data.resize(noise_cloud.row_step * noise_cloud.height);
  return noise_cloud;
}

void PolarVoxelOutlierFilterComponent::on_visibility_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!visibility_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Visibility check disabled");
    stat.add("visibility", "N/A (return type classification disabled)");
    return;
  }

  double visibility_value = visibility_.value();
  stat.add("visibility", visibility_value);
  stat.add("visibility_estimation_max_range_m", visibility_estimation_max_range_m_);
  stat.add("visibility_error_threshold", visibility_error_threshold_);
  stat.add("visibility_warn_threshold", visibility_warn_threshold_);

  if (visibility_value < visibility_error_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Visibility is critically low within " + std::to_string(visibility_estimation_max_range_m_) +
        "m range");
  } else if (visibility_value < visibility_warn_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Visibility is low within " + std::to_string(visibility_estimation_max_range_m_) + "m range");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Visibility is normal within " + std::to_string(visibility_estimation_max_range_m_) +
        "m range");
  }
}

void PolarVoxelOutlierFilterComponent::on_filter_ratio_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!filter_ratio_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No filter ratio data");
    stat.add("filter_ratio", "N/A");
    return;
  }

  double ratio_value = filter_ratio_.value();
  stat.add("filter_ratio", ratio_value);
  stat.add("filter_ratio_error_threshold", filter_ratio_error_threshold_);
  stat.add("filter_ratio_warn_threshold", filter_ratio_warn_threshold_);

  if (ratio_value < filter_ratio_error_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Filter ratio is critically low - too many points filtered");
  } else if (ratio_value < filter_ratio_warn_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Filter ratio is low - many points filtered");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Filter ratio is normal");
  }
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_simple(
  const VoxelPointCountMap & voxel_counts) const
{
  VoxelIndexSet valid_voxels;

  for (const auto & [voxel_idx, counts] : voxel_counts) {
    size_t total_points = counts.primary_count + counts.secondary_count;
    if (total_points >= static_cast<size_t>(voxel_points_threshold_)) {
      valid_voxels.insert(voxel_idx);
    }
  }

  return valid_voxels;
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_with_return_types(
  const VoxelPointCountMap & voxel_counts) const
{
  VoxelIndexSet valid_voxels;

  // Process all voxels for filtering decision
  for (const auto & [voxel_idx, counts] : voxel_counts) {
    bool primary_meets_threshold = counts.meets_primary_threshold(voxel_points_threshold_);
    bool secondary_meets_threshold = counts.meets_secondary_threshold(secondary_noise_threshold_);

    // Two-criteria filtering: Both criteria must be satisfied
    if (primary_meets_threshold && secondary_meets_threshold) {
      valid_voxels.insert(voxel_idx);
    }
  }

  return valid_voxels;
}

// Main delegation method for voxel validation
PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels(
  const VoxelPointCountMap & voxel_counts) const
{
  if (use_return_type_classification_) {
    return determine_valid_voxels_with_return_types(voxel_counts);
  }
  return determine_valid_voxels_simple(voxel_counts);
}

// Cartesian to polar voxel conversion (if needed)
PolarVoxelIndex PolarVoxelOutlierFilterComponent::cartesian_to_polar_voxel(
  const CartesianCoordinate & cartesian) const
{
  PolarCoordinate polar = cartesian_to_polar(cartesian);
  return polar_to_polar_voxel(polar);
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent)
