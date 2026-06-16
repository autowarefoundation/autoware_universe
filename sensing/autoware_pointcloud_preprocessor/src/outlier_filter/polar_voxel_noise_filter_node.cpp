// Copyright 2026 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_noise_filter_node.hpp"

#include <autoware/pointcloud_preprocessor/utility/memory.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

static constexpr size_t point_cloud_height_organized = 1;
static constexpr double TWO_PI = 2.0 * M_PI;

template <typename... T>
bool all_finite(T... values)
{
  return (... && std::isfinite(values));
}

inline double adjust_resolution_to_circle(double requested_resolution)
{
  int bins = static_cast<int>(std::round(TWO_PI / requested_resolution));
  if (bins < 1) bins = 1;
  return TWO_PI / bins;
}

PolarVoxelNoiseFilterComponent::PolarVoxelNoiseFilterComponent(const rclcpp::NodeOptions & options)
: Filter("PolarVoxelNoiseFilter", options),
  azimuth_domain_min(0.0),
  azimuth_domain_max(TWO_PI),
  elevation_domain_min(-M_PI / 2.0),
  elevation_domain_max(M_PI / 2.0)  //,
// updater_(this)
{
  radial_resolution_m_ = declare_parameter<double>("radial_resolution");
  azimuth_resolution_rad_ =
    adjust_resolution_to_circle(declare_parameter<double>("azimuth_resolution"));
  elevation_resolution_rad_ =
    adjust_resolution_to_circle(declare_parameter<double>("elevation_resolution"));
  voxel_points_threshold_ = static_cast<int>(declare_parameter<int64_t>("voxel_points_threshold"));
  min_radius_m_ = declare_parameter<double>("min_radius");
  max_radius_m_ = declare_parameter<double>("max_radius");
  use_return_type_classification_ = declare_parameter<bool>("use_return_type_classification");
  enable_secondary_return_filtering_ = declare_parameter<bool>("filter_secondary_returns");
  secondary_noise_threshold_ =
    static_cast<int>(declare_parameter<int64_t>("secondary_noise_threshold"));
  publish_noise_cloud_ = declare_parameter<bool>("publish_noise_cloud");

  auto primary_return_types_param = declare_parameter<std::vector<int64_t>>("primary_return_types");
  primary_return_types_.clear();
  primary_return_types_.reserve(primary_return_types_param.size());
  for (const auto & val : primary_return_types_param) {
    primary_return_types_.push_back(static_cast<int>(val));
    RCLCPP_DEBUG(get_logger(), "primary_return_types_ value: %d", static_cast<int>(val));
  }

  avg_intensity_threshold_ = declare_parameter<double>("avg_intensity_threshold");

  // Create noise cloud publisher if enabled
  if (publish_noise_cloud_) {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "polar_voxel_noise_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
    RCLCPP_INFO(get_logger(), "Noise cloud publishing enabled");
  } else {
    RCLCPP_INFO(get_logger(), "Noise cloud publishing disabled for performance optimization");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return param_callback(p); });

  RCLCPP_INFO(
    get_logger(),
    "Polar Voxel Noise Filter initialized - supports PointXYZIRC and PointXYZIRCAEDT with %s",
    use_return_type_classification_ ? "advanced two-criteria" : "simple occupancy");
}

void PolarVoxelNoiseFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  std::call_once(input_format_once_flag_, [this, &input, &indices]() {
    validate_filter_inputs(*input, indices);

    if (has_polar_coordinates(*input)) {
      input_format_ = InputPointCloudFormat::PointXYZIRCAEDT;
      RCLCPP_DEBUG(
        get_logger(), "Processing PointXYZIRCAEDT format with pre-computed polar coordinates");
      return;
    }

    input_format_ = InputPointCloudFormat::PointXYZIRC;
    RCLCPP_DEBUG(get_logger(), "Processing PointXYZIRC format, computing azimuth and elevation");
  });

  // Phase 2: Collect voxel information (unified for both formats)
  auto point_voxel_info = collect_voxel_info(*input);

  // Phase 3: Analyze per-voxel statistics and validate voxels
  auto voxel_stats_map = analyze_voxel_stats(point_voxel_info);
  auto valid_voxels = determine_valid_voxels(voxel_stats_map);

  // Phase 4: Create valid points mask
  auto valid_points_mask = create_valid_points_mask(point_voxel_info, valid_voxels);

  // Phase 5: Create output (normal or empty based on mode)
  create_output(*input, valid_points_mask, output);

  // Phase 6: Conditionally publish noise cloud
  if (publish_noise_cloud_) {
    publish_noise_cloud(*input, valid_points_mask);
  }
}

PolarVoxelNoiseFilterComponent::PointVoxelInfoVector
PolarVoxelNoiseFilterComponent::collect_voxel_info(const PointCloud2 & input)
{
  PointVoxelInfoVector point_voxel_info;
  point_voxel_info.reserve(input.width * input.height);

  const bool has_polar_coords =
    input_format_ == InputPointCloudFormat::PointXYZIRCAEDT || has_polar_coordinates(input);

  if (has_polar_coords) {
    process_polar_points(input, point_voxel_info);
  } else {
    process_cartesian_points(input, point_voxel_info);
  }

  return point_voxel_info;
}

void PolarVoxelNoiseFilterComponent::process_polar_points(
  const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info)
{
  // Create iterators for polar coordinates (always exist for polar format)
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(input, "distance");
  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(input, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_elevation(input, "elevation");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(input, "intensity");
  if (use_return_type_classification_) {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(input, "return_type");
    for (; iter_distance != iter_distance.end();
         ++iter_distance, ++iter_azimuth, ++iter_elevation, ++iter_intensity, ++iter_return_type) {
      point_voxel_info.emplace_back(process_polar_point(
        *iter_distance, *iter_azimuth, *iter_elevation, *iter_intensity, *iter_return_type));
    }
  } else {
    // Simple mode ignores return type, so this is only a placeholder value.
    for (; iter_distance != iter_distance.end();
         ++iter_distance, ++iter_azimuth, ++iter_elevation, ++iter_intensity) {
      point_voxel_info.emplace_back(process_polar_point(
        *iter_distance, *iter_azimuth, *iter_elevation, *iter_intensity,
        primary_return_types_.empty() ? 0U : static_cast<uint8_t>(primary_return_types_.front())));
    }
  }
}

void PolarVoxelNoiseFilterComponent::process_cartesian_points(
  const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info)
{
  // Create iterators for cartesian coordinates (always exist for cartesian format)
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(input, "intensity");
  if (use_return_type_classification_) {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(input, "return_type");
    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_return_type) {
      point_voxel_info.emplace_back(
        process_cartesian_point(*iter_x, *iter_y, *iter_z, *iter_intensity, *iter_return_type));
    }
  } else {
    // Simple mode ignores return type, so this is only a placeholder value.
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
      point_voxel_info.emplace_back(process_cartesian_point(
        *iter_x, *iter_y, *iter_z, *iter_intensity,
        primary_return_types_.empty() ? 0U : static_cast<uint8_t>(primary_return_types_.front())));
    }
  }
}

std::optional<PointVoxelInfo> PolarVoxelNoiseFilterComponent::process_polar_point(
  float distance, float azimuth, float elevation, uint8_t intensity, uint8_t return_type) const
{
  // Step 1: Extract polar coordinates and determine point classification
  auto polar_opt = extract_polar_from_dae(distance, azimuth, elevation);
  bool is_primary = is_point_primary(return_type);

  // Step 2: Early return on invalid coordinates
  if (!polar_opt.has_value()) {
    return std::nullopt;
  }

  // Step 3: Create voxel index and determine point classification
  PolarVoxelIndex voxel_idx = polar_to_polar_voxel(*polar_opt);

  return PointVoxelInfo{voxel_idx, is_primary, intensity};
}

std::optional<PointVoxelInfo> PolarVoxelNoiseFilterComponent::process_cartesian_point(
  float x, float y, float z, uint8_t intensity, uint8_t return_type) const
{
  auto polar_opt = extract_polar_from_xyz(x, y, z);

  if (!polar_opt.has_value()) {
    return std::nullopt;
  }

  return process_polar_point(
    polar_opt->radius, polar_opt->azimuth, polar_opt->elevation, intensity, return_type);
}

template <typename Predicate>
PolarVoxelNoiseFilterComponent::VoxelIndexSet
PolarVoxelNoiseFilterComponent::determine_valid_voxels_generic(
  const VoxelStatsMap & voxel_stats_map, Predicate predicate) const
{
  VoxelIndexSet valid_voxels;
  for (const auto & [voxel_idx, stats] : voxel_stats_map) {
    if (predicate(stats)) {
      valid_voxels.insert(voxel_idx);
    }
  }
  return valid_voxels;
}

bool PolarVoxelNoiseFilterComponent::is_point_primary(uint8_t return_type) const
{
  if (!use_return_type_classification_) {
    return true;  // Treat all as primary in simple mode
  }
  auto it = std::find(
    primary_return_types_.begin(), primary_return_types_.end(), static_cast<int>(return_type));
  return it != primary_return_types_.end();
}

PolarVoxelNoiseFilterComponent::ValidPointsMask
PolarVoxelNoiseFilterComponent::create_valid_points_mask(
  const PointVoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels) const
{
  ValidPointsMask valid_points_mask(point_voxel_info.size(), false);

  for (size_t i = 0; i < point_voxel_info.size(); ++i) {
    if (is_point_valid_for_mask(point_voxel_info[i], valid_voxels)) {
      valid_points_mask[i] = true;
    }
  }

  return valid_points_mask;
}

bool PolarVoxelNoiseFilterComponent::is_point_valid_for_mask(
  const std::optional<PointVoxelInfo> & optional_info, const VoxelIndexSet & valid_voxels) const
{
  if (!optional_info.has_value()) {
    return false;
  }

  const auto & info = *optional_info;

  if (!valid_voxels.count(info.voxel_idx)) {
    return false;
  }

  return passes_secondary_return_filter(info.is_primary);
}

bool PolarVoxelNoiseFilterComponent::passes_secondary_return_filter(bool is_primary) const
{
  if (!enable_secondary_return_filtering_) {
    return true;  // All points pass when filtering is disabled
  }

  return is_primary;  // Only primary returns pass when filtering is enabled
}

void PolarVoxelNoiseFilterComponent::create_filtered_output(
  const PointCloud2 & input, const ValidPointsMask & valid_points_mask, PointCloud2 & output)
{
  setup_output_header(
    output, input, std::count(valid_points_mask.begin(), valid_points_mask.end(), true));

  size_t output_idx = 0;
  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (valid_points_mask[i]) {
      std::memcpy(
        &output.data[output_idx * output.point_step], &input.data[i * input.point_step],
        input.point_step);
      output_idx++;
    }
  }
}

void PolarVoxelNoiseFilterComponent::publish_noise_cloud(
  const PointCloud2 & input, const ValidPointsMask & valid_points_mask) const
{
  if (!publish_noise_cloud_ || !noise_cloud_pub_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 noise_cloud;
  setup_output_header(
    noise_cloud, input, std::count(valid_points_mask.begin(), valid_points_mask.end(), false));

  size_t noise_idx = 0;
  for (size_t i = 0; i < valid_points_mask.size(); ++i) {
    if (!valid_points_mask[i]) {
      std::memcpy(
        &noise_cloud.data[noise_idx * noise_cloud.point_step], &input.data[i * input.point_step],
        input.point_step);
      noise_idx++;
    }
  }

  noise_cloud_pub_->publish(noise_cloud);
}

bool PolarVoxelNoiseFilterComponent::has_polar_coordinates(const PointCloud2 & input)
{
  return autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(
    input);
}

PolarVoxelNoiseFilterComponent::PolarCoordinate PolarVoxelNoiseFilterComponent::cartesian_to_polar(
  const CartesianCoordinate & cartesian)
{
  double radius =
    std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z);
  double azimuth = std::atan2(cartesian.y, cartesian.x);
  double elevation =
    std::atan2(cartesian.z, std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y));
  return {radius, azimuth, elevation};
}

PolarVoxelIndex PolarVoxelNoiseFilterComponent::polar_to_polar_voxel(
  const PolarCoordinate & polar) const
{
  PolarVoxelIndex voxel_idx{};
  voxel_idx.radius_idx = static_cast<int32_t>(std::floor(polar.radius / radial_resolution_m_));
  voxel_idx.azimuth_idx = static_cast<int32_t>(std::floor(polar.azimuth / azimuth_resolution_rad_));
  voxel_idx.elevation_idx =
    static_cast<int32_t>(std::floor(polar.elevation / elevation_resolution_rad_));
  return voxel_idx;
}

bool PolarVoxelNoiseFilterComponent::is_valid_polar_point(const PolarCoordinate & polar) const
{
  if (!has_finite_coordinates(polar)) {
    return false;
  }

  if (!is_within_radius_range(polar)) {
    return false;
  }

  if (!has_sufficient_radius(polar)) {
    return false;
  }

  return true;
}

bool PolarVoxelNoiseFilterComponent::has_finite_coordinates(const PolarCoordinate & polar) const
{
  if (!std::isfinite(polar.radius)) return false;
  if (!std::isfinite(polar.azimuth)) return false;
  if (!std::isfinite(polar.elevation)) return false;
  return true;
}

bool PolarVoxelNoiseFilterComponent::is_within_radius_range(const PolarCoordinate & polar) const
{
  return polar.radius >= min_radius_m_ && polar.radius <= max_radius_m_;
}

bool PolarVoxelNoiseFilterComponent::has_sufficient_radius(const PolarCoordinate & polar) const
{
  return std::abs(polar.radius) >= std::numeric_limits<double>::epsilon();
}

PolarVoxelNoiseFilterComponent::VoxelStatsMap PolarVoxelNoiseFilterComponent::analyze_voxel_stats(
  const PointVoxelInfoVector & point_voxel_info) const
{
  VoxelStatsMap voxel_stats_map;
  for (const auto & info_opt : point_voxel_info) {
    if (info_opt.has_value()) {
      const auto & info = info_opt.value();
      auto & voxel_stats = voxel_stats_map[info.voxel_idx];
      voxel_stats.point_count++;
      voxel_stats.intensity_sum += static_cast<float>(info.intensity);
      if (use_return_type_classification_ && !info.is_primary) {
        voxel_stats.secondary_return_count++;
      }
    }
  }

  // Calculate intensity average per voxel
  for (auto & [voxel_idx, voxel_stats] : voxel_stats_map) {
    (void)voxel_idx;
    if (voxel_stats.point_count > 0) {
      voxel_stats.intensity_avg = voxel_stats.intensity_sum / voxel_stats.point_count;
    }
  }
  return voxel_stats_map;
}

void PolarVoxelNoiseFilterComponent::update_parameter(const rclcpp::Parameter & param)
{
  using ParameterUpdater = std::function<void(const rclcpp::Parameter &)>;

  // Static map of parameter names to their update functions
  static const std::unordered_map<std::string, ParameterUpdater> parameter_updaters = {
    {"radial_resolution", [this](const auto & p) { radial_resolution_m_ = p.as_double(); }},
    {"azimuth_resolution",
     [this](const auto & p) {
       azimuth_resolution_rad_ = adjust_resolution_to_circle(p.as_double());
     }},
    {"elevation_resolution",
     [this](const auto & p) {
       elevation_resolution_rad_ = adjust_resolution_to_circle(p.as_double());
     }},
    {"voxel_points_threshold",
     [this](const auto & p) { voxel_points_threshold_ = static_cast<int>(p.as_int()); }},
    {"secondary_noise_threshold",
     [this](const auto & p) { secondary_noise_threshold_ = static_cast<int>(p.as_int()); }},
    {"avg_intensity_threshold",
     [this](const auto & p) { avg_intensity_threshold_ = p.as_double(); }},
    {"min_radius", [this](const auto & p) { min_radius_m_ = p.as_double(); }},
    {"max_radius", [this](const auto & p) { max_radius_m_ = p.as_double(); }},
    {"use_return_type_classification",
     [this](const auto & p) { use_return_type_classification_ = p.as_bool(); }},
    {"filter_secondary_returns",
     [this](const auto & p) { enable_secondary_return_filtering_ = p.as_bool(); }},
    {"primary_return_types", [this](const auto & p) { update_primary_return_types(p); }},
    {"publish_noise_cloud", [this](const auto & p) { update_publish_noise_cloud(p); }}};

  const auto & name = param.get_name();
  auto it = parameter_updaters.find(name);

  // Early return if parameter not found (no nested logic)
  if (it == parameter_updaters.end()) {
    return;
  }

  // Execute the parameter update (no nested logic)
  it->second(param);
}

void PolarVoxelNoiseFilterComponent::update_primary_return_types(const rclcpp::Parameter & param)
{
  auto values = param.as_integer_array();
  primary_return_types_.clear();
  primary_return_types_.reserve(values.size());
  for (const auto & val : values) {
    primary_return_types_.push_back(static_cast<int>(val));
  }
}

void PolarVoxelNoiseFilterComponent::update_publish_noise_cloud(const rclcpp::Parameter & param)
{
  bool new_value = param.as_bool();
  if (new_value == publish_noise_cloud_) {
    return;
  }

  publish_noise_cloud_ = new_value;

  // Recreate publisher if needed
  if (publish_noise_cloud_ && !noise_cloud_pub_) {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "polar_voxel_noise_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
  }
}

PolarVoxelNoiseFilterComponent::VoxelIndexSet
PolarVoxelNoiseFilterComponent::determine_valid_voxels(const VoxelStatsMap & voxel_stats_map) const
{
  if (use_return_type_classification_) {
    return determine_valid_voxels_with_return_types(voxel_stats_map);
  } else {
    return determine_valid_voxels_simple(voxel_stats_map);
  }
}

PolarVoxelNoiseFilterComponent::VoxelIndexSet
PolarVoxelNoiseFilterComponent::determine_valid_voxels_simple(
  const VoxelStatsMap & voxel_stats_map) const
{
  return determine_valid_voxels_generic(voxel_stats_map, [this](const VoxelStats & stats) {
    return !stats.meets_noise_simple_condition(voxel_points_threshold_, avg_intensity_threshold_);
  });
}

PolarVoxelNoiseFilterComponent::VoxelIndexSet
PolarVoxelNoiseFilterComponent::determine_valid_voxels_with_return_types(
  const VoxelStatsMap & voxel_stats_map) const
{
  return determine_valid_voxels_generic(voxel_stats_map, [this](const VoxelStats & stats) {
    if (stats.meets_noise_condition(
          voxel_points_threshold_, avg_intensity_threshold_, secondary_noise_threshold_)) {
      return false;
    } else {
      return true;
    }
  });
}

void PolarVoxelNoiseFilterComponent::setup_output_header(
  PointCloud2 & output, const PointCloud2 & input, size_t valid_count)
{
  output.header = input.header;
  output.height = point_cloud_height_organized;
  output.width = static_cast<uint32_t>(valid_count);
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.width * output.point_step;
  output.is_dense = input.is_dense;
  output.data.resize(output.row_step * output.height);
}

bool PolarVoxelNoiseFilterComponent::validate_positive_double(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_double() <= 0.0) {
    reason = param.get_name() + " must be positive";
    return false;
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_non_negative_double(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_double() < 0.0) {
    reason = param.get_name() + " must be non-negative";
    return false;
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_positive_int(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_int() < 1) {
    reason = param.get_name() + " must be at least 1";
    return false;
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_non_negative_int(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_int() < 0) {
    reason = param.get_name() + " must be non-negative";
    return false;
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_primary_return_types(
  const rclcpp::Parameter & param, std::string & reason)
{
  for (const auto & type : param.as_integer_array()) {
    if (type < 0 || type > 255) {
      reason = "primary_return_types values must be between 0 and 255";
      return false;
    }
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_normalized(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < 0.0 || val > 1.0) {
    reason = param.get_name() + " must be between 0.0 and 1.0";
    return false;
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_zero_to_two_pi(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < 0.0 || val > TWO_PI) {
    reason = param.get_name() + " must be between 0.0 and 2*PI";
    return false;
  }
  return true;
}

bool PolarVoxelNoiseFilterComponent::validate_negative_half_pi_to_half_pi(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < -M_PI / 2.0 || val > M_PI / 2.0) {
    reason = param.get_name() + " must be between -PI and PI";
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult PolarVoxelNoiseFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  std::scoped_lock lock(mutex_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  using Validator = std::function<bool(const rclcpp::Parameter &, std::string &)>;
  using Assigner = std::function<void(const rclcpp::Parameter &)>;
  struct ParamOps
  {
    Validator validator;
    Assigner assigner;
  };

  static const std::unordered_map<std::string, ParamOps> param_ops = {
    {"radial_resolution",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { radial_resolution_m_ = p.as_double(); }}},
    {"azimuth_resolution",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { azimuth_resolution_rad_ = p.as_double(); }}},
    {"elevation_resolution",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { elevation_resolution_rad_ = p.as_double(); }}},
    {"voxel_points_threshold",
     {validate_positive_int,
      [this](const rclcpp::Parameter & p) { voxel_points_threshold_ = p.as_int(); }}},
    {"min_radius",
     {validate_non_negative_double,
      [this](const rclcpp::Parameter & p) { min_radius_m_ = p.as_double(); }}},
    {"max_radius",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { max_radius_m_ = p.as_double(); }}},
    {"use_return_type_classification",
     {nullptr,
      [this](const rclcpp::Parameter & p) { use_return_type_classification_ = p.as_bool(); }}},
    {"filter_secondary_returns",
     {nullptr,
      [this](const rclcpp::Parameter & p) { enable_secondary_return_filtering_ = p.as_bool(); }}},
    {"secondary_noise_threshold",
     {validate_non_negative_int,
      [this](const rclcpp::Parameter & p) { secondary_noise_threshold_ = p.as_int(); }}},
    {"primary_return_types",
     {validate_primary_return_types,
      [this](const rclcpp::Parameter & p) {
        const auto & arr = p.as_integer_array();
        primary_return_types_.clear();
        primary_return_types_.reserve(arr.size());
        for (auto v : arr) primary_return_types_.push_back(static_cast<int>(v));
      }}},
    {"publish_noise_cloud",
     {nullptr, [this](const rclcpp::Parameter & p) { publish_noise_cloud_ = p.as_bool(); }}}};

  for (const auto & param : params) {
    auto it = param_ops.find(param.get_name());
    if (it != param_ops.end()) {
      if (it->second.validator) {
        std::string reason;
        if (!it->second.validator(param, reason)) {
          result.successful = false;
          result.reason = reason;
          return result;
        }
      }
      it->second.assigner(param);
    }
  }

  return result;
}

void PolarVoxelNoiseFilterComponent::validate_filter_inputs(
  const PointCloud2 & input, const IndicesPtr & indices)
{
  validate_indices(indices);
  validate_required_fields(input);
}

void PolarVoxelNoiseFilterComponent::validate_indices(const IndicesPtr & indices)
{
  if (indices) {
    RCLCPP_WARN_ONCE(get_logger(), "Indices are not supported and will be ignored");
  }
}

void PolarVoxelNoiseFilterComponent::validate_required_fields(const PointCloud2 & input)
{
  validate_return_type_field(input);
  validate_intensity_field(input);
}

void PolarVoxelNoiseFilterComponent::validate_return_type_field(const PointCloud2 & input)
{
  if (!use_return_type_classification_) {
    return;
  }

  if (!has_field(input, "return_type")) {
    RCLCPP_ERROR(
      get_logger(),
      "Advanced mode (use_return_type_classification=true) requires 'return_type' field. "
      "Set use_return_type_classification=false for simple mode or ensure input has return_type "
      "field.");
    throw std::invalid_argument("Advanced mode requires return_type field");
  }
}

void PolarVoxelNoiseFilterComponent::validate_intensity_field(const PointCloud2 & input)
{
  if (!has_field(input, "intensity")) {
    RCLCPP_ERROR(get_logger(), "Input point cloud must have 'intensity' field");
    throw std::invalid_argument("Input point cloud must have intensity field");
  }
}

bool PolarVoxelNoiseFilterComponent::has_field(
  const PointCloud2 & input, const std::string & field_name)
{
  for (const auto & field : input.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

void PolarVoxelNoiseFilterComponent::create_output(
  const PointCloud2 & input, const ValidPointsMask & valid_points_mask, PointCloud2 & output)
{
  create_filtered_output(input, valid_points_mask, output);
}

void PolarVoxelNoiseFilterComponent::create_empty_output(
  const PointCloud2 & input, PointCloud2 & output)
{
  setup_output_header(output, input, 0);
}

std::optional<PolarVoxelNoiseFilterComponent::PolarCoordinate>
PolarVoxelNoiseFilterComponent::extract_polar_from_dae(
  float distance, float azimuth, float elevation) const
{
  if (!all_finite(distance, azimuth, elevation)) {
    return std::nullopt;
  }

  PolarCoordinate polar(distance, azimuth, elevation);

  if (!is_valid_polar_point(polar)) {
    return std::nullopt;
  }

  return polar;
}

std::optional<PolarVoxelNoiseFilterComponent::PolarCoordinate>
PolarVoxelNoiseFilterComponent::extract_polar_from_xyz(float x, float y, float z) const
{
  CartesianCoordinate cartesian(x, y, z);
  PolarCoordinate polar = cartesian_to_polar(cartesian);

  if (!is_valid_polar_point(polar)) {
    return std::nullopt;
  }

  return polar;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelNoiseFilterComponent)
