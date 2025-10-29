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

static constexpr double diagnostics_update_period_sec = 0.1;
static constexpr size_t point_cloud_height_organized = 1;
static constexpr double TWO_PI = 2.0 * M_PI;
static constexpr int marker_resolution = 50;

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

static inline bool within_circular_range(
  const double & value_lower, const double & value_upper, const double & user_min,
  const double & user_max, const double & domain_min, const double & domain_max)
{
  double user_min_mod = std::fmod(user_min, TWO_PI);
  double user_max_mod = std::fmod(user_max, TWO_PI);
  bool within_range = false;
  if (user_min_mod < user_max_mod) {
    // Normal case
    within_range = user_min_mod <= value_lower && value_upper <= user_max_mod;
  } else {
    // Circular case
    within_range = (user_min_mod <= value_lower && value_upper <= domain_max) ||
                   (domain_min <= value_lower && value_upper <= user_max_mod);
  }
  return within_range;
}

PolarVoxelOutlierFilterComponent::PolarVoxelOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PolarVoxelOutlierFilter", options),
  azimuth_domain_min(0.0),
  azimuth_domain_max(TWO_PI),
  elevation_domain_min(-M_PI / 2.0),
  elevation_domain_max(M_PI / 2.0),
  updater_(this)
{
  radial_resolution_m_ = declare_parameter<double>("radial_resolution_m");
  azimuth_resolution_rad_ =
    adjust_resolution_to_circle(declare_parameter<double>("azimuth_resolution_rad"));
  elevation_resolution_rad_ =
    adjust_resolution_to_circle(declare_parameter<double>("elevation_resolution_rad"));
  voxel_points_threshold_ = static_cast<int>(declare_parameter<int64_t>("voxel_points_threshold"));
  min_radius_m_ = declare_parameter<double>("min_radius_m");
  max_radius_m_ = declare_parameter<double>("max_radius_m");
  visibility_estimation_max_range_m_ =
    declare_parameter<double>("visibility_estimation_max_range_m");
  visibility_estimation_min_azimuth_rad_ =
    declare_parameter<double>("visibility_estimation_min_azimuth_rad");
  visibility_estimation_max_azimuth_rad_ =
    declare_parameter<double>("visibility_estimation_max_azimuth_rad");
  visibility_estimation_min_elevation_rad_ =
    declare_parameter<double>("visibility_estimation_min_elevation_rad");
  visibility_estimation_max_elevation_rad_ =
    declare_parameter<double>("visibility_estimation_max_elevation_rad");
  use_return_type_classification_ = declare_parameter<bool>("use_return_type_classification");
  enable_secondary_return_filtering_ = declare_parameter<bool>("filter_secondary_returns");
  secondary_noise_threshold_ =
    static_cast<int>(declare_parameter<int64_t>("secondary_noise_threshold"));
  visibility_estimation_max_secondary_voxel_count_ =
    static_cast<int>(declare_parameter<int64_t>("visibility_estimation_max_secondary_voxel_count"));
  visibility_estimation_only_ = declare_parameter<bool>("visibility_estimation_only");
  publish_noise_cloud_ = declare_parameter<bool>("publish_noise_cloud");
  bool publish_area_marker = declare_parameter<bool>("publish_area_marker");
  int num_frames_hysteresis_transition = declare_parameter<int>("num_frames_hysteresis_transition");
  bool immediate_report_error = declare_parameter<bool>("immediate_report_error");
  bool immediate_relax_state = declare_parameter<bool>("immediate_relax_state");

  auto primary_return_types_param = declare_parameter<std::vector<int64_t>>("primary_return_types");
  primary_return_types_.clear();
  primary_return_types_.reserve(primary_return_types_param.size());
  for (const auto & val : primary_return_types_param) {
    primary_return_types_.push_back(static_cast<int>(val));
    RCLCPP_DEBUG(get_logger(), "primary_return_types_ value: %d", static_cast<int>(val));
  }

  visibility_error_threshold_ = declare_parameter<double>("visibility_error_threshold");
  visibility_warn_threshold_ = declare_parameter<double>("visibility_warn_threshold");
  filter_ratio_error_threshold_ = declare_parameter<double>("filter_ratio_error_threshold");
  filter_ratio_warn_threshold_ = declare_parameter<double>("filter_ratio_warn_threshold");
  intensity_threshold_ = declare_parameter<uint8_t>("intensity_threshold");

  // Initialize diagnostics
  hysteresis_state_machine_ = std::make_shared<custom_diagnostic_tasks::HysteresisStateMachine>(
    num_frames_hysteresis_transition, immediate_report_error, immediate_relax_state);
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

  // Create area marker publisher if enabled
  if (publish_area_marker) {
    area_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "polar_voxel_outlier_filter/debug/visibility_estimation_area", 10);
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return param_callback(p); });

  RCLCPP_INFO(
    get_logger(),
    "Polar Voxel Outlier Filter initialized - supports PointXYZIRC and PointXYZIRCAEDT with %s "
    "filtering%s",
    use_return_type_classification_ ? "advanced two-criteria" : "simple occupancy",
    visibility_estimation_only_ ? " (visibility estimation only - no point cloud output)" : "");
}

void PolarVoxelOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  if (!input) {
    RCLCPP_ERROR(get_logger(), "Input point cloud is null");
    throw std::invalid_argument("Input point cloud is null");
  }

  // Phase 1: Validate inputs
  validate_filter_inputs(*input, indices);

  // Check if we have pre-computed polar coordinates
  bool has_polar_coords = has_polar_coordinates(*input);

  if (has_polar_coords) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRCAEDT format with pre-computed polar coordinates");
  } else {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRC format, computing azimuth and elevation");
  }

  // Phase 2: Collect voxel information (unified for both formats)
  auto point_voxel_info = collect_voxel_info(*input);

  // Phase 3: Count points and validate voxels (mode-dependent logic)
  auto voxel_point_counts = count_voxel_points(point_voxel_info);
  auto valid_voxels = determine_valid_voxels(voxel_point_counts);

  // Phase 4: Create valid points mask
  auto valid_points_mask = create_valid_points_mask(point_voxel_info, valid_voxels);

  // Phase 5: Create output (normal or empty based on mode)
  create_output(*input, valid_points_mask, output);

  // Phase 6: Conditionally publish noise cloud
  if (publish_noise_cloud_) {
    publish_noise_cloud(*input, valid_points_mask);
  }

  // Phase 7: Publish diagnostics (always run for visibility estimation)
  publish_diagnostics(voxel_point_counts, valid_points_mask);

  // (optional) Phase 8: Publish marker to visualize area to be used for visibility estimation
  if (area_marker_pub_) {
    publish_area_marker(input->header);
  }
}

PolarVoxelOutlierFilterComponent::PointVoxelInfoVector
PolarVoxelOutlierFilterComponent::collect_voxel_info(const PointCloud2 & input)
{
  PointVoxelInfoVector point_voxel_info;
  point_voxel_info.reserve(input.width * input.height);

  bool has_polar_coords = has_polar_coordinates(input);

  if (has_polar_coords) {
    process_polar_points(input, point_voxel_info);
  } else {
    process_cartesian_points(input, point_voxel_info);
  }

  return point_voxel_info;
}

void PolarVoxelOutlierFilterComponent::process_polar_points(
  const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info)
{
  // Create iterators for polar coordinates (always exist for polar format)
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(input, "distance");
  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(input, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_elevation(input, "elevation");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(input, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(input, "return_type");

  for (; iter_distance != iter_distance.end();
       ++iter_distance, ++iter_azimuth, ++iter_elevation, ++iter_intensity, ++iter_return_type) {
    point_voxel_info.emplace_back(process_polar_point(
      *iter_distance, *iter_azimuth, *iter_elevation, *iter_intensity, *iter_return_type));
  }
}

void PolarVoxelOutlierFilterComponent::process_cartesian_points(
  const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info)
{
  // Create iterators for cartesian coordinates (always exist for cartesian format)
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(input, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(input, "return_type");

  for (; iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_return_type) {
    point_voxel_info.emplace_back(
      process_cartesian_point(*iter_x, *iter_y, *iter_z, *iter_intensity, *iter_return_type));
  }
}

std::optional<PointVoxelInfo> PolarVoxelOutlierFilterComponent::process_polar_point(
  float distance, float azimuth, float elevation, uint8_t intensity, uint8_t return_type) const
{
  // Step 1: Extract polar coordinates and determine point classification
  auto polar_opt = extract_polar_from_dae(distance, azimuth, elevation);
  bool is_primary = is_point_primary(return_type);
  bool passes_intensity = meets_intensity_threshold(intensity);

  // Step 2: Early return on invalid coordinates
  if (!polar_opt.has_value()) {
    return std::nullopt;
  }

  // Step 3: Create voxel index and determine point classification
  PolarVoxelIndex voxel_idx = polar_to_polar_voxel(*polar_opt);

  return PointVoxelInfo{voxel_idx, is_primary, passes_intensity};
}

std::optional<PointVoxelInfo> PolarVoxelOutlierFilterComponent::process_cartesian_point(
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
PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_generic(
  const VoxelPointCountMap & voxel_point_counts, Predicate predicate) const
{
  VoxelIndexSet valid_voxels;
  for (const auto & [voxel_idx, counts] : voxel_point_counts) {
    if (predicate(counts)) {
      valid_voxels.insert(voxel_idx);
    }
  }
  return valid_voxels;
}

bool PolarVoxelOutlierFilterComponent::is_point_primary(uint8_t return_type) const
{
  if (!use_return_type_classification_) {
    return true;  // Treat all as primary in simple mode
  }
  auto it = std::find(
    primary_return_types_.begin(), primary_return_types_.end(), static_cast<int>(return_type));
  return it != primary_return_types_.end();
}

bool PolarVoxelOutlierFilterComponent::meets_intensity_threshold(uint8_t intensity) const
{
  return intensity <= intensity_threshold_;
}

PolarVoxelOutlierFilterComponent::ValidPointsMask
PolarVoxelOutlierFilterComponent::create_valid_points_mask(
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

bool PolarVoxelOutlierFilterComponent::is_point_valid_for_mask(
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

bool PolarVoxelOutlierFilterComponent::passes_secondary_return_filter(bool is_primary) const
{
  if (!enable_secondary_return_filtering_) {
    return true;  // All points pass when filtering is disabled
  }

  return is_primary;  // Only primary returns pass when filtering is enabled
}

void PolarVoxelOutlierFilterComponent::create_filtered_output(
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

void PolarVoxelOutlierFilterComponent::publish_noise_cloud(
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

void PolarVoxelOutlierFilterComponent::publish_diagnostics(
  const VoxelPointCountMap & voxel_point_counts, const ValidPointsMask & valid_points_mask)
{
  // Calculate metrics
  calculate_visibility_metric(voxel_point_counts);
  calculate_filter_ratio_metric(valid_points_mask);

  // Publish metrics
  publish_visibility_metric();
  publish_filter_ratio_metric();

  // Update diagnostics
  updater_.force_update();
}

void PolarVoxelOutlierFilterComponent::calculate_visibility_metric(
  const VoxelPointCountMap & voxel_point_counts)
{
  if (!use_return_type_classification_) {
    visibility_.reset();
    return;
  }

  uint32_t low_visibility_voxels_count = 0;
  for (const auto & [voxel_idx, counts] : voxel_point_counts) {
    if (
      counts.is_in_visibility_range &&
      !counts.meets_secondary_threshold(secondary_noise_threshold_)) {
      low_visibility_voxels_count++;
    }
  }

  // Calculate visibility based on the proportion of maximum allowable voxels that fail the
  // secondary threshold test
  visibility_ = std::max(
    0.0, 1.0 - static_cast<double>(low_visibility_voxels_count) /
                 static_cast<double>(visibility_estimation_max_secondary_voxel_count_));
}

void PolarVoxelOutlierFilterComponent::calculate_filter_ratio_metric(
  const ValidPointsMask & valid_points_mask)
{
  filter_ratio_ =
    (!valid_points_mask.empty())
      ? static_cast<double>(std::count(valid_points_mask.begin(), valid_points_mask.end(), true)) /
          static_cast<double>(valid_points_mask.size())
      : 0.0;
}

void PolarVoxelOutlierFilterComponent::publish_visibility_metric()
{
  if (!visibility_pub_ || !visibility_.has_value()) {
    return;
  }

  autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
  visibility_msg.stamp = this->now();
  visibility_msg.data = static_cast<float>(visibility_.value());
  visibility_pub_->publish(visibility_msg);
}

void PolarVoxelOutlierFilterComponent::publish_filter_ratio_metric()
{
  if (!ratio_pub_) {
    return;
  }

  autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
  ratio_msg.stamp = this->now();
  ratio_msg.data = static_cast<float>(filter_ratio_.value_or(0.0));
  ratio_pub_->publish(ratio_msg);
}

bool PolarVoxelOutlierFilterComponent::has_polar_coordinates(const PointCloud2 & input)
{
  return autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(
    input);
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

bool PolarVoxelOutlierFilterComponent::is_valid_polar_point(const PolarCoordinate & polar) const
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

bool PolarVoxelOutlierFilterComponent::has_finite_coordinates(const PolarCoordinate & polar) const
{
  if (!std::isfinite(polar.radius)) return false;
  if (!std::isfinite(polar.azimuth)) return false;
  if (!std::isfinite(polar.elevation)) return false;
  return true;
}

bool PolarVoxelOutlierFilterComponent::is_within_radius_range(const PolarCoordinate & polar) const
{
  return polar.radius >= min_radius_m_ && polar.radius <= max_radius_m_;
}

bool PolarVoxelOutlierFilterComponent::has_sufficient_radius(const PolarCoordinate & polar) const
{
  return std::abs(polar.radius) >= std::numeric_limits<double>::epsilon();
}

PolarVoxelOutlierFilterComponent::VoxelPointCountMap
PolarVoxelOutlierFilterComponent::count_voxel_points(
  const PointVoxelInfoVector & point_voxel_info) const
{
  VoxelPointCountMap voxel_point_counts;
  for (const auto & info_opt : point_voxel_info) {
    if (info_opt.has_value()) {
      const auto & info = info_opt.value();
      if (info.is_primary) {
        voxel_point_counts[info.voxel_idx].primary_count++;
      } else if (info.meets_intensity_threshold) {
        voxel_point_counts[info.voxel_idx].secondary_count++;
      }
    }
  }

  auto within_azimuth_range = [this](
                                const double & val_lower, const double & val_upper,
                                const double & user_min, const double & user_max) {
    return within_circular_range(
      val_lower, val_upper, user_min, user_max, azimuth_domain_min, azimuth_domain_max);
  };

  auto within_elevation_range = [this](
                                  const double & val_lower, const double & val_upper,
                                  const double & user_min, const double & user_max) {
    return within_circular_range(
      val_lower, val_upper, user_min, user_max, elevation_domain_min, elevation_domain_max);
  };

  // Add range information for visibility calculation
  for (const auto & [voxel_idx, counts] : voxel_point_counts) {
    // Calculate the maximum radius for this voxel
    double voxel_max_radius = (voxel_idx.radius_idx + 1) * radial_resolution_m_;
    double voxel_min_azimuth = (voxel_idx.azimuth_idx) * azimuth_resolution_rad_;
    double voxel_max_azimuth = (voxel_idx.azimuth_idx + 1) * azimuth_resolution_rad_;
    double voxel_min_elevation = (voxel_idx.elevation_idx) * elevation_resolution_rad_;
    double voxel_max_elevation = (voxel_idx.elevation_idx + 1) * elevation_resolution_rad_;
    voxel_point_counts[voxel_idx].is_in_visibility_range =
      voxel_max_radius <= visibility_estimation_max_range_m_ &&
      within_azimuth_range(
        voxel_min_azimuth, voxel_max_azimuth, visibility_estimation_min_azimuth_rad_,
        visibility_estimation_max_azimuth_rad_) &&
      within_elevation_range(
        voxel_min_elevation, voxel_max_elevation, visibility_estimation_min_elevation_rad_,
        visibility_estimation_max_elevation_rad_);
  }
  return voxel_point_counts;
}

void PolarVoxelOutlierFilterComponent::update_parameter(const rclcpp::Parameter & param)
{
  using ParameterUpdater = std::function<void(const rclcpp::Parameter &)>;

  // Static map of parameter names to their update functions
  static const std::unordered_map<std::string, ParameterUpdater> parameter_updaters = {
    {"radial_resolution_m", [this](const auto & p) { radial_resolution_m_ = p.as_double(); }},
    {"azimuth_resolution_rad",
     [this](const auto & p) {
       azimuth_resolution_rad_ = adjust_resolution_to_circle(p.as_double());
     }},
    {"elevation_resolution_rad",
     [this](const auto & p) {
       elevation_resolution_rad_ = adjust_resolution_to_circle(p.as_double());
     }},
    {"voxel_points_threshold",
     [this](const auto & p) { voxel_points_threshold_ = static_cast<int>(p.as_int()); }},
    {"secondary_noise_threshold",
     [this](const auto & p) { secondary_noise_threshold_ = static_cast<int>(p.as_int()); }},
    {"intensity_threshold",
     [this](const auto & p) { intensity_threshold_ = static_cast<int>(p.as_int()); }},
    {"visibility_estimation_max_secondary_voxel_count",
     [this](const auto & p) {
       visibility_estimation_max_secondary_voxel_count_ = static_cast<int>(p.as_int());
     }},
    {"visibility_estimation_only",
     [this](const auto & p) { visibility_estimation_only_ = p.as_bool(); }},
    {"min_radius_m", [this](const auto & p) { min_radius_m_ = p.as_double(); }},
    {"max_radius_m", [this](const auto & p) { max_radius_m_ = p.as_double(); }},
    {"visibility_estimation_max_range_m",
     [this](const auto & p) { visibility_estimation_max_range_m_ = p.as_double(); }},
    {"visibility_estimation_min_azimuth_rad",
     [this](const auto & p) { visibility_estimation_min_azimuth_rad_ = p.as_double(); }},
    {"visibility_estimation_max_azimuth_rad",
     [this](const auto & p) { visibility_estimation_max_azimuth_rad_ = p.as_double(); }},
    {"visibility_estimation_min_elevation_rad",
     [this](const auto & p) { visibility_estimation_min_elevation_rad_ = p.as_double(); }},
    {"visibility_estimation_max_elevation_rad",
     [this](const auto & p) { visibility_estimation_max_elevation_rad_ = p.as_double(); }},
    {"visibility_error_threshold",
     [this](const auto & p) { visibility_error_threshold_ = p.as_double(); }},
    {"visibility_warn_threshold",
     [this](const auto & p) { visibility_warn_threshold_ = p.as_double(); }},
    {"filter_ratio_error_threshold",
     [this](const auto & p) { filter_ratio_error_threshold_ = p.as_double(); }},
    {"filter_ratio_warn_threshold",
     [this](const auto & p) { filter_ratio_warn_threshold_ = p.as_double(); }},
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

void PolarVoxelOutlierFilterComponent::update_primary_return_types(const rclcpp::Parameter & param)
{
  auto values = param.as_integer_array();
  primary_return_types_.clear();
  primary_return_types_.reserve(values.size());
  for (const auto & val : values) {
    primary_return_types_.push_back(static_cast<int>(val));
  }
}

void PolarVoxelOutlierFilterComponent::update_publish_noise_cloud(const rclcpp::Parameter & param)
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
      "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
  }
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels(
  const VoxelPointCountMap & voxel_point_counts) const
{
  if (use_return_type_classification_) {
    return determine_valid_voxels_with_return_types(voxel_point_counts);
  } else {
    return determine_valid_voxels_simple(voxel_point_counts);
  }
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_simple(
  const VoxelPointCountMap & voxel_point_counts) const
{
  return determine_valid_voxels_generic(
    voxel_point_counts, [this](const VoxelPointCounts & counts) {
      size_t total = counts.primary_count + counts.secondary_count;
      return total >= static_cast<size_t>(voxel_points_threshold_);
    });
}

PolarVoxelOutlierFilterComponent::VoxelIndexSet
PolarVoxelOutlierFilterComponent::determine_valid_voxels_with_return_types(
  const VoxelPointCountMap & voxel_point_counts) const
{
  return determine_valid_voxels_generic(
    voxel_point_counts, [this](const VoxelPointCounts & counts) {
      return counts.meets_primary_threshold(voxel_points_threshold_) &&
             counts.meets_secondary_threshold(secondary_noise_threshold_);
    });
}

void PolarVoxelOutlierFilterComponent::setup_output_header(
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

bool PolarVoxelOutlierFilterComponent::validate_positive_double(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_double() <= 0.0) {
    reason = param.get_name() + " must be positive";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_non_negative_double(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_double() < 0.0) {
    reason = param.get_name() + " must be non-negative";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_positive_int(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_int() < 1) {
    reason = param.get_name() + " must be at least 1";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_non_negative_int(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_int() < 0) {
    reason = param.get_name() + " must be non-negative";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_intensity_threshold(
  const rclcpp::Parameter & param, std::string & reason)
{
  int val = param.as_int();
  if (val < 0 || val > 255) {
    reason = "intensity_threshold must be between 0 and 255";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_primary_return_types(
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

bool PolarVoxelOutlierFilterComponent::validate_normalized(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < 0.0 || val > 1.0) {
    reason = param.get_name() + " must be between 0.0 and 1.0";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_zero_to_two_pi(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < 0.0 || val > TWO_PI) {
    reason = param.get_name() + " must be between 0.0 and 2*PI";
    return false;
  }
  return true;
}

bool PolarVoxelOutlierFilterComponent::validate_negative_half_pi_to_half_pi(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < -M_PI / 2.0 || val > M_PI / 2.0) {
    reason = param.get_name() + " must be between -PI and PI";
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult PolarVoxelOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & params)
{
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
    {"radial_resolution_m",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { radial_resolution_m_ = p.as_double(); }}},
    {"azimuth_resolution_rad",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { azimuth_resolution_rad_ = p.as_double(); }}},
    {"elevation_resolution_rad",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { elevation_resolution_rad_ = p.as_double(); }}},
    {"voxel_points_threshold",
     {validate_positive_int,
      [this](const rclcpp::Parameter & p) { voxel_points_threshold_ = p.as_int(); }}},
    {"min_radius_m",
     {validate_non_negative_double,
      [this](const rclcpp::Parameter & p) { min_radius_m_ = p.as_double(); }}},
    {"max_radius_m",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { max_radius_m_ = p.as_double(); }}},
    {"intensity_threshold",
     {validate_intensity_threshold,
      [this](const rclcpp::Parameter & p) { intensity_threshold_ = p.as_int(); }}},
    {"visibility_estimation_max_range_m",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { visibility_estimation_max_range_m_ = p.as_double(); }}},
    {"visibility_estimation_min_azimuth_rad",
     {validate_zero_to_two_pi,
      [this](const rclcpp::Parameter & p) {
        visibility_estimation_min_azimuth_rad_ = p.as_double();
      }}},
    {"visibility_estimation_max_azimuth_rad",
     {validate_zero_to_two_pi,
      [this](const rclcpp::Parameter & p) {
        visibility_estimation_max_azimuth_rad_ = p.as_double();
      }}},
    {"visibility_estimation_min_elevation_rad",
     {validate_negative_half_pi_to_half_pi,
      [this](const rclcpp::Parameter & p) {
        visibility_estimation_min_elevation_rad_ = p.as_double();
      }}},
    {"visibility_estimation_max_elevation_rad",
     {validate_negative_half_pi_to_half_pi,
      [this](const rclcpp::Parameter & p) {
        visibility_estimation_max_elevation_rad_ = p.as_double();
      }}},
    {"use_return_type_classification",
     {nullptr,
      [this](const rclcpp::Parameter & p) { use_return_type_classification_ = p.as_bool(); }}},
    {"filter_secondary_returns",
     {nullptr,
      [this](const rclcpp::Parameter & p) { enable_secondary_return_filtering_ = p.as_bool(); }}},
    {"secondary_noise_threshold",
     {validate_non_negative_int,
      [this](const rclcpp::Parameter & p) { secondary_noise_threshold_ = p.as_int(); }}},
    {"visibility_estimation_max_secondary_voxel_count",
     {validate_non_negative_int,
      [this](const rclcpp::Parameter & p) {
        visibility_estimation_max_secondary_voxel_count_ = p.as_int();
      }}},
    {"primary_return_types",
     {validate_primary_return_types,
      [this](const rclcpp::Parameter & p) {
        const auto & arr = p.as_integer_array();
        primary_return_types_.clear();
        primary_return_types_.reserve(arr.size());
        for (auto v : arr) primary_return_types_.push_back(static_cast<int>(v));
      }}},
    {"visibility_estimation_only",
     {nullptr, [this](const rclcpp::Parameter & p) { visibility_estimation_only_ = p.as_bool(); }}},
    {"publish_noise_cloud",
     {nullptr, [this](const rclcpp::Parameter & p) { publish_noise_cloud_ = p.as_bool(); }}},
    {"filter_ratio_error_threshold",
     {validate_normalized,
      [this](const rclcpp::Parameter & p) { filter_ratio_error_threshold_ = p.as_double(); }}},
    {"filter_ratio_warn_threshold",
     {validate_normalized,
      [this](const rclcpp::Parameter & p) { filter_ratio_warn_threshold_ = p.as_double(); }}},
    {"visibility_error_threshold",
     {validate_normalized,
      [this](const rclcpp::Parameter & p) { visibility_error_threshold_ = p.as_double(); }}},
    {"visibility_warn_threshold", {validate_normalized, [this](const rclcpp::Parameter & p) {
                                     visibility_warn_threshold_ = p.as_double();
                                   }}}};

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

void PolarVoxelOutlierFilterComponent::on_visibility_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!visibility_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No visibility data available");
    return;
  }

  double visibility_value = visibility_.value();

  if (visibility_value < visibility_error_threshold_) {
    hysteresis_state_machine_->update_state(diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  } else if (visibility_value < visibility_warn_threshold_) {
    hysteresis_state_machine_->update_state(diagnostic_msgs::msg::DiagnosticStatus::WARN);
  } else {
    hysteresis_state_machine_->update_state(diagnostic_msgs::msg::DiagnosticStatus::OK);
  }

  auto visibility_state = hysteresis_state_machine_->get_current_state_level();
  std::unordered_map<custom_diagnostic_tasks::DiagnosticStatus_t, std::string> message_str{
    {diagnostic_msgs::msg::DiagnosticStatus::ERROR,
     "Low visibility detected - potential adverse weather conditions"},
    {diagnostic_msgs::msg::DiagnosticStatus::WARN,
     "Reduced visibility detected - monitor environmental conditions"},
    {diagnostic_msgs::msg::DiagnosticStatus::OK, "Visibility within normal range"},
    {diagnostic_msgs::msg::DiagnosticStatus::STALE, "No visibility data available"}};
  stat.summary(visibility_state, message_str[visibility_state]);

  stat.add("Visibility", visibility_value);
  stat.add("Error Threshold", visibility_error_threshold_);
  stat.add("Warning Threshold", visibility_warn_threshold_);
  stat.add("Estimation Range (m)", visibility_estimation_max_range_m_);
  stat.add("Estimation Minimum Azimuth (rad)", visibility_estimation_min_azimuth_rad_);
  stat.add("Estimation Maximum Azimuth (rad)", visibility_estimation_max_azimuth_rad_);
  stat.add("Estimation Minimum Elevation (rad)", visibility_estimation_min_elevation_rad_);
  stat.add("Estimation Maximum Elevation (rad)", visibility_estimation_max_elevation_rad_);
  stat.add("Max Secondary Voxels", visibility_estimation_max_secondary_voxel_count_);
  stat.add("Effective state", custom_diagnostic_tasks::get_level_string(visibility_state));
  stat.add(
    "Candidate state",
    custom_diagnostic_tasks::get_level_string(hysteresis_state_machine_->get_candidate_level()));
  stat.add(
    "Candidate state observed frames", hysteresis_state_machine_->get_candidate_num_observation());
  stat.add(
    "Observed frames transition threshold", hysteresis_state_machine_->get_num_frame_transition());
  stat.add(
    "Immediate error report",
    hysteresis_state_machine_->get_immediate_error_report_param() ? "true" : "false");
  stat.add(
    "Immediate relax state",
    hysteresis_state_machine_->get_immediate_relax_state_param() ? "true" : "false");
}

void PolarVoxelOutlierFilterComponent::on_filter_ratio_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!filter_ratio_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No filter ratio data available");
    return;
  }

  double ratio_value = filter_ratio_.value();

  if (ratio_value < filter_ratio_error_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Very low filter ratio - excessive noise or sensor malfunction");
  } else if (ratio_value < filter_ratio_warn_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Low filter ratio - increased noise levels detected");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Filter ratio within normal range");
  }

  stat.add("Filter Ratio", ratio_value);
  stat.add("Error Threshold", filter_ratio_error_threshold_);
  stat.add("Warning Threshold", filter_ratio_warn_threshold_);
  stat.add("Filtering Mode", use_return_type_classification_ ? "Advanced" : "Simple");
  stat.add("Visibility Only", visibility_estimation_only_ ? "Yes" : "No");
}

void PolarVoxelOutlierFilterComponent::validate_filter_inputs(
  const PointCloud2 & input, const IndicesPtr & indices)
{
  validate_indices(indices);
  validate_required_fields(input);
}

void PolarVoxelOutlierFilterComponent::validate_indices(const IndicesPtr & indices)
{
  if (indices) {
    RCLCPP_WARN_ONCE(get_logger(), "Indices are not supported and will be ignored");
  }
}

void PolarVoxelOutlierFilterComponent::validate_required_fields(const PointCloud2 & input)
{
  validate_return_type_field(input);
  validate_intensity_field(input);
}

void PolarVoxelOutlierFilterComponent::validate_return_type_field(const PointCloud2 & input)
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

void PolarVoxelOutlierFilterComponent::validate_intensity_field(const PointCloud2 & input)
{
  if (!has_field(input, "intensity")) {
    RCLCPP_ERROR(get_logger(), "Input point cloud must have 'intensity' field");
    throw std::invalid_argument("Input point cloud must have intensity field");
  }
}

bool PolarVoxelOutlierFilterComponent::has_field(
  const PointCloud2 & input, const std::string & field_name)
{
  for (const auto & field : input.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

void PolarVoxelOutlierFilterComponent::create_output(
  const PointCloud2 & input, const ValidPointsMask & valid_points_mask, PointCloud2 & output)
{
  if (visibility_estimation_only_) {
    create_empty_output(input, output);
  } else {
    create_filtered_output(input, valid_points_mask, output);
  }
}

void PolarVoxelOutlierFilterComponent::create_empty_output(
  const PointCloud2 & input, PointCloud2 & output)
{
  setup_output_header(output, input, 0);
}

std::optional<PolarVoxelOutlierFilterComponent::PolarCoordinate>
PolarVoxelOutlierFilterComponent::extract_polar_from_dae(
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

std::optional<PolarVoxelOutlierFilterComponent::PolarCoordinate>
PolarVoxelOutlierFilterComponent::extract_polar_from_xyz(float x, float y, float z) const
{
  CartesianCoordinate cartesian(x, y, z);
  PolarCoordinate polar = cartesian_to_polar(cartesian);

  if (!is_valid_polar_point(polar)) {
    return std::nullopt;
  }

  return polar;
}

void PolarVoxelOutlierFilterComponent::publish_area_marker(
  const std_msgs::msg::Header & input_header)
{
  auto marker = visualization_msgs::msg::Marker();
  marker.header = input_header;
  marker.ns = "visibility_estimation_area";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;  // transparent green
  marker.color.b = 0.0;
  marker.pose.orientation.w = 1.0;

  auto polar_to_xyz = [](auto radius, auto azimuth, auto elevation) {
    // NOTE: This conversion assumes the following angular definitions
    // some LiDAR may not fit this definition:
    // - azimuth: starts from the y-axis, increasing in counter-corkscrew rule around the z-axis
    //   domain: [0, 2pi]
    // - elevation: starts from the x-axis, increasing in counter-corkscrew rule around the y-axis
    //   domain: [-pi/2, pi/2]
    geometry_msgs::msg::Point p;
    p.x = radius * std::cos(elevation) * std::sin(azimuth);
    p.y = radius * std::cos(elevation) * std::cos(azimuth);
    p.z = radius * std::sin(elevation);
    return p;
  };

  // Break azimuth and elevation into discrete steps to approximate the volume
  double azimuth_range_width =
    visibility_estimation_max_azimuth_rad_ - visibility_estimation_min_azimuth_rad_;
  while (azimuth_range_width < 0) {
    azimuth_range_width += TWO_PI;
  }

  double elevation_range_width =
    visibility_estimation_max_elevation_rad_ - visibility_estimation_min_elevation_rad_;
  while (elevation_range_width < 0) {
    elevation_range_width += TWO_PI;
  }

  double azimuth_portion = azimuth_range_width / marker_resolution;
  double elevation_portion = elevation_range_width / marker_resolution;

  for (int azimuth_idx = 0; azimuth_idx < marker_resolution; azimuth_idx++) {
    for (int elevation_idx = 0; elevation_idx < marker_resolution; elevation_idx++) {
      double az1 = visibility_estimation_min_azimuth_rad_ + azimuth_idx * azimuth_portion;
      double az2 = visibility_estimation_min_azimuth_rad_ + (azimuth_idx + 1) * azimuth_portion;
      double el1 = visibility_estimation_min_elevation_rad_ + elevation_idx * elevation_portion;
      double el2 =
        visibility_estimation_min_elevation_rad_ + (elevation_idx + 1) * elevation_portion;

      // if visibility_estimation_max_elevation_rad_ < visibility_estimation_min_elevation_rad_,
      // el1 and el2 can take [pi/2, 3pi/2], which is out of value domain.
      // Skip creating surface if  either el1 or el2 is in that case
      if (
        (M_PI / 2.0 < el1 && el1 < 3 * M_PI / 2.0) || (M_PI / 2.0 < el2 && el2 < 3 * M_PI / 2.0)) {
        continue;
      }

      double r1 = 0;
      double r2 = visibility_estimation_max_range_m_;

      // points for two radii slices and two elevation slices
      auto p1 = polar_to_xyz(r1, az1, el1);  // near, left, lower
      auto p2 = polar_to_xyz(r2, az1, el1);  // far, left, lower

      auto p3 = polar_to_xyz(r1, az2, el1);  // near, right, lower
      auto p4 = polar_to_xyz(r2, az2, el1);  // far, right, lower

      auto p5 = polar_to_xyz(r1, az1, el2);  // near, left, upper
      auto p6 = polar_to_xyz(r2, az1, el2);  // far, left, upper

      auto p7 = polar_to_xyz(r1, az2, el2);  // near, right, upper
      auto p8 = polar_to_xyz(r2, az2, el2);  // far, right, upper

      // Bottom surface triangles
      marker.points.push_back(p1);
      marker.points.push_back(p3);
      marker.points.push_back(p2);

      marker.points.push_back(p2);
      marker.points.push_back(p3);
      marker.points.push_back(p4);

      // Top surface triangles
      marker.points.push_back(p5);
      marker.points.push_back(p6);
      marker.points.push_back(p7);

      marker.points.push_back(p6);
      marker.points.push_back(p8);
      marker.points.push_back(p7);

      // Vertical sides between min and max radius
      marker.points.push_back(p1);
      marker.points.push_back(p2);
      marker.points.push_back(p5);

      marker.points.push_back(p2);
      marker.points.push_back(p6);
      marker.points.push_back(p5);

      marker.points.push_back(p3);
      marker.points.push_back(p4);
      marker.points.push_back(p7);

      marker.points.push_back(p4);
      marker.points.push_back(p8);
      marker.points.push_back(p7);
    }
  }

  area_marker_pub_->publish(marker);
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent)
