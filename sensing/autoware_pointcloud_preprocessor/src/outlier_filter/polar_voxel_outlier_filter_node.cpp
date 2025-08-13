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

static constexpr double DIAGNOSTICS_UPDATE_PERIOD_SEC = 0.1;
static constexpr size_t POINT_CLOUD_HEIGHT_ORGANIZED = 1;

struct VoxelInfo
{
  PolarVoxelOutlierFilterComponent::PolarVoxelIndex voxel_idx;
  bool is_primary;

  explicit VoxelInfo(const PolarVoxelOutlierFilterComponent::PolarVoxelIndex & idx, bool primary)
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

// Point processing results
struct PointProcessingResult
{
  size_t input_points;
  size_t output_points;
  size_t voxels_passed_secondary_test = 0;
  size_t populated_voxels = 0;
  size_t total_voxels = 0;
};

// Point count calculations
struct PointCounts
{
  size_t valid_count;
  size_t noise_count;
  size_t total_count;
};

struct ProcessingContext
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input;
  std::vector<bool> valid_points_mask;
  std::unordered_map<
    PolarVoxelOutlierFilterComponent::PolarVoxelIndex, VoxelCounts,
    PolarVoxelOutlierFilterComponent::PolarVoxelIndexHash>
    voxel_counts;
  std::unordered_set<
    PolarVoxelOutlierFilterComponent::PolarVoxelIndex,
    PolarVoxelOutlierFilterComponent::PolarVoxelIndexHash>
    valid_voxels;
};

PolarVoxelOutlierFilterComponent::PolarVoxelOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PolarVoxelOutlierFilter", options), updater_(this)
{
  radial_resolution_m_ = declare_parameter<double>("radial_resolution_m");
  azimuth_resolution_rad_ = declare_parameter<double>("azimuth_resolution_rad");
  elevation_resolution_rad_ = declare_parameter<double>("elevation_resolution_rad");
  voxel_points_threshold_ = declare_parameter<int>("voxel_points_threshold");
  min_radius_m_ = declare_parameter<double>("min_radius_m");
  max_radius_m_ = declare_parameter<double>("max_radius_m");
  use_return_type_classification_ = declare_parameter<bool>("use_return_type_classification", true);
  filter_secondary_returns_ = declare_parameter<bool>("filter_secondary_returns");
  secondary_noise_threshold_ = declare_parameter<int>("secondary_noise_threshold");
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
  updater_.setPeriod(DIAGNOSTICS_UPDATE_PERIOD_SEC);

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

  if (use_return_type_classification_ && !has_return_type_field(input)) {
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
  size_t voxels_passed_secondary_test = 0;
  auto valid_voxels = determine_valid_voxels(voxel_counts, voxels_passed_secondary_test);

  // Phase 3: Create valid points mask
  auto valid_points_mask = create_valid_points_mask(point_voxel_info, valid_voxels);

  // Phase 4: Create filtered output
  create_filtered_output(input, valid_points_mask, output);

  // Phase 5: Conditionally publish noise cloud
  if (publish_noise_cloud_ && noise_cloud_pub_) {
    publish_noise_cloud(input, valid_points_mask);
  }

  // Phase 6: Publish diagnostics (mode-dependent)
  ProcessingContext context{input, valid_points_mask, voxel_counts, valid_voxels};
  publish_diagnostics(context, voxels_passed_secondary_test);
}

PolarVoxelOutlierFilterComponent::VoxelInfoVector
PolarVoxelOutlierFilterComponent::collect_voxel_info(const PointCloud2ConstPtr & input) const
{
  VoxelInfoVector point_voxel_info;
  point_voxel_info.reserve(input->width * input->height);

  bool has_polar_coords = has_polar_coordinates(input);
  bool has_return_type = has_return_type_field(input);

  // Create iterators based on point cloud format
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_x, iter_y, iter_z;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> iter_distance, iter_azimuth,
    iter_elevation;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_return_type;

  if (has_polar_coords) {
    // PointXYZIRCAEDT format - use pre-computed polar coordinates
    iter_distance =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "distance");
    iter_azimuth =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "azimuth");
    iter_elevation =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "elevation");
  } else {
    // PointXYZIRC format - need Cartesian coordinates
    iter_x = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "x");
    iter_y = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "y");
    iter_z = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*input, "z");
  }

  if (has_return_type) {
    iter_return_type =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*input, "return_type");
  }

  // Determine iteration count based on format
  size_t point_count = input->width * input->height;

  for (size_t i = 0; i < point_count; ++i) {
    bool valid_point = false;
    uint8_t current_return_type = 0;

    // Store return type before advancing iterator
    if (has_return_type) {
      current_return_type = **iter_return_type;  // Dereference the iterator properly
      ++(*iter_return_type);
    }

    std::optional<PolarCoordinate> polar_opt;

    if (has_polar_coords) {
      // Use pre-computed polar coordinates
      if (
        std::isfinite(**iter_distance) && std::isfinite(**iter_azimuth) &&
        std::isfinite(**iter_elevation)) {
        PolarCoordinate polar(**iter_distance, **iter_azimuth, **iter_elevation);
        if (validate_point_polar(polar)) {
          polar_opt = polar;
          valid_point = true;
        }
      }

      // Advance polar iterators
      ++(*iter_distance);
      ++(*iter_azimuth);
      ++(*iter_elevation);
    } else {
      // Compute polar coordinates from Cartesian
      if (std::isfinite(**iter_x) && std::isfinite(**iter_y) && std::isfinite(**iter_z)) {
        CartesianCoordinate cartesian(**iter_x, **iter_y, **iter_z);
        PolarCoordinate polar = cartesian_to_polar(cartesian);
        if (validate_point_polar(polar)) {
          polar_opt = polar;
          valid_point = true;
        }
      }

      // Advance Cartesian iterators
      ++(*iter_x);
      ++(*iter_y);
      ++(*iter_z);
    }

    // Process the point
    if (!valid_point) {
      point_voxel_info.emplace_back(std::nullopt);
      continue;
    }

    PolarVoxelIndex voxel_idx = polar_to_polar_voxel(polar_opt.value());
    bool is_primary = (has_return_type && use_return_type_classification_)
                        ? is_primary_return_type(current_return_type)
                        : true;

    point_voxel_info.emplace_back(VoxelInfo{voxel_idx, is_primary});
  }

  return point_voxel_info;
}

PolarVoxelOutlierFilterComponent::ValidPointsMask
PolarVoxelOutlierFilterComponent::create_valid_points_mask(
  const VoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels) const
{
  ValidPointsMask valid_points_mask(point_voxel_info.size(), false);

  for (size_t i = 0; i < point_voxel_info.size(); ++i) {
    if (point_voxel_info[i].has_value()) {
      const auto & info = point_voxel_info[i].value();
      if (valid_voxels.count(info.voxel_idx) > 0) {
        // Include point if it's primary OR we're not filtering secondary returns
        if (info.is_primary || !filter_secondary_returns_) {
          valid_points_mask[i] = true;
        }
      }
    }
  }

  return valid_points_mask;
}

void PolarVoxelOutlierFilterComponent::create_filtered_output(
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

void PolarVoxelOutlierFilterComponent::publish_noise_cloud(
  const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask) const
{
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

void PolarVoxelOutlierFilterComponent::publish_diagnostics(
  const ProcessingContext & context, size_t voxels_passed_secondary_test) const
{
  auto result = create_processing_result(
    context.input, context.valid_points_mask, context.voxel_counts, context.valid_voxels,
    voxels_passed_secondary_test);

  // Calculate and store filter ratio
  const_cast<PolarVoxelOutlierFilterComponent *>(this)->filter_ratio_ =
    (result.input_points > 0)
      ? static_cast<double>(result.output_points) / static_cast<double>(result.input_points)
      : 0.0;

  // Calculate and store visibility (only when using return type classification)
  if (use_return_type_classification_ && result.populated_voxels > 0) {
    const_cast<PolarVoxelOutlierFilterComponent *>(this)->visibility_ =
      static_cast<double>(result.voxels_passed_secondary_test) /
      static_cast<double>(result.populated_voxels);
  } else {
    const_cast<PolarVoxelOutlierFilterComponent *>(this)->visibility_ = std::nullopt;
  }

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
  const_cast<PolarVoxelOutlierFilterComponent *>(this)->updater_.force_update();
}

bool PolarVoxelOutlierFilterComponent::has_return_type_field(
  const PointCloud2ConstPtr & input) const
{
  for (const auto & field : input->fields) {
    if (field.name == "return_type") {
      return true;
    }
  }
  return false;
}

bool PolarVoxelOutlierFilterComponent::has_polar_coordinates(
  const PointCloud2ConstPtr & input) const
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
  return PolarCoordinate(radius, azimuth, elevation);
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

rcl_interfaces::msg::SetParametersResult PolarVoxelOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  // Parameters that can be updated at runtime
  std::vector<std::string> updatable_params = {
    "radial_resolution_m",
    "azimuth_resolution_rad",
    "elevation_resolution_rad",
    "voxel_points_threshold",
    "min_radius_m",
    "max_radius_m",
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
    if (
      std::find(updatable_params.begin(), updatable_params.end(), param.get_name()) ==
      updatable_params.end()) {
      result.successful = false;
      result.reason = "Parameter " + param.get_name() + " is not updatable";
      return result;
    }

    try {
      if (param.get_name() == "radial_resolution_m") {
        radial_resolution_m_ = param.as_double();
      } else if (param.get_name() == "azimuth_resolution_rad") {
        azimuth_resolution_rad_ = param.as_double();
      } else if (param.get_name() == "elevation_resolution_rad") {
        elevation_resolution_rad_ = param.as_double();
      } else if (param.get_name() == "voxel_points_threshold") {
        voxel_points_threshold_ = param.as_int();
      } else if (param.get_name() == "min_radius_m") {
        min_radius_m_ = param.as_double();
      } else if (param.get_name() == "max_radius_m") {
        max_radius_m_ = param.as_double();
      } else if (param.get_name() == "use_return_type_classification") {
        use_return_type_classification_ = param.as_bool();
      } else if (param.get_name() == "filter_secondary_returns") {
        filter_secondary_returns_ = param.as_bool();
      } else if (param.get_name() == "secondary_noise_threshold") {
        secondary_noise_threshold_ = param.as_int();
      } else if (param.get_name() == "primary_return_types") {
        auto values = param.as_integer_array();
        primary_return_types_.clear();
        for (const auto & val : values) {
          primary_return_types_.push_back(static_cast<int>(val));
        }
      } else if (param.get_name() == "publish_noise_cloud") {
        bool new_value = param.as_bool();
        if (new_value != publish_noise_cloud_) {
          publish_noise_cloud_ = new_value;
          // Recreate publisher if needed
          if (publish_noise_cloud_ && !noise_cloud_pub_) {
            rclcpp::PublisherOptions pub_options;
            pub_options.qos_overriding_options =
              rclcpp::QosOverridingOptions::with_default_policies();
            noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
              "polar_voxel_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(),
              pub_options);
          }
        }
      } else if (param.get_name() == "visibility_error_threshold") {
        visibility_error_threshold_ = param.as_double();
      } else if (param.get_name() == "visibility_warn_threshold") {
        visibility_warn_threshold_ = param.as_double();
      } else if (param.get_name() == "filter_ratio_error_threshold") {
        filter_ratio_error_threshold_ = param.as_double();
      } else if (param.get_name() == "filter_ratio_warn_threshold") {
        filter_ratio_warn_threshold_ = param.as_double();
      }

      RCLCPP_DEBUG(get_logger(), "Updated parameter: %s", param.get_name().c_str());
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = "Failed to update parameter " + param.get_name() + ": " + e.what();
      return result;
    }
  }

  return result;
}

PointCounts PolarVoxelOutlierFilterComponent::calculate_point_counts(
  const ValidPointsMask & valid_points_mask) const
{
  PointCounts counts{};
  counts.total_count = valid_points_mask.size();
  counts.valid_count = std::count(valid_points_mask.begin(), valid_points_mask.end(), true);
  counts.noise_count = counts.total_count - counts.valid_count;
  return counts;
}

void PolarVoxelOutlierFilterComponent::setup_output_header(
  PointCloud2 & output, const PointCloud2ConstPtr & input, size_t valid_count) const
{
  output.header = input->header;
  output.height = POINT_CLOUD_HEIGHT_ORGANIZED;
  output.width = static_cast<uint32_t>(valid_count);
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.row_step = output.width * output.point_step;
  output.is_dense = input->is_dense;
  output.data.resize(output.row_step * output.height);
}

sensor_msgs::msg::PointCloud2 PolarVoxelOutlierFilterComponent::create_noise_cloud(
  const PointCloud2ConstPtr & input, size_t noise_count) const
{
  sensor_msgs::msg::PointCloud2 noise_cloud;
  noise_cloud.header = input->header;
  noise_cloud.height = POINT_CLOUD_HEIGHT_ORGANIZED;
  noise_cloud.width = static_cast<uint32_t>(noise_count);
  noise_cloud.fields = input->fields;
  noise_cloud.is_bigendian = input->is_bigendian;
  noise_cloud.point_step = input->point_step;
  noise_cloud.row_step = noise_cloud.width * noise_cloud.point_step;
  noise_cloud.is_dense = input->is_dense;
  noise_cloud.data.resize(noise_cloud.row_step * noise_cloud.height);
  return noise_cloud;
}

PointProcessingResult PolarVoxelOutlierFilterComponent::create_processing_result(
  const PointCloud2ConstPtr & /* input */, const ValidPointsMask & valid_points_mask,
  const VoxelCountMap & voxel_counts, const VoxelIndexSet & valid_voxels,
  size_t voxels_passed_secondary_test) const
{
  auto point_counts = calculate_point_counts(valid_points_mask);

  PointProcessingResult result{};
  result.input_points = point_counts.total_count;
  result.output_points = point_counts.valid_count;
  result.voxels_passed_secondary_test = voxels_passed_secondary_test;
  result.populated_voxels = voxel_counts.size();
  result.total_voxels = valid_voxels.size();

  return result;
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
  stat.add("visibility_error_threshold", visibility_error_threshold_);
  stat.add("visibility_warn_threshold", visibility_warn_threshold_);

  if (visibility_value < visibility_error_threshold_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Visibility is critically low");
  } else if (visibility_value < visibility_warn_threshold_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Visibility is low");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Visibility is normal");
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
  const VoxelCountMap & voxel_counts) const
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
  const VoxelCountMap & voxel_counts, size_t & voxels_passed_secondary_test) const
{
  if (use_return_type_classification_) {
    return determine_valid_voxels_with_return_types(voxel_counts, voxels_passed_secondary_test);
  } else {
    voxels_passed_secondary_test = 0;  // Not applicable in simple mode
    return determine_valid_voxels_simple(voxel_counts);
  }
}

// Cartesian to polar voxel conversion (if needed)
PolarVoxelOutlierFilterComponent::PolarVoxelIndex
PolarVoxelOutlierFilterComponent::cartesian_to_polar_voxel(
  const CartesianCoordinate & cartesian) const
{
  PolarCoordinate polar = cartesian_to_polar(cartesian);
  return polar_to_polar_voxel(polar);
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent)
