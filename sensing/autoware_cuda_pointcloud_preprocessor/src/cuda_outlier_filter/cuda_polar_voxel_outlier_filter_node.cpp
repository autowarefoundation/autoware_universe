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

#include <cmath>
#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{
constexpr double two_pi = 2.0 * M_PI;

inline double adjust_resolution_to_circle(double requested_resolution)
{
  int bins = static_cast<int>(std::round(two_pi / requested_resolution));
  if (bins < 1) bins = 1;
  return two_pi / bins;
}
}  // namespace

CudaPolarVoxelOutlierFilterNode::CudaPolarVoxelOutlierFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_polar_voxel_outlier_filter", node_options), updater_(this)
{
  // set initial parameters
  {
    filter_params_.radial_resolution_m = declare_parameter<double>("radial_resolution_m");
    filter_params_.azimuth_resolution_rad =
      adjust_resolution_to_circle(declare_parameter<double>("azimuth_resolution_rad"));
    filter_params_.elevation_resolution_rad =
      adjust_resolution_to_circle(declare_parameter<double>("elevation_resolution_rad"));
    filter_params_.voxel_points_threshold = declare_parameter<int>("voxel_points_threshold");
    filter_params_.min_radius_m = declare_parameter<double>("min_radius_m");
    filter_params_.max_radius_m = declare_parameter<double>("max_radius_m");
    filter_params_.visibility_estimation_max_range_m =
      declare_parameter<double>("visibility_estimation_max_range_m");
    filter_params_.use_return_type_classification =
      declare_parameter<bool>("use_return_type_classification");
    filter_params_.filter_secondary_returns = declare_parameter<bool>("filter_secondary_returns");
    filter_params_.secondary_noise_threshold = declare_parameter<int>("secondary_noise_threshold");
    filter_params_.intensity_threshold = declare_parameter<uint8_t>("intensity_threshold");
    filter_params_.visibility_error_threshold =
      declare_parameter<double>("visibility_error_threshold");
    filter_params_.visibility_warn_threshold =
      declare_parameter<double>("visibility_warn_threshold");
    filter_params_.filter_ratio_error_threshold =
      declare_parameter<double>("filter_ratio_error_threshold");
    filter_params_.filter_ratio_warn_threshold =
      declare_parameter<double>("filter_ratio_warn_threshold");
    filter_params_.publish_noise_cloud = declare_parameter<bool>("publish_noise_cloud");
    filter_params_.visibility_estimation_max_secondary_voxel_count = static_cast<int>(
      declare_parameter<int64_t>("visibility_estimation_max_secondary_voxel_count"));
    filter_params_.visibility_estimation_only =
      declare_parameter<bool>("visibility_estimation_only");

    // rclcpp always returns integer array as std::vector<int64_t>
    auto primary_return_types_param =
      declare_parameter<std::vector<int64_t>>("primary_return_types");
    primary_return_types_.clear();
    primary_return_types_.reserve(primary_return_types_param.size());
    for (const auto & val : primary_return_types_param) {
      primary_return_types_.push_back(static_cast<int>(val));
    }
  }

  cuda_polar_voxel_outlier_filter_ = std::make_unique<CudaPolarVoxelOutlierFilter>();
  cuda_polar_voxel_outlier_filter_->set_primary_return_types(primary_return_types_);

  std::string diagnostics_hardware_id =
    declare_parameter<std::string>("hardware_id", "cuda_polar_voxel_outlier_filter");

  // Initialize diagnostics
  updater_.setHardwareID(diagnostics_hardware_id);
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &CudaPolarVoxelOutlierFilterNode::on_visibility_check);
  updater_.add(
    std::string(this->get_namespace()) + ": filter_ratio_validation", this,
    &CudaPolarVoxelOutlierFilterNode::on_filter_ratio_check);
  updater_.setPeriod(diagnostics_update_period_sec);

  // Create visibility publisher
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/visibility", rclcpp::SensorDataQoS());

  // Create ratio publisher
  ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/filter_ratio", rclcpp::SensorDataQoS());

  pointcloud_sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaPolarVoxelOutlierFilterNode::pointcloud_callback, this, std::placeholders::_1));

  filtered_cloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  // Create noise cloud publisher if enabled
  if (filter_params_.publish_noise_cloud) {
    noise_cloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/debug/pointcloud_noise");
    RCLCPP_INFO(get_logger(), "Noise cloud publishing enabled");
  } else {
    RCLCPP_INFO(get_logger(), "Noise cloud publishing disabled for performance optimization");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return param_callback(p); });

  RCLCPP_INFO(
    get_logger(),
    "Polar Voxel Outlier Filter initialized - supports PointXYZIRC and PointXYZIRCAEDT "
    "%s",
    filter_params_.visibility_estimation_only
      ? " (visibility estimation only - no point cloud output)"
      : "");
}

void CudaPolarVoxelOutlierFilterNode::pointcloud_callback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // Take mutex so that node configuration will not be
  // overwritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  if (!msg) {
    RCLCPP_ERROR(this->get_logger(), "Input point cloud is null");
    throw std::invalid_argument("Input point cloud is null");
  }

  validate_filter_inputs(msg);

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  bool has_polar_coords =
    autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(*msg);
  bool has_return_type =
    autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzirc(*msg);

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filtered_cloud;
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> noise_cloud;
  CudaPolarVoxelOutlierFilter::FilterReturn filter_return{};
  if (has_polar_coords) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRCAEDT format with pre-computed polar coordinates");
    filter_return = cuda_polar_voxel_outlier_filter_->filter(
      msg, filter_params_, CudaPolarVoxelOutlierFilter::PolarDataType::PreComputed);
  } else if (has_return_type) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRC format, computing azimuth and elevation");
    filter_return = cuda_polar_voxel_outlier_filter_->filter(
      msg, filter_params_, CudaPolarVoxelOutlierFilter::PolarDataType::DeriveFromCartesian);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "PointXYZ format has not been supported by "
      "autoware_cuda_pointcloud_preprocessor::cuda_polar_voxel_outlier_filter yet.");
  }

  filtered_cloud = std::move(filter_return.filtered_cloud);
  noise_cloud = std::move(filter_return.noise_cloud);
  filter_ratio_ = filter_return.filter_ratio;
  visibility_ = filter_return.visibility;

  if (!filtered_cloud) {
    // filtered_cloud contains nullptr
    return;
  }

  // Publish results (skip if visibility estimation only)
  if (!filter_params_.visibility_estimation_only) {
    filtered_cloud_pub_->publish(std::move(filtered_cloud));
    if (filter_params_.publish_noise_cloud && noise_cloud_pub_) {
      noise_cloud_pub_->publish(std::move(noise_cloud));
    }
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "visibility estimation only mode - no point cloud output generated");
  }

  if (ratio_pub_) {
    autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
    ratio_msg.data = static_cast<float>(filter_ratio_.value_or(0.0));
    ratio_msg.stamp = this->now();
    ratio_pub_->publish(ratio_msg);
  }

  if (visibility_pub_ && visibility_.has_value()) {
    autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.data = static_cast<float>(visibility_.value());
    visibility_msg.stamp = this->now();
    visibility_pub_->publish(visibility_msg);
  }
}

bool CudaPolarVoxelOutlierFilterNode::validate_positive_double(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_double() <= 0.0) {
    reason = param.get_name() + " must be positive";
    return false;
  }
  return true;
}

bool CudaPolarVoxelOutlierFilterNode::validate_non_negative_double(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_double() < 0.0) {
    reason = param.get_name() + " must be non-negative";
    return false;
  }
  return true;
}

bool CudaPolarVoxelOutlierFilterNode::validate_positive_int(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_int() < 1) {
    reason = param.get_name() + " must be at least 1";
    return false;
  }
  return true;
}

bool CudaPolarVoxelOutlierFilterNode::validate_non_negative_int(
  const rclcpp::Parameter & param, std::string & reason)
{
  if (param.as_int() < 0) {
    reason = param.get_name() + " must be non-negative";
    return false;
  }
  return true;
}

bool CudaPolarVoxelOutlierFilterNode::validate_intensity_threshold(
  const rclcpp::Parameter & param, std::string & reason)
{
  int val = param.as_int();
  if (val < 0 || val > 255) {
    reason = "intensity_threshold must be between 0 and 255";
    return false;
  }
  return true;
}

bool CudaPolarVoxelOutlierFilterNode::validate_primary_return_types(
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

bool CudaPolarVoxelOutlierFilterNode::validate_normalized(
  const rclcpp::Parameter & param, std::string & reason)
{
  double val = param.as_double();
  if (val < 0.0 || val > 1.0) {
    reason = param.get_name() + " must be between 0.0 and 1.0";
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult CudaPolarVoxelOutlierFilterNode::param_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  std::scoped_lock lock(param_mutex_);

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
      [this](const rclcpp::Parameter & p) { filter_params_.radial_resolution_m = p.as_double(); }}},
    {"azimuth_resolution_rad",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) {
        filter_params_.azimuth_resolution_rad = adjust_resolution_to_circle(p.as_double());
      }}},
    {"elevation_resolution_rad",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) {
        filter_params_.elevation_resolution_rad = adjust_resolution_to_circle(p.as_double());
      }}},
    {"voxel_points_threshold",
     {validate_positive_int,
      [this](const rclcpp::Parameter & p) { filter_params_.voxel_points_threshold = p.as_int(); }}},
    {"min_radius_m",
     {validate_non_negative_double,
      [this](const rclcpp::Parameter & p) { filter_params_.min_radius_m = p.as_double(); }}},
    {"max_radius_m",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) { filter_params_.max_radius_m = p.as_double(); }}},
    {"intensity_threshold",
     {validate_intensity_threshold,
      [this](const rclcpp::Parameter & p) { filter_params_.intensity_threshold = p.as_int(); }}},
    {"visibility_estimation_max_range_m",
     {validate_positive_double,
      [this](const rclcpp::Parameter & p) {
        filter_params_.visibility_estimation_max_range_m = p.as_double();
      }}},
    {"use_return_type_classification",
     {nullptr,
      [this](const rclcpp::Parameter & p) {
        filter_params_.use_return_type_classification = p.as_bool();
      }}},
    {"filter_secondary_returns",
     {nullptr,
      [this](const rclcpp::Parameter & p) {
        filter_params_.filter_secondary_returns = p.as_bool();
      }}},
    {"secondary_noise_threshold",
     {validate_non_negative_int,
      [this](const rclcpp::Parameter & p) {
        filter_params_.secondary_noise_threshold = p.as_int();
      }}},
    {"visibility_estimation_max_secondary_voxel_count",
     {validate_non_negative_int,
      [this](const rclcpp::Parameter & p) {
        filter_params_.visibility_estimation_max_secondary_voxel_count = p.as_int();
      }}},
    {"primary_return_types",
     {validate_primary_return_types,
      [this](const rclcpp::Parameter & p) {
        const auto & arr = p.as_integer_array();
        primary_return_types_.clear();
        primary_return_types_.reserve(arr.size());
        for (auto v : arr) primary_return_types_.push_back(static_cast<int>(v));
        cuda_polar_voxel_outlier_filter_->set_primary_return_types(primary_return_types_);
      }}},
    {"visibility_estimation_only",
     {nullptr,
      [this](const rclcpp::Parameter & p) {
        filter_params_.visibility_estimation_only = p.as_bool();
      }}},
    {"publish_noise_cloud",
     {nullptr,
      [this](const rclcpp::Parameter & p) { filter_params_.publish_noise_cloud = p.as_bool(); }}},
    {"filter_ratio_error_threshold",
     {validate_normalized,
      [this](const rclcpp::Parameter & p) {
        filter_params_.filter_ratio_error_threshold = p.as_double();
      }}},
    {"filter_ratio_warn_threshold",
     {validate_normalized,
      [this](const rclcpp::Parameter & p) {
        filter_params_.filter_ratio_warn_threshold = p.as_double();
      }}},
    {"visibility_error_threshold",
     {validate_normalized,
      [this](const rclcpp::Parameter & p) {
        filter_params_.visibility_error_threshold = p.as_double();
      }}},
    {"visibility_warn_threshold", {validate_normalized, [this](const rclcpp::Parameter & p) {
                                     filter_params_.visibility_warn_threshold = p.as_double();
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

void CudaPolarVoxelOutlierFilterNode::on_visibility_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Take mutex so that node configuration will not be
  // overwritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  if (!visibility_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No visibility data available");
    return;
  }

  double visibility_value = visibility_.value();

  if (visibility_value < filter_params_.visibility_error_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Low visibility detected - potential adverse weather conditions");
  } else if (visibility_value < filter_params_.visibility_warn_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Reduced visibility detected - monitor environmental conditions");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Visibility within normal range");
  }

  stat.add("Visibility", visibility_value);
  stat.add("Error Threshold", filter_params_.visibility_error_threshold);
  stat.add("Warning Threshold", filter_params_.visibility_warn_threshold);
  stat.add("Estimation Range (m)", filter_params_.visibility_estimation_max_range_m);
  stat.add("Max Secondary Voxels", filter_params_.visibility_estimation_max_secondary_voxel_count);
}

void CudaPolarVoxelOutlierFilterNode::on_filter_ratio_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Take mutex so that node configuration will not be
  // overwritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  if (!filter_ratio_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No filter ratio data available");
    return;
  }

  double ratio_value = filter_ratio_.value();

  if (ratio_value < filter_params_.filter_ratio_error_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Very low filter ratio - excessive noise or sensor malfunction");
  } else if (ratio_value < filter_params_.filter_ratio_warn_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Low filter ratio - increased noise levels detected");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Filter ratio within normal range");
  }

  stat.add("Filter Ratio", ratio_value);
  stat.add("Error Threshold", filter_params_.filter_ratio_error_threshold);
  stat.add("Warning Threshold", filter_params_.filter_ratio_warn_threshold);
  stat.add("Filtering Mode", filter_params_.use_return_type_classification ? "Advanced" : "Simple");
  stat.add("Visibility Only", filter_params_.visibility_estimation_only ? "Yes" : "No");
}

void CudaPolarVoxelOutlierFilterNode::validate_filter_inputs(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud)
{
  validate_return_type_field(input_cloud);
  validate_intensity_field(input_cloud);
}

void CudaPolarVoxelOutlierFilterNode::validate_return_type_field(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud)
{
  if (!filter_params_.use_return_type_classification) {
    return;
  }

  if (!has_field(input_cloud, "return_type")) {
    RCLCPP_ERROR(
      get_logger(),
      "Advanced mode (use_return_type_classification=true) requires 'return_type' field. "
      "Set use_return_type_classification=false for simple mode or ensure input has return_type "
      "field.");
    throw std::invalid_argument("Advanced mode requires return_type field");
  }
}

void CudaPolarVoxelOutlierFilterNode::validate_intensity_field(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud)
{
  if (!has_field(input_cloud, "intensity")) {
    RCLCPP_ERROR(get_logger(), "Input point cloud must have 'intensity' field");
    throw std::invalid_argument("Input point cloud must have intensity field");
  }
}

bool CudaPolarVoxelOutlierFilterNode::has_field(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input, const std::string & field_name)
{
  for (const auto & field : input->fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPolarVoxelOutlierFilterNode)
