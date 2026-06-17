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

#include "label_based_euclidean_cluster_node.hpp"

#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
namespace
{
/// @brief Enable automatic declaration for nested parameters provided via YAML overrides.
rclcpp::NodeOptions allow_dynamic_params(const rclcpp::NodeOptions & original_options)
{
  rclcpp::NodeOptions options(original_options);
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  return options;
}

/// @brief Check whether the configured mapping explicitly ignores the class.
bool is_ignored_mapping(const std::string & mapped_label)
{
  return mapped_label == "ignore";
}

/// @brief Normalize configured label names to the uppercase form expected by toLabel().
/// NOTE: This function no longer uses once
/// https://github.com/autowarefoundation/autoware_core/pull/1184 has been merged.
std::string normalize_object_label_name(const std::string & label_name)
{
  std::string normalized = label_name;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return normalized;
}

/// @brief Convert an integer parameter into a validated shape policy.
ShapePolicy to_shape_policy(const std::uint8_t value)
{
  switch (value) {
    case ShapePolicy::ALL_POLYGON:
      return ShapePolicy::ALL_POLYGON;
    case ShapePolicy::LABEL_DEPEND:
      return ShapePolicy::LABEL_DEPEND;
    default:
      throw std::runtime_error("shape_policy must be 0 (ALL_POLYGON) or 1 (LABEL_DEPEND)");
  }
}

/// @brief Extract ordered class mappings from parameter overrides.
std::vector<std::pair<std::string, std::string>> extract_class_mappings(
  const rclcpp::NodeOptions & options)
{
  constexpr auto prefix = "class_names.";
  std::vector<std::pair<std::string, std::string>> class_mappings;

  for (const auto & parameter : options.parameter_overrides()) {
    const auto & parameter_name = parameter.get_name();
    if (parameter_name.rfind(prefix, 0) != 0) {
      continue;
    }

    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      continue;
    }

    class_mappings.emplace_back(
      parameter_name.substr(std::string(prefix).size()), parameter.as_string());
  }

  return class_mappings;
}

/// @brief Build class-id to object-label map from ordered class mappings.
std::unordered_map<std::uint8_t, std::uint8_t> build_target_label_map(
  const std::vector<std::pair<std::string, std::string>> & class_mappings)
{
  std::unordered_map<std::uint8_t, std::uint8_t> class_id_to_object_label;

  for (size_t class_id_value = 0; class_id_value < class_mappings.size(); ++class_id_value) {
    const auto & [original_class_name, mapped_label_name] = class_mappings.at(class_id_value);
    static_cast<void>(original_class_name);
    const auto class_id = static_cast<std::uint8_t>(class_id_value);
    if (is_ignored_mapping(mapped_label_name)) {
      continue;
    }

    class_id_to_object_label[class_id] =
      object_recognition_utils::toLabel(normalize_object_label_name(mapped_label_name));
  }

  return class_id_to_object_label;
}

struct NestedOverrideName
{
  std::string group_name;
  std::string key;
};

/// @brief Parse a nested override name in the form "<prefix><group>.<key>".
std::optional<NestedOverrideName> parse_nested_override_name(
  const std::string & parameter_name, const std::string_view prefix)
{
  if (parameter_name.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }

  const std::string rest = parameter_name.substr(prefix.size());
  const auto dot_pos = rest.find('.');
  if (dot_pos == std::string::npos) {
    return std::nullopt;
  }

  return NestedOverrideName{rest.substr(0, dot_pos), rest.substr(dot_pos + 1)};
}

/// @brief Extract per-label parameter prefixes from nested overrides.
std::unordered_map<std::string, std::string> load_label_cluster_parameter_prefixes(
  const rclcpp::NodeOptions & options)
{
  constexpr std::string_view prefix = "label_cluster_params.";

  std::unordered_map<std::string, std::string> outputs;

  for (const auto & parameter : options.parameter_overrides()) {
    const auto nested_name = parse_nested_override_name(parameter.get_name(), prefix);
    if (!nested_name) {
      continue;
    }

    outputs.emplace(nested_name->group_name, std::string(prefix) + nested_name->group_name + ".");
  }

  return outputs;
}

/// @brief Parse confusable_label_groups.* parameters from NodeOptions overrides.
std::vector<ConfusableLabelGroup> load_confusable_groups(const rclcpp::NodeOptions & options)
{
  constexpr std::string_view prefix = "confusable_label_groups.";
  std::unordered_map<std::string, ConfusableLabelGroup> groups_map;
  std::unordered_set<std::string> provided_keys;

  for (const auto & param : options.parameter_overrides()) {
    const auto nested_name = parse_nested_override_name(param.get_name(), prefix);
    if (!nested_name) {
      continue;
    }

    const auto & group_name = nested_name->group_name;
    const auto & key = nested_name->key;
    auto & group = groups_map[group_name];

    if (
      key == "cross_label_tolerance_m" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      group.cross_label_tolerance = static_cast<float>(param.as_double());
      provided_keys.insert(group_name + ".cross_label_tolerance_m");
    } else if (
      key == "max_merged_size_m" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      group.max_merged_size = static_cast<float>(param.as_double());
      provided_keys.insert(group_name + ".max_merged_size_m");
    } else if (
      key == "labels" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      for (const auto & label_name : param.as_string_array()) {
        group.labels.push_back(
          object_recognition_utils::toLabel(normalize_object_label_name(label_name)));
      }
    }
  }

  std::vector<ConfusableLabelGroup> result;
  for (auto & [name, group] : groups_map) {
    if (group.labels.size() < 2) {
      continue;
    }
    if (!provided_keys.count(name + ".cross_label_tolerance_m")) {
      throw std::runtime_error(
        "Confusable label group '" + name + "' is missing required 'cross_label_tolerance_m'.");
    }
    if (!provided_keys.count(name + ".max_merged_size_m")) {
      throw std::runtime_error(
        "Confusable label group '" + name + "' is missing required 'max_merged_size_m'.");
    }
    result.push_back(std::move(group));
  }
  return result;
}
}  // namespace

LabelBasedEuclideanClusterNode::LabelBasedEuclideanClusterNode(const rclcpp::NodeOptions & options)
: Node("label_based_euclidean_cluster_node", allow_dynamic_params(options))
{
  // Load parameters
  const auto min_probability = static_cast<float>(
    autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, "min_probability"));
  const auto shape_policy = to_shape_policy(
    autoware_utils_rclcpp::get_or_declare_parameter<uint8_t>(*this, "shape_policy"));

  const auto class_mappings = extract_class_mappings(options);
  const auto class_id_to_object_label = build_target_label_map(class_mappings);
  if (class_id_to_object_label.empty()) {
    throw std::runtime_error("No supported classes were configured for clustering");
  }

  // Initialize the default voxel grid based euclidean cluster
  const auto use_height =
    autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_height");
  const auto min_points_per_cluster = static_cast<int>(
    autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(*this, "min_points_per_cluster"));
  const auto tolerance = static_cast<float>(
    autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, "tolerance_m"));
  const auto voxel_leaf_size = static_cast<float>(
    autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, "voxel_leaf_size_m"));
  const auto min_points_per_voxel = static_cast<int>(
    autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(*this, "min_points_per_voxel"));
  const auto large_cluster_voxel_count_threshold =
    static_cast<int>(autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(
      *this, "large_cluster_voxel_count_threshold"));
  const auto large_cluster_max_points_per_voxel =
    static_cast<int>(autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(
      *this, "large_cluster_max_points_per_voxel"));
  const auto max_voxels_per_cluster = static_cast<int>(
    autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(*this, "max_voxels_per_cluster"));

  auto default_cluster = std::make_shared<VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);

  // Build per-label cluster overrides from label_cluster_params.<label_name>.* parameters
  std::unordered_map<std::uint8_t, std::shared_ptr<EuclideanClusterInterface>>
    label_cluster_executers;
  {
    for (const auto & entry : load_label_cluster_parameter_prefixes(options)) {
      const auto & label_name = entry.first;
      const auto & label_prefix = entry.second;
      auto has = [&](const std::string & key) { return this->has_parameter(label_prefix + key); };
      if (
        !has("tolerance_m") && !has("min_points_per_cluster") && !has("use_height") &&
        !has("voxel_leaf_size_m") && !has("min_points_per_voxel") &&
        !has("large_cluster_voxel_count_threshold") && !has("large_cluster_max_points_per_voxel") &&
        !has("max_voxels_per_cluster")) {
        continue;
      }

      auto get_bool = [&](const std::string & key, bool def) -> bool {
        return has(key) ? this->get_parameter(label_prefix + key).as_bool() : def;
      };
      auto get_int = [&](const std::string & key, int def) -> int {
        if (!has(key)) {
          return def;
        }
        const auto param = this->get_parameter(label_prefix + key);
        return param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                 ? static_cast<int>(param.as_double())
                 : static_cast<int>(param.as_int());
      };
      auto get_float = [&](const std::string & key, float def) -> float {
        if (!has(key)) {
          return def;
        }
        const auto param = this->get_parameter(label_prefix + key);
        return param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                 ? static_cast<float>(param.as_int())
                 : static_cast<float>(param.as_double());
      };

      const auto label = object_recognition_utils::toLabel(normalize_object_label_name(label_name));
      label_cluster_executers[label] = std::make_shared<VoxelGridBasedEuclideanCluster>(
        get_bool("use_height", use_height),
        get_int("min_points_per_cluster", min_points_per_cluster),
        get_float("tolerance_m", tolerance), get_float("voxel_leaf_size_m", voxel_leaf_size),
        get_int("min_points_per_voxel", min_points_per_voxel),
        get_int("large_cluster_voxel_count_threshold", large_cluster_voxel_count_threshold),
        get_int("large_cluster_max_points_per_voxel", large_cluster_max_points_per_voxel),
        get_int("max_voxels_per_cluster", max_voxels_per_cluster));

      RCLCPP_INFO(get_logger(), "Using custom cluster params for label '%s'", label_name.c_str());
    }
  }

  // Initialize the shape estimator
  auto shape_estimator = std::make_shared<autoware::shape_estimation::ShapeEstimator>(
    autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_shape_estimation_corrector"),
    autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_shape_estimation_filter"),
    autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_boost_bbox_optimizer"));

  // Load confusable label groups
  const auto confusable_groups = load_confusable_groups(options);

  // Create the core clustering processor
  processor_ = std::make_unique<LabelBasedEuclideanCluster>(
    class_id_to_object_label, min_probability, shape_policy, default_cluster,
    label_cluster_executers, shape_estimator, confusable_groups);

  // Set up ROS pub/sub
  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LabelBasedEuclideanClusterNode::on_pointcloud, this, _1));
  objects_pub_ = AUTOWARE_CREATE_PUBLISHER2(
    autoware_perception_msgs::msg::DetectedObjects, "output", rclcpp::QoS{1});

  // Initialize timing and debug
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, "~/debug");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void LabelBasedEuclideanClusterNode::on_pointcloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  // Process the point cloud using the core cluster
  auto output_msg = processor_->process(*input_msg);

  // Populate ROS-specific fields
  output_msg.header = input_msg->header;

  // Publish the result
  objects_pub_->publish(std::move(output_msg));

  // Handle timing and debug output
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - input_msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "pipeline_latency_ms", pipeline_latency_ms);
  }
}

}  // namespace autoware::euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::LabelBasedEuclideanClusterNode)
