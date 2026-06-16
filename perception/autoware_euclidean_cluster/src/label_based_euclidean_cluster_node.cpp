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

#include <Eigen/Core>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/common/common.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <memory>
#include <numeric>
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
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

struct SemanticPoint
{
  pcl::PointXYZ point;
  float probability{};
};

/// @brief Check whether a point cloud contains a field with the expected datatype.
bool has_field(
  const sensor_msgs::msg::PointCloud2 & pointcloud, const std::string & name,
  const std::uint8_t datatype)
{
  return std::any_of(pointcloud.fields.begin(), pointcloud.fields.end(), [&](const auto & field) {
    return field.name == name && field.datatype == datatype;
  });
}

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
/// NOTE: This should be unnecessary once
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

/// @brief Extract per-label parameter prefixes from nested overrides.
///        Example: "label_cluster_params.car.tolerance_m" -> {"car", "label_cluster_params.car."}
std::unordered_map<std::string, std::string> load_label_cluster_parameter_prefixes(
  const rclcpp::NodeOptions & options)
{
  constexpr std::string_view prefix = "label_cluster_params.";

  // first: label name, second: parameter prefix including the label name and trailing dot
  std::unordered_map<std::string, std::string> outputs;

  for (const auto & parameter : options.parameter_overrides()) {
    const auto & name = parameter.get_name();

    if (name.rfind(prefix, 0) != 0) {
      continue;
    }

    const std::string rest = name.substr(prefix.size());

    const auto dot_pos = rest.find('.');
    if (dot_pos != std::string::npos) {
      outputs.emplace(rest.substr(0, dot_pos), std::string(prefix) + rest.substr(0, dot_pos) + ".");
    }
  }

  return outputs;
}

/// @brief Create fallback shape and pose from the cluster axis-aligned bounding box.
std::pair<Shape, geometry_msgs::msg::Pose> create_fallback_shape_and_pose(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const std::uint8_t label)
{
  Shape shape;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;

  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  pcl::getMinMax3D(cluster, min_point, max_point);

  pose.position.x = 0.5 * (min_point.x() + max_point.x());
  pose.position.y = 0.5 * (min_point.y() + max_point.y());
  pose.position.z = 0.5 * (min_point.z() + max_point.z());

  const float dx = std::max(max_point.x() - min_point.x(), 0.1F);
  const float dy = std::max(max_point.y() - min_point.y(), 0.1F);
  const float dz = std::max(max_point.z() - min_point.z(), 0.1F);

  if (label == ObjectClassification::PEDESTRIAN) {
    shape.type = Shape::CYLINDER;
    shape.dimensions.x = std::max(dx, dy);
    shape.dimensions.y = std::max(dx, dy);
    shape.dimensions.z = dz;
  } else {
    shape.type = Shape::BOUNDING_BOX;
    shape.dimensions.x = dx;
    shape.dimensions.y = dy;
    shape.dimensions.z = dz;
  }

  return {shape, pose};
}

/// @brief Return true when the estimator populated a usable shape output.
bool has_usable_estimated_shape(const Shape & shape)
{
  switch (shape.type) {
    case Shape::BOUNDING_BOX:
    case Shape::CYLINDER:
      return shape.dimensions.x > 0.0 && shape.dimensions.y > 0.0 && shape.dimensions.z > 0.0;
    case Shape::POLYGON:
      return !shape.footprint.points.empty() && shape.dimensions.z > 0.0;
    default:
      return false;
  }
}

/// @brief Build a detected object with estimated or fallback shape and pose.
DetectedObject create_detected_object(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const std::uint8_t label, const float probability,
  const ShapePolicy shape_policy, autoware::shape_estimation::ShapeEstimator & shape_estimator)
{
  DetectedObject object;
  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;
  // UNKNOWN uses the convex hull model, which is the polygon path in ShapeEstimator.
  const std::uint8_t shape_label =
    (shape_policy == ShapePolicy::LABEL_DEPEND) ? label : ObjectClassification::UNKNOWN;
  shape_estimator.estimateShapeAndPose(
    shape_label, cluster, boost::none, boost::none, boost::none, shape, pose);

  if (!has_usable_estimated_shape(shape)) {
    std::tie(shape, pose) = create_fallback_shape_and_pose(cluster, label);
  }

  object.shape = shape;
  object.existence_probability = probability;
  object.classification.push_back(
    autoware_perception_msgs::build<ObjectClassification>().label(label).probability(probability));
  object.kinematics.pose_with_covariance.pose = pose;
  object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;

  return object;
}

/// @brief Split semantic points into buckets keyed by mapped object label.
std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> split_pointcloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const std::unordered_map<std::uint8_t, std::uint8_t> & class_id_to_object_label,
  const float min_probability)
{
  std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> buckets;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  const bool has_class_id = has_field(pointcloud, "class_id", sensor_msgs::msg::PointField::UINT8);
  const bool has_probability =
    has_field(pointcloud, "probability", sensor_msgs::msg::PointField::FLOAT32);

  if (has_class_id && has_probability) {
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> iter_class(pointcloud, "class_id");
    sensor_msgs::PointCloud2ConstIterator<float> iter_probability(pointcloud, "probability");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_class, ++iter_probability) {
      if (*iter_probability < min_probability) {
        continue;
      }

      const auto mapping = class_id_to_object_label.find(*iter_class);
      if (mapping == class_id_to_object_label.end()) {
        continue;
      }

      buckets[mapping->second].push_back(
        SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), *iter_probability});
    }
    return buckets;
  }

  if (has_class_id) {
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> iter_class(pointcloud, "class_id");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_class) {
      const auto mapping = class_id_to_object_label.find(*iter_class);
      if (mapping == class_id_to_object_label.end()) {
        continue;
      }

      buckets[mapping->second].push_back(
        SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), 1.0F});
    }
    return buckets;
  }

  if (has_probability) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_probability(pointcloud, "probability");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_probability) {
      if (*iter_probability < min_probability) {
        continue;
      }

      buckets[ObjectClassification::UNKNOWN].push_back(
        SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), *iter_probability});
    }
    return buckets;
  }

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    buckets[ObjectClassification::UNKNOWN].push_back(
      SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), 1.0F});
  }

  return buckets;
}

/// @brief Compute the average semantic probability for a set of points.
float average_probability(const std::vector<SemanticPoint> & points)
{
  if (points.empty()) {
    return 0.0F;
  }

  const float sum = std::accumulate(
    points.begin(), points.end(), 0.0F,
    [](const float acc, const auto & point) { return acc + point.probability; });
  return sum / static_cast<float>(points.size());
}

/// @brief Parse confusable_label_groups.* parameters from NodeOptions overrides.
std::vector<ConfusableLabelGroup> load_confusable_groups(const rclcpp::NodeOptions & options)
{
  constexpr std::string_view prefix = "confusable_label_groups.";
  std::unordered_map<std::string, ConfusableLabelGroup> groups_map;
  // Records which "<group>.<key>" overrides were actually provided, so required keys can be
  // validated without per-key tracking containers.
  std::unordered_set<std::string> provided_keys;

  for (const auto & param : options.parameter_overrides()) {
    const auto & name = param.get_name();
    if (name.rfind(prefix, 0) != 0) {
      continue;
    }

    const auto rest = name.substr(prefix.size());
    const auto dot = rest.find('.');
    if (dot == std::string::npos) {
      continue;
    }

    const auto group_name = rest.substr(0, dot);
    const auto key = rest.substr(dot + 1);
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
    // A group needs at least 2 labels to be confusable with each other; smaller groups are
    // meaningless and silently skipped.
    if (group.labels.size() < 2) {
      continue;
    }
    // cross_label_tolerance_m and max_merged_size_m must both be set explicitly; without them the
    // group would silently fall back to the default-constructed value (0). Treat a missing required
    // field as a hard configuration error.
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
  min_probability_ = static_cast<float>(
    autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, "min_probability"));
  shape_policy_ = to_shape_policy(
    autoware_utils_rclcpp::get_or_declare_parameter<uint8_t>(*this, "shape_policy"));

  const auto class_mappings = extract_class_mappings(options);
  if (!update_target_label_map(class_mappings)) {
    throw std::runtime_error("No supported classes were configured for clustering");
  }

  // Initialize the voxel grid based euclidean cluster
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
  default_cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);

  // Build per-label cluster overrides from label_cluster_params.<label_name>.* parameters.
  // Any omitted sub-key falls back to the global default above.
  {
    for (const auto & entry : load_label_cluster_parameter_prefixes(options)) {
      const auto & label_name = entry.first;
      const auto & label_prefix = entry.second;
      auto has = [&](const std::string & key) { return this->has_parameter(label_prefix + key); };
      // Skip labels with no overrides at all
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
      label_cluster_executers_[label] = std::make_shared<VoxelGridBasedEuclideanCluster>(
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

  {
    // Initialize the shape estimator
    const auto use_shape_estimation_corrector =
      autoware_utils_rclcpp::get_or_declare_parameter<bool>(
        *this, "use_shape_estimation_corrector");
    const auto use_shape_estimation_filter =
      autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_shape_estimation_filter");
    const auto use_boost_bbox_optimizer =
      autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_boost_bbox_optimizer");

    shape_estimator_ = std::make_unique<autoware::shape_estimation::ShapeEstimator>(
      use_shape_estimation_corrector, use_shape_estimation_filter, use_boost_bbox_optimizer);
  }

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LabelBasedEuclideanClusterNode::on_pointcloud, this, _1));
  objects_pub_ = AUTOWARE_CREATE_PUBLISHER2(DetectedObjects, "output", rclcpp::QoS{1});

  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, "~/debug");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  confusable_groups_ = load_confusable_groups(options);
  for (std::size_t g = 0; g < confusable_groups_.size(); ++g) {
    for (const auto label : confusable_groups_[g].labels) {
      const auto [it, inserted] = label_to_group_idx_.emplace(label, g);
      if (!inserted) {
        RCLCPP_WARN(
          get_logger(),
          "Label %u is listed in multiple confusable_label_groups; keeping the first assignment "
          "(group index %zu) and ignoring group index %zu.",
          static_cast<unsigned>(label), it->second, g);
      }
    }
  }
}

EuclideanClusterInterface & LabelBasedEuclideanClusterNode::get_cluster_executer(
  const std::uint8_t label) const
{
  const auto it = label_cluster_executers_.find(label);
  return (it != label_cluster_executers_.end()) ? *it->second : *default_cluster_;
}

bool LabelBasedEuclideanClusterNode::update_target_label_map(
  const std::vector<std::pair<std::string, std::string>> & class_mappings)
{
  class_id_to_object_label_.clear();

  for (size_t class_id_value = 0; class_id_value < class_mappings.size(); ++class_id_value) {
    const auto & [original_class_name, mapped_label_name] = class_mappings.at(class_id_value);
    const auto class_id = static_cast<std::uint8_t>(class_id_value);
    if (is_ignored_mapping(mapped_label_name)) {
      continue;
    }

    class_id_to_object_label_[class_id] =
      object_recognition_utils::toLabel(normalize_object_label_name(mapped_label_name));
  }

  return !class_id_to_object_label_.empty();
}

void LabelBasedEuclideanClusterNode::on_pointcloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  if (
    !has_field(*input_msg, "x", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(*input_msg, "y", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(*input_msg, "z", sensor_msgs::msg::PointField::FLOAT32)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Skipping pointcloud without required fields: x, y, z");
    return;
  }

  // 1. Split points by label and filter by probability
  auto split_points = split_pointcloud(*input_msg, class_id_to_object_label_, min_probability_);

  auto output_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(objects_pub_);
  output_msg->header = input_msg->header;

  // 2. Run per-label clustering and collect all cluster entries
  // TODO(ktro2828): Probability is averaged per label bucket before clustering, not per cluster.
  std::vector<ClusterEntry> all_entries;
  for (const auto & [label, semantic_points] : split_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    label_cloud->reserve(semantic_points.size());
    for (const auto & sp : semantic_points) {
      label_cloud->push_back(sp.point);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    get_cluster_executer(label).cluster(label_cloud, clusters);

    const float prob = average_probability(semantic_points);
    for (auto & cluster : clusters) {
      if (!cluster.empty()) {
        all_entries.push_back({std::move(cluster), label, prob});
      }
    }
  }

  // 3. Post-merge clusters that belong to the same confusable label group
  std::vector<std::vector<ClusterEntry>> per_group(confusable_groups_.size());
  std::vector<ClusterEntry> output_entries;
  output_entries.reserve(all_entries.size());

  for (auto & e : all_entries) {
    const auto it = label_to_group_idx_.find(e.label);
    if (it != label_to_group_idx_.end()) {
      per_group[it->second].push_back(std::move(e));
    } else {
      output_entries.push_back(std::move(e));
    }
  }
  for (std::size_t g = 0; g < confusable_groups_.size(); ++g) {
    for (auto & e : merge_confusable_clusters(std::move(per_group[g]), confusable_groups_[g])) {
      output_entries.push_back(std::move(e));
    }
  }

  // 4. Build detected objects from final entries
  for (const auto & e : output_entries) {
    output_msg->objects.push_back(
      create_detected_object(e.cloud, e.label, e.prob, shape_policy_, *shape_estimator_));
  }

  objects_pub_->publish(std::move(output_msg));

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
