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
#include <cstdint>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
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

/// @brief Map a configured label name to an Autoware object classification label.
std::optional<std::uint8_t> to_object_label(const std::string & mapped_label)
{
  if (mapped_label == "car") {
    return ObjectClassification::CAR;
  }
  if (mapped_label == "bus") {
    return ObjectClassification::BUS;
  }
  if (mapped_label == "truck") {
    return ObjectClassification::TRUCK;
  }
  if (mapped_label == "motorcycle") {
    return ObjectClassification::MOTORCYCLE;
  }
  if (mapped_label == "bicycle") {
    return ObjectClassification::BICYCLE;
  }
  if (mapped_label == "pedestrian") {
    return ObjectClassification::PEDESTRIAN;
  }
  return std::nullopt;
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

/// @brief Build a detected object with estimated or fallback shape and pose.
DetectedObject create_box_object(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const std::uint8_t label, const float probability,
  autoware::shape_estimation::ShapeEstimator & shape_estimator)
{
  DetectedObject object;
  object.existence_probability = probability;

  const auto classification =
    autoware_perception_msgs::build<ObjectClassification>().label(label).probability(probability);
  object.classification.push_back(classification);

  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;
  const bool estimated = shape_estimator.estimateShapeAndPose(
    label, cluster, boost::none, boost::none, boost::none, shape, pose);

  if (!estimated) {
    std::tie(shape, pose) = create_fallback_shape_and_pose(cluster, label);
  }

  object.shape = shape;
  object.kinematics.pose_with_covariance.pose = pose;
  object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;

  return object;
}

/// @brief Split semantic points into buckets keyed by mapped object label.
std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> split_by_label(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const std::unordered_map<std::uint8_t, std::uint8_t> & class_id_to_object_label,
  const float min_probability)
{
  std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> buckets;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");
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
}  // namespace

LabelBasedEuclideanClusterNode::LabelBasedEuclideanClusterNode(const rclcpp::NodeOptions & options)
: Node("label_based_euclidean_cluster_node", allow_dynamic_params(options))
{
  min_probability_ = static_cast<float>(
    autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, "min_probability"));

  const auto class_mappings = extract_class_mappings(options);
  if (!update_target_label_map(class_mappings)) {
    throw std::runtime_error("No supported classes were configured for clustering");
  }

  {
    // Initialize the euclidean cluster
    const auto use_height =
      autoware_utils_rclcpp::get_or_declare_parameter<bool>(*this, "use_height");
    const auto min_cluster_size = static_cast<int>(
      autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(*this, "min_cluster_size"));
    const auto max_cluster_size = static_cast<int>(
      autoware_utils_rclcpp::get_or_declare_parameter<int64_t>(*this, "max_cluster_size"));
    const auto tolerance = static_cast<float>(
      autoware_utils_rclcpp::get_or_declare_parameter<double>(*this, "tolerance"));
    cluster_ =
      std::make_shared<EuclideanCluster>(use_height, min_cluster_size, max_cluster_size, tolerance);
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
  objects_pub_ = this->create_publisher<DetectedObjects>("output", rclcpp::QoS{1});

  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, "~/debug");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
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

    const auto mapped_label = to_object_label(mapped_label_name);
    if (!mapped_label.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Ignoring unsupported mapped label '%s' for class '%s'",
        mapped_label_name.c_str(), original_class_name.c_str());
      continue;
    }

    class_id_to_object_label_[class_id] = mapped_label.value();
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
    !has_field(*input_msg, "z", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(*input_msg, "class_id", sensor_msgs::msg::PointField::UINT8) ||
    !has_field(*input_msg, "probability", sensor_msgs::msg::PointField::FLOAT32)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping pointcloud without required fields: x, y, z, class_id, probability");
    return;
  }

  // 1. Split points by label and filter by probability
  auto split_points = split_by_label(*input_msg, class_id_to_object_label_, min_probability_);

  DetectedObjects output_msg;
  output_msg.header = input_msg->header;

  for (const auto & [label, semantic_points] : split_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    label_cloud->reserve(semantic_points.size());
    for (const auto & semantic_point : semantic_points) {
      label_cloud->push_back(semantic_point.point);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    cluster_->cluster(label_cloud, clusters);

    const float label_probability = average_probability(semantic_points);
    for (const auto & cluster : clusters) {
      if (cluster.empty()) {
        continue;
      }

      output_msg.objects.push_back(
        create_box_object(cluster, label, label_probability, *shape_estimator_));
    }
  }

  objects_pub_->publish(output_msg);

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
