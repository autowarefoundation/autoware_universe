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

#include "autoware/euclidean_cluster/utils.hpp"

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <limits>
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
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

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

/// @brief Convert a list of class ids into a printable string for logging.
std::string to_string(const std::vector<std::int64_t> & values)
{
  std::string output = "[";
  for (size_t i = 0; i < values.size(); ++i) {
    output += std::to_string(values.at(i));
    if (i + 1 < values.size()) {
      output += ", ";
    }
  }
  output += "]";
  return output;
}

/// @brief Map a semantic class name to an Autoware object classification label.
std::optional<std::uint8_t> to_object_label(const std::string & class_name)
{
  if (class_name == "car") {
    return ObjectClassification::CAR;
  }
  if (class_name == "bus") {
    return ObjectClassification::BUS;
  }
  if (class_name == "truck") {
    return ObjectClassification::TRUCK;
  }
  if (class_name == "motorcycle") {
    return ObjectClassification::MOTORCYCLE;
  }
  if (class_name == "bicycle") {
    return ObjectClassification::BICYCLE;
  }
  if (class_name == "pedestrian") {
    return ObjectClassification::PEDESTRIAN;
  }
  return std::nullopt;
}

/// @brief Create a fallback pose from the center of the cluster bounding box.
geometry_msgs::msg::Pose create_fallback_pose(const pcl::PointCloud<pcl::PointXYZ> & cluster)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();

  for (const auto & point : cluster.points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  pose.position.x = 0.5 * (min_x + max_x);
  pose.position.y = 0.5 * (min_y + max_y);
  pose.position.z = 0.5 * (min_z + max_z);
  return pose;
}

/// @brief Create a simple fallback shape when shape estimation fails.
Shape create_fallback_shape(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const std::uint8_t label)
{
  Shape shape;

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();

  for (const auto & point : cluster.points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  const float dx = std::max(max_x - min_x, 0.1F);
  const float dy = std::max(max_y - min_y, 0.1F);
  const float dz = std::max(max_z - min_z, 0.1F);

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

  return shape;
}

/// @brief Build a feature object message from a clustered point cloud.
DetectedObjectWithFeature create_feature_object(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const std::uint8_t label, const float probability)
{
  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(cluster, ros_pointcloud);
  ros_pointcloud.header = header;

  DetectedObjectWithFeature feature_object;
  feature_object.feature.cluster = ros_pointcloud;
  feature_object.object.existence_probability = probability;
  feature_object.object.kinematics.pose_with_covariance.pose.position = getCentroid(ros_pointcloud);

  ObjectClassification classification;
  classification.label = label;
  classification.probability = probability;
  feature_object.object.classification.emplace_back(classification);
  return feature_object;
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
    shape = create_fallback_shape(cluster, label);
    pose = create_fallback_pose(cluster);
  }

  object.shape = shape;
  object.kinematics.pose_with_covariance.pose = pose;

  return object;
}

/// @brief Split semantic points into buckets keyed by mapped object label.
std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> split_by_label(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const std::unordered_map<std::uint8_t, std::uint8_t> & class_id_to_object_label,
  const float min_probability, const float default_probability, const bool has_probability_field)
{
  std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> buckets;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> iter_class(pointcloud, "class_id");
  if (has_probability_field) {
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

  if (default_probability < min_probability) {
    return buckets;
  }

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_class) {
    const auto mapping = class_id_to_object_label.find(*iter_class);
    if (mapping == class_id_to_object_label.end()) {
      continue;
    }

    buckets[mapping->second].push_back(
      SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), default_probability});
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
: Node("label_based_euclidean_cluster_node", options)
{
  const auto use_height = this->declare_parameter<bool>("use_height");
  const auto min_cluster_size =
    static_cast<int>(this->declare_parameter<int64_t>("min_cluster_size"));
  const auto max_cluster_size =
    static_cast<int>(this->declare_parameter<int64_t>("max_cluster_size"));
  const auto tolerance = static_cast<float>(this->declare_parameter<double>("tolerance"));
  min_probability_ = static_cast<float>(this->declare_parameter<double>("min_probability"));
  default_probability_ =
    static_cast<float>(this->declare_parameter<double>("default_probability"));
  const auto use_shape_estimation_corrector =
    this->declare_parameter<bool>("use_shape_estimation_corrector");
  const auto use_shape_estimation_filter =
    this->declare_parameter<bool>("use_shape_estimation_filter");
  const auto use_boost_bbox_optimizer = this->declare_parameter<bool>("use_boost_bbox_optimizer");
  const auto class_names = this->declare_parameter<std::vector<std::string>>("class_names");
  const auto target_class_ids =
    this->declare_parameter<std::vector<std::int64_t>>("target_class_ids");

  if (!update_target_label_map(class_names, target_class_ids)) {
    throw std::runtime_error("No supported PTV3 target classes were configured for clustering");
  }

  cluster_ =
    std::make_shared<EuclideanCluster>(use_height, min_cluster_size, max_cluster_size, tolerance);
  shape_estimator_ = std::make_unique<autoware::shape_estimation::ShapeEstimator>(
    use_shape_estimation_corrector, use_shape_estimation_filter, use_boost_bbox_optimizer);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LabelBasedEuclideanClusterNode::on_point_cloud, this, _1));
  labeled_cluster_pub_ =
    this->create_publisher<DetectedObjectsWithFeature>("output/labeled_clusters", rclcpp::QoS{1});
  boxes_pub_ = this->create_publisher<DetectedObjects>("output/boxes", rclcpp::QoS{1});
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);

  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, "~/debug");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

bool LabelBasedEuclideanClusterNode::update_target_label_map(
  const std::vector<std::string> & class_names, const std::vector<std::int64_t> & target_class_ids)
{
  class_id_to_object_label_.clear();

  for (const auto class_id_value : target_class_ids) {
    if (class_id_value < 0 || class_id_value >= static_cast<std::int64_t>(class_names.size())) {
      RCLCPP_WARN(
        get_logger(), "Ignoring PTV3 target class id %ld because it is outside class_names range",
        class_id_value);
      continue;
    }

    const auto class_id = static_cast<std::uint8_t>(class_id_value);
    const auto mapped_label = to_object_label(class_names.at(class_id));
    if (!mapped_label.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Ignoring unsupported PTV3 class '%s' from target_class_ids %s",
        class_names.at(class_id).c_str(), to_string(target_class_ids).c_str());
      continue;
    }

    class_id_to_object_label_[class_id] = mapped_label.value();
  }

  return !class_id_to_object_label_.empty();
}

void LabelBasedEuclideanClusterNode::on_point_cloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  if (
    !has_field(*input_msg, "x", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(*input_msg, "y", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(*input_msg, "z", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(*input_msg, "class_id", sensor_msgs::msg::PointField::UINT8)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipping pointcloud without required PTV3 fields: x, y, z, class_id");
    return;
  }

  const bool has_probability =
    has_field(*input_msg, "probability", sensor_msgs::msg::PointField::FLOAT32);

  auto split_points = split_by_label(
    *input_msg, class_id_to_object_label_, min_probability_, default_probability_, has_probability);

  DetectedObjectsWithFeature labeled_clusters_msg;
  labeled_clusters_msg.header = input_msg->header;
  DetectedObjects boxed_objects_msg;
  boxed_objects_msg.header = input_msg->header;

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

      labeled_clusters_msg.feature_objects.push_back(
        create_feature_object(input_msg->header, cluster, label, label_probability));
      boxed_objects_msg.objects.push_back(
        create_box_object(cluster, label, label_probability, *shape_estimator_));
    }
  }

  labeled_cluster_pub_->publish(labeled_clusters_msg);
  boxes_pub_->publish(boxed_objects_msg);

  if (debug_pub_->get_subscription_count() >= 1) {
    sensor_msgs::msg::PointCloud2 debug;
    convertObjectMsg2SensorMsg(labeled_clusters_msg, debug);
    debug_pub_->publish(debug);
  }

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
