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

#include "autoware/object_merger/object_fusion_merger_node.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using Shape = autoware_perception_msgs::msg::Shape;
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;
using MultiPolygon2d = boost::geometry::model::multi_polygon<Polygon2d>;
using MultiPoint2d = autoware_utils_geometry::MultiPoint2d;

/**
 * @brief Compute the combined vertical extent of one main object and its grouped sub objects.
 *
 * @param main_object Main object used as the base of the fused output.
 * @param sub_objects Sub objects uniquely grouped to the main object.
 * @return Minimum and maximum z values covered by all input objects.
 */
std::pair<double, double> get_z_range(
  const DetectedObject & main_object, const std::vector<DetectedObject> & sub_objects)
{
  const double main_half_z = main_object.shape.dimensions.z * 0.5;
  const double main_min_z =
    main_object.kinematics.pose_with_covariance.pose.position.z - main_half_z;
  const double main_max_z =
    main_object.kinematics.pose_with_covariance.pose.position.z + main_half_z;
  double min_z = main_min_z;
  double max_z = main_max_z;

  for (const auto & sub_object : sub_objects) {
    const double sub_half_z = sub_object.shape.dimensions.z * 0.5;
    min_z =
      std::min(min_z, sub_object.kinematics.pose_with_covariance.pose.position.z - sub_half_z);
    max_z =
      std::max(max_z, sub_object.kinematics.pose_with_covariance.pose.position.z + sub_half_z);
  }

  return {min_z, max_z};
}

/**
 * @brief Update the output pose z and shape height to cover the full grouped z range.
 *
 * @param output Output object to update in place.
 * @param main_object Main object used as the base of the fused output.
 * @param sub_objects Sub objects uniquely grouped to the main object.
 */
void fit_shape_height(
  DetectedObject & output, const DetectedObject & main_object,
  const std::vector<DetectedObject> & sub_objects)
{
  const auto [min_z, max_z] = get_z_range(main_object, sub_objects);
  output.kinematics.pose_with_covariance.pose.position.z = 0.5 * (min_z + max_z);
  output.shape.dimensions.z = max_z - min_z;
}

/**
 * @brief Transform a polygon into the current output local frame.
 *
 * @param output Current output object that defines the local output frame.
 * @param polygon Source polygon expressed in world coordinates.
 * @return Polygon expressed in the output local frame.
 */
Polygon2d transform_polygon_to_output_frame(
  const DetectedObject & output, const Polygon2d & polygon)
{
  Polygon2d local_polygon;
  for (const auto & point : polygon.outer()) {
    geometry_msgs::msg::Point world_point;
    world_point.x = boost::geometry::get<0>(point);
    world_point.y = boost::geometry::get<1>(point);
    world_point.z = output.kinematics.pose_with_covariance.pose.position.z;
    const auto local_point = autoware_utils_geometry::inverse_transform_point(
      world_point, output.kinematics.pose_with_covariance.pose);
    local_polygon.outer().push_back(Point2d(local_point.x, local_point.y));
  }
  boost::geometry::correct(local_polygon);
  return local_polygon;
}

/**
 * @brief Compute the exact grouped union polygons in the output local frame.
 *
 * @param output Current output object that defines the local output frame.
 * @param main_object Main object used as the base of the fused output.
 * @param sub_objects Sub objects uniquely grouped to the main object.
 * @return Grouped union polygons expressed in the output local frame.
 */
MultiPolygon2d collect_union_polygons_in_output_frame(
  const DetectedObject & output, const DetectedObject & main_object,
  const std::vector<DetectedObject> & sub_objects)
{
  MultiPolygon2d union_polygons;

  const auto append_union_polygon = [&union_polygons](const Polygon2d & polygon) {
    if (union_polygons.empty()) {
      union_polygons.push_back(polygon);
      return;
    }

    MultiPolygon2d next_union_polygons;
    boost::geometry::union_(union_polygons, polygon, next_union_polygons);
    union_polygons = std::move(next_union_polygons);
  };

  const auto main_polygon = autoware_utils_geometry::to_polygon2d(main_object);
  append_union_polygon(transform_polygon_to_output_frame(output, main_polygon));
  for (const auto & sub_object : sub_objects) {
    append_union_polygon(
      transform_polygon_to_output_frame(output, autoware_utils_geometry::to_polygon2d(sub_object)));
  }
  return union_polygons;
}

/**
 * @brief Collect all outer boundary points from grouped union polygons.
 *
 * @param union_polygons Grouped union polygons expressed in the output local frame.
 * @return Outer boundary points across all grouped union polygons.
 */
MultiPoint2d collect_union_boundary_points(const MultiPolygon2d & union_polygons)
{
  MultiPoint2d boundary_points;
  for (const auto & polygon : union_polygons) {
    for (const auto & point : polygon.outer()) {
      boundary_points.push_back(
        Point2d(boost::geometry::get<0>(point), boost::geometry::get<1>(point)));
    }
  }
  return boundary_points;
}

/**
 * @brief Select the largest polygon from a grouped union result.
 *
 * @param union_polygons Grouped union polygons expressed in the output local frame.
 * @return Largest polygon by absolute area, or nullptr if empty.
 */
const Polygon2d * find_largest_union_polygon(const MultiPolygon2d & union_polygons)
{
  if (union_polygons.empty()) {
    return nullptr;
  }

  const Polygon2d * largest_polygon = &union_polygons.front();
  double largest_area = std::abs(boost::geometry::area(*largest_polygon));
  for (const auto & polygon : union_polygons) {
    const double polygon_area = std::abs(boost::geometry::area(polygon));
    if (polygon_area > largest_area) {
      largest_polygon = &polygon;
      largest_area = polygon_area;
    }
  }
  return largest_polygon;
}

/**
 * @brief Rebuild the output footprint from the grouped union polygons.
 *
 * @param output Output object to update in place.
 * @param union_polygons Grouped union polygons expressed in the output local frame.
 */
void fit_shape_footprint(DetectedObject & output, const MultiPolygon2d & union_polygons)
{
  const auto * union_polygon = find_largest_union_polygon(union_polygons);
  if (union_polygon == nullptr) {
    return;
  }

  output.shape.footprint.points.clear();
  for (const auto & point : union_polygon->outer()) {
    output.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>()
        .x(static_cast<float>(boost::geometry::get<0>(point)))
        .y(static_cast<float>(boost::geometry::get<1>(point)))
        .z(0.0f));
  }
}

/**
 * @brief Tighten a bounding box around the grouped union in the output local frame.
 *
 * @param output Output object to update in place.
 * @param combined_points Combined footprint points expressed in the output local frame.
 */
void fit_bounding_box_shape(DetectedObject & output, const MultiPoint2d & combined_points)
{
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & point : combined_points) {
    min_x = std::min(min_x, boost::geometry::get<0>(point));
    max_x = std::max(max_x, boost::geometry::get<0>(point));
    min_y = std::min(min_y, boost::geometry::get<1>(point));
    max_y = std::max(max_y, boost::geometry::get<1>(point));
  }

  output.kinematics.pose_with_covariance.pose = autoware_utils_geometry::calc_offset_pose(
    output.kinematics.pose_with_covariance.pose, 0.5 * (min_x + max_x), 0.5 * (min_y + max_y), 0.0);
  output.shape.dimensions.x = max_x - min_x;
  output.shape.dimensions.y = max_y - min_y;
}

/**
 * @brief Expand a cylinder to cover the farthest grouped union point in the output local frame.
 *
 * @param output Output object to update in place.
 * @param combined_points Combined footprint points expressed in the output local frame.
 */
void fit_cylinder_shape(DetectedObject & output, const MultiPoint2d & combined_points)
{
  double max_radius = 0.0;
  for (const auto & point : combined_points) {
    max_radius = std::max(
      max_radius, std::hypot(boost::geometry::get<0>(point), boost::geometry::get<1>(point)));
  }
  output.shape.dimensions.x = 2.0 * max_radius;
  output.shape.dimensions.y = 2.0 * max_radius;
}

/**
 * @brief Apply shape-specific footprint or dimension fitting for fused output.
 *
 * @param output Output object to update in place.
 * @param union_polygons Grouped union polygons expressed in the output local frame.
 * @param combined_points Combined footprint points expressed in the output local frame.
 * @param keep_input_dimensions Whether to preserve input dimensions and update only footprint.
 * @return True when the shape type is supported and was handled.
 */
bool fit_shape_by_type(
  DetectedObject & output, const MultiPolygon2d & union_polygons,
  const MultiPoint2d & combined_points, const bool keep_input_dimensions)
{
  if (output.shape.type == Shape::BOUNDING_BOX) {
    if (keep_input_dimensions) {
      fit_shape_footprint(output, union_polygons);
    } else {
      fit_bounding_box_shape(output, combined_points);
    }
    return true;
  }

  if (output.shape.type == Shape::CYLINDER) {
    if (keep_input_dimensions) {
      fit_shape_footprint(output, union_polygons);
    } else {
      fit_cylinder_shape(output, combined_points);
    }
    return true;
  }

  if (output.shape.type == Shape::POLYGON) {
    fit_shape_footprint(output, union_polygons);
    return true;
  }

  return false;
}

/**
 * @brief Calculate the 2D footprint intersection area between one main and one sub object.
 *
 * @param main_object Main object candidate.
 * @param sub_object Sub object candidate.
 * @return Overlapped footprint area in square meters.
 */
double get_intersection_area(const DetectedObject & main_object, const DetectedObject & sub_object)
{
  MultiPolygon2d intersections;
  boost::geometry::intersection(
    autoware_utils_geometry::to_polygon2d(main_object),
    autoware_utils_geometry::to_polygon2d(sub_object), intersections);

  double total_area = 0.0;
  for (const auto & polygon : intersections) {
    total_area += std::abs(boost::geometry::area(polygon));
  }
  return total_area;
}

/**
 * @brief Expand a main object so its shape encloses the main and grouped sub objects.
 *
 * @param main_object Main object that defines the output shape type and metadata.
 * @param sub_objects Sub objects uniquely grouped to the main object.
 * @return Main-based object whose geometry encloses the full grouped union.
 */
DetectedObject enclose_union_with_main_shape(
  const DetectedObject & main_object, const std::vector<DetectedObject> & sub_objects,
  const bool keep_input_dimensions)
{
  DetectedObject output = main_object;
  output.shape = main_object.shape;
  if (sub_objects.empty()) {
    return output;
  }
  const auto union_polygons =
    collect_union_polygons_in_output_frame(output, main_object, sub_objects);
  if (union_polygons.empty()) {
    return output;
  }
  const auto combined_points = collect_union_boundary_points(union_polygons);
  if (combined_points.empty()) {
    return output;
  }

  if (fit_shape_by_type(output, union_polygons, combined_points, keep_input_dimensions)) {
    fit_shape_height(output, main_object, sub_objects);
    return output;
  }

  return output;
}

}  // namespace

namespace autoware::object_merger
{
/**
 * @brief Construct the fusion node and initialize subscriptions, publishers, and debug utilities.
 *
 * @param node_options ROS node options used for component construction.
 */
ObjectFusionMergerNode::ObjectFusionMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("object_fusion_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  main_object_sub_(this, "input/main_objects", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sub_object_sub_(this, "input/sub_objects", rclcpp::QoS{1}.get_rmw_qos_profile())
{
  // Get parameters
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id");
  const auto sync_queue_size = static_cast<int>(declare_parameter<int64_t>("sync_queue_size"));
  keep_input_dimensions_ = declare_parameter<bool>("keep_input_dimensions", false);

  // Set up publishers, subscribers, and synchronizer
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ =
    std::make_shared<Sync>(SyncPolicy(sync_queue_size), main_object_sub_, sub_object_sub_);
  sync_ptr_->registerCallback(std::bind(&ObjectFusionMergerNode::callback, this, _1, _2));

  fused_objects_pub_ = create_publisher<DetectedObjects>("output/objects", rclcpp::QoS{1});
  other_objects_pub_ = create_publisher<DetectedObjects>("output/other_objects", rclcpp::QoS{1});

  processing_time_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

/**
 * @brief Transform both inputs, fuse overlapping objects, and publish the two output streams.
 *
 * @param main_objects_msg Main detected objects input message.
 * @param sub_objects_msg Sub detected objects input message.
 */
void ObjectFusionMergerNode::callback(
  const DetectedObjects::ConstSharedPtr & main_objects_msg,
  const DetectedObjects::ConstSharedPtr & sub_objects_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  if (
    fused_objects_pub_->get_subscription_count() < 1 &&
    other_objects_pub_->get_subscription_count() < 1) {
    return;
  }

  DetectedObjects transformed_main_objects;
  DetectedObjects transformed_sub_objects;
  if (
    !autoware::object_recognition_utils::transformObjects(
      *main_objects_msg, base_link_frame_id_, tf_buffer_, transformed_main_objects) ||
    !autoware::object_recognition_utils::transformObjects(
      *sub_objects_msg, base_link_frame_id_, tf_buffer_, transformed_sub_objects)) {
    RCLCPP_WARN(
      get_logger(), "Failed to transform objects to %s frame. Skipping fusion.",
      base_link_frame_id_.c_str());
    return;
  }

  const auto result = fuse_objects(transformed_main_objects, transformed_sub_objects);
  fused_objects_pub_->publish(result.fused_objects);
  other_objects_pub_->publish(result.other_objects);

  // Publish debug info
  published_time_publisher_->publish_if_subscribed(
    fused_objects_pub_, result.fused_objects.header.stamp);
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/cyclic_time_ms", stop_watch_ptr_->toc("cyclic_time", true));
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

/**
 * @brief Group sub objects by unique overlap and build fused and unmatched outputs.
 *
 * @param main_objects_msg Main detected objects already transformed into the base frame.
 * @param sub_objects_msg Sub detected objects already transformed into the base frame.
 * @return Fused main-based objects and unmatched sub objects for separate publication.
 */
ObjectFusionMergerNode::FusionResult ObjectFusionMergerNode::fuse_objects(
  const DetectedObjects & main_objects_msg, const DetectedObjects & sub_objects_msg)
{
  const auto & main_objects = main_objects_msg.objects;
  const auto & sub_objects = sub_objects_msg.objects;

  std::vector<std::vector<DetectedObject>> grouped_sub_objects(main_objects.size());
  std::vector<DetectedObject> other_objects;
  other_objects.reserve(sub_objects.size());

  for (const auto & sub_object : sub_objects) {
    std::vector<std::size_t> overlapped_main_indices;
    for (std::size_t main_index = 0; main_index < main_objects.size(); ++main_index) {
      if (get_intersection_area(main_objects.at(main_index), sub_object) > 1e-6) {
        overlapped_main_indices.push_back(main_index);
      }
    }

    // If sub-object does not overlap with any main object, treat it as a matched object
    if (overlapped_main_indices.empty()) {
      other_objects.push_back(sub_object);
      continue;
    }

    // If sub-object overlaps with a single main object, group it with that main object
    // If sub-object overlaps with multiple main objects, ignore it for fusion to avoid ambiguity
    if (overlapped_main_indices.size() == 1U) {
      grouped_sub_objects.at(overlapped_main_indices.front()).push_back(sub_object);
    }
  }

  std::vector<DetectedObject> matched_objects;
  matched_objects.reserve(main_objects.size());
  for (std::size_t main_index = 0; main_index < main_objects.size(); ++main_index) {
    const auto & main_object = main_objects.at(main_index);
    const auto & grouped_subs = grouped_sub_objects.at(main_index);
    if (grouped_subs.empty()) {
      matched_objects.push_back(main_object);
      continue;
    }
    matched_objects.push_back(
      enclose_union_with_main_shape(main_object, grouped_subs, keep_input_dimensions_));
  }

  const auto header = std_msgs::build<std_msgs::msg::Header>()
                        .stamp(main_objects_msg.header.stamp)
                        .frame_id(base_link_frame_id_);

  return FusionResult{
    autoware_perception_msgs::build<DetectedObjects>().header(header).objects(matched_objects),
    autoware_perception_msgs::build<DetectedObjects>().header(header).objects(other_objects)};
}

}  // namespace autoware::object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_merger::ObjectFusionMergerNode)
