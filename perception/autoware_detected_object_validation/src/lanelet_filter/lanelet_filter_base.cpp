// Copyright 2022 TIER IV, Inc.
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

#include "lanelet_filter_base.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
using TriangleMesh = std::vector<std::array<Eigen::Vector3d, 3>>;

template <typename ObjsMsgType, typename ObjMsgType>
ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::ObjectLaneletFilterBase(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN");
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR");
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK");
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS");
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER");
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE");
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE");
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN");

  // Set filter settings
  filter_settings_.lanelet_xy_overlap_filter =
    declare_parameter<bool>("filter_settings.lanelet_xy_overlap_filter.enabled");

  filter_settings_.lanelet_direction_filter =
    declare_parameter<bool>("filter_settings.lanelet_direction_filter.enabled");
  filter_settings_.lanelet_direction_filter_velocity_yaw_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.velocity_yaw_threshold");
  filter_settings_.lanelet_direction_filter_object_speed_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.object_speed_threshold");

  filter_settings_.lanelet_object_elevation_filter =
    declare_parameter<bool>("filter_settings.lanelet_object_elevation_filter.enabled");
  filter_settings_.max_elevation_threshold = declare_parameter<double>(
    "filter_settings.lanelet_object_elevation_filter.max_elevation_threshold");
  filter_settings_.min_elevation_threshold = declare_parameter<double>(
    "filter_settings.lanelet_object_elevation_filter.min_elevation_threshold");

  filter_settings_.lanelet_extra_margin =
    declare_parameter<double>("filter_settings.lanelet_extra_margin");
  filter_settings_.debug = declare_parameter<bool>("filter_settings.debug");

  if (filter_settings_.min_elevation_threshold > filter_settings_.max_elevation_threshold) {
    RCLCPP_WARN(
      this->get_logger(),
      "parameters of object_elevation_filter do not satisfy the relation: "
      "min_elevation_threshold (%f) <= max_elevation_threshold (%f)",
      filter_settings_.min_elevation_threshold, filter_settings_.max_elevation_threshold);
  }

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObjectLaneletFilterBase::mapCallback, this, _1));
  object_sub_ = this->create_subscription<ObjsMsgType>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectLaneletFilterBase::objectCallback, this, _1));
  object_pub_ = this->create_publisher<ObjsMsgType>("output/object", rclcpp::QoS{1});

  debug_publisher_ =
    std::make_unique<autoware_utils::DebugPublisher>(this, "object_lanelet_filter");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  if (filter_settings_.debug) {
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/marker", rclcpp::QoS{1});
  }
}

bool isInPolygon(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::BasicPolygon2d & polygon,
  const double radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(current_pose.position.x, current_pose.position.y);
  return boost::geometry::distance(p, polygon) < radius + eps;
}

bool isInPolygon(
  const double & x, const double & y, const lanelet::BasicPolygon2d & polygon, const double radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(x, y);
  return boost::geometry::distance(p, polygon) < radius + eps;
}

LinearRing2d expandPolygon(const LinearRing2d & polygon, double distance)
{
  autoware_utils::MultiPolygon2d multi_polygon;
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_circle circle_strategy;
  bg::strategy::buffer::side_straight side_strategy;

  bg::buffer(
    polygon, multi_polygon, distance_strategy, side_strategy, join_strategy, end_strategy,
    circle_strategy);

  if (multi_polygon.empty()) {
    return polygon;
  }

  return multi_polygon.front().outer();
}

TriangleMesh createTriangleMeshFromLanelet(const lanelet::ConstLanelet & lanelet)
{
  TriangleMesh mesh;

  const lanelet::ConstLineString3d & left = lanelet.leftBound();
  const lanelet::ConstLineString3d & right = lanelet.rightBound();

  // bounds should have same number of points
  // if not the same, first check the first shared points then we will check it from the tail
  // since we don't have the original set of 3 points,
  // we will approximate the 3d mesh with this way
  size_t n_left = left.size();
  size_t n_right = right.size();
  const size_t n_shared = std::min(n_left, n_right);
  if (n_shared < 2) return mesh;

  // take 2 points from each side and create 2 triangles
  for (size_t i = 0; i < n_shared - 1; ++i) {
    const Eigen::Vector3d a_l(left[i].x(), left[i].y(), left[i].z());
    const Eigen::Vector3d b_l(left[i + 1].x(), left[i + 1].y(), left[i + 1].z());
    const Eigen::Vector3d a_r(right[i].x(), right[i].y(), right[i].z());
    const Eigen::Vector3d b_r(right[i + 1].x(), right[i + 1].y(), right[i + 1].z());

    //
    // b_l .--. b_r
    //     |\ |
    //     | \|
    //     .--.
    // a_l      a_r
    //
    mesh.push_back({a_l, b_l, a_r});
    mesh.push_back({b_l, b_r, a_r});
  }

  // triangulation of the remaining unmatched parts from the tail
  if (n_left > n_right) {
    size_t i = n_left - 1;
    size_t j = n_right - 1;
    const size_t n_extra = n_left - n_right;

    for (size_t k = 0; k < n_extra; ++k) {
      // we need at least 2 points from each side
      if (i < 1 || j < 1) break;

      const Eigen::Vector3d a_l(left[i - 1].x(), left[i - 1].y(), left[i - 1].z());
      const Eigen::Vector3d b_l(left[i].x(), left[i].y(), left[i].z());
      const Eigen::Vector3d a_r(right[j - 1].x(), right[j - 1].y(), right[j - 1].z());
      const Eigen::Vector3d b_r(right[j].x(), right[j].y(), right[j].z());

      mesh.push_back({b_l, a_l, b_r});
      mesh.push_back({a_l, a_r, b_r});

      --i;
      --j;
    }
  } else if (n_right > n_left) {
    size_t i = n_left - 1;
    size_t j = n_right - 1;
    const size_t n_extra = n_right - n_left;

    for (size_t k = 0; k < n_extra; ++k) {
      if (i < 1 || j < 1) break;

      const Eigen::Vector3d a_l(left[i - 1].x(), left[i - 1].y(), left[i - 1].z());
      const Eigen::Vector3d b_l(left[i].x(), left[i].y(), left[i].z());
      const Eigen::Vector3d a_r(right[j - 1].x(), right[j - 1].y(), right[j - 1].z());
      const Eigen::Vector3d b_r(right[j].x(), right[j].y(), right[j].z());

      mesh.push_back({b_l, a_l, b_r});
      mesh.push_back({a_l, a_r, b_r});

      --i;
      --j;
    }
  }

  return mesh;
}

// compute a normal vector that is pointing the Z+ from given triangle points
Eigen::Vector3d computeFaceNormal(const std::array<Eigen::Vector3d, 3> & triangle_points)
{
  const Eigen::Vector3d v1 = triangle_points[1] - triangle_points[0];
  const Eigen::Vector3d v2 = triangle_points[2] - triangle_points[0];
  Eigen::Vector3d normal = v1.cross(v2);

  // ensure the normal is pointing upward (Z+)
  if (normal.z() < 0) {
    normal = -normal;
  }

  return normal.normalized();
}

// checks whether a point is located above the lanelet triangle plane
// that is closest in the perpendicular direction
bool isPointAboveLaneletMesh(
  const Eigen::Vector3d & point, const lanelet::ConstLanelet & lanelet, const double & offset,
  const double & min_distance, const double & max_distance)
{
  const TriangleMesh mesh = createTriangleMeshFromLanelet(lanelet);

  if (mesh.empty()) return true;

  // for the query point
  double query_point_min_abs_dist = std::numeric_limits<double>::infinity();
  // for top and bottom point
  double top_min_dist = std::numeric_limits<double>::infinity();
  double top_min_abs_dist = std::numeric_limits<double>::infinity();
  double bottom_min_dist = std::numeric_limits<double>::infinity();
  double bottom_min_abs_dist = std::numeric_limits<double>::infinity();

  // search the most nearest surface from the query point
  for (const auto & tri : mesh) {
    const Eigen::Vector3d plane_normal_vec = computeFaceNormal(tri);

    // std::cos(M_PI / 3.0) -> 0.5;
    // in some environment, or more recent c++, it can be constexpr
    constexpr double cos_threshold = 0.5;
    const double cos_of_normal_and_z = plane_normal_vec.dot(Eigen::Vector3d::UnitZ());

    // if angle is too steep, consider as above for safety
    if (cos_of_normal_and_z < cos_threshold) {
      return true;
    }

    Eigen::Vector3d vec_to_point = point - tri[0];
    double signed_dist = plane_normal_vec.dot(vec_to_point);

    double abs_dist = std::abs(signed_dist);
    if (abs_dist < query_point_min_abs_dist) {
      query_point_min_abs_dist = abs_dist;

      // check top side
      vec_to_point = (point + offset * plane_normal_vec) - tri[0];
      signed_dist = plane_normal_vec.dot(vec_to_point);

      abs_dist = std::abs(signed_dist);
      if (abs_dist < top_min_abs_dist) {
        top_min_dist = signed_dist;
        top_min_abs_dist = abs_dist;
      }

      // check bottom side
      vec_to_point = (point - offset * plane_normal_vec) - tri[0];
      signed_dist = plane_normal_vec.dot(vec_to_point);

      abs_dist = std::abs(signed_dist);
      if (abs_dist < bottom_min_abs_dist) {
        bottom_min_dist = signed_dist;
        bottom_min_abs_dist = abs_dist;
      }
    }
  }

  // if at least one point is within the range, we consider it to be in the range
  if (
    (min_distance <= top_min_dist && top_min_dist <= max_distance) ||
    (min_distance <= bottom_min_dist && bottom_min_dist <= max_distance))
    return true;
  else
    return false;
}

template <typename ObjsMsgType, typename ObjMsgType>
void ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg)
{
  lanelet_frame_id_ = map_msg->header.frame_id;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
}

template <typename ObjsMsgType, typename ObjMsgType>
void ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::objectCallback(
  const typename ObjsMsgType::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->tic("processing_time");

  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  ObjsMsgType output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "No vector map received.");
    return;
  }

  ObjsMsgType transformed_objects;
  if (!autoware::object_recognition_utils::transformObjects(
        *input_msg, lanelet_frame_id_, tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to %s.", lanelet_frame_id_.c_str());
    return;
  }

  if (!transformed_objects.objects.empty()) {
    // calculate convex hull
    const auto convex_hull = getConvexHull(transformed_objects);

    // get intersected lanelets
    std::vector<BoxAndLanelet> intersected_lanelets_with_bbox = getIntersectedLanelets(convex_hull);

    // create R-Tree with intersected_lanelets for fast search
    bgi::rtree<BoxAndLanelet, RtreeAlgo> local_rtree;
    for (const auto & bbox_and_lanelet : intersected_lanelets_with_bbox) {
      local_rtree.insert(bbox_and_lanelet);
    }

    if (filter_settings_.debug) {
      publishDebugMarkers(input_msg->header.stamp, convex_hull, intersected_lanelets_with_bbox);
    }
    // filtering process
    for (size_t index = 0; index < transformed_objects.objects.size(); ++index) {
      const auto & transformed_object = transformed_objects.objects.at(index);
      const auto & input_object = input_msg->objects.at(index);
      filterObject(transformed_object, input_object, local_rtree, output_object_msg);
    }
  }

  object_pub_->publish(output_object_msg);
  published_time_publisher_->publish_if_subscribed(object_pub_, output_object_msg.header.stamp);

  // Publish debug info
  const double pipeline_latency =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (this->get_clock()->now() - output_object_msg.header.stamp).nanoseconds()))
      .count();
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/pipeline_latency_ms", pipeline_latency);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::filterObject(
  const ObjMsgType & transformed_object, const ObjMsgType & input_object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree, ObjsMsgType & output_object_msg)
{
  const auto & label = transformed_object.classification.front().label;
  if (filter_target_.isTarget(label)) {
    // no tree, then no intersection
    if (local_rtree.empty()) {
      return false;
    }

    // create a 2D polygon from the object for querying
    Polygon2d object_polygon;
    if (utils::hasBoundingBox(transformed_object)) {
      const auto footprint = setFootprint(transformed_object);
      for (const auto & point : footprint.points) {
        const geometry_msgs::msg::Point32 point_transformed = autoware_utils::transform_point(
          point, transformed_object.kinematics.pose_with_covariance.pose);
        object_polygon.outer().emplace_back(point_transformed.x, point_transformed.y);
      }
      object_polygon.outer().push_back(object_polygon.outer().front());
    } else {
      object_polygon = getConvexHullFromObjectFootprint(transformed_object);
    }

    // create a bounding box from polygon for searching the local R-tree
    bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
    bg::envelope(object_polygon, bbox_of_convex_hull);
    std::vector<BoxAndLanelet> candidates;
    // only use the lanelets that intersect with the object's bounding box
    local_rtree.query(bgi::intersects(bbox_of_convex_hull), std::back_inserter(candidates));

    bool filter_pass = true;
    // 1. is polygon overlap with road lanelets or shoulder lanelets
    if (filter_settings_.lanelet_xy_overlap_filter) {
      filter_pass = isObjectOverlapLanelets(transformed_object, object_polygon, candidates);
    }

    // 2. check if objects velocity is the same with the lanelet direction
    const bool orientation_not_available =
      transformed_object.kinematics.orientation_availability ==
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    if (filter_settings_.lanelet_direction_filter && !orientation_not_available && filter_pass) {
      filter_pass = isSameDirectionWithLanelets(transformed_object, candidates);
    }

    // 3. check if the object is above the lanelets
    if (filter_settings_.lanelet_object_elevation_filter && filter_pass) {
      filter_pass = isObjectAboveLanelet(transformed_object, candidates);
    }

    // push back to output object
    if (filter_pass) {
      output_object_msg.objects.emplace_back(input_object);
      return true;
    }
  } else {
    output_object_msg.objects.emplace_back(input_object);
    return true;
  }
  return false;
}

template <typename ObjsMsgType, typename ObjMsgType>
geometry_msgs::msg::Polygon ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::setFootprint(
  const ObjMsgType & detected_object)
{
  geometry_msgs::msg::Polygon footprint;
  if (detected_object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto object_size = detected_object.shape.dimensions;
    const double x_front = object_size.x / 2.0;
    const double x_rear = -object_size.x / 2.0;
    const double y_left = object_size.y / 2.0;
    const double y_right = -object_size.y / 2.0;

    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_left).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_left).z(0.0));
  } else {
    footprint = detected_object.shape.footprint;
  }
  return footprint;
}

template <typename ObjsMsgType, typename ObjMsgType>
LinearRing2d ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getConvexHull(
  const ObjsMsgType & detected_objects)
{
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & pos = object.kinematics.pose_with_covariance.pose.position;
    const auto footprint = setFootprint(object);
    for (const auto & p : footprint.points) {
      candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
    }
  }
  LinearRing2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  if (filter_settings_.lanelet_extra_margin > 0) {
    return expandPolygon(convex_hull, filter_settings_.lanelet_extra_margin);
  }
  return convex_hull;
}

template <typename ObjsMsgType, typename ObjMsgType>
Polygon2d ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getConvexHullFromObjectFootprint(
  const ObjMsgType & object)
{
  MultiPoint2d candidate_points;
  const auto & pos = object.kinematics.pose_with_covariance.pose.position;
  const auto footprint = setFootprint(object);

  for (const auto & p : footprint.points) {
    candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
  }

  Polygon2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

// fetch the intersected candidate lanelets with bounding box and then
// check the intersections among the lanelets and the convex hull
template <typename ObjsMsgType, typename ObjMsgType>
std::vector<BoxAndLanelet> ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getIntersectedLanelets(
  const LinearRing2d & convex_hull)
{
  std::vector<BoxAndLanelet> intersected_lanelets_with_bbox;

  // convert convex_hull to a 2D bounding box for searching in the LaneletMap
  bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
  bg::envelope(convex_hull, bbox_of_convex_hull);
  const lanelet::BoundingBox2d bbox2d(
    lanelet::BasicPoint2d(
      bg::get<bg::min_corner, 0>(bbox_of_convex_hull),
      bg::get<bg::min_corner, 1>(bbox_of_convex_hull)),
    lanelet::BasicPoint2d(
      bg::get<bg::max_corner, 0>(bbox_of_convex_hull),
      bg::get<bg::max_corner, 1>(bbox_of_convex_hull)));

  const lanelet::Lanelets candidate_lanelets = lanelet_map_ptr_->laneletLayer.search(bbox2d);
  for (const auto & lanelet : candidate_lanelets) {
    // only check the road lanelets and road shoulder lanelets
    if (
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      (lanelet.attribute(lanelet::AttributeName::Subtype).value() ==
         lanelet::AttributeValueString::Road ||
       lanelet.attribute(lanelet::AttributeName::Subtype).value() == "road_shoulder")) {
      if (bg::intersects(convex_hull, lanelet.polygon2d().basicPolygon())) {
        // create bbox using boost for making the R-tree in later phase
        auto polygon = getPolygon(lanelet);
        Box boost_bbox;
        bg::envelope(polygon, boost_bbox);

        intersected_lanelets_with_bbox.emplace_back(
          std::make_pair(boost_bbox, PolygonAndLanelet{polygon, lanelet}));
      }
    }
  }

  return intersected_lanelets_with_bbox;
}

template <typename ObjsMsgType, typename ObjMsgType>
lanelet::BasicPolygon2d ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::getPolygon(
  const lanelet::ConstLanelet & lanelet)
{
  if (filter_settings_.lanelet_extra_margin <= 0) {
    return lanelet.polygon2d().basicPolygon();
  }

  auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
  Polygon2d polygon;
  bg::assign_points(polygon, lanelet_polygon);

  bg::correct(polygon);
  auto polygon_result = expandPolygon(polygon.outer(), filter_settings_.lanelet_extra_margin);
  lanelet::BasicPolygon2d result;

  bg::assign_points(result, polygon_result);

  return result;
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isObjectOverlapLanelets(
  const ObjMsgType & object, const Polygon2d & polygon,
  const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  // if object has bounding box, use polygon overlap
  if (utils::hasBoundingBox(object)) {
    return isPolygonOverlapLanelets(polygon, lanelet_candidates);
  } else {
    for (const auto & point : object.shape.footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware_utils::transform_point(point, object.kinematics.pose_with_covariance.pose);

      for (const auto & candidate : lanelet_candidates) {
        if (isInPolygon(point_transformed.x, point_transformed.y, candidate.second.polygon, 0.0)) {
          return true;
        }
      }
    }
    return false;
  }
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isPolygonOverlapLanelets(
  const Polygon2d & polygon, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  for (const auto & box_and_lanelet : lanelet_candidates) {
    if (!bg::disjoint(polygon, box_and_lanelet.second.polygon)) {
      return true;
    }
  }

  return false;
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isSameDirectionWithLanelets(
  const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double object_velocity_norm = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  const double object_velocity_yaw = std::atan2(
                                       object.kinematics.twist_with_covariance.twist.linear.y,
                                       object.kinematics.twist_with_covariance.twist.linear.x) +
                                     object_yaw;

  if (object_velocity_norm < filter_settings_.lanelet_direction_filter_object_speed_threshold) {
    return true;
  }

  for (const auto & box_and_lanelet : lanelet_candidates) {
    const bool is_in_lanelet =
      isInPolygon(object.kinematics.pose_with_covariance.pose, box_and_lanelet.second.polygon, 0.0);
    if (!is_in_lanelet) {
      continue;
    }

    const double lane_yaw = lanelet::utils::getLaneletAngle(
      box_and_lanelet.second.lanelet, object.kinematics.pose_with_covariance.pose.position);
    const double delta_yaw = object_velocity_yaw - lane_yaw;
    const double normalized_delta_yaw = autoware_utils::normalize_radian(delta_yaw);
    const double abs_norm_delta_yaw = std::fabs(normalized_delta_yaw);

    if (abs_norm_delta_yaw < filter_settings_.lanelet_direction_filter_velocity_yaw_threshold) {
      return true;
    }
  }

  return false;
}

template <typename ObjsMsgType, typename ObjMsgType>
bool ObjectLaneletFilterBase<ObjsMsgType, ObjMsgType>::isObjectAboveLanelet(
  const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates)
{
  // assuming the positions are already the center of the cluster (convex hull)
  // for an exact calculation of the center from the points,
  // we should use autoware_utils::transform_point before computing the cluster
  const double cx = object.kinematics.pose_with_covariance.pose.position.x;
  const double cy = object.kinematics.pose_with_covariance.pose.position.y;
  const double cz = object.kinematics.pose_with_covariance.pose.position.z;
  // use the centroid as a query point
  const Eigen::Vector3d centroid(cx, cy, cz);
  const double half_dim_z = object.shape.dimensions.z * 0.5;

  lanelet::ConstLanelet nearest_lanelet;
  double closest_lanelet_z_dist = std::numeric_limits<double>::infinity();

  // search for the nearest lanelet along the z-axis in case roads are layered
  for (const auto & candidate_lanelet : lanelet_candidates) {
    const lanelet::ConstLanelet llt = candidate_lanelet.second.lanelet;
    const lanelet::ConstLineString3d line = llt.leftBound();
    if (line.empty()) continue;

    // assuming the roads have enough height difference to distinguish each other
    const double diff_z = cz - line[0].z();
    const double dist_z = diff_z * diff_z;

    // use the closest lanelet in z axis
    if (dist_z < closest_lanelet_z_dist) {
      closest_lanelet_z_dist = dist_z;
      nearest_lanelet = llt;
    }
  }

  return isPointAboveLaneletMesh(
    centroid, nearest_lanelet, half_dim_z, filter_settings_.min_elevation_threshold,
    filter_settings_.max_elevation_threshold);
}

// explicit instantiation
template class ObjectLaneletFilterBase<
  autoware_perception_msgs::msg::DetectedObjects, autoware_perception_msgs::msg::DetectedObject>;
template class ObjectLaneletFilterBase<
  autoware_perception_msgs::msg::TrackedObjects, autoware_perception_msgs::msg::TrackedObject>;

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation
