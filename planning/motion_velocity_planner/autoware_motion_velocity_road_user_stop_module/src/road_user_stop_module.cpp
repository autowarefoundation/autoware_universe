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

#include "road_user_stop_module.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

void RoadUserStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  using std::placeholders::_1;

  module_name_ = module_name;
  logger_ = node.get_logger().get_child(module_name);
  clock_ = node.get_clock();

  params_ = RoadUserStopParameters(node, "road_user_stop");

  sub_path_with_lane_id_ =
    node.create_subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", rclcpp::QoS{1},
      std::bind(&RoadUserStopModule::onPathWithLaneIdSubscription, this, _1));

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "road_user_stop");

  debug_publisher_ = node.create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/road_user_stop/debug_markers", 1);

  processing_time_publisher_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/road_user_stop/processing_time_ms", 1);
}

void RoadUserStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  // TODO(odashima): implement parameter update logic
  (void)parameters;
}

void RoadUserStopModule::publish_planning_factor()
{
  planning_factor_interface_->publish();
}

void RoadUserStopModule::onPathWithLaneIdSubscription(
  const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg)
{
  path_with_lane_id_ = msg;
}

VelocityPlanningResult RoadUserStopModule::plan(
  const std::vector<TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic();

  VelocityPlanningResult result;

  debug_data_ = DebugData();

  const auto & trajectory_points = raw_trajectory_points;
  const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  if (trajectory_points.empty()) {
    RCLCPP_WARN(logger_, "Trajectory points are empty, skipping road user stop planning");
    publishProcessingTime(stop_watch.toc());
    return result;
  }

  if (!planner_data) {
    RCLCPP_WARN(logger_, "Planner data is null, skipping road user stop planning");
    publishProcessingTime(stop_watch.toc());
    return result;
  }

  if (!path_with_lane_id_) {
    RCLCPP_WARN(logger_, "Path with lane ID is not set, skipping road user stop planning");
    publishProcessingTime(stop_watch.toc());
    return result;
  }

  RCLCPP_DEBUG(
    logger_, "Starting road user stop planning with %zu trajectory points",
    trajectory_points.size());

  const auto current_time = clock_->now();

  // 1. update tracked objects
  PredictedObjects predicted_objects;
  predicted_objects.header = planner_data->predicted_objects_header;

  for (const auto & object_ptr : planner_data->objects) {
    if (object_ptr) {
      predicted_objects.objects.push_back(object_ptr->predicted_object);
    }
  }

  updateTrackedObjects(predicted_objects, current_time);

  // 2. get relevant lanelets
  const auto relevant_lanelets = getRelevantLanelets(planner_data, trajectory_points);
  debug_data_.relevant_lanelets = relevant_lanelets;

  if (relevant_lanelets.empty()) {
    RCLCPP_WARN(logger_, "No relevant lanelets found, skipping road user stop planning");

    // still publish debug markers even if no lanelets found
    if (params_.debug.publish_debug_markers && debug_publisher_) {
      const auto debug_markers = createDebugMarkerArray();
      debug_publisher_->publish(debug_markers);
    }

    publishProcessingTime(stop_watch.toc());
    return result;
  }

  RCLCPP_DEBUG(logger_, "Found %zu relevant lanelets", relevant_lanelets.size());

  // 3. filter each object and collect stop point candidates
  std::vector<StopPointCandidate> stop_point_candidates;

  for (const auto & object : predicted_objects.objects) {
    // check if target object type
    if (!isTargetObject(object)) {
      continue;
    }

    // check temporal filtering
    const auto object_id_str = autoware::universe_utils::toHexString(object.object_id);
    if (!hasMinimumDetectionDuration(object_id_str, current_time)) {
      continue;
    }

    // check if object is on road
    if (!isObjectOnRoad(object, lanelet_map_ptr, relevant_lanelets)) {
      continue;
    }

    // check if near crosswalk (exclude if configured)
    if (
      params_.detection.exclude_crosswalk_users &&
      isNearCrosswalk(
        object.kinematics.initial_pose_with_covariance.pose.position, lanelet_map_ptr)) {
      // skip crosswalk users
      continue;
    }

    // check if on sidewalk (exclude if configured)
    if (
      params_.detection.exclude_sidewalk_users &&
      isOnSidewalk(object.kinematics.initial_pose_with_covariance.pose.position, lanelet_map_ptr)) {
      continue;
    }

    // add to filtered objects for debug visualization
    debug_data_.filtered_objects.push_back(object);

    // check if wrong-way user
    bool is_wrong_way = false;
    if (params_.in_place_stop.enable) {
      for (const auto & lanelet : relevant_lanelets) {
        if (isWrongWayUser(object, lanelet)) {
          is_wrong_way = true;
          break;
        }
      }
    }

    // debugging for only enable isWrongWayUser
    // if (!is_wrong_way) {
    //   continue;
    // }

    // create stop point candidate
    const auto candidate_opt =
      createStopPointCandidate(trajectory_points, object, is_wrong_way, planner_data);

    if (candidate_opt) {
      stop_point_candidates.push_back(candidate_opt.value());
    }
  }

  // 4. sort candidates by stop distance and select the best one
  if (!stop_point_candidates.empty()) {
    std::sort(
      stop_point_candidates.begin(), stop_point_candidates.end(),
      [](const StopPointCandidate & a, const StopPointCandidate & b) {
        // prioritize wrong-way users
        if (a.is_wrong_way != b.is_wrong_way) {
          return a.is_wrong_way;
        }
        return a.stop_distance < b.stop_distance;
      });

    // find the first candidate with acceptable deceleration
    for (const auto & candidate : stop_point_candidates) {
      // Check if deceleration is acceptable
      if (std::abs(candidate.required_deceleration) <= params_.stop_decision.max_deceleration) {
        // apply stop
        result.stop_points.push_back(candidate.stop_position);

        // update debug data
        debug_data_.stop_index = candidate.stop_index;
        debug_data_.stop_point = candidate.stop_position;
        debug_data_.stop_target_object = candidate.target_object;

        // add planning factor
        addPlanningFactor(trajectory_points, candidate, planner_data);

        // Stop at the first valid candidate
        break;
      }
    }
  }

  // 5. publish debug markers if enabled
  if (params_.debug.publish_debug_markers && debug_publisher_) {
    const auto debug_markers = createDebugMarkerArray();
    debug_publisher_->publish(debug_markers);
  }

  // publish processing time
  publishProcessingTime(stop_watch.toc());

  return result;
}

bool RoadUserStopModule::isTargetObject(const PredictedObject & object) const
{
  const auto & classification = object.classification.front();

  switch (classification.label) {
    case ObjectClassification::PEDESTRIAN:
      return params_.detection.target_object_types.check_pedestrian;
    case ObjectClassification::BICYCLE:
      return params_.detection.target_object_types.check_bicycle;
    case ObjectClassification::MOTORCYCLE:
      return params_.detection.target_object_types.check_motorcycle;
    case ObjectClassification::UNKNOWN:
      return params_.detection.target_object_types.check_unknown;
    default:
      return false;
  }
}

bool RoadUserStopModule::isObjectOnRoad(
  const PredictedObject & object, const lanelet::LaneletMapPtr & /* lanelet_map */,
  const lanelet::ConstLanelets & relevant_lanelets) const
{
  const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;

  // check if object is within any relevant lanelet
  for (const auto & lanelet : relevant_lanelets) {
    // create a pose for the lanelet check
    geometry_msgs::msg::Pose pose;
    pose.position = position;
    pose.orientation.w = 1.0;  // default orientation

    if (lanelet::utils::isInLanelet(pose, lanelet, 0.0)) {
      return true;
    }
  }

  return false;
}

bool RoadUserStopModule::isNearCrosswalk(
  const geometry_msgs::msg::Point & position, const lanelet::LaneletMapPtr & lanelet_map) const
{
  const lanelet::Point3d search_point(position.x, position.y, position.z);

  // search for crosswalk lanelets
  const auto crosswalk_lanelets = lanelet_map->laneletLayer.search(lanelet::BoundingBox2d(
    lanelet::Point2d(
      position.x - params_.detection.crosswalk_margin,
      position.y - params_.detection.crosswalk_margin),
    lanelet::Point2d(
      position.x + params_.detection.crosswalk_margin,
      position.y + params_.detection.crosswalk_margin)));

  for (const auto & lanelet : crosswalk_lanelets) {
    if (
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      lanelet.attribute(lanelet::AttributeName::Subtype) ==
        lanelet::AttributeValueString::Crosswalk) {
      return true;
    }
  }

  return false;
}

bool RoadUserStopModule::isOnSidewalk(
  const geometry_msgs::msg::Point & position, const lanelet::LaneletMapPtr & lanelet_map) const
{
  const lanelet::Point3d search_point(position.x, position.y, position.z);

  // search for sidewalk lanelets
  const auto nearby_lanelets = lanelet_map->laneletLayer.nearest(search_point, 1);

  if (!nearby_lanelets.empty()) {
    const auto & nearest_lanelet = nearby_lanelets.front();
    if (
      nearest_lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      nearest_lanelet.attribute(lanelet::AttributeName::Subtype) == "walkway") {
      return true;
    }
  }

  return false;
}

bool RoadUserStopModule::isWrongWayUser(
  const PredictedObject & object, const lanelet::ConstLanelet & lanelet) const
{
  // get object velocity
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const double object_speed = std::hypot(twist.linear.x, twist.linear.y);

  // skip if object is stopped or moving too slowly
  if (object_speed < params_.in_place_stop.min_speed_threshold) {
    return false;
  }

  // get object heading
  const double object_yaw =
    tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation);

  // get lanelet direction at object position
  const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;
  const lanelet::Point3d object_point(position.x, position.y, position.z);

  // find closest point on centerline
  const auto & centerline = lanelet.centerline();
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < centerline.size(); ++i) {
    const double dist = lanelet::geometry::distance2d(object_point, centerline[i]);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // calculate lanelet direction
  double lanelet_yaw = 0.0;
  if (closest_idx < centerline.size() - 1) {
    const auto & p1 = centerline[closest_idx];
    const auto & p2 = centerline[closest_idx + 1];
    lanelet_yaw = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
  }

  // calculate angle difference
  const double angle_diff =
    std::abs(autoware::universe_utils::normalizeRadian(object_yaw - lanelet_yaw)) * 180.0 / M_PI;

  const bool is_wrong_way_user = angle_diff > params_.in_place_stop.wrong_way_angle_threshold;

  return is_wrong_way_user;
}

lanelet::ConstLanelets RoadUserStopModule::getRelevantLanelets(
  const std::shared_ptr<const PlannerData> planner_data,
  [[maybe_unused]] const std::vector<TrajectoryPoint> & trajectory_points) const
{
  lanelet::ConstLanelets relevant_lanelets;

  if (!planner_data->route_handler) {
    RCLCPP_WARN(logger_, "Route handler is null, cannot get relevant lanelets");
    return relevant_lanelets;
  }

  const auto & route_handler = planner_data->route_handler;
  const auto lanelet_map = route_handler->getLaneletMapPtr();

  if (!lanelet_map) {
    RCLCPP_WARN(logger_, "Lanelet map is null");
    return relevant_lanelets;
  }

  // extract lane IDs from path_with_lane_id_
  std::set<lanelet::Id> unique_lane_ids;  // Use set to avoid duplicates

  if (path_with_lane_id_ && !path_with_lane_id_->points.empty()) {
    // calculate path length and extract unique lane IDs up to search_length
    double accumulated_length = 0.0;

    for (size_t i = 0; i < path_with_lane_id_->points.size(); ++i) {
      if (i > 0) {
        const auto & prev_point = path_with_lane_id_->points[i - 1].point.pose.position;
        const auto & curr_point = path_with_lane_id_->points[i].point.pose.position;
        accumulated_length += autoware::universe_utils::calcDistance3d(prev_point, curr_point);

        if (accumulated_length > params_.detection.search_length) {
          break;
        }
      }

      for (const auto lane_id : path_with_lane_id_->points[i].lane_ids) {
        unique_lane_ids.insert(lane_id);
      }
    }

    // convert lane IDs to lanelets
    for (const auto lane_id : unique_lane_ids) {
      try {
        const auto lanelet = lanelet_map->laneletLayer.get(lane_id);
        relevant_lanelets.push_back(lanelet);
      } catch (const lanelet::NoSuchPrimitiveError & e) {
        RCLCPP_WARN(logger_, "Could not find lanelet with ID %ld: %s", lane_id, e.what());
      }
    }
  } else {
    RCLCPP_WARN(logger_, "path_with_lane_id_ is not available or empty");
    return relevant_lanelets;
  }

  // store ego lanelets for debug visualization (before adding adjacent lanelets)
  debug_data_.ego_lanelets = relevant_lanelets;

  if (params_.detection.adjacent_lane_check) {
    // add adjacent lanelets
    lanelet::ConstLanelets adjacent_lanelets;
    debug_data_.adjacent_lanelets.clear();

    // create a copy to iterate over, as we'll be modifying relevant_lanelets
    auto current_lanelets = relevant_lanelets;

    for (const auto & lanelet : current_lanelets) {
      // get lanelet (including shoulder)
      const auto left_lanelet = route_handler->getLeftLanelet(
        lanelet, false /*enable_same_root*/, false /*get_shoulder_lane*/);
      if (left_lanelet) {
        adjacent_lanelets.push_back(*left_lanelet);
      }

      const auto right_lanelet = route_handler->getRightLanelet(
        lanelet, false /*enable_same_root*/, false /*get_shoulder_lane*/);
      if (right_lanelet) {
        adjacent_lanelets.push_back(*right_lanelet);
      }

      // also check for shoulder lanes specifically
      const auto left_shoulder = route_handler->getLeftShoulderLanelet(lanelet);
      if (left_shoulder) {
        adjacent_lanelets.push_back(*left_shoulder);
      }

      const auto right_shoulder = route_handler->getRightShoulderLanelet(lanelet);
      if (right_shoulder) {
        adjacent_lanelets.push_back(*right_shoulder);
      }
    }

    // store adjacent lanelets for debug
    debug_data_.adjacent_lanelets = adjacent_lanelets;

    // create trajectory polygons with lateral margin
    const auto & vehicle_info = planner_data->vehicle_info_;
    const double margin_threshold =
      vehicle_info.vehicle_width_m / 2.0 + params_.detection.adjacent_lane_margin;

    // decimate trajectory points for polygon creation
    const double decimate_step_length = 1.0;  // 1m interval
    std::vector<TrajectoryPoint> decimated_traj_points;
    double accumulated_length = 0.0;
    double total_accumulated_length = 0.0;
    decimated_traj_points.push_back(trajectory_points.front());

    for (size_t i = 1; i < trajectory_points.size(); ++i) {
      const double dist = autoware::universe_utils::calcDistance2d(
        trajectory_points[i - 1].pose.position, trajectory_points[i].pose.position);
      accumulated_length += dist;
      total_accumulated_length += dist;

      if (accumulated_length >= decimate_step_length) {
        decimated_traj_points.push_back(trajectory_points[i]);
        accumulated_length = 0.0;
      }
      if (total_accumulated_length >= params_.detection.search_length) {
        break;
      }
    }

    // create trajectory polygons with margin
    const auto traj_polygons = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, planner_data->current_odometry.pose.pose,
      margin_threshold, false /* enable_to_consider_current_pose */, 0.0 /* time_to_convergence*/,
      decimate_step_length);

    // merge all trajectory polygons into one
    autoware_utils_geometry::Polygon2d merged_traj_polygon;
    if (!traj_polygons.empty()) {
      merged_traj_polygon = traj_polygons.front();
      for (size_t i = 1; i < traj_polygons.size(); ++i) {
        std::vector<autoware_utils_geometry::Polygon2d> union_results;
        boost::geometry::union_(merged_traj_polygon, traj_polygons[i], union_results);
        if (!union_results.empty()) {
          merged_traj_polygon = union_results.front();
        }
      }
    }
    debug_data_.trajectory_polygons = {merged_traj_polygon};

    // mask adjacent lanelets by trajectory margin
    lanelet::ConstLanelets masked_adjacent_lanelets;
    debug_data_.masked_adjacent_polygons.clear();

    for (const auto & adj_llt : adjacent_lanelets) {
      // convert lanelet to polygon
      autoware_utils_geometry::Polygon2d lanelet_poly;
      for (const auto & pt : adj_llt.polygon3d()) {
        lanelet_poly.outer().emplace_back(pt.x(), pt.y());
      }
      if (!lanelet_poly.outer().empty()) {
        lanelet_poly.outer().push_back(lanelet_poly.outer().front());  // close polygon
      }
      boost::geometry::correct(lanelet_poly);

      // check intersection with merged trajectory polygon
      bool intersects_with_traj = false;
      if (!merged_traj_polygon.outer().empty()) {
        std::vector<autoware_utils_geometry::Polygon2d> intersection_results;
        boost::geometry::intersection(lanelet_poly, merged_traj_polygon, intersection_results);

        if (!intersection_results.empty()) {
          intersects_with_traj = true;
          // add intersection polygons to debug data
          for (const auto & intersection_poly : intersection_results) {
            if (!intersection_poly.outer().empty()) {
              debug_data_.masked_adjacent_polygons.push_back(intersection_poly);
            }
          }
        }
      }

      if (intersects_with_traj) {
        masked_adjacent_lanelets.push_back(adj_llt);
      }
    }

    // store masked adjacent lanelets for debug
    debug_data_.masked_adjacent_lanelets = masked_adjacent_lanelets;

    // add unique masked adjacent lanelets to relevant lanelets
    for (const auto & adj_llt : masked_adjacent_lanelets) {
      if (
        std::find(relevant_lanelets.begin(), relevant_lanelets.end(), adj_llt) ==
        relevant_lanelets.end()) {
        relevant_lanelets.push_back(adj_llt);
      }
    }
  }

  RCLCPP_DEBUG(logger_, "Found %zu relevant lanelets from path lane IDs", relevant_lanelets.size());

  return relevant_lanelets;
}

std::optional<size_t> RoadUserStopModule::calculateStopPointWithMargin(
  const std::vector<TrajectoryPoint> & trajectory_points, const PredictedObject & object,
  const VehicleInfo & vehicle_info) const
{
  if (trajectory_points.empty()) {
    return std::nullopt;
  }

  const auto & object_position = object.kinematics.initial_pose_with_covariance.pose.position;

  // find closest point on trajectory to object
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < trajectory_points.size(); ++i) {
    const double distance =
      autoware::universe_utils::calcDistance2d(trajectory_points[i].pose.position, object_position);

    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }

  // search backward from closest point to find stop position
  // Calculate total stop margin: user-defined margin + vehicle front offset
  const double stop_margin_from_bumper = params_.stop_decision.stop_margin;
  const double stop_margin_from_base_link =
    stop_margin_from_bumper + vehicle_info.max_longitudinal_offset_m;

  RCLCPP_DEBUG(
    logger_,
    "Stop margin calculation: bumper_margin=%.2fm, vehicle_front_offset=%.2fm, total=%.2fm",
    stop_margin_from_bumper, vehicle_info.max_longitudinal_offset_m, stop_margin_from_base_link);

  for (int i = static_cast<int>(closest_idx) - 1; i >= 0; --i) {
    const double dist_to_object =
      autoware::motion_utils::calcSignedArcLength(trajectory_points, i, closest_idx);

    if (dist_to_object >= stop_margin_from_base_link) {
      return i;
    }
  }

  return 0;  // Stop at beginning if cannot maintain margin
}

double RoadUserStopModule::calculateRequiredDeceleration(
  const double current_velocity, const double stop_distance) const
{
  if (stop_distance <= 0.0) {
    return params_.stop_decision.max_deceleration;
  }

  return (current_velocity * current_velocity) / (2.0 * stop_distance);
}

std::optional<size_t> RoadUserStopModule::calculateGradualStopPoint(
  const std::vector<TrajectoryPoint> & trajectory_points, const double current_velocity,
  const geometry_msgs::msg::Point & current_position) const
{
  if (trajectory_points.empty()) {
    return std::nullopt;
  }

  const double vel = std::max(current_velocity, 0.0);

  // calculate stopping distance using configured deceleration
  const double deceleration = params_.in_place_stop.deceleration;
  const double stopping_distance = (vel * vel) / (2.0 * deceleration);

  // add vehicle front offset to stopping distance
  const double total_stopping_distance = stopping_distance;
  // find the point on trajectory at stopping distance
  size_t stop_idx = trajectory_points.size() - 1;

  for (size_t i = 0; i < trajectory_points.size(); ++i) {
    const double distance_from_current = autoware::motion_utils::calcSignedArcLength(
      trajectory_points, current_position, trajectory_points[i].pose.position);

    if (distance_from_current >= total_stopping_distance) {
      stop_idx = i;
      break;
    }
  }

  return stop_idx;
}

std::optional<StopPointCandidate> RoadUserStopModule::createStopPointCandidate(
  const std::vector<TrajectoryPoint> & trajectory_points, const PredictedObject & object,
  const bool is_wrong_way, const std::shared_ptr<const PlannerData> planner_data) const
{
  StopPointCandidate candidate;
  candidate.target_object = object;
  candidate.is_wrong_way = is_wrong_way;

  if (is_wrong_way) {
    // calculate current gradual stop point
    const auto & current_pos = planner_data->current_odometry.pose.pose.position;
    const double current_velocity = planner_data->current_odometry.twist.twist.linear.x;

    const auto gradual_stop_index =
      calculateGradualStopPoint(trajectory_points, current_velocity, current_pos);

    if (!gradual_stop_index) {
      return std::nullopt;
    }

    const auto gradual_stop_position = trajectory_points[*gradual_stop_index].pose.position;

    // add to debug data (mutable access needed)
    const_cast<RoadUserStopModule *>(this)->debug_data_.gradual_stop_positions.push_back(
      gradual_stop_position);

    const double new_stop_distance = autoware::motion_utils::calcSignedArcLength(
      trajectory_points, current_pos, gradual_stop_position);

    if (new_stop_distance < 0.0) {
      RCLCPP_WARN(
        logger_, "Calculated gradual stop point is behind the current position, skipping object %s",
        autoware::universe_utils::toHexString(object.object_id).c_str());
      return std::nullopt;
    }

    candidate.stop_index = *gradual_stop_index;
    candidate.stop_position = gradual_stop_position;
    candidate.stop_distance = new_stop_distance;
    candidate.required_deceleration = params_.in_place_stop.deceleration;
  } else {
    // normal stop point calculation for non-wrong-way users
    const auto stop_index =
      calculateStopPointWithMargin(trajectory_points, object, planner_data->vehicle_info_);

    if (!stop_index) {
      return std::nullopt;
    }

    const double stop_distance = autoware::motion_utils::calcSignedArcLength(
      trajectory_points, planner_data->current_odometry.pose.pose.position,
      trajectory_points[*stop_index].pose.position);

    const double current_velocity = planner_data->current_odometry.twist.twist.linear.x;
    const double required_decel = calculateRequiredDeceleration(current_velocity, stop_distance);

    candidate.stop_index = *stop_index;
    candidate.stop_position = trajectory_points[*stop_index].pose.position;
    candidate.stop_distance = stop_distance;
    candidate.required_deceleration = required_decel;
  }

  return candidate;
}

void RoadUserStopModule::updateTrackedObjects(
  const PredictedObjects & objects, const rclcpp::Time & current_time)
{
  // remove old tracked objects
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if ((current_time - it->second.last_detection_time).seconds() > 1.0) {
      it = tracked_objects_.erase(it);
    } else {
      ++it;
    }
  }

  // update or add tracked objects
  for (const auto & object : objects.objects) {
    const auto object_id_str = autoware::universe_utils::toHexString(object.object_id);

    if (tracked_objects_.find(object_id_str) != tracked_objects_.end()) {
      // update existing
      tracked_objects_[object_id_str].last_detection_time = current_time;
      tracked_objects_[object_id_str].last_position =
        object.kinematics.initial_pose_with_covariance.pose.position;
    } else {
      // add new
      TrackedObject tracked;
      tracked.object_id = object.object_id;
      tracked.first_detection_time = current_time;
      tracked.last_detection_time = current_time;
      tracked.last_position = object.kinematics.initial_pose_with_covariance.pose.position;
      tracked_objects_[object_id_str] = tracked;
    }
  }
}

bool RoadUserStopModule::hasMinimumDetectionDuration(
  const std::string & object_id, const rclcpp::Time & current_time) const
{
  const auto it = tracked_objects_.find(object_id);
  if (it == tracked_objects_.end()) {
    return false;
  }

  const double duration = (current_time - it->second.first_detection_time).seconds();
  return duration >= params_.detection.min_detection_duration;
}

void RoadUserStopModule::publishProcessingTime(const double processing_time_ms) const
{
  if (processing_time_publisher_ && processing_time_publisher_->get_subscription_count() > 0) {
    autoware_internal_debug_msgs::msg::Float64Stamped msg;
    msg.stamp = clock_->now();
    msg.data = processing_time_ms;
    processing_time_publisher_->publish(msg);

    // Log warning if processing time exceeds 10ms threshold
    if (processing_time_ms > 10.0) {
      RCLCPP_WARN(logger_, "Processing time %.2f ms exceeds 10ms threshold", processing_time_ms);
    }
  }
}

void RoadUserStopModule::addPlanningFactor(
  const std::vector<TrajectoryPoint> & trajectory_points, const StopPointCandidate & candidate,
  const std::shared_ptr<const PlannerData> planner_data) const
{
  geometry_msgs::msg::Pose stop_pose;
  stop_pose.position = candidate.stop_position;
  stop_pose.orientation = trajectory_points[candidate.stop_index].pose.orientation;

  using SafetyFactorArray = autoware_internal_planning_msgs::msg::SafetyFactorArray;
  using SafetyFactor = autoware_internal_planning_msgs::msg::SafetyFactor;
  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = clock_->now();
  safety_factors.header.frame_id = "map";
  safety_factors.is_safe = false;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = candidate.target_object.object_id;
  safety_factor.points.push_back(
    candidate.target_object.kinematics.initial_pose_with_covariance.pose.position);

  safety_factors.factors.push_back(safety_factor);

  const std::string detail = candidate.is_wrong_way ? "wrong-way user" : "";

  planning_factor_interface_->add(
    trajectory_points, planner_data->current_odometry.pose.pose, stop_pose,
    autoware::planning_factor_interface::PlanningFactor::STOP, safety_factors, true,
    0.0 /* velocity */, 0.0 /* shift_length */, detail);
}

}  // namespace autoware::motion_velocity_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::RoadUserStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
