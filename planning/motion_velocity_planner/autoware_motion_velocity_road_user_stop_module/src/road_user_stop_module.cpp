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

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

namespace
{
std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

autoware_utils_geometry::Point2d convertPoint(const geometry_msgs::msg::Point & p)
{
  return autoware_utils_geometry::Point2d{p.x, p.y};
}

double calcMinimumDistanceToStop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}
}  // namespace

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

  // processing_time_publisher_ =
  //   node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
  //     "~/debug/road_user_stop/processing_time_ms", 1);

  processing_time_detail_pub_ = node.create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/road_user_stop", 1);
  // time keeper
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(processing_time_detail_pub_);
}

void RoadUserStopModule::updateTrackedObjects(
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const rclcpp::Time & current_time)
{
  // Clean up old tracked objects that haven't been seen for a while
  // TODO(odashima): parametrize the cleanup threshold
  const double cleanup_threshold = 2.0;  // seconds
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if ((current_time - it->second.last_detected_time).seconds() > cleanup_threshold) {
      it = tracked_objects_.erase(it);
    } else {
      ++it;
    }
  }

  // Update or add tracked objects
  for (const auto & object_ptr : objects) {
    if (!object_ptr) continue;

    const auto & predicted_object = object_ptr->predicted_object;
    const std::string object_id = autoware::universe_utils::toHexString(predicted_object.object_id);

    auto it = tracked_objects_.find(object_id);
    if (it != tracked_objects_.end()) {
      // Update existing tracked object
      it->second.last_detected_time = current_time;
      it->second.updateClassification(predicted_object.classification.front().label);
    } else {
      // Add new tracked object
      TrackedObject tracked_obj;
      tracked_obj.object_id = object_id;
      tracked_obj.first_detected_time = current_time;
      tracked_obj.last_detected_time = current_time;
      tracked_obj.updateClassification(predicted_object.classification.front().label);
      tracked_objects_[object_id] = tracked_obj;
    }
  }
}

bool RoadUserStopModule::hasMinimumDetectionDuration(
  const std::string & object_id, const rclcpp::Time & current_time) const
{
  auto it = tracked_objects_.find(object_id);
  if (it == tracked_objects_.end()) {
    return false;
  }

  const double detection_duration = (current_time - it->second.first_detected_time).seconds();
  return detection_duration >= params_.obstacle_filtering.min_detection_duration;
}

std::vector<StopObstacle> RoadUserStopModule::filterStopObstacles(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polygons,
  const lanelet::ConstLanelets & lanelets_for_vru,
  const lanelet::ConstLanelets & lanelets_for_wrongway_user, const rclcpp::Time & current_time,
  const double dist_to_bumper)
{
  updateTrackedObjects(planner_data->objects, current_time);

  std::vector<StopObstacle> stop_obstacles;
  for (const auto & object : planner_data->objects) {
    if (
      auto stop_obstacle = pickStopObstacleFromPredictedObject(
        planner_data, object, traj_points, decimated_traj_points, decimated_traj_polygons,
        lanelets_for_vru, lanelets_for_wrongway_user, current_time, dist_to_bumper)) {
      stop_obstacles.push_back(*stop_obstacle);
    }
  }

  return stop_obstacles;
}

void RoadUserStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  // Option parameters
  update_param(
    parameters, "road_user_stop.option.suppress_sudden_stop", params_.option.suppress_sudden_stop);

  // Stop planning parameters
  update_param(
    parameters, "road_user_stop.stop_planning.stop_margin", params_.stop_planning.stop_margin);
  update_param(
    parameters, "road_user_stop.stop_planning.terminal_stop_margin",
    params_.stop_planning.terminal_stop_margin);
  update_param(
    parameters, "road_user_stop.stop_planning.min_behavior_stop_margin",
    params_.stop_planning.min_behavior_stop_margin);

  update_param(
    parameters, "road_user_stop.stop_planning.hold_stop_velocity_threshold",
    params_.stop_planning.hold_stop_velocity_threshold);
  update_param(
    parameters, "road_user_stop.stop_planning.hold_stop_distance_threshold",
    params_.stop_planning.hold_stop_distance_threshold);

  // Stop on curve parameters
  update_param(
    parameters, "road_user_stop.stop_planning.stop_on_curve.enable_approaching",
    params_.stop_planning.stop_on_curve.enable_approaching);
  update_param(
    parameters, "road_user_stop.stop_planning.stop_on_curve.additional_stop_margin",
    params_.stop_planning.stop_on_curve.additional_stop_margin);
  update_param(
    parameters, "road_user_stop.stop_planning.stop_on_curve.min_stop_margin",
    params_.stop_planning.stop_on_curve.min_stop_margin);

  // Common parameters for all object types
  update_param(
    parameters, "road_user_stop.stop_planning.limit_min_acc", params_.stop_planning.limit_min_acc);
  update_param(
    parameters, "road_user_stop.stop_planning.sudden_object_acc_threshold",
    params_.stop_planning.sudden_object_acc_threshold);
  update_param(
    parameters, "road_user_stop.stop_planning.sudden_object_dist_threshold",
    params_.stop_planning.sudden_object_dist_threshold);
  update_param(
    parameters, "road_user_stop.stop_planning.abandon_to_stop",
    params_.stop_planning.abandon_to_stop);

  // Obstacle filtering parameters
  update_param(
    parameters, "road_user_stop.obstacle_filtering.adjacent_lane_margin",
    params_.obstacle_filtering.adjacent_lane_margin);
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
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // Initialize
  VelocityPlanningResult result;
  debug_data_ = DebugData();
  trajectory_polygon_for_inside_map_.clear();

  // Check prerequisites
  if (!path_with_lane_id_) {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      logger_, *clock_, 5000, "Path with lane ID is not set, skipping road user stop planning");
    return result;
  }

  // Extract parameters and current state
  const auto & trajectory_points = raw_trajectory_points;
  const auto current_time = clock_->now();
  const double dist_to_bumper = planner_data->vehicle_info_.max_longitudinal_offset_m;

  // 1. Prepare trajectory data for collision checking
  // 1.1 Decimate trajectory points to reduce computational cost
  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    trajectory_points, planner_data->current_odometry.pose.pose,
    planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold,
    planner_data->trajectory_polygon_collision_check.decimate_trajectory_step_length,
    params_.stop_planning.stop_margin);

  // 1.2 Create trajectory polygons for collision detection
  const auto & p = planner_data->trajectory_polygon_collision_check;
  const auto decimated_traj_polygons = getTrajectoryPolygons(
    decimated_traj_points, planner_data->vehicle_info_, planner_data->current_odometry.pose.pose,
    params_.obstacle_filtering.adjacent_lane_margin, p.enable_to_consider_current_pose,
    p.time_to_convergence, p.decimate_trajectory_step_length);
  debug_data_.trajectory_polygons = decimated_traj_polygons;

  // 2. Get relevant lanelets for VRU and wrong-way detection
  const auto [lanelets_for_vru, lanelets_for_wrongway_user] = getRelevantLanelets(planner_data);
  debug_data_.relevant_lanelets = lanelets_for_vru;
  RCLCPP_DEBUG(
    logger_, "Found %zu lanelets for VRU, %zu for wrongway", lanelets_for_vru.size(),
    lanelets_for_wrongway_user.size());

  // 3. Filter objects and create stop obstacles
  const auto stop_obstacles = filterStopObstacles(
    planner_data, trajectory_points, decimated_traj_points, decimated_traj_polygons,
    lanelets_for_vru, lanelets_for_wrongway_user, current_time, dist_to_bumper);

  // 4. Calculate stop point based on filtered obstacles
  const auto stop_point = planStop(planner_data, trajectory_points, stop_obstacles, dist_to_bumper);
  if (stop_point) {
    result.stop_points.push_back(*stop_point);
  }

  // 5. Publish debug information for visualization
  const auto debug_markers = createDebugMarkerArray();
  debug_publisher_->publish(debug_markers);

  return result;
}

bool RoadUserStopModule::isTargetObject(const uint8_t label) const
{
  switch (label) {
    case ObjectClassification::PEDESTRIAN:
      return params_.obstacle_filtering.object_type.target_types.pedestrian;
    case ObjectClassification::BICYCLE:
      return params_.obstacle_filtering.object_type.target_types.bicycle;
    case ObjectClassification::MOTORCYCLE:
      return params_.obstacle_filtering.object_type.target_types.motorcycle;
    case ObjectClassification::UNKNOWN:
      return params_.obstacle_filtering.object_type.target_types.unknown;
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
      position.x - params_.obstacle_filtering.crosswalk_margin,
      position.y - params_.obstacle_filtering.crosswalk_margin),
    lanelet::Point2d(
      position.x + params_.obstacle_filtering.crosswalk_margin,
      position.y + params_.obstacle_filtering.crosswalk_margin)));

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
  if (object_speed < params_.obstacle_filtering.wrong_way_detection.min_speed_threshold) {
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

  const bool is_wrong_way_user =
    angle_diff > params_.obstacle_filtering.wrong_way_detection.angle_threshold;

  return is_wrong_way_user;
}

std::pair<lanelet::ConstLanelets, lanelet::ConstLanelets> RoadUserStopModule::getRelevantLanelets(
  const std::shared_ptr<const PlannerData> planner_data) const
{
  lanelet::ConstLanelets relevant_lanelets;

  if (!planner_data->route_handler) {
    RCLCPP_WARN(logger_, "Route handler is null, cannot get relevant lanelets");
    return std::make_pair(lanelet::ConstLanelets(), lanelet::ConstLanelets());
  }

  const auto & route_handler = planner_data->route_handler;
  const auto lanelet_map = route_handler->getLaneletMapPtr();

  if (!lanelet_map) {
    RCLCPP_WARN(logger_, "Lanelet map is null");
    return std::make_pair(lanelet::ConstLanelets(), lanelet::ConstLanelets());
  }

  // extract lane IDs from path_with_lane_id_
  std::set<lanelet::Id> unique_lane_ids;
  if (path_with_lane_id_ && !path_with_lane_id_->points.empty()) {
    for (size_t i = 0; i < path_with_lane_id_->points.size(); ++i) {
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
    return std::make_pair(lanelet::ConstLanelets(), lanelet::ConstLanelets());
  }

  // store ego lanelets for debug visualization (before adding adjacent lanelets)
  debug_data_.ego_lanelets = relevant_lanelets;

  // always check adjacent lanelets
  {
    // add adjacent lanelets
    lanelet::ConstLanelets adjacent_lanelets;
    debug_data_.adjacent_lanelets.clear();

    // create a copy to iterate over, as we'll be modifying relevant_lanelets
    const auto & current_lanelets = relevant_lanelets;

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

    // Add adjacent lanelets to relevant_lanelets
    for (const auto & adj_lanelet : adjacent_lanelets) {
      if (
        std::find(relevant_lanelets.begin(), relevant_lanelets.end(), adj_lanelet) ==
        relevant_lanelets.end()) {
        relevant_lanelets.push_back(adj_lanelet);
      }
    }

    // first, merge all ego lanelets
    std::vector<autoware_utils_geometry::Polygon2d> all_ego_polygons;
    std::vector<autoware_utils_geometry::Polygon2d> all_adjacent_polygons;
    std::vector<autoware_utils_geometry::Polygon2d> polygons_with_margin;
    for (const auto & ego_llt : debug_data_.ego_lanelets) {
      autoware_utils_geometry::Polygon2d ego_poly;
      for (const auto & pt : ego_llt.polygon3d()) {
        ego_poly.outer().emplace_back(pt.x(), pt.y());
      }
      if (!ego_poly.outer().empty()) {
        ego_poly.outer().push_back(ego_poly.outer().front());  // close polygon
      }
      boost::geometry::correct(ego_poly);
      all_ego_polygons.push_back(ego_poly);
    }

    for (const auto & adj_llt : debug_data_.adjacent_lanelets) {
      autoware_utils_geometry::Polygon2d adj_poly;
      for (const auto & pt : adj_llt.polygon3d()) {
        adj_poly.outer().emplace_back(pt.x(), pt.y());
      }
      if (!adj_poly.outer().empty()) {
        adj_poly.outer().push_back(adj_poly.outer().front());  // close polygon
      }
      boost::geometry::correct(adj_poly);
      all_adjacent_polygons.push_back(adj_poly);
    }
  }

  // Store lanelets for VRU detection (ego + adjacent)
  const auto & lanelets_for_wrongway_user = relevant_lanelets;
  debug_data_.lanelets_for_wrongway_user = lanelets_for_wrongway_user;

  // For wrongway detection, also include opposite lanes
  lanelet::ConstLanelets lanelets_for_vru = relevant_lanelets;

  // Add opposite lanes for wrongway detection
  for (const auto & ego_lanelet : debug_data_.ego_lanelets) {
    // Get left opposite lanes
    const auto left_opposite_lanelets = route_handler->getLeftOppositeLanelets(ego_lanelet);
    for (const auto & opposite : left_opposite_lanelets) {
      if (
        std::find(lanelets_for_vru.begin(), lanelets_for_vru.end(), opposite) ==
        lanelets_for_vru.end()) {
        lanelets_for_vru.push_back(opposite);
      }
    }

    // Get right opposite lanes
    const auto right_opposite_lanelets = route_handler->getRightOppositeLanelets(ego_lanelet);
    for (const auto & opposite : right_opposite_lanelets) {
      if (
        std::find(lanelets_for_vru.begin(), lanelets_for_vru.end(), opposite) ==
        lanelets_for_vru.end()) {
        lanelets_for_vru.push_back(opposite);
      }
    }
  }

  debug_data_.lanelets_for_vru = lanelets_for_vru;

  RCLCPP_DEBUG(logger_, "Found %zu relevant lanelets from path lane IDs", relevant_lanelets.size());

  return std::make_pair(lanelets_for_vru, lanelets_for_wrongway_user);
}

std::optional<geometry_msgs::msg::Point> RoadUserStopModule::planStop(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & trajectory_points,
  const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper)
{
  if (stop_obstacles.empty()) {
    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  std::optional<StopObstacle> determined_stop_obstacle{};
  std::optional<double> determined_zero_vel_dist{};
  std::optional<double> determined_desired_stop_margin{};

  // Get closest stop obstacles
  const auto ego_segment_idx =
    planner_data->find_segment_index(trajectory_points, planner_data->current_odometry.pose.pose);

  for (const auto & stop_obstacle : stop_obstacles) {
    // calculate dist to collide

    // TODO(odashima): consider braking distance
    const double dist_to_collide_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(trajectory_points, 0, ego_segment_idx) +
      stop_obstacle
        .dist_to_collide_on_decimated_traj;  //  + stop_obstacle.braking_dist.value_or(0.0);

    // calculate desired stop margin
    const double desired_stop_margin = calcDesiredStopMargin(
      planner_data, trajectory_points, stop_obstacle, dist_to_bumper, ego_segment_idx,
      dist_to_collide_on_ref_traj);

    // calculate stop point against the obstacle
    const auto candidate_zero_vel_dist = calcCandidateZeroVelDist(
      planner_data, trajectory_points, stop_obstacle, dist_to_collide_on_ref_traj,
      desired_stop_margin, dist_to_bumper);
    if (!candidate_zero_vel_dist) {
      continue;
    }

    if (determined_stop_obstacle) {
      const bool is_same_param_types =
        (stop_obstacle.classification.label == determined_stop_obstacle->classification.label);
      if (
        (is_same_param_types && stop_obstacle.dist_to_collide_on_decimated_traj >
                                  determined_stop_obstacle->dist_to_collide_on_decimated_traj) ||
        (!is_same_param_types && *candidate_zero_vel_dist > determined_zero_vel_dist)) {
        continue;
      }
    }
    determined_zero_vel_dist = *candidate_zero_vel_dist;
    determined_stop_obstacle = stop_obstacle;
    determined_desired_stop_margin = desired_stop_margin;
  }

  if (!(determined_zero_vel_dist && determined_stop_obstacle && determined_desired_stop_margin)) {
    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  // Hold previous stop distance if necessary
  holdPreviousStopIfNecessary(planner_data, trajectory_points, determined_zero_vel_dist);

  // Insert stop point
  const auto stop_point = calcStopPoint(
    planner_data, trajectory_points, dist_to_bumper, determined_stop_obstacle,
    determined_zero_vel_dist);

  if (determined_stop_obstacle->velocity >= -0.5) {  // max_negative_velocity
    return stop_point;
  }

  // Update path length buffer with current stop point
  path_length_buffer_.update_buffer(
    stop_point,
    [trajectory_points](const geometry_msgs::msg::Point & point) {
      return autoware::motion_utils::calcSignedArcLength(trajectory_points, 0, point);
    },
    clock_->now(), *determined_stop_obstacle, *determined_desired_stop_margin);

  // Get nearest active stop point from buffer
  const auto buffered_stop = path_length_buffer_.get_nearest_active_item();
  if (buffered_stop) {
    // Override with buffered stop point if available
    debug_data_.stop_index = std::nullopt;  // Update debug data if needed
    debug_data_.stop_point = buffered_stop->stop_point;

    return std::make_optional(buffered_stop->stop_point);
  }

  return std::nullopt;
}

std::optional<StopObstacle> RoadUserStopModule::pickStopObstacleFromPredictedObject(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polygons,
  const lanelet::ConstLanelets & lanelets_for_vru,
  const lanelet::ConstLanelets & lanelets_for_wrongway_user, const rclcpp::Time & current_time,
  const double dist_to_bumper)
{
  const auto & predicted_object = object->predicted_object;
  const auto & predicted_objects_stamp = planner_data->predicted_objects_header.stamp;
  const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  // check temporal filtering and get robust classification
  const auto object_id_str = autoware::universe_utils::toHexString(predicted_object.object_id);
  if (!hasMinimumDetectionDuration(object_id_str, current_time)) {
    return std::nullopt;
  }

  // Use most frequent classification from tracking history for robustness
  const auto tracked_it = tracked_objects_.find(object_id_str);
  ObjectClassification robust_classification = predicted_object.classification.front();
  if (tracked_it != tracked_objects_.end()) {
    robust_classification.label = tracked_it->second.getMostFrequentClassification();
  }

  // check if target object type using robust classification
  if (!isTargetObject(robust_classification.label)) {
    return std::nullopt;
  }

  // check if wrong-way user
  const bool is_wrong_way = [&]() {
    if (params_.obstacle_filtering.wrong_way_detection.enable) {
      for (const auto & lanelet : lanelets_for_wrongway_user) {
        if (isWrongWayUser(predicted_object, lanelet)) {
          return true;
        }
      }
    }
    return false;
  }();

  // check if object is on road
  const auto & relevant_lanelets = is_wrong_way ? lanelets_for_wrongway_user : lanelets_for_vru;
  if (!isObjectOnRoad(predicted_object, lanelet_map_ptr, relevant_lanelets)) {
    return std::nullopt;
  }

  // check if near crosswalk (exclude if configured)
  if (
    params_.obstacle_filtering.exclude_crosswalk_users &&
    isNearCrosswalk(
      predicted_object.kinematics.initial_pose_with_covariance.pose.position, lanelet_map_ptr)) {
    // skip crosswalk users
    return std::nullopt;
  }

  // check if on sidewalk (exclude if configured)
  if (
    params_.obstacle_filtering.exclude_sidewalk_users &&
    isOnSidewalk(
      predicted_object.kinematics.initial_pose_with_covariance.pose.position, lanelet_map_ptr)) {
    return std::nullopt;
  }

  // add to filtered objects for debug visualization
  debug_data_.filtered_objects.push_back(predicted_object);

  // calculate collision point and distance
  const auto & obj_pose =
    object->get_predicted_current_pose(clock_->now(), predicted_objects_stamp);

  // use pre-calculated trajectory polygons
  auto collision_point = polygon_utils::get_collision_point(
    decimated_traj_points, decimated_traj_polygons, obj_pose.position, clock_->now(),
    autoware_utils_geometry::to_polygon2d(obj_pose, predicted_object.shape), dist_to_bumper);

  if (!collision_point) {
    return std::nullopt;  // no collision point found
  }

  // create stop obstacle with robust classification
  StopObstacle stop_obstacle(
    autoware::universe_utils::toHexString(predicted_object.object_id), predicted_objects_stamp,
    robust_classification, obj_pose, predicted_object.shape,
    object->get_lon_vel_relative_to_traj(traj_points), collision_point->first,
    collision_point->second);

  // set additional fields
  stop_obstacle.is_wrong_way = is_wrong_way;
  stop_obstacle.original_object = predicted_object;

  return stop_obstacle;
};

void RoadUserStopModule::holdPreviousStopIfNecessary(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  std::optional<double> & determined_zero_vel_dist)
{
  if (
    std::abs(planner_data->current_odometry.twist.twist.linear.x) <
      params_.stop_planning.hold_stop_velocity_threshold &&
    prev_stop_distance_info_) {
    // NOTE: We assume that the current trajectory's front point is ahead of the previous
    // trajectory's front point.
    const size_t traj_front_point_prev_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prev_stop_distance_info_->first, traj_points.front().pose);
    const double diff_dist_front_points = autoware::motion_utils::calcSignedArcLength(
      prev_stop_distance_info_->first, 0, traj_points.front().pose.position,
      traj_front_point_prev_seg_idx);

    const double prev_zero_vel_dist = prev_stop_distance_info_->second - diff_dist_front_points;
    if (
      std::abs(prev_zero_vel_dist - determined_zero_vel_dist.value()) <
      params_.stop_planning.hold_stop_distance_threshold) {
      determined_zero_vel_dist.value() = prev_zero_vel_dist;
    }
  }
}

std::optional<geometry_msgs::msg::Point> RoadUserStopModule::calcStopPoint(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, [[maybe_unused]] const double dist_to_bumper,
  [[maybe_unused]] const std::optional<StopObstacle> & determined_stop_obstacle,
  const std::optional<double> & determined_zero_vel_dist)
{
  auto output_traj_points = traj_points;

  // insert stop point - this function interpolates between trajectory points
  // to create a smooth stop position, avoiding discrete jumps
  const auto zero_vel_idx =
    autoware::motion_utils::insertStopPoint(0, *determined_zero_vel_dist, output_traj_points);
  if (!zero_vel_idx) {
    return std::nullopt;
  }

  // update debug data
  debug_data_.stop_index = *zero_vel_idx;
  debug_data_.stop_point = output_traj_points.at(*zero_vel_idx).pose.position;

  // update planning factor
  const auto stop_pose = output_traj_points.at(*zero_vel_idx).pose;
  planning_factor_interface_->add(
    output_traj_points, planner_data->current_odometry.pose.pose, stop_pose,
    autoware::planning_factor_interface::PlanningFactor::STOP, SafetyFactorArray{});

  prev_stop_distance_info_ = std::make_pair(output_traj_points, determined_zero_vel_dist.value());

  return stop_pose.position;
}

double RoadUserStopModule::calcDesiredStopMargin(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  [[maybe_unused]] const double dist_to_bumper, [[maybe_unused]] const size_t ego_segment_idx,
  const double dist_to_collide_on_ref_traj) const
{
  // calculate default stop margin
  const double default_stop_margin = [&]() {
    const double v_ego = planner_data->current_odometry.twist.twist.linear.x;
    const double v_obs = stop_obstacle.velocity;

    const auto ref_traj_length =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_points.size() - 1);

    // For negative velocity obstacles (approaching)
    if (v_obs < params_.stop_planning.max_negative_velocity) {
      const double a_ego = params_.stop_planning.effective_deceleration_opposing_traffic;
      const double & bumper_to_bumper_distance = stop_obstacle.dist_to_collide_on_decimated_traj;

      const double braking_distance = v_ego * v_ego / (2 * a_ego);
      const double stopping_time = v_ego / a_ego;
      const double distance_obs_ego_braking = std::abs(v_obs * stopping_time);

      const double ego_stop_margin = params_.stop_planning.stop_margin_opposing_traffic;

      const double rel_vel = v_ego - v_obs;
      constexpr double epsilon = 1e-6;  // Small threshold for numerical stability
      if (std::abs(rel_vel) <= epsilon) {
        RCLCPP_WARN(
          logger_,
          "Relative velocity (%.3f) is too close to zero. Using minimum safe value for "
          "calculation.",
          rel_vel);
        return params_.stop_planning.stop_margin;  // Return default stop margin as fallback
      }

      const double T_coast = std::max(
        (bumper_to_bumper_distance - ego_stop_margin - braking_distance +
         distance_obs_ego_braking) /
          rel_vel,
        0.0);

      const double stopping_distance = v_ego * T_coast + braking_distance;

      const double stop_margin = bumper_to_bumper_distance - stopping_distance;

      return stop_margin;
    }

    if (dist_to_collide_on_ref_traj > ref_traj_length) {
      return params_.stop_planning.terminal_stop_margin;
    }

    return params_.stop_planning.stop_margin;
  }();

  // calculate stop margin on curve
  const double stop_margin_on_curve = calcMarginFromObstacleOnCurve(
    planner_data, traj_points, stop_obstacle, dist_to_bumper, default_stop_margin);

  // calculate stop margin considering behavior's stop point
  // NOTE: If behavior stop point is ahead of the closest_obstacle_stop point within a certain
  //       margin we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const auto closest_behavior_stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(traj_points, ego_segment_idx + 1);
  if (closest_behavior_stop_idx) {
    const double closest_behavior_stop_dist_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, *closest_behavior_stop_idx);
    const double stop_dist_diff =
      closest_behavior_stop_dist_on_ref_traj - (dist_to_collide_on_ref_traj - stop_margin_on_curve);
    if (0.0 < stop_dist_diff && stop_dist_diff < stop_margin_on_curve) {
      return params_.stop_planning.min_behavior_stop_margin;
    }
  }
  return stop_margin_on_curve;
}

std::optional<double> RoadUserStopModule::calcCandidateZeroVelDist(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  [[maybe_unused]] const StopObstacle & stop_obstacle, const double dist_to_collide_on_ref_traj,
  const double desired_stop_margin, [[maybe_unused]] const double dist_to_bumper) const
{
  double candidate_zero_vel_dist = std::max(0.0, dist_to_collide_on_ref_traj - desired_stop_margin);

  if (params_.option.suppress_sudden_stop) {
    const auto acceptable_stop_acc = [&]() -> std::optional<double> {
      const double distance_to_judge_suddenness = std::min(
        calcMinimumDistanceToStop(
          planner_data->current_odometry.twist.twist.linear.x, params_.common_param.limit_max_accel,
          params_.stop_planning.sudden_object_acc_threshold),
        params_.stop_planning.sudden_object_dist_threshold);

      if (candidate_zero_vel_dist > distance_to_judge_suddenness) {
        return params_.common_param.limit_min_accel;
      }

      if (params_.stop_planning.abandon_to_stop) {
        RCLCPP_WARN(logger_, "[RoadUserStop] abandon to stop against object");
        return std::nullopt;
      } else {
        return params_.stop_planning.limit_min_acc;
      }
    }();

    if (!acceptable_stop_acc) {
      return std::nullopt;
    }

    const double acceptable_stop_pos =
      autoware::motion_utils::calcSignedArcLength(
        traj_points, 0, planner_data->current_odometry.pose.pose.position) +
      calcMinimumDistanceToStop(
        planner_data->current_odometry.twist.twist.linear.x, params_.common_param.limit_max_accel,
        acceptable_stop_acc.value());

    if (acceptable_stop_pos > candidate_zero_vel_dist) {
      candidate_zero_vel_dist = acceptable_stop_pos;
    }
  }

  return candidate_zero_vel_dist;
}

std::vector<autoware_utils_geometry::Polygon2d> RoadUserStopModule::getTrajectoryPolygons(
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  if (trajectory_polygon_for_inside_map_.count(lat_margin) == 0) {
    const auto traj_polys = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_ego_pose, lat_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length);
    trajectory_polygon_for_inside_map_.emplace(lat_margin, traj_polys);
  }
  return trajectory_polygon_for_inside_map_.at(lat_margin);
}

double RoadUserStopModule::calcMarginFromObstacleOnCurve(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_bumper, const double default_stop_margin) const
{
  const bool enable_approaching_on_curve = params_.stop_planning.stop_on_curve.enable_approaching;
  const bool use_pointcloud = false;  // road_user_stop doesn't use pointcloud

  if (!enable_approaching_on_curve || use_pointcloud) {
    return default_stop_margin;
  }

  // calculate short trajectory points towards obstacle
  const size_t obj_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, stop_obstacle.collision_point);
  std::vector<TrajectoryPoint> short_traj_points{traj_points.at(obj_segment_idx + 1)};
  double sum_short_traj_length{0.0};
  for (int i = obj_segment_idx; 0 <= i; --i) {
    short_traj_points.push_back(traj_points.at(i));

    if (
      1 < short_traj_points.size() &&
      params_.stop_planning.stop_margin + dist_to_bumper < sum_short_traj_length) {
      break;
    }
    sum_short_traj_length +=
      autoware_utils_geometry::calc_distance2d(traj_points.at(i), traj_points.at(i + 1));
  }
  std::reverse(short_traj_points.begin(), short_traj_points.end());
  if (short_traj_points.size() < 2) {
    return default_stop_margin;
  }

  // calculate collision index between straight line from ego pose and object
  const auto calculate_distance_from_straight_ego_path =
    [&](const auto & ego_pose, const auto & object_polygon) {
      const auto forward_ego_pose = autoware_utils_geometry::calc_offset_pose(
        ego_pose, params_.stop_planning.stop_margin + 3.0, 0.0, 0.0);
      const auto ego_straight_segment = autoware_utils_geometry::Segment2d{
        convertPoint(ego_pose.position), convertPoint(forward_ego_pose.position)};
      return boost::geometry::distance(ego_straight_segment, object_polygon);
    };
  const auto resampled_short_traj_points = resampleTrajectoryPoints(short_traj_points, 0.5);
  const auto object_polygon =
    autoware_utils_geometry::to_polygon2d(stop_obstacle.pose, stop_obstacle.shape);
  const auto collision_idx = [&]() -> std::optional<size_t> {
    for (size_t i = 0; i < resampled_short_traj_points.size(); ++i) {
      const double dist_to_obj = calculate_distance_from_straight_ego_path(
        resampled_short_traj_points.at(i).pose, object_polygon);
      if (dist_to_obj < planner_data->vehicle_info_.vehicle_width_m / 2.0) {
        return i;
      }
    }
    return std::nullopt;
  }();
  if (!collision_idx) {
    return params_.stop_planning.stop_on_curve.min_stop_margin;
  }
  if (*collision_idx == 0) {
    return default_stop_margin;
  }

  // calculate margin from obstacle
  const double partial_segment_length = [&]() {
    const double collision_segment_length = autoware_utils_geometry::calc_distance2d(
      resampled_short_traj_points.at(*collision_idx - 1),
      resampled_short_traj_points.at(*collision_idx));
    const double prev_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx - 1).pose, object_polygon);
    const double next_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(*collision_idx).pose, object_polygon);
    return (next_dist - planner_data->vehicle_info_.vehicle_width_m / 2.0) /
           (next_dist - prev_dist) * collision_segment_length;
  }();

  const double short_margin_from_obstacle =
    partial_segment_length +
    autoware::motion_utils::calcSignedArcLength(
      resampled_short_traj_points, *collision_idx, stop_obstacle.collision_point) -
    dist_to_bumper + params_.stop_planning.stop_on_curve.additional_stop_margin;

  return std::min(
    default_stop_margin,
    std::max(params_.stop_planning.stop_on_curve.min_stop_margin, short_margin_from_obstacle));
}

}  // namespace autoware::motion_velocity_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::RoadUserStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
