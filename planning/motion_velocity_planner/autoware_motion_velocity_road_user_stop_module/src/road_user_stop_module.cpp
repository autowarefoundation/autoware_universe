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

#include "types.hpp"
#include "utils.hpp"

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/strategies/buffer.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>
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
using road_user_stop::utils::to_polygon_2d;

namespace
{
std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(traj, interval);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

autoware_utils_geometry::Point2d convert_point(const Point & p)
{
  return autoware_utils_geometry::Point2d{p.x, p.y};
}

double calc_minimum_distance_to_stop(
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

  // Initialize parameter listener with module name as prefix
  param_listener_ = std::make_shared<::road_user_stop::ParamListener>(
    node.get_node_parameters_interface(), logger_, "road_user_stop");

  common_param_ = CommonParam(node);

  virtual_wall_publisher_ = node.create_publisher<MarkerArray>("~/road_user_stop/virtual_walls", 1);
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "road_user_stop");

  debug_publisher_ = node.create_publisher<MarkerArray>("~/road_user_stop/debug_markers", 1);

  processing_time_detail_pub_ = node.create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/road_user_stop", 1);

  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(processing_time_detail_pub_);
}

void RoadUserStopModule::update_tracked_objects(
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const rclcpp::Time & current_time)
{
  const auto param = param_listener_->get_params();

  // Clean up old tracked objects that haven't been seen for a while
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if (
      (current_time - it->second.last_detected_time).seconds() >
      param.obstacle_filtering.lost_object_retention_duration) {
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
      tracked_obj.last_stop_obstacle_time = current_time;
      tracked_obj.was_inside_detection_area = false;
      tracked_obj.polygon_expansion_length = 0.0;
      tracked_obj.updateClassification(predicted_object.classification.front().label);
      tracked_objects_[object_id] = tracked_obj;
    }
  }
}

bool RoadUserStopModule::has_minimum_detection_duration(
  const std::string & object_id, const rclcpp::Time & current_time) const
{
  const auto param = param_listener_->get_params();
  auto it = tracked_objects_.find(object_id);
  if (it == tracked_objects_.end()) {
    return false;
  }

  const double detection_duration = (current_time - it->second.first_detected_time).seconds();
  return detection_duration >= param.obstacle_filtering.min_detection_duration;
}

std::vector<StopObstacle> RoadUserStopModule::filter_stop_obstacles(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polygons, const RelevantLaneletData & lanelet_data,
  const rclcpp::Time & current_time, const double dist_to_bumper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  update_tracked_objects(planner_data->objects, current_time);

  std::set<std::string> current_object_ids;

  std::vector<StopObstacle> stop_obstacles;
  for (const auto & object : planner_data->objects) {
    if (
      auto stop_obstacle = pick_stop_obstacle_from_predicted_object(
        planner_data, object, traj_points, decimated_traj_points, decimated_traj_polygons,
        lanelet_data, current_time, dist_to_bumper)) {
      stop_obstacles.push_back(stop_obstacle.value());
      const std::string object_id =
        autoware::universe_utils::toHexString(object->predicted_object.object_id);
      current_object_ids.insert(object_id);

      // Update last_stop_obstacle_time for this object
      auto tracked_it = tracked_objects_.find(object_id);
      if (tracked_it != tracked_objects_.end()) {
        tracked_it->second.last_stop_obstacle_time = current_time;
      }
    }
  }

  for (const auto & prev_obstacle : prev_stop_obstacles_) {
    const std::string object_id = autoware::universe_utils::toHexString(prev_obstacle.uuid);

    // Check if object is still being tracked
    auto tracked_it = tracked_objects_.find(object_id);
    if (tracked_it == tracked_objects_.end()) {
      continue;  // Not tracked, skip
    }

    // Check if object is lost (not in current objects)
    if (current_object_ids.find(object_id) != current_object_ids.end()) {
      continue;  // Object is still present, skip
    }

    // Check if object was lost recently (within threshold)
    const auto param = param_listener_->get_params();
    // Use last_stop_obstacle_time to check how long it's been since this object was a stop obstacle
    const double lost_duration =
      (current_time - tracked_it->second.last_stop_obstacle_time).seconds();
    if (lost_duration > param.obstacle_filtering.lost_object_retention_duration) {
      continue;  // Object was lost too long ago, skip
    }

    // Check if this object ID is not already in current stop_obstacles
    bool already_exists = false;
    for (const auto & current_obstacle : stop_obstacles) {
      if (object_id == autoware::universe_utils::toHexString(current_obstacle.uuid)) {
        already_exists = true;
        break;
      }
    }

    if (!already_exists) {
      // Create a lost object placeholder with fixed position
      StopObstacle lost_obstacle = prev_obstacle;
      lost_obstacle.is_lost = true;
      lost_obstacle.lost_time = tracked_it->second.last_detected_time;

      stop_obstacles.push_back(lost_obstacle);
    }
  }

  prev_stop_obstacles_ = stop_obstacles;

  return stop_obstacles;
}

void RoadUserStopModule::update_parameters(const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  // Parameter updates are handled automatically by the parameter listener
  if (param_listener_) {
    param_listener_->refresh_dynamic_parameters();
  }
}

void RoadUserStopModule::publish_planning_factor()
{
  planning_factor_interface_->publish();
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
  debug_data_.object_polygons.clear();

  // Extract parameters and current state
  const auto & trajectory_points = raw_trajectory_points;
  const auto current_time = clock_->now();
  const double dist_to_bumper = planner_data->vehicle_info_.max_longitudinal_offset_m;

  // 1. Prepare trajectory data for collision checking
  // 1.1 Decimate trajectory points to reduce computational cost
  const auto param = param_listener_->get_params();
  const auto decimated_traj_points = [&]() {
    autoware_utils_debug::ScopedTimeTrack st_debug(
      "decimate_trajectory_points_from_ego", *time_keeper_);
    const auto traj_point = utils::decimate_trajectory_points_from_ego(
      trajectory_points, planner_data->current_odometry.pose.pose,
      planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold,
      planner_data->trajectory_polygon_collision_check.decimate_trajectory_step_length,
      param.stop_planning.longitudinal_margin.default_margin);

    return traj_point;
  }();

  // 1.2 Calculate trajectory lanelets
  const auto ego_lanelets = get_ego_lanelets(decimated_traj_points, planner_data);

  // 1.3 Create trajectory polygons for collision detection
  const auto & p = planner_data->trajectory_polygon_collision_check;
  const auto decimated_traj_polygons = get_trajectory_polygons(
    decimated_traj_points, planner_data->vehicle_info_, planner_data->current_odometry.pose.pose,
    param.obstacle_filtering.trajectory_lateral_margin, p.enable_to_consider_current_pose,
    p.time_to_convergence, p.decimate_trajectory_step_length);
  debug_data_.trajectory_polygons = decimated_traj_polygons;

  // 2. Get relevant lanelets for VRU and opposite traffic detection
  const auto lanelet_data = get_relevant_lanelet_data(ego_lanelets, planner_data);
  debug_data_.polygons_for_vru = lanelet_data.polygons_for_vru;
  debug_data_.polygons_for_opposing_traffic = lanelet_data.polygons_for_opposing_traffic;
  debug_data_.ego_lanelets = lanelet_data.ego_lanelets;
  debug_data_.opposite_lanelets = lanelet_data.opposite_lanelets;
  RCLCPP_DEBUG(
    logger_, "Found %zu polygons for VRU, %zu for opposing traffic",
    lanelet_data.polygons_for_vru.size(), lanelet_data.polygons_for_opposing_traffic.size());

  // 3. Filter objects and create stop obstacles
  const auto stop_obstacles = filter_stop_obstacles(
    planner_data, trajectory_points, decimated_traj_points, decimated_traj_polygons, lanelet_data,
    current_time, dist_to_bumper);

  // 4. Calculate stop point based on filtered obstacles
  const auto stop_point =
    plan_stop(planner_data, trajectory_points, stop_obstacles, dist_to_bumper);
  if (stop_point.has_value()) {
    result.stop_points.push_back(stop_point.value());
  }

  // 5. Publish debug information
  publish_debug_info();

  return result;
}

bool RoadUserStopModule::is_target_object(const uint8_t label) const
{
  const auto param = param_listener_->get_params();
  switch (label) {
    case ObjectClassification::PEDESTRIAN:
      return param.obstacle_filtering.object_type.pedestrian;
    case ObjectClassification::BICYCLE:
      return param.obstacle_filtering.object_type.bicycle;
    case ObjectClassification::MOTORCYCLE:
      return param.obstacle_filtering.object_type.motorcycle;
    case ObjectClassification::UNKNOWN:
      return param.obstacle_filtering.object_type.unknown;
    default:
      return false;
  }
}

bool RoadUserStopModule::is_object_on_road(
  const PredictedObject & object, const std::vector<Polygon2d> & relevant_polygons) const
{
  const auto param = param_listener_->get_params();
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;

  // get object polygon
  auto object_polygon = autoware_utils_geometry::to_polygon2d(obj_pose, object.shape);

  // check if object was previously inside detection area for polygon expansion
  const std::string object_id = autoware::universe_utils::toHexString(object.object_id);
  auto tracked_it = tracked_objects_.find(object_id);
  double expansion_length = 0.0;

  if (tracked_it != tracked_objects_.end() && tracked_it->second.was_inside_detection_area) {
    expansion_length = param.obstacle_filtering.polygon_expansion_length;

    // expand object polygon to reduce chattering
    // expansion_length is in meters - absolute distance to expand outward from centroid
    if (expansion_length > 0.0) {
      object_polygon = autoware_utils_geometry::expand_polygon(object_polygon, expansion_length);
    }
  }

  // add object polygon to debug data
  debug_data_.object_polygons.push_back(object_polygon);

  // check if object polygon intersects with any relevant polygon
  for (const auto & lanelet_polygon : relevant_polygons) {
    // check intersection using boost::geometry
    if (!boost::geometry::disjoint(object_polygon, lanelet_polygon)) {
      // update tracking information
      if (tracked_it != tracked_objects_.end()) {
        tracked_it->second.was_inside_detection_area = true;
        tracked_it->second.polygon_expansion_length = expansion_length;
      }
      return true;
    }
  }

  // object is not on road - update tracking information
  if (tracked_it != tracked_objects_.end()) {
    // only reset if object was previously inside and now clearly outside
    if (expansion_length == 0.0) {  // no expansion was used, so this is a clear "outside" result
      tracked_it->second.was_inside_detection_area = false;
      tracked_it->second.polygon_expansion_length = 0.0;
    }
  }

  return false;
}

bool RoadUserStopModule::is_near_crosswalk(
  const PredictedObject & predicted_object, const std::shared_ptr<const PlannerData> & planner_data,
  const lanelet::ConstLanelets & ego_lanelets) const
{
  const auto param = param_listener_->get_params();
  const auto & route_handler = planner_data->route_handler;
  const auto & overall_graphs = route_handler->getOverallGraphPtr();
  const auto & obj_pos = predicted_object.kinematics.initial_pose_with_covariance.pose.position;

  // check if position is near any crosswalk
  for (auto & ll : ego_lanelets) {
    // get conflicting crosswalk crosswalk
    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    const auto conflicts = overall_graphs->conflictingInGraph(ll, PEDESTRIAN_GRAPH_ID);
    for (const auto & crosswalk : conflicts) {
      // additional check using geometry distance calculation
      const auto crosswalk_polygon = to_polygon_2d(crosswalk.polygon2d().basicPolygon());

      const autoware_utils_geometry::Point2d object_point_2d{obj_pos.x, obj_pos.y};
      const double distance = boost::geometry::distance(object_point_2d, crosswalk_polygon);

      if (distance < param.obstacle_filtering.crosswalk.margin) {
        RCLCPP_DEBUG(
          logger_, "Object is near crosswalk (distance: %.2f < margin: %.2f)", distance,
          param.obstacle_filtering.crosswalk.margin);
        return true;
      }
    }
  }

  return false;
}

bool RoadUserStopModule::is_opposite_traffic_user(
  const PredictedObject & object, const lanelet::ConstLanelet & lanelet) const
{
  // get object velocity
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const double object_speed = std::hypot(twist.linear.x, twist.linear.y);

  // skip if object is stopped or moving too slowly
  const auto param = param_listener_->get_params();
  if (object_speed < param.obstacle_filtering.opposing_traffic_detection.min_speed_threshold) {
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

  const bool is_opposing_traffic_user =
    angle_diff > param.obstacle_filtering.opposing_traffic_detection.angle_threshold;

  return is_opposing_traffic_user;
}

lanelet::ConstLanelets RoadUserStopModule::get_ego_lanelets(
  const std::vector<TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & route_handler = planner_data->route_handler;
  const auto & vehicle_info = planner_data->vehicle_info_;
  // from trajectory points
  LineString2d trajectory_ls;
  for (const auto & p : smoothed_trajectory_points) {
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  // add a point beyond the last trajectory point to account for the ego front offset
  const auto pose_beyond = autoware_utils::calc_offset_pose(
    smoothed_trajectory_points.back().pose, vehicle_info.max_longitudinal_offset_m, 0.0, 0.0, 0.0);
  trajectory_ls.emplace_back(pose_beyond.position.x, pose_beyond.position.y);
  // calculate the lanelets overlapped by the trajectory
  auto calc_trajectory_lanelets =
    [&](
      const LineString2d & trajectory_ls,
      const std::shared_ptr<const route_handler::RouteHandler> route_handler) {
      auto is_road_lanelet = [](const lanelet::ConstLanelet & lanelet) -> bool {
        return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
               lanelet.attribute(lanelet::AttributeName::Subtype) ==
                 lanelet::AttributeValueString::Road;
      };
      const auto lanelet_map_ptr = route_handler->getLaneletMapPtr();
      lanelet::ConstLanelets trajectory_lanelets;
      const auto candidates = lanelet_map_ptr->laneletLayer.search(
        boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));
      for (const auto & ll : candidates) {
        if (
          is_road_lanelet(ll) &&
          !boost::geometry::disjoint(trajectory_ls, ll.polygon2d().basicPolygon())) {
          trajectory_lanelets.push_back(ll);
        }
      }
      return trajectory_lanelets;
    };
  return calc_trajectory_lanelets(trajectory_ls, route_handler);
}

RelevantLaneletData RoadUserStopModule::get_relevant_lanelet_data(
  const lanelet::ConstLanelets & ego_lanelets,
  const std::shared_ptr<const PlannerData> planner_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto param = param_listener_->get_params();
  const auto & route_handler = planner_data->route_handler;
  const auto lanelet_map = route_handler->getLaneletMapPtr();

  const auto intersection_polygons = [&]() {
    std::vector<Polygon2d> polygons;
    for (const auto & ego_lanelet : ego_lanelets) {
      const std::string area_id_str = ego_lanelet.attributeOr("intersection_area", "else");
      if (area_id_str != "else" && std::atoi(area_id_str.c_str())) {
        const auto polygon_opt =
          route_handler->getLaneletMapPtr()->polygonLayer.find(std::atoi(area_id_str.c_str()));
        if (polygon_opt == route_handler->getLaneletMapPtr()->polygonLayer.end()) {
          continue;
        }
        const auto & polygon = *polygon_opt;
        const auto polygon_2d = to_polygon_2d(polygon.basicPolygon());
        polygons.push_back(polygon_2d);
      }
    }
    debug_data_.intersection_polygons = polygons;

    return polygons;
  }();

  // get intersection lanelets on route
  const auto ego_lanelets_without_intersection = [&]() {
    lanelet::ConstLanelets lanelets;

    for (const auto & ego_lanelet : ego_lanelets) {
      const std::string area_id = ego_lanelet.attributeOr("intersection_area", "else");
      if (area_id != "else" && std::atoi(area_id.c_str())) {
        continue;
      }
      lanelets.push_back(ego_lanelet);
    }
    debug_data_.ego_lanelets_without_intersection = lanelets;

    return lanelets;
  }();

  const auto adjacent_lanelets = [&]() {
    lanelet::ConstLanelets adjacent_lanelets;
    const auto is_in_adjacent_lanelet = [&](const lanelet::ConstLanelet & lanelet) {
      return std::find(adjacent_lanelets.begin(), adjacent_lanelets.end(), lanelet) !=
             adjacent_lanelets.end();
    };

    for (const auto & lanelet : ego_lanelets_without_intersection) {
      const auto left_lanelet = route_handler->getLeftLanelet(lanelet);
      if (left_lanelet.has_value() && !is_in_adjacent_lanelet(left_lanelet.value())) {
        adjacent_lanelets.push_back(left_lanelet.value());
      }

      const auto right_lanelet = route_handler->getRightLanelet(lanelet);
      if (right_lanelet.has_value() && !is_in_adjacent_lanelet(right_lanelet.value())) {
        adjacent_lanelets.push_back(right_lanelet.value());
      }

      const auto left_shoulder = route_handler->getLeftShoulderLanelet(lanelet);
      if (left_shoulder.has_value() && !is_in_adjacent_lanelet(left_shoulder.value())) {
        adjacent_lanelets.push_back(left_shoulder.value());
      }

      const auto right_shoulder = route_handler->getRightShoulderLanelet(lanelet);
      if (right_shoulder.has_value() && !is_in_adjacent_lanelet(right_shoulder.value())) {
        adjacent_lanelets.push_back(right_shoulder.value());
      }
    }
    debug_data_.adjacent_lanelets = adjacent_lanelets;

    return adjacent_lanelets;
  }();

  const auto opposite_lanelets = [&]() {
    lanelet::ConstLanelets opposite_lanelets;
    for (const auto & ego_lanelet : ego_lanelets_without_intersection) {
      const auto left_opposite_lanelets = route_handler->getLeftOppositeLanelets(ego_lanelet);
      for (const auto & opposite : left_opposite_lanelets) {
        opposite_lanelets.push_back(opposite);
      }

      const auto right_opposite_lanelets = route_handler->getRightOppositeLanelets(ego_lanelet);
      for (const auto & opposite : right_opposite_lanelets) {
        opposite_lanelets.push_back(opposite);
      }
    }
    debug_data_.opposite_lanelets = opposite_lanelets;
    return opposite_lanelets;
  }();

  const auto [polygons_for_vru, polygons_for_opposing_traffic] = [&]() {
    // lanelet polygons for opposing traffic detection (ego + adjacent lanes)
    std::vector<Polygon2d> polygons_for_opposing_traffic;

    for (const auto & lanelet : ego_lanelets_without_intersection) {
      polygons_for_opposing_traffic.push_back(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
    }
    for (const auto & lanelet : adjacent_lanelets) {
      polygons_for_opposing_traffic.push_back(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
    }

    if (!param.obstacle_filtering.intersection.exclude) {
      for (const auto & polygon : intersection_polygons) {
        polygons_for_opposing_traffic.push_back(polygon);
      }
    }

    // lanelet polygons for VRU detection (ego + adjacent + opposite lanes)
    std::vector<Polygon2d> polygons_for_vru = polygons_for_opposing_traffic;
    for (const auto & lanelet : opposite_lanelets) {
      polygons_for_vru.push_back(to_polygon_2d(lanelet.polygon2d().basicPolygon()));
    }

    return std::make_pair(polygons_for_vru, polygons_for_opposing_traffic);
  }();

  RelevantLaneletData result;
  result.polygons_for_vru = polygons_for_vru;
  result.polygons_for_opposing_traffic = polygons_for_opposing_traffic;
  result.ego_lanelets = ego_lanelets;
  result.opposite_lanelets = opposite_lanelets;
  return result;
}

std::optional<Point> RoadUserStopModule::plan_stop(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & trajectory_points,
  const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (stop_obstacles.empty()) {
    // delete marker
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(clock_->now(), 0);
    autoware_utils_visualization::append_marker_array(markers, &debug_data_.stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  std::optional<StopObstacle> determined_stop_obstacle{};
  std::optional<double> determined_zero_vel_dist{};
  std::optional<double> determined_desired_stop_margin{};

  const auto ego_segment_idx =
    planner_data->find_segment_index(trajectory_points, planner_data->current_odometry.pose.pose);

  for (const auto & stop_obstacle : stop_obstacles) {
    double dist_to_collide_on_ref_traj;

    if (stop_obstacle.is_lost) {
      dist_to_collide_on_ref_traj = autoware::motion_utils::calcSignedArcLength(
        trajectory_points, ego_segment_idx, stop_obstacle.collision_point);
    } else {
      dist_to_collide_on_ref_traj =
        autoware::motion_utils::calcSignedArcLength(trajectory_points, 0, ego_segment_idx) +
        stop_obstacle.dist_to_collide_on_decimated_traj;
    }

    const double desired_stop_margin = calc_desired_stop_margin(
      planner_data, trajectory_points, stop_obstacle, dist_to_bumper, ego_segment_idx,
      dist_to_collide_on_ref_traj);

    const auto candidate_zero_vel_dist = calc_candidate_zero_vel_dist(
      planner_data, trajectory_points, dist_to_collide_on_ref_traj, desired_stop_margin,
      dist_to_bumper);
    if (!candidate_zero_vel_dist.has_value()) {
      continue;
    }

    if (determined_stop_obstacle.has_value()) {
      const bool is_same_param_types =
        (stop_obstacle.classification.label ==
         determined_stop_obstacle.value().classification.label);
      if (
        (is_same_param_types &&
         stop_obstacle.dist_to_collide_on_decimated_traj >
           determined_stop_obstacle.value().dist_to_collide_on_decimated_traj) ||
        (!is_same_param_types && candidate_zero_vel_dist.value() > determined_zero_vel_dist)) {
        continue;
      }
    }
    determined_zero_vel_dist = candidate_zero_vel_dist.value();
    determined_stop_obstacle = stop_obstacle;
    determined_desired_stop_margin = desired_stop_margin;
  }

  if (
    !determined_zero_vel_dist.has_value() || !determined_stop_obstacle.has_value() ||
    !determined_desired_stop_margin.has_value()) {
    // delete marker
    const auto markers =
      autoware::motion_utils::createDeletedStopVirtualWallMarker(clock_->now(), 0);
    autoware_utils_visualization::append_marker_array(markers, &debug_data_.stop_wall_marker);

    prev_stop_distance_info_ = std::nullopt;
    return std::nullopt;
  }

  hold_previous_stop_if_necessary(planner_data, trajectory_points, determined_zero_vel_dist);

  const auto stop_point = calc_stop_point(
    planner_data, trajectory_points, determined_stop_obstacle, determined_zero_vel_dist);

  if (determined_stop_obstacle.has_value() && determined_stop_obstacle.value().velocity >= -0.5) {
    return stop_point;
  }

  path_length_buffer_.update_buffer(
    stop_point,
    [trajectory_points](const Point & point) {
      return autoware::motion_utils::calcSignedArcLength(trajectory_points, 0, point);
    },
    clock_->now(), determined_stop_obstacle.value(), determined_desired_stop_margin.value());

  const auto buffered_stop = path_length_buffer_.get_nearest_active_item();
  if (buffered_stop.has_value()) {
    debug_data_.stop_index = std::nullopt;
    debug_data_.stop_point = buffered_stop.value().stop_point;

    return std::make_optional(buffered_stop.value().stop_point);
  }

  return std::nullopt;
}

std::optional<StopObstacle> RoadUserStopModule::pick_stop_obstacle_from_predicted_object(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::shared_ptr<PlannerData::Object> object,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polygons, const RelevantLaneletData & lanelet_data,
  const rclcpp::Time & current_time, const double dist_to_bumper)
{
  const auto & predicted_object = object->predicted_object;
  const auto & predicted_objects_stamp = planner_data->predicted_objects_header.stamp;
  const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

  // check temporal filtering and get robust classification
  const auto object_id_str = autoware::universe_utils::toHexString(predicted_object.object_id);
  if (!has_minimum_detection_duration(object_id_str, current_time)) {
    return std::nullopt;
  }

  // Use most frequent classification from tracking history for robustness
  const auto tracked_it = tracked_objects_.find(object_id_str);
  ObjectClassification robust_classification = predicted_object.classification.front();
  if (tracked_it != tracked_objects_.end()) {
    robust_classification.label = tracked_it->second.getMostFrequentClassification();
  }

  // check if target object type using robust classification
  if (!is_target_object(robust_classification.label)) {
    return std::nullopt;
  }

  // check if opposite traffic user
  const auto param = param_listener_->get_params();
  const bool is_opposing_traffic = [&]() {
    if (param.obstacle_filtering.opposing_traffic_detection.enable) {
      for (const auto & lanelet : lanelet_data.opposite_lanelets) {
        if (is_opposite_traffic_user(predicted_object, lanelet)) {
          return true;
        }
      }
    }
    return false;
  }();

  // check if object is on road
  const auto & relevant_polygons = is_opposing_traffic ? lanelet_data.polygons_for_opposing_traffic
                                                       : lanelet_data.polygons_for_vru;
  if (!is_object_on_road(predicted_object, relevant_polygons)) {
    return std::nullopt;
  }

  // check if near crosswalk (exclude if configured)
  if (
    param.obstacle_filtering.crosswalk.exclude &&
    is_near_crosswalk(predicted_object, planner_data, lanelet_data.ego_lanelets)) {
    // skip crosswalk users
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

  if (!collision_point.has_value()) {
    return std::nullopt;
  }

  return StopObstacle{
    predicted_object.object_id,
    predicted_objects_stamp,
    is_opposing_traffic,
    predicted_object,
    predicted_object.classification.at(0),
    obj_pose,
    predicted_object.shape,
    object->get_lon_vel_relative_to_traj(traj_points),
    collision_point.value().first,
    collision_point.value().second};
};

void RoadUserStopModule::hold_previous_stop_if_necessary(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  std::optional<double> & determined_zero_vel_dist)
{
  const auto param = param_listener_->get_params();
  if (
    std::abs(planner_data->current_odometry.twist.twist.linear.x) <
      param.stop_planning.hold_stop_velocity_threshold &&
    prev_stop_distance_info_.has_value()) {
    // NOTE: We assume that the current trajectory's front point is ahead of the previous
    // trajectory's front point.
    const size_t traj_front_point_prev_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        prev_stop_distance_info_.value().first, traj_points.front().pose);
    const double diff_dist_front_points = autoware::motion_utils::calcSignedArcLength(
      prev_stop_distance_info_.value().first, 0, traj_points.front().pose.position,
      traj_front_point_prev_seg_idx);

    const double prev_zero_vel_dist =
      prev_stop_distance_info_.value().second - diff_dist_front_points;
    if (
      std::abs(prev_zero_vel_dist - determined_zero_vel_dist.value()) <
      param.stop_planning.hold_stop_distance_threshold) {
      determined_zero_vel_dist.value() = prev_zero_vel_dist;
    }
  }
}

std::optional<Point> RoadUserStopModule::calc_stop_point(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::optional<StopObstacle> & determined_stop_obstacle,
  const std::optional<double> & determined_zero_vel_dist)
{
  auto output_traj_points = traj_points;
  const double dist_to_bumper = planner_data->vehicle_info_.max_longitudinal_offset_m;

  // insert stop point - this function interpolates between trajectory points
  // to create a smooth stop position, avoiding discrete jumps
  const auto zero_vel_idx = autoware::motion_utils::insertStopPoint(
    0, determined_zero_vel_dist.value(), output_traj_points);
  if (!zero_vel_idx.has_value()) {
    return std::nullopt;
  }
  const auto stop_point = output_traj_points.at(zero_vel_idx.value()).pose.position;

  // virtual wall marker for stop obstacle
  const auto markers = autoware::motion_utils::createStopVirtualWallMarker(
    output_traj_points.at(*zero_vel_idx).pose, "road user stop", clock_->now(), 0, dist_to_bumper,
    "", planner_data->is_driving_forward);
  autoware_utils_visualization::append_marker_array(markers, &debug_data_.stop_wall_marker);

  // update debug data
  debug_data_.stop_index = zero_vel_idx.value();
  debug_data_.stop_point = stop_point;

  // update planning factor
  const auto stop_pose = output_traj_points.at(zero_vel_idx.value()).pose;
  SafetyFactorArray safety_factors;
  if (determined_stop_obstacle.has_value()) {
    SafetyFactor safety_factor;
    safety_factor.type = SafetyFactor::OBJECT;
    safety_factor.object_id = determined_stop_obstacle.value().uuid;
    safety_factor.points = {determined_stop_obstacle.value().pose.position};
    safety_factor.is_safe = false;
    safety_factors.factors.push_back(safety_factor);
    safety_factors.is_safe = false;
    safety_factors.header.stamp = clock_->now();
    safety_factors.header.frame_id = "map";
  }

  planning_factor_interface_->add(
    output_traj_points, planner_data->current_odometry.pose.pose, stop_pose, PlanningFactor::STOP,
    safety_factors);

  prev_stop_distance_info_ = std::make_pair(output_traj_points, determined_zero_vel_dist.value());

  // verify stop point is on trajectory
  const auto dist_to_traj =
    autoware::motion_utils::calcLateralOffset(traj_points, stop_pose.position);
  if (std::abs(dist_to_traj) > 0.5) {
    RCLCPP_WARN(
      logger_, "Stop point lateral offset from trajectory: %.3f m at index %zu", dist_to_traj,
      zero_vel_idx.value());
  }

  return stop_pose.position;
}

double RoadUserStopModule::calc_desired_stop_margin(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_bumper, const size_t ego_segment_idx,
  const double dist_to_collide_on_ref_traj) const
{
  const auto param = param_listener_->get_params();
  // calculate default stop margin
  const double default_stop_margin = [&]() {
    const double v_ego = planner_data->current_odometry.twist.twist.linear.x;
    const double v_obs = stop_obstacle.velocity;

    const auto ref_traj_length =
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_points.size() - 1);

    // For negative velocity obstacles (approaching)
    if (v_obs < param.stop_planning.opposing_traffic.max_negative_velocity) {
      const double a_ego = param.stop_planning.opposing_traffic.effective_deceleration;
      const double & bumper_to_bumper_distance = stop_obstacle.dist_to_collide_on_decimated_traj;

      const double braking_distance = v_ego * v_ego / (2 * a_ego);
      const double stopping_time = v_ego / a_ego;
      const double distance_obs_ego_braking = std::abs(v_obs * stopping_time);

      const double ego_stop_margin = param.stop_planning.opposing_traffic.stop_margin;

      // NOTE: To avoid planning a sto position immediately in front of ego for an object that is
      //       still distant we enforce a lower speed limit in the coasting time calculation.
      const double v_ego_limitted =
        std::max(v_ego, param.stop_planning.opposing_traffic.min_velocity_for_stop_planning);
      const double rel_vel = v_ego_limitted - v_obs;

      constexpr double epsilon = 1e-6;  // Small threshold for numerical stability
      if (std::abs(rel_vel) <= epsilon) {
        RCLCPP_WARN(
          logger_,
          "Relative velocity (%.3f) is too close to zero. Using minimum safe value for "
          "calculation.",
          rel_vel);

        // Return default stop margin as fallback
        return param.stop_planning.longitudinal_margin.default_margin;
      }

      const double T_coast = std::max(
        (bumper_to_bumper_distance - ego_stop_margin - braking_distance -
         distance_obs_ego_braking) /
          rel_vel,
        0.0);

      const double distance_obs_ego_coasting = std::abs(v_obs * T_coast);

      const double stop_margin =
        ego_stop_margin + distance_obs_ego_braking + distance_obs_ego_coasting;

      return stop_margin;
    }

    if (dist_to_collide_on_ref_traj > ref_traj_length) {
      return param.stop_planning.longitudinal_margin.terminal_margin;
    }

    return param.stop_planning.longitudinal_margin.default_margin;
  }();

  // calculate stop margin on curve
  const double stop_margin_on_curve = calc_margin_from_obstacle_on_curve(
    planner_data, traj_points, stop_obstacle, dist_to_bumper, default_stop_margin);

  // calculate stop margin considering behavior's stop point
  // NOTE: If behavior stop point is ahead of the closest_obstacle_stop point within a certain
  //       margin we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const auto closest_behavior_stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(traj_points, ego_segment_idx + 1);
  if (closest_behavior_stop_idx.has_value()) {
    const double closest_behavior_stop_dist_on_ref_traj =
      autoware::motion_utils::calcSignedArcLength(
        traj_points, 0, closest_behavior_stop_idx.value());
    const double stop_dist_diff =
      closest_behavior_stop_dist_on_ref_traj - (dist_to_collide_on_ref_traj - stop_margin_on_curve);
    if (0.0 < stop_dist_diff && stop_dist_diff < stop_margin_on_curve) {
      return param.stop_planning.longitudinal_margin.minimum_margin;
    }
  }

  return stop_margin_on_curve;
}

std::optional<double> RoadUserStopModule::calc_candidate_zero_vel_dist(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const double dist_to_collide_on_ref_traj,
  const double desired_stop_margin, [[maybe_unused]] const double dist_to_bumper) const
{
  const auto param = param_listener_->get_params();
  double candidate_zero_vel_dist = std::max(0.0, dist_to_collide_on_ref_traj - desired_stop_margin);

  if (param.option.suppress_sudden_stop) {
    const auto acceptable_stop_acc = [&]() -> std::optional<double> {
      const double distance_to_judge_suddenness = std::min(
        calc_minimum_distance_to_stop(
          planner_data->current_odometry.twist.twist.linear.x, common_param_.limit_max_accel,
          param.stop_planning.sudden_object_acc_threshold),
        param.stop_planning.sudden_object_dist_threshold);

      if (candidate_zero_vel_dist > distance_to_judge_suddenness) {
        return common_param_.limit_min_accel;
      }

      if (param.stop_planning.abandon_to_stop) {
        RCLCPP_WARN(logger_, "[RoadUserStop] abandon to stop against object");
        return std::nullopt;
      } else {
        return param.stop_planning.limit_min_acc;
      }
    }();

    if (!acceptable_stop_acc.has_value()) {
      return std::nullopt;
    }

    const double acceptable_stop_pos =
      autoware::motion_utils::calcSignedArcLength(
        traj_points, 0, planner_data->current_odometry.pose.pose.position) +
      calc_minimum_distance_to_stop(
        planner_data->current_odometry.twist.twist.linear.x, common_param_.limit_max_accel,
        acceptable_stop_acc.value());

    if (acceptable_stop_pos > candidate_zero_vel_dist) {
      candidate_zero_vel_dist = acceptable_stop_pos;
    }
  }

  return candidate_zero_vel_dist;
}

std::vector<Polygon2d> RoadUserStopModule::get_trajectory_polygons(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (trajectory_polygon_for_inside_map_.count(lat_margin) == 0) {
    const auto traj_polys = polygon_utils::create_one_step_polygons(
      decimated_traj_points, vehicle_info, current_ego_pose, lat_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length);
    trajectory_polygon_for_inside_map_.emplace(lat_margin, traj_polys);
  }
  return trajectory_polygon_for_inside_map_.at(lat_margin);
}

double RoadUserStopModule::calc_margin_from_obstacle_on_curve(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
  const double dist_to_bumper, const double default_stop_margin) const
{
  const auto param = param_listener_->get_params();
  const bool enable_approaching_on_curve = param.stop_planning.stop_on_curve.enable_approaching;

  if (!enable_approaching_on_curve) {
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
      param.stop_planning.longitudinal_margin.default_margin + dist_to_bumper <
        sum_short_traj_length) {
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
        ego_pose, param.stop_planning.longitudinal_margin.default_margin + 3.0, 0.0, 0.0);
      const auto ego_straight_segment = autoware_utils_geometry::Segment2d{
        convert_point(ego_pose.position), convert_point(forward_ego_pose.position)};
      return boost::geometry::distance(ego_straight_segment, object_polygon);
    };
  const auto resampled_short_traj_points = resample_trajectory_points(short_traj_points, 0.5);
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
  if (!collision_idx.has_value()) {
    return param.stop_planning.stop_on_curve.min_stop_margin;
  }
  if (collision_idx.value() == 0) {
    return default_stop_margin;
  }

  // calculate margin from obstacle
  const double partial_segment_length = [&]() {
    const double collision_segment_length = autoware_utils_geometry::calc_distance2d(
      resampled_short_traj_points.at(collision_idx.value() - 1),
      resampled_short_traj_points.at(collision_idx.value()));
    const double prev_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(collision_idx.value() - 1).pose, object_polygon);
    const double next_dist = calculate_distance_from_straight_ego_path(
      resampled_short_traj_points.at(collision_idx.value()).pose, object_polygon);
    return (next_dist - planner_data->vehicle_info_.vehicle_width_m / 2.0) /
           (next_dist - prev_dist) * collision_segment_length;
  }();

  const double short_margin_from_obstacle =
    partial_segment_length +
    autoware::motion_utils::calcSignedArcLength(
      resampled_short_traj_points, collision_idx.value(), stop_obstacle.collision_point) -
    dist_to_bumper + param.stop_planning.stop_on_curve.additional_stop_margin;

  return std::min(
    default_stop_margin,
    std::max(param.stop_planning.stop_on_curve.min_stop_margin, short_margin_from_obstacle));
}

void RoadUserStopModule::publish_debug_info()
{
  autoware_utils_debug::ScopedTimeTrack st_debug(__func__, *time_keeper_);
  const auto debug_markers = create_debug_marker_array();
  debug_publisher_->publish(debug_markers);

  virtual_wall_publisher_->publish(debug_data_.stop_wall_marker);
}

}  // namespace autoware::motion_velocity_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::RoadUserStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
