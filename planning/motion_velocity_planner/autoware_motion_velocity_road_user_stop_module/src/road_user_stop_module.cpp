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

  // Get vehicle information
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  vehicle_front_offset_ = vehicle_info.max_longitudinal_offset_m;

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
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    smoothed_trajectory_points,
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
  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  predicted_objects.header = planner_data->predicted_objects_header;

  for (const auto & object_ptr : planner_data->objects) {
    if (object_ptr) {
      predicted_objects.objects.push_back(object_ptr->predicted_object);
    }
  }

  updateTrackedObjects(predicted_objects, current_time);

  // 2. get relevant lanelets
  const auto relevant_lanelets = getRelevantLanelets(planner_data);
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

  // 3. filter each object

  // TODO(odashima):
  // Calculate all potential stop_points first, sort by proximity, and then assign the stop_point to
  // the first object determined to be stoppable. This prevents missing later objects that could
  // have been stopped if only one stop_point was considered initially.
  std::optional<geometry_msgs::msg::Point> earliest_stop_point;
  double min_stop_distance = std::numeric_limits<double>::max();

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
    // if (params_.in_place_stop.enable) {
    //   for (const auto & lanelet : relevant_lanelets) {
    //     if (isWrongWayUser(object, lanelet)) {
    //       is_wrong_way = true;
    //       // has_wrong_way_user = true;
    //       break;
    //     }
    //   }
    // }

    // debugging for only enable isWrongWayUser
    // if (!is_wrong_way) {
    //   continue;
    // }

    if (is_wrong_way) {
      // TODO(odashima): change to stop gradually instead of immediate stop
      const auto & current_pos = planner_data->current_odometry.pose.pose.position;

      size_t stop_idx = 0;
      double min_dist = std::numeric_limits<double>::max();

      for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const double dist =
          autoware::universe_utils::calcDistance2d(current_pos, trajectory_points[i].pose.position);
        if (dist < min_dist) {
          min_dist = dist;
          stop_idx = i;
        }
      }

      if (stop_idx == 0 || min_dist < 1.0) {
        min_stop_distance = 0.0;
        earliest_stop_point = trajectory_points[0].pose.position;
        debug_data_.stop_index = 0;
        debug_data_.stop_point = earliest_stop_point;
        debug_data_.stop_target_object = object;
      }
    } else {
      // Normal stop point calculation for non-wrong-way users
      const auto stop_index = calculateStopPointWithMargin(trajectory_points, object);

      if (stop_index) {
        const double stop_distance = autoware::motion_utils::calcSignedArcLength(
          trajectory_points, planner_data->current_odometry.pose.pose.position,
          trajectory_points[*stop_index].pose.position);

        if (stop_distance < min_stop_distance) {
          min_stop_distance = stop_distance;
          earliest_stop_point = trajectory_points[*stop_index].pose.position;
          debug_data_.stop_index = stop_index;
          debug_data_.stop_point = earliest_stop_point;
          debug_data_.stop_target_object = object;  // Store the object causing stop
        }
      }
    }
  }

  // 4. apply stop if needed
  if (earliest_stop_point) {
    const double current_velocity = planner_data->current_odometry.twist.twist.linear.x;
    const double required_decel =
      calculateRequiredDeceleration(current_velocity, min_stop_distance);

    // only stop if deceleration is acceptable
    if (std::abs(required_decel) <= params_.stop_decision.max_deceleration) {
      result.stop_points.push_back(*earliest_stop_point);

      // add planning factor
      if (debug_data_.stop_index && planning_factor_interface_) {
        geometry_msgs::msg::Pose stop_pose;
        stop_pose.position = *earliest_stop_point;
        stop_pose.orientation = trajectory_points[*debug_data_.stop_index].pose.orientation;

        using SafetyFactorArray = autoware_internal_planning_msgs::msg::SafetyFactorArray;
        using SafetyFactor = autoware_internal_planning_msgs::msg::SafetyFactor;
        SafetyFactorArray safety_factors;
        safety_factors.header.stamp = clock_->now();
        safety_factors.header.frame_id = "map";
        safety_factors.is_safe = false;

        // add safety factor if we have the object that caused the stop
        if (debug_data_.stop_target_object) {
          SafetyFactor safety_factor;
          safety_factor.type = SafetyFactor::OBJECT;
          safety_factor.object_id = debug_data_.stop_target_object->object_id;
          safety_factor.points.push_back(
            debug_data_.stop_target_object->kinematics.initial_pose_with_covariance.pose.position);

          safety_factors.factors.push_back(safety_factor);
        }

        planning_factor_interface_->add(
          trajectory_points, planner_data->current_odometry.pose.pose, stop_pose,
          autoware::planning_factor_interface::PlanningFactor::STOP, safety_factors, true,
          0.0 /* velocity */, 0.0 /* shift_length */);
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

bool RoadUserStopModule::isTargetObject(
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  using autoware_perception_msgs::msg::ObjectClassification;

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
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::LaneletMapPtr & /* lanelet_map */,
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
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLanelet & lanelet) const
{
  // TODO(odashima): determine if an object is stopped or in reverse by looking at its speed.
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

  const bool is_wrong_awy_user = angle_diff > params_.in_place_stop.wrong_way_angle_threshold;

  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 1000,
    "Object %s is %sway user (angle diff: %.2f degrees, threshold: %.2f degrees)",
    autoware::universe_utils::toHexString(object.object_id).c_str(),
    is_wrong_awy_user ? "wrong" : "correct", angle_diff,
    params_.in_place_stop.wrong_way_angle_threshold);

  return is_wrong_awy_user;
}

lanelet::ConstLanelets RoadUserStopModule::getRelevantLanelets(
  const std::shared_ptr<const PlannerData> planner_data) const
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

    // store adjacent lanelets for debug visualization
    debug_data_.adjacent_lanelets = adjacent_lanelets;

    // add unique adjacent lanelets
    for (const auto & adj_llt : adjacent_lanelets) {
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
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const autoware_perception_msgs::msg::PredictedObject & object) const
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
  const double stop_margin_from_base_link = stop_margin_from_bumper + vehicle_front_offset_;

  RCLCPP_DEBUG(
    logger_,
    "Stop margin calculation: bumper_margin=%.2fm, vehicle_front_offset=%.2fm, total=%.2fm",
    stop_margin_from_bumper, vehicle_front_offset_, stop_margin_from_base_link);

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

void RoadUserStopModule::updateTrackedObjects(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const rclcpp::Time & current_time)
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

}  // namespace autoware::motion_velocity_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::RoadUserStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
