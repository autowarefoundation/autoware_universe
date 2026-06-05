// Copyright 2021 TIER IV, Inc.
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

#include "autoware/map_based_prediction/predictor_vehicle/object_processing.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/tracked_object_kinematics.hpp>

#include <lanelet2_routing/RoutingGraph.h>

#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

namespace
{
double FirstOrderLowpassFilter(
  const double prev_y, const double prev_x, const double x, const double sampling_time = 0.1,
  const double cutoff_freq = 0.1)
{
  const double wt = 2.0 * M_PI * cutoff_freq * sampling_time;
  const double a = (2.0 - wt) / (2.0 + wt);
  const double b = wt / (2.0 + wt);
  return a * prev_y + b * (prev_x + x);
}

double calcAbsLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = autoware_utils::create_point(x, y, 0.0);
  }
  return std::fabs(autoware::motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

LateralKinematicsToLanelet initLateralKinematics(
  const lanelet::ConstLanelet & lanelet, geometry_msgs::msg::Pose pose)
{
  LateralKinematicsToLanelet lateral_kinematics;
  const lanelet::ConstLineString2d left_bound = lanelet.leftBound2d();
  const lanelet::ConstLineString2d right_bound = lanelet.rightBound2d();
  const double left_dist = calcAbsLateralOffset(left_bound, pose);
  const double right_dist = calcAbsLateralOffset(right_bound, pose);
  lateral_kinematics.dist_from_left_boundary = left_dist;
  lateral_kinematics.dist_from_right_boundary = right_dist;
  lateral_kinematics.left_lateral_velocity = 0;
  lateral_kinematics.right_lateral_velocity = 0;
  lateral_kinematics.filtered_left_lateral_velocity = 0;
  lateral_kinematics.filtered_right_lateral_velocity = 0;
  return lateral_kinematics;
}

void calcLateralKinematics(
  const LateralKinematicsToLanelet & prev_lateral_kinematics,
  LateralKinematicsToLanelet & current_lateral_kinematics, const double dt, const double cutoff)
{
  current_lateral_kinematics.left_lateral_velocity =
    (current_lateral_kinematics.dist_from_left_boundary -
     prev_lateral_kinematics.dist_from_left_boundary) /
    dt;
  current_lateral_kinematics.right_lateral_velocity =
    (current_lateral_kinematics.dist_from_right_boundary -
     prev_lateral_kinematics.dist_from_right_boundary) /
    dt;
  current_lateral_kinematics.filtered_left_lateral_velocity = FirstOrderLowpassFilter(
    prev_lateral_kinematics.filtered_left_lateral_velocity,
    prev_lateral_kinematics.left_lateral_velocity, current_lateral_kinematics.left_lateral_velocity,
    dt, cutoff);
  current_lateral_kinematics.filtered_right_lateral_velocity = FirstOrderLowpassFilter(
    prev_lateral_kinematics.filtered_right_lateral_velocity,
    prev_lateral_kinematics.right_lateral_velocity,
    current_lateral_kinematics.right_lateral_velocity, dt, cutoff);
}

void updateLateralKinematicsVector(
  const RoadUser & prev_obj, RoadUser & current_obj,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr_, const double lowpass_cutoff)
{
  const double dt = (current_obj.header.stamp.sec - prev_obj.header.stamp.sec) +
                    (current_obj.header.stamp.nanosec - prev_obj.header.stamp.nanosec) * 1e-9;
  if (dt < 1e-6) {
    return;
  }
  for (auto & current_set : current_obj.lateral_kinematics_set) {
    const auto & current_lane = current_set.first;
    auto & current_lateral_kinematics = current_set.second;
    if (prev_obj.lateral_kinematics_set.count(current_lane) != 0) {
      const auto & prev_lateral_kinematics = prev_obj.lateral_kinematics_set.at(current_lane);
      calcLateralKinematics(
        prev_lateral_kinematics, current_lateral_kinematics, dt, lowpass_cutoff);
      break;
    }
    for (auto & prev_set : prev_obj.lateral_kinematics_set) {
      const auto & prev_lane = prev_set.first;
      const auto & prev_lateral_kinematics = prev_set.second;
      const bool successive_lanelet =
        routing_graph_ptr_->routingRelation(prev_lane, current_lane) ==
        lanelet::routing::RelationType::Successor;
      if (successive_lanelet) {
        calcLateralKinematics(
          prev_lateral_kinematics, current_lateral_kinematics, dt, lowpass_cutoff);
        break;
      }
    }
  }
}

lanelet::ConstLanelets getLanelets(const map_based_prediction::LaneletsData & data)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & lanelet_data : data) {
    lanelets.push_back(lanelet_data.lanelet);
  }
  return lanelets;
}
}  // namespace

ObjectTracker::ObjectTracker(rclcpp::Node & node) : node_(node)
{
}

void ObjectTracker::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = time_keeper_ptr;
}

void ObjectTracker::setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
}

void ObjectTracker::setRoutingGraph(
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr)
{
  routing_graph_ptr_ = routing_graph_ptr;
}

void ObjectTracker::setParams(const Params & params)
{
  params_ = params;
}

void ObjectTracker::removeOldHistory(double current_time, double buffer_time)
{
  utils::removeOldObjectsHistory(current_time, buffer_time, road_users_history_);
}

void ObjectTracker::updateObjectData(TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE) {
    return;
  }
  if (object.kinematics.twist_with_covariance.twist.linear.x >= 0.0) return;
  const double abs_object_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  constexpr double min_abs_speed = 1e-1;
  if (abs_object_speed < min_abs_speed) return;
  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN) {
    const auto original_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    object.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(autoware_utils::pi + original_yaw);
    object.kinematics.twist_with_covariance.twist.linear.x *= -1.0;
    object.kinematics.twist_with_covariance.twist.linear.y *= -1.0;
  }
}

LaneletsData ObjectTracker::getCurrentLanelets(const TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  return utils::getCurrentLanelets(
    object, lanelet_map_ptr_, road_users_history_, params_.dist_threshold_for_searching_lanelet,
    params_.delta_yaw_threshold_for_searching_lanelet, params_.sigma_lateral_offset,
    params_.sigma_yaw_angle_deg);
}

void ObjectTracker::updateRoadUsersHistory(
  const std_msgs::msg::Header & header, const TrackedObject & object,
  const LaneletsData & current_lanelets_data)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::string object_id = autoware_utils::to_hex_string(object.object_id);
  const auto current_lanelets = getLanelets(current_lanelets_data);

  RoadUser road_user;
  road_user.header = header;
  road_user.current_lanelets = current_lanelets;
  road_user.future_possible_lanelets = current_lanelets;
  road_user.pose = object.kinematics.pose_with_covariance.pose;
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  road_user.pose.orientation = autoware_utils::create_quaternion_from_yaw(object_yaw);
  road_user.time_delay = std::fabs((node_.get_clock()->now() - header.stamp).seconds());
  road_user.twist = object.kinematics.twist_with_covariance.twist;

  for (const auto & current_lane : current_lanelets) {
    const LateralKinematicsToLanelet lateral_kinematics =
      initLateralKinematics(current_lane, road_user.pose);
    road_user.lateral_kinematics_set[current_lane] = lateral_kinematics;
  }

  if (road_users_history_.count(object_id) == 0) {
    road_users_history_.emplace(object_id, std::deque<RoadUser>({road_user}));
  } else {
    std::deque<RoadUser> & road_users = road_users_history_.at(object_id);
    const auto prev_road_user = road_users.back();
    updateLateralKinematicsVector(
      prev_road_user, road_user, routing_graph_ptr_, params_.cutoff_freq_of_velocity_lpf);
    road_users.push_back(road_user);
  }
}

}  // namespace autoware::map_based_prediction
