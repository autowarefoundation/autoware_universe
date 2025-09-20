// Copyright 2025 Tier IV, Inc.
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

#include "autoware/behavior_velocity_blind_spot_module/time_to_collision.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>  // for smoothPath
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/reverse.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
namespace
{

bool has_lane_ids(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p, const lanelet::Id id)
{
  for (const auto & pid : p.lane_ids) {
    if (pid == id) {
      return true;
    }
  }
  return false;
}

}  // namespace

namespace autoware::behavior_velocity_planner
{

static std::vector<FuturePosition> calculate_future_profile_impl(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & current_pose, const double minimum_default_velocity,
  const double time_to_restart, const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
{
  const auto closest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  double passing_time = time_to_restart;

  std::vector<FuturePosition> future_positions;
  future_positions.emplace_back(FuturePosition{path.points.at(closest_idx), passing_time});
  for (unsigned i = closest_idx + 1; i + 1 < path.points.size(); ++i) {
    const auto & p1 = path.points.at(i);
    const auto & p2 = path.points.at(i + 1);
    const double dist = autoware_utils_geometry::calc_distance2d(p1, p2);
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    const double passing_velocity = std::max(average_velocity, minimum_default_velocity);
    passing_time += dist / passing_velocity;

    future_positions.emplace_back(FuturePosition{path.points.at(i), passing_time});
  }
  return future_positions;
}

std::vector<FuturePosition> calculate_future_profile(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const double minimum_default_velocity, const double time_to_restart,
  const std::shared_ptr<const PlannerData> & planner_data, const lanelet::Id lane_id)
{
  const double nearest_dist_threshold = planner_data->ego_nearest_dist_threshold;
  const double nearest_yaw_threshold = planner_data->ego_nearest_yaw_threshold;
  const auto & current_pose = planner_data->current_odometry->pose;
  const double current_velocity = planner_data->current_velocity->twist.linear.x;

  const auto closest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);

  autoware_internal_planning_msgs::msg::PathWithLaneId reference_path;
  bool assigned_lane_found = false;
  for (unsigned i = 0; i < path.points.size(); ++i) {
    auto reference_point = path.points.at(i);
    // assume backward velocity is current ego velocity
    if (i < closest_idx) {
      reference_point.point.longitudinal_velocity_mps = static_cast<float>(current_velocity);
    }
    reference_path.points.push_back(reference_point);
    const bool has_objective_lane_id = has_lane_ids(path.points.at(i), lane_id);
    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (reference_path.points.size() < 3 || !assigned_lane_found) {
    return {};
  }
  auto smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data)) {
    return {};
  }
  return calculate_future_profile_impl(
    smoothed_reference_path, current_pose, minimum_default_velocity, time_to_restart,
    nearest_dist_threshold, nearest_yaw_threshold);
}

std::optional<std::pair<double, double>> compute_time_interval_for_passing_line(
  const std::vector<FuturePosition> & future_positions,
  const autoware_utils_geometry::LinearRing2d & footprint, const lanelet::ConstLineString3d & line1,
  const lanelet::ConstLineString3d & line2)
{
  // search forward
  std::optional<double> entry_time{};
  for (const auto & [path_point, time] : future_positions) {
    const auto & base_pose = path_point.point.pose;
    const auto path_point_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(base_pose));
    if (boost::geometry::intersects(
          path_point_footprint, lanelet::utils::to2D(line1).basicLineString())) {
      entry_time = time;
      break;
    }
  }
  if (!entry_time) {
    return std::nullopt;
  }

  // search backward
  std::optional<double> exit_time{};
  for (const auto & [path_point, time] : future_positions | ranges::views::reverse) {
    const auto & base_pose = path_point.point.pose;
    const auto path_point_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(base_pose));
    if (boost::geometry::intersects(
          path_point_footprint, lanelet::utils::to2D(line2).basicLineString())) {
      exit_time = time;
      break;
    }
  }
  if (!exit_time) {
    return std::nullopt;
  }

  return std::make_optional<std::pair<double, double>>(entry_time.value(), exit_time.value());
}

std::vector<std::tuple<double, double, autoware_perception_msgs::msg::PredictedPath>>
compute_time_interval_for_passing_line(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLineString3d & line1, const lanelet::ConstLineString3d & entry_line,
  const lanelet::ConstLineString3d & line2)
{
  std::vector<std::tuple<double, double, autoware_perception_msgs::msg::PredictedPath>>
    passage_time_intervals;

  const auto line1_2d = lanelet::utils::to2D(line1).basicLineString();
  const auto entry_line_2d = lanelet::utils::to2D(entry_line).basicLineString();
  const auto line2_2d = lanelet::utils::to2D(line2).basicLineString();

  for (const auto & predicted_path : object.kinematics.predicted_paths) {
    if (predicted_path.path.size() < 2) {
      continue;
    }
    const double time_step = predicted_path.time_step.sec + predicted_path.time_step.nanosec * 1e-9;
    const double horizon = time_step * static_cast<double>(predicted_path.path.size());
    static constexpr double new_time_step = 0.1;
    const auto precise_predicted_path = autoware::object_recognition_utils::resamplePredictedPath(
      predicted_path, new_time_step, horizon);
    const auto & shape = object.shape;

    // search forward
    std::optional<double> entry_time{};
    for (const auto & [i, pose] : ranges::views::enumerate(precise_predicted_path.path)) {
      const auto object_poly = autoware_utils_geometry::to_polygon2d(pose, shape);
      if (boost::geometry::intersects(object_poly, line1_2d)) {
        entry_time = i * new_time_step;
        break;
      } else if (boost::geometry::intersects(object_poly, entry_line_2d)) {
        entry_time = i * new_time_step;
        break;
      }
    }
    if (!entry_time) {
      continue;
    }

    // search backward
    std::optional<double> exit_time{};
    for (const auto & [i, pose] :
         ranges::views::enumerate(precise_predicted_path.path | ranges::views::reverse)) {
      const auto object_poly = autoware_utils_geometry::to_polygon2d(pose, shape);
      const double time = horizon - i * new_time_step;
      if (entry_time && time < entry_time.value()) {
        break;
      }
      if (boost::geometry::intersects(object_poly, line2_2d)) {
        exit_time = time;
        break;
      }
    }
    if (!exit_time) {
      continue;
    }
    // in case the object is completely inside conflict_area, it is regarded entry_time = 0.0
    passage_time_intervals.emplace_back(entry_time.value(), exit_time.value(), predicted_path);
  }
  return passage_time_intervals;
}

autoware_internal_planning_msgs::msg::SafetyFactor UnsafeObject::to_safety_factor() const
{
  autoware_internal_planning_msgs::msg::SafetyFactor factor;
  factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
  factor.object_id = object.object_id;
  factor.predicted_path = predicted_path;
  factor.ttc_begin = std::get<0>(object_passage_interval);
  factor.ttc_end = std::get<1>(object_passage_interval);
  factor.points.push_back(object.kinematics.initial_pose_with_covariance.pose.position);
  factor.is_safe = false;
  return factor;
}

std::optional<double> get_unsafe_time_if_critical(
  const std::pair<double, double> & ego_passage_interval,
  const std::pair<double, double> & object_passage_interval, const double ttc_start_margin,
  const double ttc_end_margin)
{
  const auto & [ego_entry, ego_exit] = ego_passage_interval;
  const auto & [object_entry, object_exit] = object_passage_interval;
  // case0: object will be gone far away from conflict_area when ego enters conflict_area, even if
  // object's exit is delayed by ttc_end_margin due to deceleration
  if (ego_entry > object_exit + ttc_end_margin) {
    return std::nullopt;
  }
  // case1: ego will be still in conflict_area, when the object enters conflict_area
  if (object_entry - ttc_start_margin < ego_exit) {
    return object_entry;
  }
  // case2: object is still in conflict_area, if ego had entered conflict_area
  if (ego_entry - ttc_end_margin < object_exit) {
    return ego_entry;
  }
  return std::nullopt;
}

}  // namespace autoware::behavior_velocity_planner
