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

#include "time_to_collision.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <range/v3/all.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

static std::vector<FuturePose> calculate_future_profile_impl(
  const Trajectory & path, const double minimum_default_velocity, const double time_to_restart)
{
  std::vector<FuturePose> future_profile;
  auto passing_time = time_to_restart;

  const auto bases = path.get_underlying_bases();
  for (auto it = bases.begin(); it != std::prev(bases.end()); ++it) {
    const auto p1 = path.compute(*it);
    const auto p2 = path.compute(*std::next(it));
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2;
    const auto passing_velocity = std::max(average_velocity, minimum_default_velocity);
    passing_time += (*std::next(it) - *it) / passing_velocity;

    future_profile.push_back(FuturePose{p1.point.pose, passing_time});
  }

  return future_profile;
}

std::vector<FuturePose> calculate_future_profile(
  const Trajectory & path, const double minimum_default_velocity, const double time_to_restart,
  const PlannerData & planner_data, const lanelet::Id lane_id)
{
  const auto & nearest_dist_threshold = planner_data.ego_nearest_dist_threshold;
  const auto & nearest_yaw_threshold = planner_data.ego_nearest_yaw_threshold;
  const auto & current_pose = planner_data.current_odometry->pose;
  const auto & current_velocity = planner_data.current_velocity->twist.linear.x;

  const auto current_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  if (!current_s) {
    return {};
  }

  const auto intervals_on_objective_lane = autoware::experimental::trajectory::find_intervals(
    path, [lane_id](const PathPointWithLaneId & p) {
      return std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end();
    });
  if (intervals_on_objective_lane.empty()) {
    return {};
  }

  auto reference_path =
    autoware::experimental::trajectory::crop(path, 0, intervals_on_objective_lane.front().end);
  reference_path.longitudinal_velocity_mps().range(0, *current_s).set(current_velocity);

  PathWithLaneId reference_path_msg;
  reference_path_msg.points = reference_path.restore();

  PathWithLaneId smoothed_reference_path_msg;
  if (!smoothPath(reference_path_msg, smoothed_reference_path_msg, planner_data)) {
    return {};
  }
  auto smoothed_reference_path =
    autoware::experimental::trajectory::pretty_build(smoothed_reference_path_msg.points);
  if (!smoothed_reference_path) {
    return {};
  }

  smoothed_reference_path->crop(*current_s, smoothed_reference_path->length());

  return calculate_future_profile_impl(
    *smoothed_reference_path, minimum_default_velocity, time_to_restart);
}

std::optional<TimeInterval> compute_passage_time_interval(
  const std::vector<FuturePose> & future_profile,
  const autoware_utils_geometry::LinearRing2d & footprint,
  const lanelet::ConstLineString3d & entry_line, const lanelet::ConstLineString3d & exit_line)
{
  // search forward
  std::optional<double> entry_time{};
  for (const auto & [pose, time] : future_profile) {
    const auto path_point_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(pose));
    if (boost::geometry::intersects(
          path_point_footprint, lanelet::utils::to2D(entry_line).basicLineString())) {
      entry_time = time;
      break;
    }
  }
  if (!entry_time) {
    return std::nullopt;
  }

  // search backward
  std::optional<double> exit_time{};
  for (const auto & [pose, time] : future_profile | ranges::views::reverse) {
    const auto path_point_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(pose));
    if (boost::geometry::intersects(
          path_point_footprint, lanelet::utils::to2D(exit_line).basicLineString())) {
      exit_time = time;
      break;
    }
  }
  if (!exit_time) {
    return std::nullopt;
  }

  return TimeInterval{*entry_time, *exit_time};
}

std::vector<std::optional<TimeInterval>> compute_passage_time_intervals(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLineString3d & line1, const lanelet::ConstLineString3d & entry_line,
  const lanelet::ConstLineString3d & line2)
{
  std::vector<std::optional<TimeInterval>> passage_time_intervals(
    object.kinematics.predicted_paths.size(), std::nullopt);

  for (size_t i = 0; i < object.kinematics.predicted_paths.size(); ++i) {
    const auto & predicted_path = object.kinematics.predicted_paths[i];
    if (predicted_path.path.size() < 2) {
      continue;
    }

    const auto time_step = rclcpp::Duration{predicted_path.time_step}.seconds();
    const auto horizon = time_step * predicted_path.path.size();

    static constexpr auto new_time_step = 0.1;
    const auto precise_predicted_path = autoware::object_recognition_utils::resamplePredictedPath(
      predicted_path, new_time_step, horizon);
    const auto & shape = object.shape;

    // search forward
    std::optional<double> entry_time = std::nullopt;
    for (const auto & [i, pose] : ranges::views::enumerate(precise_predicted_path.path)) {
      const auto object_poly = autoware_utils_geometry::to_polygon2d(pose, shape);
      if (boost::geometry::intersects(object_poly, lanelet::utils::to2D(line1).basicLineString())) {
        entry_time = i * new_time_step;
        break;
      } else if (boost::geometry::intersects(
                   object_poly, lanelet::utils::to2D(entry_line).basicLineString())) {
        entry_time = i * new_time_step;
        break;
      }
    }
    if (!entry_time) {
      continue;
    }

    // search backward
    std::optional<double> exit_time = std::nullopt;
    for (const auto & [i, pose] :
         ranges::views::enumerate(precise_predicted_path.path | ranges::views::reverse)) {
      const auto object_poly = autoware_utils_geometry::to_polygon2d(pose, shape);
      const auto time = horizon - i * new_time_step;
      if (entry_time && time < *entry_time) {
        break;
      }
      if (boost::geometry::intersects(object_poly, lanelet::utils::to2D(line2).basicLineString())) {
        exit_time = time;
        break;
      }
    }
    if (!exit_time) {
      continue;
    }

    // in case the object is completely inside conflict_area, it is regarded entry_time = 0.0
    passage_time_intervals[i] = {*entry_time, *exit_time};
  }

  return passage_time_intervals;
}

std::optional<double> get_unsafe_time_if_critical(
  const TimeInterval & ego_passage_interval, const TimeInterval & object_passage_interval,
  const double ttc_start_margin, const double ttc_end_margin)
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

}  // namespace autoware::behavior_velocity_planner::experimental
