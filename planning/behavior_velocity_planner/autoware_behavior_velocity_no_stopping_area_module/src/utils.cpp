// Copyright 2024 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace autoware::behavior_velocity_planner::no_stopping_area
{

void PassJudge::check_if_stoppable(
  const double distance_to_stop_point, const EgoData & ego_data, const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  // compute pass_judge_line_distance
  const double stoppable_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    ego_data.current_velocity, ego_data.current_acceleration, ego_data.max_stop_acc,
    ego_data.max_stop_jerk, ego_data.delay_response_time);
  const bool distance_stoppable = stoppable_distance < distance_to_stop_point;
  const bool slow_velocity = ego_data.current_velocity < 2.0;
  // ego vehicle is high speed and can't stop before stop line -> GO
  // stoppable or not is judged only once
  RCLCPP_DEBUG(
    logger, "stoppable_distance: %lf distance_to_stop_point: %lf", stoppable_distance,
    distance_to_stop_point);

  if (distance_stoppable || pass_judged) {
    return;
  }

  pass_judged = true;
  // can't stop using maximum brake consider jerk limit
  if (!slow_velocity) {
    // pass through
    is_stoppable = false;
    RCLCPP_WARN_THROTTLE(
      logger, clock, 1000, "[NoStoppingArea] can't stop in front of no stopping area");
  } else {
    is_stoppable = true;
  }
}

bool is_vehicle_type(const autoware_perception_msgs::msg::PredictedObject & object)
{
  // TODO(anyone): should we switch to using functions from the common object_recognition_utils ?
  return !object.classification.empty() &&
         (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::CAR ||
          object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::BUS ||
          object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
          object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::TRAILER ||
          object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE);
}

std::optional<LineString2d> generate_stop_line(
  const Trajectory & path, const lanelet::ConstPolygons3d & no_stopping_areas,
  const double ego_width, const double stop_line_margin)
{
  for (const auto & no_stopping_area : no_stopping_areas) {
    auto basic_polygon = lanelet::utils::to2D(no_stopping_area).basicPolygon();
    basic_polygon.push_back(basic_polygon.front());  // close polygon
    const auto collision_points = experimental::trajectory::crossed(path, basic_polygon);
    if (collision_points.empty()) {
      continue;
    }

    const auto closest_collision_point = collision_points.front();
    const auto point = path.compute(closest_collision_point).point.pose.position;
    const auto yaw = path.azimuth(closest_collision_point);
    const auto & w = ego_width;
    const auto & l = stop_line_margin;

    LineString2d stop_line;
    stop_line.emplace_back(
      -l * std::cos(yaw) + point.x + w * std::cos(yaw + M_PI_2),
      point.y + w * std::sin(yaw + M_PI_2));
    stop_line.emplace_back(
      -l * std::cos(yaw) + point.x + w * std::cos(yaw - M_PI_2),
      point.y + w * std::sin(yaw - M_PI_2));

    return stop_line;
  }

  return {};
}

std::optional<Polygon2d> generate_ego_no_stopping_area_lane_polygon(
  const Trajectory & path, const geometry_msgs::msg::Pose & ego_pose,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem, const double margin,
  const double max_polygon_length, const double path_expand_width)
{
  const auto ego_s =
    experimental::trajectory::find_first_nearest_index(path, ego_pose, 3.0, M_PI_4);
  if (!ego_s) {
    return std::nullopt;
  }

  // return if area size is not intentional
  if (no_stopping_area_reg_elem.noStoppingAreas().size() != 1) {
    return std::nullopt;
  }

  const auto no_stopping_area = no_stopping_area_reg_elem.noStoppingAreas().front();
  const auto ego_area_intervals = experimental::trajectory::find_intervals(
    path, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p) {
      const auto & pos = p.point.pose.position;
      return bg::covered_by(
        Point2d{pos.x, pos.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon());
    });
  if (ego_area_intervals.empty()) {
    return std::nullopt;
  }

  const auto [ego_area_start_s, ego_area_end_s] = ego_area_intervals.front();
  if (ego_area_start_s - *ego_s > max_polygon_length) {
    return std::nullopt;
  }

  constexpr auto interpolation_interval = 0.5;
  const auto ego_area_bases = path.base_arange(
    {std::max({ego_area_start_s, *ego_s}),
     std::min({ego_area_end_s + margin, ego_area_start_s + max_polygon_length, path.length()})},
    interpolation_interval);

  Polygon2d ego_area;

  const auto add_edge = [&](const auto begin, const auto end, const double offset) {
    for (auto it = begin; it != end; ++it) {
      const auto pose = path.compute(*it).point.pose;
      const double yaw = tf2::getYaw(pose.orientation);
      const double x = pose.position.x + offset * std::sin(yaw);
      const double y = pose.position.y - offset * std::cos(yaw);
      ego_area.outer().emplace_back(x, y);
    }
  };

  add_edge(ego_area_bases.begin(), ego_area_bases.end(), path_expand_width);
  add_edge(ego_area_bases.rbegin(), ego_area_bases.rend(), -path_expand_width);

  bg::correct(ego_area);

  return ego_area;
}

bool check_stop_lines_in_no_stopping_area(
  const Trajectory & path, const Polygon2d & poly, no_stopping_area::DebugData & debug_data)
{
  const double stop_vel = std::numeric_limits<float>::min();

  const auto not_stopping_intervals = experimental::trajectory::find_intervals(
    path, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p) {
      return p.point.longitudinal_velocity_mps <= stop_vel;
    });

  // if the detected stop point is near goal, it's ignored.
  static constexpr double close_to_goal_distance = 1.0;

  for (const auto & [start, end] : not_stopping_intervals) {
    if (start > std::min(end, path.length() - close_to_goal_distance)) {
      continue;
    }

    const auto cropped_path = experimental::trajectory::crop(
      path, start, std::min(end, path.length() - close_to_goal_distance));
    const auto collision_points = experimental::trajectory::crossed(cropped_path, poly.outer());
    if (collision_points.empty()) {
      continue;
    }

    debug_data.stuck_points.push_back(
      cropped_path.compute(collision_points.front()).point.pose.position);
    return true;
  }

  return false;
}

std::optional<autoware_utils::LineString2d> get_stop_line(
  const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  const double stop_line_margin, const double vehicle_width)
{
  const auto & custom_stop_line = no_stopping_area_reg_elem.stopLine();
  if (custom_stop_line && custom_stop_line->size() >= 2) {
    // get stop line from map
    return planning_utils::extendSegmentToBounds(
      lanelet::utils::to2D(custom_stop_line->basicLineString()), left_bound, right_bound);
  }

  // generate stop line from no stopping area
  return generate_stop_line(
    path, no_stopping_area_reg_elem.noStoppingAreas(), vehicle_width, stop_line_margin);
}

std::optional<double> get_stop_point(
  const Trajectory & path, const autoware_utils::LineString2d & stop_line, const double margin,
  const double vehicle_offset, const lanelet::Ids & lane_ids)
{
  const auto collision_points = experimental::trajectory::crossed_with_constraint(
    path, stop_line, [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p) {
      return lane_ids.empty() ||
             std::any_of(p.lane_ids.begin(), p.lane_ids.end(), [&](const lanelet::Id id) {
               return std::find(lane_ids.begin(), lane_ids.end(), id) != lane_ids.end();
             });
    });

  if (collision_points.empty()) {
    return std::nullopt;
  }

  return collision_points.front() - margin - vehicle_offset;
}

bool is_stoppable(
  PassJudge & pass_judge, const geometry_msgs::msg::Pose & self_pose,
  const geometry_msgs::msg::Pose & line_pose, const EgoData & ego_data,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
{
  // compute pass_judge_line_distance
  const double stoppable_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    ego_data.current_velocity, ego_data.current_acceleration, ego_data.max_stop_acc,
    ego_data.max_stop_jerk, ego_data.delay_response_time);
  const double signed_arc_length =
    arc_lane_utils::calcSignedDistance(self_pose, line_pose.position);
  const bool distance_stoppable = stoppable_distance < signed_arc_length;
  const bool slow_velocity = ego_data.current_velocity < 2.0;
  // ego vehicle is high speed and can't stop before stop line -> GO
  // stoppable or not is judged only once
  RCLCPP_DEBUG(
    logger, "stoppable_dist: %lf signed_arc_length: %lf", stoppable_distance, signed_arc_length);
  if (!distance_stoppable && !pass_judge.pass_judged) {
    pass_judge.pass_judged = true;
    // can't stop using maximum brake consider jerk limit
    if (!slow_velocity) {
      // pass through
      pass_judge.is_stoppable = false;
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000, "[NoStoppingArea] can't stop in front of no stopping area");
    } else {
      pass_judge.is_stoppable = true;
    }
  }
  return pass_judge.is_stoppable;
}

}  // namespace autoware::behavior_velocity_planner::no_stopping_area
