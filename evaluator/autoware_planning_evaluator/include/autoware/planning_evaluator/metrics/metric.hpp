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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS__METRIC_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS__METRIC_HPP_

#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning_diagnostics
{
/**
 * @brief Enumeration of metrics to publish
 */
enum class Metric {
  curvature,
  point_interval,
  relative_angle,
  resampled_relative_angle,
  length,
  duration,
  velocity,
  acceleration,
  jerk,
  lateral_deviation,
  yaw_deviation,
  velocity_deviation,
  lateral_trajectory_displacement_local,
  lateral_trajectory_displacement_lookahead,
  stability,
  stability_frechet,
  obstacle_distance,
  obstacle_ttc,
  modified_goal_longitudinal_deviation,
  modified_goal_lateral_deviation,
  modified_goal_yaw_deviation,
  stop_decision,
  abnormal_stop_decision,
  blinker_change_count,
  steer_change_count,
  SIZE,
};

/** TODO(Maxime CLEMENT):
 * make the addition of metrics simpler, e.g. with some macro ADD_METRIC(metric, metric_description)
 */
static const std::unordered_map<std::string, Metric> str_to_metric = {
  {"curvature", Metric::curvature},
  {"point_interval", Metric::point_interval},
  {"relative_angle", Metric::relative_angle},
  {"resampled_relative_angle", Metric::resampled_relative_angle},
  {"length", Metric::length},
  {"duration", Metric::duration},
  {"velocity", Metric::velocity},
  {"acceleration", Metric::acceleration},
  {"jerk", Metric::jerk},
  {"lateral_deviation", Metric::lateral_deviation},
  {"yaw_deviation", Metric::yaw_deviation},
  {"velocity_deviation", Metric::velocity_deviation},
  {"lateral_trajectory_displacement_local", Metric::lateral_trajectory_displacement_local},
  {"lateral_trajectory_displacement_lookahead", Metric::lateral_trajectory_displacement_lookahead},
  {"stability", Metric::stability},
  {"stability_frechet", Metric::stability_frechet},
  {"obstacle_distance", Metric::obstacle_distance},
  {"obstacle_ttc", Metric::obstacle_ttc},
  {"modified_goal_longitudinal_deviation", Metric::modified_goal_longitudinal_deviation},
  {"modified_goal_lateral_deviation", Metric::modified_goal_lateral_deviation},
  {"modified_goal_yaw_deviation", Metric::modified_goal_yaw_deviation},
  {"stop_decision", Metric::stop_decision},
  {"abnormal_stop_decision", Metric::abnormal_stop_decision},
  {"blinker_change_count", Metric::blinker_change_count},
  {"steer_change_count", Metric::steer_change_count}};

static const std::unordered_map<Metric, std::string> metric_to_str = {
  {Metric::curvature, "curvature"},
  {Metric::point_interval, "point_interval"},
  {Metric::relative_angle, "relative_angle"},
  {Metric::resampled_relative_angle, "resampled_relative_angle"},
  {Metric::length, "length"},
  {Metric::duration, "duration"},
  {Metric::velocity, "velocity"},
  {Metric::acceleration, "acceleration"},
  {Metric::jerk, "jerk"},
  {Metric::lateral_deviation, "lateral_deviation"},
  {Metric::yaw_deviation, "yaw_deviation"},
  {Metric::velocity_deviation, "velocity_deviation"},
  {Metric::lateral_trajectory_displacement_local, "lateral_trajectory_displacement_local"},
  {Metric::lateral_trajectory_displacement_lookahead, "lateral_trajectory_displacement_lookahead"},
  {Metric::stability, "stability"},
  {Metric::stability_frechet, "stability_frechet"},
  {Metric::obstacle_distance, "obstacle_distance"},
  {Metric::obstacle_ttc, "obstacle_ttc"},
  {Metric::modified_goal_longitudinal_deviation, "modified_goal_longitudinal_deviation"},
  {Metric::modified_goal_lateral_deviation, "modified_goal_lateral_deviation"},
  {Metric::modified_goal_yaw_deviation, "modified_goal_yaw_deviation"},
  {Metric::stop_decision, "stop_decision"},
  {Metric::abnormal_stop_decision, "abnormal_stop_decision"},
  {Metric::blinker_change_count, "blinker_change_count"},
  {Metric::steer_change_count, "steer_change_count"}};

// Metrics descriptions
static const std::unordered_map<Metric, std::string> metric_descriptions = {
  {Metric::curvature, "Curvature[1/rad]"},
  {Metric::point_interval, "Interval_between_points[m]"},
  {Metric::relative_angle, "Relative_angle[rad]"},
  {Metric::resampled_relative_angle, "Resampled_relative_angle[rad]"},
  {Metric::length, "Trajectory_length[m]"},
  {Metric::duration, "Trajectory_duration[s]"},
  {Metric::velocity, "Trajectory_velocity[m/s]"},
  {Metric::acceleration, "Trajectory_acceleration[m/s²]"},
  {Metric::jerk, "Trajectory_jerk[m/s³]"},
  {Metric::lateral_deviation, "Lateral_deviation[m]"},
  {Metric::yaw_deviation, "Yaw_deviation[rad]"},
  {Metric::velocity_deviation, "Velocity_deviation[m/s]"},
  {Metric::lateral_trajectory_displacement_local, "Nearest Pose Lateral Deviation[m]"},
  {Metric::lateral_trajectory_displacement_lookahead, "Lateral_Offset_Over_Distance_Ahead[m]"},
  {Metric::stability, "Stability[m]"},
  {Metric::stability_frechet, "StabilityFrechet[m]"},
  {Metric::obstacle_distance, "Obstacle_distance[m]"},
  {Metric::obstacle_ttc, "Obstacle_time_to_collision[s]"},
  {Metric::modified_goal_longitudinal_deviation, "Modified_goal_longitudinal_deviation[m]"},
  {Metric::modified_goal_lateral_deviation, "Modified_goal_lateral_deviation[m]"},
  {Metric::modified_goal_yaw_deviation, "Modified_goal_yaw_deviation[rad]"},
  {Metric::stop_decision,
   "The keep duration[s] and distance to stop line[m] of stop decisions made by each module"},
  {Metric::abnormal_stop_decision,
   "The keep duration[s] and distance to stop line[m] of abnormal stop decisions made by each "
   "module"},
  {Metric::blinker_change_count, "Count of blinker changes in recent `window_duration_s` seconds"},
  {Metric::steer_change_count,
   "Count of steer_rate positive/negative changes in recent `window_duration_s` seconds"}};

namespace details
{
static struct CheckCorrectMetricMaps
{
  CheckCorrectMetricMaps()
  {
    if (
      str_to_metric.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_to_str.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_descriptions.size() != static_cast<size_t>(Metric::SIZE)) {
      std::cerr << "[metric/metric.hpp] Maps are not defined for all metrics: ";
      std::cerr << str_to_metric.size() << " " << metric_to_str.size() << " "
                << metric_descriptions.size() << std::endl;
    }
  }
} check_correct_metric_maps;

}  // namespace details
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS__METRIC_HPP_
