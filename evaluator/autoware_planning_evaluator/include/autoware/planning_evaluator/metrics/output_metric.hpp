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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS__OUTPUT_METRIC_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS__OUTPUT_METRIC_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning_diagnostics
{
/**
 * @brief Enumeration of metrics to save to output.json
 */
enum class OutputMetric {
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
static const std::unordered_map<std::string, OutputMetric> str_to_output_metric = {
  {"curvature", OutputMetric::curvature},
  {"point_interval", OutputMetric::point_interval},
  {"relative_angle", OutputMetric::relative_angle},
  {"resampled_relative_angle", OutputMetric::resampled_relative_angle},
  {"length", OutputMetric::length},
  {"duration", OutputMetric::duration},
  {"velocity", OutputMetric::velocity},
  {"acceleration", OutputMetric::acceleration},
  {"jerk", OutputMetric::jerk},
  {"lateral_deviation", OutputMetric::lateral_deviation},
  {"yaw_deviation", OutputMetric::yaw_deviation},
  {"velocity_deviation", OutputMetric::velocity_deviation},
  {"lateral_trajectory_displacement_local", OutputMetric::lateral_trajectory_displacement_local},
  {"lateral_trajectory_displacement_lookahead",
   OutputMetric::lateral_trajectory_displacement_lookahead},
  {"stability", OutputMetric::stability},
  {"stability_frechet", OutputMetric::stability_frechet},
  {"obstacle_distance", OutputMetric::obstacle_distance},
  {"obstacle_ttc", OutputMetric::obstacle_ttc},
  {"modified_goal_longitudinal_deviation", OutputMetric::modified_goal_longitudinal_deviation},
  {"modified_goal_lateral_deviation", OutputMetric::modified_goal_lateral_deviation},
  {"modified_goal_yaw_deviation", OutputMetric::modified_goal_yaw_deviation},
  {"stop_decision", OutputMetric::stop_decision},
  {"abnormal_stop_decision", OutputMetric::abnormal_stop_decision},
  {"blinker_change_count", OutputMetric::blinker_change_count},
  {"steer_change_count", OutputMetric::steer_change_count}};

static const std::unordered_map<OutputMetric, std::string> output_metric_to_str = {
  {OutputMetric::curvature, "curvature"},
  {OutputMetric::point_interval, "point_interval"},
  {OutputMetric::relative_angle, "relative_angle"},
  {OutputMetric::resampled_relative_angle, "resampled_relative_angle"},
  {OutputMetric::length, "length"},
  {OutputMetric::duration, "duration"},
  {OutputMetric::velocity, "velocity"},
  {OutputMetric::acceleration, "acceleration"},
  {OutputMetric::jerk, "jerk"},
  {OutputMetric::lateral_deviation, "lateral_deviation"},
  {OutputMetric::yaw_deviation, "yaw_deviation"},
  {OutputMetric::velocity_deviation, "velocity_deviation"},
  {OutputMetric::lateral_trajectory_displacement_local, "lateral_trajectory_displacement_local"},
  {OutputMetric::lateral_trajectory_displacement_lookahead,
   "lateral_trajectory_displacement_lookahead"},
  {OutputMetric::stability, "stability"},
  {OutputMetric::stability_frechet, "stability_frechet"},
  {OutputMetric::obstacle_distance, "obstacle_distance"},
  {OutputMetric::obstacle_ttc, "obstacle_ttc"},
  {OutputMetric::modified_goal_longitudinal_deviation, "modified_goal_longitudinal_deviation"},
  {OutputMetric::modified_goal_lateral_deviation, "modified_goal_lateral_deviation"},
  {OutputMetric::modified_goal_yaw_deviation, "modified_goal_yaw_deviation"},
  {OutputMetric::stop_decision, "stop_decision"},
  {OutputMetric::abnormal_stop_decision, "abnormal_stop_decision"},
  {OutputMetric::blinker_change_count, "blinker_change_count"},
  {OutputMetric::steer_change_count, "steer_change_count"}};

// OutputMetrics descriptions
static const std::unordered_map<OutputMetric, std::string> output_metric_descriptions = {
  {OutputMetric::curvature, "Statics of published curvature metrics"},
  {OutputMetric::point_interval, "Statics of published point_interval metrics"},
  {OutputMetric::relative_angle, "Statics of published relative_angle metrics"},
  {OutputMetric::resampled_relative_angle, "Statics of published resampled_relative_angle metrics"},
  {OutputMetric::length, "Statics of published length metrics"},
  {OutputMetric::duration, "Statics of published duration metrics"},
  {OutputMetric::velocity, "Statics of published velocity metrics"},
  {OutputMetric::acceleration, "Statics of published acceleration metrics"},
  {OutputMetric::jerk, "Statics of published jerk metrics"},
  {OutputMetric::lateral_deviation, "Statics of published lateral_deviation metrics"},
  {OutputMetric::yaw_deviation, "Statics of published yaw_deviation metrics"},
  {OutputMetric::velocity_deviation, "Statics of published velocity_deviation metrics"},
  {OutputMetric::lateral_trajectory_displacement_local,
   "Statics of published lateral_trajectory_displacement_local metrics"},
  {OutputMetric::lateral_trajectory_displacement_lookahead,
   "Statics of published lateral_trajectory_displacement_lookahead metrics"},
  {OutputMetric::stability, "Statics of published stability metrics"},
  {OutputMetric::stability_frechet, "Statics of published stability_frechet metrics"},
  {OutputMetric::obstacle_distance, "Statics of published obstacle_distance metrics"},
  {OutputMetric::obstacle_ttc, "Statics of published obstacle_ttc metrics"},
  {OutputMetric::modified_goal_longitudinal_deviation,
   "Statics of published modified_goal_longitudinal_deviation metrics"},
  {OutputMetric::modified_goal_lateral_deviation,
   "Statics of published modified_goal_lateral_deviation metrics"},
  {OutputMetric::modified_goal_yaw_deviation,
   "Statics of published modified_goal_yaw_deviation metrics"},
  {OutputMetric::stop_decision,
   "Count of stop decision of each module and statics of stop decision's keep duration"},
  {OutputMetric::abnormal_stop_decision,
   "Count of abnormal stop decision of each module, and statics of abnormal stop decision's keep "
   "duration"},
  {OutputMetric::blinker_change_count,
   "Statics of published blinker_change_count metrics and total count of blink changes"},
  {OutputMetric::steer_change_count,
   "Statics of published steer_change_count metrics and total count of steer changes"}};

namespace details
{
static struct CheckCorrectOutputMetricMaps
{
  CheckCorrectOutputMetricMaps()
  {
    if (
      str_to_output_metric.size() != static_cast<size_t>(OutputMetric::SIZE) ||
      output_metric_to_str.size() != static_cast<size_t>(OutputMetric::SIZE) ||
      output_metric_descriptions.size() != static_cast<size_t>(OutputMetric::SIZE)) {
      std::cerr
        << "[output_metric/output_metric.hpp] Maps are not defined for all output_metrics: ";
      std::cerr << str_to_output_metric.size() << " " << output_metric_to_str.size() << " "
                << output_metric_descriptions.size() << std::endl;
    }
  }
} check_correct_output_metric_maps;

}  // namespace details
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS__OUTPUT_METRIC_HPP_
