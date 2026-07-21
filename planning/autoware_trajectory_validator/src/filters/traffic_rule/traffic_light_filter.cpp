// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_validator/filters/traffic_rule/traffic_light_filter.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::string get_signal_label(
  const autoware_perception_msgs::msg::TrafficLightGroup & signal,
  const validator::Params::TrafficLight & params)
{
  const bool is_red = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
    signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
    autoware_perception_msgs::msg::TrafficLightElement::RED);
  const bool is_amber = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
    signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
    autoware_perception_msgs::msg::TrafficLightElement::AMBER);
  const bool is_unknown = autoware::traffic_light_utils::hasTrafficLightColor(
    signal.elements, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);

  if (is_red) return "red";
  if (is_amber) {
    if (params.treat_amber_light_as_red_light) return "amber as red";
    return "amber";
  }
  if (is_unknown && params.treat_unknown_light_as_red_light) return "unknown as red";
  return "unknown";
}

std::string to_upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

std::string format_distance(const double distance)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << distance << "m";
  return stream.str();
}

double get_yaw(const geometry_msgs::msg::Quaternion & orientation)
{
  const auto sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const auto cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

double get_stop_line_length(const lanelet::BasicLineString2d & stop_line)
{
  double length = 0.0;
  for (size_t i = 1; i < stop_line.size(); ++i) {
    length +=
      std::hypot(stop_line[i].x() - stop_line[i - 1].x(), stop_line[i].y() - stop_line[i - 1].y());
  }
  return length;
}

std::optional<std::string> is_invalid_input(
  const autoware::trajectory_validator::FilterContext & context,
  const std::shared_ptr<autoware::vehicle_info_utils::VehicleInfo> & vehicle_info)
{
  if (!context.lanelet_map) {
    return "Lanelet map is not available in the context.";
  }

  if (!context.route) {
    return "Route is not available in the context.";
  }

  if (!vehicle_info) {
    return "Vehicle info is not set.";
  }

  if (!context.traffic_light_signals) {
    return "Traffic light signals are not available in the context.";
  }

  if (!context.odometry || !context.acceleration) {
    return "Odometry or acceleration is not available in the context.";
  }

  return std::nullopt;
}

autoware::traffic_light_compliance_checker::Parameters to_checker_params(
  const validator::Params::TrafficLight & params)
{
  autoware::traffic_light_compliance_checker::Parameters p{};
  p.deceleration_limit = params.deceleration_limit;
  p.jerk_limit = params.jerk_limit;
  p.delay_response_time = params.delay_response_time;
  p.crossing_time_limit = params.crossing_time_limit;
  p.treat_amber_light_as_red_light = params.treat_amber_light_as_red_light;
  p.treat_unknown_light_as_red_light = params.treat_unknown_light_as_red_light;
  p.stop_overshoot_margin = params.stop_overshoot_margin;
  p.allow_if_cannot_stop_distance = params.allow_if_cannot_stop_distance;
  p.stable_duration_threshold_red = params.stable_duration_threshold_red;
  p.stable_duration_threshold_amber = params.stable_duration_threshold_amber;
  p.stable_duration_threshold_unknown = params.stable_duration_threshold_unknown;
  p.amber_rejection_hysteresis_duration = params.amber_rejection_hysteresis_duration;
  p.ego_stopped_velocity_threshold = params.ego_stopped_velocity_threshold;
  p.checked_trajectory_length.deceleration_limit =
    params.checked_trajectory_length.deceleration_limit;
  p.checked_trajectory_length.jerk_limit = params.checked_trajectory_length.jerk_limit;
  return p;
}
}  // namespace

namespace autoware::trajectory_validator::plugin::traffic_rule
{

TrafficLightFilter::TrafficLightFilter() : ValidatorInterface("traffic_light_filter")
{
}

void TrafficLightFilter::update_parameters(const validator::Params & params)
{
  params_ = params.traffic_light;
  if (checker_) {
    checker_->update_parameters(to_checker_params(params_));
  }
}

void TrafficLightFilter::set_vehicle_info(const VehicleInfo & vehicle_info)
{
  ValidatorInterface::set_vehicle_info(vehicle_info);
  checker_ = std::make_unique<traffic_light_compliance_checker::TrafficLightComplianceChecker>(
    to_checker_params(params_), vehicle_info);
}

TrafficLightFilter::result_t TrafficLightFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  const auto & traj_points = candidate_trajectory.points;
  if (const auto has_invalid_input = is_invalid_input(context, vehicle_info_ptr_)) {
    return tl::make_unexpected(*has_invalid_input);
  }

  if (!checker_) {
    return tl::make_unexpected("Compliance checker is not initialized.");
  }

  const auto current_time = rclcpp::Time(context.odometry->header.stamp);

  if (!last_frame_time_ || *last_frame_time_ != current_time) {
    aggregated_rejections_.clear();
    last_frame_time_ = current_time;
  }

  const traffic_light_compliance_checker::Inputs inputs{
    traj_points,
    context.lanelet_map,
    *context.route,
    *context.traffic_light_signals,
    current_time,
    context.odometry->twist.twist.linear.x,
    context.acceleration->accel.accel.linear.x};

  const auto result = checker_->check(inputs);
  if (!result) {
    return tl::make_unexpected(result.error());
  }

  bool is_crossing_red = false;
  bool is_crossing_amber = false;

  for (const auto & violation : result->violations) {
    if (violation.type == traffic_light_compliance_checker::ViolationType::RED_LIGHT) {
      is_crossing_red = true;
    } else if (violation.type == traffic_light_compliance_checker::ViolationType::AMBER_LIGHT) {
      is_crossing_amber = true;
    }
  }

  update_debug_data(
    *result, *context.traffic_light_signals, current_time, context.odometry->pose.pose.position.z);

  std::vector<MetricReport> metrics;

  auto get_risk_level = [](bool is_crossing) {
    RiskLevel risk_level;
    risk_level.level = is_crossing ? RiskLevel::DANGER : RiskLevel::SAFE;
    return risk_level;
  };

  metrics.push_back(
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_crossing_red_light")
      .metric_value(0.0)
      .risk(get_risk_level(is_crossing_red)));

  metrics.push_back(
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_crossing_amber_light")
      .metric_value(0.0)
      .risk(get_risk_level(is_crossing_amber)));

  const bool is_feasible = !is_crossing_red && !is_crossing_amber;

  return ValidationResult{is_feasible, std::move(metrics)};
}

void TrafficLightFilter::update_debug_data(
  const traffic_light_compliance_checker::ComplianceResult & result,
  const autoware_perception_msgs::msg::TrafficLightGroupArray & traffic_light_signals,
  const rclcpp::Time & current_time, const double z)
{
  const auto update_common_info =
    [&](const int64_t traffic_light_id, const lanelet::BasicLineString2d & stop_line) -> auto & {
    auto & info = aggregated_rejections_[traffic_light_id];
    if (!stop_line.empty()) {
      info.stop_line_pos.x = 0.5 * (stop_line.front().x() + stop_line.back().x());
      info.stop_line_pos.y = 0.5 * (stop_line.front().y() + stop_line.back().y());
      info.stop_line_pos.z = z;
    }

    const auto signal_it = std::find_if(
      traffic_light_signals.traffic_light_groups.begin(),
      traffic_light_signals.traffic_light_groups.end(),
      [&](const auto & group) { return group.traffic_light_group_id == traffic_light_id; });
    info.signal_label = signal_it != traffic_light_signals.traffic_light_groups.end()
                          ? get_signal_label(*signal_it, params_)
                          : "unknown";
    return info;
  };

  for (const auto & violation : result.violations) {
    auto & info = aggregated_rejections_[violation.traffic_light_id];
    info.rejection_count++;
    update_common_info(violation.traffic_light_id, violation.stop_line);
  }
  for (const auto & dilemma_zone : result.dilemma_zone_debug_info) {
    auto & info = update_common_info(dilemma_zone.traffic_light_id, dilemma_zone.stop_line);
    info.dilemma_zone = dilemma_zone;
  }

  debug_markers_.markers.clear();
  const auto make_marker = [&](const std::string & marker_namespace, const int32_t type) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = marker_namespace;
    marker.id = static_cast<int>(debug_markers_.markers.size());
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  };

  for (const auto & [tl_id, info] : aggregated_rejections_) {
    auto text_marker =
      make_marker("rejection_info", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text_marker.pose.position = info.stop_line_pos;
    text_marker.pose.position.z += 2.0;
    text_marker.scale.z = 0.5;
    text_marker.color.a = 0.9;

    if (info.signal_label == "red" || info.signal_label == "amber as red") {
      text_marker.color.r = 1.0;
    } else {
      text_marker.color.r = 1.0;
      text_marker.color.g = 0.5;
    }

    text_marker.text = "TL: " + std::to_string(tl_id) + " (" + to_upper(info.signal_label) +
                       ")\nRejections: " + std::to_string(info.rejection_count);

    if (!info.dilemma_zone.has_value()) {
      debug_markers_.markers.push_back(text_marker);
      continue;
    }

    const auto & dilemma_zone = *info.dilemma_zone;
    text_marker.text +=
      "\nDist to Stop: " + format_distance(dilemma_zone.distance_from_ego_front) +
      "\nAllow Dist limit: " + format_distance(dilemma_zone.allow_if_cannot_stop_distance) +
      "\nEgo Stop Dist: ";

    if (dilemma_zone.ego_stopping_distance.has_value()) {
      text_marker.text += format_distance(*dilemma_zone.ego_stopping_distance);
    } else {
      text_marker.text += "unavailable";
    }

    if (dilemma_zone.is_allowed) {
      text_marker.text += "\nAction: ALLOWED (Cannot Stop)";
      text_marker.color.r = 0.0;
      text_marker.color.g = 1.0;
    } else if (!dilemma_zone.ego_stopping_distance.has_value()) {
      text_marker.text += "\nAction: ENFORCED (Stop Distance Unavailable)";
    } else if (
      dilemma_zone.allow_if_cannot_stop_distance <= 0.0 ||
      dilemma_zone.distance_from_ego_front >= dilemma_zone.allow_if_cannot_stop_distance) {
      text_marker.text += "\nAction: ENFORCED (Outside Allow Zone)";
    } else {
      text_marker.text += "\nAction: ENFORCED (Can Stop)";
    }
    debug_markers_.markers.push_back(text_marker);

    auto ego_front_marker =
      make_marker("traffic_light_ego_front", visualization_msgs::msg::Marker::SPHERE);
    ego_front_marker.pose = dilemma_zone.ego_front_pose;
    ego_front_marker.scale.x = 0.4;
    ego_front_marker.scale.y = 0.4;
    ego_front_marker.scale.z = 0.4;
    ego_front_marker.color.g = 1.0;
    ego_front_marker.color.b = 1.0;
    ego_front_marker.color.a = 1.0;
    debug_markers_.markers.push_back(ego_front_marker);

    const double ego_yaw = get_yaw(dilemma_zone.ego_front_pose.orientation);
    const double direction_x = std::cos(ego_yaw);
    const double direction_y = std::sin(ego_yaw);

    auto allow_boundary_marker =
      make_marker("traffic_light_allow_boundary", visualization_msgs::msg::Marker::LINE_STRIP);
    allow_boundary_marker.scale.x = 0.15;
    allow_boundary_marker.color.r = 1.0;
    allow_boundary_marker.color.b = 1.0;
    allow_boundary_marker.color.a = 0.9;
    for (const auto & stop_line_point : dilemma_zone.stop_line) {
      geometry_msgs::msg::Point boundary_point;
      boundary_point.x =
        stop_line_point.x() - dilemma_zone.allow_if_cannot_stop_distance * direction_x;
      boundary_point.y =
        stop_line_point.y() - dilemma_zone.allow_if_cannot_stop_distance * direction_y;
      boundary_point.z = z + 0.2;
      allow_boundary_marker.points.push_back(boundary_point);
    }
    debug_markers_.markers.push_back(allow_boundary_marker);

    if (dilemma_zone.ego_stopping_distance.has_value()) {
      const double required_stopping_distance =
        *dilemma_zone.ego_stopping_distance - dilemma_zone.stop_overshoot_margin;
      auto stopping_wall_marker =
        make_marker("traffic_light_required_stopping_wall", visualization_msgs::msg::Marker::CUBE);
      stopping_wall_marker.pose = dilemma_zone.ego_front_pose;
      stopping_wall_marker.pose.position.x += required_stopping_distance * direction_x;
      stopping_wall_marker.pose.position.y += required_stopping_distance * direction_y;
      stopping_wall_marker.pose.position.z = z + 1.0;
      stopping_wall_marker.scale.x = 0.15;
      stopping_wall_marker.scale.y = std::max(3.0, get_stop_line_length(dilemma_zone.stop_line));
      stopping_wall_marker.scale.z = 2.0;
      stopping_wall_marker.color.a = 0.35;
      if (required_stopping_distance <= dilemma_zone.distance_from_ego_front) {
        stopping_wall_marker.color.r = 1.0;
      } else {
        stopping_wall_marker.color.g = 1.0;
      }
      debug_markers_.markers.push_back(stopping_wall_marker);
    }
  }
}

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::TrafficLightFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
