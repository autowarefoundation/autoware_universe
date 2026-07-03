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

#include "reporter.hpp"

#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/control_point.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <fmt/core.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::reporter
{
namespace
{
using autoware_trajectory_validator::msg::MetricReport;

struct VisualizationData
{
  std::string error_msg{};
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors{};
};

struct Color
{
  float r;
  float g;
  float b;
};

int next_marker_id(const visualization_msgs::msg::MarkerArray & debug_markers)
{
  return debug_markers.markers.empty() ? 0 : debug_markers.markers.back().id + 1;
}

Color resolve_trajectory_color(const std::string & trajectory_id)
{
  if (trajectory_id.find("_constant_curvature_path") != std::string::npos) {
    return Color{0.0F, 0.75F, 1.0F};
  }
  return Color{0.2F, 1.0F, 0.2F};
}

autoware_internal_planning_msgs::msg::SafetyFactorArray make_safety_factor_array(
  const builtin_interfaces::msg::Time & stamp, const CollisionDetail & collision_detail,
  const std::string & collision_type, double time_resolution)
{
  using autoware_internal_planning_msgs::msg::SafetyFactor;
  using autoware_internal_planning_msgs::msg::SafetyFactorArray;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = collision_detail.object_identification.uuid;
  safety_factor.ttc_begin = static_cast<float>(collision_detail.first_collision_timing.ttc);
  safety_factor.ttc_end =
    static_cast<float>(collision_detail.first_collision_timing.ttc + time_resolution);
  safety_factor.is_safe = false;
  if (!collision_detail.object_trajectory.empty()) {
    safety_factor.points.push_back(collision_detail.object_trajectory.front().position);
  }

  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = stamp;
  safety_factors.header.frame_id = "map";
  safety_factors.factors.push_back(std::move(safety_factor));
  safety_factors.is_safe = false;
  safety_factors.detail = collision_type;
  return safety_factors;
}

void add_collision_planning_factor(
  const double time_resolution, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const CollisionDetail & collision_detail,
  const std::string & collision_type,
  autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors)
{
  const auto safety_factors =
    make_safety_factor_array(stamp, collision_detail, collision_type, time_resolution);
  const auto control_point =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::ControlPoint>()
      .pose(ego_pose)
      .velocity(0.0)
      .shift_length(0.0)
      .distance(0.0);
  auto factor =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::PlanningFactor>()
      .module("")
      .is_driving_forward(true)
      .control_points({control_point})
      .behavior(autoware_internal_planning_msgs::msg::PlanningFactor::STOP)
      .detail(collision_type)
      .safety_factors(safety_factors);
  planning_factors.factors.push_back(std::move(factor));
}

void process_drac_artifacts(
  const nav_msgs::msg::Odometry & odometry,
  reporter::ContinuousDetectionTimes & drac_continuous_times, const rclcpp::Time & current_time,
  const DracArtifact & drac_artifact, VisualizationData & artifacts,
  visualization_msgs::msg::MarkerArray & debug_markers, double time_resolution)
{
  drac_continuous_times.update(
    current_time, drac_artifact.object_evaluations, [](const auto & evaluation) {
      return evaluation.detail.object_identification.trajectory_id_string();
    });

  if (drac_artifact.risk == RiskLevel::SAFE || drac_artifact.object_evaluations.empty()) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  const bool has_error = drac_artifact.risk == RiskLevel::DANGER;
  const RiskLevel::_level_type log_level = has_error ? RiskLevel::DANGER : RiskLevel::HIGH_CAUTION;
  for (const auto & evaluation : drac_artifact.object_evaluations) {
    const auto & timing = evaluation.detail;
    const auto & obj_id = timing.object_identification;

    const auto finding_msg = fmt::format(
      "DRAC collision, classification: {}, ID: {}, PET: {}, TTC: {}, DRAC: {}, duration: {}, "
      "stamp: {}.{};",
      obj_id.classification, obj_id.trajectory_id_string(), timing.worst_pet_timing.pet,
      timing.first_collision_timing.ttc,
      drac_artifact.required_acceleration.has_value()
        ? std::to_string(drac_artifact.required_acceleration.value())
        : "Cant be avoided",
      drac_continuous_times.get_time(obj_id.trajectory_id_string()), obj_id.stamp.sec,
      obj_id.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
    reporter::add_debug_markers(
      debug_markers, current_time, "drac_collision", obj_id.trajectory_id_string(),
      timing.ego_trajectory, timing.object_trajectory, timing.ego_hull, timing.object_hull);
    if (has_error) {
      add_collision_planning_factor(
        time_resolution, odometry.header.stamp, odometry.pose.pose, timing, "DRAC",
        artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(log_level, log_messages);
}

void process_rss_artifacts(
  const RssArtifact & rss_artifact, reporter::ContinuousDetectionTimes & rss_continuous_times,
  const rclcpp::Time & current_time, VisualizationData & artifacts)
{
  std::vector<RssEvaluation> violations{};
  violations.reserve(rss_artifact.object_evaluations.size());
  for (const auto & evaluation : rss_artifact.object_evaluations) {
    if (evaluation.risk == RiskLevel::SAFE) {
      continue;
    }
    violations.push_back(evaluation);
  }
  rss_continuous_times.update(current_time, violations, [](const auto & violation) {
    return violation.detail.object_identification.object_id_string();
  });

  if (rss_artifact.risk == RiskLevel::SAFE || violations.empty()) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  for (const auto & violation : violations) {
    const auto & detail = violation.detail;
    const auto object_id = detail.object_identification.object_id_string();

    const auto finding_msg = fmt::format(
      "RSS collision, classification: {}, ID: {}, duration: {}, required deceleration: {}, "
      "stamp: {}.{};",
      detail.object_identification.classification, object_id,
      rss_continuous_times.get_time(object_id), detail.rss_acceleration,
      detail.object_identification.stamp.sec, detail.object_identification.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(RiskLevel::DANGER, log_messages);
}
}  // namespace

void ContinuousDetectionTimes::clear()
{
  current_time_.reset();
  detection_start_times_.clear();
}

double ContinuousDetectionTimes::get_time(const std::string & key) const
{
  if (!current_time_) {
    return 0.0;
  }

  const auto it = detection_start_times_.find(key);
  if (it == detection_start_times_.end()) {
    return 0.0;
  }

  return (*current_time_ - it->second).seconds();
}

void add_debug_markers(
  visualization_msgs::msg::MarkerArray & debug_markers, const rclcpp::Time & stamp,
  const std::string & ns, const std::string & trajectory_id, const PoseTrajectory & ego_trajectory,
  const PoseTrajectory & object_trajectory, const Polygon2d & ego_hull,
  const Polygon2d & object_hull)
{
  int id = next_marker_id(debug_markers);
  const auto trajectory_color = resolve_trajectory_color(trajectory_id);

  auto add_poly_marker =
    [&](const Polygon2d & poly, const std::string & local_namespace, float r, float g, float b) {
      if (poly.outer().empty()) {
        return;
      }

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns + "/" + local_namespace;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.05;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 0.9;

      for (const auto & p : poly.outer()) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.0;
        m.points.push_back(pt);
      }

      geometry_msgs::msg::Point first_point;
      first_point.x = poly.outer().front().x();
      first_point.y = poly.outer().front().y();
      first_point.z = 0.0;
      m.points.push_back(first_point);

      debug_markers.markers.push_back(std::move(m));
    };

  auto add_trajectory_marker = [&](
                                 const PoseTrajectory & trajectory,
                                 const std::string & local_namespace, float r, float g, float b,
                                 float alpha) {
    if (trajectory.empty()) {
      return;
    }

    for (const auto & pose : trajectory) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = stamp;
      m.ns = ns + "/" + local_namespace;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = 0.3;
      m.scale.y = 0.18;
      m.scale.z = 0.18;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = alpha;
      debug_markers.markers.push_back(std::move(m));
    }
  };

  add_poly_marker(ego_hull, "ego_worst_pet", 0.0F, 0.0F, 1.0F);
  add_poly_marker(object_hull, "obj_worst_pet", 1.0F, 0.0F, 0.0F);
  add_trajectory_marker(ego_trajectory, "ego_trajectory", 1.0F, 1.0F, 1.0F, 0.9F);
  add_trajectory_marker(
    object_trajectory, "object_trajectory", trajectory_color.r, trajectory_color.g,
    trajectory_color.b, 0.95F);
}

void add_error_text_marker(
  visualization_msgs::msg::MarkerArray & debug_markers, const rclcpp::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const std::string & error_msg)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = "collision_check_error";
  m.id = next_marker_id(debug_markers);
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.z = 0.6;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  m.color.a = 0.95;
  m.pose = ego_pose;
  m.pose.position.z += 1.0;
  m.text = error_msg;
  debug_markers.markers.push_back(std::move(m));
}

void append_text_marker_message(std::string & text, const std::string & message)
{
  if (!message.empty()) {
    text += message + "\n";
  }
}

void log_collision_messages(const RiskLevel::_level_type level, const std::string & messages)
{
  if (messages.empty()) {
    return;
  }
  if (level == RiskLevel::DANGER) {
    RCLCPP_ERROR(rclcpp::get_logger("CollisionCheckFilter"), "Not feasible: %s", messages.c_str());
    return;
  }
  RCLCPP_WARN(rclcpp::get_logger("CollisionCheckFilter"), "Warning: %s", messages.c_str());
}

autoware_internal_planning_msgs::msg::PlanningFactorArray process_collision_artifacts(
  const nav_msgs::msg::Odometry & odometry, const DracArtifact & drac_artifact,
  ContinuousDetectionTimes & drac_continuous_times, const RssArtifact & rss_artifact,
  ContinuousDetectionTimes & rss_continuous_times,
  visualization_msgs::msg::MarkerArray & debug_markers, double time_resolution)
{
  VisualizationData visualization_data{};
  const auto current_time = rclcpp::Time{odometry.header.stamp};

  process_drac_artifacts(
    odometry, drac_continuous_times, current_time, drac_artifact, visualization_data, debug_markers,
    time_resolution);
  process_rss_artifacts(rss_artifact, rss_continuous_times, current_time, visualization_data);

  if (!visualization_data.error_msg.empty()) {
    add_error_text_marker(
      debug_markers, current_time, odometry.pose.pose, visualization_data.error_msg);
  }

  return std::move(visualization_data.planning_factors);
}
}  // namespace autoware::trajectory_validator::plugin::safety::reporter
