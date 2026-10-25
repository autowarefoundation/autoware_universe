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

// Adapted from the DiffusionPlanner map/rule context code.

#include "autoware/candidate_bank/rule_context.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
namespace autoware::candidate_bank
{
using autoware_planning_msgs::msg::Trajectory;
namespace
{
std::vector<std::array<float, 2>> extract_route_polyline(const std::vector<float> & route_lanes)
{
  const size_t single_batch_size =
    static_cast<size_t>(NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
  if (route_lanes.size() < single_batch_size) {
    return {};
  }

  std::vector<std::array<float, 2>> points;
  for (int64_t segment = 0; segment < NUM_SEGMENTS_IN_ROUTE; ++segment) {
    for (int64_t point = 0; point < POINTS_PER_SEGMENT; ++point) {
      const size_t base =
        static_cast<size_t>((segment * POINTS_PER_SEGMENT + point) * SEGMENT_POINT_DIM);
      bool valid = false;
      for (int64_t dim = 0; dim < SEGMENT_POINT_DIM; ++dim) {
        if (std::abs(route_lanes[base + static_cast<size_t>(dim)]) > 1.0e-6F) {
          valid = true;
          break;
        }
      }
      if (!valid) {
        continue;
      }
      const std::array<float, 2> candidate{route_lanes[base + X], route_lanes[base + Y]};
      if (!points.empty()) {
        const float dx = candidate[0] - points.back()[0];
        const float dy = candidate[1] - points.back()[1];
        if (std::hypot(dx, dy) < 1.0e-3F) {
          continue;
        }
      }
      points.push_back(candidate);
    }
  }
  return points;
}

// convert the existing ego-frame route tensor to a
// lightweight map-frame Trajectory used only for route deviation/progress scoring.
std::optional<Trajectory> create_candidate_route_reference_map(
  const std::vector<float> & route_lanes, const Eigen::Matrix4d & ego_to_map_transform,
  const rclcpp::Time & timestamp)
{
  const auto points_ego = extract_route_polyline(route_lanes);
  if (points_ego.size() < 2) {
    return std::nullopt;
  }

  Trajectory reference;
  reference.header.stamp = timestamp;
  reference.header.frame_id = "map";
  reference.points.reserve(points_ego.size());

  for (const auto & point_ego : points_ego) {
    Eigen::Vector4d point;
    point << static_cast<double>(point_ego[0]), static_cast<double>(point_ego[1]), 0.0, 1.0;
    const Eigen::Vector4d point_map = ego_to_map_transform * point;

    autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point_map.x();
    trajectory_point.pose.position.y = point_map.y();
    trajectory_point.pose.position.z = point_map.z();
    trajectory_point.pose.orientation.w = 1.0;
    reference.points.push_back(std::move(trajectory_point));
  }
  return reference;
}

struct PolylineProjection
{
  float arc_length_m{0.0F};
  float lateral_distance_m{0.0F};
};

struct RoutePolylineGeometry
{
  std::vector<std::array<float, 2>> points;
  std::vector<float> cumulative_distance_m;
  float ego_start_s_m{0.0F};
};

std::optional<PolylineProjection> project_to_polyline(
  const std::vector<std::array<float, 2>> & points,
  const std::vector<float> & cumulative_distance_m, const float query_x, const float query_y)
{
  if (points.size() < 2 || cumulative_distance_m.size() != points.size()) {
    return std::nullopt;
  }

  PolylineProjection best;
  best.lateral_distance_m = std::numeric_limits<float>::max();
  for (size_t i = 1; i < points.size(); ++i) {
    const float vx = points[i][0] - points[i - 1][0];
    const float vy = points[i][1] - points[i - 1][1];
    const float length_squared = vx * vx + vy * vy;
    if (length_squared < 1.0e-8F) {
      continue;
    }
    const float projection = std::clamp(
      ((query_x - points[i - 1][0]) * vx + (query_y - points[i - 1][1]) * vy) / length_squared,
      0.0F, 1.0F);
    const float px = points[i - 1][0] + projection * vx;
    const float py = points[i - 1][1] + projection * vy;
    const float distance = std::hypot(query_x - px, query_y - py);
    if (distance < best.lateral_distance_m) {
      best.lateral_distance_m = distance;
      best.arc_length_m = cumulative_distance_m[i - 1] + projection * std::sqrt(length_squared);
    }
  }
  if (!std::isfinite(best.lateral_distance_m)) {
    return std::nullopt;
  }
  return best;
}

std::optional<RoutePolylineGeometry> create_route_polyline_geometry(
  const std::vector<float> & route_lanes)
{
  RoutePolylineGeometry geometry;
  geometry.points = extract_route_polyline(route_lanes);
  if (geometry.points.size() < 2) {
    return std::nullopt;
  }

  geometry.cumulative_distance_m.resize(geometry.points.size(), 0.0F);
  for (size_t i = 1; i < geometry.points.size(); ++i) {
    geometry.cumulative_distance_m[i] =
      geometry.cumulative_distance_m[i - 1] + std::hypot(
                                                geometry.points[i][0] - geometry.points[i - 1][0],
                                                geometry.points[i][1] - geometry.points[i - 1][1]);
  }
  const auto ego_projection =
    project_to_polyline(geometry.points, geometry.cumulative_distance_m, 0.0F, 0.0F);
  if (!ego_projection.has_value()) {
    return std::nullopt;
  }
  geometry.ego_start_s_m = ego_projection->arc_length_m;
  return geometry;
}

bool traffic_signal_requires_stop(const uint8_t color, const RuleConfig & params)
{
  using autoware_perception_msgs::msg::TrafficLightElement;
  if (color == TrafficLightElement::RED) {
    return params.stop_on_red;
  }
  if (color == TrafficLightElement::AMBER) {
    return params.stop_on_amber;
  }
  if (color == TrafficLightElement::UNKNOWN) {
    return params.stop_on_unknown;
  }
  return false;
}

struct SelectedTrafficSignalStop
{
  preprocess::RouteTrafficSignalStop source;
  bool requires_stop{false};
  double effective_stop_distance_m{0.0};
  double lateral_projection_error_m{0.0};
  double stop_line_s_m{0.0};
};

std::optional<SelectedTrafficSignalStop> select_nearest_traffic_signal(
  const RoutePolylineGeometry & geometry,
  const std::vector<preprocess::RouteTrafficSignalStop> & candidates, const RuleConfig & params,
  const std::optional<double> past_stop_line_hold_distance_m = std::nullopt)
{
  std::optional<SelectedTrafficSignalStop> selected;
  for (const auto & candidate : candidates) {
    const auto projection = project_to_polyline(
      geometry.points, geometry.cumulative_distance_m,
      static_cast<float>(candidate.stop_line_center_ego.x()),
      static_cast<float>(candidate.stop_line_center_ego.y()));
    if (!projection.has_value()) {
      continue;
    }
    if (projection->lateral_distance_m > params.max_stop_line_lateral_distance_m) {
      continue;
    }
    const double raw_distance_m =
      static_cast<double>(projection->arc_length_m - geometry.ego_start_s_m);
    const double past_stop_line_limit_m =
      past_stop_line_hold_distance_m.value_or(params.stop_margin_m);
    if (raw_distance_m < -past_stop_line_limit_m) {
      continue;
    }
    const double effective_distance_m = std::max(0.0, raw_distance_m - params.stop_margin_m);
    if (!selected.has_value() || effective_distance_m < selected->effective_stop_distance_m) {
      selected = SelectedTrafficSignalStop{
        candidate, traffic_signal_requires_stop(candidate.color, params), effective_distance_m,
        projection->lateral_distance_m, static_cast<double>(projection->arc_length_m)};
    }
  }
  return selected;
}

}  // namespace
RuleSnapshot RuleContext::build(
  const nav_msgs::msg::Odometry & ego, const autoware_planning_msgs::msg::LaneletRoute & route,
  const CandidateBankConfig & bank, const rclcpp::Time & stamp)
{
  if (!lanes_) throw std::runtime_error("vector map is not ready");
  const auto & q = ego.pose.pose.orientation;
  Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
  if (rot.norm() < 1e-6) throw std::runtime_error("invalid ego quaternion");
  rot.normalize();
  Eigen::Matrix4d ego_to_map_transform = Eigen::Matrix4d::Identity();
  ego_to_map_transform.block<3, 3>(0, 0) = rot.toRotationMatrix();
  const auto & pos = ego.pose.pose.position;
  ego_to_map_transform.block<3, 1>(0, 3) =
    Eigen::Vector3d(pos.x, pos.y, pos.z) +
    rot * Eigen::Vector3d(bank.ego_base_link_to_center_m, 0, 0);
  const Eigen::Matrix4d map_to_ego_transform = ego_to_map_transform.inverse();
  const double center_x = ego_to_map_transform(0, 3), center_y = ego_to_map_transform(1, 3),
               center_z = ego_to_map_transform(2, 3);
  preprocess::process_traffic_signals({}, signals_, stamp, rules_.signal_timeout_sec);
  auto indices = lanes_->select_route_segment_indices(
    route, center_x, center_y, center_z, NUM_SEGMENTS_IN_ROUTE);
  auto data = lanes_->create_tensor_data_from_indices(
    map_to_ego_transform, signals_, indices, NUM_SEGMENTS_IN_ROUTE);
  const auto & route_lanes = data.first;
  RuleSnapshot result;
  // route scoring reference does not affect model input.
  result.route.reset();
  if (true) {
    result.route = create_candidate_route_reference_map(route_lanes, ego_to_map_transform, stamp);
  }
  result.conflicts = CandidateMapConflictContext{};
  if (true && bank.dynamic_scope_mode == "map_conflict_reachable") {
    result.conflicts = lanes_->create_candidate_map_conflict_context(
      route, center_x, center_y, center_z, bank.map_conflict_forward_search_distance_m,
      bank.map_conflict_backtrace_distance_m);
  }
  result.rules = CandidateTrafficRuleContext{};
  // 0015 restores the primary signal association to the original route-based behavior.
  result.rules.traffic_signal_scope_mode = "route_nearest";

  std::optional<preprocess::CurrentLaneRuleContext> current_lane_context;
  if (
    true && (bank.lane_boundary_enabled || bank.lane_change_deadline_enabled ||
             bank.traffic_signal_scope_mode == "current_lane")) {
    current_lane_context = lanes_->get_current_lane_rule_context(
      center_x, center_y, map_to_ego_transform, signals_,
      bank.lane_change_target_lane_signal_search_distance_m);
    result.rules.current_lane_valid = current_lane_context->valid;
    result.rules.current_lane_id = static_cast<std::int64_t>(current_lane_context->lane_id);
    for (const auto & boundary : current_lane_context->shared_boundaries_map) {
      CandidateLaneBoundaryGeometry converted;
      converted.boundary_id = static_cast<std::int64_t>(boundary.boundary_id);
      converted.adjacent_lane_id = static_cast<std::int64_t>(boundary.adjacent_lane_id);
      converted.points_map.reserve(boundary.points_map.size());
      for (const auto & point : boundary.points_map) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        converted.points_map.push_back(p);
      }
      result.rules.shared_lane_boundaries_map.push_back(std::move(converted));
    }
  }

  if (true && (bank.traffic_signal_rule_enabled || bank.lane_change_deadline_enabled)) {
    const auto route_geometry = create_route_polyline_geometry(route_lanes);
    if (route_geometry.has_value()) {
      // Restore 0012 semantics for the primary rule: always derive the relevant signal from the
      // route ahead. current_lane is retained as a compatibility parameter but is not used here.
      const auto signal_candidates = lanes_->get_route_traffic_signal_stops(
        route, center_x, center_y, center_z, map_to_ego_transform, signals_);
      const auto selected_signal_stop = select_nearest_traffic_signal(
        route_geometry.value(), signal_candidates, rules_,
        std::max(rules_.stop_margin_m, bank.traffic_signal_post_stop_hold_distance_m));
      if (selected_signal_stop.has_value()) {
        result.rules.traffic_signal_present = true;
        result.rules.traffic_signal_stop_active = selected_signal_stop->requires_stop;
        result.rules.traffic_signal_stop_line_s_m = selected_signal_stop->stop_line_s_m;
        result.rules.traffic_signal_target_stop_s_m =
          static_cast<double>(route_geometry->ego_start_s_m) +
          selected_signal_stop->effective_stop_distance_m;
        result.rules.traffic_light_id =
          static_cast<std::int64_t>(selected_signal_stop->source.traffic_light_id);
        result.rules.traffic_light_color = selected_signal_stop->source.color;
      }

      // Project each adjacent-lane signal onto the same route-progress coordinate. This lets the
      // fallback-stop evaluator use the target-lane stop requirement without replacing the
      // primary route-based signal association.
      if (current_lane_context.has_value() && current_lane_context->valid) {
        for (std::size_t i = 0; i < current_lane_context->shared_boundaries_map.size() &&
                                i < result.rules.shared_lane_boundaries_map.size();
             ++i) {
          const auto adjacent_selected = select_nearest_traffic_signal(
            route_geometry.value(),
            current_lane_context->shared_boundaries_map[i].adjacent_traffic_signal_stops, rules_);
          if (!adjacent_selected.has_value()) continue;
          auto & converted = result.rules.shared_lane_boundaries_map[i];
          converted.adjacent_signal_present = true;
          converted.adjacent_signal_stop_active = adjacent_selected->requires_stop;
          converted.adjacent_signal_stop_line_s_m = adjacent_selected->stop_line_s_m;
          converted.adjacent_signal_target_stop_s_m =
            static_cast<double>(route_geometry->ego_start_s_m) +
            adjacent_selected->effective_stop_distance_m;
          converted.adjacent_traffic_light_id =
            static_cast<std::int64_t>(adjacent_selected->source.traffic_light_id);
          converted.adjacent_traffic_light_color = adjacent_selected->source.color;
        }
      }
    }
  }

  return result;
}
}  // namespace autoware::candidate_bank
