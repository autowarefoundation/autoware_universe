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

#include "autoware/candidate_bank/candidate/trajectory_candidate_manager.hpp"

// CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
#include "autoware/candidate_bank/candidate/spatiotemporal_collision_checker.hpp"
// CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END

// CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: reuse the existing internal JSONL logger.
#include "autoware/candidate_bank/internal_debug/internal_debug_logger.hpp"
// CANDIDATE_BANK_DIAGNOSTICS_V2_END

// CANDIDATE_BANK_EXPERIMENT_V1_BEGIN: concrete staged evaluation implementation.
// NOTE: Candidate Bank still uses circle-based footprint distance, but BOUNDING_BOX
// obstacles are refined to a three-circle conservative approximation by 0010.
// Replacing it with oriented polygon distance later does not require changing DiffusionPlannerCore.

#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

// CANDIDATE_BANK_COLLISION_GEOMETRY_V1_BEGIN
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
// CANDIDATE_BANK_COLLISION_GEOMETRY_V1_END

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

// CB_COMPARE_TRACE_V1_BEGIN
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
// CB_COMPARE_TRACE_V1_END

namespace autoware::candidate_bank
{
namespace
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

constexpr double kEpsilon = 1.0e-6;

// CB_COMPARE_TRACE_V1_BEGIN
bool cb_ct_on()
{
  static const bool v = []() {
    const char * p = std::getenv("CB_COMPARE_TRACE_FILE");
    return p && *p;
  }();
  return v;
}
const char * cb_ct_label()
{
  const char * p = std::getenv("CB_COMPARE_TRACE_LABEL");
  return p && *p ? p : "unknown";
}
std::mutex & cb_ct_mutex()
{
  static std::mutex m;
  return m;
}
void cb_ct_num(std::ostringstream & o, double v)
{
  if (std::isfinite(v))
    o << std::setprecision(15) << v;
  else
    o << "null";
}
void cb_ct_write(const std::string & s)
{
  if (!cb_ct_on()) return;
  const char * p = std::getenv("CB_COMPARE_TRACE_FILE");
  if (!p || !*p) return;
  std::lock_guard<std::mutex> l(cb_ct_mutex());
  std::ofstream f(p, std::ios::app);
  if (f) f << s << '\n';
}
std::uint64_t cb_ct_sig(const Trajectory & t)
{
  std::uint64_t h = 1469598103934665603ULL;
  auto mix = [&](std::int64_t v) {
    h ^= static_cast<std::uint64_t>(v);
    h *= 1099511628211ULL;
  };
  for (const auto & p : t.points) {
    mix(std::llround(p.pose.position.x * 1000));
    mix(std::llround(p.pose.position.y * 1000));
    mix(std::llround(p.longitudinal_velocity_mps * 1000));
    mix(std::llround(rclcpp::Duration(p.time_from_start).seconds() * 1000));
  }
  return h;
}
void cb_ct_add(const TrajectoryCandidate & c, const rclcpp::Time & now)
{
  if (!cb_ct_on()) return;
  std::ostringstream o;
  o << "{\"schema\":\"cb_compare_v1\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"add\",\"candidate_id\":" << c.id
    << ",\"trajectory_signature\":" << cb_ct_sig(c.trajectory_map) << ",\"eval_time_sec\":";
  cb_ct_num(o, now.seconds());
  o << ",\"generation_time_sec\":";
  cb_ct_num(o, c.generation_time.seconds());
  o << ",\"trajectory_header_stamp_sec\":";
  cb_ct_num(o, rclcpp::Time(c.trajectory_map.header.stamp).seconds());
  o << ",\"trajectory_xyvt\":[";
  for (std::size_t i = 0; i < c.trajectory_map.points.size(); ++i) {
    if (i) o << ',';
    const auto & p = c.trajectory_map.points[i];
    o << '[';
    cb_ct_num(o, p.pose.position.x);
    o << ',';
    cb_ct_num(o, p.pose.position.y);
    o << ',';
    cb_ct_num(o, p.longitudinal_velocity_mps);
    o << ',';
    cb_ct_num(o, rclcpp::Duration(p.time_from_start).seconds());
    o << ']';
  }
  o << "]}";
  cb_ct_write(o.str());
}
void cb_ct_state(
  const char * stage, const TrajectoryCandidate & c, const nav_msgs::msg::Odometry & ego,
  const rclcpp::Time & now, const std::shared_ptr<const TrackedObjects> & tracked = {},
  const PredictedObjects * pred = nullptr)
{
  if (!cb_ct_on()) return;
  std::ostringstream o;
  o << "{\"schema\":\"cb_compare_v1\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"candidate_state\",\"stage\":\"" << stage << "\",\"candidate_id\":" << c.id
    << ",\"trajectory_signature\":" << cb_ct_sig(c.trajectory_map) << ",\"eval_time_sec\":";
  cb_ct_num(o, now.seconds());
  o << ",\"ego_stamp_sec\":";
  cb_ct_num(o, rclcpp::Time(ego.header.stamp).seconds());
  o << ",\"ego_x\":";
  cb_ct_num(o, ego.pose.pose.position.x);
  o << ",\"ego_y\":";
  cb_ct_num(o, ego.pose.pose.position.y);
  o << ",\"age_sec\":";
  cb_ct_num(o, (now - c.generation_time).seconds());
  o << ",\"live_start_index\":" << c.live_start_index << ",\"projection_error_m\":";
  cb_ct_num(o, c.projection_error_m);
  o << ",\"remaining_horizon_sec\":";
  cb_ct_num(o, c.remaining_horizon_sec);
  o << ",\"valid\":" << (c.valid ? "true" : "false") << ",\"safe\":" << (c.safe ? "true" : "false")
    << ",\"score\":";
  cb_ct_num(o, c.score);
  if (c.live_start_index < c.trajectory_map.points.size()) {
    const auto & p = c.trajectory_map.points[c.live_start_index];
    o << ",\"live_point\":{\"x\":";
    cb_ct_num(o, p.pose.position.x);
    o << ",\"y\":";
    cb_ct_num(o, p.pose.position.y);
    o << ",\"v\":";
    cb_ct_num(o, p.longitudinal_velocity_mps);
    o << ",\"original_time_from_start\":";
    cb_ct_num(o, rclcpp::Duration(p.time_from_start).seconds());
    o << '}';
  }
  if (tracked) {
    o << ",\"tracked_stamp_sec\":";
    cb_ct_num(o, rclcpp::Time(tracked->header.stamp).seconds());
  }
  if (pred) {
    o << ",\"predicted_stamp_sec\":";
    cb_ct_num(o, rclcpp::Time(pred->header.stamp).seconds());
  }
  o << '}';
  cb_ct_write(o.str());
}
void cb_ct_select(
  const TrajectoryCandidate & s, const TrajectoryCandidate & b, const CandidateBankDebugInfo & d,
  const rclcpp::Time & now)
{
  if (!cb_ct_on()) return;
  std::ostringstream o;
  o << "{\"schema\":\"cb_compare_v1\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"selection\",\"eval_time_sec\":";
  cb_ct_num(o, now.seconds());
  o << ",\"selected_id\":" << s.id << ",\"selected_signature\":" << cb_ct_sig(s.trajectory_map)
    << ",\"best_id\":" << b.id << ",\"selection_reason\":\"" << d.selection_reason
    << "\",\"hysteresis_kept\":" << (d.hysteresis_kept ? "true" : "false")
    << ",\"selected_age_sec\":";
  cb_ct_num(o, (now - s.generation_time).seconds());
  o << ",\"selected_live_start_index\":" << s.live_start_index
    << ",\"selected_projection_error_m\":";
  cb_ct_num(o, s.projection_error_m);
  o << ",\"selected_remaining_horizon_sec\":";
  cb_ct_num(o, s.remaining_horizon_sec);
  if (s.live_start_index < s.trajectory_map.points.size()) {
    const auto & p = s.trajectory_map.points[s.live_start_index];
    o << ",\"selected_live_point\":{\"v\":";
    cb_ct_num(o, p.longitudinal_velocity_mps);
    o << ",\"original_time_from_start\":";
    cb_ct_num(o, rclcpp::Duration(p.time_from_start).seconds());
    o << '}';
  }
  o << '}';
  cb_ct_write(o.str());
}
// CB_COMPARE_TRACE_V1_END

// CB_UNIFIED_TRACE_V01_BEGIN
void cb_ct_candidate_detail(
  const TrajectoryCandidate & c, const nav_msgs::msg::Odometry & ego, const rclcpp::Time & now)
{
  if (!cb_ct_on()) return;
  std::ostringstream o;
  o << "{\"schema\":\"cb_diag_v2\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"candidate_detail\",\"candidate_id\":" << c.id << ",\"eval_time_sec\":";
  cb_ct_num(o, now.seconds());
  o << ",\"ego_x\":";
  cb_ct_num(o, ego.pose.pose.position.x);
  o << ",\"ego_y\":";
  cb_ct_num(o, ego.pose.pose.position.y);
  o << ",\"age_sec\":";
  cb_ct_num(o, (now - c.generation_time).seconds());
  o << ",\"live_start_index\":" << c.live_start_index << ",\"projection_error_m\":";
  cb_ct_num(o, c.projection_error_m);
  o << ",\"remaining_horizon_sec\":";
  cb_ct_num(o, c.remaining_horizon_sec);
  o << ",\"valid\":" << (c.valid ? "true" : "false") << ",\"safe\":" << (c.safe ? "true" : "false")
    << ",\"selected\":" << (c.selected ? "true" : "false")
    << ",\"source_id\":" << static_cast<int>(c.source) << ",\"score\":";
  cb_ct_num(o, c.score);
  o << ",\"score_progress\":";
  cb_ct_num(o, c.score_progress);
  o << ",\"score_route_deviation\":";
  cb_ct_num(o, c.score_route_deviation);
  o << ",\"score_path_efficiency\":";
  cb_ct_num(o, c.score_path_efficiency);
  o << ",\"score_static_clearance\":";
  cb_ct_num(o, c.score_static_clearance);
  o << ",\"score_dynamic_margin\":";
  cb_ct_num(o, c.score_dynamic_margin);
  o << ",\"score_comfort\":";
  cb_ct_num(o, c.score_comfort);
  o << ",\"score_traffic_rules\":";
  cb_ct_num(o, c.score_traffic_rules);
  o << ",\"score_lane_boundary\":";
  cb_ct_num(o, c.score_lane_boundary);
  o << ",\"score_lane_change_deadline\":";
  cb_ct_num(o, c.score_lane_change_deadline);
  o << ",\"min_static_clearance_m\":";
  cb_ct_num(o, c.min_static_clearance_m);
  o << ",\"min_dynamic_clearance_m\":";
  cb_ct_num(o, c.min_dynamic_clearance_m);
  o << ",\"min_dynamic_time_gap_sec\":";
  cb_ct_num(o, c.min_dynamic_time_gap_sec);
  o << ",\"min_dynamic_ttc_sec\":";
  cb_ct_num(o, c.min_dynamic_ttc_sec);
  o << ",\"static_collision_margin_violated\":"
    << (c.static_collision_margin_violated ? "true" : "false")
    << ",\"dynamic_collision_margin_violated\":"
    << (c.dynamic_collision_margin_violated ? "true" : "false");
  if (c.live_start_index < c.trajectory_map.points.size()) {
    const auto & p = c.trajectory_map.points[c.live_start_index];
    o << ",\"live_x\":";
    cb_ct_num(o, p.pose.position.x);
    o << ",\"live_y\":";
    cb_ct_num(o, p.pose.position.y);
    o << ",\"live_v\":";
    cb_ct_num(o, p.longitudinal_velocity_mps);
    o << ",\"live_original_time_sec\":";
    cb_ct_num(o, rclcpp::Duration(p.time_from_start).seconds());
  }
  o << '}';
  cb_ct_write(o.str());
}

void cb_ct_cross_aggregate(const CandidateCrossAggregateResult & a, const rclcpp::Time & now)
{
  if (!cb_ct_on()) return;
  std::ostringstream o;
  o << "{\"schema\":\"cb_diag_v2\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"cross_aggregate\",\"candidate_id\":" << a.candidate_id
    << ",\"eval_time_sec\":";
  cb_ct_num(o, now.seconds());
  o << ",\"scenario_count\":" << a.scenario_count
    << ",\"valid_scenario_count\":" << a.valid_scenario_count
    << ",\"collision_scenario_count\":" << a.collision_scenario_count
    << ",\"xyt_conflict_scenario_count\":" << a.xyt_conflict_scenario_count;
  o << ",\"mean_scenario_risk\":";
  cb_ct_num(o, a.mean_scenario_risk);
  o << ",\"paired_risk\":";
  cb_ct_num(o, a.paired_risk);
  o << ",\"off_diagonal_mean_risk\":";
  cb_ct_num(o, a.off_diagonal_mean_risk);
  o << ",\"score_before_cross\":";
  cb_ct_num(o, a.score_before_cross);
  o << ",\"score_penalty\":";
  cb_ct_num(o, a.score_penalty);
  o << ",\"score_after_cross\":";
  cb_ct_num(o, a.score_after_cross);
  o << ",\"selector_effective_score\":";
  cb_ct_num(o, a.selector_effective_score);
  o << ",\"selector_eligible\":" << (a.selector_eligible ? "true" : "false")
    << ",\"selector_hard_rejected\":" << (a.selector_hard_rejected ? "true" : "false")
    << ",\"safety_tier\":\"" << a.safety_tier << "\"}";
  cb_ct_write(o.str());
}
// CB_UNIFIED_TRACE_V01_END

// CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN
const char * candidate_source_name(const CandidateSource source)
{
  switch (source) {
    case CandidateSource::LEGACY_OUTPUT:
      return "legacy_output";
    case CandidateSource::INCUMBENT_SEED:
      return "incumbent_seed";
    case CandidateSource::ROUTE_SEED:
      return "route_seed";
    case CandidateSource::PROCEED_SEED:
      return "proceed_seed";
    case CandidateSource::STOP_SEED:
      return "stop_seed";
    case CandidateSource::AVOID_LEFT_SEED:
      return "avoid_left_seed";
    case CandidateSource::AVOID_RIGHT_SEED:
      return "avoid_right_seed";
  }
  return "unknown";
}
// CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
spatiotemporal_collision::Config make_cross_collision_config(const CandidateBankConfig & config)
{
  spatiotemporal_collision::Config out;
  out.enabled = config.cross_collision_enabled;
  out.spatial_scale_m = config.cross_collision_spatial_scale_m;
  out.temporal_scale_sec = config.cross_collision_temporal_scale_sec;
  out.conflict_rho = config.cross_collision_conflict_rho;
  out.block_duration_sec = config.cross_collision_block_duration_sec;
  out.lateral_undercoverage_m = config.cross_collision_lateral_undercoverage_m;
  out.max_circles_per_body = config.cross_collision_max_circles_per_body;
  out.hard_collision_margin_m = config.cross_collision_hard_collision_margin_m;
  out.xyt_spatial_margin_m = config.cross_collision_xyt_spatial_margin_m;
  // CANDIDATE_BANK_CROSS_FOLLOWING_COST_V1_BEGIN
  out.following_enabled = config.cross_following_enabled;
  out.following_preferred_clearance_m = config.cross_following_preferred_clearance_m;
  out.following_lateral_margin_m = config.cross_following_lateral_margin_m;
  out.following_risk_weight = config.cross_following_risk_weight;
  // CANDIDATE_BANK_CROSS_FOLLOWING_COST_V1_END
  // CANDIDATE_BANK_TOI_V1_BEGIN
  out.compute_toi = config.cross_collision_compute_toi;
  // CANDIDATE_BANK_TOI_V1_END
  out.min_prediction_confidence = config.min_prediction_confidence;
  out.ego_length_m = config.ego_length_m;
  out.ego_width_m = config.ego_width_m;
  out.ego_base_link_to_center_m = config.ego_base_link_to_center_m;
  return out;
}

std::vector<CandidateCrossCollisionResult> evaluate_and_log_cross_collision_matrix(
  const std::vector<TrajectoryCandidate> & candidates, const CandidateBankConfig & bank_config,
  const rclcpp::Time & now)
{
  std::vector<CandidateCrossCollisionResult> matrix;
  if (!bank_config.cross_collision_enabled || candidates.empty()) return matrix;
  matrix.reserve(candidates.size() * candidates.size());

  const auto config = make_cross_collision_config(bank_config);
  std::vector<spatiotemporal_collision::PreparedBodyTrajectory> ego_trajectories;
  std::vector<spatiotemporal_collision::PreparedScenario> scenarios;
  std::vector<double> scenario_ages;
  ego_trajectories.reserve(candidates.size());
  scenarios.reserve(candidates.size());
  scenario_ages.reserve(candidates.size());

  // Precompute geometry once per bank entry.  The expensive sin/cos, adaptive
  // circle placement, interval AABBs and XYT block hierarchy are therefore NOT
  // repeated for every E_i x O_j matrix cell.
  for (const auto & candidate : candidates) {
    ego_trajectories.push_back(
      spatiotemporal_collision::prepare_ego(
        candidate.trajectory_map, candidate.live_start_index, config));
    const double age_sec = std::max(0.0, (now - candidate.generation_time).seconds());
    scenario_ages.push_back(age_sec);
    scenarios.push_back(
      spatiotemporal_collision::prepare_scenario(
        candidate.paired_predicted_objects, age_sec, config));
  }

  using Clock = std::chrono::steady_clock;
  for (std::size_t ego_index = 0; ego_index < candidates.size(); ++ego_index) {
    for (std::size_t scenario_index = 0; scenario_index < candidates.size(); ++scenario_index) {
      const auto start = Clock::now();
      const auto result = spatiotemporal_collision::evaluate(
        ego_trajectories[ego_index], scenarios[scenario_index], config);
      const auto finish = Clock::now();

      CandidateCrossCollisionResult matrix_entry;
      matrix_entry.ego_candidate_id = candidates[ego_index].id;
      matrix_entry.scenario_candidate_id = candidates[scenario_index].id;
      matrix_entry.paired = ego_index == scenario_index;
      matrix_entry.scenario_age_sec = scenario_ages[scenario_index];
      matrix_entry.valid = result.valid;
      matrix_entry.same_time_collision = result.same_time_collision;
      matrix_entry.xyt_conflict = result.xyt_conflict;
      matrix_entry.trajectory_conflict = result.trajectory_conflict;
      // CANDIDATE_BANK_TOI_V1_BEGIN
      matrix_entry.first_collision_time_sec = result.first_collision_time_sec;
      matrix_entry.first_collision_object_id = result.first_collision_object_id;
      matrix_entry.first_collision_path_index = result.first_collision_path_index;
      matrix_entry.first_collision_path_confidence = result.first_collision_path_confidence;
      matrix_entry.first_collision_ego_circle_index = result.first_collision_ego_circle_index;
      matrix_entry.first_collision_object_circle_index = result.first_collision_object_circle_index;
      // CANDIDATE_BANK_TOI_V1_END
      // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
      matrix_entry.scenario_weight = result.valid ? 1.0 : 0.0;
      if (result.same_time_collision) {
        matrix_entry.scenario_risk = 1.0;
      } else if (result.xyt_conflict && std::isfinite(result.min_xyt_rho)) {
        const double normalized = std::max(
          0.0,
          1.0 - result.min_xyt_rho / std::max(bank_config.cross_collision_conflict_rho, 1.0e-9));
        matrix_entry.scenario_risk = normalized * normalized;
      }
      // CANDIDATE_BANK_CROSS_FOLLOWING_COST_V1_BEGIN
      if (bank_config.cross_following_enabled) {
        // Keep scenario_risk in [0,1]. Collision/XYT and following-distance risk are alternative
        // reasons to disfavor the same E_i x O_j cell, so take the stronger one rather than
        // summing.
        matrix_entry.scenario_risk = std::max(matrix_entry.scenario_risk, result.following_risk);
      }
      // CANDIDATE_BANK_CROSS_FOLLOWING_COST_V1_END
      // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
      matrix_entry.min_same_time_clearance_m = result.min_same_time_clearance_m;
      matrix_entry.min_xyt_rho = result.min_xyt_rho;
      matrix_entry.xyt_spatial_distance_m = result.xyt_spatial_distance_m;
      matrix_entry.xyt_spatial_clearance_m = result.xyt_spatial_clearance_m;
      matrix_entry.xyt_ego_time_sec = result.xyt_ego_time_sec;
      matrix_entry.xyt_object_time_sec = result.xyt_object_time_sec;
      matrix_entry.xyt_delta_time_sec = result.xyt_delta_time_sec;
      matrix_entry.object_id = result.object_id;
      matrix_entry.path_index = result.path_index;
      matrix_entry.path_confidence = result.path_confidence;
      matrix_entry.ego_circle_index = result.ego_circle_index;
      matrix_entry.object_circle_index = result.object_circle_index;
      matrix_entry.valid_path_count = result.valid_path_count;
      matrix_entry.evaluation_us =
        std::chrono::duration<double, std::micro>(finish - start).count();
      matrix.push_back(matrix_entry);

      internal_debug::CandidateCrossCollisionLog log;
      log.ego_candidate_id = candidates[ego_index].id;
      log.scenario_candidate_id = candidates[scenario_index].id;
      log.ego_source = candidate_source_name(candidates[ego_index].source);
      log.scenario_source = candidate_source_name(candidates[scenario_index].source);
      log.paired = ego_index == scenario_index;
      log.scenario_age_sec = scenario_ages[scenario_index];
      log.valid = result.valid;
      log.same_time_collision = result.same_time_collision;
      log.xyt_conflict = result.xyt_conflict;
      log.trajectory_conflict = result.trajectory_conflict;
      // CANDIDATE_BANK_TOI_V1_BEGIN
      log.first_collision_time_sec = result.first_collision_time_sec;
      log.first_collision_object_id = result.first_collision_object_id;
      log.first_collision_path_index = result.first_collision_path_index;
      log.first_collision_path_confidence = result.first_collision_path_confidence;
      log.first_collision_ego_circle_index = result.first_collision_ego_circle_index;
      log.first_collision_object_circle_index = result.first_collision_object_circle_index;
      // CANDIDATE_BANK_TOI_V1_END
      // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
      log.scenario_weight = matrix_entry.scenario_weight;
      log.scenario_risk = matrix_entry.scenario_risk;
      // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
      log.min_same_time_clearance_m = result.min_same_time_clearance_m;
      log.min_xyt_rho = result.min_xyt_rho;
      log.xyt_spatial_distance_m = result.xyt_spatial_distance_m;
      log.xyt_spatial_clearance_m = result.xyt_spatial_clearance_m;
      log.xyt_ego_time_sec = result.xyt_ego_time_sec;
      log.xyt_object_time_sec = result.xyt_object_time_sec;
      log.xyt_delta_time_sec = result.xyt_delta_time_sec;
      log.object_id = result.object_id;
      log.path_index = result.path_index;
      log.path_confidence = result.path_confidence;
      log.ego_circle_index = result.ego_circle_index;
      log.object_circle_index = result.object_circle_index;
      log.valid_path_count = result.valid_path_count;
      log.path_tests = result.stats.path_tests;
      log.path_rejects = result.stats.path_rejects;
      log.block_tests = result.stats.block_tests;
      log.block_rejects = result.stats.block_rejects;
      log.interval_tests = result.stats.interval_tests;
      log.interval_rejects = result.stats.interval_rejects;
      log.circle_aabb_tests = result.stats.circle_aabb_tests;
      log.circle_aabb_rejects = result.stats.circle_aabb_rejects;
      log.xyt_segment_tests = result.stats.xyt_segment_tests;
      log.same_time_interval_tests = result.stats.same_time_interval_tests;
      log.same_time_circle_tests = result.stats.same_time_circle_tests;
      log.evaluation_us = matrix_entry.evaluation_us;
      internal_debug::record_candidate_cross_collision(internal_debug::current_cycle_id(), log);
    }
  }
  return matrix;
}

// CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
const CandidateCrossAggregateResult * find_cross_aggregate(
  const std::vector<CandidateCrossAggregateResult> & aggregates, const std::uint64_t candidate_id)
{
  const auto it = std::find_if(aggregates.begin(), aggregates.end(), [&](const auto & aggregate) {
    return aggregate.candidate_id == candidate_id;
  });
  return it == aggregates.end() ? nullptr : &(*it);
}

// CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
enum class CandidateSafetyTier : int {
  SAFE = 0,
  CAUTION = 1,
  UNSAFE = 2,
};

struct CandidateSafetyAssessment
{
  CandidateSafetyTier tier{CandidateSafetyTier::SAFE};
  bool any_cross_collision{false};
  bool imminent_cross_collision{false};
  std::size_t baseline_violation_count{0};
  double baseline_clearance_deficit_m{0.0};
  double collision_fraction{0.0};
  double xyt_fraction{0.0};
  double min_toi_sec{std::numeric_limits<double>::infinity()};
};

const char * safety_tier_name(const CandidateSafetyTier tier)
{
  switch (tier) {
    case CandidateSafetyTier::SAFE:
      return "SAFE";
    case CandidateSafetyTier::CAUTION:
      return "CAUTION";
    case CandidateSafetyTier::UNSAFE:
      return "UNSAFE";
  }
  return "UNKNOWN";
}

CandidateSafetyAssessment assess_candidate_safety(
  const TrajectoryCandidate & candidate,
  const std::vector<CandidateCrossAggregateResult> & aggregates, const CandidateBankConfig & config)
{
  CandidateSafetyAssessment out;
  const auto * aggregate = find_cross_aggregate(aggregates, candidate.id);
  if (aggregate != nullptr) {
    const bool raw_cross_collision = aggregate->collision_scenario_count > 0U;
    out.any_cross_collision = config.safety_tier_use_same_time_collision && raw_cross_collision;
    out.collision_fraction =
      config.safety_tier_use_same_time_collision ? aggregate->collision_scenario_fraction : 0.0;
    out.xyt_fraction = config.safety_tier_use_xyt_conflict_for_caution
                         ? aggregate->xyt_conflict_scenario_fraction
                         : 0.0;
    out.min_toi_sec = config.safety_tier_use_same_time_collision
                        ? aggregate->min_toi_sec
                        : std::numeric_limits<double>::infinity();
    out.imminent_cross_collision = out.any_cross_collision && std::isfinite(out.min_toi_sec) &&
                                   out.min_toi_sec <= config.safety_tier_imminent_horizon_sec;
  }

  if (config.safety_tier_use_static_margin && candidate.static_collision_margin_violated) {
    ++out.baseline_violation_count;
    if (std::isfinite(candidate.min_static_clearance_m)) {
      out.baseline_clearance_deficit_m +=
        std::max(0.0, config.static_collision_margin_m - candidate.min_static_clearance_m);
    }
  }
  if (config.safety_tier_use_dynamic_margin && candidate.dynamic_collision_margin_violated) {
    ++out.baseline_violation_count;
    if (std::isfinite(candidate.min_dynamic_clearance_m)) {
      out.baseline_clearance_deficit_m +=
        std::max(0.0, config.dynamic_collision_margin_m - candidate.min_dynamic_clearance_m);
    }
  }

  if (out.baseline_violation_count > 0U || out.imminent_cross_collision) {
    out.tier = CandidateSafetyTier::UNSAFE;
    return out;
  }
  const bool long_horizon_collision =
    config.safety_tier_use_same_time_collision && out.any_cross_collision;
  const bool xyt_caution =
    config.safety_tier_use_xyt_conflict_for_caution && out.xyt_fraction > 0.0;
  if (long_horizon_collision || xyt_caution) {
    out.tier = CandidateSafetyTier::CAUTION;
  }
  return out;
}

bool less_with_epsilon(const double lhs, const double rhs, const double eps = 1.0e-9)
{
  return lhs < rhs - eps;
}

bool greater_with_epsilon(const double lhs, const double rhs, const double eps = 1.0e-9)
{
  return lhs > rhs + eps;
}

bool candidate_is_preferred(
  const TrajectoryCandidate & lhs, const TrajectoryCandidate & rhs,
  const std::vector<CandidateCrossAggregateResult> & aggregates,
  const CandidateBankConfig & config);
// CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END

double selector_effective_score(
  const TrajectoryCandidate & candidate,
  const std::vector<CandidateCrossAggregateResult> & aggregates, const CandidateBankConfig & config)
{
  if (!config.cross_aggregation_enabled || !config.cross_aggregation_apply_to_score) {
    return candidate.score;
  }
  const auto * aggregate = find_cross_aggregate(aggregates, candidate.id);
  if (aggregate == nullptr || !std::isfinite(candidate.score)) return candidate.score;
  return candidate.score + aggregate->score_penalty;
}

bool selector_effective_eligible(
  const TrajectoryCandidate & candidate,
  const std::vector<CandidateCrossAggregateResult> & aggregates, const CandidateBankConfig & config)
{
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
  // Safety-tier mode never drops an otherwise valid trajectory solely because all candidates
  // violate a margin.  This prevents the old all-rejected -> nullopt -> raw Diffusion fallback.
  if (config.safety_tier_enabled) return candidate.valid;
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
  if (!candidate.valid || !candidate.safe) return false;
  if (
    !config.cross_aggregation_enabled ||
    !config.cross_aggregation_apply_short_horizon_hard_reject) {
    return true;
  }
  const auto * aggregate = find_cross_aggregate(aggregates, candidate.id);
  return aggregate == nullptr || !aggregate->short_horizon_hard_reject;
}

// CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
bool candidate_is_preferred(
  const TrajectoryCandidate & lhs, const TrajectoryCandidate & rhs,
  const std::vector<CandidateCrossAggregateResult> & aggregates, const CandidateBankConfig & config)
{
  if (!config.safety_tier_enabled) {
    return selector_effective_score(lhs, aggregates, config) <
           selector_effective_score(rhs, aggregates, config);
  }

  const auto a = assess_candidate_safety(lhs, aggregates, config);
  const auto b = assess_candidate_safety(rhs, aggregates, config);
  if (a.tier != b.tier) {
    return static_cast<int>(a.tier) < static_cast<int>(b.tier);
  }

  // SAFE: keep the existing planner objective.  Hysteresis is applied later in select().
  if (a.tier == CandidateSafetyTier::SAFE) {
    return selector_effective_score(lhs, aggregates, config) <
           selector_effective_score(rhs, aggregates, config);
  }

  // CAUTION/UNSAFE: compare physical safety before progress/comfort/route score.
  // For CAUTION, an XYT-only candidate is preferred over one with a predicted same-time
  // collision outside the imminent horizon.  For UNSAFE, a margin-only violation is preferred
  // over an imminent physical collision when other information is available.
  if (a.imminent_cross_collision != b.imminent_cross_collision) {
    return !a.imminent_cross_collision;
  }
  if (a.any_cross_collision != b.any_cross_collision) {
    return !a.any_cross_collision;
  }
  if (a.baseline_violation_count != b.baseline_violation_count) {
    return a.baseline_violation_count < b.baseline_violation_count;
  }
  if (less_with_epsilon(a.baseline_clearance_deficit_m, b.baseline_clearance_deficit_m)) {
    return true;
  }
  if (greater_with_epsilon(a.baseline_clearance_deficit_m, b.baseline_clearance_deficit_m)) {
    return false;
  }
  if (greater_with_epsilon(a.min_toi_sec, b.min_toi_sec)) return true;
  if (less_with_epsilon(a.min_toi_sec, b.min_toi_sec)) return false;
  if (less_with_epsilon(a.collision_fraction, b.collision_fraction)) return true;
  if (greater_with_epsilon(a.collision_fraction, b.collision_fraction)) return false;
  if (less_with_epsilon(a.xyt_fraction, b.xyt_fraction)) return true;
  if (greater_with_epsilon(a.xyt_fraction, b.xyt_fraction)) return false;

  if (config.safety_tier_prefer_stop_seed_when_non_safe) {
    const bool lhs_stop = lhs.source == CandidateSource::STOP_SEED;
    const bool rhs_stop = rhs.source == CandidateSource::STOP_SEED;
    if (lhs_stop != rhs_stop) return lhs_stop;
  }
  return selector_effective_score(lhs, aggregates, config) <
         selector_effective_score(rhs, aggregates, config);
}
// CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END

std::vector<CandidateCrossAggregateResult> aggregate_cross_collision(
  const std::vector<TrajectoryCandidate> & candidates,
  const std::vector<CandidateCrossCollisionResult> & matrix, const CandidateBankConfig & config)
{
  std::vector<CandidateCrossAggregateResult> aggregates;
  if (!config.cross_aggregation_enabled || candidates.empty()) return aggregates;
  aggregates.reserve(candidates.size());

  for (const auto & candidate : candidates) {
    CandidateCrossAggregateResult aggregate;
    aggregate.candidate_id = candidate.id;
    aggregate.scenario_count = candidates.size();

    double weighted_risk_sum = 0.0;
    double offdiag_weight_sum = 0.0;
    double offdiag_risk_sum = 0.0;
    double collision_toi_sum = 0.0;
    std::size_t finite_collision_toi_count = 0U;

    for (const auto & cell : matrix) {
      if (cell.ego_candidate_id != candidate.id || !cell.valid) continue;
      const double weight = cell.scenario_weight;  // uniform in 0013a8
      if (!(weight > 0.0)) continue;
      ++aggregate.valid_scenario_count;
      aggregate.scenario_weight_sum += weight;
      weighted_risk_sum += weight * cell.scenario_risk;
      if (cell.paired) {
        aggregate.paired_risk = cell.scenario_risk;
      } else {
        offdiag_weight_sum += weight;
        offdiag_risk_sum += weight * cell.scenario_risk;
      }
      if (cell.same_time_collision) {
        ++aggregate.collision_scenario_count;
        if (std::isfinite(cell.first_collision_time_sec)) {
          aggregate.min_toi_sec = std::min(aggregate.min_toi_sec, cell.first_collision_time_sec);
          collision_toi_sum += cell.first_collision_time_sec;
          ++finite_collision_toi_count;
          if (cell.first_collision_time_sec <= config.cross_aggregation_short_horizon_sec) {
            ++aggregate.short_horizon_collision_scenario_count;
          }
        }
      }
      if (cell.xyt_conflict) ++aggregate.xyt_conflict_scenario_count;
      if (std::isfinite(cell.min_same_time_clearance_m)) {
        aggregate.min_same_time_clearance_m =
          std::min(aggregate.min_same_time_clearance_m, cell.min_same_time_clearance_m);
      }
    }

    if (aggregate.valid_scenario_count > 0U) {
      const double denom = static_cast<double>(aggregate.valid_scenario_count);
      aggregate.collision_scenario_fraction =
        static_cast<double>(aggregate.collision_scenario_count) / denom;
      aggregate.xyt_conflict_scenario_fraction =
        static_cast<double>(aggregate.xyt_conflict_scenario_count) / denom;
    }
    if (aggregate.scenario_weight_sum > 0.0) {
      aggregate.mean_scenario_risk = weighted_risk_sum / aggregate.scenario_weight_sum;
    }
    if (offdiag_weight_sum > 0.0) {
      aggregate.off_diagonal_mean_risk = offdiag_risk_sum / offdiag_weight_sum;
    }
    if (finite_collision_toi_count > 0U) {
      aggregate.mean_collision_toi_sec =
        collision_toi_sum / static_cast<double>(finite_collision_toi_count);
    }

    aggregate.short_horizon_hard_reject = aggregate.short_horizon_collision_scenario_count > 0U;
    aggregate.score_penalty = config.cross_aggregation_score_weight * aggregate.mean_scenario_risk;
    aggregates.push_back(aggregate);
  }
  return aggregates;
}

void finalize_and_log_cross_aggregates(
  const std::vector<TrajectoryCandidate> & candidates,
  std::vector<CandidateCrossAggregateResult> & aggregates, const CandidateBankConfig & config)
{
  if (!config.cross_aggregation_enabled || aggregates.empty()) return;

  for (auto & aggregate : aggregates) {
    const auto it = std::find_if(candidates.begin(), candidates.end(), [&](const auto & candidate) {
      return candidate.id == aggregate.candidate_id;
    });
    if (it == candidates.end()) continue;
    aggregate.score_before_cross = it->score;
    aggregate.score_after_cross =
      std::isfinite(it->score) ? it->score + aggregate.score_penalty : it->score;
    aggregate.base_safe = it->safe;
    aggregate.selector_score_enabled = config.cross_aggregation_apply_to_score;
    aggregate.selector_hard_rejected = !config.safety_tier_enabled &&
                                       config.cross_aggregation_apply_short_horizon_hard_reject &&
                                       aggregate.short_horizon_hard_reject;
    aggregate.selector_eligible = config.safety_tier_enabled
                                    ? it->valid
                                    : (it->valid && it->safe && !aggregate.selector_hard_rejected);
    aggregate.selector_effective_score = config.cross_aggregation_apply_to_score
                                           ? aggregate.score_after_cross
                                           : aggregate.score_before_cross;
    // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
    aggregate.safety_tier =
      config.safety_tier_enabled
        ? safety_tier_name(assess_candidate_safety(*it, aggregates, config).tier)
        : "DISABLED";
    // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
    aggregate.shadow_best = false;
  }

  std::size_t best_index = aggregates.size();
  const TrajectoryCandidate * best_candidate = nullptr;
  for (std::size_t i = 0; i < aggregates.size(); ++i) {
    const auto & aggregate = aggregates[i];
    if (!aggregate.selector_eligible || !std::isfinite(aggregate.selector_effective_score))
      continue;
    const auto it = std::find_if(candidates.begin(), candidates.end(), [&](const auto & candidate) {
      return candidate.id == aggregate.candidate_id;
    });
    if (it == candidates.end()) continue;
    if (
      best_candidate == nullptr ||
      candidate_is_preferred(*it, *best_candidate, aggregates, config)) {
      best_candidate = &(*it);
      best_index = i;
    }
  }
  if (best_index < aggregates.size()) aggregates[best_index].shadow_best = true;

  for (const auto & aggregate : aggregates) {
    const auto it = std::find_if(candidates.begin(), candidates.end(), [&](const auto & candidate) {
      return candidate.id == aggregate.candidate_id;
    });
    internal_debug::CandidateCrossAggregateLog log;
    log.candidate_id = aggregate.candidate_id;
    log.source = it == candidates.end() ? "unknown" : candidate_source_name(it->source);
    log.weight_mode = config.cross_aggregation_weight_mode;
    log.aggregation_enabled = config.cross_aggregation_enabled;
    log.apply_to_score = config.cross_aggregation_apply_to_score;
    log.apply_short_horizon_hard_reject = config.cross_aggregation_apply_short_horizon_hard_reject;
    log.score_weight = config.cross_aggregation_score_weight;
    log.short_horizon_sec = config.cross_aggregation_short_horizon_sec;
    log.scenario_count = aggregate.scenario_count;
    log.valid_scenario_count = aggregate.valid_scenario_count;
    log.collision_scenario_count = aggregate.collision_scenario_count;
    log.xyt_conflict_scenario_count = aggregate.xyt_conflict_scenario_count;
    log.short_horizon_collision_scenario_count = aggregate.short_horizon_collision_scenario_count;
    log.scenario_weight_sum = aggregate.scenario_weight_sum;
    log.collision_scenario_fraction = aggregate.collision_scenario_fraction;
    log.xyt_conflict_scenario_fraction = aggregate.xyt_conflict_scenario_fraction;
    log.min_toi_sec = aggregate.min_toi_sec;
    log.mean_collision_toi_sec = aggregate.mean_collision_toi_sec;
    log.min_same_time_clearance_m = aggregate.min_same_time_clearance_m;
    log.paired_risk = aggregate.paired_risk;
    log.off_diagonal_mean_risk = aggregate.off_diagonal_mean_risk;
    log.mean_scenario_risk = aggregate.mean_scenario_risk;
    log.score_before_cross = aggregate.score_before_cross;
    log.score_penalty = aggregate.score_penalty;
    log.score_after_cross = aggregate.score_after_cross;
    log.selector_effective_score = aggregate.selector_effective_score;
    log.base_safe = aggregate.base_safe;
    log.short_horizon_hard_reject = aggregate.short_horizon_hard_reject;
    log.selector_score_enabled = aggregate.selector_score_enabled;
    log.selector_hard_rejected = aggregate.selector_hard_rejected;
    log.selector_eligible = aggregate.selector_eligible;
    // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
    log.safety_tier = aggregate.safety_tier;
    // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
    log.shadow_best = aggregate.shadow_best;
    internal_debug::record_candidate_cross_aggregate(internal_debug::current_cycle_id(), log);
  }
}
// CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END

// CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END

// CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
std::string tracked_object_id_string(const TrackedObject & object)
{
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(object.object_id.uuid.size() * 2);
  for (const auto byte : object.object_id.uuid) {
    const auto value = static_cast<unsigned int>(byte);
    out.push_back(kHex[(value >> 4U) & 0x0FU]);
    out.push_back(kHex[value & 0x0FU]);
  }
  return out;
}
// CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
// CANDIDATE_BANK_DIAGNOSTICS_V2_END

double distance_xy(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double duration_sec(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).seconds();
}

std::size_t nearest_trajectory_index(
  const Trajectory & trajectory, const Point & query, double & out_distance)
{
  out_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const double distance = distance_xy(trajectory.points[i].pose.position, query);
    if (distance < out_distance) {
      out_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

std::size_t time_aligned_trajectory_index(
  const Trajectory & trajectory, const Point & query, const double target_time_sec,
  const double search_window_sec, double & out_distance)
{
  if (trajectory.points.empty()) {
    out_distance = std::numeric_limits<double>::infinity();
    return 0;
  }

  const double target = std::max(0.0, target_time_sec);
  const double window = std::max(0.0, search_window_sec);
  std::size_t begin = trajectory.points.size();
  std::size_t end = trajectory.points.size();

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const double point_time = duration_sec(trajectory.points[i].time_from_start);
    if (std::abs(point_time - target) <= window + kEpsilon) {
      if (begin == trajectory.points.size()) {
        begin = i;
      }
      end = i + 1;
    }
  }

  // If the configured window contains no sample, fall back to the closest sample in time.
  if (begin == trajectory.points.size()) {
    double best_time_error = std::numeric_limits<double>::infinity();
    std::size_t best_time_index = 0;
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      const double point_time = duration_sec(trajectory.points[i].time_from_start);
      const double time_error = std::abs(point_time - target);
      if (time_error < best_time_error) {
        best_time_error = time_error;
        best_time_index = i;
      }
    }
    begin = best_time_index;
    end = best_time_index + 1;
  }

  out_distance = std::numeric_limits<double>::infinity();
  std::size_t best_index = begin;
  for (std::size_t i = begin; i < end; ++i) {
    const double distance = distance_xy(trajectory.points[i].pose.position, query);
    if (distance < out_distance) {
      out_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

double live_time_sec(
  const Trajectory & trajectory, const std::size_t start_index, const std::size_t index)
{
  const double start = duration_sec(trajectory.points[start_index].time_from_start);
  const double current = duration_sec(trajectory.points[index].time_from_start);
  return std::max(0.0, current - start);
}

double remaining_horizon_sec(const Trajectory & trajectory, const std::size_t start_index)
{
  if (trajectory.points.empty() || start_index >= trajectory.points.size()) {
    return 0.0;
  }
  return live_time_sec(trajectory, start_index, trajectory.points.size() - 1);
}

Trajectory make_live_suffix(
  const Trajectory & trajectory, const std::size_t start_index, const rclcpp::Time & now)
{
  Trajectory live;
  if (trajectory.points.empty() || start_index >= trajectory.points.size()) {
    return live;
  }

  live.header = trajectory.header;
  live.header.stamp = now;
  const double old_start = duration_sec(trajectory.points[start_index].time_from_start);
  live.points.reserve(trajectory.points.size() - start_index);
  for (std::size_t i = start_index; i < trajectory.points.size(); ++i) {
    auto point = trajectory.points[i];
    point.time_from_start = rclcpp::Duration::from_seconds(
      std::max(0.0, duration_sec(point.time_from_start) - old_start));
    live.points.push_back(std::move(point));
  }
  return live;
}

double path_length_m(const Trajectory & trajectory, const std::size_t start_index)
{
  if (trajectory.points.size() < 2 || start_index >= trajectory.points.size() - 1) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = start_index + 1; i < trajectory.points.size(); ++i) {
    length +=
      distance_xy(trajectory.points[i - 1].pose.position, trajectory.points[i].pose.position);
  }
  return length;
}

double yaw_from_pose(const Pose & pose);
double tracked_speed_mps(const TrackedObject & object);

// CANDIDATE_BANK_COLLISION_GEOMETRY_V1_BEGIN
//
// Runtime-selectable geometry used by both collision-margin checks and soft
// clearance cost:
//   single_circle : legacy one-circle obstacle + one-circle ego
//   three_circle  : 0010 three-circle BOUNDING_BOX + one-circle ego
//   polygon       : oriented ego rectangle + exact BOUNDING_BOX/POLYGON footprint
//                   (CYLINDER remains a true circle against the ego polygon)
//
// Polygon distance is positive while separated. When polygons overlap, a
// convex-hull SAT penetration depth is returned as a negative clearance. This
// preserves useful severity information in 0009 soft-only mode instead of
// mapping every overlap to the same zero distance.
namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;
using BgMultiPoint = bg::model::multi_point<BgPoint>;
using BgSegment = bg::model::segment<BgPoint>;

struct CollisionCircle
{
  CollisionCircle() : center() {}

  Point center;
  double radius_m{0.0};
};

struct CollisionCircleSet
{
  std::array<CollisionCircle, 3> circles{};
  std::size_t size{0};
};

double legacy_object_radius_m(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return 0.5 *
           std::hypot(
             static_cast<double>(shape.dimensions.x), static_cast<double>(shape.dimensions.y));
  }
  if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return std::max(0.0, 0.5 * static_cast<double>(shape.dimensions.x));
  }
  if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    double radius = 0.0;
    for (const auto & point : shape.footprint.points) {
      radius =
        std::max(radius, std::hypot(static_cast<double>(point.x), static_cast<double>(point.y)));
    }
    return radius;
  }
  return 0.5 * std::hypot(
                 static_cast<double>(shape.dimensions.x), static_cast<double>(shape.dimensions.y));
}

CollisionCircleSet object_collision_circles(
  const autoware_perception_msgs::msg::Shape & shape, const Pose & pose,
  const std::string & geometry_mode)
{
  CollisionCircleSet result;

  if (
    geometry_mode == "three_circle" &&
    shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    constexpr std::size_t kCircleCount = 3;
    const double length_x = std::max(0.0, static_cast<double>(shape.dimensions.x));
    const double length_y = std::max(0.0, static_cast<double>(shape.dimensions.y));
    const bool split_along_x = length_x >= length_y;
    const double major = split_along_x ? length_x : length_y;
    const double minor = split_along_x ? length_y : length_x;

    if (major <= kEpsilon) {
      CollisionCircle circle;
      circle.center = pose.position;
      circle.radius_m = 0.5 * std::hypot(length_x, length_y);
      result.circles[0] = circle;
      result.size = 1;
      return result;
    }

    const double segment_length = major / static_cast<double>(kCircleCount);
    const double radius = std::hypot(0.5 * segment_length, 0.5 * minor);
    const double yaw = yaw_from_pose(pose);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    for (std::size_t i = 0; i < kCircleCount; ++i) {
      const double local_major = -0.5 * major + (static_cast<double>(i) + 0.5) * segment_length;
      const double local_x = split_along_x ? local_major : 0.0;
      const double local_y = split_along_x ? 0.0 : local_major;

      CollisionCircle circle;
      circle.center = pose.position;
      circle.center.x += cos_yaw * local_x - sin_yaw * local_y;
      circle.center.y += sin_yaw * local_x + cos_yaw * local_y;
      circle.radius_m = radius;
      result.circles[i] = circle;
    }
    result.size = kCircleCount;
    return result;
  }

  CollisionCircle circle;
  circle.center = pose.position;
  circle.radius_m = legacy_object_radius_m(shape);
  result.circles[0] = circle;
  result.size = 1;
  return result;
}

BgPoint transform_local_point(const double x, const double y, const Pose & pose)
{
  const double yaw = yaw_from_pose(pose);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return BgPoint(
    pose.position.x + cos_yaw * x - sin_yaw * y, pose.position.y + sin_yaw * x + cos_yaw * y);
}

std::optional<BgPolygon> polygon_from_points(const std::vector<BgPoint> & points)
{
  if (points.size() < 3) {
    return std::nullopt;
  }

  BgPolygon polygon;
  polygon.outer().assign(points.begin(), points.end());
  bg::correct(polygon);
  if (std::abs(bg::area(polygon)) > kEpsilon && bg::is_valid(polygon)) {
    return polygon;
  }

  // Perception footprints are expected to be simple/convex, but use the convex
  // hull as a conservative recovery path for duplicated or malformed ordering.
  BgMultiPoint multi_point;
  multi_point.assign(points.begin(), points.end());
  BgPolygon hull;
  bg::convex_hull(multi_point, hull);
  bg::correct(hull);
  if (std::abs(bg::area(hull)) <= kEpsilon) {
    return std::nullopt;
  }
  return hull;
}

std::optional<BgPolygon> make_box_polygon(
  const double length_m, const double width_m, const Pose & center_pose)
{
  if (length_m <= kEpsilon || width_m <= kEpsilon) {
    return std::nullopt;
  }
  const double half_length = 0.5 * length_m;
  const double half_width = 0.5 * width_m;
  return polygon_from_points(
    {transform_local_point(half_length, half_width, center_pose),
     transform_local_point(half_length, -half_width, center_pose),
     transform_local_point(-half_length, -half_width, center_pose),
     transform_local_point(-half_length, half_width, center_pose)});
}

std::optional<BgPolygon> make_ego_polygon(
  const Pose & base_link_pose, const CandidateBankConfig & config)
{
  Pose center_pose = base_link_pose;
  const double yaw = yaw_from_pose(base_link_pose);
  center_pose.position.x += config.ego_base_link_to_center_m * std::cos(yaw);
  center_pose.position.y += config.ego_base_link_to_center_m * std::sin(yaw);
  return make_box_polygon(config.ego_length_m, config.ego_width_m, center_pose);
}

std::optional<BgPolygon> make_object_polygon(
  const autoware_perception_msgs::msg::Shape & shape, const Pose & pose)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return make_box_polygon(
      std::max(0.0, static_cast<double>(shape.dimensions.x)),
      std::max(0.0, static_cast<double>(shape.dimensions.y)), pose);
  }

  if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    std::vector<BgPoint> points;
    points.reserve(shape.footprint.points.size());
    for (const auto & point : shape.footprint.points) {
      points.push_back(
        transform_local_point(static_cast<double>(point.x), static_cast<double>(point.y), pose));
    }
    return polygon_from_points(points);
  }

  return std::nullopt;
}

std::vector<BgPoint> polygon_vertices(const BgPolygon & polygon)
{
  std::vector<BgPoint> points;
  const auto & ring = polygon.outer();
  if (ring.empty()) {
    return points;
  }
  const std::size_t count =
    ring.size() > 1 && bg::equals(ring.front(), ring.back()) ? ring.size() - 1 : ring.size();
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    points.push_back(ring[i]);
  }
  return points;
}

BgPolygon convex_hull_polygon(const BgPolygon & polygon)
{
  BgMultiPoint multi_point;
  for (const auto & point : polygon_vertices(polygon)) {
    multi_point.push_back(point);
  }
  BgPolygon hull;
  bg::convex_hull(multi_point, hull);
  bg::correct(hull);
  return hull;
}

std::pair<double, double> project_polygon_on_axis(
  const BgPolygon & polygon, const double axis_x, const double axis_y)
{
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const auto & point : polygon_vertices(polygon)) {
    const double projection = bg::get<0>(point) * axis_x + bg::get<1>(point) * axis_y;
    minimum = std::min(minimum, projection);
    maximum = std::max(maximum, projection);
  }
  return {minimum, maximum};
}

double sat_penetration_depth(const BgPolygon & first, const BgPolygon & second)
{
  double minimum_overlap = std::numeric_limits<double>::infinity();
  bool used_axis = false;

  const auto test_axes = [&](const BgPolygon & source) {
    const auto vertices = polygon_vertices(source);
    if (vertices.size() < 2) {
      return true;
    }
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      const auto & a = vertices[i];
      const auto & b = vertices[(i + 1) % vertices.size()];
      const double edge_x = bg::get<0>(b) - bg::get<0>(a);
      const double edge_y = bg::get<1>(b) - bg::get<1>(a);
      const double norm = std::hypot(edge_x, edge_y);
      if (norm <= kEpsilon) {
        continue;
      }
      const double axis_x = -edge_y / norm;
      const double axis_y = edge_x / norm;
      const auto first_projection = project_polygon_on_axis(first, axis_x, axis_y);
      const auto second_projection = project_polygon_on_axis(second, axis_x, axis_y);
      const double overlap = std::min(first_projection.second, second_projection.second) -
                             std::max(first_projection.first, second_projection.first);
      if (overlap < -kEpsilon) {
        return false;
      }
      used_axis = true;
      minimum_overlap = std::min(minimum_overlap, std::max(0.0, overlap));
    }
    return true;
  };

  if (!test_axes(first) || !test_axes(second) || !used_axis) {
    return 0.0;
  }
  return std::isfinite(minimum_overlap) ? minimum_overlap : 0.0;
}

double signed_polygon_clearance_m(const BgPolygon & first, const BgPolygon & second)
{
  const double separated_distance = bg::distance(first, second);
  if (separated_distance > kEpsilon) {
    return separated_distance;
  }

  const BgPolygon first_hull = convex_hull_polygon(first);
  const BgPolygon second_hull = convex_hull_polygon(second);
  const double penetration_depth = sat_penetration_depth(first_hull, second_hull);
  return penetration_depth > kEpsilon ? -penetration_depth : 0.0;
}

double distance_to_polygon_boundary_m(const BgPoint & point, const BgPolygon & polygon)
{
  const auto vertices = polygon_vertices(polygon);
  if (vertices.size() < 2) {
    return 0.0;
  }
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const BgSegment segment(vertices[i], vertices[(i + 1) % vertices.size()]);
    minimum = std::min(minimum, bg::distance(point, segment));
  }
  return std::isfinite(minimum) ? minimum : 0.0;
}

double signed_point_to_polygon_clearance_m(const BgPoint & point, const BgPolygon & polygon)
{
  if (bg::within(point, polygon)) {
    return -distance_to_polygon_boundary_m(point, polygon);
  }
  return bg::distance(point, polygon);
}

std::vector<Point> polygon_to_geometry_points(const BgPolygon & polygon)
{
  std::vector<Point> points;
  for (const auto & point : polygon_vertices(polygon)) {
    Point geometry_point;
    geometry_point.x = bg::get<0>(point);
    geometry_point.y = bg::get<1>(point);
    geometry_point.z = 0.0;
    points.push_back(geometry_point);
  }
  return points;
}

CandidateObstacleGeometryDebug make_obstacle_geometry_debug(
  const TrackedObject & object, const CandidateBankConfig & config)
{
  CandidateObstacleGeometryDebug debug;
  debug.is_static = tracked_speed_mps(object) <= config.static_velocity_threshold_mps;
  const Pose & pose = object.kinematics.pose_with_covariance.pose;

  if (config.collision_geometry_mode == "polygon") {
    if (const auto polygon = make_object_polygon(object.shape, pose); polygon.has_value()) {
      debug.polygon = polygon_to_geometry_points(polygon.value());
      return debug;
    }
    // CYLINDER and malformed polygon footprints remain circles in polygon mode.
    CandidateCollisionCircleDebug circle;
    circle.center = pose.position;
    circle.radius_m = legacy_object_radius_m(object.shape);
    debug.circles.push_back(circle);
    return debug;
  }

  const auto circles = object_collision_circles(object.shape, pose, config.collision_geometry_mode);
  debug.circles.reserve(circles.size);
  for (std::size_t i = 0; i < circles.size; ++i) {
    CandidateCollisionCircleDebug circle;
    circle.center = circles.circles[i].center;
    circle.radius_m = circles.circles[i].radius_m;
    debug.circles.push_back(circle);
  }
  return debug;
}

double min_clearance_to_object_m(
  const Pose & ego_pose, const autoware_perception_msgs::msg::Shape & shape,
  const Pose & object_pose, const CandidateBankConfig & config)
{
  if (config.collision_geometry_mode != "polygon") {
    const auto circle_set =
      object_collision_circles(shape, object_pose, config.collision_geometry_mode);
    double min_clearance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < circle_set.size; ++i) {
      const auto & circle = circle_set.circles[i];
      min_clearance = std::min(
        min_clearance,
        distance_xy(ego_pose.position, circle.center) - config.ego_radius_m - circle.radius_m);
    }
    return min_clearance;
  }

  const auto ego_polygon = make_ego_polygon(ego_pose, config);
  if (!ego_polygon.has_value()) {
    return std::numeric_limits<double>::infinity();
  }

  if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    const BgPoint center(object_pose.position.x, object_pose.position.y);
    const double radius = legacy_object_radius_m(shape);
    return signed_point_to_polygon_clearance_m(center, ego_polygon.value()) - radius;
  }

  if (
    const auto object_polygon = make_object_polygon(shape, object_pose);
    object_polygon.has_value()) {
    return signed_polygon_clearance_m(ego_polygon.value(), object_polygon.value());
  }

  // Unknown/degenerate shapes fall back to a circle while retaining the exact ego polygon.
  const BgPoint center(object_pose.position.x, object_pose.position.y);
  const double radius = legacy_object_radius_m(shape);
  return signed_point_to_polygon_clearance_m(center, ego_polygon.value()) - radius;
}
// CANDIDATE_BANK_COLLISION_GEOMETRY_V1_END

double tracked_speed_mps(const TrackedObject & object)
{
  const auto & velocity = object.kinematics.twist_with_covariance.twist.linear;
  return std::hypot(velocity.x, velocity.y);
}

double tracked_acceleration_mps2(const TrackedObject & object)
{
  const auto & acceleration = object.kinematics.acceleration_with_covariance.accel.linear;
  return std::hypot(acceleration.x, acceleration.y);
}

double yaw_from_pose(const Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Pose extrapolate_tracked_pose(const TrackedObject & object, const double time_sec)
{
  Pose pose = object.kinematics.pose_with_covariance.pose;
  const double yaw = yaw_from_pose(pose);
  const auto & velocity_local = object.kinematics.twist_with_covariance.twist.linear;
  const auto & acceleration_local = object.kinematics.acceleration_with_covariance.accel.linear;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double vx = cos_yaw * velocity_local.x - sin_yaw * velocity_local.y;
  const double vy = sin_yaw * velocity_local.x + cos_yaw * velocity_local.y;
  const double ax = cos_yaw * acceleration_local.x - sin_yaw * acceleration_local.y;
  const double ay = sin_yaw * acceleration_local.x + cos_yaw * acceleration_local.y;

  pose.position.x += vx * time_sec + 0.5 * ax * time_sec * time_sec;
  pose.position.y += vy * time_sec + 0.5 * ay * time_sec * time_sec;
  return pose;
}

bool same_object_id(const PredictedObject & predicted, const TrackedObject & tracked)
{
  return predicted.object_id.uuid == tracked.object_id.uuid;
}

const PredictedObject * find_predicted_object(
  const PredictedObjects & predicted_objects, const TrackedObject & tracked)
{
  const auto it = std::find_if(
    predicted_objects.objects.begin(), predicted_objects.objects.end(),
    [&](const PredictedObject & predicted) { return same_object_id(predicted, tracked); });
  return it == predicted_objects.objects.end() ? nullptr : &(*it);
}

std::optional<Pose> predicted_pose_at(const PredictedPath & path, const double time_sec)
{
  if (path.path.empty()) {
    return std::nullopt;
  }
  const double step = duration_sec(path.time_step);
  if (step <= kEpsilon) {
    return path.path.front();
  }
  const int64_t index_raw = std::lround(std::max(0.0, time_sec) / step);
  const std::size_t index = std::min<std::size_t>(
    static_cast<std::size_t>(std::max<int64_t>(0, index_raw)), path.path.size() - 1);
  return path.path[index];
}

struct ReferenceProjection
{
  double s{0.0};
  double distance{std::numeric_limits<double>::infinity()};
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
  std::size_t segment_index{0};
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
};

std::optional<ReferenceProjection> project_to_reference(
  const Trajectory & reference, const Point & query)
{
  if (reference.points.size() < 2) {
    return std::nullopt;
  }

  std::vector<double> cumulative(reference.points.size(), 0.0);
  for (std::size_t i = 1; i < reference.points.size(); ++i) {
    cumulative[i] =
      cumulative[i - 1] +
      distance_xy(reference.points[i - 1].pose.position, reference.points[i].pose.position);
  }

  ReferenceProjection best;
  for (std::size_t i = 1; i < reference.points.size(); ++i) {
    const auto & left = reference.points[i - 1].pose.position;
    const auto & right = reference.points[i].pose.position;
    const double vx = right.x - left.x;
    const double vy = right.y - left.y;
    const double length_squared = vx * vx + vy * vy;
    if (length_squared <= kEpsilon) {
      continue;
    }
    const double ratio =
      std::clamp(((query.x - left.x) * vx + (query.y - left.y) * vy) / length_squared, 0.0, 1.0);
    Point projected;
    projected.x = left.x + ratio * vx;
    projected.y = left.y + ratio * vy;
    const double distance = distance_xy(query, projected);
    if (distance < best.distance) {
      best.distance = distance;
      best.s = cumulative[i - 1] + ratio * std::sqrt(length_squared);
      // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
      best.segment_index = i;
      // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
    }
  }
  return std::isfinite(best.distance) ? std::make_optional(best) : std::nullopt;
}

// CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
struct RouteDeviationResult
{
  double rms_m{0.0};
  double mean_reference_weight{1.0};
  double max_reference_roughness_m{0.0};
};

std::vector<double> route_reference_roughness_m(
  const Trajectory & reference, const std::size_t window_points)
{
  std::vector<double> roughness(reference.points.size(), 0.0);
  if (reference.points.empty()) {
    return roughness;
  }

  const std::size_t half_window = std::max<std::size_t>(1, window_points / 2);
  for (std::size_t i = 0; i < reference.points.size(); ++i) {
    // Do not use an asymmetric window at either route-reference end: even a perfectly straight
    // line would otherwise appear rough because the average shifts tangentially.
    if (i < half_window || i + half_window >= reference.points.size()) {
      continue;
    }
    const std::size_t first = i - half_window;
    const std::size_t last = i + half_window;
    double mean_x = 0.0;
    double mean_y = 0.0;
    const std::size_t count = last - first + 1;
    for (std::size_t j = first; j <= last; ++j) {
      mean_x += reference.points[j].pose.position.x;
      mean_y += reference.points[j].pose.position.y;
    }
    mean_x /= static_cast<double>(count);
    mean_y /= static_cast<double>(count);

    // Use only the normal component of raw-vs-smoothed displacement. This avoids treating
    // nonuniform longitudinal point spacing on an otherwise straight lane as a discontinuity.
    const double tangent_x =
      reference.points[last].pose.position.x - reference.points[first].pose.position.x;
    const double tangent_y =
      reference.points[last].pose.position.y - reference.points[first].pose.position.y;
    const double tangent_norm = std::hypot(tangent_x, tangent_y);
    const double residual_x = reference.points[i].pose.position.x - mean_x;
    const double residual_y = reference.points[i].pose.position.y - mean_y;
    roughness[i] = tangent_norm > kEpsilon
                     ? std::abs(tangent_x * residual_y - tangent_y * residual_x) / tangent_norm
                     : std::hypot(residual_x, residual_y);
  }
  return roughness;
}

RouteDeviationResult route_deviation(
  const Trajectory & candidate, const std::size_t start_index, const Trajectory & reference,
  const CandidateBankConfig & config)
{
  RouteDeviationResult result;
  if (reference.points.size() < 2) {
    return result;
  }

  std::vector<double> reference_weights(reference.points.size(), 1.0);
  if (config.route_deviation_adaptive_weight_enabled) {
    const auto roughness =
      route_reference_roughness_m(reference, config.route_deviation_smoothing_window_points);
    for (std::size_t i = 0; i < roughness.size(); ++i) {
      result.max_reference_roughness_m = std::max(result.max_reference_roughness_m, roughness[i]);
      if (roughness[i] > config.route_deviation_roughness_threshold_m) {
        const double ratio =
          config.route_deviation_roughness_threshold_m / std::max(roughness[i], kEpsilon);
        reference_weights[i] = std::max(config.route_deviation_min_weight, ratio);
      }
    }
  }

  double weighted_sum_squared = 0.0;
  double sample_weight_sum = 0.0;
  std::size_t sample_count = 0;
  for (std::size_t i = start_index; i < candidate.points.size(); ++i) {
    const auto projection = project_to_reference(reference, candidate.points[i].pose.position);
    if (!projection.has_value()) {
      continue;
    }
    const std::size_t right = std::min(projection->segment_index, reference_weights.size() - 1);
    const std::size_t left = right > 0 ? right - 1 : right;
    const double local_weight = config.route_deviation_adaptive_weight_enabled
                                  ? std::min(reference_weights[left], reference_weights[right])
                                  : 1.0;
    weighted_sum_squared += local_weight * projection->distance * projection->distance;
    sample_weight_sum += local_weight;
    ++sample_count;
  }

  // Divide by the original sample count, not by the reduced weight sum. This intentionally
  // lowers the route-deviation contribution near a low-confidence reference discontinuity.
  result.rms_m =
    sample_count > 0 ? std::sqrt(weighted_sum_squared / static_cast<double>(sample_count)) : 0.0;
  result.mean_reference_weight =
    sample_count > 0 ? sample_weight_sum / static_cast<double>(sample_count) : 1.0;
  return result;
}
// CANDIDATE_BANK_ROUTE_RULE_COST_V1_END

double route_progress_m(
  const Trajectory & candidate, const std::size_t start_index,
  const std::optional<Trajectory> & reference)
{
  if (candidate.points.empty()) {
    return 0.0;
  }
  if (reference.has_value() && reference->points.size() >= 2) {
    const auto start =
      project_to_reference(reference.value(), candidate.points[start_index].pose.position);
    const auto end = project_to_reference(reference.value(), candidate.points.back().pose.position);
    if (start.has_value() && end.has_value()) {
      return std::max(0.0, end->s - start->s);
    }
  }
  return distance_xy(
    candidate.points[start_index].pose.position, candidate.points.back().pose.position);
}

// CANDIDATE_BANK_TRAJECTORY_SANITIZATION_V1_BEGIN
// Temporary forward-only sanitizer. Candidate geometry is position-primary downstream, so when
// route progress would decrease, hold the pose at the previous corrected point. time_from_start
// and kinematic fields are intentionally preserved; the downstream optimizer derives kinematics
// again from position geometry.
std::size_t clip_backward_points_in_place(
  Trajectory & candidate, const std::optional<Trajectory> & route_reference)
{
  if (
    candidate.points.size() < 2 || !route_reference.has_value() ||
    route_reference->points.size() < 2) {
    return 0;
  }

  constexpr double kNumericalToleranceM = 1.0e-6;
  std::size_t clipped_count = 0;
  for (std::size_t i = 1; i < candidate.points.size(); ++i) {
    const auto previous_projection =
      project_to_reference(route_reference.value(), candidate.points[i - 1].pose.position);
    const auto current_projection =
      project_to_reference(route_reference.value(), candidate.points[i].pose.position);
    if (!previous_projection.has_value() || !current_projection.has_value()) {
      continue;
    }
    if (current_projection->s < previous_projection->s - kNumericalToleranceM) {
      candidate.points[i].pose = candidate.points[i - 1].pose;
      ++clipped_count;
    }
  }
  return clipped_count;
}
// CANDIDATE_BANK_TRAJECTORY_SANITIZATION_V1_END

// CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
struct TrafficRuleEvaluation
{
  bool signal_stop_active{false};
  bool signal_crossed{false};
  double max_stop_zone_speed_mps{0.0};
  double cost{0.0};
};

TrafficRuleEvaluation evaluate_traffic_rules(
  const Trajectory & candidate, const std::size_t start_index,
  const std::optional<Trajectory> & route_reference, const CandidateTrafficRuleContext & context,
  const CandidateBankConfig & config)
{
  TrafficRuleEvaluation result;
  if (
    !config.traffic_signal_rule_enabled || !context.traffic_signal_stop_active ||
    !route_reference.has_value() || route_reference->points.size() < 2 ||
    candidate.points.empty() || start_index >= candidate.points.size()) {
    return result;
  }

  result.signal_stop_active = true;
  constexpr double kCrossingToleranceM = 0.05;
  double max_speed_excess_mps = 0.0;
  // CANDIDATE_BANK_SIGNAL_OVERRUN_SPEED_HOLD_V1_BEGIN
  double max_stop_line_overrun_m = 0.0;
  // CANDIDATE_BANK_SIGNAL_OVERRUN_SPEED_HOLD_V1_END
  const double effective_crossing_s_m =
    context.traffic_signal_stop_line_s_m - config.traffic_signal_crossing_margin_m;
  for (std::size_t i = start_index; i < candidate.points.size(); ++i) {
    const auto projection =
      project_to_reference(route_reference.value(), candidate.points[i].pose.position);
    if (!projection.has_value()) {
      continue;
    }

    // CANDIDATE_BANK_SIGNAL_OVERRUN_SPEED_HOLD_V1_BEGIN
    const double stop_line_overrun_m =
      std::max(0.0, projection->s - context.traffic_signal_stop_line_s_m);
    max_stop_line_overrun_m = std::max(max_stop_line_overrun_m, stop_line_overrun_m);
    // CANDIDATE_BANK_SIGNAL_OVERRUN_SPEED_HOLD_V1_END

    if (projection->s > effective_crossing_s_m + kCrossingToleranceM) {
      result.signal_crossed = true;
    }

    const bool in_stop_zone =
      projection->s >=
        context.traffic_signal_target_stop_s_m - config.traffic_signal_approach_distance_m &&
      projection->s <=
        context.traffic_signal_stop_line_s_m +
          std::max(kCrossingToleranceM, config.traffic_signal_post_stop_hold_distance_m);
    if (in_stop_zone) {
      const double speed_mps =
        std::abs(static_cast<double>(candidate.points[i].longitudinal_velocity_mps));
      result.max_stop_zone_speed_mps = std::max(result.max_stop_zone_speed_mps, speed_mps);
      max_speed_excess_mps = std::max(
        max_speed_excess_mps, speed_mps - config.traffic_signal_stop_velocity_threshold_mps);
    }
  }

  if (result.signal_crossed) {
    result.cost += config.traffic_signal_crossing_penalty;
  }
  // CANDIDATE_BANK_SIGNAL_OVERRUN_SPEED_HOLD_V1_BEGIN
  result.cost +=
    config.traffic_signal_overrun_weight * max_stop_line_overrun_m * max_stop_line_overrun_m;
  // CANDIDATE_BANK_SIGNAL_OVERRUN_SPEED_HOLD_V1_END
  result.cost +=
    config.traffic_signal_stop_speed_weight * max_speed_excess_mps * max_speed_excess_mps;
  return result;
}

// CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
struct LaneBoundaryEvaluation
{
  double max_continuous_straddling_sec{0.0};
  bool terminal_straddling{false};
  double prolonged_straddling_cost{0.0};
  double terminal_straddling_cost{0.0};
  double total_cost{0.0};
};

bool ego_footprint_intersects_shared_boundary(
  const Pose & pose, const CandidateTrafficRuleContext & context,
  const CandidateBankConfig & config)
{
  const auto ego_polygon = make_ego_polygon(pose, config);
  if (!ego_polygon.has_value()) {
    return false;
  }
  for (const auto & boundary : context.shared_lane_boundaries_map) {
    for (std::size_t i = 1; i < boundary.points_map.size(); ++i) {
      const auto & a = boundary.points_map[i - 1];
      const auto & b = boundary.points_map[i];
      const BgSegment segment(BgPoint(a.x, a.y), BgPoint(b.x, b.y));
      if (bg::intersects(ego_polygon.value(), segment)) {
        return true;
      }
    }
  }
  return false;
}

LaneBoundaryEvaluation evaluate_lane_boundary(
  const Trajectory & candidate, const std::size_t start_index,
  const CandidateTrafficRuleContext & context, const CandidateBankConfig & config)
{
  LaneBoundaryEvaluation result;
  if (
    !config.lane_boundary_enabled || !context.current_lane_valid ||
    context.shared_lane_boundaries_map.empty() || candidate.points.empty() ||
    start_index >= candidate.points.size()) {
    return result;
  }

  std::optional<double> continuous_start_sec;
  for (std::size_t i = start_index; i < candidate.points.size(); ++i) {
    const double time_sec = live_time_sec(candidate, start_index, i);
    const bool straddling =
      ego_footprint_intersects_shared_boundary(candidate.points[i].pose, context, config);
    if (straddling) {
      if (!continuous_start_sec.has_value()) {
        continuous_start_sec = time_sec;
      }
      result.max_continuous_straddling_sec =
        std::max(result.max_continuous_straddling_sec, time_sec - continuous_start_sec.value());
    } else {
      continuous_start_sec.reset();
    }
  }

  result.terminal_straddling =
    ego_footprint_intersects_shared_boundary(candidate.points.back().pose, context, config);
  const double excess_sec = std::max(
    0.0,
    result.max_continuous_straddling_sec - config.lane_boundary_prolonged_straddling_threshold_sec);
  result.prolonged_straddling_cost =
    config.lane_boundary_prolonged_straddling_weight * excess_sec * excess_sec;
  result.terminal_straddling_cost =
    result.terminal_straddling ? config.lane_boundary_terminal_straddling_penalty : 0.0;
  result.total_cost = result.prolonged_straddling_cost + result.terminal_straddling_cost;
  return result;
}
// CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END

// CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
struct LaneChangeDeadlineEvaluation
{
  bool active{false};
  bool attempted{false};
  bool completed{false};
  bool completed_before_deadline{false};
  bool fallback_stop_satisfied{false};
  std::int64_t boundary_id{0};
  std::int64_t target_lane_id{0};
  double deadline_s_m{std::numeric_limits<double>::infinity()};
  double completion_s_m{std::numeric_limits<double>::infinity()};
  double slack_m{-std::numeric_limits<double>::infinity()};
  double cost{0.0};
};

const CandidateLaneBoundaryGeometry * intersecting_shared_boundary(
  const Pose & pose, const CandidateTrafficRuleContext & context,
  const CandidateBankConfig & config)
{
  const auto ego_polygon = make_ego_polygon(pose, config);
  if (!ego_polygon.has_value()) return nullptr;
  for (const auto & boundary : context.shared_lane_boundaries_map) {
    for (std::size_t i = 1; i < boundary.points_map.size(); ++i) {
      const auto & a = boundary.points_map[i - 1];
      const auto & b = boundary.points_map[i];
      const BgSegment segment(BgPoint(a.x, a.y), BgPoint(b.x, b.y));
      if (bg::intersects(ego_polygon.value(), segment)) return &boundary;
    }
  }
  return nullptr;
}

double signed_side_to_boundary(
  const geometry_msgs::msg::Point & point, const CandidateLaneBoundaryGeometry & boundary)
{
  double best_d2 = std::numeric_limits<double>::infinity();
  double best_signed = 0.0;
  for (std::size_t i = 1; i < boundary.points_map.size(); ++i) {
    const auto & a = boundary.points_map[i - 1];
    const auto & b = boundary.points_map[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= 1.0e-12) continue;
    const double u = std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / len2, 0.0, 1.0);
    const double qx = a.x + u * dx;
    const double qy = a.y + u * dy;
    const double ex = point.x - qx;
    const double ey = point.y - qy;
    const double d2 = ex * ex + ey * ey;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_signed = (dx * (point.y - a.y) - dy * (point.x - a.x)) / std::sqrt(len2);
    }
  }
  return best_signed;
}

bool candidate_stops_for_rule(
  const Trajectory & candidate, const std::size_t start_index,
  const std::optional<Trajectory> & route_reference, const double stop_line_s_m,
  const double target_stop_s_m, const CandidateBankConfig & config)
{
  if (
    !route_reference.has_value() || route_reference->points.size() < 2 ||
    candidate.points.empty() || start_index >= candidate.points.size()) {
    return false;
  }
  constexpr double kCrossingToleranceM = 0.05;
  for (std::size_t i = start_index; i < candidate.points.size(); ++i) {
    const auto projection =
      project_to_reference(route_reference.value(), candidate.points[i].pose.position);
    if (projection.has_value() && projection->s > stop_line_s_m + kCrossingToleranceM) {
      return false;
    }
  }
  const auto terminal_projection =
    project_to_reference(route_reference.value(), candidate.points.back().pose.position);
  if (!terminal_projection.has_value()) return false;
  const double terminal_speed =
    std::abs(static_cast<double>(candidate.points.back().longitudinal_velocity_mps));
  const double before_target_m = target_stop_s_m - terminal_projection->s;
  return terminal_speed <= config.traffic_signal_stop_velocity_threshold_mps + 1.0e-6 &&
         before_target_m >= -kCrossingToleranceM &&
         before_target_m <= config.lane_change_fallback_stop_max_distance_m;
}

LaneChangeDeadlineEvaluation evaluate_lane_change_deadline(
  const Trajectory & candidate, const std::size_t start_index, const Pose & ego_pose,
  const std::optional<Trajectory> & route_reference, const CandidateTrafficRuleContext & context,
  const CandidateBankConfig & config)
{
  LaneChangeDeadlineEvaluation result;
  if (
    !config.lane_change_deadline_enabled || !context.traffic_signal_present ||
    !context.current_lane_valid || context.shared_lane_boundaries_map.empty() ||
    !route_reference.has_value() || candidate.points.empty() ||
    start_index >= candidate.points.size()) {
    return result;
  }

  const CandidateLaneBoundaryGeometry * boundary = nullptr;
  std::size_t last_straddling_index = start_index;
  bool candidate_straddling = false;
  for (std::size_t i = start_index; i < candidate.points.size(); ++i) {
    const auto * hit = intersecting_shared_boundary(candidate.points[i].pose, context, config);
    if (hit != nullptr) {
      if (boundary == nullptr) boundary = hit;
      if (boundary->boundary_id == hit->boundary_id) {
        candidate_straddling = true;
        last_straddling_index = i;
      }
    }
  }
  const auto * ego_boundary = intersecting_shared_boundary(ego_pose, context, config);
  const bool ego_straddling = ego_boundary != nullptr;
  if (boundary == nullptr && ego_boundary != nullptr) boundary = ego_boundary;

  const std::string & mode = config.lane_change_deadline_activation_mode;
  result.active = mode == "always" || (mode == "ego_straddling" && ego_straddling) ||
                  (mode == "candidate_straddling" && candidate_straddling);
  if (!result.active || boundary == nullptr) return result;

  result.attempted = candidate_straddling || ego_straddling;
  result.boundary_id = boundary->boundary_id;
  result.target_lane_id = boundary->adjacent_lane_id;
  result.deadline_s_m =
    context.traffic_signal_stop_line_s_m - config.lane_change_completion_margin_m;

  // A completed crossing must end clear of the boundary and on the opposite side from the
  // current ego reference point.  If ego center is numerically on the line, falling clear after
  // a straddling interval is accepted as the lightweight completion definition.
  if (
    result.attempted &&
    !ego_footprint_intersects_shared_boundary(candidate.points.back().pose, context, config)) {
    const double source_side = signed_side_to_boundary(ego_pose.position, *boundary);
    const double terminal_side =
      signed_side_to_boundary(candidate.points.back().pose.position, *boundary);
    const bool opposite_side = source_side * terminal_side < -0.01;
    const bool source_side_ambiguous = std::abs(source_side) <= 0.05;
    result.completed = opposite_side || source_side_ambiguous;
  }

  if (result.completed) {
    const std::size_t completion_index =
      std::min(last_straddling_index + 1, candidate.points.size() - 1);
    const auto projection = project_to_reference(
      route_reference.value(), candidate.points[completion_index].pose.position);
    if (projection.has_value()) {
      result.completion_s_m = projection->s;
      result.slack_m = result.deadline_s_m - result.completion_s_m;
      result.completed_before_deadline = result.slack_m >= 0.0;
      if (result.completed_before_deadline) {
        const double slack_shortfall =
          std::max(0.0, config.lane_change_comfortable_slack_m - result.slack_m);
        result.cost = config.lane_change_approaching_weight * slack_shortfall * slack_shortfall;
        return result;
      }
      const double overdue_m = -result.slack_m;
      result.cost = config.lane_change_missed_deadline_penalty +
                    config.lane_change_approaching_weight * overdue_m * overdue_m;
    }
  }

  if (!result.completed_before_deadline && config.lane_change_fallback_stop_enabled) {
    bool have_stop_rule = false;
    double fallback_stop_line_s = std::numeric_limits<double>::infinity();
    double fallback_target_stop_s = std::numeric_limits<double>::infinity();
    auto include_rule = [&](
                          const bool active, const double stop_line_s, const double target_stop_s) {
      if (!active) return;
      if (!have_stop_rule || target_stop_s < fallback_target_stop_s) {
        fallback_stop_line_s = stop_line_s;
        fallback_target_stop_s = target_stop_s;
      }
      have_stop_rule = true;
    };

    if (config.lane_change_fallback_combine_with_route_stop_condition) {
      include_rule(
        context.traffic_signal_stop_active, context.traffic_signal_stop_line_s_m,
        context.traffic_signal_target_stop_s_m);
    }
    if (config.lane_change_fallback_use_target_lane_stop_condition) {
      include_rule(
        boundary->adjacent_signal_stop_active, boundary->adjacent_signal_stop_line_s_m,
        boundary->adjacent_signal_target_stop_s_m);
    }

    if (have_stop_rule) {
      result.fallback_stop_satisfied = candidate_stops_for_rule(
        candidate, start_index, route_reference, fallback_stop_line_s, fallback_target_stop_s,
        config);
      if (result.fallback_stop_satisfied) {
        result.cost = config.lane_change_fallback_stop_penalty;
        return result;
      }
    }
  }

  if (!result.completed_before_deadline) {
    result.cost = std::max(result.cost, config.lane_change_missed_deadline_penalty);
  }
  return result;
}
// CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
// CANDIDATE_BANK_ROUTE_RULE_COST_V1_END

double comfort_cost(const Trajectory & candidate, const std::size_t start_index)
{
  if (candidate.points.empty() || start_index >= candidate.points.size()) {
    return 0.0;
  }
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t i = start_index; i < candidate.points.size(); ++i) {
    sum += std::abs(static_cast<double>(candidate.points[i].acceleration_mps2));
    sum += 0.25 * std::abs(static_cast<double>(candidate.points[i].heading_rate_rps));
    ++count;
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

// CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_BEGIN
struct StaticClearanceWitnessDebug
{
  bool valid{false};
  std::string object_id;
  double object_speed_mps{0.0};
  std::size_t trajectory_point_index{0};
  std::size_t live_point_index{0};
  double future_time_sec{0.0};
  double signed_clearance_m{std::numeric_limits<double>::infinity()};
  double margin_m{0.0};
  double deficit_m{0.0};
  Pose ego_pose;
  Pose object_pose;
  std::vector<Point> ego_polygon;
  CandidateObstacleGeometryDebug object_geometry;
};
// CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_END

struct SafetyResult
{
  bool safe{true};
  // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
  bool static_collision_margin_violated{false};
  bool dynamic_collision_margin_violated{false};
  // CANDIDATE_BANK_SOFT_COLLISION_V1_END
  double min_static_clearance_m{std::numeric_limits<double>::infinity()};
  double min_dynamic_clearance_m{std::numeric_limits<double>::infinity()};
  double min_dynamic_time_gap_sec{std::numeric_limits<double>::infinity()};
  // CANDIDATE_BANK_TTC_COST_V1_BEGIN
  double min_dynamic_ttc_sec{std::numeric_limits<double>::infinity()};
  double dynamic_clearance_soft_cost{0.0};
  double dynamic_legacy_time_gap_soft_cost{0.0};
  double dynamic_ttc_soft_cost{0.0};
  // CANDIDATE_BANK_TTC_COST_V1_END
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
  double max_dynamic_closing_rate_mps{0.0};
  double max_dynamic_uncertainty_margin_m{0.0};
  std::size_t dynamic_prediction_path_count{0};
  double dynamic_prediction_confidence_sum{0.0};
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
  // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
  CandidateDynamicEvaluationDebug dynamic_analysis_debug;
  // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
  // CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_BEGIN
  StaticClearanceWitnessDebug static_argmin;
  StaticClearanceWitnessDebug static_first_margin_violation;
  StaticClearanceWitnessDebug static_first_overlap;
  // CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_END
  double static_soft_cost{0.0};
  double dynamic_soft_cost{0.0};
};

// CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_BEGIN
void capture_static_clearance_witness(
  StaticClearanceWitnessDebug & witness, const TrajectoryCandidate & candidate,
  const TrackedObject & object, const CandidateBankConfig & config, const std::size_t point_index,
  const double clearance)
{
  witness.valid = true;
  witness.object_id = tracked_object_id_string(object);
  witness.object_speed_mps = tracked_speed_mps(object);
  witness.trajectory_point_index = point_index;
  witness.live_point_index =
    point_index >= candidate.live_start_index ? point_index - candidate.live_start_index : 0U;
  witness.future_time_sec =
    live_time_sec(candidate.trajectory_map, candidate.live_start_index, point_index);
  witness.signed_clearance_m = clearance;
  witness.margin_m = config.static_collision_margin_m;
  witness.deficit_m = std::max(0.0, config.static_collision_margin_m - clearance);
  witness.ego_pose = candidate.trajectory_map.points[point_index].pose;
  witness.object_pose = object.kinematics.pose_with_covariance.pose;
  witness.ego_polygon.clear();
  if (const auto polygon = make_ego_polygon(witness.ego_pose, config); polygon.has_value()) {
    witness.ego_polygon = polygon_to_geometry_points(polygon.value());
  }
  witness.object_geometry = make_obstacle_geometry_debug(object, config);
}
// CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_END

void evaluate_static_object(
  const TrajectoryCandidate & candidate, const TrackedObject & object,
  const CandidateBankConfig & config, SafetyResult & result)
{
  if (!config.static_safety_enabled) {
    return;
  }
  const Pose & object_pose = object.kinematics.pose_with_covariance.pose;
  const std::size_t stride = std::max<std::size_t>(1, config.evaluation_stride);

  for (std::size_t i = candidate.live_start_index; i < candidate.trajectory_map.points.size();
       i += stride) {
    const double clearance = min_clearance_to_object_m(
      candidate.trajectory_map.points[i].pose, object.shape, object_pose, config);
    // CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_BEGIN
    const double future_time_sec =
      live_time_sec(candidate.trajectory_map, candidate.live_start_index, i);
    if (!result.static_argmin.valid || clearance < result.static_argmin.signed_clearance_m) {
      capture_static_clearance_witness(
        result.static_argmin, candidate, object, config, i, clearance);
    }
    if (
      clearance <= config.static_collision_margin_m &&
      (!result.static_first_margin_violation.valid ||
       future_time_sec < result.static_first_margin_violation.future_time_sec - kEpsilon ||
       (std::abs(future_time_sec - result.static_first_margin_violation.future_time_sec) <=
          kEpsilon &&
        clearance < result.static_first_margin_violation.signed_clearance_m))) {
      capture_static_clearance_witness(
        result.static_first_margin_violation, candidate, object, config, i, clearance);
    }
    if (
      clearance <= 0.0 &&
      (!result.static_first_overlap.valid ||
       future_time_sec < result.static_first_overlap.future_time_sec - kEpsilon ||
       (std::abs(future_time_sec - result.static_first_overlap.future_time_sec) <= kEpsilon &&
        clearance < result.static_first_overlap.signed_clearance_m))) {
      capture_static_clearance_witness(
        result.static_first_overlap, candidate, object, config, i, clearance);
    }
    // CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_END
    result.min_static_clearance_m = std::min(result.min_static_clearance_m, clearance);
    if (clearance <= config.static_collision_margin_m) {
      // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
      result.static_collision_margin_violated = true;
      if (config.static_hard_reject_enabled) {
        result.safe = false;
      }
      // CANDIDATE_BANK_SOFT_COLLISION_V1_END
    }
    const double deficit = std::max(0.0, config.static_preferred_clearance_m - clearance);
    result.static_soft_cost += deficit * deficit;
  }
}

// CANDIDATE_BANK_TTC_COST_V1_BEGIN
// CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
struct DynamicPathEvaluation
{
  bool collision_margin_violated{false};
  double min_clearance_m{std::numeric_limits<double>::infinity()};
  double max_clearance_soft_cost{0.0};
  double sum_clearance_soft_cost{0.0};
  double max_closing_rate_mps{0.0};
  double max_uncertainty_margin_m{0.0};
  std::optional<double> ttc_sec;
  // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
  std::size_t worst_sample_index{0};
  std::vector<CandidateDynamicSampleDebug> samples;
  // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
};

// Evaluate one object prediction path against one ego candidate at matching future times.
//
// 0013a2 changes two mechanics that caused large dynamic costs for rear/parallel traffic:
// 1) the preferred-clearance penalty is summarized once per PredictedPath (worst point) instead
//    of being multiplied by the number of trajectory samples;
// 2) the default uncertainty envelope is based on geometric-clearance closing rate and the
//    configured timing uncertainty.  The earlier absolute-speed / t^2 horizon formula remains
//    available through dynamic_uncertainty_mode=legacy_absolute_speed.
//
// 0013a3 additionally stores the exact per-sample quantities produced by this evaluator.  They
// are diagnostic-only and are never read by the score/selection path.
DynamicPathEvaluation evaluate_dynamic_same_time_path(
  const TrajectoryCandidate & candidate, const TrackedObject & tracked,
  const PredictedPath * predicted_path, const CandidateBankConfig & config)
{
  DynamicPathEvaluation path_result;
  const double speed = tracked_speed_mps(tracked);
  const double acceleration = tracked_acceleration_mps2(tracked);
  const std::size_t stride = std::max<std::size_t>(1, config.evaluation_stride);
  std::optional<double> previous_time_sec;
  std::optional<double> previous_geometric_clearance_m;
  const bool collect_debug_samples = internal_debug::config().enabled;
  bool has_worst_sample = false;
  double worst_evaluated_clearance_m = std::numeric_limits<double>::infinity();

  const std::size_t point_count = candidate.trajectory_map.points.size();
  if (collect_debug_samples && candidate.live_start_index < point_count) {
    path_result.samples.reserve((point_count - candidate.live_start_index + stride - 1) / stride);
  }

  for (std::size_t i = candidate.live_start_index; i < candidate.trajectory_map.points.size();
       i += stride) {
    const double time_sec = live_time_sec(candidate.trajectory_map, candidate.live_start_index, i);
    Pose object_pose;
    if (predicted_path != nullptr) {
      const auto pose = predicted_pose_at(*predicted_path, time_sec);
      if (!pose.has_value()) {
        continue;
      }
      object_pose = pose.value();
    } else {
      object_pose = extrapolate_tracked_pose(tracked, time_sec);
    }

    const Pose & ego_pose = candidate.trajectory_map.points[i].pose;
    const double geometric_clearance =
      min_clearance_to_object_m(ego_pose, tracked.shape, object_pose, config);

    double closing_rate_mps = 0.0;
    if (
      previous_time_sec.has_value() && previous_geometric_clearance_m.has_value() &&
      time_sec > previous_time_sec.value() + kEpsilon) {
      closing_rate_mps = std::max(
        0.0, (previous_geometric_clearance_m.value() - geometric_clearance) /
               (time_sec - previous_time_sec.value()));
    }
    path_result.max_closing_rate_mps = std::max(path_result.max_closing_rate_mps, closing_rate_mps);

    double uncertainty_margin = 0.0;
    if (config.dynamic_uncertainty_mode == "legacy_absolute_speed") {
      uncertainty_margin =
        config.velocity_uncertainty_gain * speed * config.dynamic_time_uncertainty_sec +
        config.acceleration_uncertainty_gain * 0.5 * acceleration * time_sec * time_sec;
    } else {
      const double dt_uncertainty = config.dynamic_time_uncertainty_sec;
      uncertainty_margin =
        config.velocity_uncertainty_gain * closing_rate_mps * dt_uncertainty +
        config.acceleration_uncertainty_gain * 0.5 * acceleration * dt_uncertainty * dt_uncertainty;
    }
    if (config.dynamic_max_uncertainty_margin_m > 0.0) {
      uncertainty_margin = std::min(uncertainty_margin, config.dynamic_max_uncertainty_margin_m);
    }
    path_result.max_uncertainty_margin_m =
      std::max(path_result.max_uncertainty_margin_m, uncertainty_margin);

    const double clearance = geometric_clearance - uncertainty_margin;

    if (!path_result.ttc_sec.has_value() && geometric_clearance <= 0.0) {
      double ttc_sec = time_sec;
      if (
        previous_time_sec.has_value() && previous_geometric_clearance_m.has_value() &&
        previous_geometric_clearance_m.value() > 0.0 && time_sec > previous_time_sec.value() &&
        previous_geometric_clearance_m.value() > geometric_clearance + kEpsilon) {
        const double alpha = std::clamp(
          previous_geometric_clearance_m.value() /
            (previous_geometric_clearance_m.value() - geometric_clearance),
          0.0, 1.0);
        ttc_sec = previous_time_sec.value() + alpha * (time_sec - previous_time_sec.value());
      }
      path_result.ttc_sec = std::max(0.0, ttc_sec);
    }

    path_result.min_clearance_m = std::min(path_result.min_clearance_m, clearance);
    const bool collision_margin_violated = clearance <= config.dynamic_collision_margin_m;
    if (collision_margin_violated) {
      path_result.collision_margin_violated = true;
    }

    const double deficit = std::max(0.0, config.dynamic_preferred_clearance_m - clearance);
    const double clearance_cost = deficit * deficit;
    path_result.sum_clearance_soft_cost += clearance_cost;

    const bool better_worst_sample =
      !has_worst_sample || clearance_cost > path_result.max_clearance_soft_cost + kEpsilon ||
      (std::abs(clearance_cost - path_result.max_clearance_soft_cost) <= kEpsilon &&
       clearance < worst_evaluated_clearance_m);
    if (better_worst_sample) {
      path_result.max_clearance_soft_cost = clearance_cost;
      worst_evaluated_clearance_m = clearance;
      has_worst_sample = true;
      if (collect_debug_samples) {
        path_result.worst_sample_index = path_result.samples.size();
      }
    }

    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    if (collect_debug_samples) {
      CandidateDynamicSampleDebug sample;
      sample.time_sec = time_sec;
      sample.ego_x = ego_pose.position.x;
      sample.ego_y = ego_pose.position.y;
      sample.ego_yaw = yaw_from_pose(ego_pose);
      sample.object_x = object_pose.position.x;
      sample.object_y = object_pose.position.y;
      sample.object_yaw = yaw_from_pose(object_pose);
      sample.geometric_clearance_m = geometric_clearance;
      sample.closing_rate_mps = closing_rate_mps;
      sample.uncertainty_margin_m = uncertainty_margin;
      sample.evaluated_clearance_m = clearance;
      sample.clearance_soft_cost = clearance_cost;
      sample.collision_margin_violated = collision_margin_violated;
      path_result.samples.push_back(sample);
    }
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

    previous_time_sec = time_sec;
    previous_geometric_clearance_m = geometric_clearance;
  }

  return path_result;
}
// CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
// CANDIDATE_BANK_TTC_COST_V1_END

void accumulate_predicted_path_time_gap(
  const TrajectoryCandidate & candidate, const TrackedObject & tracked,
  const PredictedPath & predicted_path, const CandidateBankConfig & config,
  const bool apply_legacy_cost, SafetyResult & result)
{
  if (predicted_path.path.empty()) {
    return;
  }
  const double step = duration_sec(predicted_path.time_step);
  if (step <= kEpsilon) {
    return;
  }
  const std::size_t stride = std::max<std::size_t>(1, config.evaluation_stride);

  for (std::size_t i = candidate.live_start_index; i < candidate.trajectory_map.points.size();
       i += stride) {
    const double ego_time = live_time_sec(candidate.trajectory_map, candidate.live_start_index, i);
    for (std::size_t j = 0; j < predicted_path.path.size(); j += stride) {
      const double clearance = min_clearance_to_object_m(
        candidate.trajectory_map.points[i].pose, tracked.shape, predicted_path.path[j], config);
      if (clearance > config.dynamic_conflict_distance_m) {
        continue;
      }
      const double object_time = static_cast<double>(j) * step;
      const double time_gap = std::abs(ego_time - object_time);
      result.min_dynamic_time_gap_sec = std::min(result.min_dynamic_time_gap_sec, time_gap);
      if (apply_legacy_cost) {
        const double deficit = std::max(0.0, config.dynamic_preferred_time_gap_sec - time_gap);
        const double cost = predicted_path.confidence * deficit * deficit;
        result.dynamic_legacy_time_gap_soft_cost += cost;
        result.dynamic_soft_cost += cost;
      }
    }
  }
}

void accumulate_fallback_time_gap(
  const TrajectoryCandidate & candidate, const TrackedObject & tracked,
  const CandidateBankConfig & config, const bool apply_legacy_cost, SafetyResult & result)
{
  const std::size_t stride = std::max<std::size_t>(1, config.evaluation_stride);
  for (std::size_t i = candidate.live_start_index; i < candidate.trajectory_map.points.size();
       i += stride) {
    const double ego_time = live_time_sec(candidate.trajectory_map, candidate.live_start_index, i);
    for (std::size_t j = candidate.live_start_index; j < candidate.trajectory_map.points.size();
         j += stride) {
      const double object_time =
        live_time_sec(candidate.trajectory_map, candidate.live_start_index, j);
      const Pose object_pose = extrapolate_tracked_pose(tracked, object_time);
      const double clearance = min_clearance_to_object_m(
        candidate.trajectory_map.points[i].pose, tracked.shape, object_pose, config);
      if (clearance > config.dynamic_conflict_distance_m) {
        continue;
      }
      const double time_gap = std::abs(ego_time - object_time);
      result.min_dynamic_time_gap_sec = std::min(result.min_dynamic_time_gap_sec, time_gap);
      if (apply_legacy_cost) {
        const double deficit = std::max(0.0, config.dynamic_preferred_time_gap_sec - time_gap);
        const double cost = deficit * deficit;
        result.dynamic_legacy_time_gap_soft_cost += cost;
        result.dynamic_soft_cost += cost;
      }
    }
  }
}

// CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
// CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
std::uint64_t map_context_debug_signature(
  const CandidateMapConflictContext & context, const CandidateBankConfig & config)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](const std::uint64_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  mix(context.valid ? 1ULL : 0ULL);
  mix(
    static_cast<std::uint64_t>(
      std::llround(std::max(0.0, config.map_conflict_forward_search_distance_m) * 1000.0)));
  mix(
    static_cast<std::uint64_t>(
      std::llround(std::max(0.0, config.map_conflict_backtrace_distance_m) * 1000.0)));
  mix(static_cast<std::uint64_t>(context.lanes.size()));
  for (const auto & lane : context.lanes) {
    mix(static_cast<std::uint64_t>(lane.lane_id));
    mix(lane.is_route_lane ? 1ULL : 0ULL);
  }
  mix(static_cast<std::uint64_t>(context.conflicts.size()));
  for (const auto & conflict : context.conflicts) {
    mix(static_cast<std::uint64_t>(conflict.ego_route_lane_id));
    mix(static_cast<std::uint64_t>(conflict.competing_lane_id));
    for (const auto & approach : conflict.approaches) {
      mix(static_cast<std::uint64_t>(approach.lane_id));
    }
  }
  return hash;
}
// CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END

struct MapLaneProjection
{
  double s_m{0.0};
  double lateral_distance_m{std::numeric_limits<double>::infinity()};
  double tangent_yaw{0.0};
};

std::optional<MapLaneProjection> project_to_map_lane(
  const CandidateMapLaneGeometry & lane, const Point & query)
{
  if (lane.centerline_map.size() < 2) {
    return std::nullopt;
  }
  MapLaneProjection best;
  double prefix = 0.0;
  for (std::size_t i = 1; i < lane.centerline_map.size(); ++i) {
    const auto & a = lane.centerline_map[i - 1];
    const auto & b = lane.centerline_map[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= kEpsilon) {
      continue;
    }
    const double len = std::sqrt(len2);
    const double u = std::clamp(((query.x - a.x) * dx + (query.y - a.y) * dy) / len2, 0.0, 1.0);
    Point projected;
    projected.x = a.x + u * dx;
    projected.y = a.y + u * dy;
    const double distance = distance_xy(projected, query);
    if (distance < best.lateral_distance_m) {
      best.lateral_distance_m = distance;
      best.s_m = prefix + u * len;
      best.tangent_yaw = std::atan2(dy, dx);
    }
    prefix += len;
  }
  return std::isfinite(best.lateral_distance_m) ? std::make_optional(best) : std::nullopt;
}

double angle_error_rad(const double lhs, const double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

struct ObjectLaneHypothesis
{
  std::int64_t lane_id{0};
  bool is_route_lane{false};
  double s_m{0.0};
  double lateral_distance_m{std::numeric_limits<double>::infinity()};
  double heading_error_rad{std::numeric_limits<double>::infinity()};
  double predicted_path_fit_m{std::numeric_limits<double>::infinity()};
  double confidence{0.0};
};

struct MotionHeadingEstimate
{
  double yaw{0.0};
  double reliability{0.0};
  std::string source{"none"};
};

MotionHeadingEstimate object_motion_heading(
  const TrackedObject & tracked, const std::optional<double> history_heading,
  const std::optional<double> predicted_heading, const CandidateBankConfig & config)
{
  // Prefer directions already expressed in the map frame.  This deliberately reduces dependence
  // on object pose yaw, which can be noisy or sign-ambiguous around intersections.
  if (predicted_heading.has_value()) {
    return {predicted_heading.value(), 0.95, "predicted_path"};
  }
  if (history_heading.has_value()) {
    return {history_heading.value(), 0.85, "history"};
  }

  const Pose & pose = tracked.kinematics.pose_with_covariance.pose;
  const double pose_yaw = yaw_from_pose(pose);
  const auto & v = tracked.kinematics.twist_with_covariance.twist.linear;
  const double speed = std::hypot(v.x, v.y);
  if (speed >= config.map_conflict_heading_min_speed_mps) {
    // TrackedObject twist is handled consistently with the existing kinematic fallback: rotate
    // its object-local XY velocity into map coordinates using the measured object pose.
    const double c = std::cos(pose_yaw);
    const double si = std::sin(pose_yaw);
    const double vx = c * v.x - si * v.y;
    const double vy = si * v.x + c * v.y;
    if (std::hypot(vx, vy) > kEpsilon) {
      return {std::atan2(vy, vx), 0.65, "velocity"};
    }
  }
  return {pose_yaw, 0.25, "pose_yaw"};
}

std::optional<double> predicted_initial_motion_heading(
  const PredictedObject * predicted, const double min_confidence)
{
  if (predicted == nullptr) {
    return std::nullopt;
  }
  const PredictedPath * best_path = nullptr;
  for (const auto & path : predicted->kinematics.predicted_paths) {
    if (path.confidence < min_confidence || path.path.size() < 2) {
      continue;
    }
    if (best_path == nullptr || path.confidence > best_path->confidence) {
      best_path = &path;
    }
  }
  if (best_path == nullptr) {
    return std::nullopt;
  }
  const auto & first = best_path->path.front().position;
  for (std::size_t i = 1; i < best_path->path.size(); ++i) {
    const auto & p = best_path->path[i].position;
    const double dx = p.x - first.x;
    const double dy = p.y - first.y;
    if (std::hypot(dx, dy) > 0.5) {
      return std::atan2(dy, dx);
    }
  }
  return std::nullopt;
}

std::optional<double> predicted_near_term_lane_fit_m(
  const PredictedObject * predicted, const CandidateMapLaneGeometry & lane,
  const double min_confidence)
{
  if (predicted == nullptr || lane.centerline_map.size() < 2) {
    return std::nullopt;
  }
  const PredictedPath * best_path = nullptr;
  for (const auto & path : predicted->kinematics.predicted_paths) {
    if (path.confidence < min_confidence || path.path.empty()) {
      continue;
    }
    if (best_path == nullptr || path.confidence > best_path->confidence) {
      best_path = &path;
    }
  }
  if (best_path == nullptr) {
    return std::nullopt;
  }
  const double step_sec = duration_sec(best_path->time_step);
  if (step_sec <= kEpsilon) {
    return std::nullopt;
  }

  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t i = 0; i < best_path->path.size(); ++i) {
    if (static_cast<double>(i) * step_sec > 1.0 + 1.0e-6) {
      break;
    }
    const auto projection = project_to_map_lane(lane, best_path->path[i].position);
    if (!projection.has_value()) {
      continue;
    }
    sum += projection->lateral_distance_m;
    ++count;
  }
  if (count == 0) {
    return std::nullopt;
  }
  return sum / static_cast<double>(count);
}

std::vector<ObjectLaneHypothesis> associate_object_to_map_lanes(
  const TrackedObject & tracked, const PredictedObject * predicted,
  const CandidateMapConflictContext & context, const CandidateBankConfig & config,
  const std::optional<double> history_heading, MotionHeadingEstimate & motion_heading,
  bool & ambiguous)
{
  std::vector<ObjectLaneHypothesis> hypotheses;
  const Point & position = tracked.kinematics.pose_with_covariance.pose.position;
  const auto predicted_heading =
    predicted_initial_motion_heading(predicted, config.min_prediction_confidence);
  motion_heading = object_motion_heading(tracked, history_heading, predicted_heading, config);
  const double max_distance = std::max(0.1, config.map_conflict_lane_association_max_distance_m);
  const double distance_sigma = std::max(0.5, 0.5 * max_distance);
  const double heading_sigma =
    std::max(5.0, config.map_conflict_lane_association_heading_sigma_deg) * M_PI / 180.0;

  for (const auto & lane : context.lanes) {
    const auto projection = project_to_map_lane(lane, position);
    if (!projection.has_value()) {
      continue;
    }
    const auto predicted_fit =
      predicted_near_term_lane_fit_m(predicted, lane, config.min_prediction_confidence);
    // Around an intersection a tracked center point can temporarily leave the nominal lane
    // center corridor.  Retain the hypothesis when its near-term PredictedPath returns to / follows
    // the lane, while still keeping a bounded spatial gate.
    const bool current_pose_near = projection->lateral_distance_m <= max_distance;
    const bool predicted_path_near =
      predicted_fit.has_value() && predicted_fit.value() <= max_distance;
    if (!current_pose_near && !predicted_path_near) {
      continue;
    }
    const double effective_distance =
      predicted_fit.has_value() ? std::min(projection->lateral_distance_m, predicted_fit.value())
                                : projection->lateral_distance_m;
    const double heading_error = angle_error_rad(motion_heading.yaw, projection->tangent_yaw);
    const double distance_score =
      std::exp(-0.5 * std::pow(effective_distance / distance_sigma, 2.0));
    const double heading_score = std::exp(-0.5 * std::pow(heading_error / heading_sigma, 2.0));
    double confidence = distance_score * ((1.0 - motion_heading.reliability) +
                                          motion_heading.reliability * heading_score);
    if (predicted_heading.has_value() && motion_heading.source != "predicted_path") {
      const double path_error = angle_error_rad(predicted_heading.value(), projection->tangent_yaw);
      const double path_heading_score = std::exp(-0.5 * std::pow(path_error / heading_sigma, 2.0));
      confidence *= 0.7 + 0.3 * path_heading_score;
    }
    if (confidence < config.map_conflict_lane_association_min_confidence) {
      continue;
    }
    hypotheses.push_back(
      ObjectLaneHypothesis{
        lane.lane_id, lane.is_route_lane, projection->s_m, projection->lateral_distance_m,
        heading_error, predicted_fit.value_or(std::numeric_limits<double>::infinity()),
        confidence});
  }

  std::sort(hypotheses.begin(), hypotheses.end(), [](const auto & a, const auto & b) {
    if (std::abs(a.confidence - b.confidence) > kEpsilon) {
      return a.confidence > b.confidence;
    }
    return a.lateral_distance_m < b.lateral_distance_m;
  });
  if (hypotheses.size() > config.map_conflict_lane_hypothesis_top_k) {
    hypotheses.resize(config.map_conflict_lane_hypothesis_top_k);
  }
  ambiguous = hypotheses.empty() ||
              (hypotheses.size() > 1 && hypotheses[0].confidence - hypotheses[1].confidence < 0.10);
  return hypotheses;
}

std::optional<double> candidate_arrival_time_at_conflict(
  const TrajectoryCandidate & candidate, const Point & conflict_point,
  const double match_distance_m)
{
  double best_distance = std::numeric_limits<double>::infinity();
  std::optional<double> arrival;
  for (std::size_t i = candidate.live_start_index; i < candidate.trajectory_map.points.size();
       ++i) {
    const double d = distance_xy(candidate.trajectory_map.points[i].pose.position, conflict_point);
    if (d < best_distance) {
      best_distance = d;
      arrival = live_time_sec(candidate.trajectory_map, candidate.live_start_index, i);
    }
  }
  if (!arrival.has_value() || best_distance > match_distance_m) {
    return std::nullopt;
  }
  return arrival;
}

std::optional<double> predicted_arrival_time_at_conflict(
  const PredictedObject * predicted, const Point & conflict_point,
  const CandidateBankConfig & config)
{
  if (predicted == nullptr) {
    return std::nullopt;
  }
  std::optional<double> best_time;
  double best_confidence = -1.0;
  for (const auto & path : predicted->kinematics.predicted_paths) {
    if (path.confidence < config.min_prediction_confidence || path.path.empty()) {
      continue;
    }
    const double step = duration_sec(path.time_step);
    if (step <= kEpsilon) {
      continue;
    }
    double min_distance = std::numeric_limits<double>::infinity();
    std::size_t min_index = 0;
    for (std::size_t i = 0; i < path.path.size(); ++i) {
      const double d = distance_xy(path.path[i].position, conflict_point);
      if (d < min_distance) {
        min_distance = d;
        min_index = i;
      }
    }
    if (
      min_distance <= config.map_conflict_candidate_match_distance_m &&
      static_cast<double>(path.confidence) > best_confidence) {
      best_confidence = static_cast<double>(path.confidence);
      best_time = static_cast<double>(min_index) * step;
    }
  }
  return best_time;
}

struct DynamicScopeDecision
{
  bool full_evaluation{false};
  bool immediate_guard_only{false};
  std::string reason{"map_not_relevant"};
  CandidateMapConflictDecisionDebug debug;
};

DynamicScopeDecision decide_dynamic_scope(
  const TrajectoryCandidate & candidate, const nav_msgs::msg::Odometry & ego,
  const TrackedObject & tracked, const PredictedObject * predicted,
  const std::optional<Trajectory> & route_reference_map,
  const CandidateMapConflictContext & map_context, const CandidateBankConfig & config,
  const std::optional<double> history_heading)
{
  DynamicScopeDecision decision;
  decision.debug.object_id = tracked_object_id_string(tracked);

  if (config.dynamic_scope_mode == "all") {
    decision.full_evaluation = true;
    decision.reason = "scope_all";
    decision.debug.full_dynamic_evaluation = true;
    decision.debug.reason = decision.reason;
    return decision;
  }
  if (!map_context.valid) {
    decision.full_evaluation = config.map_conflict_fallback_to_all_if_map_unavailable;
    decision.immediate_guard_only =
      !decision.full_evaluation && config.map_conflict_immediate_guard_enabled;
    decision.reason =
      decision.full_evaluation ? "map_unavailable_fallback_all" : "map_unavailable_immediate_guard";
    decision.debug.full_dynamic_evaluation = decision.full_evaluation;
    decision.debug.immediate_guard_only = decision.immediate_guard_only;
    decision.debug.reason = decision.reason;
    return decision;
  }

  MotionHeadingEstimate motion_heading;
  bool ambiguous = true;
  const auto hypotheses = associate_object_to_map_lanes(
    tracked, predicted, map_context, config, history_heading, motion_heading, ambiguous);
  decision.debug.motion_heading_source = motion_heading.source;
  decision.debug.lane_association_ambiguous = ambiguous;
  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
  decision.debug.lane_associated = !hypotheses.empty();
  decision.debug.motion_heading_yaw = motion_heading.yaw;
  decision.debug.motion_heading_reliability = motion_heading.reliability;
  decision.debug.lane_hypotheses.reserve(hypotheses.size());
  for (const auto & hypothesis : hypotheses) {
    CandidateLaneHypothesisDebug logged_hypothesis;
    logged_hypothesis.lane_id = hypothesis.lane_id;
    logged_hypothesis.is_route_lane = hypothesis.is_route_lane;
    logged_hypothesis.s_m = hypothesis.s_m;
    logged_hypothesis.confidence = hypothesis.confidence;
    logged_hypothesis.lateral_distance_m = hypothesis.lateral_distance_m;
    logged_hypothesis.heading_error_deg = hypothesis.heading_error_rad * 180.0 / M_PI;
    logged_hypothesis.predicted_path_fit_m = hypothesis.predicted_path_fit_m;
    decision.debug.lane_hypotheses.push_back(std::move(logged_hypothesis));
  }
  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
  if (!hypotheses.empty()) {
    decision.debug.best_lane_id = hypotheses.front().lane_id;
    decision.debug.best_lane_confidence = hypotheses.front().confidence;
    decision.debug.best_lane_lateral_distance_m = hypotheses.front().lateral_distance_m;
    decision.debug.best_lane_heading_error_deg =
      hypotheses.front().heading_error_rad * 180.0 / M_PI;
    decision.debug.best_lane_predicted_path_fit_m = hypotheses.front().predicted_path_fit_m;
  }

  if (config.map_conflict_include_same_route_lead && route_reference_map.has_value()) {
    const auto ego_projection =
      project_to_reference(route_reference_map.value(), ego.pose.pose.position);
    const auto object_projection = project_to_reference(
      route_reference_map.value(), tracked.kinematics.pose_with_covariance.pose.position);
    if (
      ego_projection.has_value() && object_projection.has_value() &&
      object_projection->s > ego_projection->s + 0.5) {
      for (const auto & hypothesis : hypotheses) {
        if (hypothesis.is_route_lane && hypothesis.heading_error_rad < M_PI / 2.0) {
          decision.full_evaluation = true;
          decision.reason = "same_route_lead";
          decision.debug.full_dynamic_evaluation = true;
          decision.debug.reason = decision.reason;
          return decision;
        }
      }
    }
  }

  const double speed = tracked_speed_mps(tracked);
  for (const auto & hypothesis : hypotheses) {
    for (const auto & conflict : map_context.conflicts) {
      const auto approach = std::find_if(
        conflict.approaches.begin(), conflict.approaches.end(),
        [&](const auto & item) { return item.lane_id == hypothesis.lane_id; });
      if (approach == conflict.approaches.end()) {
        continue;
      }
      const double object_distance_m =
        approach->distance_from_lane_start_to_conflict_m - hypothesis.s_m;
      if (
        object_distance_m < -0.5 || object_distance_m > config.map_conflict_backtrace_distance_m) {
        continue;
      }
      const auto ego_arrival = candidate_arrival_time_at_conflict(
        candidate, conflict.point_map, config.map_conflict_candidate_match_distance_m);
      if (!ego_arrival.has_value()) {
        continue;
      }
      auto object_arrival =
        predicted_arrival_time_at_conflict(predicted, conflict.point_map, config);
      if (!object_arrival.has_value()) {
        object_arrival = std::max(0.0, object_distance_m) / std::max(0.5, speed);
      }
      if (
        std::abs(ego_arrival.value() - object_arrival.value()) <=
        config.map_conflict_arrival_time_margin_sec) {
        decision.full_evaluation = true;
        decision.reason = "map_conflict_reachable";
        decision.debug.full_dynamic_evaluation = true;
        decision.debug.reason = decision.reason;
        decision.debug.conflict_ego_lane_id = conflict.ego_route_lane_id;
        decision.debug.conflict_competing_lane_id = conflict.competing_lane_id;
        decision.debug.conflict_x_map = conflict.point_map.x;
        decision.debug.conflict_y_map = conflict.point_map.y;
        decision.debug.object_distance_to_conflict_m = object_distance_m;
        decision.debug.ego_arrival_time_sec = ego_arrival.value();
        decision.debug.object_arrival_time_sec = object_arrival.value();
        return decision;
      }
    }
  }

  decision.immediate_guard_only = config.map_conflict_immediate_guard_enabled;
  decision.reason =
    hypotheses.empty() ? "lane_unmatched" : (ambiguous ? "lane_ambiguous" : "map_not_reachable");
  decision.debug.immediate_guard_only = decision.immediate_guard_only;
  decision.debug.reason = decision.reason;
  return decision;
}

bool evaluate_dynamic_immediate_guard(
  const TrajectoryCandidate & candidate, const TrackedObject & tracked,
  const PredictedObjects & predicted_objects, const CandidateBankConfig & config,
  SafetyResult & result)
{
  const PredictedObject * predicted = find_predicted_object(predicted_objects, tracked);
  std::vector<const PredictedPath *> paths;
  if (predicted != nullptr) {
    for (const auto & path : predicted->kinematics.predicted_paths) {
      if (path.confidence >= config.min_prediction_confidence && !path.path.empty()) {
        paths.push_back(&path);
      }
    }
  }
  const std::size_t stride = std::max<std::size_t>(1, config.evaluation_stride);
  for (std::size_t i = candidate.live_start_index; i < candidate.trajectory_map.points.size();
       i += stride) {
    const double t = live_time_sec(candidate.trajectory_map, candidate.live_start_index, i);
    if (t > config.map_conflict_immediate_guard_horizon_sec + kEpsilon) {
      break;
    }
    const Pose & ego_pose = candidate.trajectory_map.points[i].pose;
    if (!paths.empty()) {
      for (const auto * path : paths) {
        const auto object_pose = predicted_pose_at(*path, t);
        if (!object_pose.has_value()) {
          continue;
        }
        const double clearance =
          min_clearance_to_object_m(ego_pose, tracked.shape, object_pose.value(), config);
        if (clearance <= config.map_conflict_immediate_guard_collision_margin_m) {
          result.dynamic_collision_margin_violated = true;
          if (config.dynamic_hard_reject_enabled) {
            result.safe = false;
          }
          return true;
        }
      }
    } else {
      const Pose object_pose = extrapolate_tracked_pose(tracked, t);
      const double clearance =
        min_clearance_to_object_m(ego_pose, tracked.shape, object_pose, config);
      if (clearance <= config.map_conflict_immediate_guard_collision_margin_m) {
        result.dynamic_collision_margin_violated = true;
        if (config.dynamic_hard_reject_enabled) {
          result.safe = false;
        }
        return true;
      }
    }
  }
  return false;
}
// CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END

void evaluate_dynamic_object(
  const TrajectoryCandidate & candidate, const TrackedObject & tracked,
  const PredictedObjects & predicted_objects, const CandidateBankConfig & config,
  SafetyResult & result)
{
  if (!config.dynamic_safety_enabled) {
    return;
  }

  const bool use_legacy_time_gap_cost = config.dynamic_temporal_cost_mode == "legacy_time_gap";
  const bool use_ttc_cost = config.dynamic_temporal_cost_mode == "ttc";
  const bool normalized_soft_cost =
    config.dynamic_soft_cost_aggregation_mode == "normalized_path_max";
  const std::string object_id = tracked_object_id_string(tracked);

  const auto merge_path_result = [&](
                                   const DynamicPathEvaluation & path_result,
                                   const double soft_weight, const double raw_path_confidence,
                                   const bool is_fallback, const int path_index) {
    result.min_dynamic_clearance_m =
      std::min(result.min_dynamic_clearance_m, path_result.min_clearance_m);
    result.max_dynamic_closing_rate_mps =
      std::max(result.max_dynamic_closing_rate_mps, path_result.max_closing_rate_mps);
    result.max_dynamic_uncertainty_margin_m =
      std::max(result.max_dynamic_uncertainty_margin_m, path_result.max_uncertainty_margin_m);

    if (path_result.collision_margin_violated) {
      result.dynamic_collision_margin_violated = true;
      const bool hard_reject_confident_enough =
        is_fallback || config.dynamic_hard_reject_min_path_confidence <= 0.0 ||
        raw_path_confidence >= config.dynamic_hard_reject_min_path_confidence;
      if (config.dynamic_hard_reject_enabled && hard_reject_confident_enough) {
        result.safe = false;
      }
    }

    const double clearance_cost =
      normalized_soft_cost ? std::max(0.0, soft_weight) * path_result.max_clearance_soft_cost
                           : path_result.sum_clearance_soft_cost;
    result.dynamic_clearance_soft_cost += clearance_cost;
    result.dynamic_soft_cost += clearance_cost;

    double ttc_cost = 0.0;
    if (path_result.ttc_sec.has_value()) {
      result.min_dynamic_ttc_sec =
        std::min(result.min_dynamic_ttc_sec, path_result.ttc_sec.value());
      const double deficit =
        std::max(0.0, config.dynamic_ttc_threshold_sec - path_result.ttc_sec.value());
      const double ttc_weight =
        normalized_soft_cost ? std::max(0.0, soft_weight) : std::max(0.0, raw_path_confidence);
      ttc_cost = ttc_weight * deficit * deficit;
      result.dynamic_ttc_soft_cost += ttc_cost;
      if (use_ttc_cost) {
        result.dynamic_soft_cost += ttc_cost;
      }
    }

    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    // Keep only the strongest same-time/TTC contributor for this candidate.  The samples are the
    // exact values used above, so the offline analyzer never re-implements collision math.
    const double path_total_cost = clearance_cost + (use_ttc_cost ? ttc_cost : 0.0);
    const bool better_debug_contributor =
      !result.dynamic_analysis_debug.valid ||
      path_total_cost > result.dynamic_analysis_debug.path_total_soft_cost + kEpsilon ||
      (std::abs(path_total_cost - result.dynamic_analysis_debug.path_total_soft_cost) <= kEpsilon &&
       path_result.min_clearance_m <
         (result.dynamic_analysis_debug.samples.empty()
            ? std::numeric_limits<double>::infinity()
            : result.dynamic_analysis_debug
                .samples[result.dynamic_analysis_debug.worst_sample_index]
                .evaluated_clearance_m));
    if (!path_result.samples.empty() && better_debug_contributor) {
      CandidateDynamicEvaluationDebug debug;
      debug.valid = true;
      debug.object_id = object_id;
      debug.path_index = path_index;
      debug.is_fallback = is_fallback;
      debug.path_confidence = raw_path_confidence;
      debug.normalized_confidence = std::max(0.0, soft_weight);
      debug.path_clearance_soft_cost = clearance_cost;
      debug.path_ttc_soft_cost = ttc_cost;
      debug.path_total_soft_cost = path_total_cost;
      debug.ttc_sec = path_result.ttc_sec.value_or(std::numeric_limits<double>::infinity());
      debug.collision_margin_violated = path_result.collision_margin_violated;
      debug.worst_sample_index = path_result.worst_sample_index;
      debug.samples = path_result.samples;
      result.dynamic_analysis_debug = std::move(debug);
    }
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
  };

  const PredictedObject * predicted = find_predicted_object(predicted_objects, tracked);
  std::vector<std::pair<std::size_t, const PredictedPath *>> usable_paths;
  double confidence_sum = 0.0;
  if (predicted != nullptr) {
    for (std::size_t path_index = 0; path_index < predicted->kinematics.predicted_paths.size();
         ++path_index) {
      const auto & path = predicted->kinematics.predicted_paths[path_index];
      if (path.confidence < config.min_prediction_confidence || path.path.empty()) {
        continue;
      }
      usable_paths.emplace_back(path_index, &path);
      confidence_sum += std::max(0.0, static_cast<double>(path.confidence));
    }
  }

  if (!usable_paths.empty()) {
    result.dynamic_prediction_path_count += usable_paths.size();
    result.dynamic_prediction_confidence_sum += confidence_sum;

    const double uniform_weight = 1.0 / static_cast<double>(usable_paths.size());
    for (const auto & [path_index, path] : usable_paths) {
      const double raw_confidence = std::max(0.0, static_cast<double>(path->confidence));
      const double normalized_weight =
        confidence_sum > kEpsilon ? raw_confidence / confidence_sum : uniform_weight;

      const DynamicPathEvaluation path_result =
        evaluate_dynamic_same_time_path(candidate, tracked, path, config);
      merge_path_result(
        path_result, normalized_weight, raw_confidence, false, static_cast<int>(path_index));

      // Keep the original all-pairs time-gap scan only for diagnostics in TTC mode.
      // legacy_time_gap explicitly restores the old scoring behavior.
      accumulate_predicted_path_time_gap(
        candidate, tracked, *path, config, use_legacy_time_gap_cost, result);
    }
    return;
  }

  // Kinematic fallback when no learned predicted path is usable.  This remains a single
  // hypothesis, so its soft weight is 1 and hard-reject confidence gating does not apply.
  result.dynamic_prediction_path_count += 1;
  result.dynamic_prediction_confidence_sum += 1.0;
  const DynamicPathEvaluation fallback_result =
    evaluate_dynamic_same_time_path(candidate, tracked, nullptr, config);
  merge_path_result(fallback_result, 1.0, 1.0, true, -1);
  accumulate_fallback_time_gap(candidate, tracked, config, use_legacy_time_gap_cost, result);
}

// CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
void record_dynamic_analysis_candidate(
  const std::uint64_t cycle_id, const TrajectoryCandidate & candidate, const std::string & role,
  const CandidateBankConfig & config)
{
  internal_debug::CandidateDynamicAnalysisLog logged;
  logged.role = role;
  logged.candidate_id = candidate.id;
  logged.source = candidate_source_name(candidate.source);
  logged.safe = candidate.safe;
  logged.progress_m = candidate.progress_m;
  logged.score = candidate.score;
  logged.score_dynamic_margin = candidate.score_dynamic_margin;
  logged.candidate_dynamic_clearance_soft_cost = candidate.dynamic_clearance_soft_cost;
  logged.candidate_dynamic_ttc_soft_cost = candidate.dynamic_ttc_soft_cost;
  logged.candidate_min_dynamic_clearance_m = candidate.min_dynamic_clearance_m;
  logged.candidate_min_dynamic_ttc_sec = candidate.min_dynamic_ttc_sec;
  logged.collision_margin_m = config.dynamic_collision_margin_m;
  logged.preferred_clearance_m = config.dynamic_preferred_clearance_m;
  logged.ttc_threshold_sec = config.dynamic_ttc_threshold_sec;
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
  logged.dynamic_scope_mode = config.dynamic_scope_mode;
  logged.map_filter_decisions.reserve(candidate.map_conflict_decisions_debug.size());
  for (const auto & decision : candidate.map_conflict_decisions_debug) {
    internal_debug::CandidateMapConflictDecisionLog out;
    out.object_id = decision.object_id;
    out.full_dynamic_evaluation = decision.full_dynamic_evaluation;
    out.immediate_guard_only = decision.immediate_guard_only;
    out.immediate_guard_violated = decision.immediate_guard_violated;
    out.reason = decision.reason;
    // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    out.lane_associated = decision.lane_associated;
    out.motion_heading_yaw = decision.motion_heading_yaw;
    out.motion_heading_reliability = decision.motion_heading_reliability;
    out.lane_hypotheses.reserve(decision.lane_hypotheses.size());
    for (const auto & hypothesis : decision.lane_hypotheses) {
      internal_debug::CandidateLaneHypothesisLog logged_hypothesis;
      logged_hypothesis.lane_id = hypothesis.lane_id;
      logged_hypothesis.is_route_lane = hypothesis.is_route_lane;
      logged_hypothesis.s_m = hypothesis.s_m;
      logged_hypothesis.confidence = hypothesis.confidence;
      logged_hypothesis.lateral_distance_m = hypothesis.lateral_distance_m;
      logged_hypothesis.heading_error_deg = hypothesis.heading_error_deg;
      logged_hypothesis.predicted_path_fit_m = hypothesis.predicted_path_fit_m;
      out.lane_hypotheses.push_back(std::move(logged_hypothesis));
    }
    // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    out.best_lane_id = decision.best_lane_id;
    out.best_lane_confidence = decision.best_lane_confidence;
    out.best_lane_lateral_distance_m = decision.best_lane_lateral_distance_m;
    out.best_lane_heading_error_deg = decision.best_lane_heading_error_deg;
    out.best_lane_predicted_path_fit_m = decision.best_lane_predicted_path_fit_m;
    out.motion_heading_source = decision.motion_heading_source;
    out.lane_association_ambiguous = decision.lane_association_ambiguous;
    out.conflict_ego_lane_id = decision.conflict_ego_lane_id;
    out.conflict_competing_lane_id = decision.conflict_competing_lane_id;
    out.conflict_x_map = decision.conflict_x_map;
    out.conflict_y_map = decision.conflict_y_map;
    out.object_distance_to_conflict_m = decision.object_distance_to_conflict_m;
    out.ego_arrival_time_sec = decision.ego_arrival_time_sec;
    out.object_arrival_time_sec = decision.object_arrival_time_sec;
    logged.map_filter_decisions.push_back(std::move(out));
  }
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END

  const auto & debug = candidate.dynamic_analysis_debug;
  logged.detail_valid = debug.valid;
  logged.object_id = debug.object_id;
  logged.path_index = debug.path_index;
  logged.is_fallback = debug.is_fallback;
  logged.path_confidence = debug.path_confidence;
  logged.normalized_confidence = debug.normalized_confidence;
  logged.path_clearance_soft_cost = debug.path_clearance_soft_cost;
  logged.path_ttc_soft_cost = debug.path_ttc_soft_cost;
  logged.path_total_soft_cost = debug.path_total_soft_cost;
  logged.ttc_sec = debug.ttc_sec;
  logged.collision_margin_violated = debug.collision_margin_violated;
  logged.worst_sample_index = debug.worst_sample_index;
  logged.samples.reserve(debug.samples.size());
  for (const auto & sample : debug.samples) {
    internal_debug::CandidateDynamicSampleLog out;
    out.time_sec = sample.time_sec;
    out.ego_x = sample.ego_x;
    out.ego_y = sample.ego_y;
    out.ego_yaw = sample.ego_yaw;
    out.object_x = sample.object_x;
    out.object_y = sample.object_y;
    out.object_yaw = sample.object_yaw;
    out.geometric_clearance_m = sample.geometric_clearance_m;
    out.closing_rate_mps = sample.closing_rate_mps;
    out.uncertainty_margin_m = sample.uncertainty_margin_m;
    out.evaluated_clearance_m = sample.evaluated_clearance_m;
    out.clearance_soft_cost = sample.clearance_soft_cost;
    out.collision_margin_violated = sample.collision_margin_violated;
    logged.samples.push_back(out);
  }
  internal_debug::record_candidate_dynamic_analysis(cycle_id, logged);
}

std::vector<TrajectoryCandidate>::const_iterator highest_progress_candidate(
  const std::vector<TrajectoryCandidate> & candidates)
{
  return std::max_element(
    candidates.begin(), candidates.end(),
    [](const TrajectoryCandidate & lhs, const TrajectoryCandidate & rhs) {
      if (lhs.valid != rhs.valid) {
        return !lhs.valid && rhs.valid;
      }
      if (!lhs.valid && !rhs.valid) {
        return lhs.id < rhs.id;
      }
      return lhs.progress_m < rhs.progress_m;
    });
}
// CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

}  // namespace

// CB_UNIFIED_TRACE_V01_BEGIN
void cb_compare_trace_trajectory_event(
  const char * stage, const char * mode, const nav_msgs::msg::Odometry & ego,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const rclcpp::Time & eval_time,
  const std::uint64_t candidate_id, const double hold_age_sec)
{
  if (!cb_ct_on()) return;
  const auto yaw_of = [](const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };
  const double ego_yaw = yaw_of(ego.pose.pose.orientation);
  const auto & ev = ego.twist.twist.linear;
  const double ego_speed = std::hypot(ev.x, ev.y);
  std::ostringstream o;
  o << "{\"schema\":\"cb_diag_v2\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"trajectory_stage\",\"stage\":\"" << stage << "\",\"mode\":\"" << mode
    << "\",\"candidate_id\":" << candidate_id << ",\"eval_time_sec\":";
  cb_ct_num(o, eval_time.seconds());
  o << ",\"ego_stamp_sec\":";
  cb_ct_num(o, rclcpp::Time(ego.header.stamp).seconds());
  o << ",\"ego_x\":";
  cb_ct_num(o, ego.pose.pose.position.x);
  o << ",\"ego_y\":";
  cb_ct_num(o, ego.pose.pose.position.y);
  o << ",\"ego_yaw\":";
  cb_ct_num(o, ego_yaw);
  o << ",\"ego_speed_mps\":";
  cb_ct_num(o, ego_speed);
  o << ",\"trajectory_header_stamp_sec\":";
  cb_ct_num(o, rclcpp::Time(trajectory.header.stamp).seconds());
  o << ",\"hold_age_sec\":";
  cb_ct_num(o, hold_age_sec);
  o << ",\"point_count\":" << trajectory.points.size();
  if (!trajectory.points.empty()) {
    const auto & p = trajectory.points.front();
    const double dx = p.pose.position.x - ego.pose.pose.position.x;
    const double dy = p.pose.position.y - ego.pose.pose.position.y;
    double heading_error = yaw_of(p.pose.orientation) - ego_yaw;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));
    o << ",\"first_x\":";
    cb_ct_num(o, p.pose.position.x);
    o << ",\"first_y\":";
    cb_ct_num(o, p.pose.position.y);
    o << ",\"first_v\":";
    cb_ct_num(o, p.longitudinal_velocity_mps);
    o << ",\"first_time_sec\":";
    cb_ct_num(o, rclcpp::Duration(p.time_from_start).seconds());
    o << ",\"start_offset_m\":";
    cb_ct_num(o, std::hypot(dx, dy));
    o << ",\"start_heading_error_rad\":";
    cb_ct_num(o, heading_error);
  }
  o << ",\"trajectory_xyvt\":[";
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    if (i) {
      o << ',';
    }
    const auto & p = trajectory.points[i];
    o << '[';
    cb_ct_num(o, p.pose.position.x);
    o << ',';
    cb_ct_num(o, p.pose.position.y);
    o << ',';
    cb_ct_num(o, p.longitudinal_velocity_mps);
    o << ',';
    cb_ct_num(o, rclcpp::Duration(p.time_from_start).seconds());
    o << ']';
  }
  o << "]}";
  cb_ct_write(o.str());
}

void cb_compare_trace_bank_summary(
  const char * phase, const nav_msgs::msg::Odometry & ego, const CandidateBankDebugInfo & d,
  const rclcpp::Time & eval_time, const std::size_t admitted_now)
{
  if (!cb_ct_on()) return;
  std::ostringstream o;
  o << "{\"schema\":\"cb_diag_v2\",\"impl\":\"" << cb_ct_label()
    << "\",\"event\":\"bank_summary\",\"phase\":\"" << phase << "\",\"eval_time_sec\":";
  cb_ct_num(o, eval_time.seconds());
  o << ",\"ego_x\":";
  cb_ct_num(o, ego.pose.pose.position.x);
  o << ",\"ego_y\":";
  cb_ct_num(o, ego.pose.pose.position.y);
  o << ",\"bank_size\":" << d.bank_size << ",\"safe_count\":" << d.safe_count
    << ",\"admitted_now\":" << admitted_now << ",\"newest_candidate_id\":" << d.newest_candidate_id
    << ",\"best_candidate_id\":" << d.best_candidate_id
    << ",\"selected_candidate_id\":" << d.selected_candidate_id
    << ",\"previous_selected_candidate_id\":" << d.previous_selected_candidate_id
    << ",\"selection_switched\":" << (d.selection_switched ? "true" : "false")
    << ",\"hysteresis_kept\":" << (d.hysteresis_kept ? "true" : "false")
    << ",\"selection_reason\":\"" << d.selection_reason << "\""
    << ",\"selected_safety_tier\":\"" << d.selected_safety_tier << "\""
    << ",\"best_safety_tier\":\"" << d.best_safety_tier << "\"";
  o << ",\"best_score\":";
  cb_ct_num(o, d.best_score);
  o << ",\"selected_score\":";
  cb_ct_num(o, d.selected_score);
  o << '}';
  cb_ct_write(o.str());
}
// CB_UNIFIED_TRACE_V01_END

TrajectoryCandidateManager::TrajectoryCandidateManager(const CandidateBankConfig & config)
: config_(config)
{
  debug_info_.mode = config_.mode;
  debug_info_.collision_geometry_mode = config_.collision_geometry_mode;
  // CANDIDATE_BANK_TTC_COST_V1_BEGIN
  debug_info_.dynamic_temporal_cost_mode = config_.dynamic_temporal_cost_mode;
  // CANDIDATE_BANK_TTC_COST_V1_END
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
  debug_info_.dynamic_soft_cost_aggregation_mode = config_.dynamic_soft_cost_aggregation_mode;
  debug_info_.dynamic_uncertainty_mode = config_.dynamic_uncertainty_mode;
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
  debug_info_.dynamic_scope_mode = config_.dynamic_scope_mode;
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
  debug_info_.route_deviation_adaptive_weight_enabled =
    config_.route_deviation_adaptive_weight_enabled;
  debug_info_.traffic_signal_rule_enabled = config_.traffic_signal_rule_enabled;
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
  debug_info_.traffic_signal_scope_mode = "route_nearest";
  debug_info_.lane_boundary_enabled = config_.lane_boundary_enabled;
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
  debug_info_.lane_change_deadline_enabled = config_.lane_change_deadline_enabled;
  debug_info_.lane_change_deadline_activation_mode = config_.lane_change_deadline_activation_mode;
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
}

void TrajectoryCandidateManager::set_config(const CandidateBankConfig & config)
{
  const bool was_enabled = enabled();
  config_ = config;
  debug_info_.mode = config_.mode;
  debug_info_.collision_geometry_mode = config_.collision_geometry_mode;
  // CANDIDATE_BANK_TTC_COST_V1_BEGIN
  debug_info_.dynamic_temporal_cost_mode = config_.dynamic_temporal_cost_mode;
  // CANDIDATE_BANK_TTC_COST_V1_END
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
  debug_info_.dynamic_soft_cost_aggregation_mode = config_.dynamic_soft_cost_aggregation_mode;
  debug_info_.dynamic_uncertainty_mode = config_.dynamic_uncertainty_mode;
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
  debug_info_.dynamic_scope_mode = config_.dynamic_scope_mode;
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
  debug_info_.traffic_signal_scope_mode = "route_nearest";
  debug_info_.lane_boundary_enabled = config_.lane_boundary_enabled;
  debug_info_.lane_change_deadline_enabled = config_.lane_change_deadline_enabled;
  debug_info_.lane_change_deadline_activation_mode = config_.lane_change_deadline_activation_mode;
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
  if (was_enabled && !enabled()) {
    clear();
  }
  enforce_size_limit();
  refresh_debug_info();
}

void TrajectoryCandidateManager::clear()
{
  candidates_.clear();
  cross_collision_results_.clear();
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
  cross_aggregate_results_.clear();
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
  obstacle_geometry_debug_.clear();
  object_history_.clear();
  incumbent_id_.reset();
  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
  has_logged_map_context_signature_ = false;
  last_logged_map_context_signature_ = 0;
  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
  debug_info_ = CandidateBankDebugInfo{};
  debug_info_.mode = config_.mode;
  debug_info_.collision_geometry_mode = config_.collision_geometry_mode;
  debug_info_.dynamic_temporal_cost_mode = config_.dynamic_temporal_cost_mode;
  debug_info_.dynamic_soft_cost_aggregation_mode = config_.dynamic_soft_cost_aggregation_mode;
  debug_info_.dynamic_uncertainty_mode = config_.dynamic_uncertainty_mode;
  debug_info_.dynamic_scope_mode = config_.dynamic_scope_mode;
}

void TrajectoryCandidateManager::update_object_history(
  const std::shared_ptr<const TrackedObjects> & tracked_objects, const rclcpp::Time & now)
{
  if (!tracked_objects) {
    return;
  }
  const double keep_sec = std::max(0.2, config_.map_conflict_history_window_sec);
  std::set<std::string> seen;
  for (const auto & object : tracked_objects->objects) {
    const std::string id = tracked_object_id_string(object);
    seen.insert(id);
    auto & history = object_history_[id];
    ObjectHistorySample sample;
    sample.stamp = now;
    sample.position = object.kinematics.pose_with_covariance.pose.position;
    history.push_back(sample);
    while (!history.empty() && (now - history.front().stamp).seconds() > keep_sec) {
      history.pop_front();
    }
  }
  for (auto it = object_history_.begin(); it != object_history_.end();) {
    if (
      seen.count(it->first) == 0U &&
      (it->second.empty() || (now - it->second.back().stamp).seconds() > keep_sec)) {
      it = object_history_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<double> TrajectoryCandidateManager::history_motion_heading(
  const std::string & object_id, const Point & current_position, const rclcpp::Time & now) const
{
  const auto found = object_history_.find(object_id);
  if (found == object_history_.end() || found->second.size() < 2) {
    return std::nullopt;
  }
  const double window = std::max(0.2, config_.map_conflict_history_window_sec);
  for (const auto & sample : found->second) {
    const double age = (now - sample.stamp).seconds();
    if (age < 0.15 || age > window + 1.0e-6) {
      continue;
    }
    const double dx = current_position.x - sample.position.x;
    const double dy = current_position.y - sample.position.y;
    if (std::hypot(dx, dy) >= 0.4) {
      return std::atan2(dy, dx);
    }
  }
  return std::nullopt;
}

bool TrajectoryCandidateManager::enabled() const
{
  return config_.mode != "off";
}

bool TrajectoryCandidateManager::evaluation_enabled() const
{
  return config_.mode == "evaluate" || config_.mode == "select";
}

bool TrajectoryCandidateManager::selection_enabled() const
{
  return config_.mode == "select";
}

void TrajectoryCandidateManager::add(
  const Trajectory & trajectory_map, const CandidateSource source,
  const PredictedObjects & paired_predicted_objects,
  const std::int64_t paired_prediction_batch_index,
  const std::optional<Trajectory> & route_reference_map, const rclcpp::Time & now)
{
  if (!enabled() || trajectory_map.points.size() < 2 || trajectory_map.header.frame_id != "map") {
    return;
  }

  // CANDIDATE_BANK_COLLISION_GEOMETRY_V1_BEGIN
  // Prevent stale obstacle geometry from remaining visible in observe-only mode.
  obstacle_geometry_debug_.clear();
  // CANDIDATE_BANK_COLLISION_GEOMETRY_V1_END

  // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: add() is called once per planner cycle.
  // Reset this per-cycle counter before the first purge; evaluate() may call purge() again.
  debug_info_.purged_last_cycle = 0;
  // CANDIDATE_BANK_DIAGNOSTICS_V2_END

  TrajectoryCandidate candidate;
  candidate.id = next_id_++;
  candidate.source = source;
  candidate.trajectory_map = trajectory_map;
  // CANDIDATE_BANK_TRAJECTORY_SANITIZATION_V1_BEGIN
  if (config_.clip_backward_points_enabled) {
    clip_backward_points_in_place(candidate.trajectory_map, route_reference_map);
  }
  // CANDIDATE_BANK_TRAJECTORY_SANITIZATION_V1_END
  // CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
  candidate.paired_predicted_objects = paired_predicted_objects;
  candidate.paired_prediction_batch_index = paired_prediction_batch_index;
  // CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
  candidate.generation_time = now;
  candidate.last_evaluation_time = now;
  candidates_.push_back(std::move(candidate));
  cb_ct_add(candidates_.back(), now);

  // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: geometry is written once, not every cycle.
  internal_debug::record_candidate_generated(
    internal_debug::current_cycle_id(), candidates_.back().id,
    candidate_source_name(candidates_.back().source), candidates_.back().trajectory_map);
  // CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
  // Log the paired other-agent future exactly once at candidate generation.  Re-evaluation events
  // refer to candidate_id and do not duplicate the trajectory arrays every planning cycle.
  internal_debug::record_candidate_joint_prediction(
    internal_debug::current_cycle_id(), candidates_.back().id,
    candidate_source_name(candidates_.back().source), candidates_.back().paired_predicted_objects,
    candidates_.back().paired_prediction_batch_index);
  // CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
  // CANDIDATE_BANK_DIAGNOSTICS_V2_END

  debug_info_.newest_candidate_id = candidates_.back().id;
  debug_info_.newest_source = candidate_source_name(candidates_.back().source);
  enforce_size_limit();
  refresh_debug_info();
}

void TrajectoryCandidateManager::purge(
  const nav_msgs::msg::Odometry & ego, const rclcpp::Time & now)
{
  if (!enabled()) {
    return;
  }

  const std::size_t before = candidates_.size();
  for (auto & candidate : candidates_) {
    const double age_sec = (now - candidate.generation_time).seconds();
    candidate.live_start_index =
      config_.temporal_alignment_enabled
        ? time_aligned_trajectory_index(
            candidate.trajectory_map, ego.pose.pose.position, std::max(0.0, age_sec),
            config_.temporal_alignment_search_window_sec, candidate.projection_error_m)
        : nearest_trajectory_index(
            candidate.trajectory_map, ego.pose.pose.position, candidate.projection_error_m);
    candidate.remaining_horizon_sec =
      remaining_horizon_sec(candidate.trajectory_map, candidate.live_start_index);
    candidate.valid = age_sec <= config_.max_generation_age_sec &&
                      candidate.projection_error_m <= config_.max_projection_error_m &&
                      candidate.remaining_horizon_sec >= config_.min_remaining_horizon_sec;
    cb_ct_state("purge", candidate, ego, now);

    // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: record the first invalidation reason.
    if (!candidate.valid) {
      std::string purge_reason{"unknown"};
      if (age_sec > config_.max_generation_age_sec) {
        purge_reason = "age";
      } else if (candidate.projection_error_m > config_.max_projection_error_m) {
        purge_reason = "projection_error";
      } else if (candidate.remaining_horizon_sec < config_.min_remaining_horizon_sec) {
        purge_reason = "remaining_horizon";
      }
      internal_debug::record_candidate_purged(
        internal_debug::current_cycle_id(), candidate.id, candidate_source_name(candidate.source),
        purge_reason, age_sec, candidate.projection_error_m, candidate.remaining_horizon_sec);
    }
    // CANDIDATE_BANK_DIAGNOSTICS_V2_END
  }

  candidates_.erase(
    std::remove_if(
      candidates_.begin(), candidates_.end(),
      [](const TrajectoryCandidate & candidate) { return !candidate.valid; }),
    candidates_.end());

  if (incumbent_id_.has_value()) {
    const bool exists = std::any_of(
      candidates_.begin(), candidates_.end(),
      [&](const TrajectoryCandidate & candidate) { return candidate.id == incumbent_id_.value(); });
    if (!exists) {
      incumbent_id_.reset();
    }
  }

  enforce_size_limit();
  debug_info_.purged_last_cycle += before - candidates_.size();
  refresh_debug_info();
}

void TrajectoryCandidateManager::evaluate(
  const nav_msgs::msg::Odometry & ego,
  const std::shared_ptr<const TrackedObjects> & tracked_objects,
  const PredictedObjects & predicted_objects, const std::optional<Trajectory> & route_reference_map,
  const CandidateMapConflictContext & map_conflict_context,
  const CandidateTrafficRuleContext & traffic_rule_context, const rclcpp::Time & now)
{
  if (!evaluation_enabled()) {
    return;
  }

  purge(ego, now);
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
  update_object_history(tracked_objects, now);
  debug_info_.dynamic_scope_mode = config_.dynamic_scope_mode;
  debug_info_.map_conflict_count = map_conflict_context.conflicts.size();
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END

  // CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
  // Cross-pairs are deliberately diagnostic in 0013a7: E_i is tested against the
  // complete paired scene O_j, but the result does not modify score/safe/selection.
  // This lets us first verify whether counterfactual historical hypotheses are
  // useful before assigning calibrated probabilities or hard planner authority.
  cross_collision_results_ = evaluate_and_log_cross_collision_matrix(candidates_, config_, now);
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
  cross_aggregate_results_ =
    aggregate_cross_collision(candidates_, cross_collision_results_, config_);
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
  // CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END

  // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: one compact environment snapshot per evaluation cycle.
  std::vector<internal_debug::CandidateObstacleLog> logged_obstacles;
  std::vector<internal_debug::CandidateObstaclePointLog> logged_ego_polygon;
  obstacle_geometry_debug_.clear();

  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1A_BEGIN
  // Always log the configured ego box for offline visualization, independent of the collision
  // geometry mode selected for evaluation. This is diagnostics-only and does not change safety.
  if (const auto ego_polygon = make_ego_polygon(ego.pose.pose, config_); ego_polygon.has_value()) {
    for (const auto & point : polygon_to_geometry_points(ego_polygon.value())) {
      internal_debug::CandidateObstaclePointLog logged_point;
      logged_point.x = point.x;
      logged_point.y = point.y;
      logged_ego_polygon.push_back(logged_point);
    }
  }
  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1A_END

  if (tracked_objects) {
    logged_obstacles.reserve(tracked_objects->objects.size());
    obstacle_geometry_debug_.reserve(tracked_objects->objects.size());
    for (const auto & object : tracked_objects->objects) {
      const auto geometry_debug = make_obstacle_geometry_debug(object, config_);
      obstacle_geometry_debug_.push_back(geometry_debug);

      internal_debug::CandidateObstacleLog logged;
      // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
      logged.object_id = tracked_object_id_string(object);
      // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
      const Pose & object_pose = object.kinematics.pose_with_covariance.pose;
      logged.x = object_pose.position.x;
      logged.y = object_pose.position.y;
      logged.yaw = yaw_from_pose(object_pose);
      logged.length_m = std::max(0.0, static_cast<double>(object.shape.dimensions.x));
      logged.width_m = std::max(0.0, static_cast<double>(object.shape.dimensions.y));
      logged.shape_type = static_cast<int>(object.shape.type);
      logged.radius_m = legacy_object_radius_m(object.shape);
      for (const auto & circle : geometry_debug.circles) {
        internal_debug::CandidateObstacleCircleLog circle_log;
        circle_log.x = circle.center.x;
        circle_log.y = circle.center.y;
        circle_log.radius_m = circle.radius_m;
        logged.collision_circles.push_back(circle_log);
      }
      for (const auto & point : geometry_debug.polygon) {
        internal_debug::CandidateObstaclePointLog point_log;
        point_log.x = point.x;
        point_log.y = point.y;
        logged.collision_polygon.push_back(point_log);
      }
      logged.speed_mps = tracked_speed_mps(object);
      logged.is_static = logged.speed_mps <= config_.static_velocity_threshold_mps;
      logged_obstacles.push_back(std::move(logged));
    }
  }
  internal_debug::record_candidate_observation(
    internal_debug::current_cycle_id(), ego.pose.pose.position.x, ego.pose.pose.position.y,
    yaw_from_pose(ego.pose.pose), config_.collision_geometry_mode, logged_ego_polygon,
    logged_obstacles);

  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
  // Map geometry is static for a given lane/conflict set, so write it only when that set changes.
  // The offline analyzer carries the most recent context forward to following cycles.
  if (internal_debug::config().enabled && config_.dynamic_scope_mode == "map_conflict_reachable") {
    const std::uint64_t signature = map_context_debug_signature(map_conflict_context, config_);
    if (!has_logged_map_context_signature_ || signature != last_logged_map_context_signature_) {
      internal_debug::CandidateMapContextLog logged_context;
      logged_context.valid = map_conflict_context.valid;
      logged_context.forward_search_distance_m = config_.map_conflict_forward_search_distance_m;
      logged_context.backtrace_distance_m = config_.map_conflict_backtrace_distance_m;

      std::set<std::int64_t> competing_lane_ids;
      for (const auto & conflict : map_conflict_context.conflicts) {
        competing_lane_ids.insert(conflict.competing_lane_id);
        for (const auto & approach : conflict.approaches) {
          competing_lane_ids.insert(approach.lane_id);
        }
      }

      logged_context.lanes.reserve(map_conflict_context.lanes.size());
      for (const auto & lane : map_conflict_context.lanes) {
        internal_debug::CandidateMapContextLaneLog logged_lane;
        logged_lane.lane_id = lane.lane_id;
        logged_lane.is_route_lane = lane.is_route_lane;
        logged_lane.is_competing_lane = competing_lane_ids.count(lane.lane_id) != 0U;
        logged_lane.route_s_start_m = lane.route_s_start_m;
        logged_lane.centerline.reserve(lane.centerline_map.size());
        for (const auto & point : lane.centerline_map) {
          internal_debug::CandidateMapContextPointLog logged_point;
          logged_point.x = point.x;
          logged_point.y = point.y;
          logged_lane.centerline.push_back(logged_point);
        }
        logged_context.lanes.push_back(std::move(logged_lane));
      }

      logged_context.conflicts.reserve(map_conflict_context.conflicts.size());
      for (const auto & conflict : map_conflict_context.conflicts) {
        internal_debug::CandidateMapContextConflictLog logged_conflict;
        logged_conflict.ego_route_lane_id = conflict.ego_route_lane_id;
        logged_conflict.competing_lane_id = conflict.competing_lane_id;
        logged_conflict.x = conflict.point_map.x;
        logged_conflict.y = conflict.point_map.y;
        logged_conflict.ego_route_distance_m = conflict.ego_route_distance_m;
        logged_conflict.approach_lane_ids.reserve(conflict.approaches.size());
        for (const auto & approach : conflict.approaches) {
          logged_conflict.approach_lane_ids.push_back(approach.lane_id);
        }
        logged_context.conflicts.push_back(std::move(logged_conflict));
      }
      internal_debug::record_candidate_map_context(
        internal_debug::current_cycle_id(), logged_context);
      has_logged_map_context_signature_ = true;
      last_logged_map_context_signature_ = signature;
    }
  }
  // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
  // CANDIDATE_BANK_DIAGNOSTICS_V2_END

  for (auto & candidate : candidates_) {
    candidate.last_evaluation_time = now;
    candidate.safe = true;
    candidate.min_static_clearance_m = std::numeric_limits<double>::infinity();
    candidate.min_dynamic_clearance_m = std::numeric_limits<double>::infinity();
    candidate.min_dynamic_time_gap_sec = std::numeric_limits<double>::infinity();
    // CANDIDATE_BANK_TTC_COST_V1_BEGIN
    candidate.min_dynamic_ttc_sec = std::numeric_limits<double>::infinity();
    candidate.dynamic_clearance_soft_cost = 0.0;
    candidate.dynamic_legacy_time_gap_soft_cost = 0.0;
    candidate.dynamic_ttc_soft_cost = 0.0;
    // CANDIDATE_BANK_TTC_COST_V1_END
    // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
    candidate.max_dynamic_closing_rate_mps = 0.0;
    candidate.max_dynamic_uncertainty_margin_m = 0.0;
    candidate.dynamic_prediction_path_count = 0;
    candidate.dynamic_prediction_confidence_sum = 0.0;
    // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    candidate.dynamic_analysis_debug = CandidateDynamicEvaluationDebug{};
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
    // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
    candidate.map_conflict_decisions_debug.clear();
    // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END

    candidate.path_length_m = path_length_m(candidate.trajectory_map, candidate.live_start_index);
    candidate.progress_m =
      route_progress_m(candidate.trajectory_map, candidate.live_start_index, route_reference_map);
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
    const RouteDeviationResult route_deviation_result =
      route_reference_map.has_value() ? route_deviation(
                                          candidate.trajectory_map, candidate.live_start_index,
                                          route_reference_map.value(), config_)
                                      : RouteDeviationResult{};
    candidate.route_deviation_rms_m = route_deviation_result.rms_m;
    candidate.route_reference_weight_mean = route_deviation_result.mean_reference_weight;
    candidate.route_reference_max_roughness_m = route_deviation_result.max_reference_roughness_m;
    const TrafficRuleEvaluation traffic_rule_evaluation = evaluate_traffic_rules(
      candidate.trajectory_map, candidate.live_start_index, route_reference_map,
      traffic_rule_context, config_);
    candidate.traffic_signal_stop_active = traffic_rule_evaluation.signal_stop_active;
    candidate.traffic_signal_crossed = traffic_rule_evaluation.signal_crossed;
    candidate.traffic_signal_max_stop_zone_speed_mps =
      traffic_rule_evaluation.max_stop_zone_speed_mps;
    candidate.traffic_rule_cost = traffic_rule_evaluation.cost;
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
    candidate.traffic_signal_scope_mode = traffic_rule_context.traffic_signal_scope_mode;
    candidate.current_lane_valid = traffic_rule_context.current_lane_valid;
    candidate.current_lane_id = traffic_rule_context.current_lane_id;
    const LaneBoundaryEvaluation lane_boundary_evaluation = evaluate_lane_boundary(
      candidate.trajectory_map, candidate.live_start_index, traffic_rule_context, config_);
    candidate.max_continuous_straddling_sec =
      lane_boundary_evaluation.max_continuous_straddling_sec;
    candidate.terminal_straddling = lane_boundary_evaluation.terminal_straddling;
    candidate.prolonged_straddling_cost = lane_boundary_evaluation.prolonged_straddling_cost;
    candidate.terminal_straddling_cost = lane_boundary_evaluation.terminal_straddling_cost;
    candidate.lane_boundary_cost = lane_boundary_evaluation.total_cost;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
    const LaneChangeDeadlineEvaluation lane_change_evaluation = evaluate_lane_change_deadline(
      candidate.trajectory_map, candidate.live_start_index, ego.pose.pose, route_reference_map,
      traffic_rule_context, config_);
    candidate.lane_change_deadline_active = lane_change_evaluation.active;
    candidate.lane_change_attempted = lane_change_evaluation.attempted;
    candidate.lane_change_completed = lane_change_evaluation.completed;
    candidate.lane_change_completed_before_deadline =
      lane_change_evaluation.completed_before_deadline;
    candidate.lane_change_fallback_stop_satisfied = lane_change_evaluation.fallback_stop_satisfied;
    candidate.lane_change_boundary_id = lane_change_evaluation.boundary_id;
    candidate.lane_change_target_lane_id = lane_change_evaluation.target_lane_id;
    candidate.lane_change_deadline_s_m = lane_change_evaluation.deadline_s_m;
    candidate.lane_change_completion_s_m = lane_change_evaluation.completion_s_m;
    candidate.lane_change_slack_m = lane_change_evaluation.slack_m;
    candidate.lane_change_deadline_cost = lane_change_evaluation.cost;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
    candidate.comfort_cost = comfort_cost(candidate.trajectory_map, candidate.live_start_index);

    SafetyResult safety;
    if (tracked_objects) {
      for (const auto & object : tracked_objects->objects) {
        if (tracked_speed_mps(object) <= config_.static_velocity_threshold_mps) {
          evaluate_static_object(candidate, object, config_, safety);
          continue;
        }

        // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
        if (!config_.dynamic_safety_enabled) {
          continue;
        }
        const std::string object_id = tracked_object_id_string(object);
        const auto history_heading = history_motion_heading(
          object_id, object.kinematics.pose_with_covariance.pose.position, now);
        const PredictedObject * predicted = find_predicted_object(predicted_objects, object);
        DynamicScopeDecision scope = decide_dynamic_scope(
          candidate, ego, object, predicted, route_reference_map, map_conflict_context, config_,
          history_heading);
        if (scope.full_evaluation) {
          evaluate_dynamic_object(candidate, object, predicted_objects, config_, safety);
        } else if (scope.immediate_guard_only) {
          scope.debug.immediate_guard_violated =
            evaluate_dynamic_immediate_guard(candidate, object, predicted_objects, config_, safety);
        }
        candidate.map_conflict_decisions_debug.push_back(std::move(scope.debug));
        // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END
      }
    }

    candidate.safe = safety.safe;
    // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
    candidate.static_collision_margin_violated = safety.static_collision_margin_violated;
    candidate.dynamic_collision_margin_violated = safety.dynamic_collision_margin_violated;
    // CANDIDATE_BANK_SOFT_COLLISION_V1_END
    candidate.min_static_clearance_m = safety.min_static_clearance_m;
    candidate.min_dynamic_clearance_m = safety.min_dynamic_clearance_m;
    candidate.min_dynamic_time_gap_sec = safety.min_dynamic_time_gap_sec;
    // CANDIDATE_BANK_TTC_COST_V1_BEGIN
    candidate.min_dynamic_ttc_sec = safety.min_dynamic_ttc_sec;
    candidate.dynamic_clearance_soft_cost = safety.dynamic_clearance_soft_cost;
    candidate.dynamic_legacy_time_gap_soft_cost = safety.dynamic_legacy_time_gap_soft_cost;
    candidate.dynamic_ttc_soft_cost = safety.dynamic_ttc_soft_cost;
    // CANDIDATE_BANK_TTC_COST_V1_END
    // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
    candidate.max_dynamic_closing_rate_mps = safety.max_dynamic_closing_rate_mps;
    candidate.max_dynamic_uncertainty_margin_m = safety.max_dynamic_uncertainty_margin_m;
    candidate.dynamic_prediction_path_count = safety.dynamic_prediction_path_count;
    candidate.dynamic_prediction_confidence_sum = safety.dynamic_prediction_confidence_sum;
    // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    candidate.dynamic_analysis_debug = std::move(safety.dynamic_analysis_debug);
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

    const double path_efficiency_penalty =
      std::max(0.0, candidate.path_length_m - candidate.progress_m);

    // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: keep each weighted contribution separately.
    candidate.score_progress = -config_.progress_weight * candidate.progress_m;
    candidate.score_route_deviation =
      config_.route_deviation_weight * candidate.route_deviation_rms_m;
    candidate.score_path_efficiency = config_.path_efficiency_weight * path_efficiency_penalty;
    candidate.score_static_clearance = config_.clearance_weight * safety.static_soft_cost;
    candidate.score_dynamic_margin = config_.dynamic_margin_weight * safety.dynamic_soft_cost;
    candidate.score_comfort = config_.comfort_weight * candidate.comfort_cost;
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
    candidate.score_traffic_rules = candidate.traffic_rule_cost;
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
    candidate.score_lane_boundary = candidate.lane_boundary_cost;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
    candidate.score_lane_change_deadline = candidate.lane_change_deadline_cost;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
    candidate.score = candidate.score_progress + candidate.score_route_deviation +
                      candidate.score_path_efficiency + candidate.score_static_clearance +
                      candidate.score_dynamic_margin + candidate.score_comfort +
                      candidate.score_traffic_rules + candidate.score_lane_boundary +
                      candidate.score_lane_change_deadline;

    // CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_BEGIN
    if (safety.static_argmin.valid) {
      internal_debug::CandidateStaticClearanceLog static_log;
      static_log.candidate_id = candidate.id;
      static_log.source = candidate_source_name(candidate.source);
      static_log.collision_geometry_mode = config_.collision_geometry_mode;
      const auto copy_witness = [&](
                                  const StaticClearanceWitnessDebug & src,
                                  internal_debug::CandidateStaticClearanceWitnessLog & dst) {
        if (!src.valid) return;
        dst.valid = true;
        dst.object_id = src.object_id;
        dst.object_speed_mps = src.object_speed_mps;
        dst.trajectory_point_index = src.trajectory_point_index;
        dst.live_point_index = src.live_point_index;
        dst.future_time_sec = src.future_time_sec;
        dst.signed_clearance_m = src.signed_clearance_m;
        dst.margin_m = src.margin_m;
        dst.deficit_m = src.deficit_m;
        dst.ego_x = src.ego_pose.position.x;
        dst.ego_y = src.ego_pose.position.y;
        dst.ego_yaw = yaw_from_pose(src.ego_pose);
        dst.object_x = src.object_pose.position.x;
        dst.object_y = src.object_pose.position.y;
        dst.object_yaw = yaw_from_pose(src.object_pose);
        dst.ego_radius_m = config_.ego_radius_m;
        dst.ego_polygon.reserve(src.ego_polygon.size());
        for (const auto & point : src.ego_polygon) {
          internal_debug::CandidateObstaclePointLog out;
          out.x = point.x;
          out.y = point.y;
          dst.ego_polygon.push_back(out);
        }
        dst.object_circles.reserve(src.object_geometry.circles.size());
        for (const auto & circle : src.object_geometry.circles) {
          internal_debug::CandidateObstacleCircleLog out;
          out.x = circle.center.x;
          out.y = circle.center.y;
          out.radius_m = circle.radius_m;
          dst.object_circles.push_back(out);
        }
        dst.object_polygon.reserve(src.object_geometry.polygon.size());
        for (const auto & point : src.object_geometry.polygon) {
          internal_debug::CandidateObstaclePointLog out;
          out.x = point.x;
          out.y = point.y;
          dst.object_polygon.push_back(out);
        }
      };
      copy_witness(safety.static_argmin, static_log.argmin);
      copy_witness(safety.static_first_margin_violation, static_log.first_margin_violation);
      copy_witness(safety.static_first_overlap, static_log.first_overlap);
      internal_debug::record_candidate_static_clearance(
        internal_debug::current_cycle_id(), static_log);
    }
    // CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_END

    cb_ct_state("evaluate", candidate, ego, now, tracked_objects, &predicted_objects);
    cb_ct_candidate_detail(candidate, ego, now);
    internal_debug::CandidateEvaluationLog logged;
    logged.candidate_id = candidate.id;
    logged.source = candidate_source_name(candidate.source);
    logged.was_incumbent = incumbent_id_.has_value() && incumbent_id_.value() == candidate.id;
    logged.valid = candidate.valid;
    logged.safe = candidate.safe;
    // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
    logged.static_collision_margin_violated = candidate.static_collision_margin_violated;
    logged.dynamic_collision_margin_violated = candidate.dynamic_collision_margin_violated;
    // CANDIDATE_BANK_SOFT_COLLISION_V1_END
    logged.live_start_index = candidate.live_start_index;
    logged.age_sec = (now - candidate.generation_time).seconds();
    logged.projection_error_m = candidate.projection_error_m;
    logged.remaining_horizon_sec = candidate.remaining_horizon_sec;
    logged.path_length_m = candidate.path_length_m;
    logged.progress_m = candidate.progress_m;
    logged.route_deviation_rms_m = candidate.route_deviation_rms_m;
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
    logged.route_reference_weight_mean = candidate.route_reference_weight_mean;
    logged.route_reference_max_roughness_m = candidate.route_reference_max_roughness_m;
    logged.traffic_signal_stop_active = candidate.traffic_signal_stop_active;
    logged.traffic_signal_crossed = candidate.traffic_signal_crossed;
    logged.traffic_signal_max_stop_zone_speed_mps =
      candidate.traffic_signal_max_stop_zone_speed_mps;
    logged.traffic_rule_cost = candidate.traffic_rule_cost;
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
    logged.traffic_signal_scope_mode = candidate.traffic_signal_scope_mode;
    logged.current_lane_valid = candidate.current_lane_valid;
    logged.current_lane_id = candidate.current_lane_id;
    logged.max_continuous_straddling_sec = candidate.max_continuous_straddling_sec;
    logged.terminal_straddling = candidate.terminal_straddling;
    logged.prolonged_straddling_cost = candidate.prolonged_straddling_cost;
    logged.terminal_straddling_cost = candidate.terminal_straddling_cost;
    logged.lane_boundary_cost = candidate.lane_boundary_cost;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
    logged.lane_change_deadline_active = candidate.lane_change_deadline_active;
    logged.lane_change_attempted = candidate.lane_change_attempted;
    logged.lane_change_completed = candidate.lane_change_completed;
    logged.lane_change_completed_before_deadline = candidate.lane_change_completed_before_deadline;
    logged.lane_change_fallback_stop_satisfied = candidate.lane_change_fallback_stop_satisfied;
    logged.lane_change_boundary_id = candidate.lane_change_boundary_id;
    logged.lane_change_target_lane_id = candidate.lane_change_target_lane_id;
    logged.lane_change_deadline_s_m = candidate.lane_change_deadline_s_m;
    logged.lane_change_completion_s_m = candidate.lane_change_completion_s_m;
    logged.lane_change_slack_m = candidate.lane_change_slack_m;
    logged.lane_change_deadline_cost = candidate.lane_change_deadline_cost;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
    logged.comfort_cost = candidate.comfort_cost;
    logged.min_static_clearance_m = candidate.min_static_clearance_m;
    logged.min_dynamic_clearance_m = candidate.min_dynamic_clearance_m;
    logged.min_dynamic_time_gap_sec = candidate.min_dynamic_time_gap_sec;
    // CANDIDATE_BANK_TTC_COST_V1_BEGIN
    logged.dynamic_temporal_cost_mode = config_.dynamic_temporal_cost_mode;
    logged.min_dynamic_ttc_sec = candidate.min_dynamic_ttc_sec;
    logged.dynamic_clearance_soft_cost = candidate.dynamic_clearance_soft_cost;
    logged.dynamic_legacy_time_gap_soft_cost = candidate.dynamic_legacy_time_gap_soft_cost;
    logged.dynamic_ttc_soft_cost = candidate.dynamic_ttc_soft_cost;
    // CANDIDATE_BANK_TTC_COST_V1_END
    // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
    logged.dynamic_soft_cost_aggregation_mode = config_.dynamic_soft_cost_aggregation_mode;
    logged.dynamic_uncertainty_mode = config_.dynamic_uncertainty_mode;
    logged.max_dynamic_closing_rate_mps = candidate.max_dynamic_closing_rate_mps;
    logged.max_dynamic_uncertainty_margin_m = candidate.max_dynamic_uncertainty_margin_m;
    logged.dynamic_prediction_path_count = candidate.dynamic_prediction_path_count;
    logged.dynamic_prediction_confidence_sum = candidate.dynamic_prediction_confidence_sum;
    // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
    logged.score = candidate.score;
    logged.score_progress = candidate.score_progress;
    logged.score_route_deviation = candidate.score_route_deviation;
    logged.score_path_efficiency = candidate.score_path_efficiency;
    logged.score_static_clearance = candidate.score_static_clearance;
    logged.score_dynamic_margin = candidate.score_dynamic_margin;
    logged.score_comfort = candidate.score_comfort;
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
    logged.score_traffic_rules = candidate.score_traffic_rules;
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
    logged.score_lane_boundary = candidate.score_lane_boundary;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
    logged.score_lane_change_deadline = candidate.score_lane_change_deadline;
    // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
    // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
    // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
    internal_debug::record_candidate_evaluation(internal_debug::current_cycle_id(), logged);
    // CANDIDATE_BANK_DIAGNOSTICS_V2_END
  }

  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
  // Finalize only after the baseline evaluator has written this cycle's score/safe.
  // This preserves the original score decomposition while still letting select()
  // use an effective CoPlanner-style score/eligibility state.
  finalize_and_log_cross_aggregates(candidates_, cross_aggregate_results_, config_);
  for (const auto & aggregate : cross_aggregate_results_) cb_ct_cross_aggregate(aggregate, now);
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END

  refresh_debug_info();
}

std::optional<Trajectory> TrajectoryCandidateManager::select(const rclcpp::Time & now)
{
  if (!selection_enabled()) {
    return std::nullopt;
  }

  // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN: capture selection cause before changing incumbent_id_.
  const std::uint64_t previous_selected_id = incumbent_id_.value_or(0);
  debug_info_.previous_selected_candidate_id = previous_selected_id;
  debug_info_.selection_switched = false;
  debug_info_.hysteresis_kept = false;
  debug_info_.selection_reason = "best";
  // CANDIDATE_BANK_DIAGNOSTICS_V2_END

  auto best = candidates_.end();
  for (auto it = candidates_.begin(); it != candidates_.end(); ++it) {
    if (!selector_effective_eligible(*it, cross_aggregate_results_, config_)) continue;
    if (
      best == candidates_.end() ||
      candidate_is_preferred(*it, *best, cross_aggregate_results_, config_)) {
      best = it;
    }
  }
  if (best == candidates_.end()) {
    incumbent_id_.reset();
    if (config_.safety_tier_enabled) {
      debug_info_.selection_reason = "no_valid_candidate";
    } else {
      const bool has_baseline_safe_candidate = std::any_of(
        candidates_.begin(), candidates_.end(),
        [](const TrajectoryCandidate & candidate) { return candidate.valid && candidate.safe; });
      debug_info_.selection_reason = has_baseline_safe_candidate &&
                                         config_.cross_aggregation_enabled &&
                                         config_.cross_aggregation_apply_short_horizon_hard_reject
                                       ? "no_contingency_eligible_candidate"
                                       : "no_safe_candidate";
    }
    debug_info_.selection_switched = previous_selected_id != 0;
    internal_debug::record_candidate_selection(
      internal_debug::current_cycle_id(), previous_selected_id, 0, 0,
      debug_info_.selection_switched, false, debug_info_.selection_reason,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
      config_.safety_tier_enabled ? "NONE" : "DISABLED",
      config_.safety_tier_enabled ? "NONE" : "DISABLED");
    const auto highest_progress = highest_progress_candidate(candidates_);
    if (highest_progress != candidates_.end() && highest_progress->valid) {
      record_dynamic_analysis_candidate(
        internal_debug::current_cycle_id(), *highest_progress, "highest_progress", config_);
    }
    refresh_debug_info();
    return std::nullopt;
  }

  const auto best_assessment = assess_candidate_safety(*best, cross_aggregate_results_, config_);
  if (config_.safety_tier_enabled) {
    if (best_assessment.tier == CandidateSafetyTier::SAFE) {
      debug_info_.selection_reason = "safety_tier_safe_best";
    } else if (best_assessment.tier == CandidateSafetyTier::CAUTION) {
      debug_info_.selection_reason = "safety_tier_caution_best";
    } else {
      debug_info_.selection_reason = "safety_tier_unsafe_fallback";
    }
  }

  auto selected = best;
  if (config_.hysteresis_enabled && incumbent_id_.has_value()) {
    auto incumbent = std::find_if(
      candidates_.begin(), candidates_.end(),
      [&](const TrajectoryCandidate & candidate) { return candidate.id == incumbent_id_.value(); });
    if (
      incumbent != candidates_.end() &&
      selector_effective_eligible(*incumbent, cross_aggregate_results_, config_)) {
      if (config_.safety_tier_enabled) {
        const auto incumbent_assessment =
          assess_candidate_safety(*incumbent, cross_aggregate_results_, config_);
        if (best_assessment.tier < incumbent_assessment.tier) {
          // A safer tier always wins immediately; switch_margin must never retain a less-safe path.
          debug_info_.selection_reason = "safety_tier_immediate_upgrade";
        } else if (
          best_assessment.tier == CandidateSafetyTier::SAFE &&
          incumbent_assessment.tier == CandidateSafetyTier::SAFE) {
          const double best_effective_score =
            selector_effective_score(*best, cross_aggregate_results_, config_);
          const double incumbent_effective_score =
            selector_effective_score(*incumbent, cross_aggregate_results_, config_);
          const bool meaningfully_better =
            best_effective_score < incumbent_effective_score - config_.switch_margin;
          if (!meaningfully_better) {
            selected = incumbent;
            if (selected->id != best->id) {
              debug_info_.hysteresis_kept = true;
              debug_info_.selection_reason = "safety_tier_safe_hysteresis_keep";
            } else {
              debug_info_.selection_reason = "safety_tier_safe_incumbent_is_best";
            }
          }
        } else {
          // CAUTION/UNSAFE deliberately do not use score hysteresis.  The relative-safety
          // comparator is allowed to switch immediately to a safer fallback within the tier.
          if (best->id == incumbent->id) {
            debug_info_.selection_reason = best_assessment.tier == CandidateSafetyTier::CAUTION
                                             ? "safety_tier_caution_incumbent_best"
                                             : "safety_tier_unsafe_incumbent_best";
          }
        }
      } else {
        const double best_effective_score =
          selector_effective_score(*best, cross_aggregate_results_, config_);
        const double incumbent_effective_score =
          selector_effective_score(*incumbent, cross_aggregate_results_, config_);
        const bool meaningfully_better =
          best_effective_score < incumbent_effective_score - config_.switch_margin;
        if (!meaningfully_better) {
          selected = incumbent;
          if (selected->id != best->id) {
            debug_info_.hysteresis_kept = true;
            debug_info_.selection_reason = "hysteresis_keep";
          } else {
            debug_info_.selection_reason = "incumbent_is_best";
          }
        }
      }
    } else if (!config_.safety_tier_enabled) {
      debug_info_.selection_reason = incumbent != candidates_.end() && incumbent->valid &&
                                         incumbent->safe && config_.cross_aggregation_enabled &&
                                         config_.cross_aggregation_apply_short_horizon_hard_reject
                                       ? "incumbent_contingency_rejected"
                                       : "incumbent_invalid_or_unsafe";
    }
  }

  for (auto & candidate : candidates_) {
    candidate.selected = false;
  }
  selected->selected = true;
  incumbent_id_ = selected->id;

  // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN
  debug_info_.selection_switched = previous_selected_id != selected->id;
  const auto selected_assessment =
    assess_candidate_safety(*selected, cross_aggregate_results_, config_);
  internal_debug::record_candidate_selection(
    internal_debug::current_cycle_id(), previous_selected_id, best->id, selected->id,
    debug_info_.selection_switched, debug_info_.hysteresis_kept, debug_info_.selection_reason,
    selector_effective_score(*best, cross_aggregate_results_, config_),
    selector_effective_score(*selected, cross_aggregate_results_, config_),
    config_.safety_tier_enabled ? safety_tier_name(best_assessment.tier) : "DISABLED",
    config_.safety_tier_enabled ? safety_tier_name(selected_assessment.tier) : "DISABLED");
  // CANDIDATE_BANK_DIAGNOSTICS_V2_END

  // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
  // Log only two detailed traces per cycle to keep runtime logging bounded: the trajectory that
  // will actually be executed and the valid trajectory with the largest geometric progress.
  record_dynamic_analysis_candidate(
    internal_debug::current_cycle_id(), *selected, "selected", config_);
  const auto highest_progress = highest_progress_candidate(candidates_);
  if (
    highest_progress != candidates_.end() && highest_progress->valid &&
    highest_progress->id != selected->id) {
    record_dynamic_analysis_candidate(
      internal_debug::current_cycle_id(), *highest_progress, "highest_progress", config_);
  }
  // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

  cb_ct_select(*selected, *best, debug_info_, now);

  refresh_debug_info();

  return make_live_suffix(selected->trajectory_map, selected->live_start_index, now);
}

std::optional<Trajectory> TrajectoryCandidateManager::incumbent() const
{
  if (!incumbent_id_.has_value()) {
    return std::nullopt;
  }
  const auto it = std::find_if(
    candidates_.begin(), candidates_.end(),
    [&](const TrajectoryCandidate & candidate) { return candidate.id == incumbent_id_.value(); });
  if (it == candidates_.end()) {
    return std::nullopt;
  }
  return it->trajectory_map;
}

void TrajectoryCandidateManager::enforce_size_limit()
{
  while (candidates_.size() > config_.max_size) {
    auto oldest = candidates_.end();
    for (auto it = candidates_.begin(); it != candidates_.end(); ++it) {
      if (incumbent_id_.has_value() && it->id == incumbent_id_.value()) {
        continue;
      }
      if (oldest == candidates_.end() || it->generation_time < oldest->generation_time) {
        oldest = it;
      }
    }
    if (oldest == candidates_.end()) {
      break;
    }
    // CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN
    internal_debug::record_candidate_purged(
      internal_debug::current_cycle_id(), oldest->id, candidate_source_name(oldest->source),
      "bank_size_limit", 0.0, oldest->projection_error_m, oldest->remaining_horizon_sec);
    // CANDIDATE_BANK_DIAGNOSTICS_V2_END
    candidates_.erase(oldest);
  }
}

void TrajectoryCandidateManager::refresh_debug_info()
{
  debug_info_.mode = config_.mode;
  debug_info_.collision_geometry_mode = config_.collision_geometry_mode;
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
  debug_info_.route_deviation_adaptive_weight_enabled =
    config_.route_deviation_adaptive_weight_enabled;
  debug_info_.traffic_signal_rule_enabled = config_.traffic_signal_rule_enabled;
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
  // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
  debug_info_.static_hard_reject_enabled = config_.static_hard_reject_enabled;
  debug_info_.dynamic_hard_reject_enabled = config_.dynamic_hard_reject_enabled;
  // CANDIDATE_BANK_SOFT_COLLISION_V1_END
  debug_info_.bank_size = candidates_.size();
  debug_info_.safe_count = static_cast<std::size_t>(std::count_if(
    candidates_.begin(), candidates_.end(),
    [](const TrajectoryCandidate & candidate) { return candidate.valid && candidate.safe; }));
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
  debug_info_.safety_tier_enabled = config_.safety_tier_enabled;
  debug_info_.safety_tier_safe_count = 0;
  debug_info_.safety_tier_caution_count = 0;
  debug_info_.safety_tier_unsafe_count = 0;
  debug_info_.best_safety_tier = config_.safety_tier_enabled ? "NONE" : "DISABLED";
  debug_info_.selected_safety_tier = config_.safety_tier_enabled ? "NONE" : "DISABLED";
  if (config_.safety_tier_enabled) {
    for (const auto & candidate : candidates_) {
      if (!candidate.valid) continue;
      const auto assessment = assess_candidate_safety(candidate, cross_aggregate_results_, config_);
      if (assessment.tier == CandidateSafetyTier::SAFE)
        ++debug_info_.safety_tier_safe_count;
      else if (assessment.tier == CandidateSafetyTier::CAUTION)
        ++debug_info_.safety_tier_caution_count;
      else
        ++debug_info_.safety_tier_unsafe_count;
    }
  }
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END

  debug_info_.best_candidate_id = 0;
  debug_info_.best_score = std::numeric_limits<double>::infinity();
  debug_info_.best_source = "none";
  const TrajectoryCandidate * debug_best = nullptr;
  for (const auto & candidate : candidates_) {
    if (!selector_effective_eligible(candidate, cross_aggregate_results_, config_)) continue;
    if (
      debug_best == nullptr ||
      candidate_is_preferred(candidate, *debug_best, cross_aggregate_results_, config_)) {
      debug_best = &candidate;
    }
  }
  if (debug_best != nullptr) {
    debug_info_.best_score =
      selector_effective_score(*debug_best, cross_aggregate_results_, config_);
    debug_info_.best_candidate_id = debug_best->id;
    debug_info_.best_source = candidate_source_name(debug_best->source);
    if (config_.safety_tier_enabled) {
      debug_info_.best_safety_tier = safety_tier_name(
        assess_candidate_safety(*debug_best, cross_aggregate_results_, config_).tier);
    }
  }

  debug_info_.selected_candidate_id = 0;
  debug_info_.selected_score = std::numeric_limits<double>::infinity();
  debug_info_.selected_source = "none";
  debug_info_.selected_static_clearance_m = std::numeric_limits<double>::infinity();
  debug_info_.selected_dynamic_clearance_m = std::numeric_limits<double>::infinity();
  debug_info_.selected_dynamic_time_gap_sec = std::numeric_limits<double>::infinity();
  // CANDIDATE_BANK_TTC_COST_V1_BEGIN
  debug_info_.dynamic_temporal_cost_mode = config_.dynamic_temporal_cost_mode;
  debug_info_.selected_dynamic_ttc_sec = std::numeric_limits<double>::infinity();
  debug_info_.selected_dynamic_clearance_soft_cost = 0.0;
  debug_info_.selected_dynamic_legacy_time_gap_soft_cost = 0.0;
  debug_info_.selected_dynamic_ttc_soft_cost = 0.0;
  // CANDIDATE_BANK_TTC_COST_V1_END
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
  debug_info_.dynamic_soft_cost_aggregation_mode = config_.dynamic_soft_cost_aggregation_mode;
  debug_info_.dynamic_uncertainty_mode = config_.dynamic_uncertainty_mode;
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
  debug_info_.dynamic_scope_mode = config_.dynamic_scope_mode;
  // CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END
  debug_info_.selected_max_dynamic_closing_rate_mps = 0.0;
  debug_info_.selected_max_dynamic_uncertainty_margin_m = 0.0;
  debug_info_.selected_dynamic_prediction_path_count = 0;
  debug_info_.selected_dynamic_prediction_confidence_sum = 0.0;
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
  debug_info_.selected_route_reference_weight_mean = 1.0;
  debug_info_.selected_route_reference_max_roughness_m = 0.0;
  debug_info_.selected_traffic_signal_stop_active = false;
  debug_info_.selected_traffic_signal_crossed = false;
  debug_info_.selected_traffic_rule_cost = 0.0;
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
  debug_info_.current_lane_valid = false;
  debug_info_.current_lane_id = 0;
  debug_info_.selected_max_continuous_straddling_sec = 0.0;
  debug_info_.selected_terminal_straddling = false;
  debug_info_.selected_lane_boundary_cost = 0.0;
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
  debug_info_.lane_change_deadline_enabled = config_.lane_change_deadline_enabled;
  debug_info_.lane_change_deadline_activation_mode = config_.lane_change_deadline_activation_mode;
  debug_info_.selected_lane_change_deadline_active = false;
  debug_info_.selected_lane_change_completed_before_deadline = false;
  debug_info_.selected_lane_change_fallback_stop_satisfied = false;
  debug_info_.selected_lane_change_deadline_cost = 0.0;
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
  // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
  debug_info_.selected_static_collision_margin_violated = false;
  debug_info_.selected_dynamic_collision_margin_violated = false;
  // CANDIDATE_BANK_SOFT_COLLISION_V1_END
  if (incumbent_id_.has_value()) {
    const auto incumbent = std::find_if(
      candidates_.begin(), candidates_.end(),
      [&](const TrajectoryCandidate & candidate) { return candidate.id == incumbent_id_.value(); });
    if (incumbent != candidates_.end()) {
      debug_info_.selected_candidate_id = incumbent->id;
      // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
      debug_info_.selected_score =
        selector_effective_score(*incumbent, cross_aggregate_results_, config_);
      // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
      debug_info_.selected_source = candidate_source_name(incumbent->source);
      // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
      if (config_.safety_tier_enabled) {
        debug_info_.selected_safety_tier = safety_tier_name(
          assess_candidate_safety(*incumbent, cross_aggregate_results_, config_).tier);
      }
      // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
      debug_info_.selected_static_clearance_m = incumbent->min_static_clearance_m;
      debug_info_.selected_dynamic_clearance_m = incumbent->min_dynamic_clearance_m;
      debug_info_.selected_dynamic_time_gap_sec = incumbent->min_dynamic_time_gap_sec;
      // CANDIDATE_BANK_TTC_COST_V1_BEGIN
      debug_info_.selected_dynamic_ttc_sec = incumbent->min_dynamic_ttc_sec;
      debug_info_.selected_dynamic_clearance_soft_cost = incumbent->dynamic_clearance_soft_cost;
      debug_info_.selected_dynamic_legacy_time_gap_soft_cost =
        incumbent->dynamic_legacy_time_gap_soft_cost;
      debug_info_.selected_dynamic_ttc_soft_cost = incumbent->dynamic_ttc_soft_cost;
      // CANDIDATE_BANK_TTC_COST_V1_END
      // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
      debug_info_.selected_max_dynamic_closing_rate_mps = incumbent->max_dynamic_closing_rate_mps;
      debug_info_.selected_max_dynamic_uncertainty_margin_m =
        incumbent->max_dynamic_uncertainty_margin_m;
      debug_info_.selected_dynamic_prediction_path_count = incumbent->dynamic_prediction_path_count;
      debug_info_.selected_dynamic_prediction_confidence_sum =
        incumbent->dynamic_prediction_confidence_sum;
      // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
      // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
      debug_info_.selected_route_reference_weight_mean = incumbent->route_reference_weight_mean;
      debug_info_.selected_route_reference_max_roughness_m =
        incumbent->route_reference_max_roughness_m;
      debug_info_.selected_traffic_signal_stop_active = incumbent->traffic_signal_stop_active;
      debug_info_.selected_traffic_signal_crossed = incumbent->traffic_signal_crossed;
      debug_info_.selected_traffic_rule_cost = incumbent->traffic_rule_cost;
      // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
      debug_info_.current_lane_valid = incumbent->current_lane_valid;
      debug_info_.current_lane_id = incumbent->current_lane_id;
      debug_info_.selected_max_continuous_straddling_sec = incumbent->max_continuous_straddling_sec;
      debug_info_.selected_terminal_straddling = incumbent->terminal_straddling;
      debug_info_.selected_lane_boundary_cost = incumbent->lane_boundary_cost;
      // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
      debug_info_.selected_lane_change_deadline_active = incumbent->lane_change_deadline_active;
      debug_info_.selected_lane_change_completed_before_deadline =
        incumbent->lane_change_completed_before_deadline;
      debug_info_.selected_lane_change_fallback_stop_satisfied =
        incumbent->lane_change_fallback_stop_satisfied;
      debug_info_.selected_lane_change_deadline_cost = incumbent->lane_change_deadline_cost;
      // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
      // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
      // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
      // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
      debug_info_.selected_static_collision_margin_violated =
        incumbent->static_collision_margin_violated;
      debug_info_.selected_dynamic_collision_margin_violated =
        incumbent->dynamic_collision_margin_violated;
      // CANDIDATE_BANK_SOFT_COLLISION_V1_END
    }
  }
}

}  // namespace autoware::candidate_bank

// CANDIDATE_BANK_EXPERIMENT_V1_END: concrete staged evaluation implementation.
