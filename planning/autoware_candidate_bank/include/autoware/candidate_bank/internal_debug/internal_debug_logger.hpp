// Copyright 2026 Contributor
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
#ifndef AUTOWARE__CANDIDATE_BANK__INTERNAL_DEBUG__INTERNAL_DEBUG_LOGGER_HPP_
#define AUTOWARE__CANDIDATE_BANK__INTERNAL_DEBUG__INTERNAL_DEBUG_LOGGER_HPP_
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>
namespace autoware::candidate_bank::internal_debug
{
using NamedFloatArrays = std::vector<std::pair<std::string, std::vector<float>>>;
// scalar records keep per-cycle logging compact.
struct CandidateObstacleCircleLog
{
  double x{0.0};
  double y{0.0};
  double radius_m{0.0};
};
struct CandidateObstaclePointLog
{
  double x{0.0};
  double y{0.0};
};

struct CandidateObstacleLog
{
  std::string object_id;
  double x{0.0};
  double y{0.0};
  // `radius_m` is retained for backward-compatible log consumers. New code
  // should prefer collision_circles/collision_polygon, which represent the
  // geometry actually used by the selected collision_geometry mode.
  double radius_m{0.0};
  double yaw{0.0};
  double length_m{0.0};
  double width_m{0.0};
  int shape_type{0};
  std::vector<CandidateObstacleCircleLog> collision_circles;
  std::vector<CandidateObstaclePointLog> collision_polygon;
  double speed_mps{0.0};
  bool is_static{false};
};
// Exact witnesses from the baseline static-clearance evaluator.  These are
// diagnostics only: they never participate in score/safety/selection.
struct CandidateStaticClearanceWitnessLog
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
  double ego_x{0.0};
  double ego_y{0.0};
  double ego_yaw{0.0};
  double object_x{0.0};
  double object_y{0.0};
  double object_yaw{0.0};
  double ego_radius_m{0.0};
  std::vector<CandidateObstaclePointLog> ego_polygon;
  std::vector<CandidateObstacleCircleLog> object_circles;
  std::vector<CandidateObstaclePointLog> object_polygon;
};

struct CandidateStaticClearanceLog
{
  std::uint64_t candidate_id{0};
  std::string source;
  std::string collision_geometry_mode;
  CandidateStaticClearanceWitnessLog argmin;
  CandidateStaticClearanceWitnessLog first_margin_violation;
  CandidateStaticClearanceWitnessLog first_overlap;
};
struct CandidateDynamicSampleLog
{
  double time_sec{0.0};
  double ego_x{0.0};
  double ego_y{0.0};
  double ego_yaw{0.0};
  double object_x{0.0};
  double object_y{0.0};
  double object_yaw{0.0};
  double geometric_clearance_m{0.0};
  double closing_rate_mps{0.0};
  double uncertainty_margin_m{0.0};
  double evaluated_clearance_m{0.0};
  double clearance_soft_cost{0.0};
  bool collision_margin_violated{false};
};
struct CandidateLaneHypothesisLog
{
  std::int64_t lane_id{0};
  bool is_route_lane{false};
  double s_m{0.0};
  double confidence{0.0};
  double lateral_distance_m{0.0};
  double heading_error_deg{0.0};
  double predicted_path_fit_m{std::numeric_limits<double>::quiet_NaN()};
};

struct CandidateMapConflictDecisionLog
{
  std::string object_id;
  bool full_dynamic_evaluation{false};
  bool immediate_guard_only{false};
  bool immediate_guard_violated{false};
  std::string reason;
  bool lane_associated{false};
  std::vector<CandidateLaneHypothesisLog> lane_hypotheses;
  double motion_heading_yaw{0.0};
  double motion_heading_reliability{0.0};
  std::int64_t best_lane_id{0};
  double best_lane_confidence{0.0};
  double best_lane_lateral_distance_m{0.0};
  double best_lane_heading_error_deg{0.0};
  double best_lane_predicted_path_fit_m{std::numeric_limits<double>::quiet_NaN()};
  std::string motion_heading_source;
  bool lane_association_ambiguous{true};
  std::int64_t conflict_ego_lane_id{0};
  std::int64_t conflict_competing_lane_id{0};
  double conflict_x_map{std::numeric_limits<double>::quiet_NaN()};
  double conflict_y_map{std::numeric_limits<double>::quiet_NaN()};
  double object_distance_to_conflict_m{0.0};
  double ego_arrival_time_sec{0.0};
  double object_arrival_time_sec{0.0};
};
struct CandidateMapContextPointLog
{
  double x{0.0};
  double y{0.0};
};

struct CandidateMapContextLaneLog
{
  std::int64_t lane_id{0};
  bool is_route_lane{false};
  bool is_competing_lane{false};
  double route_s_start_m{0.0};
  std::vector<CandidateMapContextPointLog> centerline;
};

struct CandidateMapContextConflictLog
{
  std::int64_t ego_route_lane_id{0};
  std::int64_t competing_lane_id{0};
  double x{0.0};
  double y{0.0};
  double ego_route_distance_m{0.0};
  std::vector<std::int64_t> approach_lane_ids;
};

struct CandidateMapContextLog
{
  bool valid{false};
  double forward_search_distance_m{0.0};
  double backtrace_distance_m{0.0};
  std::vector<CandidateMapContextLaneLog> lanes;
  std::vector<CandidateMapContextConflictLog> conflicts;
};

struct CandidateDynamicAnalysisLog
{
  std::string role;  // selected | highest_progress
  std::uint64_t candidate_id{0};
  std::string source;
  bool safe{false};
  double progress_m{0.0};
  double score{0.0};
  double score_dynamic_margin{0.0};
  double candidate_dynamic_clearance_soft_cost{0.0};
  double candidate_dynamic_ttc_soft_cost{0.0};
  double candidate_min_dynamic_clearance_m{0.0};
  double candidate_min_dynamic_ttc_sec{0.0};
  double collision_margin_m{0.0};
  double preferred_clearance_m{0.0};
  double ttc_threshold_sec{0.0};

  bool detail_valid{false};
  std::string object_id;
  int path_index{-1};
  bool is_fallback{false};
  double path_confidence{0.0};
  double normalized_confidence{0.0};
  double path_clearance_soft_cost{0.0};
  double path_ttc_soft_cost{0.0};
  double path_total_soft_cost{0.0};
  double ttc_sec{0.0};
  bool collision_margin_violated{false};
  std::size_t worst_sample_index{0};
  std::vector<CandidateDynamicSampleLog> samples;
  std::string dynamic_scope_mode{"all"};
  std::vector<CandidateMapConflictDecisionLog> map_filter_decisions;
};
struct CandidateCrossCollisionLog
{
  std::uint64_t ego_candidate_id{0};
  std::uint64_t scenario_candidate_id{0};
  std::string ego_source;
  std::string scenario_source;
  bool paired{false};
  double scenario_age_sec{0.0};
  bool valid{false};
  bool same_time_collision{false};
  bool xyt_conflict{false};
  bool trajectory_conflict{false};
  double first_collision_time_sec{std::numeric_limits<double>::infinity()};
  std::string first_collision_object_id;
  int first_collision_path_index{-1};
  double first_collision_path_confidence{0.0};
  int first_collision_ego_circle_index{-1};
  int first_collision_object_circle_index{-1};
  double scenario_weight{1.0};
  double scenario_risk{0.0};
  double min_same_time_clearance_m{std::numeric_limits<double>::infinity()};
  double min_xyt_rho{std::numeric_limits<double>::infinity()};
  double xyt_spatial_distance_m{std::numeric_limits<double>::infinity()};
  double xyt_spatial_clearance_m{std::numeric_limits<double>::infinity()};
  double xyt_ego_time_sec{0.0};
  double xyt_object_time_sec{0.0};
  double xyt_delta_time_sec{0.0};
  std::string object_id;
  int path_index{-1};
  double path_confidence{0.0};
  int ego_circle_index{-1};
  int object_circle_index{-1};
  std::size_t valid_path_count{0};
  std::uint64_t path_tests{0};
  std::uint64_t path_rejects{0};
  std::uint64_t block_tests{0};
  std::uint64_t block_rejects{0};
  std::uint64_t interval_tests{0};
  std::uint64_t interval_rejects{0};
  std::uint64_t circle_aabb_tests{0};
  std::uint64_t circle_aabb_rejects{0};
  std::uint64_t xyt_segment_tests{0};
  std::uint64_t same_time_interval_tests{0};
  std::uint64_t same_time_circle_tests{0};
  double evaluation_us{0.0};
};
struct CandidateCrossAggregateLog
{
  std::uint64_t candidate_id{0};
  std::string source;
  std::string weight_mode{"uniform"};
  bool aggregation_enabled{false};
  bool apply_to_score{false};
  bool apply_short_horizon_hard_reject{false};
  double score_weight{1.0};
  double short_horizon_sec{1.5};
  std::size_t scenario_count{0};
  std::size_t valid_scenario_count{0};
  std::size_t collision_scenario_count{0};
  std::size_t xyt_conflict_scenario_count{0};
  std::size_t short_horizon_collision_scenario_count{0};
  double scenario_weight_sum{0.0};
  double collision_scenario_fraction{0.0};
  double xyt_conflict_scenario_fraction{0.0};
  double min_toi_sec{std::numeric_limits<double>::infinity()};
  double mean_collision_toi_sec{std::numeric_limits<double>::infinity()};
  double min_same_time_clearance_m{std::numeric_limits<double>::infinity()};
  double paired_risk{std::numeric_limits<double>::infinity()};
  double off_diagonal_mean_risk{std::numeric_limits<double>::infinity()};
  double mean_scenario_risk{0.0};
  double score_before_cross{std::numeric_limits<double>::infinity()};
  double score_penalty{0.0};
  double score_after_cross{std::numeric_limits<double>::infinity()};
  double selector_effective_score{std::numeric_limits<double>::infinity()};
  bool base_safe{false};
  bool short_horizon_hard_reject{false};
  bool selector_score_enabled{false};
  bool selector_hard_rejected{false};
  bool selector_eligible{false};
  std::string safety_tier{"DISABLED"};
  bool shadow_best{false};
};

struct CandidateEvaluationLog
{
  std::uint64_t candidate_id{0};
  std::string source;
  bool was_incumbent{false};
  bool valid{false};
  bool safe{false};
  bool static_collision_margin_violated{false};
  bool dynamic_collision_margin_violated{false};
  std::size_t live_start_index{0};
  double age_sec{0.0};
  double projection_error_m{0.0};
  double remaining_horizon_sec{0.0};
  double path_length_m{0.0};
  double progress_m{0.0};
  double route_deviation_rms_m{0.0};
  double route_reference_weight_mean{1.0};
  double route_reference_max_roughness_m{0.0};
  bool traffic_signal_stop_active{false};
  bool traffic_signal_crossed{false};
  double traffic_signal_max_stop_zone_speed_mps{0.0};
  double traffic_rule_cost{0.0};
  std::string traffic_signal_scope_mode{"route_nearest"};
  bool current_lane_valid{false};
  std::int64_t current_lane_id{0};
  double max_continuous_straddling_sec{0.0};
  bool terminal_straddling{false};
  double prolonged_straddling_cost{0.0};
  double terminal_straddling_cost{0.0};
  double lane_boundary_cost{0.0};
  bool lane_change_deadline_active{false};
  bool lane_change_attempted{false};
  bool lane_change_completed{false};
  bool lane_change_completed_before_deadline{false};
  bool lane_change_fallback_stop_satisfied{false};
  std::int64_t lane_change_boundary_id{0};
  std::int64_t lane_change_target_lane_id{0};
  double lane_change_deadline_s_m{0.0};
  double lane_change_completion_s_m{0.0};
  double lane_change_slack_m{0.0};
  double lane_change_deadline_cost{0.0};
  double comfort_cost{0.0};
  double min_static_clearance_m{0.0};
  double min_dynamic_clearance_m{0.0};
  double min_dynamic_time_gap_sec{0.0};
  std::string dynamic_temporal_cost_mode{"ttc"};
  double min_dynamic_ttc_sec{0.0};
  double dynamic_clearance_soft_cost{0.0};
  double dynamic_legacy_time_gap_soft_cost{0.0};
  double dynamic_ttc_soft_cost{0.0};
  std::string dynamic_soft_cost_aggregation_mode{"normalized_path_max"};
  std::string dynamic_uncertainty_mode{"closing_rate"};
  double max_dynamic_closing_rate_mps{0.0};
  double max_dynamic_uncertainty_margin_m{0.0};
  std::size_t dynamic_prediction_path_count{0};
  double dynamic_prediction_confidence_sum{0.0};
  double score{0.0};
  double score_progress{0.0};
  double score_route_deviation{0.0};
  double score_path_efficiency{0.0};
  double score_static_clearance{0.0};
  double score_dynamic_margin{0.0};
  double score_comfort{0.0};
  double score_traffic_rules{0.0};
  double score_lane_boundary{0.0};
  double score_lane_change_deadline{0.0};
};

struct LoggerConfig
{
  bool enabled{false};
  std::string output_root{"/tmp/candidate_bank_internal_debug"};
  std::string run_name{"auto"};
  std::string output_directory{};
  int stage_stride{1};
  std::size_t max_array_values{0};
  bool write_effective_parameters{true};
};
void configure(const LoggerConfig & requested_config);
const LoggerConfig & config();
void write_effective_parameters(const std::string & yaml_text);
std::uint64_t begin_cycle();
std::uint64_t current_cycle_id();
void record_input_map(
  std::uint64_t, const NamedFloatArrays &, const std::vector<float> &, const std::vector<float> &);
void record_solver_initial(std::uint64_t, const std::vector<float> &, float, int);
void record_solver_stage(std::uint64_t, int, float, const std::vector<float> &);
void record_solver_final(std::uint64_t, const std::vector<float> &);
void record_note(std::uint64_t, const std::string &, const std::string &);
void record_mid_replacement(
  std::uint64_t cycle_id, const std::string & configured_mode, bool applied, float configured_t,
  float actual_t, float beta, const std::string & skip_reason);
void record_candidate_generated(
  std::uint64_t cycle_id, std::uint64_t candidate_id, const std::string & source,
  const autoware_planning_msgs::msg::Trajectory & trajectory_map);
void record_candidate_joint_prediction(
  std::uint64_t cycle_id, std::uint64_t candidate_id, const std::string & source,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  std::int64_t batch_index);
void record_candidate_cross_collision(
  std::uint64_t cycle_id, const CandidateCrossCollisionLog & collision);
void record_candidate_cross_aggregate(
  std::uint64_t cycle_id, const CandidateCrossAggregateLog & aggregate);
void record_candidate_observation(
  std::uint64_t cycle_id, double ego_x, double ego_y, double ego_yaw,
  const std::string & collision_geometry_mode,
  const std::vector<CandidateObstaclePointLog> & ego_polygon,
  const std::vector<CandidateObstacleLog> & obstacles);
void record_candidate_map_context(std::uint64_t cycle_id, const CandidateMapContextLog & context);
void record_candidate_static_clearance(
  std::uint64_t cycle_id, const CandidateStaticClearanceLog & clearance);
void record_candidate_evaluation(std::uint64_t cycle_id, const CandidateEvaluationLog & evaluation);
void record_candidate_dynamic_analysis(
  std::uint64_t cycle_id, const CandidateDynamicAnalysisLog & analysis);
void record_candidate_purged(
  std::uint64_t cycle_id, std::uint64_t candidate_id, const std::string & source,
  const std::string & reason, double age_sec, double projection_error_m,
  double remaining_horizon_sec);
void record_candidate_selection(
  std::uint64_t cycle_id, std::uint64_t previous_selected_id, std::uint64_t best_id,
  std::uint64_t selected_id, bool switched, bool hysteresis_kept, const std::string & reason,
  double best_score, double selected_score, const std::string & best_safety_tier,
  const std::string & selected_safety_tier);
void record_candidate_seed_selection(
  std::uint64_t cycle_id, const std::string & seed_mode, const std::string & candidate_source,
  const std::string & effective_reference_source);

// record what is actually handed to CandidateTrajectories.
void record_candidate_output_handoff(
  std::uint64_t cycle_id, bool bridge_applied, std::uint64_t selected_id,
  const std::string & selected_source, const std::string & reason,
  const autoware_planning_msgs::msg::Trajectory & downstream_trajectory,
  std::size_t downstream_candidate_count);
}  // namespace autoware::candidate_bank::internal_debug
#endif  // AUTOWARE__CANDIDATE_BANK__INTERNAL_DEBUG__INTERNAL_DEBUG_LOGGER_HPP_
