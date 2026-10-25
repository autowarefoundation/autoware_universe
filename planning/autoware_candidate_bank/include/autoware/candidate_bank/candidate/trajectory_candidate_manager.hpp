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

#ifndef AUTOWARE__CANDIDATE_BANK__CANDIDATE__TRAJECTORY_CANDIDATE_MANAGER_HPP_
#define AUTOWARE__CANDIDATE_BANK__CANDIDATE__TRAJECTORY_CANDIDATE_MANAGER_HPP_

// persistent map-frame trajectory bank and evaluator.

#include "autoware/candidate_bank/candidate/map_conflict_context.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::candidate_bank
{

enum class CandidateSource {
  LEGACY_OUTPUT,
  INCUMBENT_SEED,
  ROUTE_SEED,
  PROCEED_SEED,
  STOP_SEED,
  AVOID_LEFT_SEED,
  AVOID_RIGHT_SEED,
};

struct CandidateBankConfig
{
  std::string mode{"off"};  // off | observe | evaluate | select

  // Persistent-bank lifetime constraints.
  std::size_t max_size{16};
  double max_generation_age_sec{5.0};
  double max_projection_error_m{2.0};
  double min_remaining_horizon_sec{1.0};

  // Temporal alignment for persistent trajectories. The evaluation time is supplied by the
  // standalone adapter (latest Odometry stamp). A stored trajectory is first advanced to its
  // expected time_from_start, then spatially projected only inside this local time window.
  // Default OFF: preserve the original Candidate Bank spatial live-start projection.
  // Can be enabled only for explicit A/B experiments.
  bool temporal_alignment_enabled{false};
  double temporal_alignment_search_window_sec{0.75};
  // Collision checking defaults to the exact oriented polygon model used by the
  // post-0011 experiments. Legacy single_circle/three_circle code is retained
  // only for log compatibility; the standalone node rejects those modes.
  std::string collision_geometry_mode{"polygon"};
  double ego_radius_m{1.5};
  double ego_length_m{4.8};
  double ego_width_m{1.8};
  double ego_base_link_to_center_m{1.4};
  double static_velocity_threshold_mps{0.3};
  std::size_t evaluation_stride{1};
  // `enabled` controls whether obstacle clearance/collision costs are evaluated.
  // `hard_reject_enabled=false` keeps the measurement and soft cost but does NOT
  // set candidate.safe=false when the collision margin is violated.
  bool static_safety_enabled{false};
  bool static_hard_reject_enabled{true};
  double static_collision_margin_m{0.3};
  double static_preferred_clearance_m{1.0};

  bool dynamic_safety_enabled{false};
  bool dynamic_hard_reject_enabled{true};
  double dynamic_collision_margin_m{0.5};
  double dynamic_preferred_clearance_m{1.5};
  double dynamic_conflict_distance_m{2.5};
  double dynamic_preferred_time_gap_sec{2.0};
  // legacy_time_gap: preserve the 0012 all-(ego_i, object_j) accumulated time-gap score.
  // ttc: disable that accumulated score and use one confidence-weighted predicted TTC cost
  //      per predicted path. min_dynamic_time_gap_sec is still measured for diagnostics.
  std::string dynamic_temporal_cost_mode{"ttc"};
  double dynamic_ttc_threshold_sec{1.5};
  double dynamic_time_uncertainty_sec{0.2};
  double velocity_uncertainty_gain{1.0};
  double acceleration_uncertainty_gain{0.25};
  double min_prediction_confidence{0.05};
  // normalized_path_max removes trajectory-sampling and PredictedPath-count scaling from
  // dynamic soft cost. legacy_sum reproduces the 0013a accumulation behavior.
  std::string dynamic_soft_cost_aggregation_mode{"normalized_path_max"};
  // closing_rate interprets time_uncertainty_sec as a timing error and enlarges clearance only
  // in the direction in which geometric clearance is actually closing. legacy_absolute_speed
  // preserves the earlier absolute-object-speed / prediction-horizon envelope.
  std::string dynamic_uncertainty_mode{"closing_rate"};
  double dynamic_max_uncertainty_margin_m{2.0};
  // 0.0 preserves the existing "any qualifying path may hard reject" behavior.
  // Set >0 to require at least this raw PredictedPath confidence for hard rejection.
  double dynamic_hard_reject_min_path_confidence{0.0};
  // E_i x O_j cross-scenario trajectory conflict evaluation.
  // 0013a7 behavior remains observer-only unless aggregation apply_* options are enabled.
  bool cross_collision_enabled{false};
  double cross_collision_spatial_scale_m{1.0};
  double cross_collision_temporal_scale_sec{0.5};
  double cross_collision_conflict_rho{1.0};
  double cross_collision_block_duration_sec{1.0};
  double cross_collision_lateral_undercoverage_m{0.10};
  std::size_t cross_collision_max_circles_per_body{10};
  double cross_collision_hard_collision_margin_m{0.0};
  double cross_collision_xyt_spatial_margin_m{0.0};
  bool cross_following_enabled{false};
  double cross_following_preferred_clearance_m{3.0};
  double cross_following_lateral_margin_m{0.5};
  double cross_following_risk_weight{1.0};
  bool cross_collision_compute_toi{true};
  bool cross_aggregation_enabled{false};
  std::string cross_aggregation_weight_mode{"uniform"};
  bool cross_aggregation_apply_to_score{false};
  bool cross_aggregation_apply_short_horizon_hard_reject{false};
  double cross_aggregation_score_weight{1.0};
  double cross_aggregation_short_horizon_sec{1.5};
  // all: preserve current behavior. map_conflict_reachable: evaluate full dynamic risk only for
  // same-route lead traffic or map competing-lane traffic whose conflict arrival time overlaps.
  std::string dynamic_scope_mode{"all"};
  double map_conflict_forward_search_distance_m{80.0};
  double map_conflict_backtrace_distance_m{80.0};
  double map_conflict_arrival_time_margin_sec{1.5};
  double map_conflict_candidate_match_distance_m{3.0};
  double map_conflict_lane_association_max_distance_m{4.0};
  double map_conflict_lane_association_min_confidence{0.15};
  double map_conflict_lane_association_heading_sigma_deg{35.0};
  double map_conflict_heading_min_speed_mps{0.8};
  double map_conflict_history_window_sec{1.0};
  std::size_t map_conflict_lane_hypothesis_top_k{3};
  bool map_conflict_include_same_route_lead{true};
  bool map_conflict_fallback_to_all_if_map_unavailable{true};
  bool map_conflict_immediate_guard_enabled{true};
  double map_conflict_immediate_guard_horizon_sec{1.0};
  double map_conflict_immediate_guard_collision_margin_m{0.2};

  // Lower total score is better.
  double progress_weight{1.0};
  double route_deviation_weight{0.5};
  double path_efficiency_weight{0.2};
  double clearance_weight{2.0};
  double dynamic_margin_weight{2.0};
  double comfort_weight{0.2};
  // Reduce route-centerline tracking pressure only where the current route reference itself
  // contains a sharp/index-local discontinuity compared with a short moving-average reference.
  bool route_deviation_adaptive_weight_enabled{false};
  std::size_t route_deviation_smoothing_window_points{7};
  double route_deviation_roughness_threshold_m{0.5};
  double route_deviation_min_weight{0.1};

  // Traffic-signal rule is a soft score term. Signal color policy / stop margin remain shared
  // with the existing SDEdit traffic-signal parameters in DiffusionPlannerCore.
  bool traffic_signal_rule_enabled{false};
  double traffic_signal_crossing_penalty{100.0};
  double traffic_signal_stop_speed_weight{10.0};
  double traffic_signal_stop_velocity_threshold_mps{0.2};
  double traffic_signal_approach_distance_m{2.0};
  // Added Candidate Bank traffic-signal soft terms. Defaults preserve prior behavior.
  double traffic_signal_overrun_weight{0.0};
  double traffic_signal_post_stop_hold_distance_m{0.0};
  // Crossing is considered at stop_line_s - crossing_margin_m. The speed term remains off.
  double traffic_signal_crossing_margin_m{0.5};
  // When enabled, a point whose route progress decreases copies the previous corrected pose.
  bool clip_backward_points_enabled{false};
  // route_nearest preserves 0012 behavior. current_lane restricts the traffic-light candidates to
  // the lane currently occupied by ego before projecting its stop line onto the route reference.
  std::string traffic_signal_scope_mode{"route_nearest"};

  // Simple lane-boundary occupancy score. Only boundaries shared by at least two lanelets are used.
  bool lane_boundary_enabled{false};
  double lane_boundary_prolonged_straddling_threshold_sec{1.5};
  double lane_boundary_prolonged_straddling_weight{5.0};
  double lane_boundary_terminal_straddling_penalty{20.0};
  // Lightweight maneuver semantics layered on top of the existing boundary geometry.
  // The feature is soft-cost only so it remains compatible with the current no-safe fallback.
  bool lane_change_deadline_enabled{false};
  std::string lane_change_deadline_activation_mode{"ego_straddling"};
  double lane_change_completion_margin_m{5.0};
  double lane_change_comfortable_slack_m{10.0};
  double lane_change_approaching_weight{5.0};
  double lane_change_missed_deadline_penalty{300.0};
  bool lane_change_fallback_stop_enabled{true};
  bool lane_change_fallback_use_target_lane_stop_condition{true};
  bool lane_change_fallback_combine_with_route_stop_condition{true};
  double lane_change_fallback_stop_penalty{25.0};
  double lane_change_fallback_stop_max_distance_m{8.0};
  double lane_change_target_lane_signal_search_distance_m{80.0};
  // Hierarchical selector: safety class is compared before the normal trajectory score.
  // SAFE candidates use the existing score + hysteresis.  If none are SAFE, CAUTION and then
  // UNSAFE candidates are retained and compared by relative safety rather than returning nullopt.
  bool safety_tier_enabled{false};
  double safety_tier_imminent_horizon_sec{2.0};
  bool safety_tier_use_static_margin{true};
  bool safety_tier_use_dynamic_margin{true};
  bool safety_tier_use_same_time_collision{true};
  bool safety_tier_use_xyt_conflict_for_caution{true};
  bool safety_tier_prefer_stop_seed_when_non_safe{true};

  // Keep the current selected candidate unless a new one is clearly better.
  bool hysteresis_enabled{true};
  double switch_margin{0.2};
};
struct CandidateLaneBoundaryGeometry
{
  std::int64_t boundary_id{0};
  std::int64_t adjacent_lane_id{0};
  std::vector<geometry_msgs::msg::Point> points_map;
  bool adjacent_signal_present{false};
  bool adjacent_signal_stop_active{false};
  double adjacent_signal_stop_line_s_m{0.0};
  double adjacent_signal_target_stop_s_m{0.0};
  std::int64_t adjacent_traffic_light_id{0};
  std::uint8_t adjacent_traffic_light_color{0};
};

struct CandidateTrafficRuleContext
{
  std::string traffic_signal_scope_mode{"route_nearest"};
  bool current_lane_valid{false};
  std::int64_t current_lane_id{0};
  std::vector<CandidateLaneBoundaryGeometry> shared_lane_boundaries_map;
  bool traffic_signal_present{false};
  bool traffic_signal_stop_active{false};
  double traffic_signal_stop_line_s_m{0.0};
  double traffic_signal_target_stop_s_m{0.0};
  std::int64_t traffic_light_id{0};
  std::uint8_t traffic_light_color{0};
};
struct CandidateCollisionCircleDebug
{
  geometry_msgs::msg::Point center;
  double radius_m{0.0};
};

struct CandidateObstacleGeometryDebug
{
  bool is_static{false};
  std::vector<CandidateCollisionCircleDebug> circles;
  std::vector<geometry_msgs::msg::Point> polygon;
};
// Per-cycle transient diagnostics for the exact same-time dynamic evaluation.
// These fields never participate in scoring/selection; they are retained only until select()
// logs the selected and highest-progress candidates for offline visualization.
struct CandidateDynamicSampleDebug
{
  double time_sec{0.0};
  double ego_x{0.0};
  double ego_y{0.0};
  double ego_yaw{0.0};
  double object_x{0.0};
  double object_y{0.0};
  double object_yaw{0.0};
  double geometric_clearance_m{std::numeric_limits<double>::infinity()};
  double closing_rate_mps{0.0};
  double uncertainty_margin_m{0.0};
  double evaluated_clearance_m{std::numeric_limits<double>::infinity()};
  double clearance_soft_cost{0.0};
  bool collision_margin_violated{false};
};
struct CandidateLaneHypothesisDebug
{
  std::int64_t lane_id{0};
  bool is_route_lane{false};
  double s_m{0.0};
  double confidence{0.0};
  double lateral_distance_m{std::numeric_limits<double>::infinity()};
  double heading_error_deg{std::numeric_limits<double>::infinity()};
  double predicted_path_fit_m{std::numeric_limits<double>::infinity()};
};

struct CandidateMapConflictDecisionDebug
{
  std::string object_id;
  bool full_dynamic_evaluation{false};
  bool immediate_guard_only{false};
  bool immediate_guard_violated{false};
  std::string reason{"not_evaluated"};
  bool lane_associated{false};
  std::vector<CandidateLaneHypothesisDebug> lane_hypotheses;
  double motion_heading_yaw{0.0};
  double motion_heading_reliability{0.0};
  std::int64_t best_lane_id{0};
  double best_lane_confidence{0.0};
  double best_lane_lateral_distance_m{std::numeric_limits<double>::infinity()};
  double best_lane_heading_error_deg{std::numeric_limits<double>::infinity()};
  double best_lane_predicted_path_fit_m{std::numeric_limits<double>::infinity()};
  std::string motion_heading_source{"none"};
  bool lane_association_ambiguous{true};
  std::int64_t conflict_ego_lane_id{0};
  std::int64_t conflict_competing_lane_id{0};
  double conflict_x_map{std::numeric_limits<double>::quiet_NaN()};
  double conflict_y_map{std::numeric_limits<double>::quiet_NaN()};
  double object_distance_to_conflict_m{std::numeric_limits<double>::infinity()};
  double ego_arrival_time_sec{std::numeric_limits<double>::infinity()};
  double object_arrival_time_sec{std::numeric_limits<double>::infinity()};
};

struct CandidateDynamicEvaluationDebug
{
  bool valid{false};
  std::string object_id;
  int path_index{-1};
  bool is_fallback{false};
  double path_confidence{0.0};
  double normalized_confidence{0.0};
  double path_clearance_soft_cost{0.0};
  double path_ttc_soft_cost{0.0};
  double path_total_soft_cost{0.0};
  double ttc_sec{std::numeric_limits<double>::infinity()};
  bool collision_margin_violated{false};
  std::size_t worst_sample_index{0};
  std::vector<CandidateDynamicSampleDebug> samples;
};

struct TrajectoryCandidate
{
  std::uint64_t id{0};
  CandidateSource source{CandidateSource::LEGACY_OUTPUT};
  autoware_planning_msgs::msg::Trajectory trajectory_map;
  // Joint prediction generated by the same Diffusion Planner sample as trajectory_map.
  // This is observation-only in 0013a6; safety/score/selection still use the CURRENT prediction.
  autoware_perception_msgs::msg::PredictedObjects paired_predicted_objects;
  std::int64_t paired_prediction_batch_index{0};

  rclcpp::Time generation_time;
  rclcpp::Time last_evaluation_time;

  bool valid{true};
  bool safe{true};
  bool selected{false};
  // These flags remain observable even when hard rejection is disabled.
  bool static_collision_margin_violated{false};
  bool dynamic_collision_margin_violated{false};
  std::size_t live_start_index{0};

  double score{0.0};
  double projection_error_m{0.0};
  double remaining_horizon_sec{0.0};
  double min_static_clearance_m{std::numeric_limits<double>::infinity()};
  double min_dynamic_clearance_m{std::numeric_limits<double>::infinity()};
  double min_dynamic_time_gap_sec{std::numeric_limits<double>::infinity()};
  double min_dynamic_ttc_sec{std::numeric_limits<double>::infinity()};
  double dynamic_clearance_soft_cost{0.0};
  double dynamic_legacy_time_gap_soft_cost{0.0};
  double dynamic_ttc_soft_cost{0.0};
  double max_dynamic_closing_rate_mps{0.0};
  double max_dynamic_uncertainty_margin_m{0.0};
  std::size_t dynamic_prediction_path_count{0};
  double dynamic_prediction_confidence_sum{0.0};
  CandidateDynamicEvaluationDebug dynamic_analysis_debug;
  std::vector<CandidateMapConflictDecisionDebug> map_conflict_decisions_debug;
  double path_length_m{0.0};
  double progress_m{0.0};
  double route_deviation_rms_m{0.0};
  double comfort_cost{0.0};
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
  double lane_change_deadline_s_m{std::numeric_limits<double>::infinity()};
  double lane_change_completion_s_m{std::numeric_limits<double>::infinity()};
  double lane_change_slack_m{-std::numeric_limits<double>::infinity()};
  double lane_change_deadline_cost{0.0};

  // weighted score decomposition for logging/analysis.
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
struct CandidateCrossCollisionResult
{
  std::uint64_t ego_candidate_id{0};
  std::uint64_t scenario_candidate_id{0};
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
  double evaluation_us{0.0};
};
struct CandidateCrossAggregateResult
{
  std::uint64_t candidate_id{0};
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

struct CandidateBankDebugInfo
{
  std::string mode{"off"};
  std::string collision_geometry_mode{"polygon"};
  std::string dynamic_temporal_cost_mode{"ttc"};
  std::string dynamic_soft_cost_aggregation_mode{"normalized_path_max"};
  std::string dynamic_uncertainty_mode{"closing_rate"};
  std::string dynamic_scope_mode{"all"};
  std::size_t map_conflict_count{0};
  std::size_t bank_size{0};
  std::size_t safe_count{0};
  std::size_t purged_last_cycle{0};
  std::uint64_t newest_candidate_id{0};
  std::uint64_t best_candidate_id{0};
  std::uint64_t selected_candidate_id{0};
  double best_score{std::numeric_limits<double>::infinity()};
  double selected_score{std::numeric_limits<double>::infinity()};
  double selected_static_clearance_m{std::numeric_limits<double>::infinity()};
  double selected_dynamic_clearance_m{std::numeric_limits<double>::infinity()};
  double selected_dynamic_time_gap_sec{std::numeric_limits<double>::infinity()};
  double selected_dynamic_ttc_sec{std::numeric_limits<double>::infinity()};
  double selected_dynamic_clearance_soft_cost{0.0};
  double selected_dynamic_legacy_time_gap_soft_cost{0.0};
  double selected_dynamic_ttc_soft_cost{0.0};
  double selected_max_dynamic_closing_rate_mps{0.0};
  double selected_max_dynamic_uncertainty_margin_m{0.0};
  std::size_t selected_dynamic_prediction_path_count{0};
  double selected_dynamic_prediction_confidence_sum{0.0};
  bool static_hard_reject_enabled{true};
  bool dynamic_hard_reject_enabled{true};
  bool selected_static_collision_margin_violated{false};
  bool selected_dynamic_collision_margin_violated{false};
  bool route_deviation_adaptive_weight_enabled{false};
  double selected_route_reference_weight_mean{1.0};
  double selected_route_reference_max_roughness_m{0.0};
  bool traffic_signal_rule_enabled{false};
  bool selected_traffic_signal_stop_active{false};
  bool selected_traffic_signal_crossed{false};
  double selected_traffic_rule_cost{0.0};
  std::string traffic_signal_scope_mode{"route_nearest"};
  bool current_lane_valid{false};
  std::int64_t current_lane_id{0};
  bool lane_boundary_enabled{false};
  bool lane_change_deadline_enabled{false};
  std::string lane_change_deadline_activation_mode{"ego_straddling"};
  bool selected_lane_change_deadline_active{false};
  bool selected_lane_change_completed_before_deadline{false};
  bool selected_lane_change_fallback_stop_satisfied{false};
  double selected_lane_change_deadline_cost{0.0};
  double selected_max_continuous_straddling_sec{0.0};
  bool selected_terminal_straddling{false};
  double selected_lane_boundary_cost{0.0};
  std::string newest_source{"none"};
  std::string best_source{"none"};
  std::string selected_source{"none"};
  std::uint64_t previous_selected_candidate_id{0};
  bool selection_switched{false};
  bool hysteresis_kept{false};
  std::string selection_reason{"none"};
  bool safety_tier_enabled{false};
  std::size_t safety_tier_safe_count{0};
  std::size_t safety_tier_caution_count{0};
  std::size_t safety_tier_unsafe_count{0};
  std::string best_safety_tier{"DISABLED"};
  std::string selected_safety_tier{"DISABLED"};
};
void cb_compare_trace_trajectory_event(
  const char * stage, const char * mode, const nav_msgs::msg::Odometry & ego,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const rclcpp::Time & eval_time,
  std::uint64_t candidate_id = 0, double hold_age_sec = -1.0);

void cb_compare_trace_bank_summary(
  const char * phase, const nav_msgs::msg::Odometry & ego, const CandidateBankDebugInfo & debug,
  const rclcpp::Time & eval_time, std::size_t admitted_now = 0);

class TrajectoryCandidateManager
{
public:
  explicit TrajectoryCandidateManager(const CandidateBankConfig & config);

  void set_config(const CandidateBankConfig & config);
  void clear();

  bool enabled() const;
  bool evaluation_enabled() const;
  bool selection_enabled() const;

  void add(
    const autoware_planning_msgs::msg::Trajectory & trajectory_map, CandidateSource source,
    const autoware_perception_msgs::msg::PredictedObjects & paired_predicted_objects,
    std::int64_t paired_prediction_batch_index,
    const std::optional<autoware_planning_msgs::msg::Trajectory> & route_reference_map,
    const rclcpp::Time & now);

  // OBSERVE: map-frame persistence, ego projection, stale/unusable deletion.
  void purge(const nav_msgs::msg::Odometry & ego, const rclcpp::Time & now);

  // EVALUATE/SELECT: recompute every candidate against the CURRENT observation/prediction.
  // `tracked_objects` remains available even when DiffusionPlanner ignore_neighbors=true.
  // `predicted_objects` is preferred for dynamic motion when a matching predicted path exists;
  // otherwise a constant-acceleration fallback is computed from the tracked state.
  void evaluate(
    const nav_msgs::msg::Odometry & ego,
    const std::shared_ptr<const autoware_perception_msgs::msg::TrackedObjects> & tracked_objects,
    const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
    const std::optional<autoware_planning_msgs::msg::Trajectory> & route_reference_map,
    const CandidateMapConflictContext & map_conflict_context,
    const CandidateTrafficRuleContext & traffic_rule_context, const rclcpp::Time & now);

  // SELECT: return the live suffix of the best safe candidate. The currently selected safe
  // candidate is protected by hysteresis unless the new candidate improves by switch_margin.
  std::optional<autoware_planning_msgs::msg::Trajectory> select(const rclcpp::Time & now);

  std::optional<autoware_planning_msgs::msg::Trajectory> incumbent() const;

  const CandidateBankDebugInfo & debug_info() const { return debug_info_; }

  // immutable view used only by RViz/log diagnostics.
  const std::vector<TrajectoryCandidate> & candidates() const { return candidates_; }
  // Last E_i x O_j matrix.  Raw matrix values remain available even when selector connection is
  // enabled.
  const std::vector<CandidateCrossCollisionResult> & cross_collision_results() const
  {
    return cross_collision_results_;
  }
  const std::vector<CandidateCrossAggregateResult> & cross_aggregate_results() const
  {
    return cross_aggregate_results_;
  }
  const std::vector<CandidateObstacleGeometryDebug> & obstacle_geometry_debug() const
  {
    return obstacle_geometry_debug_;
  }

private:
  struct ObjectHistorySample
  {
    rclcpp::Time stamp;
    geometry_msgs::msg::Point position;
  };

  void enforce_size_limit();
  void refresh_debug_info();
  void update_object_history(
    const std::shared_ptr<const autoware_perception_msgs::msg::TrackedObjects> & tracked_objects,
    const rclcpp::Time & now);
  std::optional<double> history_motion_heading(
    const std::string & object_id, const geometry_msgs::msg::Point & current_position,
    const rclcpp::Time & now) const;

  CandidateBankConfig config_;
  std::vector<TrajectoryCandidate> candidates_;
  CandidateBankDebugInfo debug_info_;
  std::vector<CandidateCrossCollisionResult> cross_collision_results_;
  std::vector<CandidateCrossAggregateResult> cross_aggregate_results_;
  std::vector<CandidateObstacleGeometryDebug> obstacle_geometry_debug_;
  std::optional<std::uint64_t> incumbent_id_;
  std::uint64_t next_id_{1};
  bool has_logged_map_context_signature_{false};
  std::uint64_t last_logged_map_context_signature_{0};
  std::unordered_map<std::string, std::deque<ObjectHistorySample>> object_history_;
};

}  // namespace autoware::candidate_bank

// persistent map-frame trajectory bank and evaluator.

#endif  // AUTOWARE__CANDIDATE_BANK__CANDIDATE__TRAJECTORY_CANDIDATE_MANAGER_HPP_
