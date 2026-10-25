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

#include "autoware/candidate_bank/internal_debug/internal_debug_logger.hpp"
#include "autoware/candidate_bank/rule_context.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::candidate_bank
{
using Candidates = autoware_internal_planning_msgs::msg::CandidateTrajectories;
using Prediction = autoware_perception_msgs::msg::PredictedObjects;
using Tracked = autoware_perception_msgs::msg::TrackedObjects;
using Odometry = nav_msgs::msg::Odometry;
using Route = autoware_planning_msgs::msg::LaneletRoute;
using Map = autoware_map_msgs::msg::LaneletMapBin;
using Signal = autoware_perception_msgs::msg::TrafficLightGroupArray;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using Marker = visualization_msgs::msg::Marker;
using Markers = visualization_msgs::msg::MarkerArray;
using Json = nlohmann::json;

static std::size_t positive_size(int64_t value, const char * name)
{
  if (value < 1) throw std::invalid_argument(std::string(name) + " must be positive");
  return static_cast<std::size_t>(value);
}

class CandidateBankNode : public rclcpp::Node
{
public:
  CandidateBankNode() : Node("candidate_bank_node")
  {
    auto & node = *this;
    CandidateBankConfig c;
    c.mode = node.declare_parameter<std::string>("bank.mode", c.mode);
    c.max_size = positive_size(
      node.declare_parameter<int64_t>("bank.max_size", static_cast<int64_t>(c.max_size)),
      "bank.max_size");
    c.max_generation_age_sec =
      node.declare_parameter<double>("bank.max_generation_age_sec", c.max_generation_age_sec);
    c.max_projection_error_m =
      node.declare_parameter<double>("bank.max_projection_error_m", c.max_projection_error_m);
    c.min_remaining_horizon_sec =
      node.declare_parameter<double>("bank.min_remaining_horizon_sec", c.min_remaining_horizon_sec);
    c.temporal_alignment_enabled =
      node.declare_parameter<bool>("bank.temporal_alignment_enabled", c.temporal_alignment_enabled);
    c.temporal_alignment_search_window_sec = node.declare_parameter<double>(
      "bank.temporal_alignment_search_window_sec", c.temporal_alignment_search_window_sec);
    c.collision_geometry_mode = node.declare_parameter<std::string>(
      "bank.collision_geometry_mode", c.collision_geometry_mode);
    c.static_velocity_threshold_mps = node.declare_parameter<double>(
      "bank.static_velocity_threshold_mps", c.static_velocity_threshold_mps);
    c.evaluation_stride = positive_size(
      node.declare_parameter<int64_t>(
        "bank.evaluation_stride", static_cast<int64_t>(c.evaluation_stride)),
      "bank.evaluation_stride");
    c.static_safety_enabled =
      node.declare_parameter<bool>("bank.static_safety_enabled", c.static_safety_enabled);
    c.static_hard_reject_enabled =
      node.declare_parameter<bool>("bank.static_hard_reject_enabled", c.static_hard_reject_enabled);
    c.static_collision_margin_m =
      node.declare_parameter<double>("bank.static_collision_margin_m", c.static_collision_margin_m);
    c.static_preferred_clearance_m = node.declare_parameter<double>(
      "bank.static_preferred_clearance_m", c.static_preferred_clearance_m);
    c.dynamic_safety_enabled =
      node.declare_parameter<bool>("bank.dynamic_safety_enabled", c.dynamic_safety_enabled);
    c.dynamic_hard_reject_enabled = node.declare_parameter<bool>(
      "bank.dynamic_hard_reject_enabled", c.dynamic_hard_reject_enabled);
    c.dynamic_collision_margin_m = node.declare_parameter<double>(
      "bank.dynamic_collision_margin_m", c.dynamic_collision_margin_m);
    c.dynamic_preferred_clearance_m = node.declare_parameter<double>(
      "bank.dynamic_preferred_clearance_m", c.dynamic_preferred_clearance_m);
    c.dynamic_conflict_distance_m = node.declare_parameter<double>(
      "bank.dynamic_conflict_distance_m", c.dynamic_conflict_distance_m);
    c.dynamic_preferred_time_gap_sec = node.declare_parameter<double>(
      "bank.dynamic_preferred_time_gap_sec", c.dynamic_preferred_time_gap_sec);
    c.dynamic_temporal_cost_mode = node.declare_parameter<std::string>(
      "bank.dynamic_temporal_cost_mode", c.dynamic_temporal_cost_mode);
    c.dynamic_ttc_threshold_sec =
      node.declare_parameter<double>("bank.dynamic_ttc_threshold_sec", c.dynamic_ttc_threshold_sec);
    c.dynamic_time_uncertainty_sec = node.declare_parameter<double>(
      "bank.dynamic_time_uncertainty_sec", c.dynamic_time_uncertainty_sec);
    c.velocity_uncertainty_gain =
      node.declare_parameter<double>("bank.velocity_uncertainty_gain", c.velocity_uncertainty_gain);
    c.acceleration_uncertainty_gain = node.declare_parameter<double>(
      "bank.acceleration_uncertainty_gain", c.acceleration_uncertainty_gain);
    c.min_prediction_confidence =
      node.declare_parameter<double>("bank.min_prediction_confidence", c.min_prediction_confidence);
    c.dynamic_soft_cost_aggregation_mode = node.declare_parameter<std::string>(
      "bank.dynamic_soft_cost_aggregation_mode", c.dynamic_soft_cost_aggregation_mode);
    c.dynamic_uncertainty_mode = node.declare_parameter<std::string>(
      "bank.dynamic_uncertainty_mode", c.dynamic_uncertainty_mode);
    c.dynamic_max_uncertainty_margin_m = node.declare_parameter<double>(
      "bank.dynamic_max_uncertainty_margin_m", c.dynamic_max_uncertainty_margin_m);
    c.dynamic_hard_reject_min_path_confidence = node.declare_parameter<double>(
      "bank.dynamic_hard_reject_min_path_confidence", c.dynamic_hard_reject_min_path_confidence);
    c.cross_collision_enabled =
      node.declare_parameter<bool>("bank.cross_collision_enabled", c.cross_collision_enabled);
    c.cross_collision_spatial_scale_m = node.declare_parameter<double>(
      "bank.cross_collision_spatial_scale_m", c.cross_collision_spatial_scale_m);
    c.cross_collision_temporal_scale_sec = node.declare_parameter<double>(
      "bank.cross_collision_temporal_scale_sec", c.cross_collision_temporal_scale_sec);
    c.cross_collision_conflict_rho = node.declare_parameter<double>(
      "bank.cross_collision_conflict_rho", c.cross_collision_conflict_rho);
    c.cross_collision_block_duration_sec = node.declare_parameter<double>(
      "bank.cross_collision_block_duration_sec", c.cross_collision_block_duration_sec);
    c.cross_collision_lateral_undercoverage_m = node.declare_parameter<double>(
      "bank.cross_collision_lateral_undercoverage_m", c.cross_collision_lateral_undercoverage_m);
    c.cross_collision_max_circles_per_body = positive_size(
      node.declare_parameter<int64_t>(
        "bank.cross_collision_max_circles_per_body",
        static_cast<int64_t>(c.cross_collision_max_circles_per_body)),
      "bank.cross_collision_max_circles_per_body");
    c.cross_collision_hard_collision_margin_m = node.declare_parameter<double>(
      "bank.cross_collision_hard_collision_margin_m", c.cross_collision_hard_collision_margin_m);
    c.cross_collision_xyt_spatial_margin_m = node.declare_parameter<double>(
      "bank.cross_collision_xyt_spatial_margin_m", c.cross_collision_xyt_spatial_margin_m);
    c.cross_following_enabled =
      node.declare_parameter<bool>("bank.cross_following_enabled", c.cross_following_enabled);
    c.cross_following_preferred_clearance_m = node.declare_parameter<double>(
      "bank.cross_following_preferred_clearance_m", c.cross_following_preferred_clearance_m);
    c.cross_following_lateral_margin_m = node.declare_parameter<double>(
      "bank.cross_following_lateral_margin_m", c.cross_following_lateral_margin_m);
    c.cross_following_risk_weight = node.declare_parameter<double>(
      "bank.cross_following_risk_weight", c.cross_following_risk_weight);
    c.cross_collision_compute_toi = node.declare_parameter<bool>(
      "bank.cross_collision_compute_toi", c.cross_collision_compute_toi);
    c.cross_aggregation_enabled =
      node.declare_parameter<bool>("bank.cross_aggregation_enabled", c.cross_aggregation_enabled);
    c.cross_aggregation_weight_mode = node.declare_parameter<std::string>(
      "bank.cross_aggregation_weight_mode", c.cross_aggregation_weight_mode);
    c.cross_aggregation_apply_to_score = node.declare_parameter<bool>(
      "bank.cross_aggregation_apply_to_score", c.cross_aggregation_apply_to_score);
    c.cross_aggregation_apply_short_horizon_hard_reject = node.declare_parameter<bool>(
      "bank.cross_aggregation_apply_short_horizon_hard_reject",
      c.cross_aggregation_apply_short_horizon_hard_reject);
    c.cross_aggregation_score_weight = node.declare_parameter<double>(
      "bank.cross_aggregation_score_weight", c.cross_aggregation_score_weight);
    c.cross_aggregation_short_horizon_sec = node.declare_parameter<double>(
      "bank.cross_aggregation_short_horizon_sec", c.cross_aggregation_short_horizon_sec);
    c.dynamic_scope_mode =
      node.declare_parameter<std::string>("bank.dynamic_scope_mode", c.dynamic_scope_mode);
    c.map_conflict_forward_search_distance_m = node.declare_parameter<double>(
      "bank.map_conflict_forward_search_distance_m", c.map_conflict_forward_search_distance_m);
    c.map_conflict_backtrace_distance_m = node.declare_parameter<double>(
      "bank.map_conflict_backtrace_distance_m", c.map_conflict_backtrace_distance_m);
    c.map_conflict_arrival_time_margin_sec = node.declare_parameter<double>(
      "bank.map_conflict_arrival_time_margin_sec", c.map_conflict_arrival_time_margin_sec);
    c.map_conflict_candidate_match_distance_m = node.declare_parameter<double>(
      "bank.map_conflict_candidate_match_distance_m", c.map_conflict_candidate_match_distance_m);
    c.map_conflict_lane_association_max_distance_m = node.declare_parameter<double>(
      "bank.map_conflict_lane_association_max_distance_m",
      c.map_conflict_lane_association_max_distance_m);
    c.map_conflict_lane_association_min_confidence = node.declare_parameter<double>(
      "bank.map_conflict_lane_association_min_confidence",
      c.map_conflict_lane_association_min_confidence);
    c.map_conflict_lane_association_heading_sigma_deg = node.declare_parameter<double>(
      "bank.map_conflict_lane_association_heading_sigma_deg",
      c.map_conflict_lane_association_heading_sigma_deg);
    c.map_conflict_heading_min_speed_mps = node.declare_parameter<double>(
      "bank.map_conflict_heading_min_speed_mps", c.map_conflict_heading_min_speed_mps);
    c.map_conflict_history_window_sec = node.declare_parameter<double>(
      "bank.map_conflict_history_window_sec", c.map_conflict_history_window_sec);
    c.map_conflict_lane_hypothesis_top_k = positive_size(
      node.declare_parameter<int64_t>(
        "bank.map_conflict_lane_hypothesis_top_k",
        static_cast<int64_t>(c.map_conflict_lane_hypothesis_top_k)),
      "bank.map_conflict_lane_hypothesis_top_k");
    c.map_conflict_include_same_route_lead = node.declare_parameter<bool>(
      "bank.map_conflict_include_same_route_lead", c.map_conflict_include_same_route_lead);
    c.map_conflict_fallback_to_all_if_map_unavailable = node.declare_parameter<bool>(
      "bank.map_conflict_fallback_to_all_if_map_unavailable",
      c.map_conflict_fallback_to_all_if_map_unavailable);
    c.map_conflict_immediate_guard_enabled = node.declare_parameter<bool>(
      "bank.map_conflict_immediate_guard_enabled", c.map_conflict_immediate_guard_enabled);
    c.map_conflict_immediate_guard_horizon_sec = node.declare_parameter<double>(
      "bank.map_conflict_immediate_guard_horizon_sec", c.map_conflict_immediate_guard_horizon_sec);
    c.map_conflict_immediate_guard_collision_margin_m = node.declare_parameter<double>(
      "bank.map_conflict_immediate_guard_collision_margin_m",
      c.map_conflict_immediate_guard_collision_margin_m);
    c.progress_weight = node.declare_parameter<double>("bank.progress_weight", c.progress_weight);
    c.route_deviation_weight =
      node.declare_parameter<double>("bank.route_deviation_weight", c.route_deviation_weight);
    c.path_efficiency_weight =
      node.declare_parameter<double>("bank.path_efficiency_weight", c.path_efficiency_weight);
    c.clearance_weight =
      node.declare_parameter<double>("bank.clearance_weight", c.clearance_weight);
    c.dynamic_margin_weight =
      node.declare_parameter<double>("bank.dynamic_margin_weight", c.dynamic_margin_weight);
    c.comfort_weight = node.declare_parameter<double>("bank.comfort_weight", c.comfort_weight);
    c.route_deviation_adaptive_weight_enabled = node.declare_parameter<bool>(
      "bank.route_deviation_adaptive_weight_enabled", c.route_deviation_adaptive_weight_enabled);
    c.route_deviation_smoothing_window_points = positive_size(
      node.declare_parameter<int64_t>(
        "bank.route_deviation_smoothing_window_points",
        static_cast<int64_t>(c.route_deviation_smoothing_window_points)),
      "bank.route_deviation_smoothing_window_points");
    c.route_deviation_roughness_threshold_m = node.declare_parameter<double>(
      "bank.route_deviation_roughness_threshold_m", c.route_deviation_roughness_threshold_m);
    c.route_deviation_min_weight = node.declare_parameter<double>(
      "bank.route_deviation_min_weight", c.route_deviation_min_weight);
    c.traffic_signal_rule_enabled = node.declare_parameter<bool>(
      "bank.traffic_signal_rule_enabled", c.traffic_signal_rule_enabled);
    c.traffic_signal_crossing_penalty = node.declare_parameter<double>(
      "bank.traffic_signal_crossing_penalty", c.traffic_signal_crossing_penalty);
    c.traffic_signal_stop_speed_weight = node.declare_parameter<double>(
      "bank.traffic_signal_stop_speed_weight", c.traffic_signal_stop_speed_weight);
    c.traffic_signal_stop_velocity_threshold_mps = node.declare_parameter<double>(
      "bank.traffic_signal_stop_velocity_threshold_mps",
      c.traffic_signal_stop_velocity_threshold_mps);
    c.traffic_signal_approach_distance_m = node.declare_parameter<double>(
      "bank.traffic_signal_approach_distance_m", c.traffic_signal_approach_distance_m);
    c.traffic_signal_overrun_weight = node.declare_parameter<double>(
      "bank.traffic_signal_overrun_weight", c.traffic_signal_overrun_weight);
    c.traffic_signal_post_stop_hold_distance_m = node.declare_parameter<double>(
      "bank.traffic_signal_post_stop_hold_distance_m", c.traffic_signal_post_stop_hold_distance_m);
    c.traffic_signal_crossing_margin_m = node.declare_parameter<double>(
      "bank.traffic_signal_crossing_margin_m", c.traffic_signal_crossing_margin_m);
    c.clip_backward_points_enabled = node.declare_parameter<bool>(
      "bank.clip_backward_points_enabled", c.clip_backward_points_enabled);
    c.traffic_signal_scope_mode = node.declare_parameter<std::string>(
      "bank.traffic_signal_scope_mode", c.traffic_signal_scope_mode);
    c.lane_boundary_enabled =
      node.declare_parameter<bool>("bank.lane_boundary_enabled", c.lane_boundary_enabled);
    c.lane_boundary_prolonged_straddling_threshold_sec = node.declare_parameter<double>(
      "bank.lane_boundary_prolonged_straddling_threshold_sec",
      c.lane_boundary_prolonged_straddling_threshold_sec);
    c.lane_boundary_prolonged_straddling_weight = node.declare_parameter<double>(
      "bank.lane_boundary_prolonged_straddling_weight",
      c.lane_boundary_prolonged_straddling_weight);
    c.lane_boundary_terminal_straddling_penalty = node.declare_parameter<double>(
      "bank.lane_boundary_terminal_straddling_penalty",
      c.lane_boundary_terminal_straddling_penalty);
    c.lane_change_deadline_enabled = node.declare_parameter<bool>(
      "bank.lane_change_deadline_enabled", c.lane_change_deadline_enabled);
    c.lane_change_deadline_activation_mode = node.declare_parameter<std::string>(
      "bank.lane_change_deadline_activation_mode", c.lane_change_deadline_activation_mode);
    c.lane_change_completion_margin_m = node.declare_parameter<double>(
      "bank.lane_change_completion_margin_m", c.lane_change_completion_margin_m);
    c.lane_change_comfortable_slack_m = node.declare_parameter<double>(
      "bank.lane_change_comfortable_slack_m", c.lane_change_comfortable_slack_m);
    c.lane_change_approaching_weight = node.declare_parameter<double>(
      "bank.lane_change_approaching_weight", c.lane_change_approaching_weight);
    c.lane_change_missed_deadline_penalty = node.declare_parameter<double>(
      "bank.lane_change_missed_deadline_penalty", c.lane_change_missed_deadline_penalty);
    c.lane_change_fallback_stop_enabled = node.declare_parameter<bool>(
      "bank.lane_change_fallback_stop_enabled", c.lane_change_fallback_stop_enabled);
    c.lane_change_fallback_use_target_lane_stop_condition = node.declare_parameter<bool>(
      "bank.lane_change_fallback_use_target_lane_stop_condition",
      c.lane_change_fallback_use_target_lane_stop_condition);
    c.lane_change_fallback_combine_with_route_stop_condition = node.declare_parameter<bool>(
      "bank.lane_change_fallback_combine_with_route_stop_condition",
      c.lane_change_fallback_combine_with_route_stop_condition);
    c.lane_change_fallback_stop_penalty = node.declare_parameter<double>(
      "bank.lane_change_fallback_stop_penalty", c.lane_change_fallback_stop_penalty);
    c.lane_change_fallback_stop_max_distance_m = node.declare_parameter<double>(
      "bank.lane_change_fallback_stop_max_distance_m", c.lane_change_fallback_stop_max_distance_m);
    c.lane_change_target_lane_signal_search_distance_m = node.declare_parameter<double>(
      "bank.lane_change_target_lane_signal_search_distance_m",
      c.lane_change_target_lane_signal_search_distance_m);
    c.safety_tier_enabled =
      node.declare_parameter<bool>("bank.safety_tier_enabled", c.safety_tier_enabled);
    c.safety_tier_imminent_horizon_sec = node.declare_parameter<double>(
      "bank.safety_tier_imminent_horizon_sec", c.safety_tier_imminent_horizon_sec);
    c.safety_tier_use_static_margin = node.declare_parameter<bool>(
      "bank.safety_tier_use_static_margin", c.safety_tier_use_static_margin);
    c.safety_tier_use_dynamic_margin = node.declare_parameter<bool>(
      "bank.safety_tier_use_dynamic_margin", c.safety_tier_use_dynamic_margin);
    c.safety_tier_use_same_time_collision = node.declare_parameter<bool>(
      "bank.safety_tier_use_same_time_collision", c.safety_tier_use_same_time_collision);
    c.safety_tier_use_xyt_conflict_for_caution = node.declare_parameter<bool>(
      "bank.safety_tier_use_xyt_conflict_for_caution", c.safety_tier_use_xyt_conflict_for_caution);
    c.safety_tier_prefer_stop_seed_when_non_safe = node.declare_parameter<bool>(
      "bank.safety_tier_prefer_stop_seed_when_non_safe",
      c.safety_tier_prefer_stop_seed_when_non_safe);
    c.hysteresis_enabled =
      node.declare_parameter<bool>("bank.hysteresis_enabled", c.hysteresis_enabled);
    c.switch_margin = node.declare_parameter<double>("bank.switch_margin", c.switch_margin);

    if (
      c.max_generation_age_sec <= 0 || c.max_projection_error_m <= 0 ||
      c.min_remaining_horizon_sec < 0 || c.temporal_alignment_search_window_sec < 0 ||
      c.cross_collision_spatial_scale_m <= 0 || c.cross_collision_temporal_scale_sec <= 0 ||
      c.cross_collision_block_duration_sec <= 0 || c.cross_collision_conflict_rho <= 0 ||
      c.cross_collision_lateral_undercoverage_m < 0 || c.cross_aggregation_short_horizon_sec <= 0)
      throw std::invalid_argument("invalid bank distance/time limits");
    if (c.mode != "select" && c.mode != "evaluate" && c.mode != "observe" && c.mode != "off")
      throw std::invalid_argument("invalid bank.mode");
    // CB_SAME_SETTINGS_PARITY_V01_BEGIN
    if (
      c.collision_geometry_mode != "polygon" && c.collision_geometry_mode != "three_circle" &&
      c.collision_geometry_mode != "single_circle")
      throw std::invalid_argument("invalid bank.collision_geometry_mode");
    // CB_SAME_SETTINGS_PARITY_V01_END
    if (c.traffic_signal_scope_mode != "route_nearest")
      throw std::invalid_argument("only route_nearest signal scope is implemented");
    const auto vehicle = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    c.ego_length_m = vehicle.front_overhang_m + vehicle.wheel_base_m + vehicle.rear_overhang_m;
    c.ego_width_m = vehicle.left_overhang_m + vehicle.wheel_tread_m + vehicle.right_overhang_m;
    c.ego_base_link_to_center_m =
      (vehicle.front_overhang_m + vehicle.wheel_base_m - vehicle.rear_overhang_m) / 2.0;
    c.ego_radius_m = 0.5 * std::hypot(c.ego_length_m, c.ego_width_m);
    config_ = c;
    manager_ = std::make_unique<TrajectoryCandidateManager>(c);

    max_age_ = declare_parameter<double>("input.max_generation_age_sec", 1.0);
    observation_age_ = declare_parameter<double>("input.max_observation_age_sec", 0.5);
    future_tolerance_ = declare_parameter<double>("input.future_tolerance_sec", 0.1);
    jump_m_ = declare_parameter<double>("input.reset_distance_m", 10.0);

    // CB_TRAJECTORY_TRIGGERED_ADAPTER_V01_BEGIN
    // The raw CandidateTrajectories callback is the only normal decision trigger.
    // Odometry / TrackedObjects / PredictedObjects / TrafficSignals callbacks only
    // write bounded timestamp buffers.  The default one-slot trajectory buffer
    // gives real-time "latest generation" semantics: if the node cannot keep up,
    // old unprocessed generations are discarded instead of building a backlog.
    state_buffer_size_ = positive_size(
      declare_parameter<int64_t>("input.state_buffer_size", 10), "input.state_buffer_size");
    trajectory_buffer_size_ = positive_size(
      declare_parameter<int64_t>("input.trajectory_buffer_size", 1),
      "input.trajectory_buffer_size");
    batch_ready_generations_ = declare_parameter<bool>("input.batch_ready_generations", false);
    // CB_TRAJECTORY_TRIGGERED_ADAPTER_V01_END

    // Legacy parameters are kept so existing YAMLs remain loadable.  In trajectory-triggered
    // mode they no longer define the decision clock.
    evaluation_min_period_sec_ = declare_parameter<double>("input.evaluation_min_period_sec", 0.05);
    evaluate_only_on_new_generation_ =
      declare_parameter<bool>("input.evaluate_only_on_new_generation", true);
    one_generation_per_decision_cycle_ =
      declare_parameter<bool>("input.one_generation_per_decision_cycle", true);
    align_current_prediction_to_eval_ =
      declare_parameter<bool>("input.align_current_prediction_to_eval", false);
    hold_last_selected_sec_ = declare_parameter<double>("output.hold_last_selected_sec", 1.0);
    tracked_buffer_sec_ = declare_parameter<double>("input.tracked_buffer_sec", 2.0);
    max_tracked_hold_sec_ = declare_parameter<double>("input.max_tracked_hold_sec", 1.0);
    failure_ = declare_parameter<std::string>("output.no_selection", "suppress");
    if (failure_ != "suppress")
      throw std::invalid_argument(
        "output.no_selection must be suppress; raw passthrough is disabled");
    if (
      max_age_ <= 0 || observation_age_ <= 0 || future_tolerance_ < 0 || jump_m_ <= 0 ||
      evaluation_min_period_sec_ < 0 || hold_last_selected_sec_ < 0 || tracked_buffer_sec_ <= 0 ||
      max_tracked_hold_sec_ <= 0) {
      throw std::invalid_argument("invalid trajectory-triggered input limits");
    }

    auto & r = context_.rules_;
    r.stop_on_red = declare_parameter<bool>("rules.stop_on_red", true);
    r.stop_on_amber = declare_parameter<bool>("rules.stop_on_amber", true);
    r.stop_on_unknown = declare_parameter<bool>("rules.stop_on_unknown", true);
    r.stop_margin_m = declare_parameter<double>("rules.stop_margin_m", 2.0);
    r.max_stop_line_lateral_distance_m =
      declare_parameter<double>("rules.max_stop_line_lateral_distance_m", 5.0);
    r.signal_timeout_sec = declare_parameter<double>("rules.signal_timeout_sec", 1.0);
    debug_ = declare_parameter<bool>("debug.publish_markers", true);
    internal_debug::LoggerConfig logger;
    logger.enabled = declare_parameter<bool>("debug.log_enabled", true);
    const char * home = std::getenv("HOME");
    logger.output_root = declare_parameter<std::string>(
      "debug.log_root", std::string(home ? home : "/tmp") + "/aw/logs/candidate_bank_standalone");
    logger.run_name = declare_parameter<std::string>("debug.run_name", "auto");
    internal_debug::configure(logger);

    pub_ = create_publisher<Candidates>("~/output/trajectories", 1);
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
    markers_ = create_publisher<Markers>("~/debug/markers", 1);
    status_ = create_publisher<std_msgs::msg::String>("~/debug/status", 1);
    matrix_ = create_publisher<std_msgs::msg::String>("~/debug/cross_matrix", 1);

    // CB_TRAJECTORY_TRIGGERED_ADAPTER_V01_BEGIN
    // DiffusionPlanner publishes CandidateTrajectories before PredictedObjects in the same planner
    // cycle.  Therefore this callback processes the previous ready generation, then stores the
    // just-arrived generation.  With a one-slot trajectory buffer this creates one-generation
    // pipeline latency while preserving exact Candidate/Prediction stamp pairing and 10 Hz cadence.
    //
    // Candidate input is deliberately KeepLast(1)+best_effort by default.  If processing cannot
    // keep up, intermediate trajectory generations are dropped instead of accumulating latency.
    const auto live = rclcpp::QoS(20).reliable();
    const auto trajectory_qos =
      rclcpp::QoS(rclcpp::KeepLast(trajectory_buffer_size_)).best_effort();
    // Register lightweight state subscriptions before the heavy trajectory callback.  If multiple
    // subscriptions are ready when a long trajectory callback returns, this also favors draining
    // buffered state (especially Prediction N) before handling a later Candidate N+1.
    prediction_sub_ = create_subscription<Prediction>(
      "~/input/predicted_objects", live, [this](Prediction::ConstSharedPtr msg) {
        check_clock();
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (stamp < generation_floor_ || admitted_generation_stamps_.count(stamp) != 0U) return;
        push_state_sample(prediction_buffer_, stamp, msg);
      });
    ego_sub_ =
      create_subscription<Odometry>("~/input/odometry", live, [this](Odometry::ConstSharedPtr msg) {
        check_clock();
        push_state_sample(odometry_buffer_, rclcpp::Time(msg->header.stamp).nanoseconds(), msg);
      });
    objects_sub_ = create_subscription<Tracked>(
      "~/input/tracked_objects", live, [this](Tracked::ConstSharedPtr msg) {
        check_clock();
        push_state_sample(tracked_buffer_, rclcpp::Time(msg->header.stamp).nanoseconds(), msg);
      });
    route_sub_ = create_subscription<Route>(
      "~/input/route", rclcpp::QoS(1).transient_local(), [this](Route::ConstSharedPtr msg) {
        reset("route_received");
        route_ = msg;
      });
    map_sub_ = create_subscription<Map>(
      "~/input/vector_map", rclcpp::QoS(1).transient_local(), [this](Map::ConstSharedPtr msg) {
        reset("map_update");
        context_.clear_map();
        try {
          context_.set_map(autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
          reset("map_received");
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "map: %s", e.what());
        }
      });
    signal_sub_ = create_subscription<Signal>(
      "~/input/traffic_signals", live, [this](Signal::ConstSharedPtr msg) {
        check_clock();
        push_state_sample(signal_buffer_, now().nanoseconds(), msg);
      });
    candidates_sub_ = create_subscription<Candidates>(
      "~/input/trajectories", trajectory_qos,
      [this](Candidates::ConstSharedPtr msg) { on_trajectory(msg); });
    // CB_TRAJECTORY_TRIGGERED_ADAPTER_V01_END
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset", [this](
                   const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        reset("manual_reset");
        res->success = true;
        res->message = "Candidate Bank cleared";
      });
    heartbeat_ = create_wall_timer(std::chrono::seconds(1), [this] {
      check_clock();
      if (!last_success_ || (now() - *last_success_).seconds() > max_age_)
        report("waiting_for_selected_output");
    });
    parameter_guard_ =
      add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : ps)
          if (p.get_name() != "use_sim_time") {
            result.successful = false;
            result.reason = "Restart Candidate Bank to change its configuration";
          }
        return result;
      });

    std::ostringstream params;
    for (const auto & name : list_parameters({}, 10).names)
      params << name << ": " << get_parameter(name).value_to_string() << "\n";
    internal_debug::write_effective_parameters(params.str());
    RCLCPP_INFO(
      get_logger(),
      "Standalone Candidate Bank: trajectory-triggered processing, bounded state buffers, exact "
      "generation association, raw passthrough disabled");
  }

private:
  void check_clock()
  {
    const auto t = now();
    if (last_clock_ && t < *last_clock_) {
      reset("clock_rewind");
      odometry_buffer_.clear();
      tracked_buffer_.clear();
      prediction_buffer_.clear();
      signal_buffer_.clear();
      ego_.reset();
      last_evaluated_ego_.reset();
      context_.clear_signals();
    }
    last_clock_ = t;
  }

  int64_t reset_floor_stamp() const
  {
    return odometry_buffer_.empty() ? now().nanoseconds() : odometry_buffer_.back().first;
  }

  void reset(const std::string & reason)
  {
    manager_->clear();
    trajectory_buffer_.clear();
    prediction_buffer_.clear();
    admitted_generation_stamps_.clear();
    latest_candidate_template_.reset();
    latest_prediction_.reset();
    latest_template_stamp_ = 0;
    last_eval_stamp_.reset();
    last_success_.reset();
    last_selected_output_.reset();
    last_selected_decision_stamp_.reset();
    last_evaluated_ego_.reset();
    generation_floor_ = reset_floor_stamp();
    if (debug_) {
      Markers m;
      Marker clear;
      clear.action = Marker::DELETEALL;
      m.markers.push_back(clear);
      markers_->publish(m);
    }
    report(reason);
  }

  bool source_fresh(
    const rclcpp::Time & stamp, double limit, const rclcpp::Time & receipt_now) const
  {
    const double age = (receipt_now - stamp).seconds();
    return age >= -future_tolerance_ && age <= limit;
  }

  template <typename PtrT>
  void push_state_sample(
    std::deque<std::pair<int64_t, PtrT>> & buffer, int64_t stamp, const PtrT & msg)
  {
    auto it = std::lower_bound(
      buffer.begin(), buffer.end(), stamp,
      [](const auto & item, int64_t value) { return item.first < value; });
    if (it != buffer.end() && it->first == stamp) {
      it->second = msg;
    } else {
      buffer.insert(it, {stamp, msg});
    }
    while (buffer.size() > state_buffer_size_) buffer.pop_front();
  }

  template <typename PtrT>
  PtrT sample_exact(const std::deque<std::pair<int64_t, PtrT>> & buffer, int64_t stamp) const
  {
    auto it = std::lower_bound(
      buffer.begin(), buffer.end(), stamp,
      [](const auto & item, int64_t value) { return item.first < value; });
    return it != buffer.end() && it->first == stamp ? it->second : PtrT{};
  }

  template <typename PtrT>
  PtrT sample_at_or_before(const std::deque<std::pair<int64_t, PtrT>> & buffer, int64_t stamp) const
  {
    for (auto it = buffer.rbegin(); it != buffer.rend(); ++it) {
      if (it->first <= stamp) return it->second;
    }
    return {};
  }

  template <typename PtrT>
  void erase_sample(std::deque<std::pair<int64_t, PtrT>> & buffer, int64_t stamp)
  {
    auto it = std::lower_bound(
      buffer.begin(), buffer.end(), stamp,
      [](const auto & item, int64_t value) { return item.first < value; });
    if (it != buffer.end() && it->first == stamp) buffer.erase(it);
  }

  double latest_tracked_age_for_status() const
  {
    if (!last_eval_stamp_) return std::numeric_limits<double>::quiet_NaN();
    const auto tracked = sample_at_or_before(tracked_buffer_, last_eval_stamp_->nanoseconds());
    if (!tracked) return std::numeric_limits<double>::quiet_NaN();
    return (*last_eval_stamp_ - rclcpp::Time(tracked->header.stamp)).seconds();
  }

  void report(const std::string & state)
  {
    const auto & d = manager_->debug_info();
    const double mean_callback_ms =
      trajectory_callbacks_ > 0 ? callback_total_ms_ / static_cast<double>(trajectory_callbacks_)
                                : 0.0;
    Json j = {
      {"state", state},
      {"stamp_sec", now().seconds()},
      {"bank_size", d.bank_size},
      {"safe_count", d.safe_count},
      {"selected_id", d.selected_candidate_id},
      {"pending_candidates", trajectory_buffer_.size()},
      {"pending_predictions", prediction_buffer_.size()},
      {"admitted_generations", admitted_generations_},
      {"processed", processed_},
      {"dropped", dropped_},
      {"selected_outputs", selected_outputs_},
      {"held_selected_outputs", held_selected_outputs_},
      {"repeated_outputs", repeated_outputs_},
      {"no_selection_suppressed", no_selection_suppressed_},
      {"raw_output_count", 0},
      {"trajectory_callbacks", trajectory_callbacks_},
      {"trajectory_buffer_overwrites", trajectory_buffer_overwrites_},
      {"pair_not_ready_on_trigger", pair_not_ready_on_trigger_},
      {"decision_cycles", decision_cycles_},
      {"last_callback_ms", last_callback_ms_},
      {"max_callback_ms", max_callback_ms_},
      {"mean_callback_ms", mean_callback_ms},
      {"trajectory_live_start_policy",
       config_.temporal_alignment_enabled ? "temporal_window" : "original_spatial"},
      {"processing_trigger", "candidate_trajectories_callback"},
      {"state_buffer_size", state_buffer_size_},
      {"trajectory_buffer_size", trajectory_buffer_size_},
      {"batch_ready_generations", batch_ready_generations_},
      {"current_prediction_temporal_shift", align_current_prediction_to_eval_},
      {"association_clock", "trajectory_generation_stamp"},
      {"evaluation_clock", "latest_odometry_on_trajectory_trigger"}};
    if (last_eval_stamp_) j["evaluation_stamp_sec"] = last_eval_stamp_->seconds();
    if (ego_) j["evaluation_odometry_stamp_sec"] = rclcpp::Time(ego_->header.stamp).seconds();
    const double tracked_age = latest_tracked_age_for_status();
    if (std::isfinite(tracked_age)) j["tracked_age_to_eval_sec"] = tracked_age;
    std_msgs::msg::String out;
    out.data = j.dump();
    status_->publish(out);
  }

  bool valid_candidate(const Candidates & input) const
  {
    if (input.candidate_trajectories.empty()) return false;
    const auto & candidate = input.candidate_trajectories.front();
    if (candidate.header.frame_id != "map" || candidate.points.size() < 2) return false;
    double previous = -1.0;
    for (const auto & pt : candidate.points) {
      const double tm = rclcpp::Duration(pt.time_from_start).seconds();
      if (
        !std::isfinite(pt.pose.position.x) || !std::isfinite(pt.pose.position.y) ||
        !std::isfinite(pt.longitudinal_velocity_mps) || tm < 0 || tm < previous)
        return false;
      previous = tm;
    }
    return previous > 0.0;
  }

  bool valid_odometry(const Odometry & ego) const
  {
    const auto & ep = ego.pose.pose;
    return ego.header.frame_id == "map" && std::isfinite(ep.position.x) &&
           std::isfinite(ep.position.y) && std::isfinite(ep.position.z) &&
           std::isfinite(ep.orientation.x) && std::isfinite(ep.orientation.y) &&
           std::isfinite(ep.orientation.z) && std::isfinite(ep.orientation.w);
  }

  Prediction align_prediction_to_eval(const Prediction & input, const rclcpp::Time & t_eval) const
  {
    Prediction output = input;
    const double age_sec = std::max(0.0, (t_eval - rclcpp::Time(input.header.stamp)).seconds());
    for (auto & object : output.objects) {
      for (auto & path : object.kinematics.predicted_paths) {
        if (path.path.empty()) continue;
        const double step = rclcpp::Duration(path.time_step).seconds();
        if (step <= 1.0e-6) continue;
        const std::size_t skip = std::min<std::size_t>(
          static_cast<std::size_t>(std::llround(age_sec / step)), path.path.size() - 1);
        if (skip > 0)
          path.path.erase(path.path.begin(), path.path.begin() + static_cast<std::ptrdiff_t>(skip));
      }
    }
    output.header.stamp = t_eval;
    return output;
  }

  struct DecisionOutcome
  {
    bool decision_ran{false};
    bool published{false};
  };

  void enqueue_trajectory(int64_t stamp, const Candidates::ConstSharedPtr & msg)
  {
    auto it = std::lower_bound(
      trajectory_buffer_.begin(), trajectory_buffer_.end(), stamp,
      [](const auto & item, int64_t value) { return item.first < value; });
    if (it != trajectory_buffer_.end() && it->first == stamp) {
      it->second = msg;
    } else {
      trajectory_buffer_.insert(it, {stamp, msg});
    }
    while (trajectory_buffer_.size() > trajectory_buffer_size_) {
      trajectory_buffer_.pop_front();
      ++trajectory_buffer_overwrites_;
      ++dropped_;
    }
  }

  void prune_admitted(const rclcpp::Time & t_eval)
  {
    const int64_t old_admitted_ns =
      (t_eval -
       rclcpp::Duration::from_seconds(std::max(max_age_, config_.max_generation_age_sec) + 1.0))
        .nanoseconds();
    for (auto it = admitted_generation_stamps_.begin(); it != admitted_generation_stamps_.end();) {
      if (*it < old_admitted_ns)
        it = admitted_generation_stamps_.erase(it);
      else
        ++it;
    }
  }

  DecisionOutcome process_ready_generations(int64_t trigger_stamp)
  {
    struct ReadyPair
    {
      int64_t stamp;
      Candidates::ConstSharedPtr candidates;
      Prediction::ConstSharedPtr prediction;
    };
    std::vector<ReadyPair> ready;
    for (const auto & item : trajectory_buffer_) {
      if (item.first > trigger_stamp) break;
      if (item.first < generation_floor_ || admitted_generation_stamps_.count(item.first) != 0U)
        continue;
      const auto prediction = sample_exact(prediction_buffer_, item.first);
      if (prediction) ready.push_back({item.first, item.second, prediction});
    }
    if (ready.empty()) {
      ++pair_not_ready_on_trigger_;
      return {};
    }
    if (!batch_ready_generations_ && ready.size() > 1) ready.resize(1);

    // CB_TRAJECTORY_TRIGGERED_LATEST_ODOM_V02
    // Candidate/Prediction generation stamp is for exact association and candidate age.
    // Candidate Bank evaluation uses the newest buffered Odometry available when this
    // trajectory callback runs, matching the previously-working standalone semantics.
    if (odometry_buffer_.empty()) {
      report("waiting_for_buffered_odometry");
      return {false, false};
    }
    const auto eval_ego = odometry_buffer_.back().second;
    const rclcpp::Time t_eval(eval_ego->header.stamp);
    if (!route_ || !context_.ready()) {
      report("waiting_for_context");
      return {false, false};
    }
    if (!valid_odometry(*eval_ego)) {
      ++dropped_;
      report("invalid_buffered_odometry");
      return {true, false};
    }

    if (
      last_evaluated_ego_ &&
      std::hypot(
        eval_ego->pose.pose.position.x - last_evaluated_ego_->pose.pose.position.x,
        eval_ego->pose.pose.position.y - last_evaluated_ego_->pose.pose.position.y) > jump_m_) {
      reset("localization_jump");
      return {true, false};
    }

    const auto signal = sample_at_or_before(signal_buffer_, t_eval.nanoseconds());
    if (signal) context_.signals(signal, t_eval);

    const auto snapshot = context_.build(*eval_ego, *route_, config_, t_eval);
    if (!snapshot.route) {
      ++dropped_;
      report("route_geometry_unavailable");
      return {false, false};
    }

    double tracked_age_sec = std::numeric_limits<double>::infinity();
    const auto tracked = sample_at_or_before(tracked_buffer_, t_eval.nanoseconds());
    if (tracked) tracked_age_sec = (t_eval - rclcpp::Time(tracked->header.stamp)).seconds();
    const bool tracked_valid =
      tracked && tracked_age_sec >= 0.0 && tracked_age_sec <= max_tracked_hold_sec_;
    if ((config_.static_safety_enabled || config_.dynamic_safety_enabled) && !tracked_valid) {
      report("waiting_for_buffered_tracked_objects");
      return {false, false};
    }

    internal_debug::begin_cycle();
    std::size_t admitted_now = 0;
    for (const auto & pair : ready) {
      const auto candidate_msg = pair.candidates;
      const auto prediction_msg = pair.prediction;
      if (
        !candidate_msg || !prediction_msg || !valid_candidate(*candidate_msg) ||
        prediction_msg->header.frame_id != "map") {
        ++dropped_;
        erase_sample(trajectory_buffer_, pair.stamp);
        erase_sample(prediction_buffer_, pair.stamp);
        continue;
      }
      const auto & candidate = candidate_msg->candidate_trajectories.front();
      const double age = (t_eval - rclcpp::Time(candidate.header.stamp)).seconds();
      if (age < 0.0 || age > max_age_) {
        ++dropped_;
        erase_sample(trajectory_buffer_, pair.stamp);
        erase_sample(prediction_buffer_, pair.stamp);
        continue;
      }

      Trajectory raw;
      raw.header = candidate.header;
      raw.points = candidate.points;
      ego_ = eval_ego;
      cb_compare_trace_trajectory_event("raw", "raw_diffusion", *ego_, raw, t_eval);
      manager_->add(
        raw, CandidateSource::LEGACY_OUTPUT, *prediction_msg, 0, snapshot.route,
        rclcpp::Time(candidate.header.stamp));
      admitted_generation_stamps_.insert(pair.stamp);
      ++admitted_generations_;
      ++admitted_now;
      if (pair.stamp >= latest_template_stamp_) {
        latest_template_stamp_ = pair.stamp;
        latest_candidate_template_ = candidate_msg;
        latest_prediction_ = prediction_msg;
      }
      erase_sample(trajectory_buffer_, pair.stamp);
      erase_sample(prediction_buffer_, pair.stamp);
    }
    prune_admitted(t_eval);
    if (admitted_now == 0) {
      report("no_valid_ready_generation");
      return {true, false};
    }

    ego_ = eval_ego;
    last_evaluated_ego_ = eval_ego;
    last_eval_stamp_ = t_eval;
    ++decision_cycles_;

    Prediction current_prediction;
    current_prediction.header.frame_id = "map";
    current_prediction.header.stamp = t_eval;
    if (latest_prediction_) {
      current_prediction = align_current_prediction_to_eval_
                             ? align_prediction_to_eval(*latest_prediction_, t_eval)
                             : *latest_prediction_;
    }

    if (manager_->evaluation_enabled()) {
      manager_->evaluate(
        *ego_, tracked_valid ? tracked : Tracked::ConstSharedPtr{}, current_prediction,
        snapshot.route, snapshot.conflicts, snapshot.rules, t_eval);
    } else {
      manager_->purge(*ego_, t_eval);
    }

    auto selected =
      manager_->selection_enabled() ? manager_->select(t_eval) : std::optional<Trajectory>{};
    const auto & d = manager_->debug_info();
    if (selected) {
      cb_compare_trace_trajectory_event(
        "selected", "new_selection", *ego_, *selected, t_eval, d.selected_candidate_id);
    }
    cb_compare_trace_bank_summary(
      selected ? "post_select" : "no_selection", *ego_, d, t_eval, admitted_now);

    if (!selected) {
      ++no_selection_suppressed_;
      publish_debug(snapshot, t_eval);
      report("no_selection_after_trajectory_trigger");
      return {true, false};
    }
    if (!latest_candidate_template_) {
      ++dropped_;
      publish_debug(snapshot, t_eval);
      report("selected_without_output_template");
      return {true, false};
    }

    last_selected_output_ = *selected;
    // Hold lifetime follows input-generation time; t_eval is newer Odometry.
    last_selected_decision_stamp_ = rclcpp::Time(ready.back().stamp);
    Trajectory published = *selected;
    published.header.stamp = now();
    cb_compare_trace_trajectory_event(
      "published", "standalone_selected_trajectory_triggered", *ego_, published, t_eval,
      d.selected_candidate_id);

    Candidates output = *latest_candidate_template_;
    output.candidate_trajectories.resize(1);
    output.candidate_trajectories[0].header = published.header;
    output.candidate_trajectories[0].points = published.points;
    pub_->publish(output);
    trajectory_pub_->publish(published);
    internal_debug::record_candidate_output_handoff(
      internal_debug::current_cycle_id(), true, d.selected_candidate_id, d.selected_source,
      "standalone_selected_trajectory_triggered", published, output.candidate_trajectories.size());
    ++processed_;
    ++selected_outputs_;
    last_success_ = now();
    publish_debug(snapshot, t_eval);
    report(
      tracked_age_sec > observation_age_ ? "selected_trajectory_triggered_with_held_tracked_objects"
                                         : "selected_trajectory_triggered");
    return {true, true};
  }

  bool republish_held_on_trajectory_input(const rclcpp::Time & trigger_stamp)
  {
    if (!last_selected_output_ || !last_selected_decision_stamp_ || !latest_candidate_template_)
      return false;
    const double hold_age = (trigger_stamp - *last_selected_decision_stamp_).seconds();
    if (hold_age < 0.0 || hold_age > hold_last_selected_sec_) return false;

    Trajectory published = *last_selected_output_;
    published.header.stamp = now();
    if (ego_) {
      cb_compare_trace_trajectory_event(
        "published", "held_on_trajectory_input", *ego_, published, trigger_stamp,
        manager_->debug_info().selected_candidate_id, hold_age);
      cb_compare_trace_bank_summary(
        "held_on_trajectory_input", *ego_, manager_->debug_info(), trigger_stamp, 0);
    }

    Candidates output = *latest_candidate_template_;
    output.candidate_trajectories.resize(1);
    output.candidate_trajectories[0].header = published.header;
    output.candidate_trajectories[0].points = published.points;
    pub_->publish(output);
    trajectory_pub_->publish(published);
    ++processed_;
    ++held_selected_outputs_;
    ++repeated_outputs_;
    last_success_ = now();
    report("held_selected_on_trajectory_input");
    return true;
  }

  void on_trajectory(Candidates::ConstSharedPtr msg)
  {
    const auto started = std::chrono::steady_clock::now();
    check_clock();
    ++trajectory_callbacks_;

    if (msg->candidate_trajectories.empty()) {
      ++dropped_;
      report("empty_candidates");
    } else {
      const int64_t stamp =
        rclcpp::Time(msg->candidate_trajectories.front().header.stamp).nanoseconds();
      if (stamp >= generation_floor_ && admitted_generation_stamps_.count(stamp) == 0U) {
        // Process previously buffered exact Candidate/Prediction pairs first.  This is normally
        // generation N-1 because DiffusionPlanner publishes CandidateTrajectories before
        // PredictedObjects within generation N.
        auto outcome = process_ready_generations(stamp);

        // Only after the previous decision has had a chance to run do we enqueue generation N.
        enqueue_trajectory(stamp, msg);

        // Handle unusual transport ordering where Prediction N arrived before Candidate N.
        if (!outcome.decision_ran) outcome = process_ready_generations(stamp);

        // If no new exact pair was ready, repeat the last selected output on the raw trajectory
        // input cadence.  An explicit no-selection decision remains fail-closed and is not
        // repeated.
        if (!outcome.decision_ran && !republish_held_on_trajectory_input(rclcpp::Time(stamp))) {
          report("waiting_for_ready_generation_on_trajectory_input");
        }
      }
    }

    last_callback_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    callback_total_ms_ += last_callback_ms_;
    max_callback_ms_ = std::max(max_callback_ms_, last_callback_ms_);
  }
  Marker line(
    const std::string & ns, int id, const rclcpp::Time & t, double width, float r, float g, float b)
  {
    Marker m;
    m.header.frame_id = "map";
    m.header.stamp = t;
    m.ns = ns;
    m.id = id;
    m.type = Marker::LINE_STRIP;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1;
    m.scale.x = width;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.85;
    return m;
  }

  void publish_debug(const RuleSnapshot & snapshot, const rclcpp::Time & t)
  {
    // CB_TRAJECTORY_TRIGGERED_ADAPTER_V01: when debug.publish_markers=false, skip both the
    // expensive cross-matrix JSON construction and RViz marker construction.
    if (!debug_) return;
    Json cells = Json::array();
    for (const auto & c : manager_->cross_collision_results())
      cells.push_back(
        {{"ego_id", c.ego_candidate_id},
         {"scenario_id", c.scenario_candidate_id},
         {"valid", c.valid},
         {"same_time_collision", c.same_time_collision},
         {"xyt_conflict", c.xyt_conflict},
         {"toi_sec", c.first_collision_time_sec},
         {"risk", c.scenario_risk}});
    std_msgs::msg::String matrix;
    matrix.data = Json({{"stamp_sec", t.seconds()}, {"cells", cells}}).dump();
    matrix_->publish(matrix);
    Markers out;
    Marker clear;
    clear.action = Marker::DELETEALL;
    out.markers.push_back(clear);
    const auto & d = manager_->debug_info();
    int id = 0;
    for (const auto & c : manager_->candidates()) {
      auto m = line(
        "candidates", id++, t, c.selected ? 0.20 : 0.06, c.safe ? 0.2f : 1.f, c.safe ? 0.4f : 0.1f,
        c.safe ? 1.f : 0.1f);
      if (c.selected) {
        m.color.r = 0;
        m.color.g = 1;
        m.color.b = 0;
      } else if (c.id == d.best_candidate_id) {
        m.color.r = 0;
        m.color.g = 1;
        m.color.b = 1;
      } else if (c.id == d.newest_candidate_id) {
        m.color.r = 1;
        m.color.g = 0.5;
        m.color.b = 0;
      }
      for (std::size_t i = c.live_start_index; i < c.trajectory_map.points.size(); ++i)
        m.points.push_back(c.trajectory_map.points[i].pose.position);
      out.markers.push_back(m);
      for (const auto & object : c.paired_predicted_objects.objects)
        for (const auto & path : object.kinematics.predicted_paths) {
          auto p = line("paired_predictions", id++, t, 0.035, 0.8, 0.5, 0.9);
          for (const auto & pose : path.path) p.points.push_back(pose.position);
          out.markers.push_back(p);
        }
    }
    for (const auto & obstacle : manager_->obstacle_geometry_debug()) {
      if (!obstacle.polygon.empty()) {
        auto m = line("obstacles", id++, t, 0.08, 1, 0.3, 0.2);
        m.points = obstacle.polygon;
        m.points.push_back(m.points.front());
        out.markers.push_back(m);
      }
      for (const auto & circle : obstacle.circles) {
        auto m = line("obstacle_circles", id++, t, 0.03, 1, 0.3, 0.2);
        for (int k = 0; k <= 32; ++k) {
          auto pt = circle.center;
          pt.x += circle.radius_m * std::cos(k * 2 * M_PI / 32);
          pt.y += circle.radius_m * std::sin(k * 2 * M_PI / 32);
          m.points.push_back(pt);
        }
        out.markers.push_back(m);
      }
    }
    if (snapshot.route) {
      auto m = line("route", id++, t, 0.035, 0.8, 0.8, 0.8);
      for (const auto & pt : snapshot.route->points) m.points.push_back(pt.pose.position);
      out.markers.push_back(m);
    }
    for (const auto & boundary : snapshot.rules.shared_lane_boundaries_map) {
      auto m = line("lane_boundaries", id++, t, 0.05, 1, 1, 0);
      m.points = boundary.points_map;
      out.markers.push_back(m);
    }
    Marker label;
    label.header.frame_id = "map";
    label.header.stamp = t;
    label.ns = "status";
    label.id = id++;
    label.type = Marker::TEXT_VIEW_FACING;
    label.action = Marker::ADD;
    label.pose = ego_->pose.pose;
    label.pose.position.z += 3;
    label.scale.z = 0.6;
    label.color.r = label.color.g = label.color.b = label.color.a = 1;
    label.text = "Bank=" + std::to_string(d.bank_size) +
                 " selected=" + std::to_string(d.selected_candidate_id) +
                 " signal=" + std::to_string(snapshot.rules.traffic_light_id) +
                 (snapshot.rules.traffic_signal_stop_active ? " STOP" : "");
    out.markers.push_back(label);
    markers_->publish(out);
  }

  CandidateBankConfig config_;
  RuleContext context_;
  std::unique_ptr<TrajectoryCandidateManager> manager_;
  // CB_TRAJECTORY_TRIGGERED_ADAPTER_V01: bounded timestamp buffers.
  std::deque<std::pair<int64_t, Candidates::ConstSharedPtr>> trajectory_buffer_;
  std::deque<std::pair<int64_t, Prediction::ConstSharedPtr>> prediction_buffer_;
  std::deque<std::pair<int64_t, Odometry::ConstSharedPtr>> odometry_buffer_;
  std::deque<std::pair<int64_t, Tracked::ConstSharedPtr>> tracked_buffer_;
  std::deque<std::pair<int64_t, Signal::ConstSharedPtr>> signal_buffer_;
  std::set<int64_t> admitted_generation_stamps_;
  Odometry::ConstSharedPtr ego_, last_evaluated_ego_;
  Route::ConstSharedPtr route_;
  Candidates::ConstSharedPtr latest_candidate_template_;
  Prediction::ConstSharedPtr latest_prediction_;
  std::optional<rclcpp::Time> last_clock_, last_success_, last_eval_stamp_,
    last_selected_decision_stamp_;
  std::optional<Trajectory> last_selected_output_;
  uint64_t processed_{0}, dropped_{0}, admitted_generations_{0}, selected_outputs_{0},
    held_selected_outputs_{0}, repeated_outputs_{0}, no_selection_suppressed_{0},
    trajectory_callbacks_{0}, trajectory_buffer_overwrites_{0}, pair_not_ready_on_trigger_{0},
    decision_cycles_{0};
  int64_t generation_floor_{0}, latest_template_stamp_{0};
  std::size_t state_buffer_size_{10}, trajectory_buffer_size_{1};
  double max_age_, observation_age_, future_tolerance_, jump_m_, evaluation_min_period_sec_,
    hold_last_selected_sec_, tracked_buffer_sec_, max_tracked_hold_sec_;
  double last_callback_ms_{0.0}, max_callback_ms_{0.0}, callback_total_ms_{0.0};
  bool evaluate_only_on_new_generation_{true}, one_generation_per_decision_cycle_{true},
    align_current_prediction_to_eval_{false}, batch_ready_generations_{false};
  std::string failure_;
  bool debug_;
  rclcpp::Publisher<Candidates>::SharedPtr pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Markers>::SharedPtr markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_, matrix_;
  rclcpp::Subscription<Candidates>::SharedPtr candidates_sub_;
  rclcpp::Subscription<Prediction>::SharedPtr prediction_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr ego_sub_;
  rclcpp::Subscription<Tracked>::SharedPtr objects_sub_;
  rclcpp::Subscription<Route>::SharedPtr route_sub_;
  rclcpp::Subscription<Map>::SharedPtr map_sub_;
  rclcpp::Subscription<Signal>::SharedPtr signal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
}  // namespace autoware::candidate_bank

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<autoware::candidate_bank::CandidateBankNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Candidate Bank startup failed: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
