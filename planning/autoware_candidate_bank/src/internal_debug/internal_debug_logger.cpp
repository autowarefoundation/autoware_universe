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
#include "autoware/candidate_bank/internal_debug/internal_debug_logger.hpp"

#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
namespace autoware::candidate_bank::internal_debug
{
namespace
{
std::atomic<std::uint64_t> g_next_cycle{1};
thread_local std::uint64_t g_cycle_id{0};
std::mutex g_write_mutex;
LoggerConfig g_config;
std::string timestamp_name()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream out;
  out << std::put_time(&local, "%Y%m%d_%H%M%S");
  return out.str();
}
double wall_time_sec()
{
  using Clock = std::chrono::system_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}
std::string escape_json(const std::string & value)
{
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
    }
  }
  return out.str();
}
void append_array(
  std::ostringstream & out, const std::vector<float> & values, const std::size_t limit)
{
  const std::size_t count = limit == 0 ? values.size() : std::min(values.size(), limit);
  out << '[';
  for (std::size_t i = 0; i < count; ++i) {
    if (i != 0) out << ',';
    if (std::isfinite(values[i]))
      out << std::setprecision(9) << values[i];
    else
      out << "null";
  }
  out << ']';
}
// CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN
void append_double(std::ostringstream & out, const double value)
{
  if (std::isfinite(value)) {
    out << std::setprecision(12) << value;
  } else {
    out << "null";
  }
}

void append_bool(std::ostringstream & out, const bool value)
{
  out << (value ? "true" : "false");
}

// CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
template <typename UUIDT>
std::string uuid_hex(const UUIDT & id)
{
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(id.uuid.size() * 2);
  for (const auto byte : id.uuid) {
    const auto value = static_cast<unsigned int>(byte);
    out.push_back(kHex[(value >> 4U) & 0x0FU]);
    out.push_back(kHex[value & 0x0FU]);
  }
  return out;
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
// CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END
// CANDIDATE_BANK_DIAGNOSTICS_V2_END
void write_line(const std::string & line)
{
  if (!g_config.enabled) return;
  std::lock_guard<std::mutex> lock(g_write_mutex);
  std::filesystem::create_directories(g_config.output_directory);
  std::ofstream stream(
    std::filesystem::path(g_config.output_directory) / "internal_events.jsonl",
    std::ios::out | std::ios::app);
  stream << line << '\n';
}
std::string event_prefix(const std::uint64_t cycle_id, const std::string & event)
{
  std::ostringstream out;
  out << "{\"schema_version\":7"
      << ",\"cycle_id\":" << cycle_id << ",\"wall_time_sec\":" << std::setprecision(16)
      << wall_time_sec() << ",\"thread_id\":\"" << std::this_thread::get_id() << "\""
      << ",\"event\":\"" << escape_json(event) << "\"";
  return out.str();
}
}  // namespace
void configure(const LoggerConfig & requested)
{
  g_config = requested;
  g_config.stage_stride = std::max(1, g_config.stage_stride);
  if (g_config.output_root.empty()) g_config.output_root = "/tmp/diffusion_planner_internal_debug";
  if (g_config.run_name.empty() || g_config.run_name == "auto")
    g_config.run_name = timestamp_name();
  g_config.output_directory =
    (std::filesystem::path(g_config.output_root) / g_config.run_name).string();
  g_next_cycle.store(1);
  g_cycle_id = 0;
  if (!g_config.enabled) return;
  std::filesystem::create_directories(g_config.output_directory);
  std::ofstream info(
    std::filesystem::path(g_config.output_directory) / "run_information.json",
    std::ios::out | std::ios::trunc);
  info << "{\n  \"schema_version\": 7,\n  \"run_name\": \"" << escape_json(g_config.run_name)
       << "\",\n  \"output_directory\": \"" << escape_json(g_config.output_directory) << "\"\n}\n";
}
const LoggerConfig & config()
{
  return g_config;
}
void write_effective_parameters(const std::string & yaml_text)
{
  if (!g_config.enabled || !g_config.write_effective_parameters) return;
  std::lock_guard<std::mutex> lock(g_write_mutex);
  std::ofstream stream(
    std::filesystem::path(g_config.output_directory) / "effective_parameters.yaml",
    std::ios::out | std::ios::trunc);
  stream << yaml_text;
}
std::uint64_t begin_cycle()
{
  if (!g_config.enabled) return 0;
  g_cycle_id = g_next_cycle.fetch_add(1);
  write_line(event_prefix(g_cycle_id, "cycle_begin") + "}");
  return g_cycle_id;
}
std::uint64_t current_cycle_id()
{
  return g_cycle_id;
}
void record_input_map(
  const std::uint64_t cycle_id, const NamedFloatArrays & input_data_map,
  const std::vector<float> & state_mean, const std::vector<float> & state_std)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "model_input") << ",\"state_mean\":";
  append_array(out, state_mean, g_config.max_array_values);
  out << ",\"state_std\":";
  append_array(out, state_std, g_config.max_array_values);
  out << ",\"tensors\":{";
  bool first = true;
  for (const auto & [name, values] : input_data_map) {
    if (!first) out << ',';
    first = false;
    out << '"' << escape_json(name) << "\":{\"size\":" << values.size() << ",\"values\":";
    append_array(out, values, g_config.max_array_values);
    out << '}';
  }
  out << "}}";
  write_line(out.str());
}
void record_solver_initial(
  const std::uint64_t cycle_id, const std::vector<float> & state, const float start_t,
  const int steps)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "solver_initial") << ",\"start_t\":" << start_t
      << ",\"steps\":" << steps << ",\"state_size\":" << state.size() << ",\"state\":";
  append_array(out, state, g_config.max_array_values);
  out << '}';
  write_line(out.str());
}
void record_solver_stage(
  const std::uint64_t cycle_id, const int step, const float time, const std::vector<float> & state)
{
  if (!g_config.enabled || cycle_id == 0 || step % g_config.stage_stride != 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "solver_stage") << ",\"step\":" << step << ",\"time\":" << time
      << ",\"state_size\":" << state.size() << ",\"state\":";
  append_array(out, state, g_config.max_array_values);
  out << '}';
  write_line(out.str());
}
void record_solver_final(const std::uint64_t cycle_id, const std::vector<float> & state)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "solver_final") << ",\"state_size\":" << state.size()
      << ",\"state\":";
  append_array(out, state, g_config.max_array_values);
  out << '}';
  write_line(out.str());
}
void record_note(const std::uint64_t cycle_id, const std::string & event, const std::string & text)
{
  if (!g_config.enabled || cycle_id == 0) return;
  write_line(event_prefix(cycle_id, event) + ",\"text\":\"" + escape_json(text) + "\"}");
}
void record_mid_replacement(
  const std::uint64_t cycle_id, const std::string & mode, const bool applied,
  const float configured_t, const float actual_t, const float beta, const std::string & reason)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "ego_mid_replacement") << ",\"configured_mode\":\""
      << escape_json(mode) << "\""
      << ",\"applied\":" << (applied ? "true" : "false")
      << ",\"replace_t_configured\":" << configured_t << ",\"replace_t_actual\":" << actual_t
      << ",\"beta\":" << beta << ",\"skip_reason\":\"" << escape_json(reason) << "\"}";
  write_line(out.str());
}

// CANDIDATE_BANK_DIAGNOSTICS_V2_BEGIN
void record_candidate_generated(
  const std::uint64_t cycle_id, const std::uint64_t candidate_id, const std::string & source,
  const autoware_planning_msgs::msg::Trajectory & trajectory_map)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_generated") << ",\"candidate_id\":" << candidate_id
      << ",\"source\":\"" << escape_json(source) << "\""
      << ",\"frame_id\":\"" << escape_json(trajectory_map.header.frame_id) << "\""
      << ",\"point_count\":" << trajectory_map.points.size() << ",\"trajectory_xyvt\":[";
  for (std::size_t i = 0; i < trajectory_map.points.size(); ++i) {
    if (i != 0) out << ',';
    const auto & point = trajectory_map.points[i];
    out << '[';
    append_double(out, point.pose.position.x);
    out << ',';
    append_double(out, point.pose.position.y);
    out << ',';
    append_double(out, point.longitudinal_velocity_mps);
    out << ',';
    append_double(out, rclcpp::Duration(point.time_from_start).seconds());
    out << ']';
  }
  out << "]}";
  write_line(out.str());
}

// CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_BEGIN
void record_candidate_joint_prediction(
  const std::uint64_t cycle_id, const std::uint64_t candidate_id, const std::string & source,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const std::int64_t batch_index)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_joint_prediction")
      << ",\"candidate_id\":" << candidate_id << ",\"source\":\"" << escape_json(source) << "\""
      << ",\"batch_index\":" << batch_index << ",\"frame_id\":\""
      << escape_json(predicted_objects.header.frame_id) << "\""
      << ",\"object_count\":" << predicted_objects.objects.size() << ",\"objects\":[";
  for (std::size_t object_index = 0; object_index < predicted_objects.objects.size();
       ++object_index) {
    if (object_index != 0) out << ',';
    const auto & object = predicted_objects.objects[object_index];
    const auto & initial_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & twist = object.kinematics.initial_twist_with_covariance.twist.linear;
    out << '{';
    out << "\"object_index\":" << object_index;
    out << ",\"object_id\":\"" << uuid_hex(object.object_id) << "\"";
    out << ",\"initial_x\":";
    append_double(out, initial_pose.position.x);
    out << ",\"initial_y\":";
    append_double(out, initial_pose.position.y);
    out << ",\"initial_yaw\":";
    append_double(out, quaternion_yaw(initial_pose.orientation));
    out << ",\"speed_mps\":";
    append_double(out, std::hypot(twist.x, twist.y));
    out << ",\"shape_type\":" << static_cast<int>(object.shape.type);
    out << ",\"length_m\":";
    append_double(out, object.shape.dimensions.x);
    out << ",\"width_m\":";
    append_double(out, object.shape.dimensions.y);
    out << ",\"footprint_xy\":[";
    for (std::size_t point_index = 0; point_index < object.shape.footprint.points.size();
         ++point_index) {
      if (point_index != 0) out << ',';
      const auto & point = object.shape.footprint.points[point_index];
      out << '[';
      append_double(out, point.x);
      out << ',';
      append_double(out, point.y);
      out << ']';
    }
    out << "],\"paths\":[";
    for (std::size_t path_index = 0; path_index < object.kinematics.predicted_paths.size();
         ++path_index) {
      if (path_index != 0) out << ',';
      const auto & path = object.kinematics.predicted_paths[path_index];
      out << '{';
      out << "\"path_index\":" << path_index;
      out << ",\"confidence\":";
      append_double(out, path.confidence);
      out << ",\"time_step_sec\":";
      append_double(out, rclcpp::Duration(path.time_step).seconds());
      out << ",\"point_count\":" << path.path.size();
      out << ",\"path_xyyaw\":[";
      for (std::size_t i = 0; i < path.path.size(); ++i) {
        if (i != 0) out << ',';
        const auto & pose = path.path[i];
        out << '[';
        append_double(out, pose.position.x);
        out << ',';
        append_double(out, pose.position.y);
        out << ',';
        append_double(out, quaternion_yaw(pose.orientation));
        out << ']';
      }
      out << "]}";
    }
    out << "]}";
  }
  out << "]}";
  write_line(out.str());
}
// CANDIDATE_BANK_PAIRED_JOINT_PREDICTION_V1_END

// CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_BEGIN
void record_candidate_cross_collision(
  const std::uint64_t cycle_id, const CandidateCrossCollisionLog & c)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_cross_collision")
      << ",\"ego_candidate_id\":" << c.ego_candidate_id
      << ",\"scenario_candidate_id\":" << c.scenario_candidate_id << ",\"ego_source\":\""
      << escape_json(c.ego_source) << "\""
      << ",\"scenario_source\":\"" << escape_json(c.scenario_source) << "\""
      << ",\"paired\":";
  append_bool(out, c.paired);
  out << ",\"scenario_age_sec\":";
  append_double(out, c.scenario_age_sec);
  out << ",\"valid\":";
  append_bool(out, c.valid);
  out << ",\"same_time_collision\":";
  append_bool(out, c.same_time_collision);
  out << ",\"xyt_conflict\":";
  append_bool(out, c.xyt_conflict);
  out << ",\"trajectory_conflict\":";
  append_bool(out, c.trajectory_conflict);
  // CANDIDATE_BANK_TOI_V1_BEGIN
  out << ",\"first_collision_time_sec\":";
  append_double(out, c.first_collision_time_sec);
  out << ",\"first_collision_object_id\":\"" << escape_json(c.first_collision_object_id) << "\""
      << ",\"first_collision_path_index\":" << c.first_collision_path_index;
  out << ",\"first_collision_path_confidence\":";
  append_double(out, c.first_collision_path_confidence);
  out << ",\"first_collision_ego_circle_index\":" << c.first_collision_ego_circle_index
      << ",\"first_collision_object_circle_index\":" << c.first_collision_object_circle_index;
  // CANDIDATE_BANK_TOI_V1_END
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
  out << ",\"scenario_weight\":";
  append_double(out, c.scenario_weight);
  out << ",\"scenario_risk\":";
  append_double(out, c.scenario_risk);
  // CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END
  out << ",\"min_same_time_clearance_m\":";
  append_double(out, c.min_same_time_clearance_m);
  out << ",\"min_xyt_rho\":";
  append_double(out, c.min_xyt_rho);
  out << ",\"xyt_spatial_distance_m\":";
  append_double(out, c.xyt_spatial_distance_m);
  out << ",\"xyt_spatial_clearance_m\":";
  append_double(out, c.xyt_spatial_clearance_m);
  out << ",\"xyt_ego_time_sec\":";
  append_double(out, c.xyt_ego_time_sec);
  out << ",\"xyt_object_time_sec\":";
  append_double(out, c.xyt_object_time_sec);
  out << ",\"xyt_delta_time_sec\":";
  append_double(out, c.xyt_delta_time_sec);
  out << ",\"object_id\":\"" << escape_json(c.object_id) << "\""
      << ",\"path_index\":" << c.path_index;
  out << ",\"path_confidence\":";
  append_double(out, c.path_confidence);
  out << ",\"ego_circle_index\":" << c.ego_circle_index
      << ",\"object_circle_index\":" << c.object_circle_index
      << ",\"valid_path_count\":" << c.valid_path_count << ",\"stats\":{"
      << "\"path_tests\":" << c.path_tests << ",\"path_rejects\":" << c.path_rejects
      << ",\"block_tests\":" << c.block_tests << ",\"block_rejects\":" << c.block_rejects
      << ",\"interval_tests\":" << c.interval_tests
      << ",\"interval_rejects\":" << c.interval_rejects
      << ",\"circle_aabb_tests\":" << c.circle_aabb_tests
      << ",\"circle_aabb_rejects\":" << c.circle_aabb_rejects
      << ",\"xyt_segment_tests\":" << c.xyt_segment_tests
      << ",\"same_time_interval_tests\":" << c.same_time_interval_tests
      << ",\"same_time_circle_tests\":" << c.same_time_circle_tests << '}';
  out << ",\"evaluation_us\":";
  append_double(out, c.evaluation_us);
  out << '}';
  write_line(out.str());
}

// CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_BEGIN
void record_candidate_cross_aggregate(
  const std::uint64_t cycle_id, const CandidateCrossAggregateLog & a)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_cross_aggregate")
      << ",\"candidate_id\":" << a.candidate_id << ",\"source\":\"" << escape_json(a.source) << "\""
      << ",\"weight_mode\":\"" << escape_json(a.weight_mode) << "\"";
  out << ",\"aggregation_enabled\":";
  append_bool(out, a.aggregation_enabled);
  out << ",\"apply_to_score\":";
  append_bool(out, a.apply_to_score);
  out << ",\"apply_short_horizon_hard_reject\":";
  append_bool(out, a.apply_short_horizon_hard_reject);
  out << ",\"score_weight\":";
  append_double(out, a.score_weight);
  out << ",\"short_horizon_sec\":";
  append_double(out, a.short_horizon_sec);
  out << ",\"scenario_count\":" << a.scenario_count
      << ",\"valid_scenario_count\":" << a.valid_scenario_count
      << ",\"collision_scenario_count\":" << a.collision_scenario_count
      << ",\"xyt_conflict_scenario_count\":" << a.xyt_conflict_scenario_count
      << ",\"short_horizon_collision_scenario_count\":" << a.short_horizon_collision_scenario_count;
  out << ",\"scenario_weight_sum\":";
  append_double(out, a.scenario_weight_sum);
  out << ",\"collision_scenario_fraction\":";
  append_double(out, a.collision_scenario_fraction);
  out << ",\"xyt_conflict_scenario_fraction\":";
  append_double(out, a.xyt_conflict_scenario_fraction);
  out << ",\"min_toi_sec\":";
  append_double(out, a.min_toi_sec);
  out << ",\"mean_collision_toi_sec\":";
  append_double(out, a.mean_collision_toi_sec);
  out << ",\"min_same_time_clearance_m\":";
  append_double(out, a.min_same_time_clearance_m);
  out << ",\"paired_risk\":";
  append_double(out, a.paired_risk);
  out << ",\"off_diagonal_mean_risk\":";
  append_double(out, a.off_diagonal_mean_risk);
  out << ",\"mean_scenario_risk\":";
  append_double(out, a.mean_scenario_risk);
  out << ",\"score_before_cross\":";
  append_double(out, a.score_before_cross);
  out << ",\"score_penalty\":";
  append_double(out, a.score_penalty);
  out << ",\"score_after_cross\":";
  append_double(out, a.score_after_cross);
  out << ",\"selector_effective_score\":";
  append_double(out, a.selector_effective_score);
  out << ",\"base_safe\":";
  append_bool(out, a.base_safe);
  out << ",\"short_horizon_hard_reject\":";
  append_bool(out, a.short_horizon_hard_reject);
  out << ",\"selector_score_enabled\":";
  append_bool(out, a.selector_score_enabled);
  out << ",\"selector_hard_rejected\":";
  append_bool(out, a.selector_hard_rejected);
  out << ",\"selector_eligible\":";
  append_bool(out, a.selector_eligible);
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
  out << ",\"safety_tier\":\"" << escape_json(a.safety_tier) << "\"";
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
  out << ",\"shadow_best\":";
  append_bool(out, a.shadow_best);
  out << '}';
  write_line(out.str());
}
// CANDIDATE_BANK_CONTINGENCY_AGGREGATION_V1_END

// CANDIDATE_BANK_XYT_CROSS_COLLISION_V1_END

void record_candidate_observation(
  const std::uint64_t cycle_id, const double ego_x, const double ego_y, const double ego_yaw,
  const std::string & collision_geometry_mode,
  const std::vector<CandidateObstaclePointLog> & ego_polygon,
  const std::vector<CandidateObstacleLog> & obstacles)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_observation") << ",\"collision_geometry_mode\":\""
      << escape_json(collision_geometry_mode) << "\""
      << ",\"ego\":{";
  out << "\"x\":";
  append_double(out, ego_x);
  out << ",\"y\":";
  append_double(out, ego_y);
  out << ",\"yaw\":";
  append_double(out, ego_yaw);
  out << ",\"collision_polygon\":[";
  for (std::size_t i = 0; i < ego_polygon.size(); ++i) {
    if (i != 0) out << ',';
    out << '[';
    append_double(out, ego_polygon[i].x);
    out << ',';
    append_double(out, ego_polygon[i].y);
    out << ']';
  }
  out << "]},\"objects\":[";
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    if (i != 0) out << ',';
    const auto & object = obstacles[i];
    out << '{';
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
    out << "\"object_id\":\"" << escape_json(object.object_id) << "\"";
    out << ",\"x\":";
    append_double(out, object.x);
    // CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END
    out << ",\"y\":";
    append_double(out, object.y);
    out << ",\"radius_m\":";
    append_double(out, object.radius_m);
    out << ",\"yaw\":";
    append_double(out, object.yaw);
    out << ",\"length_m\":";
    append_double(out, object.length_m);
    out << ",\"width_m\":";
    append_double(out, object.width_m);
    out << ",\"shape_type\":" << object.shape_type;
    out << ",\"collision_circles\":[";
    for (std::size_t j = 0; j < object.collision_circles.size(); ++j) {
      if (j != 0) out << ',';
      const auto & circle = object.collision_circles[j];
      out << '{';
      out << "\"x\":";
      append_double(out, circle.x);
      out << ",\"y\":";
      append_double(out, circle.y);
      out << ",\"radius_m\":";
      append_double(out, circle.radius_m);
      out << '}';
    }
    out << ']';
    out << ",\"collision_polygon\":[";
    for (std::size_t j = 0; j < object.collision_polygon.size(); ++j) {
      if (j != 0) out << ',';
      out << '[';
      append_double(out, object.collision_polygon[j].x);
      out << ',';
      append_double(out, object.collision_polygon[j].y);
      out << ']';
    }
    out << ']';
    out << ",\"speed_mps\":";
    append_double(out, object.speed_mps);
    out << ",\"is_static\":";
    append_bool(out, object.is_static);
    out << '}';
  }
  out << "]}";
  write_line(out.str());
}

// CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
void record_candidate_map_context(
  const std::uint64_t cycle_id, const CandidateMapContextLog & context)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_map_context") << ",\"valid\":";
  append_bool(out, context.valid);
  out << ",\"forward_search_distance_m\":";
  append_double(out, context.forward_search_distance_m);
  out << ",\"backtrace_distance_m\":";
  append_double(out, context.backtrace_distance_m);
  out << ",\"lanes\":[";
  for (std::size_t i = 0; i < context.lanes.size(); ++i) {
    if (i != 0) out << ',';
    const auto & lane = context.lanes[i];
    out << '{';
    out << "\"lane_id\":" << lane.lane_id;
    out << ",\"is_route_lane\":";
    append_bool(out, lane.is_route_lane);
    out << ",\"is_competing_lane\":";
    append_bool(out, lane.is_competing_lane);
    out << ",\"route_s_start_m\":";
    append_double(out, lane.route_s_start_m);
    out << ",\"centerline_xy\":[";
    for (std::size_t j = 0; j < lane.centerline.size(); ++j) {
      if (j != 0) out << ',';
      out << '[';
      append_double(out, lane.centerline[j].x);
      out << ',';
      append_double(out, lane.centerline[j].y);
      out << ']';
    }
    out << "]}";
  }
  out << "],\"conflicts\":[";
  for (std::size_t i = 0; i < context.conflicts.size(); ++i) {
    if (i != 0) out << ',';
    const auto & conflict = context.conflicts[i];
    out << '{';
    out << "\"ego_route_lane_id\":" << conflict.ego_route_lane_id;
    out << ",\"competing_lane_id\":" << conflict.competing_lane_id;
    out << ",\"x\":";
    append_double(out, conflict.x);
    out << ",\"y\":";
    append_double(out, conflict.y);
    out << ",\"ego_route_distance_m\":";
    append_double(out, conflict.ego_route_distance_m);
    out << ",\"approach_lane_ids\":[";
    for (std::size_t j = 0; j < conflict.approach_lane_ids.size(); ++j) {
      if (j != 0) out << ',';
      out << conflict.approach_lane_ids[j];
    }
    out << "]}";
  }
  out << "]}";
  write_line(out.str());
}
// CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END

// CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_BEGIN
namespace
{
void append_static_clearance_witness(
  std::ostringstream & out, const CandidateStaticClearanceWitnessLog & w)
{
  out << '{';
  out << "\"valid\":";
  append_bool(out, w.valid);
  out << ",\"object_id\":\"" << escape_json(w.object_id) << "\"";
  out << ",\"object_speed_mps\":";
  append_double(out, w.object_speed_mps);
  out << ",\"trajectory_point_index\":" << w.trajectory_point_index;
  out << ",\"live_point_index\":" << w.live_point_index;
  out << ",\"future_time_sec\":";
  append_double(out, w.future_time_sec);
  out << ",\"signed_clearance_m\":";
  append_double(out, w.signed_clearance_m);
  out << ",\"margin_m\":";
  append_double(out, w.margin_m);
  out << ",\"deficit_m\":";
  append_double(out, w.deficit_m);
  out << ",\"ego_pose\":{";
  out << "\"x\":";
  append_double(out, w.ego_x);
  out << ",\"y\":";
  append_double(out, w.ego_y);
  out << ",\"yaw\":";
  append_double(out, w.ego_yaw);
  out << '}';
  out << ",\"object_pose\":{";
  out << "\"x\":";
  append_double(out, w.object_x);
  out << ",\"y\":";
  append_double(out, w.object_y);
  out << ",\"yaw\":";
  append_double(out, w.object_yaw);
  out << '}';
  out << ",\"ego_radius_m\":";
  append_double(out, w.ego_radius_m);
  out << ",\"ego_polygon\":[";
  for (std::size_t i = 0; i < w.ego_polygon.size(); ++i) {
    if (i != 0) out << ',';
    out << '[';
    append_double(out, w.ego_polygon[i].x);
    out << ',';
    append_double(out, w.ego_polygon[i].y);
    out << ']';
  }
  out << ']';
  out << ",\"object_circles\":[";
  for (std::size_t i = 0; i < w.object_circles.size(); ++i) {
    if (i != 0) out << ',';
    out << '{';
    out << "\"x\":";
    append_double(out, w.object_circles[i].x);
    out << ",\"y\":";
    append_double(out, w.object_circles[i].y);
    out << ",\"radius_m\":";
    append_double(out, w.object_circles[i].radius_m);
    out << '}';
  }
  out << ']';
  out << ",\"object_polygon\":[";
  for (std::size_t i = 0; i < w.object_polygon.size(); ++i) {
    if (i != 0) out << ',';
    out << '[';
    append_double(out, w.object_polygon[i].x);
    out << ',';
    append_double(out, w.object_polygon[i].y);
    out << ']';
  }
  out << "]}";
}
}  // namespace

void record_candidate_static_clearance(
  const std::uint64_t cycle_id, const CandidateStaticClearanceLog & c)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_static_clearance")
      << ",\"candidate_id\":" << c.candidate_id << ",\"source\":\"" << escape_json(c.source) << "\""
      << ",\"collision_geometry_mode\":\"" << escape_json(c.collision_geometry_mode) << "\"";
  out << ",\"argmin\":";
  append_static_clearance_witness(out, c.argmin);
  out << ",\"first_margin_violation\":";
  append_static_clearance_witness(out, c.first_margin_violation);
  out << ",\"first_overlap\":";
  append_static_clearance_witness(out, c.first_overlap);
  out << '}';
  write_line(out.str());
}
// CANDIDATE_BANK_STATIC_CLEARANCE_DEBUG_V1_END

void record_candidate_evaluation(const std::uint64_t cycle_id, const CandidateEvaluationLog & e)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_evaluation") << ",\"candidate_id\":" << e.candidate_id
      << ",\"source\":\"" << escape_json(e.source) << "\""
      << ",\"was_incumbent\":";
  append_bool(out, e.was_incumbent);
  out << ",\"valid\":";
  append_bool(out, e.valid);
  out << ",\"safe\":";
  append_bool(out, e.safe);
  // CANDIDATE_BANK_SOFT_COLLISION_V1_BEGIN
  out << ",\"static_collision_margin_violated\":";
  append_bool(out, e.static_collision_margin_violated);
  out << ",\"dynamic_collision_margin_violated\":";
  append_bool(out, e.dynamic_collision_margin_violated);
  // CANDIDATE_BANK_SOFT_COLLISION_V1_END
  out << ",\"live_start_index\":" << e.live_start_index;
  out << ",\"age_sec\":";
  append_double(out, e.age_sec);
  out << ",\"projection_error_m\":";
  append_double(out, e.projection_error_m);
  out << ",\"remaining_horizon_sec\":";
  append_double(out, e.remaining_horizon_sec);
  out << ",\"path_length_m\":";
  append_double(out, e.path_length_m);
  out << ",\"progress_m\":";
  append_double(out, e.progress_m);
  out << ",\"route_deviation_rms_m\":";
  append_double(out, e.route_deviation_rms_m);
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
  out << ",\"route_reference_weight_mean\":";
  append_double(out, e.route_reference_weight_mean);
  out << ",\"route_reference_max_roughness_m\":";
  append_double(out, e.route_reference_max_roughness_m);
  out << ",\"traffic_signal_stop_active\":";
  append_bool(out, e.traffic_signal_stop_active);
  out << ",\"traffic_signal_crossed\":";
  append_bool(out, e.traffic_signal_crossed);
  out << ",\"traffic_signal_max_stop_zone_speed_mps\":";
  append_double(out, e.traffic_signal_max_stop_zone_speed_mps);
  out << ",\"traffic_rule_cost\":";
  append_double(out, e.traffic_rule_cost);
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
  out << ",\"traffic_signal_scope_mode\":\"" << escape_json(e.traffic_signal_scope_mode) << "\"";
  out << ",\"current_lane_valid\":";
  append_bool(out, e.current_lane_valid);
  out << ",\"current_lane_id\":" << e.current_lane_id;
  out << ",\"max_continuous_straddling_sec\":";
  append_double(out, e.max_continuous_straddling_sec);
  out << ",\"terminal_straddling\":";
  append_bool(out, e.terminal_straddling);
  out << ",\"prolonged_straddling_cost\":";
  append_double(out, e.prolonged_straddling_cost);
  out << ",\"terminal_straddling_cost\":";
  append_double(out, e.terminal_straddling_cost);
  out << ",\"lane_boundary_cost\":";
  append_double(out, e.lane_boundary_cost);
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
  out << ",\"lane_change_deadline_active\":";
  append_bool(out, e.lane_change_deadline_active);
  out << ",\"lane_change_attempted\":";
  append_bool(out, e.lane_change_attempted);
  out << ",\"lane_change_completed\":";
  append_bool(out, e.lane_change_completed);
  out << ",\"lane_change_completed_before_deadline\":";
  append_bool(out, e.lane_change_completed_before_deadline);
  out << ",\"lane_change_fallback_stop_satisfied\":";
  append_bool(out, e.lane_change_fallback_stop_satisfied);
  out << ",\"lane_change_boundary_id\":" << e.lane_change_boundary_id;
  out << ",\"lane_change_target_lane_id\":" << e.lane_change_target_lane_id;
  out << ",\"lane_change_deadline_s_m\":";
  append_double(out, e.lane_change_deadline_s_m);
  out << ",\"lane_change_completion_s_m\":";
  append_double(out, e.lane_change_completion_s_m);
  out << ",\"lane_change_slack_m\":";
  append_double(out, e.lane_change_slack_m);
  out << ",\"lane_change_deadline_cost\":";
  append_double(out, e.lane_change_deadline_cost);
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
  out << ",\"comfort_cost\":";
  append_double(out, e.comfort_cost);
  out << ",\"min_static_clearance_m\":";
  append_double(out, e.min_static_clearance_m);
  out << ",\"min_dynamic_clearance_m\":";
  append_double(out, e.min_dynamic_clearance_m);
  out << ",\"min_dynamic_time_gap_sec\":";
  append_double(out, e.min_dynamic_time_gap_sec);
  // CANDIDATE_BANK_TTC_COST_V1_BEGIN
  out << ",\"dynamic_temporal_cost_mode\":\"" << escape_json(e.dynamic_temporal_cost_mode) << "\"";
  out << ",\"min_dynamic_ttc_sec\":";
  append_double(out, e.min_dynamic_ttc_sec);
  out << ",\"dynamic_clearance_soft_cost\":";
  append_double(out, e.dynamic_clearance_soft_cost);
  out << ",\"dynamic_legacy_time_gap_soft_cost\":";
  append_double(out, e.dynamic_legacy_time_gap_soft_cost);
  out << ",\"dynamic_ttc_soft_cost\":";
  append_double(out, e.dynamic_ttc_soft_cost);
  // CANDIDATE_BANK_TTC_COST_V1_END
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_BEGIN
  out << ",\"dynamic_soft_cost_aggregation_mode\":\""
      << escape_json(e.dynamic_soft_cost_aggregation_mode) << "\"";
  out << ",\"dynamic_uncertainty_mode\":\"" << escape_json(e.dynamic_uncertainty_mode) << "\"";
  out << ",\"max_dynamic_closing_rate_mps\":";
  append_double(out, e.max_dynamic_closing_rate_mps);
  out << ",\"max_dynamic_uncertainty_margin_m\":";
  append_double(out, e.max_dynamic_uncertainty_margin_m);
  out << ",\"dynamic_prediction_path_count\":" << e.dynamic_prediction_path_count;
  out << ",\"dynamic_prediction_confidence_sum\":";
  append_double(out, e.dynamic_prediction_confidence_sum);
  // CANDIDATE_BANK_DYNAMIC_NORMALIZATION_V1_END
  out << ",\"score\":";
  append_double(out, e.score);
  out << ",\"score_terms\":{";
  out << "\"progress\":";
  append_double(out, e.score_progress);
  out << ",\"route_deviation\":";
  append_double(out, e.score_route_deviation);
  out << ",\"path_efficiency\":";
  append_double(out, e.score_path_efficiency);
  out << ",\"static_clearance\":";
  append_double(out, e.score_static_clearance);
  out << ",\"dynamic_margin\":";
  append_double(out, e.score_dynamic_margin);
  out << ",\"comfort\":";
  append_double(out, e.score_comfort);
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_BEGIN
  out << ",\"traffic_rules\":";
  append_double(out, e.score_traffic_rules);
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_BEGIN
  out << ",\"lane_boundary\":";
  append_double(out, e.score_lane_boundary);
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_BEGIN
  out << ",\"lane_change_deadline\":";
  append_double(out, e.score_lane_change_deadline);
  // CANDIDATE_BANK_LANE_CHANGE_DEADLINE_V1_END
  // CANDIDATE_BANK_CURRENT_LANE_RULE_V1_END
  // CANDIDATE_BANK_ROUTE_RULE_COST_V1_END
  out << "}}";
  write_line(out.str());
}

// CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_BEGIN
void record_candidate_dynamic_analysis(
  const std::uint64_t cycle_id, const CandidateDynamicAnalysisLog & e)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_dynamic_analysis") << ",\"role\":\""
      << escape_json(e.role) << "\""
      << ",\"candidate_id\":" << e.candidate_id << ",\"source\":\"" << escape_json(e.source) << "\""
      << ",\"safe\":";
  append_bool(out, e.safe);
  out << ",\"progress_m\":";
  append_double(out, e.progress_m);
  out << ",\"score\":";
  append_double(out, e.score);
  out << ",\"score_dynamic_margin\":";
  append_double(out, e.score_dynamic_margin);
  out << ",\"candidate_dynamic_clearance_soft_cost\":";
  append_double(out, e.candidate_dynamic_clearance_soft_cost);
  out << ",\"candidate_dynamic_ttc_soft_cost\":";
  append_double(out, e.candidate_dynamic_ttc_soft_cost);
  out << ",\"candidate_min_dynamic_clearance_m\":";
  append_double(out, e.candidate_min_dynamic_clearance_m);
  out << ",\"candidate_min_dynamic_ttc_sec\":";
  append_double(out, e.candidate_min_dynamic_ttc_sec);
  out << ",\"collision_margin_m\":";
  append_double(out, e.collision_margin_m);
  out << ",\"preferred_clearance_m\":";
  append_double(out, e.preferred_clearance_m);
  out << ",\"ttc_threshold_sec\":";
  append_double(out, e.ttc_threshold_sec);
  out << ",\"detail_valid\":";
  append_bool(out, e.detail_valid);
  out << ",\"object_id\":\"" << escape_json(e.object_id) << "\"";
  out << ",\"path_index\":" << e.path_index;
  out << ",\"is_fallback\":";
  append_bool(out, e.is_fallback);
  out << ",\"path_confidence\":";
  append_double(out, e.path_confidence);
  out << ",\"normalized_confidence\":";
  append_double(out, e.normalized_confidence);
  out << ",\"path_clearance_soft_cost\":";
  append_double(out, e.path_clearance_soft_cost);
  out << ",\"path_ttc_soft_cost\":";
  append_double(out, e.path_ttc_soft_cost);
  out << ",\"path_total_soft_cost\":";
  append_double(out, e.path_total_soft_cost);
  out << ",\"ttc_sec\":";
  append_double(out, e.ttc_sec);
  out << ",\"collision_margin_violated\":";
  append_bool(out, e.collision_margin_violated);
  out << ",\"worst_sample_index\":" << e.worst_sample_index;
  out << ",\"dynamic_scope_mode\":\"" << escape_json(e.dynamic_scope_mode) << "\"";
  out << ",\"samples\":[";
  for (std::size_t i = 0; i < e.samples.size(); ++i) {
    if (i != 0) out << ',';
    const auto & sample = e.samples[i];
    out << '{';
    out << "\"time_sec\":";
    append_double(out, sample.time_sec);
    out << ",\"ego_x\":";
    append_double(out, sample.ego_x);
    out << ",\"ego_y\":";
    append_double(out, sample.ego_y);
    out << ",\"ego_yaw\":";
    append_double(out, sample.ego_yaw);
    out << ",\"object_x\":";
    append_double(out, sample.object_x);
    out << ",\"object_y\":";
    append_double(out, sample.object_y);
    out << ",\"object_yaw\":";
    append_double(out, sample.object_yaw);
    out << ",\"geometric_clearance_m\":";
    append_double(out, sample.geometric_clearance_m);
    out << ",\"closing_rate_mps\":";
    append_double(out, sample.closing_rate_mps);
    out << ",\"uncertainty_margin_m\":";
    append_double(out, sample.uncertainty_margin_m);
    out << ",\"evaluated_clearance_m\":";
    append_double(out, sample.evaluated_clearance_m);
    out << ",\"clearance_soft_cost\":";
    append_double(out, sample.clearance_soft_cost);
    out << ",\"collision_margin_violated\":";
    append_bool(out, sample.collision_margin_violated);
    out << '}';
  }
  out << "]";
  out << ",\"map_filter_decisions\":[";
  for (std::size_t i = 0; i < e.map_filter_decisions.size(); ++i) {
    if (i != 0) out << ',';
    const auto & d = e.map_filter_decisions[i];
    out << '{';
    out << "\"object_id\":\"" << escape_json(d.object_id) << "\"";
    out << ",\"full_dynamic_evaluation\":";
    append_bool(out, d.full_dynamic_evaluation);
    out << ",\"immediate_guard_only\":";
    append_bool(out, d.immediate_guard_only);
    out << ",\"immediate_guard_violated\":";
    append_bool(out, d.immediate_guard_violated);
    out << ",\"reason\":\"" << escape_json(d.reason) << "\"";
    // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_BEGIN
    out << ",\"lane_associated\":";
    append_bool(out, d.lane_associated);
    out << ",\"motion_heading_yaw\":";
    append_double(out, d.motion_heading_yaw);
    out << ",\"motion_heading_reliability\":";
    append_double(out, d.motion_heading_reliability);
    out << ",\"lane_hypotheses\":[";
    for (std::size_t j = 0; j < d.lane_hypotheses.size(); ++j) {
      if (j != 0) out << ',';
      const auto & h = d.lane_hypotheses[j];
      out << '{';
      out << "\"lane_id\":" << h.lane_id;
      out << ",\"is_route_lane\":";
      append_bool(out, h.is_route_lane);
      out << ",\"s_m\":";
      append_double(out, h.s_m);
      out << ",\"confidence\":";
      append_double(out, h.confidence);
      out << ",\"lateral_distance_m\":";
      append_double(out, h.lateral_distance_m);
      out << ",\"heading_error_deg\":";
      append_double(out, h.heading_error_deg);
      out << ",\"predicted_path_fit_m\":";
      append_double(out, h.predicted_path_fit_m);
      out << '}';
    }
    out << ']';
    // CANDIDATE_BANK_MAP_FILTER_VISUALIZATION_V1_END
    out << ",\"best_lane_id\":" << d.best_lane_id;
    out << ",\"best_lane_confidence\":";
    append_double(out, d.best_lane_confidence);
    out << ",\"best_lane_lateral_distance_m\":";
    append_double(out, d.best_lane_lateral_distance_m);
    out << ",\"best_lane_heading_error_deg\":";
    append_double(out, d.best_lane_heading_error_deg);
    out << ",\"best_lane_predicted_path_fit_m\":";
    append_double(out, d.best_lane_predicted_path_fit_m);
    out << ",\"motion_heading_source\":\"" << escape_json(d.motion_heading_source) << "\"";
    out << ",\"lane_association_ambiguous\":";
    append_bool(out, d.lane_association_ambiguous);
    out << ",\"conflict_ego_lane_id\":" << d.conflict_ego_lane_id;
    out << ",\"conflict_competing_lane_id\":" << d.conflict_competing_lane_id;
    out << ",\"conflict_x_map\":";
    append_double(out, d.conflict_x_map);
    out << ",\"conflict_y_map\":";
    append_double(out, d.conflict_y_map);
    out << ",\"object_distance_to_conflict_m\":";
    append_double(out, d.object_distance_to_conflict_m);
    out << ",\"ego_arrival_time_sec\":";
    append_double(out, d.ego_arrival_time_sec);
    out << ",\"object_arrival_time_sec\":";
    append_double(out, d.object_arrival_time_sec);
    out << '}';
  }
  out << "]}";
  write_line(out.str());
}
// CANDIDATE_BANK_DYNAMIC_ANALYSIS_V1_END

void record_candidate_purged(
  const std::uint64_t cycle_id, const std::uint64_t candidate_id, const std::string & source,
  const std::string & reason, const double age_sec, const double projection_error_m,
  const double remaining_horizon_sec)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_purged") << ",\"candidate_id\":" << candidate_id
      << ",\"source\":\"" << escape_json(source) << "\""
      << ",\"reason\":\"" << escape_json(reason) << "\""
      << ",\"age_sec\":";
  append_double(out, age_sec);
  out << ",\"projection_error_m\":";
  append_double(out, projection_error_m);
  out << ",\"remaining_horizon_sec\":";
  append_double(out, remaining_horizon_sec);
  out << '}';
  write_line(out.str());
}

void record_candidate_selection(
  const std::uint64_t cycle_id, const std::uint64_t previous_selected_id,
  const std::uint64_t best_id, const std::uint64_t selected_id, const bool switched,
  const bool hysteresis_kept, const std::string & reason, const double best_score,
  const double selected_score, const std::string & best_safety_tier,
  const std::string & selected_safety_tier)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_selection")
      << ",\"previous_selected_id\":" << previous_selected_id << ",\"best_id\":" << best_id
      << ",\"selected_id\":" << selected_id << ",\"switched\":";
  append_bool(out, switched);
  out << ",\"hysteresis_kept\":";
  append_bool(out, hysteresis_kept);
  out << ",\"reason\":\"" << escape_json(reason) << "\""
      << ",\"best_score\":";
  append_double(out, best_score);
  out << ",\"selected_score\":";
  append_double(out, selected_score);
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_BEGIN
  out << ",\"best_safety_tier\":\"" << escape_json(best_safety_tier) << "\"";
  out << ",\"selected_safety_tier\":\"" << escape_json(selected_safety_tier) << "\"";
  // CANDIDATE_BANK_SAFETY_TIER_SELECTOR_V1_END
  out << '}';
  write_line(out.str());
}

void record_candidate_seed_selection(
  const std::uint64_t cycle_id, const std::string & seed_mode, const std::string & candidate_source,
  const std::string & effective_reference_source)
{
  if (!g_config.enabled || cycle_id == 0) return;
  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_seed_selection") << ",\"seed_mode\":\""
      << escape_json(seed_mode) << "\""
      << ",\"candidate_source\":\"" << escape_json(candidate_source) << "\""
      << ",\"effective_reference_source\":\"" << escape_json(effective_reference_source) << "\"}";
  write_line(out.str());
}

// CANDIDATE_BANK_OUTPUT_BRIDGE_V1_BEGIN
void record_candidate_output_handoff(
  const std::uint64_t cycle_id, const bool bridge_applied, const std::uint64_t selected_id,
  const std::string & selected_source, const std::string & reason,
  const autoware_planning_msgs::msg::Trajectory & downstream_trajectory,
  const std::size_t downstream_candidate_count)
{
  if (!g_config.enabled || cycle_id == 0) return;

  double velocity_first_mps = 0.0;
  double velocity_min_mps = 0.0;
  double velocity_last_mps = 0.0;
  if (!downstream_trajectory.points.empty()) {
    velocity_first_mps = downstream_trajectory.points.front().longitudinal_velocity_mps;
    velocity_last_mps = downstream_trajectory.points.back().longitudinal_velocity_mps;
    velocity_min_mps = velocity_first_mps;
    for (const auto & point : downstream_trajectory.points) {
      velocity_min_mps =
        std::min(velocity_min_mps, static_cast<double>(point.longitudinal_velocity_mps));
    }
  }

  std::ostringstream out;
  out << event_prefix(cycle_id, "candidate_output_handoff") << ",\"bridge_applied\":";
  append_bool(out, bridge_applied);
  out << ",\"selected_id\":" << selected_id << ",\"selected_source\":\""
      << escape_json(selected_source) << "\""
      << ",\"reason\":\"" << escape_json(reason) << "\""
      << ",\"frame_id\":\"" << escape_json(downstream_trajectory.header.frame_id) << "\""
      << ",\"point_count\":" << downstream_trajectory.points.size()
      << ",\"downstream_candidate_count\":" << downstream_candidate_count;
  out << ",\"velocity_first_mps\":";
  append_double(out, velocity_first_mps);
  out << ",\"velocity_min_mps\":";
  append_double(out, velocity_min_mps);
  out << ",\"velocity_last_mps\":";
  append_double(out, velocity_last_mps);
  out << "}";
  write_line(out.str());
}
// CANDIDATE_BANK_OUTPUT_BRIDGE_V1_END
// CANDIDATE_BANK_DIAGNOSTICS_V2_END
}  // namespace autoware::candidate_bank::internal_debug
