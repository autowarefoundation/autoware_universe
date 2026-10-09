// Copyright 2026 Xinchen Lin. Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__CAMP_DIFFUSION_ADAPTER__INPUT_CHAIN_AUDIT_HPP_
#define AUTOWARE__CAMP_DIFFUSION_ADAPTER__INPUT_CHAIN_AUDIT_HPP_

#include <autoware/diffusion_planner/diffusion_planner_core.hpp>
#include <autoware/diffusion_planner/dimensions.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::camp_diffusion_adapter
{
namespace input_chain_audit_detail
{
namespace native = autoware::diffusion_planner;
using Json = nlohmann::json;

inline bool & attempted()
{
  static bool value = false;  // The adapter runs in its single-threaded executor.
  return value;
}

template <class Shape>
std::vector<int64_t> batched(Shape const & shape, int64_t batch)
{
  std::vector<int64_t> result(std::begin(shape), std::end(shape));
  if (result.empty() || batch <= 0) throw std::runtime_error("Invalid audit batch/shape");
  result.front() = batch;
  return result;
}

inline std::map<std::string, std::vector<int64_t>> shapes(int64_t batch)
{
  using namespace native;
  return {
    {"sampled_trajectories", batched(SAMPLED_TRAJECTORIES_SHAPE, batch)},
    {"ego_agent_past", batched(EGO_HISTORY_SHAPE, batch)},
    {"ego_current_state", batched(EGO_CURRENT_STATE_SHAPE, batch)},
    {"neighbor_agents_past", batched(NEIGHBOR_SHAPE, batch)},
    {"static_objects", batched(STATIC_OBJECTS_SHAPE, batch)},
    {"lanes", batched(LANES_SHAPE, batch)},
    {"lanes_speed_limit", batched(LANES_SPEED_LIMIT_SHAPE, batch)},
    {"route_lanes", batched(ROUTE_LANES_SHAPE, batch)},
    {"route_lanes_speed_limit", batched(ROUTE_LANES_SPEED_LIMIT_SHAPE, batch)},
    {"polygons", batched(POLYGONS_SHAPE, batch)},
    {"line_strings", batched(LINE_STRINGS_SHAPE, batch)},
    {"goal_pose", batched(GOAL_POSE_SHAPE, batch)},
    {"ego_shape", batched(EGO_SHAPE_SHAPE, batch)},
    {"turn_indicators", batched(TURN_INDICATORS_SHAPE, batch)},
    {"delay", batched(DELAY_SHAPE, batch)}};
}

template <class Map>
Json tensors(Map const & input, int64_t batch)
{
  auto const expected = shapes(batch);
  Json result = Json::object();
  for (auto const & [name, values] : input) {
    for (float value : values) {
      if (!std::isfinite(value)) throw std::runtime_error("Non-finite actual tensor: " + name);
    }
    Json item = {{"dtype", "float32"}, {"numel", values.size()}, {"values", values}};
    auto const found = expected.find(name);
    item["shape"] = found == expected.end() ? Json(nullptr) : Json(found->second);
    result[name] = std::move(item);
  }
  return result;
}

template <class Observation>
Json observation(Observation const & normalization)
{
  Json result = Json::object();
  for (auto const & [name, pair] : normalization) {
    result[name] = {{"mean", pair.first}, {"std", pair.second}};
  }
  return result;
}

template <class State>
Json state(State const & normalization)
{
  return {{"mean", normalization.first}, {"std", normalization.second}};
}

template <class Matrix>
Json matrix(Matrix const & value)
{
  Json result = Json::array();
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) result.push_back(value(row, column));
  }
  return result;
}

template <class Message>
Json cdr(Message const & message, char const * type)
{
  rclcpp::Serialization<Message> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  auto const & raw = serialized.get_rcl_serialized_message();
  std::vector<uint8_t> bytes(raw.buffer, raw.buffer + raw.buffer_length);
  return {{"type", type}, {"format", "ROS serialized CDR"}, {"bytes", bytes}};
}
}  // namespace input_chain_audit_detail

inline bool input_chain_audit_requested() noexcept
{
  using namespace input_chain_audit_detail;
  if (attempted()) return false;
  auto const * directory = std::getenv("CAMP_INPUT_CHAIN_AUDIT_DIR");
  if (!directory || !*directory) return false;
  try {
    return !std::filesystem::exists(
      std::filesystem::path(directory) / "first_actual_input_chain.json");
  } catch (std::exception const & error) {
    attempted() = true;
    std::cerr << "CAMP input audit unavailable: " << error.what() << '\n';
    return false;
  }
}

// Caller copies this SAME create_input_data result before capture/normalization,
// then calls here immediately before its existing run_inference call.
// This function neither constructs planner inputs nor invokes a model/solver.
template <class Observation, class State>
bool record_actual_input_chain_once(
  autoware::diffusion_planner::FrameContext const & frame,
  autoware::diffusion_planner::InputDataMap const & raw,
  autoware::diffusion_planner::InputDataMap const & normalized,
  Observation const & core_observation, State const & adapter_state, std::string const & args_path,
  int64_t batch)
{
  using namespace input_chain_audit_detail;
  static_assert(
    sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "Audit requires IEEE float32");
  if (!input_chain_audit_requested()) return false;
  attempted() = true;
  try {
    auto const directory = std::filesystem::path(std::getenv("CAMP_INPUT_CHAIN_AUDIT_DIR"));
    std::ifstream args_file(args_path, std::ios::binary);
    if (!args_file) throw std::runtime_error("Cannot capture frozen args: " + args_path);
    std::ostringstream args_text;
    args_text << args_file.rdbuf();
    if (args_file.bad()) throw std::runtime_error("Cannot read frozen args");
    Json context = {
      {"frame_time_nanoseconds", frame.frame_time.nanoseconds()},
      {"frame_time_seconds", frame.frame_time.seconds()},
      {"ego_to_map_transform_row_major", matrix(frame.ego_to_map_transform)},
      {"ego_kinematic_state", cdr(frame.ego_kinematic_state, "nav_msgs/msg/Odometry")},
      {"ego_acceleration",
       cdr(frame.ego_acceleration, "geometry_msgs/msg/AccelWithCovarianceStamped")},
      {"neighbor_history_count", frame.ego_centric_neighbor_histories.size()},
      {"neighbor_histories", Json::array()}};
    context["snapped_pose_row_major"] =
      frame.snapped_pose ? matrix(*frame.snapped_pose) : Json(nullptr);
    context["snapped_interpolation_time_s"] = frame.snapped_interpolation_time_s
                                                ? Json(*frame.snapped_interpolation_time_s)
                                                : Json(nullptr);
    for (size_t index = 0; index < frame.ego_centric_neighbor_histories.size(); ++index) {
      auto const & history = frame.ego_centric_neighbor_histories[index];
      Json item = {{"slot", index}, {"size", history.size()}, {"empty", history.empty()}};
      if (!history.empty()) {
        item["latest_original_info"] = cdr(
          history.get_latest_state().original_info, "autoware_perception_msgs/msg/TrackedObject");
      }
      context["neighbor_histories"].push_back(std::move(item));
    }
    Json audit = {
      {"schema_version", 1},
      {"status", "actual_same_frame_capture"},
      {"scope",
       "Actual 15-tensor InputDataMap boundary; one unchanged create_input_data call and its "
       "actual normalization, before inference"},
      {"batch_size", batch},
      {"float_bytes", sizeof(float)},
      {"float_ieee754", true},
      {"frame", std::move(context)},
      {"raw_input_data", tensors(raw, batch)},
      {"normalized_input_data", tensors(normalized, batch)},
      {"frozen_args", {{"path", args_path}, {"text", args_text.str()}}},
      {"core_observation_normalization", observation(core_observation)},
      {"adapter_state_normalization", state(adapter_state)},
      {"normalization_function", "autoware::diffusion_planner::preprocess::normalize_input_data"},
      {"normalization_source",
       "core get_observation_normalization; adapter state normalization loaded from the same args "
       "path"},
      {"unobserved",
       {"encoder internal features", "DPM solver internal states",
        "full private per-step neighbor history timestamps/original objects"}},
      {"noise_scope",
       "sampled_trajectories values captured from this same raw input; no reseeding, "
       "reconstruction, second native input construction or inference"}};
    auto const text = audit.dump();
    if (text.size() > (size_t{128} << 20))
      throw std::runtime_error("Actual audit exceeds 128 MiB cap; no truncation performed");
    std::filesystem::create_directories(directory);
    auto const target = directory / "first_actual_input_chain.json";
    auto const temporary = directory / "first_actual_input_chain.json.tmp";
    std::ofstream output(temporary, std::ios::binary);
    output.write(text.data(), static_cast<std::streamsize>(text.size()));
    output.close();
    if (!output) throw std::runtime_error("Cannot write actual input audit");
    std::filesystem::rename(temporary, target);
    std::cerr << "CAMP actual input audit saved: " << target.string() << '\n';
    return true;
  } catch (std::exception const & error) {
    // Diagnostic failure must never alter the planned geometry or model call.
    std::cerr << "CAMP actual input audit failed: " << error.what() << '\n';
    return false;
  }
}
}  // namespace autoware::camp_diffusion_adapter
#endif
