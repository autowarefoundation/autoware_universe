// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/diffusion_planner_node.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/marker_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner
{
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
std::string compute_file_hash_hex(const std::string & path)
{
  constexpr std::size_t HASH_READ_BUFFER_BYTES = 64 * 1024;
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    return "<failed to open>";
  }
  std::array<char, HASH_READ_BUFFER_BYTES> buffer{};
  std::size_t combined = 0;
  std::hash<std::string_view> hasher;
  while (ifs) {
    ifs.read(buffer.data(), buffer.size());
    const std::streamsize n = ifs.gcount();
    if (n <= 0) {
      break;
    }
    const std::size_t chunk = hasher(std::string_view(buffer.data(), static_cast<std::size_t>(n)));
    // boost::hash_combine: 0x9e3779b97f4a7c15 is 2^64 / golden ratio.
    combined ^= chunk + 0x9e3779b97f4a7c15ULL + (combined << 6) + (combined >> 2);
  }
  std::ostringstream oss;
  oss << std::hex << std::setw(sizeof(std::size_t) * 2) << std::setfill('0') << combined;
  return oss.str();
}
}  // namespace

DiffusionPlanner::DiffusionPlanner(const rclcpp::NodeOptions & options)
: Node("diffusion_planner", options), generator_uuid_(autoware_utils_uuid::generate_uuid())
{
  // Initialize the node
  pub_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_trajectories_ = this->create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
  pub_objects_ =
    this->create_publisher<PredictedObjects>("~/output/predicted_objects", rclcpp::QoS(1));
  pub_route_marker_ = this->create_publisher<MarkerArray>("~/debug/route_marker", 10);
  pub_lane_marker_ = this->create_publisher<MarkerArray>("~/debug/lane_marker", 10);
  pub_linestring_marker_ = this->create_publisher<MarkerArray>("~/debug/linestring_marker", 10);
  pub_turn_indicators_ =
    this->create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators", 1);
  pub_traffic_signal_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightGroup>(
    "~/output/debug/traffic_signal", 1);
  debug_processing_time_detail_pub_ = this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  debug_processing_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);
  pub_inference_time_ =
    this->create_publisher<std_msgs::msg::Float64>("~/debug/inference_time_ms", 1);
  pub_denoising_steps_ =
    this->create_publisher<std_msgs::msg::Float32MultiArray>("~/debug/denoising_steps", 1);
  pub_guidance_status_ = this->create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/debug/guidance_status", 1);

  set_up_params();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  RCLCPP_INFO_STREAM(
    get_logger(),
    "vehicle_info: wheel_base_m=" << vehicle_info_.wheel_base_m
                                  << ", front_overhang_m=" << vehicle_info_.front_overhang_m
                                  << ", rear_overhang_m=" << vehicle_info_.rear_overhang_m
                                  << ", left_overhang_m=" << vehicle_info_.left_overhang_m
                                  << ", right_overhang_m=" << vehicle_info_.right_overhang_m
                                  << ", wheel_tread_m=" << vehicle_info_.wheel_tread_m);

  // Create core instance
  core_ = std::make_unique<DiffusionPlannerCore>(params_, vehicle_info_);

  // Services to enable/disable guidance modules
  set_start_guidance_enabled_service_ = this->create_service<SetBool>(
    "~/service/set_start_guidance_enabled", std::bind(
                                              &DiffusionPlanner::on_set_start_guidance_enabled,
                                              this, std::placeholders::_1, std::placeholders::_2));
  set_stop_guidance_enabled_service_ = this->create_service<SetBool>(
    "~/service/set_stop_guidance_enabled", std::bind(
                                             &DiffusionPlanner::on_set_stop_guidance_enabled, this,
                                             std::placeholders::_1, std::placeholders::_2));
  set_centerline_guidance_enabled_service_ = this->create_service<SetBool>(
    "~/service/set_centerline_guidance_enabled",
    std::bind(
      &DiffusionPlanner::on_set_centerline_guidance_enabled, this, std::placeholders::_1,
      std::placeholders::_2));

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      this, "diffusion_planner");

  diagnostics_inference_ = std::make_unique<DiagnosticsInterface>(this, "inference_status");
  try {
    load_model();
    if (params_.build_only) {
      RCLCPP_INFO(get_logger(), "Build only mode enabled. Exiting after loading model.");
      std::exit(EXIT_SUCCESS);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Inference will be disabled.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(get_clock()->now());
    if (params_.build_only) {
      RCLCPP_ERROR(get_logger(), "Build only mode: exiting due to model load failure.");
      std::exit(EXIT_FAILURE);
    }
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&DiffusionPlanner::on_timer, this));

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DiffusionPlanner::on_map, this, std::placeholders::_1));

  // Parameter Callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&DiffusionPlanner::on_parameter, this, std::placeholders::_1));
}

DiffusionPlanner::~DiffusionPlanner() = default;

void DiffusionPlanner::set_up_params()
{
  // node params
  params_.model_type = this->declare_parameter<std::string>("model.type", "single_step");
  params_.args_path = this->declare_parameter<std::string>("model.args_path", "");
  params_.single_step_model_path =
    this->declare_parameter<std::string>("model.single_step_model.onnx_model_path", "");
  params_.encoder_model_path =
    this->declare_parameter<std::string>("model.multi_step_model.encoder_onnx_model_path", "");
  params_.decoder_model_path =
    this->declare_parameter<std::string>("model.multi_step_model.decoder_onnx_model_path", "");
  params_.turn_indicator_model_path = this->declare_parameter<std::string>(
    "model.multi_step_model.turn_indicator_onnx_model_path", "");
  params_.dpm_solver_steps =
    this->declare_parameter<int>("model.multi_step_model.dpm_solver_steps", 10);
  params_.backend = this->declare_parameter<std::string>("model.backend", "tensorrt");
  params_.trt_precision = this->declare_parameter<std::string>("model.precision", "fp32");
  params_.use_cuda_graph = this->declare_parameter<bool>("model.use_cuda_graph", true);
  params_.plugins_path = this->declare_parameter<std::string>("plugins_path", "");
  params_.build_only = this->declare_parameter<bool>("build_only", false);
  params_.planning_frequency_hz = this->declare_parameter<double>("planning_frequency_hz", 10.0);
  params_.ignore_neighbors = this->declare_parameter<bool>("ignore_neighbors", false);
  params_.ignore_unknown_neighbors =
    this->declare_parameter<bool>("ignore_unknown_neighbors", false);
  params_.traffic_light_group_msg_timeout_seconds =
    this->declare_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
  params_.batch_size = this->declare_parameter<int>("batch_size", 1);
  params_.temperature_list = this->declare_parameter<std::vector<double>>("temperature", {0.0});
  params_.velocity_smoothing_window =
    this->declare_parameter<int64_t>("velocity_smoothing_window", 8);
  params_.stopping_threshold = this->declare_parameter<double>("stopping_threshold", 0.3);
  params_.turn_indicator_keep_offset =
    this->declare_parameter<float>("turn_indicator_keep_offset", -1.25f);
  params_.turn_indicator_hold_duration =
    this->declare_parameter<double>("turn_indicator_hold_duration", 0.0);
  params_.shift_x = this->declare_parameter<bool>("shift_x", false);
  params_.delay_step = this->declare_parameter<int64_t>("delay_step", 0);
  params_.line_string_max_step_m = this->declare_parameter<double>("line_string_max_step_m", 5.0);
  params_.use_time_interpolation = this->declare_parameter<bool>("use_time_interpolation", false);
  params_.start_guidance_reference_distance_m =
    this->declare_parameter<double>("guidance.start_guidance.reference_distance_m", 10.0);
  params_.start_guidance_max_scale =
    this->declare_parameter<double>("guidance.start_guidance.max_scale", 30.0);
  params_.stop_guidance_stop_acceleration_mps2 =
    this->declare_parameter<double>("guidance.stop_guidance.stop_acceleration_mps2", 1.0);
  params_.centerline_guidance_start_time_s =
    this->declare_parameter<double>("guidance.centerline_guidance.start_time_s", 2.0);

  // planning factor params
  planning_factor_params_.enable_stop =
    this->declare_parameter<bool>("planning_factor.enable_stop", false);
  planning_factor_params_.enable_slowdown =
    this->declare_parameter<bool>("planning_factor.enable_slowdown", false);
  planning_factor_params_.detection_config.stop_velocity_threshold =
    this->declare_parameter<double>("planning_factor.stop_velocity_threshold", 0.1);
  planning_factor_params_.detection_config.stop_keep_duration_threshold =
    this->declare_parameter<double>("planning_factor.stop_keep_duration_threshold", 1.0);
  planning_factor_params_.detection_config.slowdown_accel_threshold =
    this->declare_parameter<double>("planning_factor.slowdown_accel_threshold", -0.3);

  // debug params
  debug_params_.publish_debug_map =
    this->declare_parameter<bool>("debug_params.publish_debug_map", false);
  debug_params_.publish_debug_route =
    this->declare_parameter<bool>("debug_params.publish_debug_route", true);
  debug_params_.publish_debug_linestrings =
    this->declare_parameter<bool>("debug_params.publish_debug_linestrings", true);
}

void DiffusionPlanner::load_model()
{
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Loading model");
  diagnostics_inference_->publish(get_clock()->now());
  core_->load_model();
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::OK, "Model loaded");
  diagnostics_inference_->publish(get_clock()->now());

  if (params_.model_type == "single_step") {
    RCLCPP_INFO_STREAM(
      get_logger(), "Loaded single_step_model_path="
                      << params_.single_step_model_path
                      << " (hash=" << compute_file_hash_hex(params_.single_step_model_path) << ")");
  } else if (params_.model_type == "multi_step") {
    RCLCPP_INFO_STREAM(
      get_logger(), "Loaded encoder_model_path="
                      << params_.encoder_model_path
                      << " (hash=" << compute_file_hash_hex(params_.encoder_model_path) << ")");
    RCLCPP_INFO_STREAM(
      get_logger(), "Loaded decoder_model_path="
                      << params_.decoder_model_path
                      << " (hash=" << compute_file_hash_hex(params_.decoder_model_path) << ")");
    RCLCPP_INFO_STREAM(
      get_logger(), "Loaded turn_indicator_model_path="
                      << params_.turn_indicator_model_path << " (hash="
                      << compute_file_hash_hex(params_.turn_indicator_model_path) << ")");
  }
  RCLCPP_INFO_STREAM(
    get_logger(), "Loaded args_path=" << params_.args_path << " (hash="
                                      << compute_file_hash_hex(params_.args_path) << ")");
}

SetParametersResult DiffusionPlanner::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  {
    DiffusionPlannerParams temp_params = params_;
    const auto previous_args_path = params_.args_path;
    const auto previous_model_type = params_.model_type;
    const auto previous_single_step_model_path = params_.single_step_model_path;
    const auto previous_encoder_model_path = params_.encoder_model_path;
    const auto previous_decoder_model_path = params_.decoder_model_path;
    const auto previous_turn_indicator_model_path = params_.turn_indicator_model_path;
    const auto previous_batch_size = params_.batch_size;
    const auto previous_dpm_solver_steps = params_.dpm_solver_steps;
    const auto previous_backend = params_.backend;
    const auto previous_trt_precision = params_.trt_precision;
    const auto previous_use_cuda_graph = params_.use_cuda_graph;
    const auto previous_line_string_max_step_m = params_.line_string_max_step_m;
    update_param<std::string>(parameters, "model.type", temp_params.model_type);
    update_param<std::string>(parameters, "model.args_path", temp_params.args_path);
    update_param<std::string>(
      parameters, "model.single_step_model.onnx_model_path", temp_params.single_step_model_path);
    update_param<std::string>(
      parameters, "model.multi_step_model.encoder_onnx_model_path", temp_params.encoder_model_path);
    update_param<std::string>(
      parameters, "model.multi_step_model.decoder_onnx_model_path", temp_params.decoder_model_path);
    update_param<std::string>(
      parameters, "model.multi_step_model.turn_indicator_onnx_model_path",
      temp_params.turn_indicator_model_path);
    update_param<int>(
      parameters, "model.multi_step_model.dpm_solver_steps", temp_params.dpm_solver_steps);
    update_param<std::string>(parameters, "model.backend", temp_params.backend);
    update_param<std::string>(parameters, "model.precision", temp_params.trt_precision);
    update_param<bool>(parameters, "model.use_cuda_graph", temp_params.use_cuda_graph);
    update_param<bool>(
      parameters, "ignore_unknown_neighbors", temp_params.ignore_unknown_neighbors);
    update_param<bool>(parameters, "ignore_neighbors", temp_params.ignore_neighbors);
    update_param<double>(
      parameters, "traffic_light_group_msg_timeout_seconds",
      temp_params.traffic_light_group_msg_timeout_seconds);
    update_param<int>(parameters, "batch_size", temp_params.batch_size);
    update_param<std::vector<double>>(parameters, "temperature", temp_params.temperature_list);
    update_param<int64_t>(
      parameters, "velocity_smoothing_window", temp_params.velocity_smoothing_window);
    update_param<double>(parameters, "stopping_threshold", temp_params.stopping_threshold);
    update_param<float>(
      parameters, "turn_indicator_keep_offset", temp_params.turn_indicator_keep_offset);
    update_param<double>(
      parameters, "turn_indicator_hold_duration", temp_params.turn_indicator_hold_duration);
    update_param<bool>(parameters, "shift_x", temp_params.shift_x);
    update_param<int64_t>(parameters, "delay_step", temp_params.delay_step);
    update_param<double>(parameters, "line_string_max_step_m", temp_params.line_string_max_step_m);
    update_param<bool>(parameters, "use_time_interpolation", temp_params.use_time_interpolation);
    update_param<double>(
      parameters, "guidance.start_guidance.reference_distance_m",
      temp_params.start_guidance_reference_distance_m);
    update_param<double>(
      parameters, "guidance.start_guidance.max_scale", temp_params.start_guidance_max_scale);
    update_param<double>(
      parameters, "guidance.stop_guidance.stop_acceleration_mps2",
      temp_params.stop_guidance_stop_acceleration_mps2);
    update_param<double>(
      parameters, "guidance.centerline_guidance.start_time_s",
      temp_params.centerline_guidance_start_time_s);
    if (temp_params.trt_precision != "fp32" && temp_params.trt_precision != "fp16") {
      SetParametersResult result;
      result.successful = false;
      result.reason = "model.precision must be either 'fp32' or 'fp16'";
      return result;
    }
    const bool valid_backend = temp_params.backend == "tensorrt"
#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
                               || temp_params.backend == "ort_cpu" ||
                               temp_params.backend == "ort_cuda" ||
                               temp_params.backend == "ort_tensorrt"
#endif
      ;
    if (!valid_backend) {
      SetParametersResult result;
      result.successful = false;
      result.reason = "model.backend must be 'tensorrt'";
#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
      result.reason += ", 'ort_cpu', 'ort_cuda', or 'ort_tensorrt'";
#else
      result.reason += "; ONNX Runtime support is not available in this build";
#endif
      return result;
    }
    const bool args_path_changed = temp_params.args_path != previous_args_path;
    const bool model_paths_changed =
      temp_params.model_type != previous_model_type ||
      temp_params.single_step_model_path != previous_single_step_model_path ||
      temp_params.encoder_model_path != previous_encoder_model_path ||
      temp_params.decoder_model_path != previous_decoder_model_path ||
      temp_params.turn_indicator_model_path != previous_turn_indicator_model_path;
    const bool batch_size_changed = temp_params.batch_size != previous_batch_size;
    const bool dpm_solver_steps_changed = temp_params.dpm_solver_steps != previous_dpm_solver_steps;
    const bool backend_changed = temp_params.backend != previous_backend;
    const bool trt_config_changed = temp_params.trt_precision != previous_trt_precision ||
                                    temp_params.use_cuda_graph != previous_use_cuda_graph;
    const bool line_string_max_step_changed =
      temp_params.line_string_max_step_m != previous_line_string_max_step_m;
    params_ = temp_params;
    core_->update_params(params_);

    if (
      args_path_changed || model_paths_changed || batch_size_changed || dpm_solver_steps_changed ||
      backend_changed || trt_config_changed) {
      try {
        load_model();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Failed to reload model.");
        SetParametersResult result;
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    }

    if (line_string_max_step_changed && lanelet_map_ptr_) {
      core_->set_map(lanelet_map_ptr_);
    }
  }

  {
    DiffusionPlannerDebugParams temp_debug_params = debug_params_;
    update_param<bool>(
      parameters, "debug_params.publish_debug_map", temp_debug_params.publish_debug_map);
    update_param<bool>(
      parameters, "debug_params.publish_debug_route", temp_debug_params.publish_debug_route);
    update_param<bool>(
      parameters, "debug_params.publish_debug_linestrings",
      temp_debug_params.publish_debug_linestrings);
    debug_params_ = temp_debug_params;
  }

  SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void DiffusionPlanner::on_set_start_guidance_enabled(
  const SetBool::Request::SharedPtr request, const SetBool::Response::SharedPtr response)
{
  core_->set_start_guidance_enabled(request->data);

  response->success = true;
  response->message = request->data ? "Start guidance enabled" : "Start guidance disabled";
}

void DiffusionPlanner::on_set_stop_guidance_enabled(
  const SetBool::Request::SharedPtr request, const SetBool::Response::SharedPtr response)
{
  core_->set_stop_guidance_enabled(request->data);

  response->success = true;
  response->message = request->data ? "Stop guidance enabled" : "Stop guidance disabled";
}

void DiffusionPlanner::on_set_centerline_guidance_enabled(
  const SetBool::Request::SharedPtr request, const SetBool::Response::SharedPtr response)
{
  core_->set_centerline_guidance_enabled(request->data);

  response->success = true;
  response->message =
    request->data ? "Centerline guidance enabled" : "Centerline guidance disabled";
}

void DiffusionPlanner::publish_first_traffic_light_on_route(
  const FrameContext & frame_context) const
{
  const auto msg = core_->get_first_traffic_light_on_route(frame_context);
  pub_traffic_signal_->publish(msg);
}

void DiffusionPlanner::publish_debug_markers(
  const InputDataMap & input_data_map, const Eigen::Matrix4d & ego_to_map_transform,
  const rclcpp::Time & timestamp) const
{
  if (debug_params_.publish_debug_route) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto route_markers = utils::create_lane_marker(
      ego_to_map_transform, input_data_map.at("route_lanes"),
      std::vector<int64_t>(ROUTE_LANES_SHAPE.begin(), ROUTE_LANES_SHAPE.end()), timestamp, lifetime,
      {0.8, 0.8, 0.8, 0.8}, "map", true);
    pub_route_marker_->publish(route_markers);
  }

  if (debug_params_.publish_debug_map) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto lane_markers = utils::create_lane_marker(
      ego_to_map_transform, input_data_map.at("lanes"),
      std::vector<int64_t>(LANES_SHAPE.begin(), LANES_SHAPE.end()), timestamp, lifetime,
      {0.1, 0.1, 0.7, 0.8}, "map", true);
    pub_lane_marker_->publish(lane_markers);
  }

  if (debug_params_.publish_debug_linestrings) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto linestring_markers = utils::create_linestring_marker(
      ego_to_map_transform, input_data_map.at("line_strings"),
      std::vector<int64_t>(LINE_STRINGS_SHAPE.begin(), LINE_STRINGS_SHAPE.end()), timestamp,
      lifetime, "map");
    pub_linestring_marker_->publish(linestring_markers);
  }
}

void DiffusionPlanner::on_timer()
{
  // Timer callback function
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing_time");

  diagnostics_inference_->clear();

  const rclcpp::Time current_time(get_clock()->now());
  if (!core_->is_model_loaded()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Model not loaded. Inference is disabled. Check model.* parameters.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, "Model not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  if (!core_->is_map_loaded()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Waiting for map data...");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Map data not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Take data from subscribers. Odometry drains as a buffer (All policy); see create_frame_context.
  auto objects = sub_tracked_objects_.take_data();
  auto ego_states = sub_current_odometry_.take_data();
  auto ego_acceleration = sub_current_acceleration_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto temp_route_ptr = route_subscriber_.take_data();
  auto turn_indicators_ptr = sub_turn_indicators_.take_data();

  // Prepare frame context using core
  const std::optional<FrameContext> frame_context = core_->create_frame_context(
    ego_states, ego_acceleration, objects, traffic_signals, turn_indicators_ptr, temp_route_ptr,
    this->now());

  if (!frame_context) {
    // Log detailed information about missing inputs
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "There is no input data. objects: "
        << (objects ? "true" : "false") << ", ego_states: " << (ego_states.empty() ? "0" : "some")
        << ", ego_acceleration: " << (ego_acceleration ? "true" : "false")
        << ", route: " << (core_->get_route() ? "true" : "false")
        << ", turn_indicators: " << (turn_indicators_ptr ? "true" : "false"));
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "No input data available for inference");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Warn (but proceed) when the closest buffered odometry is far from the object timestamp, e.g.
  // during an odometry dropout.
  if (frame_context->ego_object_time_diff_s > constants::MAX_EGO_OBJECT_TIME_DIFF_S) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Closest odometry is %.3f s from the tracked-objects timestamp (threshold %.3f s); "
      "proceeding with the closest sample.",
      frame_context->ego_object_time_diff_s, constants::MAX_EGO_OBJECT_TIME_DIFF_S);
  }

  if (traffic_signals.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "no traffic signal received. traffic light info will not be updated");
  }

  const rclcpp::Time frame_time(frame_context->output_time);
  InputDataMap input_data_map = core_->create_input_data(*frame_context);

  publish_debug_markers(input_data_map, frame_context->ego_to_map_transform, frame_time);

  publish_first_traffic_light_on_route(*frame_context);

  // Calculate and record metrics for diagnostics using core
  diagnostics_inference_->add_key_value(
    "valid_lane_count", core_->count_valid_elements(input_data_map, "lanes"));
  diagnostics_inference_->add_key_value(
    "valid_route_count", core_->count_valid_elements(input_data_map, "route_lanes"));
  diagnostics_inference_->add_key_value(
    "valid_polygon_count", core_->count_valid_elements(input_data_map, "polygons"));
  diagnostics_inference_->add_key_value(
    "valid_line_string_count", core_->count_valid_elements(input_data_map, "line_strings"));
  diagnostics_inference_->add_key_value(
    "valid_neighbor_count", core_->count_valid_elements(input_data_map, "neighbor_agents_past"));

  // normalization of data
  preprocess::normalize_input_data(input_data_map, core_->get_observation_normalization());
  if (!utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "Input data contains invalid values");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run inference using core
  auto inference_result = core_->run_inference(input_data_map);

  if (!inference_result) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Inference failed: " << inference_result.error());
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, inference_result.error());
    diagnostics_inference_->publish(frame_time);
    return;
  }

  std_msgs::msg::Float64 inference_time_msg;
  inference_time_msg.data = inference_result->inference_time_ms;
  pub_inference_time_->publish(inference_time_msg);

  PlannerOutput planner_output;
  try {
    planner_output =
      core_->create_planner_output(*inference_result, *frame_context, frame_time, generator_uuid_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Postprocessing failed: " << e.what());
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(frame_time);
    return;
  }

  if (!planner_output.denoising_steps.data.empty()) {
    pub_denoising_steps_->publish(planner_output.denoising_steps);
  }

  publish_guidance_status(planner_output.guidance_triggered, frame_time);

  pub_trajectory_->publish(planner_output.trajectory);
  pub_trajectories_->publish(planner_output.candidate_trajectories);
  pub_objects_->publish(planner_output.predicted_objects);
  pub_turn_indicators_->publish(planner_output.turn_indicator_command);

  publish_planning_factor(planner_output.trajectory);

  // Publish diagnostics
  diagnostics_inference_->publish(frame_time);
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch_ptr_->toc("processing_time", true);
  debug_processing_time_pub_->publish(processing_time_msg);
}

void DiffusionPlanner::publish_guidance_status(
  const std::unordered_map<std::string, std::vector<bool>> & guidance_triggered,
  const rclcpp::Time & timestamp)
{
  if (guidance_triggered.empty()) {
    return;
  }

  autoware_internal_debug_msgs::msg::StringStamped msg;
  msg.stamp = timestamp;

  std::vector<std::string> batch_entries;
  size_t batch_size = 0;
  for (const auto & [name, triggered_list] : guidance_triggered) {
    batch_size = std::max(batch_size, triggered_list.size());
  }

  for (size_t b = 0; b < batch_size; ++b) {
    std::string entry = "[" + std::to_string(b) + "]";
    for (const auto & [name, triggered_list] : guidance_triggered) {
      if (b < triggered_list.size() && triggered_list[b]) {
        entry += "\n  - " + name;
      }
    }
    batch_entries.push_back(entry);
  }

  std::string result;
  result += "Guidance Status:\n";
  for (size_t i = 0; i < batch_entries.size(); ++i) {
    if (i > 0) {
      result += '\n';
    }
    result += batch_entries[i];
  }
  msg.data = result;

  pub_guidance_status_->publish(msg);
}

void DiffusionPlanner::publish_planning_factor(const Trajectory & trajectory)
{
  const auto & points = trajectory.points;
  const auto detection_result =
    detect_planning_factors(points, planning_factor_params_.detection_config);

  if (planning_factor_params_.enable_stop && detection_result.stop) {
    const auto & stop = *detection_result.stop;
    planning_factor_interface_->add(
      points, stop.ego_pose, stop.stop_pose, PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{});
  }

  if (planning_factor_params_.enable_slowdown && detection_result.slowdown) {
    const auto & slowdown = *detection_result.slowdown;
    planning_factor_interface_->add(
      points, slowdown.ego_pose, slowdown.start_pose, slowdown.end_pose, PlanningFactor::SLOW_DOWN,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true, slowdown.start_velocity,
      slowdown.end_velocity);
  }

  planning_factor_interface_->publish();
}

void DiffusionPlanner::on_map(const HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_msg);
  core_->set_map(lanelet_map_ptr_);
}

}  // namespace autoware::diffusion_planner
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diffusion_planner::DiffusionPlanner)
