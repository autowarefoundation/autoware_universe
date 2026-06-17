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

#include "autoware/tensorrt_oneplanner/oneplanner_node.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
constexpr int LOG_THROTTLE_INTERVAL_MS = 5000;
}  // namespace

OnePlannerNode::OnePlannerNode(const rclcpp::NodeOptions & options)
: Node("tensorrt_oneplanner", options),
  tf_buffer_(this->get_clock()),
  generator_uuid_(autoware_utils_uuid::generate_uuid())
{
  set_up_params();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  core_ = std::make_unique<OnePlannerCore>(params_, vehicle_info_);
  core_->load_normalization();

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  load_models();

  // Publishers
  pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", rclcpp::QoS(1));
  pub_trajectories_ =
    this->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
      "~/output/trajectories", rclcpp::QoS(1));
  pub_turn_indicators_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "~/output/turn_indicators", rclcpp::QoS(1));
  debug_processing_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", rclcpp::QoS(1));

  diagnostics_inference_ = std::make_unique<DiagnosticsInterface>(this, "oneplanner_inference");

  // Subscriptions
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&OnePlannerNode::on_map, this, std::placeholders::_1));
  cloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&OnePlannerNode::on_pointcloud, this, std::placeholders::_1));

  if (this->declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engines were built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void OnePlannerNode::set_up_params()
{
  // BEV encoder (lidar branch) params
  bev_params_.onnx_path = this->declare_parameter<std::string>("bev_encoder.onnx_path", "");
  bev_params_.engine_path = this->declare_parameter<std::string>("bev_encoder.engine_path", "");
  bev_params_.trt_precision =
    this->declare_parameter<std::string>("bev_encoder.trt_precision", "fp16");
  bev_params_.densification_world_frame_id =
    this->declare_parameter<std::string>("densification_world_frame_id", "map");
  bev_params_.densification_num_past_frames =
    this->declare_parameter<int64_t>("densification_num_past_frames", 1);
  bev_params_.cloud_capacity = this->declare_parameter<int64_t>("cloud_capacity", 2000000);
  bev_params_.max_points_per_voxel = this->declare_parameter<int64_t>("max_points_per_voxel", 10);
  bev_params_.voxels_num = this->declare_parameter<std::vector<int64_t>>("voxels_num");
  const auto point_cloud_range_double =
    this->declare_parameter<std::vector<double>>("point_cloud_range");
  bev_params_.point_cloud_range =
    std::vector<float>(point_cloud_range_double.begin(), point_cloud_range_double.end());
  const auto voxel_size_double = this->declare_parameter<std::vector<double>>("voxel_size");
  bev_params_.voxel_size = std::vector<float>(voxel_size_double.begin(), voxel_size_double.end());
  bev_params_.out_size_factor = this->declare_parameter<int64_t>("out_size_factor", 8);
  bev_params_.bev_feature_channels = this->declare_parameter<int64_t>("bev_feature_channels", 512);
  bev_params_.use_intensity = this->declare_parameter<bool>("use_intensity", true);

  // Planner head params
  planner_onnx_path_ = this->declare_parameter<std::string>("planner.onnx_path", "");
  plugins_path_ = this->declare_parameter<std::string>("plugins_path", "");
  bev_params_.plugins_path = plugins_path_;

  params_.args_path = this->declare_parameter<std::string>("args_path", "");
  params_.temperature = this->declare_parameter<double>("temperature", 0.0);
  params_.velocity_smoothing_window =
    this->declare_parameter<int64_t>("velocity_smoothing_window", 8);
  params_.stopping_threshold = this->declare_parameter<double>("stopping_threshold", 0.3);
  params_.turn_indicator_keep_offset =
    this->declare_parameter<float>("turn_indicator_keep_offset", -1.25f);
  params_.turn_indicator_hold_duration =
    this->declare_parameter<double>("turn_indicator_hold_duration", 1.0);
  params_.shift_x = this->declare_parameter<bool>("shift_x", false);
  params_.delay_step = this->declare_parameter<int64_t>("delay_step", 0);
  params_.line_string_max_step_m = this->declare_parameter<double>("line_string_max_step_m", 5.0);
  params_.use_time_interpolation = this->declare_parameter<bool>("use_time_interpolation", false);
  params_.traffic_light_group_msg_timeout_seconds =
    this->declare_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
}

void OnePlannerNode::load_models()
{
  bev_encoder_ = std::make_unique<BevEncoder>(bev_params_, stream_);
  planner_inference_ = std::make_unique<PlannerInference>(
    planner_onnx_path_, plugins_path_, /*batch_size=*/1, bev_encoder_->bev_feature_d(),
    bev_encoder_->bev_channels(), bev_encoder_->bev_size(), stream_);
}

void OnePlannerNode::on_map(const LaneletMapBin::ConstSharedPtr map_msg)
{
  const auto lanelet_map_ptr =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_msg);
  core_->set_map(lanelet_map_ptr);
}

void OnePlannerNode::on_pointcloud(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("processing_time");

  diagnostics_inference_->clear();
  const rclcpp::Time current_time(get_clock()->now());

  if (!core_->is_map_loaded()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS, "Waiting for map data...");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Map data not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Take data from subscribers
  auto ego_kinematic_state = sub_current_odometry_.take_data();
  auto ego_acceleration = sub_current_acceleration_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto route_ptr = route_subscriber_.take_data();
  auto turn_indicators_ptr = sub_turn_indicators_.take_data();

  const auto frame_context = core_->create_frame_context(
    ego_kinematic_state, ego_acceleration, traffic_signals, turn_indicators_ptr, route_ptr,
    current_time);

  if (!frame_context) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Missing input data. ego_kinematic_state: "
        << (ego_kinematic_state ? "true" : "false")
        << ", ego_acceleration: " << (ego_acceleration ? "true" : "false")
        << ", route: " << (core_->get_route() ? "true" : "false")
        << ", turn_indicators: " << (turn_indicators_ptr ? "true" : "false"));
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "No input data available for inference");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run the BEV encoder on the incoming pointcloud
  if (!bev_encoder_->encode(msg_ptr, tf_buffer_)) {
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, "BEV encoder inference failed");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Build and normalize planner inputs
  auto input_data_map = core_->create_input_data(*frame_context);
  autoware::diffusion_planner::preprocess::normalize_input_data(
    input_data_map, core_->get_normalization_map());
  if (!autoware::diffusion_planner::utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "Input data contains invalid values");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run the planner head (consumes the BEV feature map directly on-device)
  const auto inference_result = planner_inference_->infer(input_data_map);
  if (!inference_result.outputs) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Inference failed: " << inference_result.error_msg);
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, inference_result.error_msg);
    diagnostics_inference_->publish(current_time);
    return;
  }
  const auto & [prediction, turn_indicator_logit] = inference_result.outputs.value();

  const rclcpp::Time frame_time(frame_context->frame_time);
  OnePlannerOutput planner_output;
  try {
    planner_output = core_->create_planner_output(
      prediction, turn_indicator_logit, *frame_context, frame_time, generator_uuid_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Postprocessing failed: " << e.what());
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(frame_time);
    return;
  }

  pub_trajectory_->publish(planner_output.trajectory);
  pub_trajectories_->publish(planner_output.candidate_trajectories);
  pub_turn_indicators_->publish(planner_output.turn_indicator_command);

  diagnostics_inference_->publish(frame_time);
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc("processing_time", true);
  debug_processing_time_pub_->publish(processing_time_msg);
}

}  // namespace autoware::tensorrt_oneplanner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_oneplanner::OnePlannerNode)
