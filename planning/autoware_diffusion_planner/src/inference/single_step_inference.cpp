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

#include "autoware/diffusion_planner/inference/single_step_inference.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <chrono>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware::tensorrt_common::ProfileDims;

SingleStepInference::SingleStepInference(
  const std::string & model_path, const std::string & plugins_path, int batch_size,
  const std::string & precision, bool use_cuda_graph)
: batch_size_(batch_size),
  plugins_path_(plugins_path),
  precision_(precision),
  use_cuda_graph_(use_cuda_graph)
{
  const size_t sampled_trajectories_size =
    batch_size_ * num_elements_without_batch(SAMPLED_TRAJECTORIES_SHAPE);
  const size_t ego_history_size = batch_size_ * num_elements_without_batch(EGO_HISTORY_SHAPE);
  const size_t ego_current_state_size =
    batch_size_ * num_elements_without_batch(EGO_CURRENT_STATE_SHAPE);
  const size_t neighbor_agents_past_size = batch_size_ * num_elements_without_batch(NEIGHBOR_SHAPE);
  const size_t static_objects_size = batch_size_ * num_elements_without_batch(STATIC_OBJECTS_SHAPE);
  const size_t lanes_size = batch_size_ * num_elements_without_batch(LANES_SHAPE);
  const size_t lanes_has_speed_limit_size =
    batch_size_ * num_elements_without_batch(LANES_HAS_SPEED_LIMIT_SHAPE);
  const size_t lanes_speed_limit_size =
    batch_size_ * num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE);
  const size_t route_lanes_size = batch_size_ * num_elements_without_batch(ROUTE_LANES_SHAPE);
  const size_t route_lanes_has_speed_limit_size =
    batch_size_ * num_elements_without_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE);
  const size_t route_lanes_speed_limit_size =
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE);
  const size_t polygons_size = batch_size_ * num_elements_without_batch(POLYGONS_SHAPE);
  const size_t line_strings_size = batch_size_ * num_elements_without_batch(LINE_STRINGS_SHAPE);
  const size_t goal_pose_size = batch_size_ * num_elements_without_batch(GOAL_POSE_SHAPE);
  const size_t ego_shape_size = batch_size_ * num_elements_without_batch(EGO_SHAPE_SHAPE);
  const size_t turn_indicators_size =
    batch_size_ * num_elements_without_batch(TURN_INDICATORS_SHAPE);
  const size_t delay_size = batch_size_ * num_elements_without_batch(DELAY_SHAPE);
  const size_t output_size = batch_size_ * num_elements_without_batch(OUTPUT_SHAPE);
  const size_t turn_indicator_logit_size =
    batch_size_ * num_elements_without_batch(TURN_INDICATOR_LOGIT_SHAPE);

  sampled_trajectories_d_ = autoware::cuda_utils::make_unique<float[]>(sampled_trajectories_size);
  ego_history_d_ = autoware::cuda_utils::make_unique<float[]>(ego_history_size);
  ego_current_state_d_ = autoware::cuda_utils::make_unique<float[]>(ego_current_state_size);
  neighbor_agents_past_d_ = autoware::cuda_utils::make_unique<float[]>(neighbor_agents_past_size);
  static_objects_d_ = autoware::cuda_utils::make_unique<float[]>(static_objects_size);
  lanes_d_ = autoware::cuda_utils::make_unique<float[]>(lanes_size);
  lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(lanes_has_speed_limit_size);
  lanes_speed_limit_d_ = autoware::cuda_utils::make_unique<float[]>(lanes_speed_limit_size);
  route_lanes_d_ = autoware::cuda_utils::make_unique<float[]>(route_lanes_size);
  route_lanes_has_speed_limit_d_ =
    autoware::cuda_utils::make_unique<bool[]>(route_lanes_has_speed_limit_size);
  route_lanes_speed_limit_d_ =
    autoware::cuda_utils::make_unique<float[]>(route_lanes_speed_limit_size);
  polygons_d_ = autoware::cuda_utils::make_unique<float[]>(polygons_size);
  line_strings_d_ = autoware::cuda_utils::make_unique<float[]>(line_strings_size);
  goal_pose_d_ = autoware::cuda_utils::make_unique<float[]>(goal_pose_size);
  ego_shape_d_ = autoware::cuda_utils::make_unique<float[]>(ego_shape_size);
  turn_indicators_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicators_size);
  delay_d_ = autoware::cuda_utils::make_unique<float[]>(delay_size);

  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_size);
  turn_indicator_logit_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicator_logit_size);

  // Pre-allocate pinned host buffers for fast async D2H transfers
  output_num_elements_ = output_size;
  logit_num_elements_ = turn_indicator_logit_size;
  output_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(output_num_elements_, cudaHostAllocDefault);
  logit_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(logit_num_elements_, cudaHostAllocDefault);

  load_engine(model_path);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

SingleStepInference::~SingleStepInference()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

void SingleStepInference::load_engine(const std::string & model_path)
{
  std::vector<ProfileDims> profile_dims;
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  const auto add_input_tensor = [&](const std::string & name, const auto & shape) {
    const auto dims = to_dynamic_dims(shape, batch_size_);
    profile_dims.emplace_back(make_profile_dims(name, dims, batch_size_));
    network_io.emplace_back(name, dims);
  };

  add_input_tensor("sampled_trajectories", SAMPLED_TRAJECTORIES_SHAPE);
  add_input_tensor("ego_agent_past", EGO_HISTORY_SHAPE);
  add_input_tensor("ego_current_state", EGO_CURRENT_STATE_SHAPE);
  add_input_tensor("neighbor_agents_past", NEIGHBOR_SHAPE);
  add_input_tensor("static_objects", STATIC_OBJECTS_SHAPE);
  add_input_tensor("lanes", LANES_SHAPE);
  add_input_tensor("lanes_has_speed_limit", LANES_HAS_SPEED_LIMIT_SHAPE);
  add_input_tensor("lanes_speed_limit", LANES_SPEED_LIMIT_SHAPE);
  add_input_tensor("route_lanes", ROUTE_LANES_SHAPE);
  add_input_tensor("polygons", POLYGONS_SHAPE);
  add_input_tensor("line_strings", LINE_STRINGS_SHAPE);
  add_input_tensor("route_lanes_has_speed_limit", ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE);
  add_input_tensor("route_lanes_speed_limit", ROUTE_LANES_SPEED_LIMIT_SHAPE);
  add_input_tensor("goal_pose", GOAL_POSE_SHAPE);
  add_input_tensor("ego_shape", EGO_SHAPE_SHAPE);
  add_input_tensor("turn_indicators", TURN_INDICATORS_SHAPE);
  add_input_tensor("delay", DELAY_SHAPE);

  network_io.emplace_back("prediction", to_dynamic_dims(OUTPUT_SHAPE, batch_size_));
  network_io.emplace_back(
    "turn_indicator_logit", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE, batch_size_));

  network_trt_ptr_ =
    setup_engine(model_path, plugins_path_, batch_size_, precision_, network_io, profile_dims);

  bindBuffers();
}

void SingleStepInference::bindBuffers()
{
  // Set input shapes once (fixed batch_size)
  network_trt_ptr_->setInputShape(
    "sampled_trajectories", to_dims_with_batch(SAMPLED_TRAJECTORIES_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "ego_agent_past", to_dims_with_batch(EGO_HISTORY_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "ego_current_state", to_dims_with_batch(EGO_CURRENT_STATE_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "neighbor_agents_past", to_dims_with_batch(NEIGHBOR_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "static_objects", to_dims_with_batch(STATIC_OBJECTS_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape("lanes", to_dims_with_batch(LANES_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "lanes_has_speed_limit", to_dims_with_batch(LANES_HAS_SPEED_LIMIT_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "lanes_speed_limit", to_dims_with_batch(LANES_SPEED_LIMIT_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "route_lanes", to_dims_with_batch(ROUTE_LANES_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape("polygons", to_dims_with_batch(POLYGONS_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "line_strings", to_dims_with_batch(LINE_STRINGS_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "route_lanes_speed_limit", to_dims_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "route_lanes_has_speed_limit",
    to_dims_with_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape("goal_pose", to_dims_with_batch(GOAL_POSE_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape("ego_shape", to_dims_with_batch(EGO_SHAPE_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape(
    "turn_indicators", to_dims_with_batch(TURN_INDICATORS_SHAPE, batch_size_));
  network_trt_ptr_->setInputShape("delay", to_dims_with_batch(DELAY_SHAPE, batch_size_));

  // Bind tensor addresses once (GPU buffers are pre-allocated and stable)
  network_trt_ptr_->setTensorAddress("sampled_trajectories", sampled_trajectories_d_.get());
  network_trt_ptr_->setTensorAddress("ego_agent_past", ego_history_d_.get());
  network_trt_ptr_->setTensorAddress("ego_current_state", ego_current_state_d_.get());
  network_trt_ptr_->setTensorAddress("neighbor_agents_past", neighbor_agents_past_d_.get());
  network_trt_ptr_->setTensorAddress("static_objects", static_objects_d_.get());
  network_trt_ptr_->setTensorAddress("lanes", lanes_d_.get());
  network_trt_ptr_->setTensorAddress("lanes_has_speed_limit", lanes_has_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("lanes_speed_limit", lanes_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("route_lanes", route_lanes_d_.get());
  network_trt_ptr_->setTensorAddress("route_lanes_speed_limit", route_lanes_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress(
    "route_lanes_has_speed_limit", route_lanes_has_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("polygons", polygons_d_.get());
  network_trt_ptr_->setTensorAddress("line_strings", line_strings_d_.get());
  network_trt_ptr_->setTensorAddress("goal_pose", goal_pose_d_.get());
  network_trt_ptr_->setTensorAddress("ego_shape", ego_shape_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicators", turn_indicators_d_.get());
  network_trt_ptr_->setTensorAddress("delay", delay_d_.get());
  network_trt_ptr_->setTensorAddress("prediction", output_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicator_logit", turn_indicator_logit_d_.get());
}

void SingleStepInference::transferInputsToDevice(const preprocess::InputDataMap & input_data_map)
{
  transfer_float_input(input_data_map.at("sampled_trajectories"), sampled_trajectories_d_, stream_);
  transfer_float_input(input_data_map.at("ego_agent_past"), ego_history_d_, stream_);
  transfer_float_input(input_data_map.at("ego_current_state"), ego_current_state_d_, stream_);
  transfer_float_input(input_data_map.at("neighbor_agents_past"), neighbor_agents_past_d_, stream_);
  transfer_float_input(input_data_map.at("static_objects"), static_objects_d_, stream_);
  transfer_float_input(input_data_map.at("lanes"), lanes_d_, stream_);
  transfer_float_input(input_data_map.at("lanes_speed_limit"), lanes_speed_limit_d_, stream_);
  transfer_float_input(input_data_map.at("route_lanes"), route_lanes_d_, stream_);
  transfer_float_input(
    input_data_map.at("route_lanes_speed_limit"), route_lanes_speed_limit_d_, stream_);
  transfer_float_input(input_data_map.at("polygons"), polygons_d_, stream_);
  transfer_float_input(input_data_map.at("line_strings"), line_strings_d_, stream_);
  transfer_float_input(input_data_map.at("goal_pose"), goal_pose_d_, stream_);
  transfer_float_input(input_data_map.at("ego_shape"), ego_shape_d_, stream_);
  transfer_float_input(input_data_map.at("turn_indicators"), turn_indicators_d_, stream_);
  transfer_float_input(input_data_map.at("delay"), delay_d_, stream_);

  transfer_speed_mask(
    input_data_map.at("lanes_speed_limit"), lanes_has_speed_limit_d_,
    batch_size_ * num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE), stream_);
  transfer_speed_mask(
    input_data_map.at("route_lanes_speed_limit"), route_lanes_has_speed_limit_d_,
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE), stream_);
}

SingleStepInference::InferenceResult SingleStepInference::infer(
  const preprocess::InputDataMap & input_data_map)
{
  auto start = std::chrono::steady_clock::now();

  transferInputsToDevice(input_data_map);

  const bool status = enqueue_trt(*network_trt_ptr_, network_cuda_graph_, stream_, use_cuda_graph_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!status) {
    return tl::unexpected(std::string{"Failed to enqueue and do inference."});
  }

  // Async D2H via pre-allocated pinned host buffers
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output_pinned_.get(), output_d_.get(), output_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    logit_pinned_.get(), turn_indicator_logit_d_.get(), logit_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  std::vector<float> output_host(output_pinned_.get(), output_pinned_.get() + output_num_elements_);
  std::vector<float> logit_host(logit_pinned_.get(), logit_pinned_.get() + logit_num_elements_);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;

  InferenceOutput output;
  output.outputs = std::make_pair(std::move(output_host), std::move(logit_host));
  output.inference_time_ms = elapsed.count();
  output.is_denormalized = true;
  return output;
}

}  // namespace autoware::diffusion_planner
