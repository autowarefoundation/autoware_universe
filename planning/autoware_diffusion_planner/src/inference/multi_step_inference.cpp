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

#include "autoware/diffusion_planner/inference/multi_step_inference.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;

MultiStepInference::MultiStepInference(
  const std::string & encoder_model_path, const std::string & decoder_model_path,
  const std::string & turn_indicator_model_path, const std::string & plugins_path, int batch_size,
  const std::string & precision, bool use_cuda_graph, int dpm_solver_steps,
  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances)
: batch_size_(batch_size),
  dpm_solver_steps_(dpm_solver_steps),
  plugins_path_(plugins_path),
  precision_(precision),
  use_cuda_graph_(use_cuda_graph),
  guidances_(std::move(guidances))
{
  const std::vector<int64_t> encoding_shape = {batch_size_, ENCODING_TOKEN_NUM, HIDDEN_DIM};
  const std::vector<int64_t> diffusion_time_shape = {batch_size_, MAX_NUM_AGENTS, OUTPUT_T + 1, 1};
  const std::vector<int64_t> model_output_shape = {
    batch_size_, MAX_NUM_AGENTS, OUTPUT_T + 1, POSE_DIM};

  sampled_trajectories_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(SAMPLED_TRAJECTORIES_SHAPE));
  diffusion_time_d_ =
    autoware::cuda_utils::make_unique<float[]>(num_elements(diffusion_time_shape));
  ego_history_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(EGO_HISTORY_SHAPE));
  neighbor_agents_past_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(NEIGHBOR_SHAPE));
  static_objects_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(STATIC_OBJECTS_SHAPE));
  lanes_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(LANES_SHAPE));
  lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(
    batch_size_ * num_elements_without_batch(LANES_HAS_SPEED_LIMIT_SHAPE));
  lanes_speed_limit_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE));
  route_lanes_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SHAPE));
  route_lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(
    batch_size_ * num_elements_without_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  route_lanes_speed_limit_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  polygons_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(POLYGONS_SHAPE));
  line_strings_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(LINE_STRINGS_SHAPE));
  goal_pose_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(GOAL_POSE_SHAPE));
  ego_shape_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(EGO_SHAPE_SHAPE));
  turn_indicators_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements_without_batch(TURN_INDICATORS_SHAPE));

  encoding_num_elements_ = num_elements(encoding_shape);
  model_output_num_elements_ = num_elements(model_output_shape);
  logit_num_elements_ = batch_size_ * num_elements_without_batch(TURN_INDICATOR_LOGIT_SHAPE);
  encoding_d_ = autoware::cuda_utils::make_unique<float[]>(encoding_num_elements_);
  model_output_d_ = autoware::cuda_utils::make_unique<float[]>(model_output_num_elements_);
  turn_indicator_logit_d_ = autoware::cuda_utils::make_unique<float[]>(logit_num_elements_);
  model_output_pinned_ = autoware::cuda_utils::make_unique_host<float[]>(
    model_output_num_elements_, cudaHostAllocDefault);
  logit_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(logit_num_elements_, cudaHostAllocDefault);

  load_engines(encoder_model_path, decoder_model_path, turn_indicator_model_path);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

MultiStepInference::~MultiStepInference()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

void MultiStepInference::load_engines(
  const std::string & encoder_model_path, const std::string & decoder_model_path,
  const std::string & turn_indicator_model_path)
{
  const std::vector<int64_t> encoding_shape = {1, ENCODING_TOKEN_NUM, HIDDEN_DIM};
  const std::vector<int64_t> diffusion_time_shape = {1, MAX_NUM_AGENTS, OUTPUT_T + 1, 1};
  const std::vector<int64_t> model_output_shape = {1, MAX_NUM_AGENTS, OUTPUT_T + 1, POSE_DIM};

  std::vector<ProfileDims> encoder_profile_dims;
  std::vector<NetworkIO> encoder_network_io;
  const auto add_encoder_tensor = [&](const std::string & name, const auto & shape) {
    const auto dims = to_dynamic_dims(shape, batch_size_);
    encoder_profile_dims.emplace_back(make_profile_dims(name, dims, batch_size_));
    encoder_network_io.emplace_back(name, dims);
  };
  add_encoder_tensor("ego_agent_past", EGO_HISTORY_SHAPE);
  add_encoder_tensor("neighbor_agents_past", NEIGHBOR_SHAPE);
  add_encoder_tensor("static_objects", STATIC_OBJECTS_SHAPE);
  add_encoder_tensor("lanes", LANES_SHAPE);
  add_encoder_tensor("lanes_speed_limit", LANES_SPEED_LIMIT_SHAPE);
  add_encoder_tensor("lanes_has_speed_limit", LANES_HAS_SPEED_LIMIT_SHAPE);
  add_encoder_tensor("route_lanes", ROUTE_LANES_SHAPE);
  add_encoder_tensor("route_lanes_speed_limit", ROUTE_LANES_SPEED_LIMIT_SHAPE);
  add_encoder_tensor("route_lanes_has_speed_limit", ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE);
  add_encoder_tensor("polygons", POLYGONS_SHAPE);
  add_encoder_tensor("line_strings", LINE_STRINGS_SHAPE);
  add_encoder_tensor("goal_pose", GOAL_POSE_SHAPE);
  add_encoder_tensor("ego_shape", EGO_SHAPE_SHAPE);
  add_encoder_tensor("turn_indicators", TURN_INDICATORS_SHAPE);
  encoder_network_io.emplace_back("encoding", to_dynamic_dims(encoding_shape, batch_size_));

  encoder_trt_ptr_ = setup_engine(
    encoder_model_path, plugins_path_, batch_size_, precision_, encoder_network_io,
    encoder_profile_dims);

  std::vector<ProfileDims> decoder_profile_dims;
  std::vector<NetworkIO> decoder_network_io;
  const auto add_decoder_tensor = [&](const std::string & name, const auto & shape) {
    const auto dims = to_dynamic_dims(shape, batch_size_);
    decoder_profile_dims.emplace_back(make_profile_dims(name, dims, batch_size_));
    decoder_network_io.emplace_back(name, dims);
  };
  add_decoder_tensor("encoding", encoding_shape);
  add_decoder_tensor("sampled_trajectories", SAMPLED_TRAJECTORIES_SHAPE);
  add_decoder_tensor("diffusion_time", diffusion_time_shape);
  add_decoder_tensor("neighbor_agents_past", NEIGHBOR_SHAPE);
  decoder_network_io.emplace_back("model_output", to_dynamic_dims(model_output_shape, batch_size_));

  decoder_trt_ptr_ = setup_engine(
    decoder_model_path, plugins_path_, batch_size_, precision_, decoder_network_io,
    decoder_profile_dims);

  std::vector<ProfileDims> turn_indicator_profile_dims;
  std::vector<NetworkIO> turn_indicator_network_io;
  const auto add_turn_indicator_tensor = [&](const std::string & name, const auto & shape) {
    const auto dims = to_dynamic_dims(shape, batch_size_);
    turn_indicator_profile_dims.emplace_back(make_profile_dims(name, dims, batch_size_));
    turn_indicator_network_io.emplace_back(name, dims);
  };
  add_turn_indicator_tensor("encoding", encoding_shape);
  add_turn_indicator_tensor("final_x0", model_output_shape);
  turn_indicator_network_io.emplace_back(
    "turn_indicator_logit", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE, batch_size_));

  turn_indicator_trt_ptr_ = setup_engine(
    turn_indicator_model_path, plugins_path_, batch_size_, precision_, turn_indicator_network_io,
    turn_indicator_profile_dims);

  bind_encoder_buffers();
  bind_decoder_buffers();
  bind_turn_indicator_buffers();
}

void MultiStepInference::bind_encoder_buffers()
{
  encoder_trt_ptr_->setInputShape(
    "ego_agent_past", to_dims_with_batch(EGO_HISTORY_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "neighbor_agents_past", to_dims_with_batch(NEIGHBOR_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "static_objects", to_dims_with_batch(STATIC_OBJECTS_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape("lanes", to_dims_with_batch(LANES_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "lanes_speed_limit", to_dims_with_batch(LANES_SPEED_LIMIT_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "lanes_has_speed_limit", to_dims_with_batch(LANES_HAS_SPEED_LIMIT_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "route_lanes", to_dims_with_batch(ROUTE_LANES_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "route_lanes_speed_limit", to_dims_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "route_lanes_has_speed_limit",
    to_dims_with_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape("polygons", to_dims_with_batch(POLYGONS_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "line_strings", to_dims_with_batch(LINE_STRINGS_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape("goal_pose", to_dims_with_batch(GOAL_POSE_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape("ego_shape", to_dims_with_batch(EGO_SHAPE_SHAPE, batch_size_));
  encoder_trt_ptr_->setInputShape(
    "turn_indicators", to_dims_with_batch(TURN_INDICATORS_SHAPE, batch_size_));

  encoder_trt_ptr_->setTensorAddress("ego_agent_past", ego_history_d_.get());
  encoder_trt_ptr_->setTensorAddress("neighbor_agents_past", neighbor_agents_past_d_.get());
  encoder_trt_ptr_->setTensorAddress("static_objects", static_objects_d_.get());
  encoder_trt_ptr_->setTensorAddress("lanes", lanes_d_.get());
  encoder_trt_ptr_->setTensorAddress("lanes_speed_limit", lanes_speed_limit_d_.get());
  encoder_trt_ptr_->setTensorAddress("lanes_has_speed_limit", lanes_has_speed_limit_d_.get());
  encoder_trt_ptr_->setTensorAddress("route_lanes", route_lanes_d_.get());
  encoder_trt_ptr_->setTensorAddress("route_lanes_speed_limit", route_lanes_speed_limit_d_.get());
  encoder_trt_ptr_->setTensorAddress(
    "route_lanes_has_speed_limit", route_lanes_has_speed_limit_d_.get());
  encoder_trt_ptr_->setTensorAddress("polygons", polygons_d_.get());
  encoder_trt_ptr_->setTensorAddress("line_strings", line_strings_d_.get());
  encoder_trt_ptr_->setTensorAddress("goal_pose", goal_pose_d_.get());
  encoder_trt_ptr_->setTensorAddress("ego_shape", ego_shape_d_.get());
  encoder_trt_ptr_->setTensorAddress("turn_indicators", turn_indicators_d_.get());
  encoder_trt_ptr_->setTensorAddress("encoding", encoding_d_.get());
}

void MultiStepInference::bind_decoder_buffers()
{
  const std::vector<int64_t> encoding_shape = {1, ENCODING_TOKEN_NUM, HIDDEN_DIM};
  const std::vector<int64_t> diffusion_time_shape = {1, MAX_NUM_AGENTS, OUTPUT_T + 1, 1};

  decoder_trt_ptr_->setInputShape("encoding", to_dims_with_batch(encoding_shape, batch_size_));
  decoder_trt_ptr_->setInputShape(
    "sampled_trajectories", to_dims_with_batch(SAMPLED_TRAJECTORIES_SHAPE, batch_size_));
  decoder_trt_ptr_->setInputShape(
    "diffusion_time", to_dims_with_batch(diffusion_time_shape, batch_size_));
  decoder_trt_ptr_->setInputShape(
    "neighbor_agents_past", to_dims_with_batch(NEIGHBOR_SHAPE, batch_size_));

  decoder_trt_ptr_->setTensorAddress("encoding", encoding_d_.get());
  decoder_trt_ptr_->setTensorAddress("sampled_trajectories", sampled_trajectories_d_.get());
  decoder_trt_ptr_->setTensorAddress("diffusion_time", diffusion_time_d_.get());
  decoder_trt_ptr_->setTensorAddress("neighbor_agents_past", neighbor_agents_past_d_.get());
  decoder_trt_ptr_->setTensorAddress("model_output", model_output_d_.get());
}

void MultiStepInference::bind_turn_indicator_buffers()
{
  const std::vector<int64_t> encoding_shape = {1, ENCODING_TOKEN_NUM, HIDDEN_DIM};
  const std::vector<int64_t> final_x0_shape = {1, MAX_NUM_AGENTS, OUTPUT_T + 1, POSE_DIM};

  turn_indicator_trt_ptr_->setInputShape(
    "encoding", to_dims_with_batch(encoding_shape, batch_size_));
  turn_indicator_trt_ptr_->setInputShape(
    "final_x0", to_dims_with_batch(final_x0_shape, batch_size_));

  turn_indicator_trt_ptr_->setTensorAddress("encoding", encoding_d_.get());
  turn_indicator_trt_ptr_->setTensorAddress("final_x0", model_output_d_.get());
  turn_indicator_trt_ptr_->setTensorAddress("turn_indicator_logit", turn_indicator_logit_d_.get());
}

void MultiStepInference::transfer_inputs_to_device(const preprocess::InputDataMap & input_data_map)
{
  transfer_float_input(input_data_map.at("sampled_trajectories"), sampled_trajectories_d_, stream_);
  transfer_float_input(input_data_map.at("ego_agent_past"), ego_history_d_, stream_);
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

  const auto diffusion_time_it = input_data_map.find("diffusion_time");
  if (diffusion_time_it != input_data_map.end()) {
    transfer_float_input(diffusion_time_it->second, diffusion_time_d_, stream_);
  } else {
    const std::vector<float> diffusion_time(batch_size_ * MAX_NUM_AGENTS * (OUTPUT_T + 1), 1.0f);
    transfer_float_input(diffusion_time, diffusion_time_d_, stream_);
  }

  transfer_speed_mask(
    input_data_map.at("lanes_speed_limit"), lanes_has_speed_limit_d_,
    batch_size_ * num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE), stream_);
  transfer_speed_mask(
    input_data_map.at("route_lanes_speed_limit"), route_lanes_has_speed_limit_d_,
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE), stream_);
}

std::vector<float> MultiStepInference::create_diffusion_time(float t) const
{
  return std::vector<float>(batch_size_ * MAX_NUM_AGENTS * (OUTPUT_T + 1), t);
}

std::vector<float> MultiStepInference::create_current_states(
  const preprocess::InputDataMap & input_data_map) const
{
  const auto & ego_current_state = input_data_map.at("ego_current_state");
  const auto & neighbor_agents_past = input_data_map.at("neighbor_agents_past");

  std::vector<float> current_states(batch_size_ * MAX_NUM_AGENTS * POSE_DIM, 0.0f);
  for (int b = 0; b < batch_size_; ++b) {
    for (int64_t d = 0; d < POSE_DIM; ++d) {
      current_states[(b * MAX_NUM_AGENTS * POSE_DIM) + d] =
        ego_current_state[b * EGO_CURRENT_STATE_SHAPE[1] + d];
    }

    for (int64_t agent = 1; agent < MAX_NUM_AGENTS; ++agent) {
      for (int64_t d = 0; d < POSE_DIM; ++d) {
        const size_t neighbor_idx =
          (((static_cast<size_t>(b) * MAX_NUM_NEIGHBORS + (agent - 1)) * (INPUT_T + 1) + INPUT_T) *
           11) +
          d;
        const size_t current_idx = (static_cast<size_t>(b) * MAX_NUM_AGENTS + agent) * POSE_DIM + d;
        current_states[current_idx] = neighbor_agents_past[neighbor_idx];
      }
    }
  }
  return current_states;
}

void MultiStepInference::apply_prefix_constraint(
  std::vector<float> & x, const std::vector<float> & current_states) const
{
  for (int b = 0; b < batch_size_; ++b) {
    for (int64_t agent = 0; agent < MAX_NUM_AGENTS; ++agent) {
      for (int64_t d = 0; d < POSE_DIM; ++d) {
        const size_t x_idx =
          ((static_cast<size_t>(b) * MAX_NUM_AGENTS + agent) * (OUTPUT_T + 1)) * POSE_DIM + d;
        const size_t current_idx = (static_cast<size_t>(b) * MAX_NUM_AGENTS + agent) * POSE_DIM + d;
        x[x_idx] = current_states[current_idx];
      }
    }
  }
}

std::vector<float> MultiStepInference::evaluate_decoder(const std::vector<float> & x, float t)
{
  const auto diffusion_time = create_diffusion_time(t);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    sampled_trajectories_d_.get(), x.data(), x.size() * sizeof(float), cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    diffusion_time_d_.get(), diffusion_time.data(), diffusion_time.size() * sizeof(float),
    cudaMemcpyHostToDevice, stream_));

  const bool status = enqueue_trt(*decoder_trt_ptr_, decoder_cuda_graph_, stream_, use_cuda_graph_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  if (!status) {
    throw std::runtime_error("Failed to enqueue decoder inference.");
  }

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    model_output_pinned_.get(), model_output_d_.get(), model_output_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return std::vector<float>(
    model_output_pinned_.get(), model_output_pinned_.get() + model_output_num_elements_);
}

DpmSolver::SampleResult MultiStepInference::run_dpm_solver(
  const preprocess::InputDataMap & input_data_map)
{
  const std::vector<float> current_states = create_current_states(input_data_map);
  const auto model_fn = [this](const std::vector<float> & x, float timestep) {
    return evaluate_decoder(x, timestep);
  };
  const auto correcting_fn = [this, &current_states](std::vector<float> & x) {
    apply_prefix_constraint(x, current_states);
  };

  const DpmSolver solver(dpm_solver_steps_);
  return solver.sample(
    input_data_map.at("sampled_trajectories"), model_fn, correcting_fn, guidances_);
}

MultiStepInference::InferenceResult MultiStepInference::infer(
  const preprocess::InputDataMap & input_data_map)
{
  auto start = std::chrono::steady_clock::now();

  transfer_inputs_to_device(input_data_map);

  bool status = enqueue_trt(*encoder_trt_ptr_, encoder_cuda_graph_, stream_, use_cuda_graph_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  if (!status) {
    return tl::unexpected(std::string{"Failed to enqueue encoder inference."});
  }

  DpmSolver::SampleResult solver_result;
  try {
    solver_result = run_dpm_solver(input_data_map);
  } catch (const std::exception & e) {
    return tl::unexpected(std::string{e.what()});
  }

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    model_output_d_.get(), solver_result.final_x.data(),
    solver_result.final_x.size() * sizeof(float), cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  status =
    enqueue_trt(*turn_indicator_trt_ptr_, turn_indicator_cuda_graph_, stream_, use_cuda_graph_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  if (!status) {
    return tl::unexpected(std::string{"Failed to enqueue turn indicator inference."});
  }

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    logit_pinned_.get(), turn_indicator_logit_d_.get(), logit_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  std::vector<float> denoising_predictions;
  for (const auto & step : solver_result.denoising_steps) {
    denoising_predictions.insert(denoising_predictions.end(), step.begin(), step.end());
  }
  std::vector<float> logit_host(logit_pinned_.get(), logit_pinned_.get() + logit_num_elements_);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;

  InferenceOutput output;
  output.outputs = std::make_pair(std::move(solver_result.final_x), std::move(logit_host));
  output.denoising_predictions = std::move(denoising_predictions);
  output.denoising_timesteps = std::move(solver_result.denoising_timesteps);
  output.inference_time_ms = elapsed.count();
  output.is_denormalized = false;
  output.guidance_triggered = std::move(solver_result.guidance_triggered);
  return output;
}

}  // namespace autoware::diffusion_planner
