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

#include "autoware/diffusion_planner/inference/onnxruntime_inference.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <onnxruntime_c_api.h>

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
namespace
{
template <class Shape>
std::vector<int64_t> shape_with_batch(const Shape & shape, const int batch_size)
{
  std::vector<int64_t> dims(shape.begin(), shape.end());
  dims.at(0) = batch_size;
  return dims;
}

size_t num_elements_from_shape(const std::vector<int64_t> & shape)
{
  size_t size = 1;
  for (const auto dim : shape) {
    size *= static_cast<size_t>(dim);
  }
  return size;
}

std::vector<uint8_t> make_speed_mask(const std::vector<float> & speed_limit)
{
  std::vector<uint8_t> mask(speed_limit.size(), 0);
  for (size_t i = 0; i < speed_limit.size(); ++i) {
    mask[i] = speed_limit[i] > 0.0f ? 1U : 0U;
  }
  return mask;
}

struct FloatInput
{
  std::string name;
  const std::vector<float> * data;
  std::vector<int64_t> shape;
};

void append_cpu_provider(Ort::SessionOptions &)
{
  // CPUExecutionProvider is enabled by default.
}

void append_cuda_provider(Ort::SessionOptions & session_options)
{
  OrtCUDAProviderOptionsV2 * cuda_options = nullptr;
  Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cuda_options));
  Ort::ThrowOnError(
    Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA_V2(session_options, cuda_options));
  Ort::GetApi().ReleaseCUDAProviderOptions(cuda_options);
}

void append_tensorrt_provider(
  Ort::SessionOptions & session_options, const std::string & plugins_path)
{
  OrtTensorRTProviderOptionsV2 * trt_options = nullptr;
  Ort::ThrowOnError(Ort::GetApi().CreateTensorRTProviderOptions(&trt_options));

  std::vector<const char *> keys;
  std::vector<const char *> values;
  if (!plugins_path.empty()) {
    keys.push_back("trt_extra_plugin_lib_paths");
    values.push_back(plugins_path.c_str());
  }
  if (!keys.empty()) {
    Ort::ThrowOnError(
      Ort::GetApi().UpdateTensorRTProviderOptions(
        trt_options, keys.data(), values.data(), keys.size()));
  }
  Ort::ThrowOnError(
    Ort::GetApi().SessionOptionsAppendExecutionProvider_TensorRT_V2(session_options, trt_options));
  Ort::GetApi().ReleaseTensorRTProviderOptions(trt_options);
}

std::vector<FloatInput> single_step_float_inputs(
  const preprocess::InputDataMap & input_data_map, const int batch_size)
{
  return {
    {"sampled_trajectories", &input_data_map.at("sampled_trajectories"),
     shape_with_batch(SAMPLED_TRAJECTORIES_SHAPE, batch_size)},
    {"ego_agent_past", &input_data_map.at("ego_agent_past"),
     shape_with_batch(EGO_HISTORY_SHAPE, batch_size)},
    {"ego_current_state", &input_data_map.at("ego_current_state"),
     shape_with_batch(EGO_CURRENT_STATE_SHAPE, batch_size)},
    {"neighbor_agents_past", &input_data_map.at("neighbor_agents_past"),
     shape_with_batch(NEIGHBOR_SHAPE, batch_size)},
    {"static_objects", &input_data_map.at("static_objects"),
     shape_with_batch(STATIC_OBJECTS_SHAPE, batch_size)},
    {"lanes", &input_data_map.at("lanes"), shape_with_batch(LANES_SHAPE, batch_size)},
    {"lanes_speed_limit", &input_data_map.at("lanes_speed_limit"),
     shape_with_batch(LANES_SPEED_LIMIT_SHAPE, batch_size)},
    {"route_lanes", &input_data_map.at("route_lanes"),
     shape_with_batch(ROUTE_LANES_SHAPE, batch_size)},
    {"route_lanes_speed_limit", &input_data_map.at("route_lanes_speed_limit"),
     shape_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE, batch_size)},
    {"polygons", &input_data_map.at("polygons"), shape_with_batch(POLYGONS_SHAPE, batch_size)},
    {"line_strings", &input_data_map.at("line_strings"),
     shape_with_batch(LINE_STRINGS_SHAPE, batch_size)},
    {"goal_pose", &input_data_map.at("goal_pose"), shape_with_batch(GOAL_POSE_SHAPE, batch_size)},
    {"ego_shape", &input_data_map.at("ego_shape"), shape_with_batch(EGO_SHAPE_SHAPE, batch_size)},
    {"turn_indicators", &input_data_map.at("turn_indicators"),
     shape_with_batch(TURN_INDICATORS_SHAPE, batch_size)},
    {"delay", &input_data_map.at("delay"), shape_with_batch(DELAY_SHAPE, batch_size)}};
}

std::vector<FloatInput> encoder_float_inputs(
  const preprocess::InputDataMap & input_data_map, const int batch_size)
{
  return {
    {"ego_agent_past", &input_data_map.at("ego_agent_past"),
     shape_with_batch(EGO_HISTORY_SHAPE, batch_size)},
    {"neighbor_agents_past", &input_data_map.at("neighbor_agents_past"),
     shape_with_batch(NEIGHBOR_SHAPE, batch_size)},
    {"static_objects", &input_data_map.at("static_objects"),
     shape_with_batch(STATIC_OBJECTS_SHAPE, batch_size)},
    {"lanes", &input_data_map.at("lanes"), shape_with_batch(LANES_SHAPE, batch_size)},
    {"lanes_speed_limit", &input_data_map.at("lanes_speed_limit"),
     shape_with_batch(LANES_SPEED_LIMIT_SHAPE, batch_size)},
    {"route_lanes", &input_data_map.at("route_lanes"),
     shape_with_batch(ROUTE_LANES_SHAPE, batch_size)},
    {"route_lanes_speed_limit", &input_data_map.at("route_lanes_speed_limit"),
     shape_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE, batch_size)},
    {"polygons", &input_data_map.at("polygons"), shape_with_batch(POLYGONS_SHAPE, batch_size)},
    {"line_strings", &input_data_map.at("line_strings"),
     shape_with_batch(LINE_STRINGS_SHAPE, batch_size)},
    {"goal_pose", &input_data_map.at("goal_pose"), shape_with_batch(GOAL_POSE_SHAPE, batch_size)},
    {"ego_shape", &input_data_map.at("ego_shape"), shape_with_batch(EGO_SHAPE_SHAPE, batch_size)},
    {"turn_indicators", &input_data_map.at("turn_indicators"),
     shape_with_batch(TURN_INDICATORS_SHAPE, batch_size)}};
}

std::unordered_map<std::string, std::vector<uint8_t>> speed_limit_bool_inputs(
  const preprocess::InputDataMap & input_data_map)
{
  return {
    {"lanes_has_speed_limit", make_speed_mask(input_data_map.at("lanes_speed_limit"))},
    {"route_lanes_has_speed_limit", make_speed_mask(input_data_map.at("route_lanes_speed_limit"))}};
}

std::unordered_map<std::string, std::vector<float>> to_float_input_map(
  const std::vector<FloatInput> & inputs)
{
  std::unordered_map<std::string, std::vector<float>> map;
  for (const auto & input : inputs) {
    map.emplace(input.name, *input.data);
  }
  return map;
}

}  // namespace

OnnxruntimeExecutionProvider parse_execution_provider(const std::string & execution_provider)
{
  if (execution_provider == "cpu") {
    return OnnxruntimeExecutionProvider::CPU;
  }
  if (execution_provider == "cuda") {
    return OnnxruntimeExecutionProvider::CUDA;
  }
  if (execution_provider == "tensorrt") {
    return OnnxruntimeExecutionProvider::TensorRT;
  }
  throw std::invalid_argument(
    "Unsupported model.ort_execution_provider '" + execution_provider +
    "'. Expected 'cpu', 'cuda', or 'tensorrt'.");
}

OrtModel::OrtModel(
  const std::string & model_path, const OnnxruntimeExecutionProvider execution_provider,
  const std::string & plugins_path)
: env_(ORT_LOGGING_LEVEL_WARNING, "diffusion_planner"),
  session_(nullptr),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  if (execution_provider == OnnxruntimeExecutionProvider::TensorRT) {
    append_tensorrt_provider(session_options_, plugins_path);
    append_cuda_provider(session_options_);
  } else if (execution_provider == OnnxruntimeExecutionProvider::CUDA) {
    append_cuda_provider(session_options_);
  } else {
    append_cpu_provider(session_options_);
  }

  session_ = Ort::Session(env_, model_path.c_str(), session_options_);
}

std::unordered_map<std::string, std::vector<float>> OrtModel::run(
  const std::unordered_map<std::string, std::vector<float>> & float_inputs,
  const std::unordered_map<std::string, std::vector<uint8_t>> & bool_inputs,
  const std::vector<std::string> & output_names)
{
  std::vector<std::string> input_names;
  std::vector<const char *> input_name_ptrs;
  std::vector<Ort::Value> input_tensors;
  const auto input_count = float_inputs.size() + bool_inputs.size();
  input_names.reserve(input_count);
  input_name_ptrs.reserve(input_count);
  input_tensors.reserve(input_count);

  const auto add_float_tensor = [&](
                                  const std::string & name, const std::vector<float> & data,
                                  const std::vector<int64_t> & shape) {
    if (data.size() != num_elements_from_shape(shape)) {
      throw std::runtime_error("Input size mismatch for " + name);
    }
    input_names.push_back(name);
    input_name_ptrs.push_back(input_names.back().c_str());
    input_tensors.push_back(
      Ort::Value::CreateTensor<float>(
        memory_info_, const_cast<float *>(data.data()), data.size(), shape.data(), shape.size()));
  };

  const auto add_bool_tensor = [&](
                                 const std::string & name, const std::vector<uint8_t> & data,
                                 const std::vector<int64_t> & shape) {
    if (data.size() != num_elements_from_shape(shape)) {
      throw std::runtime_error("Input size mismatch for " + name);
    }
    input_names.push_back(name);
    input_name_ptrs.push_back(input_names.back().c_str());
    input_tensors.push_back(
      Ort::Value::CreateTensor(
        memory_info_, const_cast<uint8_t *>(data.data()), data.size() * sizeof(uint8_t),
        shape.data(), shape.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL));
  };

  for (const auto & [name, data] : float_inputs) {
    if (name == "sampled_trajectories") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / (MAX_NUM_AGENTS * (OUTPUT_T + 1) * POSE_DIM)),
         MAX_NUM_AGENTS, OUTPUT_T + 1, POSE_DIM});
    } else if (name == "ego_agent_past") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / ((INPUT_T + 1) * POSE_DIM)), INPUT_T + 1, POSE_DIM});
    } else if (name == "ego_current_state") {
      add_float_tensor(name, data, {static_cast<int64_t>(data.size() / 10), 10});
    } else if (name == "neighbor_agents_past") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / (MAX_NUM_NEIGHBORS * (INPUT_T + 1) * 11)),
         MAX_NUM_NEIGHBORS, INPUT_T + 1, 11});
    } else if (name == "static_objects") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / (NUM_STATIC_OBJECTS * 10)), NUM_STATIC_OBJECTS, 10});
    } else if (name == "lanes") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(
           data.size() / (NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM)),
         NUM_SEGMENTS_IN_LANE, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM});
    } else if (name == "lanes_speed_limit") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / NUM_SEGMENTS_IN_LANE), NUM_SEGMENTS_IN_LANE, 1});
    } else if (name == "route_lanes") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(
           data.size() / (NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM)),
         NUM_SEGMENTS_IN_ROUTE, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM});
    } else if (name == "route_lanes_speed_limit") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / NUM_SEGMENTS_IN_ROUTE), NUM_SEGMENTS_IN_ROUTE, 1});
    } else if (name == "polygons") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(
           data.size() / (NUM_POLYGONS * POINTS_PER_POLYGON * (2 + POLYGON_TYPE_NUM))),
         NUM_POLYGONS, POINTS_PER_POLYGON, 2 + POLYGON_TYPE_NUM});
    } else if (name == "line_strings") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(
           data.size() / (NUM_LINE_STRINGS * POINTS_PER_LINE_STRING * (2 + LINE_STRING_TYPE_NUM))),
         NUM_LINE_STRINGS, POINTS_PER_LINE_STRING, 2 + LINE_STRING_TYPE_NUM});
    } else if (name == "goal_pose") {
      add_float_tensor(name, data, {static_cast<int64_t>(data.size() / POSE_DIM), POSE_DIM});
    } else if (name == "ego_shape") {
      add_float_tensor(name, data, {static_cast<int64_t>(data.size() / 3), 3});
    } else if (name == "turn_indicators") {
      add_float_tensor(
        name, data, {static_cast<int64_t>(data.size() / (INPUT_T + 1)), INPUT_T + 1});
    } else if (name == "delay") {
      add_float_tensor(name, data, {static_cast<int64_t>(data.size()), 1});
    } else if (name == "encoding") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / (ENCODING_TOKEN_NUM * HIDDEN_DIM)), ENCODING_TOKEN_NUM,
         HIDDEN_DIM});
    } else if (name == "diffusion_time") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / (MAX_NUM_AGENTS * (OUTPUT_T + 1))), MAX_NUM_AGENTS,
         OUTPUT_T + 1, 1});
    } else if (name == "final_x0" || name == "model_output") {
      add_float_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / (MAX_NUM_AGENTS * (OUTPUT_T + 1) * POSE_DIM)),
         MAX_NUM_AGENTS, OUTPUT_T + 1, POSE_DIM});
    } else {
      throw std::runtime_error("Unsupported ONNX Runtime input: " + name);
    }
  }

  for (const auto & [name, data] : bool_inputs) {
    if (name == "lanes_has_speed_limit") {
      add_bool_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / NUM_SEGMENTS_IN_LANE), NUM_SEGMENTS_IN_LANE, 1});
    } else if (name == "route_lanes_has_speed_limit") {
      add_bool_tensor(
        name, data,
        {static_cast<int64_t>(data.size() / NUM_SEGMENTS_IN_ROUTE), NUM_SEGMENTS_IN_ROUTE, 1});
    } else {
      throw std::runtime_error("Unsupported ONNX Runtime bool input: " + name);
    }
  }

  std::vector<const char *> output_name_ptrs;
  output_name_ptrs.reserve(output_names.size());
  for (const auto & name : output_names) {
    output_name_ptrs.push_back(name.c_str());
  }

  auto output_tensors = session_.Run(
    Ort::RunOptions{nullptr}, input_name_ptrs.data(), input_tensors.data(), input_tensors.size(),
    output_name_ptrs.data(), output_name_ptrs.size());

  std::unordered_map<std::string, std::vector<float>> outputs;
  for (size_t i = 0; i < output_tensors.size(); ++i) {
    auto info = output_tensors[i].GetTensorTypeAndShapeInfo();
    const auto element_count = info.GetElementCount();
    const float * data = output_tensors[i].GetTensorData<float>();
    outputs.emplace(output_names.at(i), std::vector<float>(data, data + element_count));
  }
  return outputs;
}

OnnxruntimeSingleStepInference::OnnxruntimeSingleStepInference(
  const std::string & model_path, const std::string & execution_provider,
  const std::string & plugins_path, const int batch_size)
: batch_size_(batch_size),
  model_(model_path, parse_execution_provider(execution_provider), plugins_path)
{
}

InferenceResult OnnxruntimeSingleStepInference::infer(
  const preprocess::InputDataMap & input_data_map)
{
  auto start = std::chrono::steady_clock::now();
  try {
    const auto outputs = model_.run(
      to_float_input_map(single_step_float_inputs(input_data_map, batch_size_)),
      speed_limit_bool_inputs(input_data_map), {"prediction", "turn_indicator_logit"});

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    InferenceOutput output;
    output.outputs = std::make_pair(outputs.at("prediction"), outputs.at("turn_indicator_logit"));
    output.inference_time_ms = elapsed.count();
    output.is_denormalized = true;
    return output;
  } catch (const std::exception & e) {
    return tl::unexpected(std::string{e.what()});
  }
}

OnnxruntimeMultiStepInference::OnnxruntimeMultiStepInference(
  const std::string & encoder_model_path, const std::string & decoder_model_path,
  const std::string & turn_indicator_model_path, const std::string & execution_provider,
  const std::string & plugins_path, const int batch_size, const int dpm_solver_steps,
  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances)
: batch_size_(batch_size),
  dpm_solver_steps_(dpm_solver_steps),
  guidances_(std::move(guidances)),
  encoder_model_(encoder_model_path, parse_execution_provider(execution_provider), plugins_path),
  decoder_model_(decoder_model_path, parse_execution_provider(execution_provider), plugins_path),
  turn_indicator_model_(
    turn_indicator_model_path, parse_execution_provider(execution_provider), plugins_path)
{
}

std::vector<float> OnnxruntimeMultiStepInference::create_diffusion_time(float t) const
{
  return std::vector<float>(batch_size_ * MAX_NUM_AGENTS * (OUTPUT_T + 1), t);
}

std::vector<float> OnnxruntimeMultiStepInference::create_current_states(
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

void OnnxruntimeMultiStepInference::apply_prefix_constraint(
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

std::vector<float> OnnxruntimeMultiStepInference::evaluate_decoder(
  const std::vector<float> & x, const float t)
{
  const auto diffusion_time = create_diffusion_time(t);
  const auto outputs = decoder_model_.run(
    {{"encoding", encoding_},
     {"sampled_trajectories", x},
     {"diffusion_time", diffusion_time},
     {"neighbor_agents_past", decoder_neighbor_agents_past_}},
    {}, {"model_output"});
  return outputs.at("model_output");
}

DpmSolver::SampleResult OnnxruntimeMultiStepInference::run_dpm_solver(
  const preprocess::InputDataMap & input_data_map)
{
  const std::vector<float> current_states = create_current_states(input_data_map);
  decoder_neighbor_agents_past_ = input_data_map.at("neighbor_agents_past");
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

InferenceResult OnnxruntimeMultiStepInference::infer(
  const preprocess::InputDataMap & input_data_map)
{
  auto start = std::chrono::steady_clock::now();
  try {
    const auto encoder_outputs = encoder_model_.run(
      to_float_input_map(encoder_float_inputs(input_data_map, batch_size_)),
      speed_limit_bool_inputs(input_data_map), {"encoding"});
    encoding_ = encoder_outputs.at("encoding");

    auto solver_result = run_dpm_solver(input_data_map);

    const auto turn_indicator_outputs = turn_indicator_model_.run(
      {{"encoding", encoding_}, {"final_x0", solver_result.final_x}}, {}, {"turn_indicator_logit"});

    std::vector<float> denoising_predictions;
    for (const auto & step : solver_result.denoising_steps) {
      denoising_predictions.insert(denoising_predictions.end(), step.begin(), step.end());
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    InferenceOutput output;
    output.outputs = std::make_pair(
      std::move(solver_result.final_x), turn_indicator_outputs.at("turn_indicator_logit"));
    output.denoising_predictions = std::move(denoising_predictions);
    output.denoising_timesteps = std::move(solver_result.denoising_timesteps);
    output.inference_time_ms = elapsed.count();
    output.is_denormalized = false;
    output.guidance_triggered = std::move(solver_result.guidance_triggered);
    return output;
  } catch (const std::exception & e) {
    return tl::unexpected(std::string{e.what()});
  }
}

}  // namespace autoware::diffusion_planner
