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

#include "autoware/diffusion_planner/inference/tensorrt_inference.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;

namespace
{
template <class Container>
size_t num_elements(const Container & shape)
{
  return std::accumulate(shape.begin() + 1, shape.end(), size_t{1}, std::multiplies<>());
}
}  // namespace

TensorrtInference::TensorrtInference(
  const std::string & model_path, const std::string & plugins_path, int batch_size)
: batch_size_(batch_size), plugins_path_(plugins_path)
{
  const size_t sampled_trajectories_size = batch_size_ * num_elements(SAMPLED_TRAJECTORIES_SHAPE);
  const size_t ego_history_size = batch_size_ * num_elements(EGO_HISTORY_SHAPE);
  const size_t ego_current_state_size = batch_size_ * num_elements(EGO_CURRENT_STATE_SHAPE);
  const size_t neighbor_agents_past_size = batch_size_ * num_elements(NEIGHBOR_SHAPE);
  const size_t static_objects_size = batch_size_ * num_elements(STATIC_OBJECTS_SHAPE);
  const size_t lanes_size = batch_size_ * num_elements(LANES_SHAPE);
  const size_t lanes_has_speed_limit_size = batch_size_ * num_elements(LANES_HAS_SPEED_LIMIT_SHAPE);
  const size_t lanes_speed_limit_size = batch_size_ * num_elements(LANES_SPEED_LIMIT_SHAPE);
  const size_t route_lanes_size = batch_size_ * num_elements(ROUTE_LANES_SHAPE);
  const size_t route_lanes_has_speed_limit_size =
    batch_size_ * num_elements(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE);
  const size_t route_lanes_speed_limit_size =
    batch_size_ * num_elements(ROUTE_LANES_SPEED_LIMIT_SHAPE);
  const size_t polygons_size = batch_size_ * num_elements(POLYGONS_SHAPE);
  const size_t line_strings_size = batch_size_ * num_elements(LINE_STRINGS_SHAPE);
  const size_t goal_pose_size = batch_size_ * num_elements(GOAL_POSE_SHAPE);
  const size_t ego_shape_size = batch_size_ * num_elements(EGO_SHAPE_SHAPE);
  const size_t turn_indicators_size = batch_size_ * num_elements(TURN_INDICATORS_SHAPE);
  const size_t delay_size = batch_size_ * num_elements(DELAY_SHAPE);
  const size_t output_size = batch_size_ * num_elements(OUTPUT_SHAPE);
  const size_t turn_indicator_logit_size = batch_size_ * num_elements(TURN_INDICATOR_LOGIT_SHAPE);

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

TensorrtInference::~TensorrtInference()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

void TensorrtInference::load_engine(const std::string & model_path)
{
  const int batch_size = batch_size_;

  const auto to_dynamic_dims = [batch_size](auto const & arr) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(arr.size());
    dims.d[0] = (batch_size == 1 ? 1 : -1);
    for (size_t i = 1; i < arr.size(); ++i) {
      dims.d[i] = static_cast<int>(arr[i]);
    }
    return dims;
  };

  const auto make_dynamic_dims = [batch_size](
                                   const std::string & name, const nvinfer1::Dims & dims) {
    nvinfer1::Dims min_dims = dims, opt_dims = dims, max_dims = dims;
    min_dims.d[0] = 1;
    opt_dims.d[0] = batch_size;
    max_dims.d[0] = batch_size;
    return ProfileDims{name, min_dims, opt_dims, max_dims};
  };

  const std::filesystem::path engine_path(model_path);
  const std::string engine_file_path =
    (engine_path.parent_path() /
     (engine_path.stem().string() + "_batch" + std::to_string(batch_size) + "_fp32.engine"))
      .string();

  const auto trt_config =
    tensorrt_common::TrtCommonConfig(model_path, "fp32", engine_file_path, 1ULL << 30, -1, false);
  trt_common_ = std::make_unique<autoware::tensorrt_common::TrtConvCalib>(trt_config);

  std::vector<ProfileDims> profile_dims;
  profile_dims.emplace_back(
    make_dynamic_dims("sampled_trajectories", to_dynamic_dims(SAMPLED_TRAJECTORIES_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("ego_agent_past", to_dynamic_dims(EGO_HISTORY_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("ego_current_state", to_dynamic_dims(EGO_CURRENT_STATE_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("neighbor_agents_past", to_dynamic_dims(NEIGHBOR_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("static_objects", to_dynamic_dims(STATIC_OBJECTS_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("lanes", to_dynamic_dims(LANES_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("lanes_speed_limit", to_dynamic_dims(LANES_SPEED_LIMIT_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("lanes_has_speed_limit", to_dynamic_dims(LANES_HAS_SPEED_LIMIT_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("route_lanes", to_dynamic_dims(ROUTE_LANES_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("polygons", to_dynamic_dims(POLYGONS_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("line_strings", to_dynamic_dims(LINE_STRINGS_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims(
    "route_lanes_has_speed_limit", to_dynamic_dims(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("route_lanes_speed_limit", to_dynamic_dims(ROUTE_LANES_SPEED_LIMIT_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("goal_pose", to_dynamic_dims(GOAL_POSE_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("ego_shape", to_dynamic_dims(EGO_SHAPE_SHAPE)));
  profile_dims.emplace_back(
    make_dynamic_dims("turn_indicators", to_dynamic_dims(TURN_INDICATORS_SHAPE)));
  profile_dims.emplace_back(make_dynamic_dims("delay", to_dynamic_dims(DELAY_SHAPE)));

  std::vector<autoware::tensorrt_common::NetworkIO> network_io;
  network_io.emplace_back("sampled_trajectories", to_dynamic_dims(SAMPLED_TRAJECTORIES_SHAPE));
  network_io.emplace_back("ego_agent_past", to_dynamic_dims(EGO_HISTORY_SHAPE));
  network_io.emplace_back("ego_current_state", to_dynamic_dims(EGO_CURRENT_STATE_SHAPE));
  network_io.emplace_back("neighbor_agents_past", to_dynamic_dims(NEIGHBOR_SHAPE));
  network_io.emplace_back("static_objects", to_dynamic_dims(STATIC_OBJECTS_SHAPE));
  network_io.emplace_back("lanes", to_dynamic_dims(LANES_SHAPE));
  network_io.emplace_back("lanes_has_speed_limit", to_dynamic_dims(LANES_HAS_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("lanes_speed_limit", to_dynamic_dims(LANES_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("route_lanes", to_dynamic_dims(ROUTE_LANES_SHAPE));
  network_io.emplace_back("polygons", to_dynamic_dims(POLYGONS_SHAPE));
  network_io.emplace_back("line_strings", to_dynamic_dims(LINE_STRINGS_SHAPE));
  network_io.emplace_back(
    "route_lanes_has_speed_limit", to_dynamic_dims(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  network_io.emplace_back(
    "route_lanes_speed_limit", to_dynamic_dims(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("goal_pose", to_dynamic_dims(GOAL_POSE_SHAPE));
  network_io.emplace_back("ego_shape", to_dynamic_dims(EGO_SHAPE_SHAPE));
  network_io.emplace_back("turn_indicators", to_dynamic_dims(TURN_INDICATORS_SHAPE));
  network_io.emplace_back("delay", to_dynamic_dims(DELAY_SHAPE));
  network_io.emplace_back("prediction", to_dynamic_dims(OUTPUT_SHAPE));
  network_io.emplace_back("turn_indicator_logit", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE));

  auto network_io_ptr = std::make_unique<std::vector<NetworkIO>>(network_io);
  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);

  network_trt_ptr_ = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{plugins_path_});

  // Force single-stream execution to reduce scratch memory (~384MB → ~27MB).
  auto builder_config = network_trt_ptr_->getBuilderConfig();
  if (builder_config) {
    builder_config->setMaxAuxStreams(0);
  }

  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine." + plugins_path_);
  }

  bindBuffers();
}

void TensorrtInference::bindBuffers()
{
  const int batch_size = batch_size_;
  const auto to_dims_with_batch = [batch_size](auto const & arr) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(arr.size());
    dims.d[0] = batch_size;
    for (size_t i = 1; i < arr.size(); ++i) {
      dims.d[i] = static_cast<int>(arr[i]);
    }
    return dims;
  };

  // Set input shapes once (fixed batch_size)
  network_trt_ptr_->setInputShape(
    "sampled_trajectories", to_dims_with_batch(SAMPLED_TRAJECTORIES_SHAPE));
  network_trt_ptr_->setInputShape("ego_agent_past", to_dims_with_batch(EGO_HISTORY_SHAPE));
  network_trt_ptr_->setInputShape("ego_current_state", to_dims_with_batch(EGO_CURRENT_STATE_SHAPE));
  network_trt_ptr_->setInputShape("neighbor_agents_past", to_dims_with_batch(NEIGHBOR_SHAPE));
  network_trt_ptr_->setInputShape("static_objects", to_dims_with_batch(STATIC_OBJECTS_SHAPE));
  network_trt_ptr_->setInputShape("lanes", to_dims_with_batch(LANES_SHAPE));
  network_trt_ptr_->setInputShape(
    "lanes_has_speed_limit", to_dims_with_batch(LANES_HAS_SPEED_LIMIT_SHAPE));
  network_trt_ptr_->setInputShape("lanes_speed_limit", to_dims_with_batch(LANES_SPEED_LIMIT_SHAPE));
  network_trt_ptr_->setInputShape("route_lanes", to_dims_with_batch(ROUTE_LANES_SHAPE));
  network_trt_ptr_->setInputShape("polygons", to_dims_with_batch(POLYGONS_SHAPE));
  network_trt_ptr_->setInputShape("line_strings", to_dims_with_batch(LINE_STRINGS_SHAPE));
  network_trt_ptr_->setInputShape(
    "route_lanes_speed_limit", to_dims_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  network_trt_ptr_->setInputShape(
    "route_lanes_has_speed_limit", to_dims_with_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  network_trt_ptr_->setInputShape("goal_pose", to_dims_with_batch(GOAL_POSE_SHAPE));
  network_trt_ptr_->setInputShape("ego_shape", to_dims_with_batch(EGO_SHAPE_SHAPE));
  network_trt_ptr_->setInputShape("turn_indicators", to_dims_with_batch(TURN_INDICATORS_SHAPE));
  network_trt_ptr_->setInputShape("delay", to_dims_with_batch(DELAY_SHAPE));

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

void TensorrtInference::transferInputsToDevice(const preprocess::InputDataMap & input_data_map)
{
  // Helper lambda for async H2D of float tensors
  const auto h2d = [this](const auto & host_vec, const auto & device_ptr) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      device_ptr.get(), host_vec.data(), host_vec.size() * sizeof(float), cudaMemcpyHostToDevice,
      stream_));
  };

  h2d(input_data_map.at("sampled_trajectories"), sampled_trajectories_d_);
  h2d(input_data_map.at("ego_agent_past"), ego_history_d_);
  h2d(input_data_map.at("ego_current_state"), ego_current_state_d_);
  h2d(input_data_map.at("neighbor_agents_past"), neighbor_agents_past_d_);
  h2d(input_data_map.at("static_objects"), static_objects_d_);
  h2d(input_data_map.at("lanes"), lanes_d_);
  h2d(input_data_map.at("lanes_speed_limit"), lanes_speed_limit_d_);
  h2d(input_data_map.at("route_lanes"), route_lanes_d_);
  h2d(input_data_map.at("route_lanes_speed_limit"), route_lanes_speed_limit_d_);
  h2d(input_data_map.at("polygons"), polygons_d_);
  h2d(input_data_map.at("line_strings"), line_strings_d_);
  h2d(input_data_map.at("goal_pose"), goal_pose_d_);
  h2d(input_data_map.at("ego_shape"), ego_shape_d_);
  h2d(input_data_map.at("turn_indicators"), turn_indicators_d_);
  h2d(input_data_map.at("delay"), delay_d_);

  // CPU-side float→bool conversion for speed limit masks
  const auto convert_speed_mask =
    [this](const std::vector<float> & speed_limit, const auto & device_ptr, size_t count) {
      std::vector<uint8_t> bool_array(count);
      for (size_t i = 0; i < count; ++i) {
        bool_array[i] = speed_limit[i] > std::numeric_limits<float>::epsilon();
      }
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        device_ptr.get(), bool_array.data(), count * sizeof(uint8_t), cudaMemcpyHostToDevice,
        stream_));
    };

  convert_speed_mask(
    input_data_map.at("lanes_speed_limit"), lanes_has_speed_limit_d_,
    batch_size_ * num_elements(LANES_SPEED_LIMIT_SHAPE));
  convert_speed_mask(
    input_data_map.at("route_lanes_speed_limit"), route_lanes_has_speed_limit_d_,
    batch_size_ * num_elements(ROUTE_LANES_SPEED_LIMIT_SHAPE));
}

TensorrtInference::InferenceResult TensorrtInference::infer(
  const preprocess::InputDataMap & input_data_map)
{
  transferInputsToDevice(input_data_map);

  const bool status = network_trt_ptr_->enqueueV3(stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!status) {
    InferenceResult result;
    result.error_msg = "Failed to enqueue and do inference.";
    return result;
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

  InferenceResult result;
  result.outputs = std::make_pair(std::move(output_host), std::move(logit_host));
  return result;
}

}  // namespace autoware::diffusion_planner
