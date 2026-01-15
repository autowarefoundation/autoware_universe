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

TensorrtInference::TensorrtInference(
  const std::string & model_path, const std::string & plugins_path, int batch_size)
: batch_size_(batch_size), plugins_path_(plugins_path)
{
  const int batch_size_local = batch_size_;

  const size_t sampled_trajectories_size =
    batch_size_local * std::accumulate(
                         SAMPLED_TRAJECTORIES_SHAPE.begin() + 1, SAMPLED_TRAJECTORIES_SHAPE.end(),
                         1L, std::multiplies<>());
  const size_t ego_history_size =
    batch_size_local *
    std::accumulate(
      EGO_HISTORY_SHAPE.begin() + 1, EGO_HISTORY_SHAPE.end(), 1L, std::multiplies<>());
  const size_t ego_current_state_size =
    batch_size_local *
    std::accumulate(
      EGO_CURRENT_STATE_SHAPE.begin() + 1, EGO_CURRENT_STATE_SHAPE.end(), 1L, std::multiplies<>());
  const size_t neighbor_agents_past_size =
    batch_size_local *
    std::accumulate(NEIGHBOR_SHAPE.begin() + 1, NEIGHBOR_SHAPE.end(), 1L, std::multiplies<>());
  const size_t static_objects_size =
    batch_size_local *
    std::accumulate(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end(), 1L, std::multiplies<>());
  const size_t lanes_size =
    batch_size_local *
    std::accumulate(LANES_SHAPE.begin() + 1, LANES_SHAPE.end(), 1L, std::multiplies<>());
  const size_t lanes_has_speed_limit_size =
    batch_size_local * std::accumulate(
                         LANES_HAS_SPEED_LIMIT_SHAPE.begin() + 1, LANES_HAS_SPEED_LIMIT_SHAPE.end(),
                         1L, std::multiplies<>());
  const size_t lanes_speed_limit_size =
    batch_size_local *
    std::accumulate(
      LANES_SPEED_LIMIT_SHAPE.begin() + 1, LANES_SPEED_LIMIT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t route_lanes_size =
    batch_size_local *
    std::accumulate(
      ROUTE_LANES_SHAPE.begin() + 1, ROUTE_LANES_SHAPE.end(), 1L, std::multiplies<>());
  const size_t route_lanes_has_speed_limit_size =
    batch_size_local * std::accumulate(
                         ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.begin() + 1,
                         ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t route_lanes_speed_limit_size =
    batch_size_local * std::accumulate(
                         ROUTE_LANES_SPEED_LIMIT_SHAPE.begin() + 1,
                         ROUTE_LANES_SPEED_LIMIT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t polygons_size =
    batch_size_local *
    std::accumulate(POLYGONS_SHAPE.begin() + 1, POLYGONS_SHAPE.end(), 1L, std::multiplies<>());
  const size_t line_strings_size =
    batch_size_local *
    std::accumulate(
      LINE_STRINGS_SHAPE.begin() + 1, LINE_STRINGS_SHAPE.end(), 1L, std::multiplies<>());
  const size_t goal_pose_size =
    batch_size_local *
    std::accumulate(GOAL_POSE_SHAPE.begin() + 1, GOAL_POSE_SHAPE.end(), 1L, std::multiplies<>());
  const size_t ego_shape_size =
    batch_size_local *
    std::accumulate(EGO_SHAPE_SHAPE.begin() + 1, EGO_SHAPE_SHAPE.end(), 1L, std::multiplies<>());
  const size_t turn_indicators_size =
    batch_size_local *
    std::accumulate(
      TURN_INDICATORS_SHAPE.begin() + 1, TURN_INDICATORS_SHAPE.end(), 1L, std::multiplies<>());
  const size_t output_size =
    batch_size_local *
    std::accumulate(OUTPUT_SHAPE.begin() + 1, OUTPUT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t turn_indicator_logit_size =
    batch_size_local * std::accumulate(
                         TURN_INDICATOR_LOGIT_SHAPE.begin() + 1, TURN_INDICATOR_LOGIT_SHAPE.end(),
                         1L, std::multiplies<>());

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

  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_size);
  turn_indicator_logit_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicator_logit_size);
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

  const std::string precision = "fp32";

  const std::filesystem::path engine_path(model_path);
  const std::string engine_file_path =
    (engine_path.parent_path() /
     (engine_path.stem().string() + "_batch" + std::to_string(batch_size) + ".engine"))
      .string();

  const auto trt_config = tensorrt_common::TrtCommonConfig(model_path, precision, engine_file_path);
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
  network_io.emplace_back("prediction", to_dynamic_dims(OUTPUT_SHAPE));
  network_io.emplace_back("turn_indicator_logit", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE));

  auto network_io_ptr = std::make_unique<std::vector<NetworkIO>>(network_io);
  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);

  network_trt_ptr_ = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{plugins_path_});

  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine." + plugins_path_);
  }
}

TensorrtInference::InferenceResult TensorrtInference::infer(
  const preprocess::InputDataMap & input_data_map)
{
  const auto sampled_trajectories = input_data_map.at("sampled_trajectories");
  const auto ego_history = input_data_map.at("ego_agent_past");
  const auto ego_current_state = input_data_map.at("ego_current_state");
  const auto neighbor_agents_past = input_data_map.at("neighbor_agents_past");
  const auto static_objects = input_data_map.at("static_objects");
  const auto lanes = input_data_map.at("lanes");
  const auto lanes_speed_limit = input_data_map.at("lanes_speed_limit");
  const auto route_lanes = input_data_map.at("route_lanes");
  const auto route_lanes_speed_limit = input_data_map.at("route_lanes_speed_limit");
  const auto polygons = input_data_map.at("polygons");
  const auto line_strings = input_data_map.at("line_strings");
  const auto goal_pose = input_data_map.at("goal_pose");
  const auto ego_shape = input_data_map.at("ego_shape");
  const auto turn_indicators = input_data_map.at("turn_indicators");

  const int batch_size = batch_size_;
  const size_t lane_speed_tensor_num_elements =
    batch_size *
    std::accumulate(
      LANES_SPEED_LIMIT_SHAPE.begin() + 1, LANES_SPEED_LIMIT_SHAPE.end(), 1, std::multiplies<>());
  std::vector<uint8_t> speed_bool_array(lane_speed_tensor_num_elements);
  for (size_t i = 0; i < lane_speed_tensor_num_elements; ++i) {
    speed_bool_array[i] = (lanes_speed_limit[i] > std::numeric_limits<float>::epsilon()) ? 1 : 0;
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    sampled_trajectories_d_.get(), sampled_trajectories.data(),
    sampled_trajectories.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    ego_history_d_.get(), ego_history.data(), ego_history.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    ego_current_state_d_.get(), ego_current_state.data(), ego_current_state.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    neighbor_agents_past_d_.get(), neighbor_agents_past.data(),
    neighbor_agents_past.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    static_objects_d_.get(), static_objects.data(), static_objects.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(
    cudaMemcpy(lanes_d_.get(), lanes.data(), lanes.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    lanes_speed_limit_d_.get(), lanes_speed_limit.data(), lanes_speed_limit.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    route_lanes_d_.get(), route_lanes.data(), route_lanes.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    polygons_d_.get(), polygons.data(), polygons.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    line_strings_d_.get(), line_strings.data(), line_strings.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    lanes_has_speed_limit_d_.get(), speed_bool_array.data(),
    lane_speed_tensor_num_elements * sizeof(uint8_t), cudaMemcpyHostToDevice));

  const size_t route_lanes_has_speed_limit_tensor_num_elements =
    batch_size * std::accumulate(
                   ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.begin() + 1,
                   ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.end(), 1, std::multiplies<>());
  std::vector<uint8_t> route_has_speed_bool_array(route_lanes_has_speed_limit_tensor_num_elements);
  for (size_t i = 0; i < route_lanes_has_speed_limit_tensor_num_elements; ++i) {
    route_has_speed_bool_array[i] =
      (route_lanes_speed_limit[i] > std::numeric_limits<float>::epsilon()) ? 1 : 0;
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    route_lanes_speed_limit_d_.get(), route_lanes_speed_limit.data(),
    route_lanes_speed_limit.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    route_lanes_has_speed_limit_d_.get(), route_has_speed_bool_array.data(),
    route_lanes_has_speed_limit_tensor_num_elements * sizeof(uint8_t), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    goal_pose_d_.get(), goal_pose.data(), goal_pose.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    ego_shape_d_.get(), ego_shape.data(), ego_shape.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    turn_indicators_d_.get(), turn_indicators.data(), turn_indicators.size() * sizeof(float),
    cudaMemcpyHostToDevice));

  const auto to_dims_with_batch = [batch_size](auto const & arr) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(arr.size());
    dims.d[0] = batch_size;
    for (size_t i = 1; i < arr.size(); ++i) {
      dims.d[i] = static_cast<int>(arr[i]);
    }
    return dims;
  };

  bool set_input_shapes = true;
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "sampled_trajectories", to_dims_with_batch(SAMPLED_TRAJECTORIES_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("ego_agent_past", to_dims_with_batch(EGO_HISTORY_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "ego_current_state", to_dims_with_batch(EGO_CURRENT_STATE_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("neighbor_agents_past", to_dims_with_batch(NEIGHBOR_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("static_objects", to_dims_with_batch(STATIC_OBJECTS_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape("lanes", to_dims_with_batch(LANES_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "lanes_has_speed_limit", to_dims_with_batch(LANES_HAS_SPEED_LIMIT_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "lanes_speed_limit", to_dims_with_batch(LANES_SPEED_LIMIT_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("route_lanes", to_dims_with_batch(ROUTE_LANES_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("polygons", to_dims_with_batch(POLYGONS_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("line_strings", to_dims_with_batch(LINE_STRINGS_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "route_lanes_speed_limit", to_dims_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "route_lanes_has_speed_limit", to_dims_with_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("goal_pose", to_dims_with_batch(GOAL_POSE_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("ego_shape", to_dims_with_batch(EGO_SHAPE_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("turn_indicators", to_dims_with_batch(TURN_INDICATORS_SHAPE));

  if (!set_input_shapes) {
    InferenceResult result;
    result.error_msg = "Failed to set input shapes for inference.";
    return result;
  }

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
  network_trt_ptr_->setTensorAddress("prediction", output_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicator_logit", turn_indicator_logit_d_.get());

  const auto status = network_trt_ptr_->enqueueV3(stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!status) {
    InferenceResult result;
    result.error_msg = "Failed to enqueue and do inference.";
    return result;
  }

  const size_t output_num_elements =
    batch_size *
    std::accumulate(OUTPUT_SHAPE.begin() + 1, OUTPUT_SHAPE.end(), 1UL, std::multiplies<>());
  std::vector<float> output_host(output_num_elements);
  cudaMemcpy(
    output_host.data(), output_d_.get(), output_num_elements * sizeof(float),
    cudaMemcpyDeviceToHost);

  const size_t turn_indicator_num_elements =
    batch_size * std::accumulate(
                   TURN_INDICATOR_LOGIT_SHAPE.begin() + 1, TURN_INDICATOR_LOGIT_SHAPE.end(), 1UL,
                   std::multiplies<>());

  std::vector<float> logit_host(turn_indicator_num_elements);
  cudaMemcpy(
    logit_host.data(), turn_indicator_logit_d_.get(), turn_indicator_num_elements * sizeof(float),
    cudaMemcpyDeviceToHost);

  InferenceResult result;
  result.outputs = std::make_pair(std::move(output_host), std::move(logit_host));
  return result;
}

}  // namespace autoware::diffusion_planner
