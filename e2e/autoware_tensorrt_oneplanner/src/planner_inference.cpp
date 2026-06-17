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

#include "autoware/tensorrt_oneplanner/planner_inference.hpp"

#include "autoware/tensorrt_oneplanner/detail.hpp"
#include "autoware/tensorrt_oneplanner/dimensions.hpp"

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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_oneplanner
{
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;

// All conditioning-input shapes are shared with the diffusion planner.
using autoware::diffusion_planner::DELAY_SHAPE;
using autoware::diffusion_planner::EGO_CURRENT_STATE_SHAPE;
using autoware::diffusion_planner::EGO_HISTORY_SHAPE;
using autoware::diffusion_planner::EGO_SHAPE_SHAPE;
using autoware::diffusion_planner::GOAL_POSE_SHAPE;
using autoware::diffusion_planner::LANES_HAS_SPEED_LIMIT_SHAPE;
using autoware::diffusion_planner::LANES_SHAPE;
using autoware::diffusion_planner::LANES_SPEED_LIMIT_SHAPE;
using autoware::diffusion_planner::LINE_STRINGS_SHAPE;
using autoware::diffusion_planner::POLYGONS_SHAPE;
using autoware::diffusion_planner::ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE;
using autoware::diffusion_planner::ROUTE_LANES_SHAPE;
using autoware::diffusion_planner::ROUTE_LANES_SPEED_LIMIT_SHAPE;
using autoware::diffusion_planner::TURN_INDICATOR_LOGIT_SHAPE;
using autoware::diffusion_planner::TURN_INDICATORS_SHAPE;

using detail::num_elements;

PlannerInference::PlannerInference(
  const std::string & model_path, const std::string & plugins_path, int batch_size,
  const float * bev_feature_d, int64_t bev_channels, int64_t bev_size, cudaStream_t stream)
: batch_size_(batch_size),
  plugins_path_(plugins_path),
  bev_feature_d_(bev_feature_d),
  bev_channels_(bev_channels),
  bev_size_(bev_size),
  stream_(stream)
{
  if (batch_size_ != 1) {
    // The BEV feature map is produced once per frame; batched sampling would require
    // replicating it on-device. Restrict to 1 until that is needed.
    throw std::invalid_argument("autoware_tensorrt_oneplanner currently supports batch_size == 1");
  }

  const size_t output_size = batch_size_ * num_elements(EGO_PREDICTION_SHAPE);
  const size_t turn_indicator_logit_size = batch_size_ * num_elements(TURN_INDICATOR_LOGIT_SHAPE);

  sampled_trajectories_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements(EGO_SAMPLED_TRAJECTORIES_SHAPE));
  ego_history_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(EGO_HISTORY_SHAPE));
  ego_current_state_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(EGO_CURRENT_STATE_SHAPE));
  lanes_d_ = autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(LANES_SHAPE));
  lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(
    batch_size_ * num_elements(LANES_HAS_SPEED_LIMIT_SHAPE));
  lanes_speed_limit_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(LANES_SPEED_LIMIT_SHAPE));
  route_lanes_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(ROUTE_LANES_SHAPE));
  route_lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(
    batch_size_ * num_elements(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  route_lanes_speed_limit_d_ = autoware::cuda_utils::make_unique<float[]>(
    batch_size_ * num_elements(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  polygons_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(POLYGONS_SHAPE));
  line_strings_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(LINE_STRINGS_SHAPE));
  goal_pose_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(GOAL_POSE_SHAPE));
  ego_shape_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(EGO_SHAPE_SHAPE));
  turn_indicators_d_ =
    autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(TURN_INDICATORS_SHAPE));
  delay_d_ = autoware::cuda_utils::make_unique<float[]>(batch_size_ * num_elements(DELAY_SHAPE));

  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_size);
  turn_indicator_logit_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicator_logit_size);

  output_num_elements_ = output_size;
  logit_num_elements_ = turn_indicator_logit_size;
  output_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(output_num_elements_, cudaHostAllocDefault);
  logit_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(logit_num_elements_, cudaHostAllocDefault);

  load_engine(model_path);
}

void PlannerInference::load_engine(const std::string & model_path)
{
  const auto to_dims = [](auto const & arr) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(arr.size());
    for (size_t i = 0; i < arr.size(); ++i) {
      dims.d[i] = static_cast<int>(arr[i]);
    }
    return dims;
  };

  const std::filesystem::path engine_path(model_path);
  const std::string engine_file_path =
    (engine_path.parent_path() /
     (engine_path.stem().string() + "_batch" + std::to_string(batch_size_) + "_fp32.engine"))
      .string();

  const auto trt_config =
    tensorrt_common::TrtCommonConfig(model_path, "fp32", engine_file_path, 1ULL << 30U, -1, false);

  const nvinfer1::Dims bev_dims{4, {1, bev_channels_, bev_size_, bev_size_}};

  std::vector<NetworkIO> network_io;
  network_io.emplace_back("sampled_trajectories", to_dims(EGO_SAMPLED_TRAJECTORIES_SHAPE));
  network_io.emplace_back("ego_agent_past", to_dims(EGO_HISTORY_SHAPE));
  network_io.emplace_back("ego_current_state", to_dims(EGO_CURRENT_STATE_SHAPE));
  network_io.emplace_back("lanes", to_dims(LANES_SHAPE));
  network_io.emplace_back("lanes_has_speed_limit", to_dims(LANES_HAS_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("lanes_speed_limit", to_dims(LANES_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("route_lanes", to_dims(ROUTE_LANES_SHAPE));
  network_io.emplace_back(
    "route_lanes_has_speed_limit", to_dims(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("route_lanes_speed_limit", to_dims(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  network_io.emplace_back("polygons", to_dims(POLYGONS_SHAPE));
  network_io.emplace_back("line_strings", to_dims(LINE_STRINGS_SHAPE));
  network_io.emplace_back("goal_pose", to_dims(GOAL_POSE_SHAPE));
  network_io.emplace_back("ego_shape", to_dims(EGO_SHAPE_SHAPE));
  network_io.emplace_back("turn_indicators", to_dims(TURN_INDICATORS_SHAPE));
  network_io.emplace_back("delay", to_dims(DELAY_SHAPE));
  network_io.emplace_back("bev_feature_map", bev_dims);
  network_io.emplace_back("prediction", to_dims(EGO_PREDICTION_SHAPE));
  network_io.emplace_back("turn_indicator_logit", to_dims(TURN_INDICATOR_LOGIT_SHAPE));

  auto network_io_ptr = std::make_unique<std::vector<NetworkIO>>(network_io);

  network_trt_ptr_ = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{plugins_path_});

  // Force single-stream execution to reduce scratch memory.
  auto builder_config = network_trt_ptr_->getBuilderConfig();
  if (builder_config) {
    builder_config->setMaxAuxStreams(0);
  }

  if (!network_trt_ptr_->setup(nullptr, std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup the OnePlanner head TRT engine.");
  }

  bind_buffers();
}

void PlannerInference::bind_buffers()
{
  network_trt_ptr_->setTensorAddress("sampled_trajectories", sampled_trajectories_d_.get());
  network_trt_ptr_->setTensorAddress("ego_agent_past", ego_history_d_.get());
  network_trt_ptr_->setTensorAddress("ego_current_state", ego_current_state_d_.get());
  network_trt_ptr_->setTensorAddress("lanes", lanes_d_.get());
  network_trt_ptr_->setTensorAddress("lanes_has_speed_limit", lanes_has_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("lanes_speed_limit", lanes_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("route_lanes", route_lanes_d_.get());
  network_trt_ptr_->setTensorAddress(
    "route_lanes_has_speed_limit", route_lanes_has_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("route_lanes_speed_limit", route_lanes_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("polygons", polygons_d_.get());
  network_trt_ptr_->setTensorAddress("line_strings", line_strings_d_.get());
  network_trt_ptr_->setTensorAddress("goal_pose", goal_pose_d_.get());
  network_trt_ptr_->setTensorAddress("ego_shape", ego_shape_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicators", turn_indicators_d_.get());
  network_trt_ptr_->setTensorAddress("delay", delay_d_.get());
  // Bound directly to the BevEncoder output buffer: device-to-device, no host round trip.
  network_trt_ptr_->setTensorAddress("bev_feature_map", const_cast<float *>(bev_feature_d_));
  network_trt_ptr_->setTensorAddress("prediction", output_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicator_logit", turn_indicator_logit_d_.get());
}

void PlannerInference::transfer_inputs_to_device(const InputDataMap & input_data_map)
{
  const auto h2d = [this](const auto & host_vec, const auto & device_ptr) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      device_ptr.get(), host_vec.data(), host_vec.size() * sizeof(float), cudaMemcpyHostToDevice,
      stream_));
  };

  h2d(input_data_map.at("sampled_trajectories"), sampled_trajectories_d_);
  h2d(input_data_map.at("ego_agent_past"), ego_history_d_);
  h2d(input_data_map.at("ego_current_state"), ego_current_state_d_);
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

PlannerInference::InferenceResult PlannerInference::infer(const InputDataMap & input_data_map)
{
  transfer_inputs_to_device(input_data_map);

  // enqueueV3 returns the enqueue status synchronously; the inference itself runs
  // on the stream. The output D2H copies are queued on the same stream, so they
  // are ordered after inference without a separate synchronization — a single
  // sync at the end suffices (one fewer host/GPU round-trip per frame).
  if (!network_trt_ptr_->enqueueV3(stream_)) {
    InferenceResult result;
    result.error_msg = "Failed to enqueue and do inference.";
    return result;
  }

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

}  // namespace autoware::tensorrt_oneplanner
