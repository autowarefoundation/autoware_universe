// Copyright 2025 TIER IV.
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

#ifndef AUTOWARE__TENSORRT_VAD__NETWORKS__NET_HPP_
#define AUTOWARE__TENSORRT_VAD__NETWORKS__NET_HPP_

#include "autoware/tensorrt_vad/networks/tensor.hpp"
#include "autoware/tensorrt_vad/ros_vad_logger.hpp"
#include "autoware/tensorrt_vad/vad_config.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_vad
{

// Network type enumeration
enum class NetworkType { BACKBONE, HEAD, HEAD_NO_PREV };

// Helper function to convert NetworkType to string (for backward compatibility)
std::string toString(NetworkType type);

std::unique_ptr<autoware::tensorrt_common::TrtCommon> build_engine(
  const autoware::tensorrt_common::TrtCommonConfig & trt_common_config,
  const std::vector<autoware::tensorrt_common::NetworkIO> & network_io,
  const std::string & engine_name, const std::string & plugins_path,
  std::shared_ptr<VadLogger> logger);

class Net
{
public:
  TensorMap bindings;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common;
  std::shared_ptr<VadLogger> logger_;
  NetworkType network_type_;

public:
  Net(
    const VadConfig & vad_config,
    const autoware::tensorrt_common::TrtCommonConfig & trt_common_config, NetworkType network_type,
    const std::string & plugins_path, std::shared_ptr<VadLogger> logger);

  virtual std::vector<autoware::tensorrt_common::NetworkIO> setup_network_io(
    const VadConfig & vad_config) = 0;

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> init_tensorrt(
    const VadConfig & vad_config,
    const autoware::tensorrt_common::TrtCommonConfig & trt_common_config,
    const std::string & plugins_path);

  void enqueue(cudaStream_t stream);
  void set_input_tensor(TensorMap & ext);

  virtual ~Net();
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__NETWORKS__NET_HPP_
