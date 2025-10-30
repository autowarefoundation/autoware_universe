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

#include "autoware/tensorrt_vad/networks/net.hpp"

namespace autoware::tensorrt_vad
{

// NetworkType utility functions implementation
std::string toString(NetworkType type)
{
  switch (type) {
    case NetworkType::BACKBONE:
      return "backbone";
    case NetworkType::HEAD:
      return "head";
    case NetworkType::HEAD_NO_PREV:
      return "head_no_prev";
    default:
      return "unknown";
  }
}

std::unique_ptr<autoware::tensorrt_common::TrtCommon> build_engine(
  const autoware::tensorrt_common::TrtCommonConfig & trt_common_config,
  const std::vector<autoware::tensorrt_common::NetworkIO> & network_io,
  const std::string & engine_name, const std::string & plugins_path,
  std::shared_ptr<VadLogger> logger)
{
  logger->info("Building " + engine_name + " engine...");

  auto trt_common = std::make_unique<autoware::tensorrt_common::TrtCommon>(
    trt_common_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
    std::vector<std::string>{plugins_path});
  auto network_io_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io);
  if (!trt_common->setup(nullptr, std::move(network_io_ptr))) {
    logger->error("Failed to setup " + engine_name + " TrtCommon");
    return nullptr;
  }
  logger->info(engine_name + " engine built successfully");
  return trt_common;
}

std::unique_ptr<autoware::tensorrt_common::TrtCommon> Net::init_tensorrt(
  const VadConfig & vad_config,
  const autoware::tensorrt_common::TrtCommonConfig & trt_common_config,
  const std::string & plugins_path)
{
  logger_->info("Initializing TensorRT engine");

  // Generate NetworkIO using the overridden implementation
  auto network_io = setup_network_io(vad_config);

  // Build engine using network type name
  std::string engine_name = toString(network_type_);
  auto engine = build_engine(trt_common_config, network_io, engine_name, plugins_path, logger_);
  if (!engine) {
    logger_->error("Failed to build " + engine_name + " engine");
    return nullptr;
  }

  logger_->info(engine_name + " engine initialization completed successfully");
  return engine;
}

// Net class implementation

Net::Net(
  const VadConfig & /*vad_config*/,
  const autoware::tensorrt_common::TrtCommonConfig & /*trt_common_config*/,
  NetworkType network_type, const std::string & /*plugins_path*/, std::shared_ptr<VadLogger> logger)
: logger_(logger), network_type_(network_type)
{
  // TensorRT initialization is handled by derived classes
}

void Net::set_input_tensor(TensorMap & ext)
{
  int32_t nb = trt_common->getNbIOTensors();

  for (int32_t n = 0; n < nb; n++) {
    std::string name = trt_common->getIOTensorName(n);
    nvinfer1::Dims d = trt_common->getTensorShape(name.c_str());
    nvinfer1::DataType dtype = nvinfer1::DataType::kFLOAT;

    if (ext.find(name) != ext.end()) {
      // use external memory
      trt_common->setTensorAddress(name.c_str(), ext[name]->ptr);
    } else {
      bindings[name] = std::make_shared<Tensor>(name, d, dtype, logger_);
      trt_common->setTensorAddress(name.c_str(), bindings[name]->ptr);
    }
  }
}

void Net::enqueue(cudaStream_t stream)
{
  trt_common->enqueueV3(stream);
}

Net::~Net()
{
  for (auto & pair : bindings) {
    pair.second.reset();
  }
  bindings.clear();
}

}  // namespace autoware::tensorrt_vad
