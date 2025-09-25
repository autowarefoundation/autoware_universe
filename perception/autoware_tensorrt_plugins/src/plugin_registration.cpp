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

#include "autoware/tensorrt_plugins/argsort_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/get_indices_pairs_implicit_gemm_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/get_indices_pairs_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/implicit_gemm_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/indice_conv_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/multi_scale_deformable_attention_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/quick_cumsum_cuda_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/rotate_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/segment_csr_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/select_and_pad_plugin_creator.hpp"
#include "autoware/tensorrt_plugins/unique_plugin_creator.hpp"

#include <NvInferRuntime.h>

#include <cstdint>
#include <mutex>

class ThreadSafeLoggerFinder
{
public:
  ThreadSafeLoggerFinder() = default;

  // Set the logger finder.
  void setLoggerFinder(nvinfer1::ILoggerFinder * finder)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (logger_finder_ == nullptr && finder != nullptr) {
      logger_finder_ = finder;
    }
  }

  // Get the logger.
  nvinfer1::ILogger * getLogger() noexcept
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (logger_finder_ != nullptr) {
      return logger_finder_->findLogger();
    }
    return nullptr;
  }

private:
  nvinfer1::ILoggerFinder * logger_finder_{nullptr};
  std::mutex mutex_;
};

ThreadSafeLoggerFinder logger_finder;

extern "C" void setLoggerFinder(nvinfer1::ILoggerFinder * finder)
{
  logger_finder.setLoggerFinder(finder);
}

extern "C" nvinfer1::IPluginCreatorInterface * const * getCreators(std::int32_t & num_creators)
{
  num_creators = 11;
  static nvinfer1::plugin::ArgsortPluginCreator argsort_plugin_creator{};
  static nvinfer1::plugin::QuickCumsumCudaPluginCreator quick_cumsum_cuda_plugin_creator{};
  static nvinfer1::plugin::GetIndicesPairsImplicitGemmPluginCreator
    get_indices_pairs_implicit_gemm_plugin_creator{};
  static nvinfer1::plugin::GetIndicesPairsPluginCreator get_indices_pairs_plugin_creator{};
  static nvinfer1::plugin::ImplicitGemmPluginCreator implicit_gemm_plugin_creator{};
  static nvinfer1::plugin::IndiceConvPluginCreator
    indice_conv_plugin_creator{};  // cSpell:ignore Indice
  static autoware::tensorrt_plugins::MultiScaleDeformableAttentionPluginCreator
    multi_scale_deformable_attention_plugin_creator{};
  static autoware::tensorrt_plugins::RotatePluginCreator rotate_plugin_creator{};
  static nvinfer1::plugin::SegmentCSRPluginCreator segment_csr_plugin_creator{};
  static autoware::tensorrt_plugins::SelectAndPadPluginCreator select_and_pad_plugin_creator{};
  static nvinfer1::plugin::UniquePluginCreator unique_plugin_creator{};

  static nvinfer1::IPluginCreatorInterface * const plugin_creator_list[] = {
    &argsort_plugin_creator,
    &quick_cumsum_cuda_plugin_creator,
    &get_indices_pairs_implicit_gemm_plugin_creator,
    &get_indices_pairs_plugin_creator,
    &implicit_gemm_plugin_creator,
    &indice_conv_plugin_creator,
    &multi_scale_deformable_attention_plugin_creator,
    &rotate_plugin_creator,
    &segment_csr_plugin_creator,
    &select_and_pad_plugin_creator,
    &unique_plugin_creator};
  return plugin_creator_list;
}
