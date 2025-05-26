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

#include "autoware/tensorrt_plugins/segment_csr_plugin_creator.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"
#include "autoware/tensorrt_plugins/segment_csr_plugin.hpp"

#include <NvInferRuntimePlugin.h>

#include <exception>
#include <string>

namespace nvinfer1::plugin
{

REGISTER_TENSORRT_PLUGIN(SegmentCSRPluginCreator);

SegmentCSRPluginCreator::SegmentCSRPluginCreator()
{
  plugin_attributes_.clear();
  plugin_attributes_.emplace_back("reduce", nullptr, PluginFieldType::kCHAR, 1);

  fc_.nbFields = plugin_attributes_.size();
  fc_.fields = plugin_attributes_.data();
}

nvinfer1::PluginFieldCollection const * SegmentCSRPluginCreator::getFieldNames() noexcept
{
  // This is only used in the build phase.
  return &fc_;
}

IPluginV3 * SegmentCSRPluginCreator::createPlugin(
  char const * name, PluginFieldCollection const * fc,
  [[maybe_unused]] TensorRTPhase phase) noexcept
{
  try {
    PLUGIN_VALIDATE(fc != nullptr);
    PLUGIN_VALIDATE(fc->nbFields == 1);
    PLUGIN_VALIDATE(std::string(fc->fields[0].name) == "reduce");
    std::string reduce(static_cast<char const *>(fc->fields[0].data), fc->fields[0].length);

    std::size_t null_pos = reduce.find('\0');
    if (null_pos != std::string::npos) {
      reduce.resize(null_pos);
    }

    SegmentCSRPlugin * const plugin(new SegmentCSRPlugin(std::string(name), reduce));
    return plugin;
  } catch (std::exception & e) {
    caughtError(e);
  }
  return nullptr;
}

}  // namespace nvinfer1::plugin
