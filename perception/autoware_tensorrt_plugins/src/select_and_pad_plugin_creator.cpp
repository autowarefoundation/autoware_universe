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

#include "autoware/tensorrt_plugins/select_and_pad_plugin_creator.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"
#include "autoware/tensorrt_plugins/select_and_pad_plugin.hpp"

#include <NvInferRuntimePlugin.h>

#include <cstring>
#include <string>

namespace autoware::tensorrt_plugins
{

REGISTER_TENSORRT_PLUGIN(SelectAndPadPluginCreator);

SelectAndPadPluginCreator::SelectAndPadPluginCreator()
{
  plugin_attributes_.clear();
  plugin_attributes_.emplace_back(
    nvinfer1::PluginField{"P", nullptr, nvinfer1::PluginFieldType::kINT32, 1});
  fc_.nbFields = plugin_attributes_.size();
  fc_.fields = plugin_attributes_.data();
}

nvinfer1::PluginFieldCollection const * SelectAndPadPluginCreator::getFieldNames() noexcept
{
  return &fc_;
}

nvinfer1::IPluginV3 * SelectAndPadPluginCreator::createPlugin(
  char const * name, nvinfer1::PluginFieldCollection const * fc,
  [[maybe_unused]] nvinfer1::TensorRTPhase phase) noexcept
{
  int P = 200;  // Default value

  // Parse plugin fields
  const nvinfer1::PluginField * fields = fc->fields;
  for (int i = 0; i < fc->nbFields; i++) {
    if (strcmp(fields[i].name, "P") == 0) {
      P = reinterpret_cast<const int *>(fields[i].data)[0];
    }
  }

  return new (std::nothrow) SelectAndPadPlugin(std::string(name), P);
}

}  // namespace autoware::tensorrt_plugins
