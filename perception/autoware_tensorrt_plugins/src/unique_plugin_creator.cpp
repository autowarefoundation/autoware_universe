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

#include "autoware/tensorrt_plugins/unique_plugin_creator.hpp"

#include "autoware/tensorrt_plugins/unique_plugin.hpp"

#include <NvInferRuntimePlugin.h>

#include <string>

namespace nvinfer1::plugin
{

REGISTER_TENSORRT_PLUGIN(UniquePluginCreator);

UniquePluginCreator::UniquePluginCreator()
{
  plugin_attributes_.clear();
  fc_.nbFields = plugin_attributes_.size();
  fc_.fields = plugin_attributes_.data();
}

nvinfer1::PluginFieldCollection const * UniquePluginCreator::getFieldNames() noexcept
{
  // This is only used in the build phase.
  return &fc_;
}

IPluginV3 * UniquePluginCreator::createPlugin(
  char const * name, [[maybe_unused]] PluginFieldCollection const * fc,
  [[maybe_unused]] TensorRTPhase phase) noexcept
{
  return new (std::nothrow) UniquePlugin(std::string(name));
}

}  // namespace nvinfer1::plugin
