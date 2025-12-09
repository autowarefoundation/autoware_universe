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

#include "autoware/tensorrt_plugins/indice_conv_plugin_creator.hpp"

#include "autoware/tensorrt_plugins/indice_conv_plugin.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntimePlugin.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>

namespace nvinfer1::plugin
{

// cSpell:ignore Indice
REGISTER_TENSORRT_PLUGIN(IndiceConvPluginCreator);

IndiceConvPluginCreator::IndiceConvPluginCreator()
{
  plugin_attributes_.clear();
  plugin_attributes_.emplace_back(
    "is_subm", nullptr, PluginFieldType::kINT32, 1);  // cSpell:ignore subm
  fc_.nbFields = plugin_attributes_.size();
  fc_.fields = plugin_attributes_.data();
}

nvinfer1::PluginFieldCollection const * IndiceConvPluginCreator::getFieldNames() noexcept
{
  // This is only used in the build phase.
  return &fc_;
}

IPluginV3 * IndiceConvPluginCreator::createPlugin(
  char const * name, PluginFieldCollection const * fc, TensorRTPhase phase) noexcept
{
  // The build phase and the deserialization phase are handled differently.
  if (phase == TensorRTPhase::kBUILD || phase == TensorRTPhase::kRUNTIME) {
    // The attributes from the ONNX node will be parsed and passed via fc.
    try {
      nvinfer1::PluginField const * fields{fc->fields};
      std::int32_t num_fields{fc->nbFields};

      PLUGIN_VALIDATE(num_fields == 1);

      IndiceConvParameters parameters;
      const std::string attr_name = fields[0].name;
      const nvinfer1::PluginFieldType type = fields[0].type;

      if (attr_name == "is_subm") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kINT32);
        parameters.is_subm = static_cast<std::int32_t const *>(fields[0].data)[0];
      }

      // Log the attributes parsed from ONNX node.
      std::stringstream ss;
      ss << name << " plugin Attributes:";
      logDebug(ss.str().c_str());

      ss.str("");
      ss << "is_subm: " << parameters.is_subm;
      logDebug(ss.str().c_str());

      IndiceConvPlugin * const plugin{new IndiceConvPlugin{std::string(name), parameters}};
      return plugin;
    } catch (std::exception const & e) {
      caughtError(e);
    }
    return nullptr;
  } else if (phase == TensorRTPhase::kRUNTIME) {
    // The attributes from the serialized plugin will be passed via fc.
    try {
      nvinfer1::PluginField const * fields{fc->fields};
      std::int32_t num_fields{fc->nbFields};
      PLUGIN_VALIDATE(num_fields == 1);

      char const * attr_name = fields[0].name;
      PLUGIN_VALIDATE(!strcmp(attr_name, "parameters"));
      PLUGIN_VALIDATE(fields[0].type == nvinfer1::PluginFieldType::kUNKNOWN);
      PLUGIN_VALIDATE(fields[0].length == sizeof(IndiceConvParameters));
      IndiceConvParameters params{*(static_cast<IndiceConvParameters const *>(fields[0].data))};

      IndiceConvPlugin * const plugin{new IndiceConvPlugin{std::string(name), params}};
      return plugin;
    } catch (std::exception const & e) {
      caughtError(e);
    }
    return nullptr;
  } else {
    return nullptr;
  }
}

}  // namespace nvinfer1::plugin
