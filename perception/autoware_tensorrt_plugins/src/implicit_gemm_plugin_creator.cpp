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

#include "autoware/tensorrt_plugins/implicit_gemm_plugin_creator.hpp"

#include "autoware/tensorrt_plugins/implicit_gemm_plugin.hpp"
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

REGISTER_TENSORRT_PLUGIN(ImplicitGemmPluginCreator);

ImplicitGemmPluginCreator::ImplicitGemmPluginCreator()
{
  plugin_attributes_.clear();
  plugin_attributes_.emplace_back("act_alpha", nullptr, PluginFieldType::kFLOAT32, 1);
  plugin_attributes_.emplace_back("act_beta", nullptr, PluginFieldType::kFLOAT32, 1);
  plugin_attributes_.emplace_back(
    "is_subm", nullptr, PluginFieldType::kINT32, 1);  // cSpell:ignore subm
  plugin_attributes_.emplace_back("is_train", nullptr, PluginFieldType::kINT32, 1);
  plugin_attributes_.emplace_back("output_add_scale", nullptr, PluginFieldType::kFLOAT32, 1);
  plugin_attributes_.emplace_back("output_scale", nullptr, PluginFieldType::kFLOAT32, 1);
  plugin_attributes_.emplace_back("act_type", nullptr, PluginFieldType::kINT32, 1);
  // INT8 (precision==1) attributes. Absent on FP nodes (default precision=0).
  plugin_attributes_.emplace_back("precision", nullptr, PluginFieldType::kINT32, 1);
  plugin_attributes_.emplace_back("input_scale", nullptr, PluginFieldType::kFLOAT32, 1);

  fc_.nbFields = plugin_attributes_.size();
  fc_.fields = plugin_attributes_.data();
}

nvinfer1::PluginFieldCollection const * ImplicitGemmPluginCreator::getFieldNames() noexcept
{
  // This is only used in the build phase.
  return &fc_;
}

IPluginV3 * ImplicitGemmPluginCreator::createPlugin(
  char const * name, PluginFieldCollection const * fc, TensorRTPhase phase) noexcept
{
  auto parse_fields = [](
                        nvinfer1::PluginField const * fields, std::int32_t num_fields,
                        ImplicitGemmParameters & parameters) {
    // Accept 6 (legacy without act_type) or 7 (with act_type).
    PLUGIN_VALIDATE(num_fields == 6 || num_fields == 7);
    for (std::int32_t i{0}; i < num_fields; ++i) {
      const std::string attr_name = fields[i].name;
      const nvinfer1::PluginFieldType type = fields[i].type;

      if (attr_name == "act_alpha") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kFLOAT32);
        parameters.act_alpha = static_cast<float const *>(fields[i].data)[0];
      } else if (attr_name == "act_beta") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kFLOAT32);
        parameters.act_beta = static_cast<float const *>(fields[i].data)[0];
      } else if (attr_name == "is_subm") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kINT32);
        parameters.is_subm = static_cast<std::int32_t const *>(fields[i].data)[0];
      } else if (attr_name == "is_train") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kINT32);
        parameters.is_train = static_cast<std::int32_t const *>(fields[i].data)[0];
      } else if (attr_name == "output_add_scale") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kFLOAT32);
        parameters.output_add_scale = static_cast<float const *>(fields[i].data)[0];
      } else if (attr_name == "output_scale") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kFLOAT32);
        parameters.output_scale = static_cast<float const *>(fields[i].data)[0];
      } else if (attr_name == "act_type") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kINT32);
        parameters.act_type = static_cast<std::int32_t const *>(fields[i].data)[0];
        PLUGIN_VALIDATE(parameters.act_type >= 0 && parameters.act_type <= 3);
      } else if (attr_name == "precision") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kINT32);
        parameters.precision = static_cast<std::int32_t const *>(fields[i].data)[0];
        PLUGIN_VALIDATE(
          parameters.precision == kIMPLICIT_GEMM_PRECISION_FP ||
          parameters.precision == kIMPLICIT_GEMM_PRECISION_INT8);
      } else if (attr_name == "input_scale") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kFLOAT32);
        parameters.input_scale = static_cast<float const *>(fields[i].data)[0];
      }
    }
  };

  // The build phase and the deserialization phase are handled differently.
  if (phase == TensorRTPhase::kBUILD) {
    // The attributes from the ONNX node will be parsed and passed via fc.
    try {
      nvinfer1::PluginField const * fields{fc->fields};
      std::int32_t num_fields{fc->nbFields};

      ImplicitGemmParameters parameters{};
      parse_fields(fields, num_fields, parameters);

      // Log the attributes parsed from ONNX node.
      std::stringstream ss;
      ss << name << " plugin Attributes:"
         << " act_alpha=" << parameters.act_alpha << " act_beta=" << parameters.act_beta
         << " is_subm=" << parameters.is_subm << " is_train=" << parameters.is_train
         << " output_add_scale=" << parameters.output_add_scale
         << " output_scale=" << parameters.output_scale << " act_type=" << parameters.act_type
         << " precision=" << parameters.precision << " input_scale=" << parameters.input_scale;
      logDebug(ss.str().c_str());

      ImplicitGemmPlugin * const plugin{new ImplicitGemmPlugin{std::string(name), parameters}};
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
      // An engine built with another layout of the parameters (an older plugin) must not run:
      // reading it would reinterpret its fields. PLUGIN_VALIDATE only logs, and TensorRT 10.8
      // crashes on a null plugin here, so stop with the reason instead.
      bool const same_layout =
        num_fields == 1 && fields[0].name != nullptr && strcmp(fields[0].name, "parameters") == 0 &&
        fields[0].type == nvinfer1::PluginFieldType::kUNKNOWN &&
        fields[0].length == static_cast<std::int32_t>(sizeof(ImplicitGemmParameters)) &&
        fields[0].data != nullptr;
      if (!same_layout) {
        std::stringstream ss;
        ss << "ImplicitGemm " << name
           << ": the engine was serialized by a different ImplicitGemm plugin (parameters of "
           << (num_fields == 1 ? fields[0].length : -1) << " bytes, this plugin expects "
           << sizeof(ImplicitGemmParameters) << "). Rebuild the TensorRT engine.";
        PLUGIN_ASSERT_MSG(same_layout, ss.str().c_str());
      }
      auto params = *(static_cast<ImplicitGemmParameters const *>(fields[0].data));
      ImplicitGemmPlugin * const plugin{new ImplicitGemmPlugin{std::string(name), params}};
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
