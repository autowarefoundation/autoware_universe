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

#include <algorithm>
#include <cmath>
#include <cstddef>
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
  // Quantized (INT8) nodes only: the scale of the QuantizeLinear in front and the type the
  // epilogue writes. The ONNX parser forwards only the attributes declared here.
  plugin_attributes_.emplace_back("input_scale", nullptr, PluginFieldType::kFLOAT32, 1);
  plugin_attributes_.emplace_back("output_type", nullptr, PluginFieldType::kCHAR, 0);

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
  // Returns why the attributes are refused, or an empty string.
  auto parse_fields = [](
                        nvinfer1::PluginField const * fields, std::int32_t num_fields,
                        ImplicitGemmParameters & parameters) -> std::string {
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
      } else if (attr_name == "input_scale") {
        PLUGIN_VALIDATE(type == nvinfer1::PluginFieldType::kFLOAT32);
        parameters.input_scale = static_cast<float const *>(fields[i].data)[0];
        parameters.has_input_scale = 1;
      } else if (attr_name == "output_type") {
        if (type != nvinfer1::PluginFieldType::kCHAR || fields[i].data == nullptr) {
          return "output_type must be a string";
        }
        // The ONNX parser counts the terminating NUL in the length.
        auto const * chars = static_cast<char const *>(fields[i].data);
        std::string const value(
          chars, strnlen(chars, static_cast<std::size_t>(std::max(fields[i].length, 0))));
        if (value == "f16") {
          parameters.output_type = kIMPLICIT_GEMM_OUTPUT_TYPE_F16;
        } else if (value == "bf16") {
          return "output_type \"bf16\" is not supported: this plugin writes FP16 only";
        } else if (value == "s8" || value == "e4m3") {
          return "output_type \"" + value +
                 "\" asks for an 8-bit output, which this plugin does not write";
        } else {
          return "output_type \"" + value +
                 "\" is not a known type (\"f16\", \"bf16\", \"s8\", \"e4m3\")";
        }
      }
    }
    // The exporter writes output_scale = 1.0 on every node; the output is never quantized here.
    if (parameters.output_scale != 1.0F) {
      return "output_scale " + std::to_string(parameters.output_scale) +
             " is not supported: the output is not quantized, so only 1.0 is accepted";
    }
    if (
      parameters.has_input_scale != 0 &&
      !(std::isfinite(parameters.input_scale) && parameters.input_scale > 0.0F)) {
      return "input_scale must be positive and finite, got " +
             std::to_string(parameters.input_scale);
    }
    return {};
  };

  // The build phase and the deserialization phase are handled differently.
  if (phase == TensorRTPhase::kBUILD) {
    // The attributes from the ONNX node will be parsed and passed via fc.
    try {
      nvinfer1::PluginField const * fields{fc->fields};
      std::int32_t num_fields{fc->nbFields};

      ImplicitGemmParameters parameters{};
      std::string const refusal = parse_fields(fields, num_fields, parameters);
      if (!refusal.empty()) {
        std::string const message = std::string("ImplicitGemm ") + name + ": " + refusal;
        logError(message.c_str());
        return nullptr;
      }

      // Log the attributes parsed from ONNX node.
      std::stringstream ss;
      ss << name << " plugin Attributes:"
         << " act_alpha=" << parameters.act_alpha << " act_beta=" << parameters.act_beta
         << " is_subm=" << parameters.is_subm << " is_train=" << parameters.is_train
         << " output_add_scale=" << parameters.output_add_scale
         << " output_scale=" << parameters.output_scale << " act_type=" << parameters.act_type
         << " input_scale=" << (parameters.has_input_scale != 0 ? parameters.input_scale : 0.0F)
         << " output_type=" << parameters.output_type;
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
