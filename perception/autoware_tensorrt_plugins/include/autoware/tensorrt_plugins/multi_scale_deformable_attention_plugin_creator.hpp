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

#ifndef AUTOWARE__TENSORRT_PLUGINS__MULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_CREATOR_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__MULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_CREATOR_HPP_

#include "autoware/tensorrt_plugins/multi_scale_deformable_attention_plugin.hpp"

#include <NvInferRuntime.h>

#include <vector>

namespace autoware::tensorrt_plugins
{

class MultiScaleDeformableAttentionPluginCreator : public nvinfer1::IPluginCreatorV3One
{
public:
  MultiScaleDeformableAttentionPluginCreator();

  ~MultiScaleDeformableAttentionPluginCreator() override = default;

  char const * getPluginNamespace() const noexcept override
  {
    return kMULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_NAMESPACE;
  }

  char const * getPluginName() const noexcept override
  {
    return kMULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_NAME;
  }

  char const * getPluginVersion() const noexcept override
  {
    return kMULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_VERSION;
  }

  nvinfer1::PluginFieldCollection const * getFieldNames() noexcept override;

  nvinfer1::IPluginV3 * createPlugin(
    char const * name, nvinfer1::PluginFieldCollection const * fc,
    nvinfer1::TensorRTPhase phase) noexcept override;

private:
  nvinfer1::PluginFieldCollection fc_;
  std::vector<nvinfer1::PluginField> plugin_attributes_;
};

}  // namespace autoware::tensorrt_plugins

#endif  // AUTOWARE__TENSORRT_PLUGINS__MULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_CREATOR_HPP_
