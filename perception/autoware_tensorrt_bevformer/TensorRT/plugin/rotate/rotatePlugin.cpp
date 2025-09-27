// Copyright 2025 The Autoware Contributors
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
//
/*
 * This file includes portions of code directly from the BEVFormer TensorRT implementation
 * by Derry Lin, available at:
 *   https://github.com/DerryHub/BEVFormer_tensorrt
 *
 * The included code is used under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * The Autoware Contributors have reused this code as-is in 2025, with no modifications.
 * Original creation by Derry Lin on 2022/10/22.
 */

#include "rotatePlugin.h"

#include "checkMacrosPlugin.h"
#include "rotateKernel.h"
#include "serialize.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using nvinfer1::DataType;
using nvinfer1::Dims;
using nvinfer1::DimsExprs;
using nvinfer1::IPluginV2DynamicExt;
using nvinfer1::PluginField;
using nvinfer1::PluginFieldCollection;
using trt_plugin::RotatePlugin;
using trt_plugin::RotatePluginCreator;
using trt_plugin::RotatePluginCreator2;

namespace
{
constexpr char const * R_PLUGIN_VERSION{"1"};
constexpr char const * R_PLUGIN_NAME{"RotateTRT"};
constexpr char const * R_PLUGIN_NAME2{"RotateTRT2"};
}  // namespace

PluginFieldCollection RotatePluginCreator::mFC{};
std::vector<PluginField> RotatePluginCreator::mPluginAttributes;

PluginFieldCollection RotatePluginCreator2::mFC{};
std::vector<PluginField> RotatePluginCreator2::mPluginAttributes;

RotatePlugin::RotatePlugin(const int mode, bool use_h2) : use_h2(use_h2)
{
  switch (mode) {
    case 0:
      mMode = RotateInterpolation::Bilinear;
      break;
    case 1:
      mMode = RotateInterpolation::Nearest;
      break;
    default:
      break;
  }
}

RotatePlugin::RotatePlugin(const void * serialData, size_t serialLength, bool use_h2)
: use_h2(use_h2)
{
  deserialize_value(&serialData, &serialLength, &mMode);
}

RotatePlugin::~RotatePlugin()
{
  terminate();
}

int32_t RotatePlugin::getNbOutputs() const noexcept
{
  return 1;
}

DimsExprs RotatePlugin::getOutputDimensions(
  int32_t outputIndex, const nvinfer1::DimsExprs * inputs, int32_t nbInputs,
  nvinfer1::IExprBuilder & exprBuilder) noexcept
{
  DimsExprs outputDim;
  outputDim.nbDims = 3;
  outputDim.d[0] = inputs[0].d[0];
  outputDim.d[1] = inputs[0].d[1];
  outputDim.d[2] = inputs[0].d[2];
  return outputDim;
}

int32_t RotatePlugin::initialize() noexcept
{
  return 0;
}

void RotatePlugin::terminate() noexcept
{
}

size_t RotatePlugin::getWorkspaceSize(
  const nvinfer1::PluginTensorDesc * inputs, int32_t nbInputs,
  const nvinfer1::PluginTensorDesc * outputs, int32_t nbOutputs) const noexcept
{
  return 0;
}

int32_t RotatePlugin::enqueue(
  const nvinfer1::PluginTensorDesc * inputDesc, const nvinfer1::PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  Dims input_dims = inputDesc[0].dims;
  auto data_type = inputDesc[0].type;
  auto data_type_angle = inputDesc[1].type;
  float scale_i = inputDesc[0].scale, scale_o = outputDesc[0].scale;

  switch (data_type) {
    case DataType::kFLOAT: {
      rotate<float>(
        reinterpret_cast<float *>(outputs[0]), reinterpret_cast<const float *>(inputs[0]),
        reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<const float *>(inputs[2]),
        &(input_dims.d[0]), mMode, stream);
      break;
    }

    case DataType::kHALF: {
      if (use_h2) {
        rotate_h2(
          reinterpret_cast<__half2 *>(outputs[0]), reinterpret_cast<const __half2 *>(inputs[0]),
          reinterpret_cast<const __half *>(inputs[1]), reinterpret_cast<const __half *>(inputs[2]),
          &(input_dims.d[0]), mMode, stream);
      } else {
        rotate<__half>(
          reinterpret_cast<__half *>(outputs[0]), reinterpret_cast<const __half *>(inputs[0]),
          reinterpret_cast<const __half *>(inputs[1]), reinterpret_cast<const __half *>(inputs[2]),
          &(input_dims.d[0]), mMode, stream);
      }
      break;
    }

    case DataType::kINT8: {
      if (data_type_angle == DataType::kFLOAT) {
        rotate_int8(
          reinterpret_cast<int8_4 *>(outputs[0]), scale_o,
          reinterpret_cast<const int8_4 *>(inputs[0]), scale_i,
          reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<const float *>(inputs[2]),
          &(input_dims.d[0]), mMode, stream);
      } else {
        rotate_int8(
          reinterpret_cast<int8_4 *>(outputs[0]), scale_o,
          reinterpret_cast<const int8_4 *>(inputs[0]), scale_i,
          reinterpret_cast<const __half *>(inputs[1]), reinterpret_cast<const __half *>(inputs[2]),
          &(input_dims.d[0]), mMode, stream);
      }
      break;
    }

    default:
      printf("[Rotate] Unsupported data type: %d\n", static_cast<int>(data_type));
      return 1;
  }
  return 0;
}

size_t RotatePlugin::getSerializationSize() const noexcept
{
  return serialized_size(mMode);
}

void RotatePlugin::serialize(void * buffer) const noexcept
{
  serialize_value(&buffer, mMode);
}

bool RotatePlugin::supportsFormatCombination(
  int32_t pos, const nvinfer1::PluginTensorDesc * inOut, int32_t nbInputs,
  int32_t nbOutputs) noexcept
{
  if (pos == 0) {
    if (use_h2) {
      return (inOut[pos].type == nvinfer1::DataType::kFLOAT &&
              inOut[pos].format == nvinfer1::TensorFormat::kLINEAR) ||
             (inOut[pos].type == nvinfer1::DataType::kHALF &&
              inOut[pos].format == nvinfer1::TensorFormat::kCHW2) ||
             (inOut[pos].type == nvinfer1::DataType::kINT8 &&
              inOut[pos].format == nvinfer1::TensorFormat::kCHW4);
    }
    return ((inOut[pos].type == nvinfer1::DataType::kFLOAT ||
             inOut[pos].type == nvinfer1::DataType::kHALF) &&
            inOut[pos].format == nvinfer1::TensorFormat::kLINEAR) ||
           (inOut[pos].type == nvinfer1::DataType::kINT8 &&
            inOut[pos].format == nvinfer1::TensorFormat::kCHW4);
  } else if (pos == nbInputs) {
    return inOut[pos].type == inOut[0].type && inOut[pos].format == inOut[0].format;
  } else if (inOut[0].type == nvinfer1::DataType::kINT8) {
    return (inOut[pos].type == nvinfer1::DataType::kFLOAT ||
            inOut[pos].type == nvinfer1::DataType::kHALF) &&
           inOut[pos].format == nvinfer1::TensorFormat::kLINEAR && inOut[pos].type == inOut[1].type;
  } else {
    return inOut[pos].type == inOut[0].type && inOut[pos].format == nvinfer1::TensorFormat::kLINEAR;
  }
}

char const * RotatePlugin::getPluginType() const noexcept
{
  return use_h2 ? R_PLUGIN_NAME2 : R_PLUGIN_NAME;
}

char const * RotatePlugin::getPluginVersion() const noexcept
{
  return R_PLUGIN_VERSION;
}

void RotatePlugin::destroy() noexcept
{
  delete this;
}

IPluginV2DynamicExt * RotatePlugin::clone() const noexcept
{
  try {
    int mode, paddingMode;
    switch (mMode) {
      case RotateInterpolation::Bilinear:
        mode = 0;
        break;
      case RotateInterpolation::Nearest:
        mode = 1;
        break;
      default:
        mode = 0;  // Default to bilinear
        break;
    }
    auto * plugin = new RotatePlugin{mode, use_h2};
    plugin->setPluginNamespace(mPluginNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

void RotatePlugin::setPluginNamespace(const char * pluginNamespace) noexcept
{
  mPluginNamespace = pluginNamespace;
}

char const * RotatePlugin::getPluginNamespace() const noexcept
{
  return mPluginNamespace.c_str();
}
DataType RotatePlugin::getOutputDataType(
  int32_t index, const nvinfer1::DataType * inputTypes, int32_t nbInputs) const noexcept
{
  // cppcheck-suppress unknownMacro
  PLUGIN_ASSERT(inputTypes && nbInputs > 0 && index == 0)
  return inputTypes[0];
}

void RotatePlugin::attachToContext(
  cudnnContext * cudnn, cublasContext * cublas, nvinfer1::IGpuAllocator * allocator) noexcept
{
}

void RotatePlugin::detachFromContext() noexcept
{
}
void RotatePlugin::configurePlugin(
  const nvinfer1::DynamicPluginTensorDesc * in, int32_t nbInputs,
  const nvinfer1::DynamicPluginTensorDesc * out,
  int32_t nbOutputs) noexcept  // cppcheck-suppress unknownMacro
  {PLUGIN_ASSERT(nbInputs == 3)}

RotatePluginCreator::RotatePluginCreator()
{
  mPluginAttributes.clear();
  mPluginAttributes.emplace_back(nvinfer1::PluginField("interpolation"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("center"));

  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

char const * RotatePluginCreator::getPluginName() const noexcept
{
  return R_PLUGIN_NAME;
}

char const * RotatePluginCreator::getPluginVersion() const noexcept
{
  return R_PLUGIN_VERSION;
}

PluginFieldCollection const * RotatePluginCreator::getFieldNames() noexcept
{
  return &mFC;
}

IPluginV2DynamicExt * RotatePluginCreator::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  try {
    int mode = 0;
    PluginField const * fields = fc->fields;
    for (int i = 0; i < fc->nbFields; i++) {
      char const * attrName = fields[i].name;
      if (!strcmp(attrName, "interpolation")) {
        PLUGIN_VALIDATE(fields[i].type == nvinfer1::PluginFieldType::kINT32);
        mode = *(static_cast<int const *>(fields[i].data));
      }
    }

    auto * plugin = new RotatePlugin(mode, false);
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

IPluginV2DynamicExt * RotatePluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  try {
    auto * plugin = new RotatePlugin{serialData, serialLength, false};
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

RotatePluginCreator2::RotatePluginCreator2()
{
  mPluginAttributes.clear();
  mPluginAttributes.emplace_back(nvinfer1::PluginField("interpolation"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("center"));

  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

char const * RotatePluginCreator2::getPluginName() const noexcept
{
  return R_PLUGIN_NAME2;
}

char const * RotatePluginCreator2::getPluginVersion() const noexcept
{
  return R_PLUGIN_VERSION;
}

PluginFieldCollection const * RotatePluginCreator2::getFieldNames() noexcept
{
  return &mFC;
}

IPluginV2DynamicExt * RotatePluginCreator2::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  try {
    int mode = 0;
    PluginField const * fields = fc->fields;
    for (int i = 0; i < fc->nbFields; i++) {
      char const * attrName = fields[i].name;
      if (!strcmp(attrName, "interpolation")) {
        PLUGIN_VALIDATE(fields[i].type == nvinfer1::PluginFieldType::kINT32);
        mode = *(static_cast<int const *>(fields[i].data));
      }
    }

    auto * plugin = new RotatePlugin(mode, true);
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

IPluginV2DynamicExt * RotatePluginCreator2::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  try {
    auto * plugin = new RotatePlugin{serialData, serialLength, true};
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

REGISTER_TENSORRT_PLUGIN(RotatePluginCreator);
REGISTER_TENSORRT_PLUGIN(RotatePluginCreator2);
