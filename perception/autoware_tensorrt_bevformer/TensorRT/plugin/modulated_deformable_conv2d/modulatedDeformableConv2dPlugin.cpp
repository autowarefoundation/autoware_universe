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

#include "modulatedDeformableConv2dPlugin.h"

#include "checkMacrosPlugin.h"
#include "modulatedDeformableConv2dKernel.h"
#include "serialize.h"

#include <cuda_fp16.h>

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using nvinfer1::DataType;
using nvinfer1::Dims;
using nvinfer1::DimsExprs;
using nvinfer1::IPluginV2DynamicExt;
using nvinfer1::PluginField;
using nvinfer1::PluginFieldCollection;
using trt_plugin::ModulatedDeformableConv2dPlugin;
using trt_plugin::ModulatedDeformableConv2dPluginCreator;
using trt_plugin::ModulatedDeformableConv2dPluginCreator2;

namespace
{
constexpr char const * MDC_PLUGIN_VERSION{"1"};
constexpr char const * MDC_PLUGIN_NAME{"ModulatedDeformableConv2dTRT"};
constexpr char const * MDC_PLUGIN_NAME2{"ModulatedDeformableConv2dTRT2"};
}  // namespace

PluginFieldCollection ModulatedDeformableConv2dPluginCreator::mFC{};
std::vector<PluginField> ModulatedDeformableConv2dPluginCreator::mPluginAttributes;

PluginFieldCollection ModulatedDeformableConv2dPluginCreator2::mFC{};
std::vector<PluginField> ModulatedDeformableConv2dPluginCreator2::mPluginAttributes;

ModulatedDeformableConv2dPlugin::ModulatedDeformableConv2dPlugin(
  nvinfer1::Dims stride, nvinfer1::Dims padding, nvinfer1::Dims dilation, int deformableGroup,
  int group, bool use_h2)
: mStride(stride),
  mPadding(padding),
  mDilation(dilation),
  mDeformableGroup(deformableGroup),
  mGroup(group),
  use_h2(use_h2)
{
}

ModulatedDeformableConv2dPlugin::ModulatedDeformableConv2dPlugin(
  const void * data, size_t length, bool use_h2)
: use_h2(use_h2)
{
  deserialize_value(&data, &length, &mStride);
  deserialize_value(&data, &length, &mPadding);
  deserialize_value(&data, &length, &mDilation);
  deserialize_value(&data, &length, &mDeformableGroup);
  deserialize_value(&data, &length, &mGroup);
}

ModulatedDeformableConv2dPlugin::~ModulatedDeformableConv2dPlugin()
{
  terminate();
}

int32_t ModulatedDeformableConv2dPlugin::getNbOutputs() const noexcept
{
  return 1;
}

DimsExprs ModulatedDeformableConv2dPlugin::getOutputDimensions(
  int32_t outputIndex, const nvinfer1::DimsExprs * inputs, int32_t nbInputs,
  nvinfer1::IExprBuilder & exprBuilder) noexcept
{
  DimsExprs outputDim;
  outputDim.nbDims = 4;
  outputDim.d[0] = inputs[0].d[0];
  outputDim.d[1] = inputs[3].d[0];
  outputDim.d[2] = inputs[1].d[2];
  outputDim.d[3] = inputs[1].d[3];
  return outputDim;
}

int32_t ModulatedDeformableConv2dPlugin::initialize() noexcept
{
  return 0;
}

void ModulatedDeformableConv2dPlugin::terminate() noexcept
{
}

size_t ModulatedDeformableConv2dPlugin::getWorkspaceSize(
  const nvinfer1::PluginTensorDesc * inputs, int32_t nbInputs,
  const nvinfer1::PluginTensorDesc * outputs, int32_t nbOutputs) const noexcept
{
  size_t sizeof_dtype;
  switch (outputs[0].type) {
    case nvinfer1::DataType::kFLOAT:
      sizeof_dtype = 4;
      break;
    case nvinfer1::DataType::kHALF:
      sizeof_dtype = 2;
      break;
    case nvinfer1::DataType::kINT8:
      sizeof_dtype = 1;
      break;
    default:
      sizeof_dtype = 4;  // Default to float size
      break;
  }

  size_t nInputPlane = inputs[0].dims.d[1];

  size_t nOutputPlane = outputs[0].dims.d[1];
  size_t outputHeight = outputs[0].dims.d[2];
  size_t outputWidth = outputs[0].dims.d[3];

  size_t kW = inputs[3].dims.d[2];
  size_t kH = inputs[3].dims.d[3];

  size_t col_size;
  if (sizeof_dtype == 2) {
    col_size = (nInputPlane + 1) / 2 * 2 * kW * kH * outputHeight * outputWidth * sizeof_dtype;
  } else if (sizeof_dtype == 1) {
    col_size = (nInputPlane + 3) / 4 * 4 * kW * kH * ((outputHeight * outputWidth + 3) / 4 * 4) *
               sizeof_dtype;
    col_size += nOutputPlane / mGroup * ((outputHeight * outputWidth + 3) / 4 * 4) * 4;
  } else {
    col_size = nInputPlane * kW * kH * outputHeight * outputWidth * sizeof_dtype;
  }

  col_size = size_t((col_size + 16 - 1) / 16) * 16;
  return col_size;
}

int32_t ModulatedDeformableConv2dPlugin::enqueue(
  const nvinfer1::PluginTensorDesc * inputDesc, const nvinfer1::PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workSpace,
  cudaStream_t stream) noexcept
{
  int batch = inputDesc[0].dims.d[0];
  int channels = inputDesc[0].dims.d[1];
  int height = inputDesc[0].dims.d[2];
  int width = inputDesc[0].dims.d[3];
  int channels_out = outputDesc[0].dims.d[1];
  int output_height = outputDesc[0].dims.d[2];
  int output_width = outputDesc[0].dims.d[3];
  int kernel_h = inputDesc[3].dims.d[2];
  int kernel_w = inputDesc[3].dims.d[3];

  const float scale_o = outputDesc[0].scale, scale_i = inputDesc[0].scale,
              scale_off = inputDesc[1].scale, scale_mask = inputDesc[2].scale,
              scale_w = inputDesc[3].scale;

  const void * x = inputs[0];
  const void * offset = inputs[1];
  const void * mask = inputs[2];
  const void * weight = inputs[3];

  const void * bias = mWithBias ? inputs[4] : nullptr;
  void * output = outputs[0];
  int im2col_step = std::min(batch, 32);

  auto data_type = inputDesc[0].type;
  nvinfer1::DataType data_type_bias = nvinfer1::DataType::kFLOAT;
  if (mWithBias) {
    data_type_bias = inputDesc[4].type;
  }

  switch (data_type) {
    case nvinfer1::DataType::kFLOAT: {
      ModulatedDeformConvForwardCUDAKernel<float>(
        reinterpret_cast<const float *>(x), reinterpret_cast<const float *>(weight),
        reinterpret_cast<const float *>(bias), reinterpret_cast<const float *>(offset),
        reinterpret_cast<const float *>(mask), reinterpret_cast<float *>(output), workSpace, batch,
        channels, height, width, channels_out, kernel_w, kernel_h, mStride.d[0], mStride.d[1],
        mPadding.d[0], mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup, mDeformableGroup,
        im2col_step, m_cublas_handle, stream);
      break;
    }

    case nvinfer1::DataType::kHALF: {
      if (use_h2) {
        ModulatedDeformConvForwardCUDAKernel<__half2>(
          reinterpret_cast<const __half2 *>(x), reinterpret_cast<const __half2 *>(weight),
          reinterpret_cast<const __half2 *>(bias), reinterpret_cast<const __half2 *>(offset),
          reinterpret_cast<const __half2 *>(mask), reinterpret_cast<__half2 *>(output), workSpace,
          batch, channels, height, width, channels_out, kernel_w, kernel_h, mStride.d[0],
          mStride.d[1], mPadding.d[0], mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup,
          mDeformableGroup, im2col_step, m_cublas_handle, stream);
      } else {
        ModulatedDeformConvForwardCUDAKernel<__half>(
          reinterpret_cast<const __half *>(x), reinterpret_cast<const __half *>(weight),
          reinterpret_cast<const __half *>(bias), reinterpret_cast<const __half *>(offset),
          reinterpret_cast<const __half *>(mask), reinterpret_cast<__half *>(output), workSpace,
          batch, channels, height, width, channels_out, kernel_w, kernel_h, mStride.d[0],
          mStride.d[1], mPadding.d[0], mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup,
          mDeformableGroup, im2col_step, m_cublas_handle, stream);
      }
      break;
    }

    case nvinfer1::DataType::kINT8: {
      if (data_type_bias == nvinfer1::DataType::kFLOAT) {
        ModulatedDeformConvForwardCUDAKernel_int8<float>(
          reinterpret_cast<const int8_4 *>(x), scale_i, reinterpret_cast<const int8_4 *>(weight),
          scale_w, reinterpret_cast<const float *>(bias), reinterpret_cast<const int8_t *>(offset),
          scale_off, reinterpret_cast<const int8_t *>(mask), scale_mask,
          reinterpret_cast<int8_t *>(output), scale_o, workSpace, batch, channels, height, width,
          channels_out, kernel_w, kernel_h, mStride.d[0], mStride.d[1], mPadding.d[0],
          mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup, mDeformableGroup, im2col_step,
          m_cublas_handle, stream);
      } else {
        ModulatedDeformConvForwardCUDAKernel_int8<__half>(
          reinterpret_cast<const int8_4 *>(x), scale_i, reinterpret_cast<const int8_4 *>(weight),
          scale_w, reinterpret_cast<const __half *>(bias), reinterpret_cast<const int8_t *>(offset),
          scale_off, reinterpret_cast<const int8_t *>(mask), scale_mask,
          reinterpret_cast<int8_t *>(output), scale_o, workSpace, batch, channels, height, width,
          channels_out, kernel_w, kernel_h, mStride.d[0], mStride.d[1], mPadding.d[0],
          mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup, mDeformableGroup, im2col_step,
          m_cublas_handle, stream);
      }
      break;
    }

    default:
      printf(
        "[ModulatedDeformableConv2d] Unsupported data type: %d\n", static_cast<int>(data_type));
      return 1;
  }

  return 0;
}

size_t ModulatedDeformableConv2dPlugin::getSerializationSize() const noexcept
{
  return sizeof(mStride) + sizeof(mPadding) + sizeof(mDilation) + sizeof(mDeformableGroup) +
         sizeof(mGroup);
}

void ModulatedDeformableConv2dPlugin::serialize(void * buffer) const noexcept
{
  serialize_value(&buffer, mStride);
  serialize_value(&buffer, mPadding);
  serialize_value(&buffer, mDilation);
  serialize_value(&buffer, mDeformableGroup);
  serialize_value(&buffer, mGroup);
}

bool ModulatedDeformableConv2dPlugin::supportsFormatCombination(
  int pos, const nvinfer1::PluginTensorDesc * inOut, int nbInputs, int nbOutputs) noexcept
{
  const int channels_out = inOut[nbInputs].dims.d[1] / mGroup;
  const int channels_in = inOut[0].dims.d[1];
  const bool use_int8 = channels_in % 4 == 0 && channels_out % 4 == 0;

  if (pos == 0) {
    if (use_h2) {
      return (inOut[pos].type == nvinfer1::DataType::kFLOAT &&
              inOut[pos].format == nvinfer1::TensorFormat::kLINEAR) ||
             (inOut[pos].type == nvinfer1::DataType::kHALF &&
              inOut[pos].format == nvinfer1::TensorFormat::kCHW2) ||
             (inOut[pos].type == nvinfer1::DataType::kINT8 &&
              inOut[pos].format == nvinfer1::TensorFormat::kCHW4 && use_int8);
    }
    return ((inOut[pos].type == nvinfer1::DataType::kFLOAT ||
             inOut[pos].type == nvinfer1::DataType::kHALF) &&
            inOut[pos].format == nvinfer1::TensorFormat::kLINEAR) ||
           (inOut[pos].type == nvinfer1::DataType::kINT8 &&
            inOut[pos].format == nvinfer1::TensorFormat::kCHW4 && use_int8);
  } else if (nbInputs == 5 && pos == 4 && inOut[0].type == nvinfer1::DataType::kINT8) {
    return (inOut[pos].type == nvinfer1::DataType::kFLOAT ||
            inOut[pos].type == nvinfer1::DataType::kHALF) &&
           inOut[pos].format == nvinfer1::TensorFormat::kLINEAR;
  } else if (
    (nbInputs == 5 && pos == 4) || pos == nbInputs || pos == 2 ||
    (pos == 1 && inOut[0].type == nvinfer1::DataType::kINT8)) {
    return inOut[pos].type == inOut[0].type && inOut[pos].format == nvinfer1::TensorFormat::kLINEAR;
  } else {
    return inOut[pos].type == inOut[0].type && inOut[pos].format == inOut[0].format;
  }
}

char const * ModulatedDeformableConv2dPlugin::getPluginType() const noexcept
{
  return use_h2 ? MDC_PLUGIN_NAME2 : MDC_PLUGIN_NAME;
}

char const * ModulatedDeformableConv2dPlugin::getPluginVersion() const noexcept
{
  return MDC_PLUGIN_VERSION;
}

void ModulatedDeformableConv2dPlugin::destroy() noexcept
{
  delete this;
}

nvinfer1::IPluginV2DynamicExt * ModulatedDeformableConv2dPlugin::clone() const noexcept
{
  try {
    auto * plugin = new ModulatedDeformableConv2dPlugin(
      mStride, mPadding, mDilation, mDeformableGroup, mGroup, use_h2);
    plugin->setPluginNamespace(getPluginNamespace());
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

void ModulatedDeformableConv2dPlugin::setPluginNamespace(const char * pluginNamespace) noexcept
{
  mPluginNamespace = pluginNamespace;
}

char const * ModulatedDeformableConv2dPlugin::getPluginNamespace() const noexcept
{
  return mPluginNamespace.c_str();
}

DataType ModulatedDeformableConv2dPlugin::getOutputDataType(
  int32_t index, const nvinfer1::DataType * inputTypes, int32_t nbInputs) const noexcept
{
  // cppcheck-suppress unknownMacro
  PLUGIN_ASSERT(inputTypes && nbInputs > 0 && index == 0)
  return inputTypes[0];
}

void ModulatedDeformableConv2dPlugin::attachToContext(
  cudnnContext * cudnn, cublasContext * cublas, nvinfer1::IGpuAllocator * allocator) noexcept
{
  m_cublas_handle = cublas;
}

void ModulatedDeformableConv2dPlugin::detachFromContext() noexcept
{
}

void ModulatedDeformableConv2dPlugin::configurePlugin(
  const nvinfer1::DynamicPluginTensorDesc * inputs, int nbInputs,
  const nvinfer1::DynamicPluginTensorDesc * outputs, int nbOutputs) noexcept
{
  if (nbInputs == 5) {
    mWithBias = true;
  }
  int channels_out = outputs[0].desc.dims.d[1];
  int channels_in = inputs[0].desc.dims.d[1];
  if (channels_out % mGroup != 0 || channels_in % mGroup != 0) {
    printf(
      "%s (channels_out = %d, channels_in = %d, group = %d): channels mod "
      "group should be zero.",
      this->getPluginType(), channels_out, channels_in, mGroup);
    exit(1);
  }
  if (inputs[0].desc.format == nvinfer1::TensorFormat::kCHW2) {
    int channels = inputs[0].desc.dims.d[1];
    if (mGroup > 1 && (channels / mGroup) % 2 == 1) {
      printf(
        "%s (channels = %d, group = %d): channels / group should be an "
        "even number.\n",
        MDC_PLUGIN_NAME2, channels, mGroup);
      exit(1);
    }
    if (mDeformableGroup > 1 && (channels / mDeformableGroup) % 2 == 1) {
      printf(
        "%s (channels = %d, deformable_group = %d): channels / "
        "deformable_group should be an even number.\n",
        MDC_PLUGIN_NAME2, channels, mDeformableGroup);
      exit(1);
    }
  } else if (inputs[0].desc.format == nvinfer1::TensorFormat::kCHW4) {
    int channels = inputs[0].desc.dims.d[1];
    if (mGroup > 1 && (channels / mGroup) % 4 == 1) {
      printf(
        "%s (channels = %d, deformable_group = %d): (channels / "
        "mDeformableGroup) mod 4 should be zero.\n",
        MDC_PLUGIN_NAME2, channels, mGroup);
      exit(1);
    }
    if (mDeformableGroup > 1 && (channels / mDeformableGroup) % 4 == 1) {
      printf(
        "%s (channels = %d, deformable_group = %d): (channels / "
        "mDeformableGroup) mod 4 should be zero.\n",
        MDC_PLUGIN_NAME2, channels, mDeformableGroup);
      exit(1);
    }
  }
}

ModulatedDeformableConv2dPluginCreator::ModulatedDeformableConv2dPluginCreator()
{
  mPluginAttributes.clear();
  mPluginAttributes.emplace_back(nvinfer1::PluginField("stride"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("padding"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("dilation"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("groups"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("deform_groups"));
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char * ModulatedDeformableConv2dPluginCreator::getPluginName() const noexcept
{
  return MDC_PLUGIN_NAME;
}

const char * ModulatedDeformableConv2dPluginCreator::getPluginVersion() const noexcept
{
  return MDC_PLUGIN_VERSION;
}

PluginFieldCollection const * ModulatedDeformableConv2dPluginCreator::getFieldNames() noexcept
{
  return &mFC;
}

IPluginV2DynamicExt * ModulatedDeformableConv2dPluginCreator::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  nvinfer1::Dims stride{2, {1, 1}};
  nvinfer1::Dims padding{2, {0, 0}};
  nvinfer1::Dims dilation{2, {1, 1}};
  int deformableGroup = 1;
  int group = 1;

  for (int i = 0; i < fc->nbFields; i++) {
    if (fc->fields[i].data == nullptr) {
      continue;
    }
    char const * attrName = fc->fields[i].name;
    if (!strcmp(attrName, "deform_groups")) {
      deformableGroup = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (!strcmp(attrName, "groups")) {
      group = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (!strcmp(attrName, "stride")) {
      stride.nbDims = 2;
      stride.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      stride.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }

    if (!strcmp(attrName, "padding")) {
      padding.nbDims = 2;
      padding.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      padding.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }

    if (!strcmp(attrName, "dilation")) {
      dilation.nbDims = 2;
      dilation.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      dilation.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }
  }

  try {
    auto * plugin =
      new ModulatedDeformableConv2dPlugin(stride, padding, dilation, deformableGroup, group, false);
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

IPluginV2DynamicExt * ModulatedDeformableConv2dPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  try {
    auto * plugin = new ModulatedDeformableConv2dPlugin{serialData, serialLength, false};
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

ModulatedDeformableConv2dPluginCreator2::ModulatedDeformableConv2dPluginCreator2()
{
  mPluginAttributes.clear();
  mPluginAttributes.emplace_back(nvinfer1::PluginField("stride"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("padding"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("dilation"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("groups"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("deform_groups"));
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char * ModulatedDeformableConv2dPluginCreator2::getPluginName() const noexcept
{
  return MDC_PLUGIN_NAME2;
}

const char * ModulatedDeformableConv2dPluginCreator2::getPluginVersion() const noexcept
{
  return MDC_PLUGIN_VERSION;
}

PluginFieldCollection const * ModulatedDeformableConv2dPluginCreator2::getFieldNames() noexcept
{
  return &mFC;
}

IPluginV2DynamicExt * ModulatedDeformableConv2dPluginCreator2::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  nvinfer1::Dims stride{2, {1, 1}};
  nvinfer1::Dims padding{2, {0, 0}};
  nvinfer1::Dims dilation{2, {1, 1}};
  int deformableGroup = 1;
  int group = 1;

  for (int i = 0; i < fc->nbFields; i++) {
    if (fc->fields[i].data == nullptr) {
      continue;
    }
    char const * attrName = fc->fields[i].name;
    if (!strcmp(attrName, "deform_groups")) {
      deformableGroup = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (!strcmp(attrName, "groups")) {
      group = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (!strcmp(attrName, "stride")) {
      stride.nbDims = 2;
      stride.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      stride.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }

    if (!strcmp(attrName, "padding")) {
      padding.nbDims = 2;
      padding.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      padding.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }

    if (!strcmp(attrName, "dilation")) {
      dilation.nbDims = 2;
      dilation.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      dilation.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }
  }

  try {
    auto * plugin =
      new ModulatedDeformableConv2dPlugin(stride, padding, dilation, deformableGroup, group, true);
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

IPluginV2DynamicExt * ModulatedDeformableConv2dPluginCreator2::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  try {
    auto * plugin = new ModulatedDeformableConv2dPlugin{serialData, serialLength, true};
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

REGISTER_TENSORRT_PLUGIN(ModulatedDeformableConv2dPluginCreator);
REGISTER_TENSORRT_PLUGIN(ModulatedDeformableConv2dPluginCreator2);
