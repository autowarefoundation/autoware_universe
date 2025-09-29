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

// cspell:ignore MSDA
#include "multiScaleDeformableAttnPlugin.h"

#include "checkMacrosPlugin.h"
#include "multiScaleDeformableAttnKernel.h"
#include "serialize.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>

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
using trt_plugin::MultiScaleDeformableAttnPlugin;
using trt_plugin::MultiScaleDeformableAttnPluginCreator;
using trt_plugin::MultiScaleDeformableAttnPluginCreator2;

namespace
{
constexpr char const * MSDA_PLUGIN_VERSION{"1"};
constexpr char const * MSDA_PLUGIN_NAME{"MultiScaleDeformableAttnTRT"};
constexpr char const * MSDA_PLUGIN_NAME2{"MultiScaleDeformableAttnTRT2"};
}  // namespace

PluginFieldCollection MultiScaleDeformableAttnPluginCreator::mFC{};
std::vector<PluginField> MultiScaleDeformableAttnPluginCreator::mPluginAttributes;

PluginFieldCollection MultiScaleDeformableAttnPluginCreator2::mFC{};
std::vector<PluginField> MultiScaleDeformableAttnPluginCreator2::mPluginAttributes;

MultiScaleDeformableAttnPlugin::MultiScaleDeformableAttnPlugin(bool use_h2) : use_h2(use_h2)
{
}

MultiScaleDeformableAttnPlugin::MultiScaleDeformableAttnPlugin(
  const void * serialData, size_t serialLength, bool use_h2)
: use_h2(use_h2)
{
}

MultiScaleDeformableAttnPlugin::~MultiScaleDeformableAttnPlugin()
{
  terminate();
}

int32_t MultiScaleDeformableAttnPlugin::getNbOutputs() const noexcept
{
  return 1;
}

DimsExprs MultiScaleDeformableAttnPlugin::getOutputDimensions(
  int32_t outputIndex, const nvinfer1::DimsExprs * inputs, int32_t nbInputs,
  nvinfer1::IExprBuilder & exprBuilder) noexcept
{
  DimsExprs outputDim;
  outputDim.nbDims = 4;
  outputDim.d[0] = inputs[0].d[0];
  outputDim.d[1] = inputs[3].d[1];
  outputDim.d[2] = inputs[0].d[2];
  outputDim.d[3] = inputs[0].d[3];
  return outputDim;
}

int32_t MultiScaleDeformableAttnPlugin::initialize() noexcept
{
  return 0;
}

void MultiScaleDeformableAttnPlugin::terminate() noexcept
{
}

size_t MultiScaleDeformableAttnPlugin::getWorkspaceSize(
  const nvinfer1::PluginTensorDesc * inputs, int32_t nbInputs,
  const nvinfer1::PluginTensorDesc * outputs, int32_t nbOutputs) const noexcept
{
  return 0;
}

size_t MultiScaleDeformableAttnPlugin::getSerializationSize() const noexcept
{
  return 0;
}

void printTensorValuesM(
  const char * name, const void * data, nvinfer1::DataType dataType, int count, cudaStream_t stream)
{
  if (!data) {
    std::cout << "[Plugin Debug] " << name << " is nullptr\n";
    return;
  }

  std::cout << "[Plugin Debug] " << name << " values (first " << count << " elements):\n";

  float * host_values = nullptr;
  try {
    host_values = new float[count];
  } catch (std::exception const & e) {
    std::cerr << "Allocation failed for host_values: " << e.what() << std::endl;
    return;
  }
  cudaError_t err;

  switch (dataType) {
    case nvinfer1::DataType::kFLOAT: {
      err =
        cudaMemcpyAsync(host_values, data, count * sizeof(float), cudaMemcpyDeviceToHost, stream);
      cudaStreamSynchronize(stream);
      break;
    }
    case nvinfer1::DataType::kHALF: {
      __half * host_half = new __half[count];
      err =
        cudaMemcpyAsync(host_half, data, count * sizeof(__half), cudaMemcpyDeviceToHost, stream);
      cudaStreamSynchronize(stream);
      for (int i = 0; i < count; ++i) {
        host_values[i] = __half2float(host_half[i]);
      }
      delete[] host_half;
      break;
    }
    case nvinfer1::DataType::kINT8: {
      int8_t * host_int8 = new int8_t[count];
      err =
        cudaMemcpyAsync(host_int8, data, count * sizeof(int8_t), cudaMemcpyDeviceToHost, stream);
      cudaStreamSynchronize(stream);
      for (int i = 0; i < count; ++i) {
        host_values[i] = static_cast<float>(host_int8[i]);
      }
      delete[] host_int8;
      break;
    }
    case nvinfer1::DataType::kINT32: {
      int32_t * host_int32 = new int32_t[count];
      err =
        cudaMemcpyAsync(host_int32, data, count * sizeof(int32_t), cudaMemcpyDeviceToHost, stream);
      cudaStreamSynchronize(stream);
      for (int i = 0; i < count; ++i) {
        host_values[i] = static_cast<float>(host_int32[i]);
      }
      delete[] host_int32;
      break;
    }
    default:
      std::cout << "[Plugin Debug] Unsupported data type for " << name << "\n";
      delete[] host_values;
      return;
  }

  if (err != cudaSuccess) {
    std::cerr << "[Plugin Debug] cudaMemcpyAsync failed for " << name << ": "
              << cudaGetErrorString(err) << "\n";
    delete[] host_values;
    return;
  }

  for (int i = 0; i < count; ++i) {
    std::cout << "  " << name << "[" << i << "] = " << host_values[i] << "\n";
  }

  delete[] host_values;
}

int32_t MultiScaleDeformableAttnPlugin::enqueue(
  const nvinfer1::PluginTensorDesc * inputDesc, const nvinfer1::PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  float scale_value = inputDesc[0].scale, scale_ref = inputDesc[2].scale,
        scale_offset = inputDesc[3].scale, scale_weight = inputDesc[4].scale,
        scale_out = outputDesc[0].scale;
  Dims value_dims = inputDesc[0].dims;
  const int batch = value_dims.d[0];
  const int spatial_size = value_dims.d[1];
  const int num_heads = value_dims.d[2];
  const int channels = value_dims.d[3];

  const int num_levels = inputDesc[1].dims.d[0];

  const int points_per_group = inputDesc[2].dims.d[3] / 2;
  const int num_query = inputDesc[3].dims.d[1];
  const int num_point = inputDesc[4].dims.d[3] / num_levels;

  auto data_type = inputDesc[0].type;
  auto data_type_rp = inputDesc[2].type;
  ASSERT(
    data_type == DataType::kFLOAT || data_type == DataType::kHALF || data_type == DataType::kINT8)
  ASSERT(data_type_rp == DataType::kFLOAT || data_type_rp == DataType::kHALF)

  // Create a temporary buffer for int32 spatial_shapes if input is INT64
  int32_t * spatial_shapes_int32 = nullptr;
  bool allocated_temp_buffer = false;

  if (inputDesc[1].type == nvinfer1::DataType::kINT64) {
    int32_t * spatial_shapes_int32_temp = nullptr;
    size_t buffer_size = num_levels * 2 * sizeof(int32_t);
    cudaMalloc(reinterpret_cast<void **>(&spatial_shapes_int32_temp), buffer_size);
    allocated_temp_buffer = true;

    int64_t * host_int64 = nullptr;
    int32_t * host_int32 = nullptr;
    try {
      host_int64 = new int64_t[num_levels * 2];
      host_int32 = new int32_t[num_levels * 2];
    } catch (std::exception const & e) {
      std::cerr << "Allocation failed in enqueue: " << e.what() << std::endl;
      delete[] host_int64;
      delete[] host_int32;
      return 1;
    }

    cudaMemcpy(host_int64, inputs[1], num_levels * 2 * sizeof(int64_t), cudaMemcpyDeviceToHost);

    for (int i = 0; i < num_levels * 2; i++) {
      host_int32[i] = static_cast<int32_t>(host_int64[i]);
    }

    cudaMemcpy(spatial_shapes_int32_temp, host_int32, buffer_size, cudaMemcpyHostToDevice);

    delete[] host_int64;
    delete[] host_int32;

    spatial_shapes_int32 = spatial_shapes_int32_temp;
  }

  switch (data_type) {
    case DataType::kFLOAT: {
      ms_deformable_im2col_cuda<float>(
        reinterpret_cast<const float *>(inputs[0]), spatial_shapes_int32,
        reinterpret_cast<const float *>(inputs[2]), reinterpret_cast<const float *>(inputs[3]),
        reinterpret_cast<const float *>(inputs[4]), batch, spatial_size, num_heads, channels,
        num_levels, num_query, num_point, points_per_group, reinterpret_cast<float *>(outputs[0]),
        stream);
      break;
    }

    case DataType::kHALF: {
      if (use_h2) {
        ms_deformable_im2col_cuda_h2(
          reinterpret_cast<const __half2 *>(inputs[0]), spatial_shapes_int32,
          reinterpret_cast<const __half2 *>(inputs[2]),
          reinterpret_cast<const __half2 *>(inputs[3]), reinterpret_cast<const __half *>(inputs[4]),
          batch, spatial_size, num_heads, channels, num_levels, num_query, num_point,
          points_per_group, reinterpret_cast<__half2 *>(outputs[0]), stream);
      } else {
        ms_deformable_im2col_cuda<__half>(
          reinterpret_cast<const __half *>(inputs[0]), spatial_shapes_int32,
          reinterpret_cast<const __half *>(inputs[2]), reinterpret_cast<const __half *>(inputs[3]),
          reinterpret_cast<const __half *>(inputs[4]), batch, spatial_size, num_heads, channels,
          num_levels, num_query, num_point, points_per_group,
          reinterpret_cast<__half *>(outputs[0]), stream);
      }
      break;
    }

    case DataType::kINT8: {
      if (data_type_rp == DataType::kHALF) {
        ms_deformable_im2col_cuda_int8<__half2>(
          reinterpret_cast<const int8_4 *>(inputs[0]), scale_value,
          reinterpret_cast<const int32_t *>(inputs[1]),
          reinterpret_cast<const __half2 *>(inputs[2]), reinterpret_cast<const int8_4 *>(inputs[3]),
          scale_offset, reinterpret_cast<const int8_4 *>(inputs[4]), scale_weight, batch,
          spatial_size, num_heads, channels, num_levels, num_query, num_point, points_per_group,
          reinterpret_cast<int8_4 *>(outputs[0]), scale_out, stream);
      } else {
        ms_deformable_im2col_cuda_int8<float>(
          reinterpret_cast<const int8_4 *>(inputs[0]), scale_value,
          reinterpret_cast<const int32_t *>(inputs[1]), reinterpret_cast<const float *>(inputs[2]),
          reinterpret_cast<const int8_4 *>(inputs[3]), scale_offset,
          reinterpret_cast<const int8_4 *>(inputs[4]), scale_weight, batch, spatial_size, num_heads,
          channels, num_levels, num_query, num_point, points_per_group,
          reinterpret_cast<int8_4 *>(outputs[0]), scale_out, stream);
      }
      break;
    }
    default:
      return 1;
  }

  // Clean up if we allocated a buffer
  if (allocated_temp_buffer && spatial_shapes_int32) {
    cudaFree(spatial_shapes_int32);
  }

  return 0;
}

void MultiScaleDeformableAttnPlugin::serialize(void * buffer) const noexcept
{
}

bool MultiScaleDeformableAttnPlugin::supportsFormatCombination(
  int32_t pos, const nvinfer1::PluginTensorDesc * inOut, int32_t nbInputs,
  int32_t nbOutputs) noexcept
{
  const int channels = inOut[0].dims.d[3];
  const int point_num = inOut[4].dims.d[3] / inOut[1].dims.d[0];
  bool use_int8 = true;
  if (channels % 4 != 0 || point_num % 4 != 0) {
    use_int8 = false;
  }

  bool result = false;

  switch (pos) {
    case 0: {
      bool cond1 =
        (inOut[pos].type == nvinfer1::DataType::kFLOAT &&
         inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);
      bool cond2 =
        (inOut[pos].type == nvinfer1::DataType::kHALF &&
         inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);
      bool cond3 =
        (inOut[pos].type == nvinfer1::DataType::kINT8 &&
         inOut[pos].format == nvinfer1::TensorFormat::kLINEAR && use_int8);

      result = cond1 || cond2 || cond3;

      return result;
    }

    case 1: {
      bool type_ok =
        (inOut[pos].type == nvinfer1::DataType::kINT32 ||
         inOut[pos].type == nvinfer1::DataType::kINT64);
      bool format_ok = (inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);

      result = type_ok && format_ok;
      return result;
    }

    case 2: {
      bool input_is_float_or_half =
        (inOut[0].type == nvinfer1::DataType::kFLOAT || inOut[0].type == nvinfer1::DataType::kHALF);

      if (input_is_float_or_half) {
        bool type_match = (inOut[pos].type == inOut[0].type);
        bool format_ok = (inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);
        result = type_match && format_ok;
      } else {
        bool type_ok =
          (inOut[pos].type == nvinfer1::DataType::kHALF ||
           inOut[pos].type == nvinfer1::DataType::kFLOAT);
        bool format_ok = (inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);
        result = type_ok && format_ok;
      }
      return result;
    }

    case 3: {
      bool type_match = (inOut[pos].type == inOut[0].type);
      bool format_ok = (inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);
      result = type_match && format_ok;
      return result;
    }

    case 4: {
      bool type_match = (inOut[pos].type == inOut[0].type);
      bool format_ok = (inOut[pos].format == nvinfer1::TensorFormat::kLINEAR);
      result = type_match && format_ok;
      return result;
    }

    case 5: {
      bool type_match = (inOut[pos].type == inOut[0].type);
      bool format_match = (inOut[pos].format == inOut[0].format);
      result = type_match && format_match;
      return result;
    }

    default:
      return false;
  }
}

char const * MultiScaleDeformableAttnPlugin::getPluginType() const noexcept
{
  return use_h2 ? MSDA_PLUGIN_NAME2 : MSDA_PLUGIN_NAME;
}

char const * MultiScaleDeformableAttnPlugin::getPluginVersion() const noexcept
{
  return MSDA_PLUGIN_VERSION;
}

void MultiScaleDeformableAttnPlugin::destroy() noexcept
{
  delete this;
}

IPluginV2DynamicExt * MultiScaleDeformableAttnPlugin::clone() const noexcept
{
  try {
    auto * plugin = new MultiScaleDeformableAttnPlugin{use_h2};
    plugin->setPluginNamespace(mPluginNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

void MultiScaleDeformableAttnPlugin::setPluginNamespace(const char * pluginNamespace) noexcept
{
  mPluginNamespace = pluginNamespace;
}

char const * MultiScaleDeformableAttnPlugin::getPluginNamespace() const noexcept
{
  return mPluginNamespace.c_str();
}

DataType MultiScaleDeformableAttnPlugin::getOutputDataType(
  int32_t index, const nvinfer1::DataType * inputTypes, int32_t nbInputs) const noexcept
{
  // cppcheck-suppress unknownMacro
  PLUGIN_ASSERT(inputTypes && nbInputs > 0 && index == 0)
  return inputTypes[0];
}

void MultiScaleDeformableAttnPlugin::attachToContext(
  cudnnContext * cudnn, cublasContext * cublas, nvinfer1::IGpuAllocator * allocator) noexcept
{
}

void MultiScaleDeformableAttnPlugin::detachFromContext() noexcept
{
}

void MultiScaleDeformableAttnPlugin::configurePlugin(
  const nvinfer1::DynamicPluginTensorDesc * in, int32_t nbInputs,
  const nvinfer1::DynamicPluginTensorDesc * out, int32_t nbOutputs) noexcept
{
  // cppcheck-suppress unknownMacro
  PLUGIN_ASSERT(nbInputs == 5)
  const int channels = in[0].desc.dims.d[3];
  if (use_h2) {
    if (channels % 2 != 0) {
      use_h2 = false;
    }
  }
}

MultiScaleDeformableAttnPluginCreator::MultiScaleDeformableAttnPluginCreator()
{
  mPluginAttributes.clear();
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

char const * MultiScaleDeformableAttnPluginCreator::getPluginName() const noexcept
{
  return MSDA_PLUGIN_NAME;
}

char const * MultiScaleDeformableAttnPluginCreator::getPluginVersion() const noexcept
{
  return MSDA_PLUGIN_VERSION;
}

PluginFieldCollection const * MultiScaleDeformableAttnPluginCreator::getFieldNames() noexcept
{
  return &mFC;
}

IPluginV2DynamicExt * MultiScaleDeformableAttnPluginCreator::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  try {
    auto * plugin = new MultiScaleDeformableAttnPlugin(false);
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

IPluginV2DynamicExt * MultiScaleDeformableAttnPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  try {
    auto * plugin = new MultiScaleDeformableAttnPlugin{serialData, serialLength, false};
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

MultiScaleDeformableAttnPluginCreator2::MultiScaleDeformableAttnPluginCreator2()
{
  mPluginAttributes.clear();
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

char const * MultiScaleDeformableAttnPluginCreator2::getPluginName() const noexcept
{
  return MSDA_PLUGIN_NAME2;
}

char const * MultiScaleDeformableAttnPluginCreator2::getPluginVersion() const noexcept
{
  return MSDA_PLUGIN_VERSION;
}

PluginFieldCollection const * MultiScaleDeformableAttnPluginCreator2::getFieldNames() noexcept
{
  return &mFC;
}

IPluginV2DynamicExt * MultiScaleDeformableAttnPluginCreator2::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  try {
    auto * plugin = new MultiScaleDeformableAttnPlugin(true);
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

IPluginV2DynamicExt * MultiScaleDeformableAttnPluginCreator2::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  try {
    auto * plugin = new MultiScaleDeformableAttnPlugin{serialData, serialLength, true};
    plugin->setPluginNamespace(mNamespace.c_str());
    plugin->initialize();
    return plugin;
  } catch (std::exception const & e) {
    nvinfer1::plugin::caughtError(e);
  }
  return nullptr;
}

REGISTER_TENSORRT_PLUGIN(MultiScaleDeformableAttnPluginCreator);
REGISTER_TENSORRT_PLUGIN(MultiScaleDeformableAttnPluginCreator2);
