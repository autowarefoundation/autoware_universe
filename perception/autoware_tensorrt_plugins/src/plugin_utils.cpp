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

#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sstream>

namespace
{
void logAssertionFailure(
  char const * msg, char const * detail, char const * file, std::int32_t line)
{
  std::ostringstream stream;
  stream << "Assertion failed: " << msg << '\n';
  if (detail != nullptr && std::strlen(detail) > 0) {
    stream << detail << '\n';
  }
  stream << file << ':' << line << '\n' << "Aborting..." << '\n';
  getLogger()->log(nvinfer1::ILogger::Severity::kINTERNAL_ERROR, stream.str().c_str());
}

void logValidationFailure(
  char const * msg, char const * detail, char const * file, std::int32_t line)
{
  std::ostringstream stream;
  stream << "Validation failed: " << msg << '\n';
  if (detail != nullptr && std::strlen(detail) > 0) {
    stream << detail << '\n';
  }
  stream << file << ':' << line << '\n';
  getLogger()->log(nvinfer1::ILogger::Severity::kINTERNAL_ERROR, stream.str().c_str());
}
}  // namespace

void caughtError(std::exception const & e)
{
  getLogger()->log(nvinfer1::ILogger::Severity::kINTERNAL_ERROR, e.what());
}

void logDebug(char const * msg)
{
  getLogger()->log(nvinfer1::ILogger::Severity::kVERBOSE, msg);
}

void logWarning(char const * msg)
{
  getLogger()->log(nvinfer1::ILogger::Severity::kWARNING, msg);
}

cudaError_t reportCudaStatus(
  cudaError_t status, char const * msg, char const * file, std::int32_t line)
{
  if (status != cudaSuccess) {
    std::ostringstream stream;
    stream << "CUDA call failed: " << msg << std::endl
           << file << ':' << line << std::endl
           << cudaGetErrorName(status) << ": " << cudaGetErrorString(status) << std::endl;
    getLogger()->log(nvinfer1::ILogger::Severity::kERROR, stream.str().c_str());
  }
  return status;
}

void reportAssertion(bool success, char const * msg, char const * file, std::int32_t line)
{
  if (!success) {
    logAssertionFailure(msg, nullptr, file, line);
    std::abort();
  }
}

void reportAssertionMsg(
  bool success, char const * msg, char const * detail, char const * file, std::int32_t line)
{
  if (!success) {
    logAssertionFailure(msg, detail, file, line);
    std::abort();
  }
}

void reportValidation(bool success, char const * msg, char const * file, std::int32_t line)
{
  if (!success) {
    logValidationFailure(msg, nullptr, file, line);
  }
}

void reportValidationMsg(
  bool success, char const * msg, char const * detail, char const * file, std::int32_t line)
{
  if (!success) {
    logValidationFailure(msg, detail, file, line);
  }
}

bool isStreamCapturing(cudaStream_t stream)
{
  cudaStreamCaptureStatus capture_status{cudaStreamCaptureStatusNone};
  if (cudaStreamIsCapturing(stream, &capture_status) != cudaSuccess) {
    return false;
  }
  return capture_status != cudaStreamCaptureStatusNone;
}

cudaError_t zeroPluginOutputs(
  nvinfer1::PluginTensorDesc const * output_desc, std::int32_t num_outputs, void * const * outputs,
  cudaStream_t stream)
{
  for (std::int32_t output_index = 0; output_index < num_outputs; ++output_index) {
    auto const & desc = output_desc[output_index];
    std::size_t num_elements = 1;
    for (std::int32_t dim = 0; dim < desc.dims.nbDims; ++dim) {
      num_elements *= static_cast<std::size_t>(std::max<std::int64_t>(desc.dims.d[dim], 0));
    }
    if (num_elements == 0) {
      continue;
    }
    std::size_t element_size = 0;
    switch (desc.type) {
      case nvinfer1::DataType::kINT64:
        element_size = 8;
        break;
      case nvinfer1::DataType::kFLOAT:
      case nvinfer1::DataType::kINT32:
        element_size = 4;
        break;
      case nvinfer1::DataType::kHALF:
      case nvinfer1::DataType::kBF16:
        element_size = 2;
        break;
      default:
        element_size = 1;
        break;
    }
    if (
      auto const status =
        cudaMemsetAsync(outputs[output_index], 0, num_elements * element_size, stream);
      status != cudaSuccess) {
      return status;
    }
  }
  return cudaSuccess;
}

void warnOnceStreamCaptureUnsupported(char const * plugin_name, bool & already_warned)
{
  if (already_warned) {
    return;
  }
  already_warned = true;
  std::ostringstream stream;
  stream << plugin_name
         << ": the stream is being captured into a CUDA graph, which this plugin cannot support "
            "because it derives its output extents on the host. Writing zero outputs instead. "
            "This is expected while TensorRT times tactics during engine build; if it appears "
            "during inference, the CUDA graph result is not valid.";
  // getLogger() yields null until TensorRT hands the library a logger finder, which does not
  // happen when the plugin library is loaded directly rather than through the plugin registry.
  if (auto * logger = getLogger(); logger != nullptr) {
    logger->log(nvinfer1::ILogger::Severity::kWARNING, stream.str().c_str());
  }
}
