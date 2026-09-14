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
