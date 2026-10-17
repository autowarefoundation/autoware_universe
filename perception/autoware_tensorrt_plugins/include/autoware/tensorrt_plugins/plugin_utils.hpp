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

#ifndef AUTOWARE__TENSORRT_PLUGINS__PLUGIN_UTILS_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__PLUGIN_UTILS_HPP_

#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include <cstdint>
#include <exception>

void caughtError(std::exception const & e);

void logDebug(char const * msg);

void logWarning(char const * msg);

cudaError_t reportCudaStatus(
  cudaError_t status, char const * msg, char const * file, std::int32_t line);

/// \brief Whether \p stream is currently being captured into a CUDA graph.
bool isStreamCapturing(cudaStream_t stream);

/// \brief Zero every output of a plugin, for use when its real work cannot run.
///
/// The spconv-backed plugins compute their output extents on the host, which forces host
/// synchronization and library handle creation that CUDA rejects while a stream is capturing.
/// TensorRT captures the stream when it times tactics during engine build, so those plugins
/// cannot execute there. Writing deterministic zeros lets the build proceed; the tactic timing it
/// produces for these layers is meaningless, which is harmless because they expose a single
/// tactic. Callers must only use this while \ref isStreamCapturing is true.
cudaError_t zeroPluginOutputs(
  nvinfer1::PluginTensorDesc const * output_desc, std::int32_t num_outputs, void * const * outputs,
  cudaStream_t stream);

/// \brief Log, at most once per plugin instance, that capture forced the zero-output path.
void warnOnceStreamCaptureUnsupported(char const * plugin_name, bool & already_warned);

#define PLUGIN_CUDA_CHECK(val) reportCudaStatus((val), #val, __FILE__, __LINE__)

#define PLUGIN_ASSERT(val) reportAssertion((val), #val, __FILE__, __LINE__)
void reportAssertion(bool success, char const * msg, char const * file, std::int32_t line);

#define PLUGIN_ASSERT_MSG(val, detail) reportAssertionMsg((val), #val, (detail), __FILE__, __LINE__)
void reportAssertionMsg(
  bool success, char const * msg, char const * detail, char const * file, std::int32_t line);

#define PLUGIN_VALIDATE(val) reportValidation((val), #val, __FILE__, __LINE__)
void reportValidation(bool success, char const * msg, char const * file, std::int32_t line);

#define PLUGIN_VALIDATE_MSG(val, detail) \
  reportValidationMsg((val), #val, (detail), __FILE__, __LINE__)
void reportValidationMsg(
  bool success, char const * msg, char const * detail, char const * file, std::int32_t line);

#endif  // AUTOWARE__TENSORRT_PLUGINS__PLUGIN_UTILS_HPP_
