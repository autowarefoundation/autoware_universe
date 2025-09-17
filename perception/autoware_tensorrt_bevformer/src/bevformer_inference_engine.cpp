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
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bevformer_inference_engine.hpp"

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ros_utils.hpp"

#include <rclcpp/logging.hpp>

#include <NvOnnxParser.h>
#include <dlfcn.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

// Logger implementation for TensorRT
class Logger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cout << "[TensorRT] " << msg << std::endl;
    }
  }
};
static Logger gLogger;

BEVFormerInferenceEngine::BEVFormerInferenceEngine(const rclcpp::Logger & logger) : logger_(logger)
{
}

BEVFormerInferenceEngine::~BEVFormerInferenceEngine()
{
  // Proper cleanup for TensorRT objects
  if (context_) {
    delete context_;
    context_ = nullptr;
  }
  if (engine_) {
    delete engine_;
    engine_ = nullptr;
  }
  if (runtime_) {
    delete runtime_;
    runtime_ = nullptr;
  }

  // Clean up plugin library handle
  if (plugin_handle_) {
    dlclose(plugin_handle_);
    plugin_handle_ = nullptr;
  }
}

std::vector<char> BEVFormerInferenceEngine::readEngineFile(const std::string & engine_file)
{
  std::ifstream file(engine_file, std::ios::binary);
  RCLCPP_INFO(logger_, "Reading engine file: %s", engine_file.c_str());
  if (!file) {
    std::string error_msg = "Failed to open engine file: " + engine_file;
    if (access(engine_file.c_str(), F_OK) == -1) {
      error_msg += " (file does not exist)";
    } else if (access(engine_file.c_str(), R_OK) == -1) {
      error_msg += " (permission denied)";
    }
    throw std::runtime_error(error_msg);
  }
  return std::vector<char>(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

std::string BEVFormerInferenceEngine::findPluginLibrary(const std::string & plugin_path)
{
  if (!plugin_path.empty()) {
    if (access(plugin_path.c_str(), F_OK) == 0) {
      RCLCPP_INFO(logger_, "Using user-provided plugin path: %s", plugin_path.c_str());
      return plugin_path;
    } else {
      throw std::runtime_error("Provided plugin path does not exist: " + plugin_path);
    }
  }

  const std::string package_name = "autoware_tensorrt_bevformer";
  std::string package_prefix;

  try {
    package_prefix = ament_index_cpp::get_package_prefix(package_name);
  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    RCLCPP_ERROR(
      logger_, "Could not find package '%s'. Is the environment sourced?", package_name.c_str());
  }

  const std::string library_path = package_prefix + "/lib/libtensorrt_ops.so";

  if (access(library_path.c_str(), F_OK) == 0) {
    RCLCPP_INFO(logger_, "Found plugin library at standard location: %s", library_path.c_str());
    return library_path;
  }

  RCLCPP_ERROR(
    logger_, "Found package '%s' but the plugin library is missing at the expected path: %s",
    package_name.c_str(), library_path.c_str());

  throw std::runtime_error(
    "Could not find libtensorrt_ops.so. The 'autoware_tensorrt_bevformer' package was found, "
    "but the library is missing from its 'lib' directory. Please rebuild the package.");
}

bool BEVFormerInferenceEngine::loadPlugins(const std::string & plugin_path)
{
  std::string effective_plugin_path;

  try {
    effective_plugin_path = findPluginLibrary(plugin_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  }

  RCLCPP_INFO(logger_, "Loading plugins from: %s", effective_plugin_path.c_str());

  // Load with RTLD_GLOBAL to ensure symbols are globally visible
  void * handle = dlopen(effective_plugin_path.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!handle) {
    RCLCPP_ERROR(logger_, "Failed to load plugin library: %s", dlerror());
    return false;
  }

  plugin_handle_ = handle;

  // To Get and register plugin registry
  auto getPluginRegistryFn = (nvinfer1::IPluginRegistry * (*)()) dlsym(handle, "getPluginRegistry");
  if (!getPluginRegistryFn) {
    RCLCPP_ERROR(logger_, "Failed to find getPluginRegistry function: %s", dlerror());
    dlclose(handle);
    plugin_handle_ = nullptr;
    return false;
  }

  const auto * registry = getPluginRegistryFn();
  if (!registry) {
    RCLCPP_ERROR(logger_, "Plugin registry is null");
    dlclose(handle);
    plugin_handle_ = nullptr;
    return false;
  }

  RCLCPP_INFO(logger_, "Successfully loaded plugin registry");
  return true;
}

bool BEVFormerInferenceEngine::initialize(
  const std::string & engine_file, const std::string & plugin_path)
{
  try {
    // Create runtime with logger
    runtime_ = nvinfer1::createInferRuntime(gLogger);
    if (!runtime_) {
      RCLCPP_ERROR(logger_, "Failed to create TensorRT runtime");
      return false;
    }

    // Load plugins first
    if (!loadPlugins(plugin_path)) {
      RCLCPP_ERROR(logger_, "Failed to load plugins");
      return false;
    }

    // Read the engine file
    auto engine_data = readEngineFile(engine_file);
    if (engine_data.empty()) {
      RCLCPP_ERROR(logger_, "Engine file is empty or could not be read");
      return false;
    }

    RCLCPP_INFO(logger_, "Engine size: %zu bytes", engine_data.size());

    // Create CUDA stream for initialization
    cudaStream_t stream;
    CHECK_CUDA(cudaStreamCreate(&stream));

    // Deserialize engine
    RCLCPP_INFO(logger_, "Deserializing TensorRT engine...");
    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
    if (!engine_) {
      RCLCPP_ERROR(logger_, "Failed to deserialize TensorRT engine");
      CHECK_CUDA(cudaStreamDestroy(stream));
      return false;
    }

    RCLCPP_INFO(logger_, "Creating execution context...");
    context_ = engine_->createExecutionContext();

    if (!context_) {
      RCLCPP_ERROR(logger_, "Failed to create TensorRT execution context");
      CHECK_CUDA(cudaStreamDestroy(stream));
      return false;
    }

    // Clean up stream
    CHECK_CUDA(cudaStreamDestroy(stream));

    // Get tensor shapes from the engine
    getTensorShapes();

    // Print engine information
    RCLCPP_INFO(logger_, "TensorRT engine initialized successfully");
    RCLCPP_INFO(logger_, "Engine has %d IO tensors", engine_->getNbIOTensors());

    // Print all tensor names
    for (int i = 0; i < engine_->getNbIOTensors(); i++) {
      const char * name = engine_->getIOTensorName(i);
      bool isInput = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
      RCLCPP_INFO(logger_, "Tensor %d: %s (%s)", i, name, isInput ? "input" : "output");

      nvinfer1::Dims dims = engine_->getTensorShape(name);
      std::string shape = "(";
      for (int j = 0; j < dims.nbDims; j++) {
        shape += std::to_string(dims.d[j]);
        if (j < dims.nbDims - 1) {
          shape += ", ";
        }
      }
      shape += ")";
      RCLCPP_INFO(logger_, "  Shape: %s", shape.c_str());
    }

    initialized_ = true;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during engine initialization: %s", e.what());
    return false;
  }
}

bool BEVFormerInferenceEngine::buildEngineFromOnnx(
  const std::string & onnx_file, const std::string & engine_file, const std::string & plugin_path,
  int workspace_size, PrecisionType precision)
{
  std::string effective_engine_file = engine_file;
  if (precision == PrecisionType::FP32) {
    // Add _fp32 suffix before the extension
    size_t pos = effective_engine_file.rfind(".");
    if (pos != std::string::npos) {
      effective_engine_file.insert(pos, "_fp32");
    } else {
      effective_engine_file += "_fp32";
    }
  }

  // Check if engine file already exists
  if (checkFileExists(effective_engine_file)) {
    RCLCPP_INFO(
      logger_, "Engine file already exists at %s. Skipping build.", effective_engine_file.c_str());
    return initialize(effective_engine_file, plugin_path);
  }

  // Check if ONNX file exists
  if (!checkFileExists(onnx_file)) {
    RCLCPP_ERROR(logger_, "ONNX file not found: %s", onnx_file.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "Building TensorRT engine from ONNX model: %s", onnx_file.c_str());
  RCLCPP_INFO(logger_, "Engine will be saved to: %s", effective_engine_file.c_str());
  RCLCPP_INFO(logger_, "Using precision: %s", getPrecisionString(precision).c_str());

  // Load plugins first - use the centralized method
  if (!loadPlugins(plugin_path)) {
    RCLCPP_ERROR(logger_, "Failed to load plugins");
    return false;
  }

  // Starts to build engine
  auto start_time = std::chrono::high_resolution_clock::now();

  nvinfer1::IBuilder * builder = nvinfer1::createInferBuilder(gLogger);
  if (!builder) {
    RCLCPP_ERROR(logger_, "Failed to create TensorRT builder");
    return false;
  }

  // Create network definition with explicit batch
  const auto explicit_batch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  nvinfer1::INetworkDefinition * network = builder->createNetworkV2(explicit_batch);
  if (!network) {
    RCLCPP_ERROR(logger_, "Failed to create TensorRT network");
    delete builder;
    return false;
  }

  // Create ONNX parser
  nvonnxparser::IParser * parser = nvonnxparser::createParser(*network, gLogger);
  if (!parser) {
    RCLCPP_ERROR(logger_, "Failed to create ONNX parser");
    delete network;
    delete builder;
    return false;
  }

  // Parse ONNX model
  RCLCPP_INFO(logger_, "Parsing ONNX file...");
  bool parsed = parser->parseFromFile(
    onnx_file.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING));
  if (!parsed) {
    RCLCPP_ERROR(logger_, "Failed to parse ONNX file");
    delete parser;
    delete network;
    delete builder;
    return false;
  }

  nvinfer1::IBuilderConfig * config = builder->createBuilderConfig();
  if (!config) {
    RCLCPP_ERROR(logger_, "Failed to create builder config");
    delete parser;
    delete network;
    delete builder;
    return false;
  }

  // Set workspace size (in MB)
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size * (1ULL << 20));

  // Set precision based on parameter
  if (precision == PrecisionType::FP16) {
    // Enable FP16 precision
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    RCLCPP_INFO(logger_, "FP16 precision enabled for TensorRT engine");
  } else {
    RCLCPP_INFO(logger_, "FP32 precision enabled for TensorRT engine");
  }

  // Enable strict type constraints to force TensorRT to respect layer precision settings
  config->setFlag(nvinfer1::BuilderFlag::kOBEY_PRECISION_CONSTRAINTS);
  RCLCPP_INFO(logger_, "Precision constraints enabled to enforce layer precision settings");

  // Enable tactics from CUBLAS, CUBLAS_LT, and CUDNN
  config->setTacticSources(
    static_cast<nvinfer1::TacticSources>(
      static_cast<int32_t>(config->getTacticSources()) |
      static_cast<int32_t>(nvinfer1::TacticSource::kCUBLAS) |
      static_cast<int32_t>(nvinfer1::TacticSource::kCUBLAS_LT) |
      static_cast<int32_t>(nvinfer1::TacticSource::kCUDNN)));

  // Enable verbose logging during build
  config->setProfilingVerbosity(nvinfer1::ProfilingVerbosity::kDETAILED);

  // Build the engine
  RCLCPP_INFO(logger_, "Building TensorRT engine (this may take several minutes)...");
  nvinfer1::IHostMemory * serialized_engine = builder->buildSerializedNetwork(*network, *config);

  // Clean up resources
  delete config;
  delete parser;
  delete network;
  delete builder;

  if (!serialized_engine) {
    RCLCPP_ERROR(logger_, "Failed to build serialized engine");
    return false;
  }

  // Save engine to disk
  std::vector<char> engine_data(
    static_cast<char *>(serialized_engine->data()),
    static_cast<char *>(serialized_engine->data()) + serialized_engine->size());

  // Clean up serialized engine
  delete serialized_engine;

  bool saved = saveEngineToDisk(engine_data, effective_engine_file);

  if (!saved) {
    RCLCPP_ERROR(logger_, "Failed to save engine to disk");
    return false;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

  RCLCPP_INFO(logger_, "Engine built and saved successfully in %ld seconds", duration);

  // Initialize with the newly created engine
  return initialize(effective_engine_file, plugin_path);
}

bool BEVFormerInferenceEngine::saveEngineToDisk(
  const std::vector<char> & engine_data, const std::string & engine_file)
{
  std::ofstream file(engine_file, std::ios::binary);
  if (!file) {
    RCLCPP_ERROR(logger_, "Failed to open file for writing: %s", engine_file.c_str());
    return false;
  }

  file.write(engine_data.data(), engine_data.size());
  if (file.fail()) {
    RCLCPP_ERROR(logger_, "Failed to write engine data to file");
    return false;
  }

  file.close();
  RCLCPP_INFO(logger_, "Successfully wrote %zu bytes to engine file", engine_data.size());
  return true;
}

std::string BEVFormerInferenceEngine::getPrecisionString(PrecisionType precision)
{
  switch (precision) {
    case PrecisionType::FP16:
      return "FP16";
    case PrecisionType::FP32:
      return "FP32";
    default:
      return "Unknown";
  }
}

bool BEVFormerInferenceEngine::checkFileExists(const std::string & file_path)
{
  std::ifstream file(file_path);
  return file.good();
}

void BEVFormerInferenceEngine::getTensorShapes()
{
  // Get input shapes from the engine
  nvinfer1::Dims dims;

  // Image tensor shape
  dims = engine_->getTensorShape("image");
  input_shape_image_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    input_shape_image_[i] = dims.d[i];
  }

  // Previous BEV tensor shape
  dims = engine_->getTensorShape("prev_bev");
  input_shape_prev_bev_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    input_shape_prev_bev_[i] = dims.d[i];
  }

  // CAN bus tensor shape
  dims = engine_->getTensorShape("can_bus");
  input_shape_can_bus_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    input_shape_can_bus_[i] = dims.d[i];
  }

  // Lidar2img tensor shape
  dims = engine_->getTensorShape("lidar2img");
  input_shape_lidar2img_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    input_shape_lidar2img_[i] = dims.d[i];
  }

  // Use prev BEV tensor shape
  dims = engine_->getTensorShape("use_prev_bev");
  input_shape_use_prev_bev_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    input_shape_use_prev_bev_[i] = dims.d[i];
  }

  // Output tensor shapes
  dims = engine_->getTensorShape("bev_embed");
  output_shape_bev_embed_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    output_shape_bev_embed_[i] = dims.d[i];
  }

  dims = engine_->getTensorShape("outputs_classes");
  output_shape_outputs_classes_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    output_shape_outputs_classes_[i] = dims.d[i];
  }

  dims = engine_->getTensorShape("outputs_coords");
  output_shape_outputs_coords_.resize(dims.nbDims);
  for (int i = 0; i < dims.nbDims; i++) {
    output_shape_outputs_coords_[i] = dims.d[i];
  }
}

bool BEVFormerInferenceEngine::validateInputs(
  const cv::Mat & img_tensor, const std::vector<float> & prev_bev,
  const std::vector<float> & can_bus, const std::vector<float> & lidar2img_flat)
{
  // Calculate expected sizes based on tensor shapes
  size_t expected_img_size = 1;
  for (const auto & dim : input_shape_image_) {
    expected_img_size *= dim;
  }

  size_t expected_prev_bev_size = 1;
  for (const auto & dim : input_shape_prev_bev_) {
    expected_prev_bev_size *= dim;
  }

  size_t expected_can_bus_size = input_shape_can_bus_[0];
  size_t expected_lidar2img_size = 1;
  for (const auto & dim : input_shape_lidar2img_) {
    expected_lidar2img_size *= dim;
  }

  // Check image tensor
  if (img_tensor.total() != expected_img_size) {
    RCLCPP_ERROR(
      logger_, "Invalid image tensor size: %zu, expected: %zu", img_tensor.total(),
      expected_img_size);
    return false;
  }

  // Check data type of image tensor
  if (img_tensor.type() != CV_32F) {
    RCLCPP_ERROR(
      logger_, "Invalid image tensor data type: %d, expected: %d (CV_32F)", img_tensor.type(),
      CV_32F);
    return false;
  }

  // Check prev_bev
  if (prev_bev.size() != expected_prev_bev_size) {
    RCLCPP_ERROR(
      logger_, "Invalid prev_bev size: %zu, expected: %zu", prev_bev.size(),
      expected_prev_bev_size);
    return false;
  }

  // Check can_bus
  if (can_bus.size() != expected_can_bus_size) {
    RCLCPP_ERROR(
      logger_, "Invalid can_bus size: %zu, expected: %zu", can_bus.size(), expected_can_bus_size);
    return false;
  }

  // Check lidar2img
  if (lidar2img_flat.size() != expected_lidar2img_size) {
    RCLCPP_ERROR(
      logger_, "Invalid lidar2img size: %zu, expected: %zu", lidar2img_flat.size(),
      expected_lidar2img_size);
    return false;
  }

  // Check for NaN or inf values in inputs
  auto check_nan_inf = [this](const std::vector<float> & data, const std::string & name) -> bool {
    for (size_t i = 0; i < std::min<size_t>(100, data.size()); ++i) {
      if (std::isnan(data[i]) || std::isinf(data[i])) {
        RCLCPP_ERROR(logger_, "%s contains NaN or Inf at index %zu: %f", name.c_str(), i, data[i]);
        return false;
      }
    }
    return true;
  };

  if (
    !check_nan_inf(prev_bev, "prev_bev") || !check_nan_inf(can_bus, "can_bus") ||
    !check_nan_inf(lidar2img_flat, "lidar2img_flat")) {
    return false;
  }

  return true;
}

// RAII wrapper for CUDA memory management
class CudaMemoryManager
{
public:
  struct CudaBuffer
  {
    void * ptr;
    size_t size;
    explicit CudaBuffer(size_t s) : ptr(nullptr), size(s)
    {
      if (cudaMalloc(&ptr, size) != cudaSuccess) {
        throw std::runtime_error("Failed to allocate CUDA memory");
      }
    }
    ~CudaBuffer()
    {
      if (ptr) {
        cudaFree(ptr);
      }
    }
    CudaBuffer(const CudaBuffer &) = delete;
    CudaBuffer & operator=(const CudaBuffer &) = delete;
    CudaBuffer(CudaBuffer && other) noexcept : ptr(other.ptr), size(other.size)
    {
      other.ptr = nullptr;
    }
    CudaBuffer & operator=(CudaBuffer && other) noexcept
    {
      if (this != &other) {
        if (ptr) {
          cudaFree(ptr);
        }
        ptr = other.ptr;
        size = other.size;
        other.ptr = nullptr;
      }
      return *this;
    }
  };

  std::vector<std::unique_ptr<CudaBuffer>> buffers;

  void * allocate(size_t size)
  {
    auto buffer = std::make_unique<CudaBuffer>(size);
    void * ptr = buffer->ptr;
    buffers.push_back(std::move(buffer));
    return ptr;
  }
};

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>
BEVFormerInferenceEngine::runInference(
  const cv::Mat & img_tensor, const std::vector<float> & prev_bev, float use_prev_bev,
  const std::vector<float> & can_bus, const std::vector<float> & lidar2img_flat)
{
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Inference engine not initialized");
    return std::make_tuple(std::vector<float>(), std::vector<float>(), std::vector<float>());
  }

  // Validate inputs
  if (!validateInputs(img_tensor, prev_bev, can_bus, lidar2img_flat)) {
    return std::make_tuple(std::vector<float>(), std::vector<float>(), std::vector<float>());
  }

  // Convert image tensor to vector for comparison
  std::vector<float> img_tensor_vec(
    img_tensor.ptr<float>(), img_tensor.ptr<float>() + img_tensor.total());

  // Find tensor indices by name
  int img_tensor_idx = -1, prev_bev_idx = -1, use_prev_bev_idx = -1, can_bus_idx = -1,
      lidar2img_idx = -1;
  int bev_embed_idx = -1, outputs_classes_idx = -1, outputs_coords_idx = -1;

  for (int i = 0; i < engine_->getNbIOTensors(); i++) {
    const char * name = engine_->getIOTensorName(i);
    std::string tensor_name(name);
    if (tensor_name == "image") {
      img_tensor_idx = i;
    } else if (tensor_name == "prev_bev") {
      prev_bev_idx = i;
    } else if (tensor_name == "use_prev_bev") {
      use_prev_bev_idx = i;
    } else if (tensor_name == "can_bus") {
      can_bus_idx = i;
    } else if (tensor_name == "lidar2img") {
      lidar2img_idx = i;
    } else if (tensor_name == "bev_embed") {
      bev_embed_idx = i;
    } else if (tensor_name == "outputs_classes") {
      outputs_classes_idx = i;
    } else if (tensor_name == "outputs_coords") {
      outputs_coords_idx = i;
    }
  }

  if (
    img_tensor_idx == -1 || prev_bev_idx == -1 || use_prev_bev_idx == -1 || can_bus_idx == -1 ||
    lidar2img_idx == -1 || bev_embed_idx == -1 || outputs_classes_idx == -1 ||
    outputs_coords_idx == -1) {
    RCLCPP_ERROR(logger_, "Failed to find all required tensors in the engine");
    return std::make_tuple(std::vector<float>(), std::vector<float>(), std::vector<float>());
  }

  // Calculate output buffer sizes
  size_t bev_embed_size = 1;
  size_t outputs_classes_size = 1;
  size_t outputs_coords_size = 1;

  const nvinfer1::Dims bev_embed_dims =
    engine_->getTensorShape(engine_->getIOTensorName(bev_embed_idx));
  const nvinfer1::Dims outputs_classes_dims =
    engine_->getTensorShape(engine_->getIOTensorName(outputs_classes_idx));
  const nvinfer1::Dims outputs_coords_dims =
    engine_->getTensorShape(engine_->getIOTensorName(outputs_coords_idx));

  for (int i = 0; i < bev_embed_dims.nbDims; i++) {
    bev_embed_size *= bev_embed_dims.d[i];
  }
  for (int i = 0; i < outputs_classes_dims.nbDims; i++) {
    outputs_classes_size *= outputs_classes_dims.d[i];
  }
  for (int i = 0; i < outputs_coords_dims.nbDims; i++) {
    outputs_coords_size *= outputs_coords_dims.d[i];
  }

  // Create CUDA stream for inference
  cudaStream_t stream;
  CHECK_CUDA(cudaStreamCreate(&stream));

  cudaEvent_t start_event, end_event;
  CHECK_CUDA(cudaEventCreate(&start_event));
  CHECK_CUDA(cudaEventCreate(&end_event));

  std::vector<float> outputs_classes(outputs_classes_size);
  std::vector<float> outputs_coords(outputs_coords_size);
  std::vector<float> bev_embed_output(bev_embed_size);

  try {
    // Use RAII for CUDA memory management
    CudaMemoryManager cuda_mem;

    // Allocate device memory for inputs and outputs
    void * d_img_tensor = cuda_mem.allocate(sizeof(float) * img_tensor.total());
    void * d_prev_bev = cuda_mem.allocate(sizeof(float) * prev_bev.size());
    void * d_use_prev_bev = cuda_mem.allocate(sizeof(float));
    void * d_can_bus = cuda_mem.allocate(sizeof(float) * can_bus.size());
    void * d_lidar2img = cuda_mem.allocate(sizeof(float) * lidar2img_flat.size());
    void * d_bev_embed = cuda_mem.allocate(sizeof(float) * bev_embed_size);
    void * d_outputs_classes = cuda_mem.allocate(sizeof(float) * outputs_classes_size);
    void * d_outputs_coords = cuda_mem.allocate(sizeof(float) * outputs_coords_size);

    // Clear output buffers
    CHECK_CUDA(cudaMemsetAsync(d_bev_embed, 0, sizeof(float) * bev_embed_size, stream));
    CHECK_CUDA(cudaMemsetAsync(d_outputs_classes, 0, sizeof(float) * outputs_classes_size, stream));
    CHECK_CUDA(cudaMemsetAsync(d_outputs_coords, 0, sizeof(float) * outputs_coords_size, stream));

    // Copy input data to device
    CHECK_CUDA(cudaMemcpyAsync(
      d_img_tensor, img_tensor.ptr<float>(), sizeof(float) * img_tensor.total(),
      cudaMemcpyHostToDevice, stream));
    CHECK_CUDA(cudaMemcpyAsync(
      d_prev_bev, prev_bev.data(), sizeof(float) * prev_bev.size(), cudaMemcpyHostToDevice,
      stream));
    CHECK_CUDA(cudaMemcpyAsync(
      d_use_prev_bev, &use_prev_bev, sizeof(float), cudaMemcpyHostToDevice, stream));
    CHECK_CUDA(cudaMemcpyAsync(
      d_can_bus, can_bus.data(), sizeof(float) * can_bus.size(), cudaMemcpyHostToDevice, stream));
    CHECK_CUDA(cudaMemcpyAsync(
      d_lidar2img, lidar2img_flat.data(), sizeof(float) * lidar2img_flat.size(),
      cudaMemcpyHostToDevice, stream));

    CHECK_CUDA(cudaStreamSynchronize(stream));

    // Set tensor addresses
    context_->setTensorAddress(engine_->getIOTensorName(img_tensor_idx), d_img_tensor);
    context_->setTensorAddress(engine_->getIOTensorName(prev_bev_idx), d_prev_bev);
    context_->setTensorAddress(engine_->getIOTensorName(use_prev_bev_idx), d_use_prev_bev);
    context_->setTensorAddress(engine_->getIOTensorName(can_bus_idx), d_can_bus);
    context_->setTensorAddress(engine_->getIOTensorName(lidar2img_idx), d_lidar2img);
    context_->setTensorAddress(engine_->getIOTensorName(bev_embed_idx), d_bev_embed);
    context_->setTensorAddress(engine_->getIOTensorName(outputs_classes_idx), d_outputs_classes);
    context_->setTensorAddress(engine_->getIOTensorName(outputs_coords_idx), d_outputs_coords);

    // Execute inference
    CHECK_CUDA(cudaEventRecord(start_event, stream));
    CHECK_CUDA(cudaStreamSynchronize(stream));  // To ensure all data is transferred

    if (!context_->enqueueV3(stream)) {
      RCLCPP_ERROR(logger_, "Failed to run inference");
      throw std::runtime_error("TensorRT inference execution failed");
    }

    CHECK_CUDA(cudaStreamSynchronize(stream));
    CHECK_CUDA(cudaEventRecord(end_event, stream));
    CHECK_CUDA(cudaEventSynchronize(end_event));

    float inference_time_ms = 0.0f;
    CHECK_CUDA(cudaEventElapsedTime(&inference_time_ms, start_event, end_event));
    RCLCPP_INFO(logger_, "TRT Engine : %.3f ms", inference_time_ms);

    // Copy outputs from device
    CHECK_CUDA(cudaMemcpyAsync(
      outputs_classes.data(), d_outputs_classes, sizeof(float) * outputs_classes_size,
      cudaMemcpyDeviceToHost, stream));
    CHECK_CUDA(cudaMemcpyAsync(
      outputs_coords.data(), d_outputs_coords, sizeof(float) * outputs_coords_size,
      cudaMemcpyDeviceToHost, stream));
    CHECK_CUDA(cudaMemcpyAsync(
      bev_embed_output.data(), d_bev_embed, sizeof(float) * bev_embed_size, cudaMemcpyDeviceToHost,
      stream));

    CHECK_CUDA(cudaStreamSynchronize(stream));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during inference: %s", e.what());
    // Cleanup events and stream
    CHECK_CUDA(cudaEventDestroy(start_event));
    CHECK_CUDA(cudaEventDestroy(end_event));
    CHECK_CUDA(cudaStreamDestroy(stream));
    return std::make_tuple(std::vector<float>(), std::vector<float>(), std::vector<float>());
  }

  CHECK_CUDA(cudaEventDestroy(start_event));
  CHECK_CUDA(cudaEventDestroy(end_event));
  CHECK_CUDA(cudaStreamDestroy(stream));

  return std::make_tuple(outputs_classes, outputs_coords, bev_embed_output);
}

}  // namespace tensorrt_bevformer
}  // namespace autoware
