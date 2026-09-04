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

#include "scene_understanding_tensorrt/tensorrt_engine.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace scene_understanding_tensorrt
{

void TensorRTEngine::Logger::log(Severity severity, const char * msg) noexcept
{
  if (severity <= Severity::kWARNING) {
    std::cout << "[TensorRT] " << msg << std::endl;
  }
}

TensorRTEngine::TensorRTEngine() : logger_(std::make_unique<Logger>())
{
  // Initialize CUDA buffers to nullptr
  for (int i = 0; i < 24; ++i) {
    buffers_[i] = nullptr;
  }
}

TensorRTEngine::~TensorRTEngine()
{
  // Free CUDA buffers
  for (int i = 0; i < 24; ++i) {
    if (buffers_[i] != nullptr) {
      cudaFree(buffers_[i]);
    }
  }
}

bool TensorRTEngine::initialize(
  const std::string & onnx_model_path, int max_batch_size, bool fp16, bool int8)
{
  // Create builder
  auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(*logger_));
  if (!builder) {
    std::cerr << "Failed to create TensorRT builder" << std::endl;
    return false;
  }

  // Create network
  const auto explicitBatch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network =
    std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
  if (!network) {
    std::cerr << "Failed to create TensorRT network" << std::endl;
    return false;
  }

  // Create ONNX parser
  auto parser =
    std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, *logger_));
  if (!parser) {
    std::cerr << "Failed to create ONNX parser" << std::endl;
    return false;
  }

  // Parse ONNX model
  if (!parser->parseFromFile(
        onnx_model_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    std::cerr << "Failed to parse ONNX model: " << onnx_model_path << std::endl;
    return false;
  }

  // Build engine configuration
  auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    std::cerr << "Failed to create builder config" << std::endl;
    return false;
  }

  // Set workspace size (1GB)
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30);

  // Set precision flags
  if (fp16 && builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    std::cout << "Enabled FP16 precision" << std::endl;
  }
  if (int8 && builder->platformHasFastInt8()) {
    config->setFlag(nvinfer1::BuilderFlag::kINT8);
    std::cout << "Enabled INT8 precision" << std::endl;
  }

  // Create optimization profile for dynamic shapes
  auto profile = builder->createOptimizationProfile();

  // Define input tensor dimensions (assuming "input" is the input tensor name)
  // Set min, opt, max dimensions for the input tensor
  profile->setDimensions(
    "input", nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims4(1, INPUT_C, INPUT_H, INPUT_W));
  profile->setDimensions(
    "input", nvinfer1::OptProfileSelector::kOPT,
    nvinfer1::Dims4(max_batch_size, INPUT_C, INPUT_H, INPUT_W));
  profile->setDimensions(
    "input", nvinfer1::OptProfileSelector::kMAX,
    nvinfer1::Dims4(max_batch_size, INPUT_C, INPUT_H, INPUT_W));

  // Add the profile to the config
  config->addOptimizationProfile(profile);

  // Build engine
  auto plan =
    std::unique_ptr<nvinfer1::IHostMemory>(builder->buildSerializedNetwork(*network, *config));
  if (!plan) {
    std::cerr << "Failed to build TensorRT engine" << std::endl;
    return false;
  }

  // Create runtime
  runtime_ = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(*logger_));
  if (!runtime_) {
    std::cerr << "Failed to create TensorRT runtime" << std::endl;
    return false;
  }

  // Deserialize engine
  engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(plan->data(), plan->size()));
  if (!engine_) {
    std::cerr << "Failed to deserialize TensorRT engine" << std::endl;
    return false;
  }

  // Create execution context
  context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    std::cerr << "Failed to create execution context" << std::endl;
    return false;
  }

  // Set the optimization profile for the execution context
  context_->setOptimizationProfileAsync(0, 0);

  // Allocate CUDA buffers
  int nb_bindings = engine_->getNbIOTensors();
  std::cout << "Number of bindings: " << nb_bindings << std::endl;

  // Input buffer
  auto input_dims = engine_->getTensorShape("input");
  input_size_ = max_batch_size * INPUT_C * INPUT_H * INPUT_W * sizeof(float);
  cudaMalloc(&buffers_[0], input_size_);

  // Output buffers - following the order from metadata
  const std::vector<std::pair<std::string, size_t>> outputs = {
    {"exclusive_Intersections", 3},
    {"exclusive_Lane Position", 4},
    {"exclusive_Lighting", 2},
    {"exclusive_Road Geometry", 3},
    {"exclusive_Road Surface", 2},
    {"exclusive_Road Type", 3},
    {"exclusive_Traffic Density", 4},
    {"exclusive_Visibility", 3},
    {"exclusive_Weather", 1},
    {"binary_Road Features_Construction zone", 1},
    {"binary_Road Features_Crosswalk", 1},
    {"binary_Road Features_Merge area", 1},
    {"binary_Road Features_Railway crossing", 1},
    {"binary_Special Vehicles_Emergency vehicle", 1},
    {"binary_Special Vehicles_Heavy vehicle", 1},
    {"binary_Special Vehicles_Public transport", 1},
    {"binary_Traffic Control_No traffic controls", 1},
    {"binary_Traffic Control_Stop sign", 1},
    {"binary_Traffic Control_Traffic light", 1},
    {"binary_Traffic Control_Yield sign", 1},
    {"binary_Vulnerable Road Users_Animals", 1},
    {"binary_Vulnerable Road Users_Cyclists", 1},
    {"binary_Vulnerable Road Users_Pedestrians", 1}};

  for (size_t i = 0; i < outputs.size(); ++i) {
    size_t output_size = max_batch_size * outputs[i].second * sizeof(float);
    cudaMalloc(&buffers_[i + 1], output_size);
    output_sizes_.push_back(outputs[i].second);

    // Set tensor addresses
    context_->setTensorAddress(outputs[i].first.c_str(), buffers_[i + 1]);
  }

  // Set input tensor address
  context_->setTensorAddress("input", buffers_[0]);

  std::cout << "TensorRT engine initialized successfully" << std::endl;
  return true;
}

cv::Mat TensorRTEngine::preprocessImage(const cv::Mat & image)
{
  // Resize image to 224x224
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(INPUT_W, INPUT_H));

  // Convert BGR to RGB
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

  // Convert to float and normalize
  cv::Mat normalized;
  rgb.convertTo(normalized, CV_32FC3, 1.0 / 255.0);

  // Apply mean and std normalization
  std::vector<cv::Mat> channels(3);
  cv::split(normalized, channels);

  for (int i = 0; i < 3; ++i) {
    channels[i] = (channels[i] - MEAN[i]) / STD[i];
  }

  cv::Mat result;
  cv::merge(channels, result);

  return result;
}

bool TensorRTEngine::infer(const cv::Mat & image, SceneUnderstandingResult & result)
{
  if (!context_) {
    std::cerr << "TensorRT engine not initialized" << std::endl;
    return false;
  }

  // Preprocess image
  cv::Mat preprocessed = preprocessImage(image);

  // Convert HWC to CHW format
  std::vector<float> input_data(INPUT_C * INPUT_H * INPUT_W);
  for (int c = 0; c < INPUT_C; ++c) {
    for (int h = 0; h < INPUT_H; ++h) {
      for (int w = 0; w < INPUT_W; ++w) {
        input_data[c * INPUT_H * INPUT_W + h * INPUT_W + w] = preprocessed.at<cv::Vec3f>(h, w)[c];
      }
    }
  }

  // Copy input to GPU
  cudaMemcpy(buffers_[0], input_data.data(), input_size_, cudaMemcpyHostToDevice);

  // Set input dimensions for this inference
  context_->setInputShape("input", nvinfer1::Dims4(1, INPUT_C, INPUT_H, INPUT_W));

  // Run inference
  auto start = std::chrono::high_resolution_clock::now();
  bool status = context_->executeV2(buffers_);
  auto end = std::chrono::high_resolution_clock::now();

  if (!status) {
    std::cerr << "TensorRT inference failed" << std::endl;
    return false;
  }

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Inference time: " << duration.count() << " ms" << std::endl;

  // Copy outputs from GPU and process
  std::vector<std::vector<float>> outputs;
  for (size_t i = 0; i < output_sizes_.size(); ++i) {
    std::vector<float> output(output_sizes_[i]);
    cudaMemcpy(
      output.data(), buffers_[i + 1], output_sizes_[i] * sizeof(float), cudaMemcpyDeviceToHost);
    outputs.push_back(output);
  }

  // Process exclusive categories
  result.intersection = static_cast<Intersection>(
    std::distance(outputs[0].begin(), std::max_element(outputs[0].begin(), outputs[0].end())));
  result.lane_position = static_cast<LanePosition>(
    std::distance(outputs[1].begin(), std::max_element(outputs[1].begin(), outputs[1].end())));
  result.lighting = static_cast<Lighting>(
    std::distance(outputs[2].begin(), std::max_element(outputs[2].begin(), outputs[2].end())));
  result.road_geometry = static_cast<RoadGeometry>(
    std::distance(outputs[3].begin(), std::max_element(outputs[3].begin(), outputs[3].end())));
  result.road_surface = static_cast<RoadSurface>(
    std::distance(outputs[4].begin(), std::max_element(outputs[4].begin(), outputs[4].end())));
  result.road_type = static_cast<RoadType>(
    std::distance(outputs[5].begin(), std::max_element(outputs[5].begin(), outputs[5].end())));
  result.traffic_density = static_cast<TrafficDensity>(
    std::distance(outputs[6].begin(), std::max_element(outputs[6].begin(), outputs[6].end())));
  result.visibility = static_cast<Visibility>(
    std::distance(outputs[7].begin(), std::max_element(outputs[7].begin(), outputs[7].end())));
  result.weather = static_cast<Weather>(
    std::distance(outputs[8].begin(), std::max_element(outputs[8].begin(), outputs[8].end())));

  // Process binary categories (apply sigmoid)
  result.road_features_construction_zone = sigmoid(outputs[9][0]);
  result.road_features_crosswalk = sigmoid(outputs[10][0]);
  result.road_features_merge_area = sigmoid(outputs[11][0]);
  result.road_features_railway_crossing = sigmoid(outputs[12][0]);
  result.special_vehicles_emergency_vehicle = sigmoid(outputs[13][0]);
  result.special_vehicles_heavy_vehicle = sigmoid(outputs[14][0]);
  result.special_vehicles_public_transport = sigmoid(outputs[15][0]);
  result.traffic_control_no_traffic_controls = sigmoid(outputs[16][0]);
  result.traffic_control_stop_sign = sigmoid(outputs[17][0]);
  result.traffic_control_traffic_light = sigmoid(outputs[18][0]);
  result.traffic_control_yield_sign = sigmoid(outputs[19][0]);
  result.vulnerable_road_users_animals = sigmoid(outputs[20][0]);
  result.vulnerable_road_users_cyclists = sigmoid(outputs[21][0]);
  result.vulnerable_road_users_pedestrians = sigmoid(outputs[22][0]);

  return true;
}

// String conversion functions
std::string TensorRTEngine::intersectionToString(Intersection value)
{
  switch (value) {
    case Intersection::T_JUNCTION:
      return "T-junction";
    case Intersection::NO_INTERSECTION:
      return "No intersection";
    case Intersection::FOUR_WAY_INTERSECTION:
      return "4-way intersection";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::lanePositionToString(LanePosition value)
{
  switch (value) {
    case LanePosition::SPECIAL_LANE:
      return "Special lane";
    case LanePosition::CENTER_LANE:
      return "Center lane";
    case LanePosition::LEFT_LANE:
      return "Left lane";
    case LanePosition::RIGHT_LANE:
      return "Right lane";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::lightingToString(Lighting value)
{
  switch (value) {
    case Lighting::DAYTIME:
      return "Daytime";
    case Lighting::GLARE_CONDITIONS:
      return "Glare conditions";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::roadGeometryToString(RoadGeometry value)
{
  switch (value) {
    case RoadGeometry::STRAIGHT:
      return "Straight";
    case RoadGeometry::CURVED:
      return "Curved";
    case RoadGeometry::SLOPE:
      return "Slope";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::roadSurfaceToString(RoadSurface value)
{
  switch (value) {
    case RoadSurface::DRY:
      return "Dry";
    case RoadSurface::DEGRADED:
      return "Degraded";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::roadTypeToString(RoadType value)
{
  switch (value) {
    case RoadType::URBAN_STREET:
      return "Urban street";
    case RoadType::RESIDENTIAL_AREA:
      return "Residential area";
    case RoadType::BRIDGE:
      return "Bridge";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::trafficDensityToString(TrafficDensity value)
{
  switch (value) {
    case TrafficDensity::EMPTY:
      return "Empty";
    case TrafficDensity::LIGHT:
      return "Light";
    case TrafficDensity::MODERATE:
      return "Moderate";
    case TrafficDensity::HEAVY:
      return "Heavy";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::visibilityToString(Visibility value)
{
  switch (value) {
    case Visibility::CLEAR:
      return "Clear";
    case Visibility::PARTIALLY_OBSTRUCTED:
      return "Partially obstructed";
    case Visibility::SEVERELY_LIMITED:
      return "Severely limited";
    default:
      return "Unknown";
  }
}

std::string TensorRTEngine::weatherToString(Weather value)
{
  switch (value) {
    case Weather::CLOUDY:
      return "Cloudy";
    default:
      return "Unknown";
  }
}

}  // namespace scene_understanding_tensorrt
