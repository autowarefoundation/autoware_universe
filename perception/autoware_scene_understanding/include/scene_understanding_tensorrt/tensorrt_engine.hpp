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

#ifndef SCENE_UNDERSTANDING_TENSORRT__TENSORRT_ENGINE_HPP_
#define SCENE_UNDERSTANDING_TENSORRT__TENSORRT_ENGINE_HPP_

#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace scene_understanding_tensorrt
{

// Enum definitions for exclusive categories
enum class Intersection { T_JUNCTION = 0, NO_INTERSECTION = 1, FOUR_WAY_INTERSECTION = 2 };

enum class LanePosition { SPECIAL_LANE = 0, CENTER_LANE = 1, LEFT_LANE = 2, RIGHT_LANE = 3 };

enum class Lighting { DAYTIME = 0, GLARE_CONDITIONS = 1 };

enum class RoadGeometry { STRAIGHT = 0, CURVED = 1, SLOPE = 2 };

enum class RoadSurface { DRY = 0, DEGRADED = 1 };

enum class RoadType { URBAN_STREET = 0, RESIDENTIAL_AREA = 1, BRIDGE = 2 };

enum class TrafficDensity { EMPTY = 0, LIGHT = 1, MODERATE = 2, HEAVY = 3 };

enum class Visibility { CLEAR = 0, PARTIALLY_OBSTRUCTED = 1, SEVERELY_LIMITED = 2 };

enum class Weather { CLOUDY = 0 };

// Result structure
struct SceneUnderstandingResult
{
  // Exclusive categories
  Intersection intersection;
  LanePosition lane_position;
  Lighting lighting;
  RoadGeometry road_geometry;
  RoadSurface road_surface;
  RoadType road_type;
  TrafficDensity traffic_density;
  Visibility visibility;
  Weather weather;

  // Binary categories (probabilities)
  float road_features_construction_zone;
  float road_features_crosswalk;
  float road_features_merge_area;
  float road_features_railway_crossing;
  float special_vehicles_emergency_vehicle;
  float special_vehicles_heavy_vehicle;
  float special_vehicles_public_transport;
  float traffic_control_no_traffic_controls;
  float traffic_control_stop_sign;
  float traffic_control_traffic_light;
  float traffic_control_yield_sign;
  float vulnerable_road_users_animals;
  float vulnerable_road_users_cyclists;
  float vulnerable_road_users_pedestrians;
};

class TensorRTEngine
{
public:
  TensorRTEngine();
  ~TensorRTEngine();

  /**
   * @brief Initialize TensorRT engine from ONNX model
   * @param onnx_model_path Path to ONNX model file
   * @param max_batch_size Maximum batch size (default: 1)
   * @param fp16 Enable FP16 precision if supported (default: true)
   * @param int8 Enable INT8 precision if supported (default: false)
   * @return true if initialization successful
   */
  bool initialize(
    const std::string & onnx_model_path, int max_batch_size = 1, bool fp16 = true,
    bool int8 = false);

  /**
   * @brief Run inference on input image
   * @param image Input image (BGR format from OpenCV)
   * @param result Output scene understanding result
   * @return true if inference successful
   */
  bool infer(const cv::Mat & image, SceneUnderstandingResult & result);

  /**
   * @brief Convert enum values to string representations
   */
  static std::string intersectionToString(Intersection value);
  static std::string lanePositionToString(LanePosition value);
  static std::string lightingToString(Lighting value);
  static std::string roadGeometryToString(RoadGeometry value);
  static std::string roadSurfaceToString(RoadSurface value);
  static std::string roadTypeToString(RoadType value);
  static std::string trafficDensityToString(TrafficDensity value);
  static std::string visibilityToString(Visibility value);
  static std::string weatherToString(Weather value);

private:
  // TensorRT Logger
  class Logger : public nvinfer1::ILogger
  {
    void log(Severity severity, const char * msg) noexcept override;
  };

  // Image preprocessing
  cv::Mat preprocessImage(const cv::Mat & image);

  // Apply sigmoid activation
  float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

  // TensorRT components
  std::unique_ptr<Logger> logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  // CUDA buffers
  void * buffers_[24];  // 9 exclusive + 14 binary + 1 input = 24
  size_t input_size_;
  std::vector<size_t> output_sizes_;

  // Model parameters (hardcoded from metadata)
  static constexpr int INPUT_H = 224;
  static constexpr int INPUT_W = 224;
  static constexpr int INPUT_C = 3;
  static constexpr std::array<float, 3> MEAN = {0.485f, 0.456f, 0.406f};
  static constexpr std::array<float, 3> STD = {0.229f, 0.224f, 0.225f};
};

}  // namespace scene_understanding_tensorrt

#endif  // SCENE_UNDERSTANDING_TENSORRT__TENSORRT_ENGINE_HPP_
