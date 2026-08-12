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

// cspell:ignore BEVFORMER

#ifndef BEVFORMER_INFERENCE_ENGINE_HPP_
#define BEVFORMER_INFERENCE_ENGINE_HPP_

#include "bevformer_data_loader.hpp"
#include "ros_utils.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

// Define precision type enum
enum class PrecisionType { FP16, FP32 };

class BEVFormerInferenceEngine
{
public:
  explicit BEVFormerInferenceEngine(const rclcpp::Logger & logger);
  ~BEVFormerInferenceEngine();

  /**
   * @brief Initialize the TensorRT engine
   *
   * @param engine_file Path to the TensorRT engine file
   * @param plugin_path Path to the TensorRT plugin library
   * @return bool True if initialization is successful
   */
  bool initialize(const std::string & engine_file, const std::string & plugin_path);

  bool buildEngineFromOnnx(
    const std::string & onnx_file, const std::string & engine_file, const std::string & plugin_path,
    int workspace_size = 4096, PrecisionType precision = PrecisionType::FP16);

  /**
   * @brief Run inference on the input tensor
   *
   * @param img_tensor Image tensor in the format [1, 6, 3, H, W]
   * @param prev_bev Previous BEV features
   * @param use_prev_bev Whether to use previous BEV features
   * @param can_bus CAN bus data
   * @param lidar2img_matrices Lidar to image transformation matrices
   * @return std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>
   *         Tuple of (outputs_classes, outputs_coords, bev_embed)
   */
  std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> runInference(
    const cv::Mat & img_tensor, const std::vector<float> & prev_bev, float use_prev_bev,
    const std::vector<float> & can_bus, const std::vector<float> & lidar2img_flat);

  /**
   * @brief Validate the inference inputs
   *
   * @param img_tensor Image tensor
   * @param prev_bev Previous BEV features
   * @param can_bus CAN bus data
   * @param lidar2img_flat Flattened lidar2img matrices
   * @return bool True if all inputs are valid
   */
  bool validateInputs(
    const cv::Mat & img_tensor, const std::vector<float> & prev_bev,
    const std::vector<float> & can_bus, const std::vector<float> & lidar2img_flat);

  /**
   * @brief Check if the engine is initialized
   *
   * @return bool True if initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get the input tensor shapes
   */
  std::vector<int64_t> getInputImageShape() const { return input_shape_image_; }
  std::vector<int64_t> getInputPrevBevShape() const { return input_shape_prev_bev_; }
  std::vector<int64_t> getInputCanBusShape() const { return input_shape_can_bus_; }
  std::vector<int64_t> getInputLidar2imgShape() const { return input_shape_lidar2img_; }
  std::vector<int64_t> getOutputBevEmbedShape() const { return output_shape_bev_embed_; }

private:
  // TensorRT engine components
  nvinfer1::IRuntime * runtime_ = nullptr;
  nvinfer1::ICudaEngine * engine_ = nullptr;
  nvinfer1::IExecutionContext * context_ = nullptr;

  rclcpp::Logger logger_;

  bool initialized_ = false;

  // Helper function to get a string representation of precision type
  static std::string getPrecisionString(PrecisionType precision);

  void * plugin_handle_ = nullptr;

  std::string findPluginLibrary(const std::string & plugin_path);
  bool loadPlugins(const std::string & plugin_path);

  // Input and output tensor shapes
  std::vector<int64_t> input_shape_image_;
  std::vector<int64_t> input_shape_prev_bev_;
  std::vector<int64_t> input_shape_can_bus_;
  std::vector<int64_t> input_shape_lidar2img_;
  std::vector<int64_t> input_shape_use_prev_bev_;
  std::vector<int64_t> output_shape_bev_embed_;
  std::vector<int64_t> output_shape_outputs_classes_;
  std::vector<int64_t> output_shape_outputs_coords_;

  // Helper function to read the engine file
  std::vector<char> readEngineFile(const std::string & engine_file);

  // Helper function to get tensor shapes from the engine
  void getTensorShapes();

  bool saveEngineToDisk(const std::vector<char> & engine_data, const std::string & engine_file);
  static bool checkFileExists(const std::string & file_path);
};

}  // namespace tensorrt_bevformer
}  // namespace autoware

#endif  // BEVFORMER_INFERENCE_ENGINE_HPP_
