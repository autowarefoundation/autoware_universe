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

#include "autoware/cuda_pointcloud_preprocessor/dag/filters/cropbox_filter.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <stdexcept>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void CropBoxFilter::initialize(const std::map<std::string, std::any> & parameters)
{
  // Parse crop_boxes parameter
  if (parameters.find("crop_boxes") != parameters.end()) {
    try {
      // crop_boxes is a vector of maps, each containing min_x, min_y, min_z, max_x, max_y, max_z
      const auto & crop_boxes_any = parameters.at("crop_boxes");

      // Handle vector<map<string, double>>
      if (crop_boxes_any.type() == typeid(std::vector<std::map<std::string, any>>)) {
        const auto & boxes = std::any_cast<std::vector<std::map<std::string, any>>>(crop_boxes_any);

        for (const auto & box_map : boxes) {
          CropBoxParameters box;

          if (box_map.find("min_x") == box_map.end()) {
            throw std::runtime_error("CropBoxFilter: crop_boxes entry missing 'min_x'");
          }
          if (box_map.find("min_y") == box_map.end()) {
            throw std::runtime_error("CropBoxFilter: crop_boxes entry missing 'min_y'");
          }
          if (box_map.find("min_z") == box_map.end()) {
            throw std::runtime_error("CropBoxFilter: crop_boxes entry missing 'min_z'");
          }
          if (box_map.find("max_x") == box_map.end()) {
            throw std::runtime_error("CropBoxFilter: crop_boxes entry missing 'max_x'");
          }
          if (box_map.find("max_y") == box_map.end()) {
            throw std::runtime_error("CropBoxFilter: crop_boxes entry missing 'max_y'");
          }
          if (box_map.find("max_z") == box_map.end()) {
            throw std::runtime_error("CropBoxFilter: crop_boxes entry missing 'max_z'");
          }

          box.min_x = std::any_cast<double>(box_map.at("min_x"));
          box.min_y = std::any_cast<double>(box_map.at("min_y"));
          box.min_z = std::any_cast<double>(box_map.at("min_z"));
          box.max_x = std::any_cast<double>(box_map.at("max_x"));
          box.max_y = std::any_cast<double>(box_map.at("max_y"));
          box.max_z = std::any_cast<double>(box_map.at("max_z"));

          crop_boxes_.push_back(box);
        }
      } else if (crop_boxes_any.type() == typeid(std::vector<any>)) {
        const auto & vec = std::any_cast<std::vector<any>>(crop_boxes_any);
        if (!vec.empty()) {
          throw std::runtime_error("CropBoxFilter: crop_boxes as vector<double> should be empty");
        }
        // Empty crop_boxes is valid - no cropping will be applied
      }
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "CropBoxFilter: Failed to parse 'crop_boxes' parameter: " + std::string(e.what()));
    }
  }

  // Parse optional enabled parameter
  if (parameters.find("inplace") != parameters.end()) {
    try {
      inplace_ = std::any_cast<bool>(parameters.at("inplace"));
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "RingOutlierFilter: Failed to parse 'inplace' as bool: " + std::string(e.what()));
    }
  }
  // Empty crop_boxes is valid - no cropping will be applied
}

void CropBoxFilter::process(
  const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context, const std::vector<std::string> & output_names)
{
  // Get the first (and only) input - name can be anything
  if (inputs.processing_states.empty()) {
    throw std::runtime_error("CropBoxFilter: No input provided");
  }
  auto input_state = inputs.processing_states.begin()->second;

  // Apply crop box filter with filter-specific parameters
  // NOTE: This only updates masks, doesn't modify device_data (zero-copy!)
  context.shared_preprocessor->applyCropBoxPublic(*input_state, crop_boxes_, inplace_);

  // Pass through state (actual point filtering happens in finalize step)
  // Use the output name specified in YAML configuration
  outputs[output_names[0]] = std::static_pointer_cast<void>(input_state);
}

FilterMetadata CropBoxFilter::getMetadata() const
{
  return {
    .filter_type = "CropBoxFilter",
    .required_inputs = {"*"},  // Accepts any single input
    .optional_inputs = {},
    .outputs = {"filtered"},
    .input_types = {{"*", "cuda_blackboard::CudaPointCloud2"}},
    .output_types = {{"filtered", "cuda_blackboard::CudaPointCloud2"}}};
}

bool CropBoxFilter::validateInputs(const TypedInputs & inputs) const
{
  return !inputs.processing_states.empty();
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
