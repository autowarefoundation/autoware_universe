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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__BUILDER_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__BUILDER_HPP_

#include "autoware/predicted_path_postprocessor/interface.hpp"
#include "autoware/predicted_path_postprocessor/processors/refine_by_speed.hpp"
#include "autoware/predicted_path_postprocessor/processors/sample_processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor
{
/**
 * @brief Build processors from their names.
 *
 * @param node_ptr Pointer to the ROS node.
 * @param processor_names Vector of processor names.
 * @return Vector of unique pointers to ProcessorInterface objects.
 */
inline std::vector<ProcessorInterface::UniquePtr> build_processors(
  rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names)
{
  std::vector<ProcessorInterface::UniquePtr> outputs;
  for (const auto & name : processor_names) {
    if (name == "sample_processor") {
      outputs.emplace_back(std::make_unique<processors::SampleProcessor>(node_ptr, name));
    } else if (name == "refine_by_speed") {
      outputs.emplace_back(std::make_unique<processors::RefineBySpeed>(node_ptr, name));
    } else {
      RCLCPP_ERROR_STREAM(node_ptr->get_logger(), "Unknown processor name: " << name);
      continue;
    }
  }
  return outputs;
}
}  // namespace autoware::predicted_path_postprocessor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__BUILDER_HPP_
