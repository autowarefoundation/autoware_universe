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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__BUILDER_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__BUILDER_HPP_

#include "autoware/predicted_path_postprocessor/processor/interface.hpp"
#include "autoware/predicted_path_postprocessor/processor/refine_by_speed.hpp"
#include "autoware/predicted_path_postprocessor/processor/refine_penetration_by_static_objects.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
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
    if (name == "refine_by_speed") {
      outputs.push_back(std::make_unique<RefineBySpeed>(node_ptr, name));
    } else if (name == "refine_penetration_by_static_objects") {
      outputs.push_back(std::make_unique<RefinePenetrationByStaticObjects>(node_ptr, name));
    } else {
      RCLCPP_ERROR_STREAM(node_ptr->get_logger(), "Unknown processor: " << name);
      continue;
    }
  }
  return outputs;
}
}  // namespace autoware::predicted_path_postprocessor::processor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__BUILDER_HPP_
