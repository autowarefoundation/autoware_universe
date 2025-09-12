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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__COMPOSABLE_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__COMPOSABLE_HPP_

#include "autoware/predicted_path_postprocessor/builder.hpp"
#include "autoware/predicted_path_postprocessor/interface.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor
{
/**
 * @brief A class composing multiple processors to process predicted objects.
 */
class ComposableProcessor
{
public:
  /**
   * @brief Constructor for ComposableProcessor.
   *
   * @param node_ptr Pointer to the ROS node.
   * @param processor_names Vector of processor names.
   */
  explicit ComposableProcessor(
    rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names)
  {
    processors_ = build_processors(node_ptr, processor_names);
  }

  /**
   * @brief Process predicted objects using a composite filter.
   *
   * @param objects Shared pointer to the input predicted objects.
   * @param context Context information for processing.
   * @return Processed predicted objects.
   */
  autoware_perception_msgs::msg::PredictedObjects process(
    const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
    const Context & context)
  {
    std::vector<autoware_perception_msgs::msg::PredictedObject> results;
    for (const auto & object : objects->objects) {
      // copy to execute recursively
      auto target = object;

      for (const auto & processor : processors_) {
        target = processor->process(target, context);
      }

      results.push_back(std::move(target));
    }

    return autoware_perception_msgs::build<autoware_perception_msgs::msg::PredictedObjects>()
      .header(objects->header)
      .objects(std::move(results));
  }

private:
  std::vector<ProcessorInterface::UniquePtr> processors_;  //!< Set of processors.
};
}  // namespace autoware::predicted_path_postprocessor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__COMPOSABLE_HPP_
