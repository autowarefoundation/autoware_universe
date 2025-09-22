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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__DEBUG__INTERMEDIATE_PUBLISHER_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__DEBUG__INTERMEDIATE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::predicted_path_postprocessor::debug
{
/**
 * @brief Publishes intermediate processing results for debugging purposes.
 */
class IntermediatePublisher
{
public:
  /**
   * @brief Constructor for IntermediatePublisher.
   * @param node_ptr Pointer to the ROS 2 node.
   * @param processor_names Vector of processor names.
   */
  IntermediatePublisher(rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names);

  /**
   * @brief Publishes intermediate processing results for debugging purposes.
   * @param intermediates Map of predicted objects produced by each processor.
   */
  void publish(
    const std::unordered_map<std::string, autoware_perception_msgs::msg::PredictedObjects> &
      intermediates);

private:
  std::unordered_map<
    std::string, rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr>
    publishers_;  //!< Map of publishers for each processor.
};
}  // namespace autoware::predicted_path_postprocessor::debug
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__DEBUG__INTERMEDIATE_PUBLISHER_HPP_
