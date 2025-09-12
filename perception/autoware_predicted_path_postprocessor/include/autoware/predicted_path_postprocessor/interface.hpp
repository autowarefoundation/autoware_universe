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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__INTERFACE_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__INTERFACE_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor
{
/**
 * @brief A context which processors can access.
 */
class Context
{
public:
  Context() = default;

  /**
   * @brief Get the reference to the current set of predicted objects.
   *
   * @return The current set of predicted objects.
   */
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects() const
  {
    return objects_;
  }

  /**
   * @brief Update the context with a new set of predicted objects.
   *
   * @param new_objects The new set of predicted objects.
   */
  void update(autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr new_objects)
  {
    objects_ = std::move(new_objects);
  }

private:
  // NOTE: In the future, we may want to add more context information here, such as lanelet,
  // traffic light, etc.
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_{nullptr};
};

/**
 * @brief Interface for processors that process a single predicted object.
 */
class ProcessorInterface
{
public:
  using UniquePtr = std::unique_ptr<ProcessorInterface>;
  using DeclareParametersFunc = std::function<void(rclcpp::Node *, const std::string &)>;

  /**
   * @brief Load configuration parameters for the processor.
   *
   * @param node_ptr The node to load parameters from.
   * @param processor_name The name of the processor.
   * @param declare_parameters The function to declare parameters.
   */
  void load_config(
    rclcpp::Node * node_ptr, const std::string & processor_name,
    const DeclareParametersFunc & declare_parameters)
  {
    declare_parameters(node_ptr, processor_name);

    auto parameters_client = rclcpp::SyncParametersClient(node_ptr);

    const auto package_share_dir =
      ament_index_cpp::get_package_share_directory("autoware_predicted_path_postprocessor");
    auto yaml_filepath = package_share_dir + "/config/" + processor_name + ".param.yaml";

    const auto results = parameters_client.load_parameters(std::move(yaml_filepath));

    for (const auto & result : results) {
      if (!result.successful) {
        RCLCPP_ERROR_STREAM(
          node_ptr->get_logger(),
          "Failed to load parameters for " << processor_name << ": " << result.reason);
      }
    }
  }

  /**
   * @brief Process a single predicted object.
   *
   * @param target The predicted object to process.
   * @param context The context in which the object is processed.
   * @return The processed predicted object.
   */
  virtual autoware_perception_msgs::msg::PredictedObject process(
    const autoware_perception_msgs::msg::PredictedObject & target, const Context & context) = 0;
};
}  // namespace autoware::predicted_path_postprocessor

#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__INTERFACE_HPP_
