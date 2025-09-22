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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSORS__SAMPLE_PROCESSOR_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSORS__SAMPLE_PROCESSOR_HPP_

#include "autoware/predicted_path_postprocessor/interface.hpp"

#include <rclcpp/parameter_value.hpp>

#include <string>

namespace autoware::predicted_path_postprocessor::processors
{
/**
 * NOTE: THIS CLASS IS JUST A SAMPLE FILTER!!
 */
class SampleProcessor final : public ProcessorInterface
{
public:
  SampleProcessor(rclcpp::Node * node_ptr, const std::string & processor_name)
  : ProcessorInterface(processor_name)
  {
    load_config(
      node_ptr, processor_name, [](rclcpp::Node * node_ptr, const std::string & processor_name) {
        node_ptr->declare_parameter<double>(processor_name + ".double_param", 0.0);
        node_ptr->declare_parameter<std::string>(processor_name + ".string_param", "default");
      });

    auto double_param = node_ptr->get_parameter(processor_name + ".double_param").as_double();
    auto string_param = node_ptr->get_parameter(processor_name + ".string_param").as_string();

    RCLCPP_INFO_STREAM(
      node_ptr->get_logger(), "SampleProcessor initialized!! ["
                                << processor_name << "]: double_param=" << double_param
                                << ", string_param=" << string_param);
  }

  autoware_perception_msgs::msg::PredictedObject process(
    const autoware_perception_msgs::msg::PredictedObject & target, const Context &) override
  {
    RCLCPP_INFO(rclcpp::get_logger("predicted_path_postprocessor"), "SampleProcessor processed!!");
    autoware_perception_msgs::msg::PredictedObject output(target);
    return output;
  }
};
}  // namespace autoware::predicted_path_postprocessor::processors
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSORS__SAMPLE_PROCESSOR_HPP_
