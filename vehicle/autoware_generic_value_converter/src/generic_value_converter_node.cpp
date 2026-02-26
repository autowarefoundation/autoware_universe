//
// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "autoware_generic_value_converter/generic_value_converter_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

namespace autoware::generic_value_converter
{

GenericValueConverterNode::GenericValueConverterNode(const rclcpp::NodeOptions & node_options)
: Node("generic_value_converter", node_options)
{
  using std::placeholders::_1;

  // ROS parameters
  const std::string csv_path_value_map = declare_parameter<std::string>("csv_path_value_map", "");
  max_value_ = declare_parameter<double>("max_value", 3.0);
  min_value_ = declare_parameter<double>("min_value", -5.0);
  convert_value_cmd_ = declare_parameter<bool>("convert_value_cmd", true);
  use_value_ff_ = declare_parameter<bool>("use_value_ff", true);

  // Load map from CSV
  if (!value_map_.readValueMapFromCSV(csv_path_value_map)) {
    RCLCPP_ERROR(get_logger(), "Cannot read value map from CSV: %s", csv_path_value_map.c_str());
    // Continue with default/empty map
  } else {
    RCLCPP_INFO(get_logger(), "Successfully loaded value map from: %s", csv_path_value_map.c_str());
  }

  // Publisher
  pub_output_value_ = create_publisher<Float64Stamped>("~/output/value", rclcpp::QoS{1});

  // Subscriber
  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", rclcpp::QoS{1},
    std::bind(&GenericValueConverterNode::onControlCmd, this, _1));

  current_twist_ptr_ = std::make_unique<TwistStamped>();

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  RCLCPP_INFO(get_logger(), "GenericValueConverter initialized successfully");
}

void GenericValueConverterNode::onControlCmd(const Control::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;

  // Get current odometry
  current_odometry_ = sub_odometry_.take_data();

  if (!current_odometry_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Odometry is not available, cannot convert command");
    return;
  }

  // Update current twist
  current_twist_ptr_->header = current_odometry_->header;
  current_twist_ptr_->twist = current_odometry_->twist.twist;

  publishOutputValue();
}

double GenericValueConverterNode::calculateValueFromMap(
  const double current_velocity, const double desired_acc)
{
  double output_value = 0.0;

  if (convert_value_cmd_ && use_value_ff_) {
    // Use map to convert acceleration to value
    if (!value_map_.getValue(desired_acc, current_velocity, output_value)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to get value from map for acc=%.3f, vel=%.3f",
        desired_acc, current_velocity);
      output_value = 0.0;
    }

    // Clamp output
    output_value = std::max(min_value_, std::min(max_value_, output_value));
  } else {
    // Direct passthrough (or default behavior)
    output_value = desired_acc;
  }

  return output_value;
}

void GenericValueConverterNode::publishOutputValue()
{
  if (!control_cmd_ptr_ || !current_twist_ptr_) {
    return;
  }

  const double current_velocity = current_twist_ptr_->twist.linear.x;
  const double desired_acc = control_cmd_ptr_->longitudinal.acceleration;

  double output_value = calculateValueFromMap(current_velocity, desired_acc);

  // Publish output
  Float64Stamped output_msg;
  output_msg.stamp = this->now();
  output_msg.data = output_value;

  pub_output_value_->publish(output_msg);

  RCLCPP_DEBUG(
    get_logger(), "Converted acc=%.3f at vel=%.3f to value=%.3f", desired_acc, current_velocity,
    output_value);
}

}  // namespace autoware::generic_value_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::generic_value_converter::GenericValueConverterNode)
