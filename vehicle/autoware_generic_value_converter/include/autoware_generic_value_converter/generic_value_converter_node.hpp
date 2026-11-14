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

#ifndef AUTOWARE_GENERIC_VALUE_CONVERTER__GENERIC_VALUE_CONVERTER_NODE_HPP_
#define AUTOWARE_GENERIC_VALUE_CONVERTER__GENERIC_VALUE_CONVERTER_NODE_HPP_

#include "autoware_generic_value_converter/value_map.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <string>

namespace autoware::generic_value_converter
{
using Control = autoware_control_msgs::msg::Control;
using tier4_debug_msgs::msg::Float64Stamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Odometry = nav_msgs::msg::Odometry;
using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;

class GenericValueConverterNode : public rclcpp::Node
{
public:
  explicit GenericValueConverterNode(const rclcpp::NodeOptions & node_options);

  //!< @brief topic publisher for converted output
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_output_value_;

  //!< @brief subscriber for control command
  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;

  // polling subscribers
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{this, "~/input/odometry"};

  std::unique_ptr<TwistStamped> current_twist_ptr_;  // [m/s]
  Odometry::ConstSharedPtr current_odometry_;
  Control::ConstSharedPtr control_cmd_ptr_;

  ValueMap value_map_;

  double max_value_;
  double min_value_;

  bool use_value_ff_;
  bool convert_value_cmd_;  //!< @brief convert acceleration to value or not

  double calculateValueFromMap(const double current_velocity, const double desired_acc);
  void onControlCmd(const Control::ConstSharedPtr msg);
  void publishOutputValue();

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace autoware::generic_value_converter

#endif  // AUTOWARE_GENERIC_VALUE_CONVERTER__GENERIC_VALUE_CONVERTER_NODE_HPP_
