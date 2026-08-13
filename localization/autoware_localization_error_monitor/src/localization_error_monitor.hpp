// Copyright 2020 Tier IV, Inc.
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

#ifndef LOCALIZATION_ERROR_MONITOR_HPP_
#define LOCALIZATION_ERROR_MONITOR_HPP_

#include "autoware/localization_util/covariance_ellipse.hpp"

#include <Eigen/Dense>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace autoware::localization_error_monitor
{
class LocalizationErrorMonitor : public autoware::agnocast_wrapper::Node
{
private:
  AUTOWARE_SUBSCRIPTION_PTR(nav_msgs::msg::Odometry) odom_sub_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::Marker) ellipse_marker_pub_;

  AUTOWARE_TIMER_PTR timer_;

  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  std::unique_ptr<
    autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>>
    diagnostics_error_monitor_;

  double scale_;
  double error_ellipse_size_;
  double warn_ellipse_size_;
  double error_ellipse_size_lateral_direction_;
  double warn_ellipse_size_lateral_direction_;
  autoware::localization_util::Ellipse ellipse_;

  void on_odom(AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) input_msg);

public:
  explicit LocalizationErrorMonitor(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::localization_error_monitor

#endif  // LOCALIZATION_ERROR_MONITOR_HPP_
