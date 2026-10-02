// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__ROS_LOG_WRITER_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__ROS_LOG_WRITER_HPP_

#include "autoware/mpc_lateral_controller/log_writer.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string_view>
#include <utility>

// Nothing in this file is specific to this package, apart from the namespace below. To use
// the file in another package, copy it, change the namespace and the include path above,
// and let the include guard hook fix the guard.

namespace autoware::motion::control::mpc_lateral_controller
{
/// Writes each line to a logger of ROS, and drops a line whose level the logger does not
/// take. The waiting time comes from the clock the node was built with.
class RosLogWriter : public LogWriter
{
public:
  RosLogWriter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : logger_(std::move(logger)), clock_(std::move(clock))
  {
  }

  bool shouldWrite(Level level, double after_s, double & last_sent_s) const override;
  void write(Level level, std::string_view line) const override;

private:
  static int severityOf(Level level);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__ROS_LOG_WRITER_HPP_
