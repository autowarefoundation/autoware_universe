// Copyright 2025 TIER IV.
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

#ifndef AUTOWARE__TENSORRT_VAD__ROS_VAD_LOGGER_HPP_
#define AUTOWARE__TENSORRT_VAD__ROS_VAD_LOGGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::tensorrt_vad
{

// VadLogger interface definition
class VadLogger
{
public:
  virtual ~VadLogger() = default;

  // Define each log level method as pure virtual function
  virtual void debug(const std::string & message) = 0;
  virtual void info(const std::string & message) = 0;
  virtual void warn(const std::string & message) = 0;
  virtual void error(const std::string & message) = 0;
};

/**
 * @brief VadLogger implementation for ROS 2
 */
class RosVadLogger : public VadLogger
{
private:
  rclcpp::Logger logger_;

public:
  explicit RosVadLogger(rclcpp::Node::SharedPtr node) : logger_(node->get_logger()) {}

  explicit RosVadLogger(const rclcpp::Logger & logger) : logger_(logger) {}

  void debug(const std::string & message) override
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000, "%s", message.c_str());
  }

  void info(const std::string & message) override
  {
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000, "%s", message.c_str());
  }

  void warn(const std::string & message) override
  {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000, "%s", message.c_str());
  }

  void error(const std::string & message) override
  {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000, "%s", message.c_str());
  }
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__ROS_VAD_LOGGER_HPP_
