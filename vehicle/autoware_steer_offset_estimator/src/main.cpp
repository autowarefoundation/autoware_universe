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

#include "autoware/steer_offset_estimator/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <glog/logging.h>

#include <memory>

int main(int argc, char ** argv)
{
  google::InitGoogleLogging("steer_offset_estimator");
  google::InstallFailureSignalHandler();

  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware::steer_offset_estimator::SteerOffsetEstimatorNode>(
    rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
