// Copyright 2026 The Autoware Contributors
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

#ifndef NODE__DRIVING_MODE_MAPPING_HPP_
#define NODE__DRIVING_MODE_MAPPING_HPP_

#include "types/forward.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/driving_mode_flag.hpp>

#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

class DrivingModeMapping
{
public:
  DrivingModeMapping(rclcpp::Node & node, const Graph & graph);
  void update(const rclcpp::Time & stamp) const;

private:
  using DrivingModeFlag = tier4_system_msgs::msg::DrivingModeFlag;
  rclcpp::Publisher<DrivingModeFlag>::SharedPtr pub_available_;
  rclcpp::Publisher<DrivingModeFlag>::SharedPtr pub_continuable_;

  std::unordered_map<uint32_t, BaseUnit *> mode_to_unit_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // NODE__DRIVING_MODE_MAPPING_HPP_
