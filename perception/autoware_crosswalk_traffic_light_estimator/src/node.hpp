// Copyright 2022-2025 TIER IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "crosswalk_traffic_light_estimator.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <memory>

namespace autoware::crosswalk_traffic_light_estimator
{

using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_utils::DebugPublisher;
using autoware_utils::StopWatch;

class CrosswalkTrafficLightEstimatorNode : public rclcpp::Node
{
public:
  explicit CrosswalkTrafficLightEstimatorNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_light_array_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_traffic_light_array_;

  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_traffic_light_array(const TrafficSignalArray::ConstSharedPtr msg);

  CrosswalkTrafficLightEstimator estimator_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;

  // Debug
  std::shared_ptr<DebugPublisher> pub_processing_time_;
};

}  // namespace autoware::crosswalk_traffic_light_estimator

#endif  // NODE_HPP_
