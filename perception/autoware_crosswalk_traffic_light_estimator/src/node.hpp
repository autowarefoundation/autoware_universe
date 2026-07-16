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

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <memory>

namespace autoware::crosswalk_traffic_light_estimator
{

using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_utils::StopWatch;

class CrosswalkTrafficLightEstimatorNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit CrosswalkTrafficLightEstimatorNode(const rclcpp::NodeOptions & options);

private:
  AUTOWARE_SUBSCRIPTION_PTR(LaneletMapBin) sub_map_;
  AUTOWARE_SUBSCRIPTION_PTR(TrafficSignalArray) sub_traffic_light_array_;
  AUTOWARE_PUBLISHER_PTR(TrafficSignalArray) pub_traffic_light_array_;

  void on_map(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletMapBin) & msg);
  void on_traffic_light_array(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(TrafficSignalArray) & msg);

  CrosswalkTrafficLightEstimator estimator_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;

  // Debug
  std::shared_ptr<autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    pub_processing_time_;
};

}  // namespace autoware::crosswalk_traffic_light_estimator

#endif  // NODE_HPP_
