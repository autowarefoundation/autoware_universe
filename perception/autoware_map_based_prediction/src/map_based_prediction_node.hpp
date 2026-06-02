// Copyright 2021 TIER IV, Inc.
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

#ifndef MAP_BASED_PREDICTION_NODE_HPP_
#define MAP_BASED_PREDICTION_NODE_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/map_based_prediction_node/callbacks.hpp"
#include "autoware/map_based_prediction/map_based_prediction_node/diagnostics.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace autoware::map_based_prediction
{

class MapBasedPredictionNode : public rclcpp::Node
{
public:
  explicit MapBasedPredictionNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;

  NodeState state_;
  std::unique_ptr<Diagnostics> diagnostics_;
  std::unique_ptr<MapCallback> map_callback_;
  std::unique_ptr<ObjectsCallback> objects_callback_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
};

}  // namespace autoware::map_based_prediction

#endif  // MAP_BASED_PREDICTION_NODE_HPP_
