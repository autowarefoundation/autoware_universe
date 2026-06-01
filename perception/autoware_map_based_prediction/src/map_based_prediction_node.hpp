// Copyright 2021 Tier IV, Inc.
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
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"
#include "autoware/map_based_prediction/predictor_vru/predictor_vru.hpp"

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
  // ROS subscriptions
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;

  // Predictors (kept here for onParam access)
  std::shared_ptr<PredictorVehicle> predictor_vehicle_;
  std::shared_ptr<PredictorVru> predictor_vru_;
  std::shared_ptr<PathGenerator> path_generator_;

  // Time keeper (shared with predictors and callbacks)
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  // Parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);

  // Callbacks class (owns the callback logic and related state)
  std::unique_ptr<Callbacks> callbacks_;
};

}  // namespace autoware::map_based_prediction

#endif  // MAP_BASED_PREDICTION_NODE_HPP_
