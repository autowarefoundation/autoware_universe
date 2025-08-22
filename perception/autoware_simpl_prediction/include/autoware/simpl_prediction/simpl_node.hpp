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

#ifndef AUTOWARE__SIMPL_PREDICTION__SIMPL_NODE_HPP_
#define AUTOWARE__SIMPL_PREDICTION__SIMPL_NODE_HPP_

#include "autoware/simpl_prediction/archetype/agent.hpp"
#include "autoware/simpl_prediction/conversion/lanelet.hpp"
#include "autoware/simpl_prediction/processing/postprocessor.hpp"
#include "autoware/simpl_prediction/processing/preprocessor.hpp"
#include "autoware/simpl_prediction/trt_simpl.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simpl_prediction
{
/**
 * @brief A ROS 2 node class for SIMPL.
 */
class SimplNode : public rclcpp::Node
{
public:
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
  using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
  using TrackedObject = autoware_perception_msgs::msg::TrackedObject;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using Odometry = nav_msgs::msg::Odometry;
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;
  using Shape = autoware_perception_msgs::msg::Shape;

  template <typename T>
  using InterProcessPollingSubscriber = autoware_utils::InterProcessPollingSubscriber<T>;

  /**
   * @brief Construct a new SimplNode object.
   *
   * @param options Node options.
   */
  explicit SimplNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Main callback which performs predicting objects future trajectories and publish them.
   *
   * @param objects_msg Tracked objects message.
   */
  void callback(const TrackedObjects::ConstSharedPtr objects_msg);

  /**
   * @brief Subscribe lanelet map and convert to set of polylines.
   *
   * @param map_msg Lanelet map binary message.
   */
  void on_map(const LaneletMapBin::ConstSharedPtr map_msg);

  /**
   * @brief Subscribe the latest ego vehicle state as a `AgentState`.
   *
   * @return std::optional<archetype::AgentState> Return current ego state or `std::nullopt` if
   * failed to subscribe.
   */
  std::optional<archetype::AgentState> subscribe_ego();

  /**
   * @brief Update history container.
   *
   * @param objects_msg Tracked objects.
   * @return std::vector<archetype::AgentHistory> Return updated history container.
   */
  std::vector<archetype::AgentHistory> update_history(
    const TrackedObjects::ConstSharedPtr objects_msg);

  //////////////////////// member variables ////////////////////////
  //!< Tracked objects subscription.
  rclcpp::Subscription<TrackedObjects>::SharedPtr objects_subscription_;

  //!< Lanelet subscription.
  rclcpp::Subscription<LaneletMapBin>::SharedPtr lanelet_subscription_;

  //!< Ego vehicle odometry subscription.
  InterProcessPollingSubscriber<Odometry> odometry_subscription_{
    this, "/localization/kinematic_state"};

  //!< Predicted objects publisher.
  rclcpp::Publisher<PredictedObjects>::SharedPtr objects_publisher_;

  //!< Pointer to lanelet converter.
  std::unique_ptr<conversion::LaneletConverter> lanelet_converter_ptr_;

  //!< Hasmap of agent ID and its history.
  std::unordered_map<std::string, archetype::AgentHistory> history_map_;

  //!< Hashmap of agent ID and tracked object message.
  std::unordered_map<std::string, TrackedObject> tracked_object_map_;

  //!< SIMPL detector.
  std::unique_ptr<TrtSimpl> detector_;

  //!< Pre-processor.
  std::unique_ptr<processing::PreProcessor> preprocessor_;

  //!< Post-processor.
  std::unique_ptr<processing::PostProcessor> postprocessor_;

  //!< Debugger for processing time.
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stopwatch_ptr_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> processing_time_publisher_;

  //!< Number of past timestamps.
  int num_past_;
};
}  // namespace autoware::simpl_prediction
#endif  // AUTOWARE__SIMPL_PREDICTION__SIMPL_NODE_HPP_
