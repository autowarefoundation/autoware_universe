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

#ifndef AUTOWARE__TRAJECTORY_SELECTOR__TRAJECTORY_SELECTOR_NODE_HPP_
#define AUTOWARE__TRAJECTORY_SELECTOR__TRAJECTORY_SELECTOR_NODE_HPP_

#include "autoware/trajectory_validator/detail/validator_context.hpp"
#include "autoware/trajectory_validator/trajectory_validator_wrapper.hpp"
#include "autoware_trajectory_selector/autoware_trajectory_selector_param.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/trajectory_concatenator/trajectory_concatenator_wrapper.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

namespace autoware::trajectory_selector
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

/**
 * @brief Concatenates candidate trajectories from multiple planners, validates them, and
 * publishes the surviving set.
 */
class TrajectorySelectorNode : public autoware::agnocast_wrapper::Node
{
public:
  /**
   * @brief Constructs the node, declares parameters, and initialises all components.
   * @param node_options Node options.
   */
  explicit TrajectorySelectorNode(const rclcpp::NodeOptions & node_options);

private:
  /** @brief Creates all subscriptions. */
  void subscribers();

  /** @brief Creates all publishers and initialises the time keeper. */
  void publishers();

  /**
   * @brief Converts and stores the received lanelet map.
   * @param msg Binary lanelet map message.
   */
  void map_callback(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletMapBin) & msg);

  /**
   * @brief Forwards incoming candidate trajectories to the concatenator and trigger the
   * concatenation.
   * @param msg Incoming candidate trajectories message.
   * @warning must be in the same callback group as the timer callback as they both call
   * on_anchor_trajectories
   */
  void on_anchor_trajectories(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) & msg);
  /**
   * @brief Forwards incoming candidate trajectories to the concatenator.
   * @param msg Incoming candidate trajectories message.
   */
  void on_trajectories(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) & msg);

  /**
   * @brief Concatenates buffered trajectories, validates them, and publishes the result.
   */
  void concatenate_and_validate();

  /** @brief Collects the latest sensor data needed for validation; returns an error string if any
   * mandatory input is unavailable. */
  tl::expected<trajectory_validator::FilterContext, std::string> take_validator_data();

  /** @brief Update parameters */
  void update_parameters();

  /** @brief Update the fallback timer */
  void update_fallback_timer();

  std::unique_ptr<trajectory_concatenator::TrajectoryConcatenatorWrapper> concatenator_ptr_;
  std::unique_ptr<trajectory_validator::TrajectoryValidatorWrapper> validator_ptr_;
  AUTOWARE_TIMER_PTR timer_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  // Polling Subscribers
  autoware::agnocast_wrapper::polling::PollingSubscriber<Odometry>::SharedPtr sub_odometry_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<PredictedObjects>::SharedPtr sub_objects_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    sub_acceleration_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr sub_traffic_lights_;

  // Normal Subscribers
  AUTOWARE_SUBSCRIPTION_PTR(LaneletMapBin) sub_map_;
  AUTOWARE_SUBSCRIPTION_PTR(CandidateTrajectories) sub_trajectories_generative_;
  AUTOWARE_SUBSCRIPTION_PTR(CandidateTrajectories) sub_trajectories_backup_;

  // Publishers
  AUTOWARE_PUBLISHER_PTR(CandidateTrajectories) pub_trajectories_;
  AUTOWARE_PUBLISHER_PTR(autoware_utils_debug::ProcessingTimeDetail) pub_processing_time_detail_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  // Parameter interfaces
  selector::ParamListener selector_params_listener_{get_node_parameters_interface()};
  selector::Params selector_params_;
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR__TRAJECTORY_SELECTOR_NODE_HPP_
