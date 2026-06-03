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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/trajectory_concatenator/trajectory_concatenator_wrapper.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
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
class TrajectorySelectorNode : public rclcpp::Node
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
  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  /**
   * @brief Forwards incoming candidate trajectories to the concatenator.
   * @param msg Incoming candidate trajectories message.
   */
  void on_trajectories(const CandidateTrajectories::ConstSharedPtr msg);

  /** @brief Concatenates buffered trajectories, validates them, and publishes the result. */
  void on_timer();

  /** @brief Collects the latest sensor data needed for validation; returns an error string if any
   * mandatory input is unavailable. */
  tl::expected<trajectory_validator::FilterContext, std::string> take_validator_data();

  std::unique_ptr<trajectory_concatenator::TrajectoryConcatenatorWrapper> concatenator_ptr_;
  std::unique_ptr<trajectory_validator::TrajectoryValidatorWrapper> validator_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  // Polling Subscribers
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_acceleration_{this, "~/input/acceleration"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_lights_{this, "~/input/traffic_signals"};

  // Normal Subscribers
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_generative_;
  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_backup_;

  // Publishers
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    pub_processing_time_detail_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_selector

#endif  // AUTOWARE__TRAJECTORY_SELECTOR__TRAJECTORY_SELECTOR_NODE_HPP_
