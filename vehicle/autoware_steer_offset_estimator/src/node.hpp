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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware/steer_offset_estimator/steer_offset_estimator.hpp"
#include "autoware_utils_rclcpp/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief Steer offset estimator namespace
 */
namespace autoware::steer_offset_estimator
{
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;

template <typename T>
using PollingSubscriber = autoware_utils_rclcpp::InterProcessPollingSubscriber<
  T, autoware_utils_rclcpp::polling_policy::All>;

/**
 * @brief Load parameters from ROS parameter server
 * @param node Pointer to ROS node for parameter access
 * @return SteerOffsetEstimatorParameters Loaded parameters with default values
 */
SteerOffsetEstimatorParameters load_parameters(rclcpp::Node * node);

/**
 * @brief ROS 2 node for steer offset estimation
 *
 * This node estimates steering wheel offset by comparing expected steering
 * angles calculated from vehicle motion with actual steering sensor readings.
 * The estimation is performed only when operational constraints are satisfied to
 * ensure reliable results.
 */
class SteerOffsetEstimatorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param node_options ROS 2 node options
   */
  explicit SteerOffsetEstimatorNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Steer offset estimator instance
   */
  SteerOffsetEstimator estimator_;

  // Subscribers
  /**
   * @brief Subscriber for pose
   */
  PollingSubscriber<PoseStamped>::SharedPtr sub_pose_;

  /**
   * @brief Subscriber for steering report
   */
  PollingSubscriber<SteeringReport>::SharedPtr sub_steer_;

  // Publishers
  /**
   * @brief Publisher for estimated steer offset
   */
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steer_offset_;

  /**
   * @brief Publisher for steer offset covariance
   */
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_steer_offset_covariance_;

  /**
   * @brief Publisher for steer offset estimation result
   */
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_info_;
  // Timer
  /**
   * @brief Timer for periodic processing
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Timer callback for processing pose and steering updates
   */
  void on_timer();
};

}  // namespace autoware::steer_offset_estimator

#endif  // NODE_HPP_
