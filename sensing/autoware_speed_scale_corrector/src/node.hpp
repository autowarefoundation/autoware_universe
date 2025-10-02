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

#include "autoware/speed_scale_corrector/speed_scale_estimator.hpp"
#include "autoware_utils_rclcpp/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

/**
 * @brief Speed scale corrector namespace
 */
namespace autoware::speed_scale_corrector
{
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_utils_rclcpp::InterProcessPollingSubscriber;
using autoware_utils_rclcpp::polling_policy::All;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

template <typename T, template <typename> class PollingPolicy>
using PollingSubscriber = InterProcessPollingSubscriber<T, PollingPolicy>;

/**
 * @brief ROS 2 node for speed scale correction
 *
 * This node estimates and corrects speed scale factors by comparing velocities
 * calculated from odometry with velocities reported by the vehicle's velocity sensors.
 * The estimation is performed only when operational constraints are satisfied to
 * ensure reliable results.
 */
class SpeedScaleCorrectorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param node_options ROS 2 node options
   */
  explicit SpeedScaleCorrectorNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Speed scale estimator instance
   */
  SpeedScaleEstimator speed_scale_estimator_;

  // Publishers
  /**
   * @brief Publisher for estimated speed scale factor
   */
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_estimated_speed_scale_factor_;

  /**
   * @brief Publisher for debug information
   */
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_info_;

  // Subscribers

  /**
   * @brief Polling subscriber for pose data
   */
  PollingSubscriber<PoseStamped, All>::SharedPtr sub_pose_;

  /**
   * @brief Polling subscriber for velocity reports
   */
  PollingSubscriber<VelocityReport, All>::SharedPtr sub_velocity_report_;

  /**
   * @brief Polling subscriber for IMU data
   */
  PollingSubscriber<Imu, All>::SharedPtr sub_imu_;

  // Timer
  /**
   * @brief Timer for periodic updates
   */
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  /**
   * @brief Timer callback
   */
  void on_timer();
};

}  // namespace autoware::speed_scale_corrector

#endif  // NODE_HPP_
