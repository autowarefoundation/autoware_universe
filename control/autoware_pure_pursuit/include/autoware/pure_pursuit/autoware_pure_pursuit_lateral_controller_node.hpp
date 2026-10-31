// Copyright 2020-2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_NODE_HPP_

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp"
#include "autoware/pure_pursuit/ros_log_writer.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float64_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include <memory>
#include <string>

using autoware::motion::control::trajectory_follower::LateralControllerInterface;

namespace autoware::pure_pursuit
{

class PurePursuitLateralControllerNode : public LateralControllerInterface
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit PurePursuitLateralControllerNode(rclcpp::Node & node);

  /// Read the parameters and the vehicle information that the controller needs.
  static PurePursuitLateralControllerConfig createConfig(rclcpp::Node & node);

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  RosLogWriter writer_;

  // Debug Publisher
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    pub_debug_values_;
  // Predicted Trajectory publish
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_predicted_trajectory_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_ms_;

  std::unique_ptr<PurePursuitLateralController> controller_;

  /**
   * @brief compute control command for path follow with a constant control period
   */
  tl::expected<LateralOutput, std::string> run(
    const InputData & input_data, const rclcpp::Time & stamp) override;
};

}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_NODE_HPP_
