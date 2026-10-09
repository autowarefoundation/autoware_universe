// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/mpc_lateral_controller/ros_log_writer.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseStamped;

/// The node side of the lateral controller. It declares the parameters, builds the
/// controller from them, publishes what a cycle produced and writes what it reported.
class MpcLateralControllerNode : public trajectory_follower::LateralControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit MpcLateralControllerNode(
    rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater);
  ~MpcLateralControllerNode() = default;

  void set_steering_offset(double offset) override { m_controller->set_steering_offset(offset); }

  /// Read the parameters the controller needs from the node, declaring them on it.
  static MpcLateralControllerConfig createConfig(rclcpp::Node & node);

private:
  bool isReady(const trajectory_follower::InputData & input_data) override;

  trajectory_follower::LateralOutput run(
    trajectory_follower::InputData const & input_data) override;

  void publish(const MpcLateralControllerResult & result);

  void setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /// Declare the parameters of the optimisation, which can be changed while the node runs.
  static MPCParam declareMPCparameters(rclcpp::Node & node);

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  RosLogWriter writer_;

  std::unique_ptr<MpcLateralController> m_controller;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj_frenet;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_resampled_reference_traj;
  rclcpp::Publisher<PoseStamped>::SharedPtr m_pub_nearest_pose;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_nearest_segment_traj;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_nearest_info;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_debug_values;
  rclcpp::Publisher<Float32Stamped>::SharedPtr m_pub_steer_offset;
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_
