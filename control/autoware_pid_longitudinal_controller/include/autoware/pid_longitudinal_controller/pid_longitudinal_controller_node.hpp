// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
using visualization_msgs::msg::MarkerArray;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \class PidLongitudinalControllerNode
/// \brief the rclcpp::Node wrapper around PidLongitudinalController: parameter
/// declaration/reconfiguration, publishers, logging, and diagnostics.
class PidLongitudinalControllerNode : public trajectory_follower::LongitudinalControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit PidLongitudinalControllerNode(
    rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater);

private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  // ros variables
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    m_pub_slope;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    m_pub_debug;
  rclcpp::Publisher<MarkerArray>::SharedPtr m_pub_virtual_wall_marker;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  PidLongitudinalControllerConfig config;
  PidLongitudinalController controller_;

  // for diagnostics
  ControlState control_state_{ControlState::STOPPED};

  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_{};  // Diagnostic updater for publishing diagnostic data.
  void setupDiagnosticUpdater();
  void checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  bool isReady(const trajectory_follower::InputData & input_data) override;

  /**
   * @brief compute control command, and publish periodically
   */
  trajectory_follower::LongitudinalOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief publish debug data
   * @param [in] result output of the core control logic for this cycle
   * @param [in] current_time time captured once per control cycle in run()
   */
  void publishDebugData(
    const PidLongitudinalControllerResult & result, const rclcpp::Time & current_time);

  /**
   * @brief publish the virtual wall marker created during this cycle, if any
   */
  void publishVirtualWallMarker(const PidLongitudinalControllerResult & result);

  /**
   * @brief emit the critical logs raised by the core logic during this cycle
   */
  void emitLogs(const PidLongitudinalControllerResult & result);
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_NODE_HPP_
