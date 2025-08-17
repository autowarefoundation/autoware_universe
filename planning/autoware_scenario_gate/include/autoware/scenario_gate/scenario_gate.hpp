// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__SCENARIO_GATE__SCENARIO_GATE_HPP_
#define AUTOWARE__SCENARIO_GATE__SCENARIO_GATE_HPP_

#include <autoware/scenario_selector_base.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace autoware::scenario_gate
{

class ScenarioGateNode : public rclcpp::Node
{
public:
  explicit ScenarioGateNode(const rclcpp::NodeOptions & options);
  ~ScenarioGateNode() override;

private:
  // Callbacks
  void onSelectorScenario(const autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr msg);
  void onSelectorTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  // Subscriptions
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr
    sub_selector_scenario_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_selector_traj_;

  // Publishers
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr pub_scenario_;

  std::string selector_info_;
  std::unique_ptr<pluginlib::ClassLoader<autoware::scenario_selector::ScenarioSelectorBase>>
    loader_;
  std::shared_ptr<autoware::scenario_selector::ScenarioSelectorBase> selector_plugin_;

  // 讓 executor 能加 node 進去 spin
  rclcpp::Node * selector_node_raw_{nullptr};
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> selector_executor_;
  std::thread selector_spin_thread_;
};

}  // namespace autoware::scenario_gate

#endif  // AUTOWARE__SCENARIO_GATE__SCENARIO_GATE_HPP_