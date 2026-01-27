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

#include "../src/scene.hpp"

#include <autoware/behavior_velocity_planner/test_utils.hpp>
#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::behavior_velocity_planner
{

class TrafficLightModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup common objects
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    logger_ = rclcpp::get_logger("test_logger");
    time_keeper_ = nullptr;
    planning_factor_interface_ = nullptr;

    // Default params
    planner_param_.stop_margin = 0.0;
    planner_param_.tl_state_timeout = 1.0;
    planner_param_.yellow_lamp_period = 3.0;
    planner_param_.yellow_light_stop_velocity = 1.0;
    planner_param_.stop_time_hysteresis = 0.0;
    planner_param_.enable_pass_judge = false;
    planner_param_.enable_arrow_aware_yellow_passing = true;  // Key param for this test
    planner_param_.max_behind_dist_to_stop_for_restart_suppression = 0.0;
    planner_param_.min_behind_dist_to_stop_for_restart_suppression = 0.0;
    planner_param_.v2i_use_remaining_time = false;
    planner_param_.v2i_last_time_allowed_to_pass = 0.0;
    planner_param_.v2i_velocity_threshold = 0.0;
    planner_param_.v2i_required_time_to_departure = 0.0;

    // Dummy lanelet data
    lanelet::Points3d points;
    points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
    points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
    lanelet::LineString3d ls(lanelet::utils::getId(), points);

    traffic_light_ptr_ = lanelet::TrafficLight::make(12345, {}, {ls});
    const auto & traffic_light_reg_elem = *traffic_light_ptr_;
    lanelet::Lanelet lane(100);
    lanelet::LineString3d stop_line(200);

    // Create module
    module_ = std::make_shared<TrafficLightModule>(
      100, traffic_light_reg_elem, lane, stop_line, true /* is_turn_lane */,
      true /* has_static_arrow */, planner_param_, logger_, clock_, time_keeper_,
      planning_factor_interface_);

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    // Setup planner data
    auto node = std::make_shared<rclcpp::Node>("test_node", get_node_options());
    planner_data_ = std::make_shared<PlannerData>(*node);
    planner_data_->current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
    planner_data_->current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
    planner_data_->current_acceleration =
      std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    planner_data_->is_simulation = false;
    setPlannerData(planner_data_);
  }

  rclcpp::NodeOptions get_node_options()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"wheel_radius", 0.3},
       {"wheel_width", 0.2},
       {"wheel_base", 2.7},
       {"wheel_tread", 1.5},
       {"front_overhang", 1.0},
       {"rear_overhang", 1.0},
       {"left_overhang", 0.5},
       {"right_overhang", 0.5},
       {"vehicle_height", 1.5},
       {"max_steer_angle", 0.5},
       {"max_accel", 1.0},
       {"min_accel", -1.0},
       {"max_jerk", 1.0},
       {"min_jerk", -1.0},
       {"system_delay", 0.0},
       {"delay_response_time", 0.0},
       {"max_stop_acceleration_threshold", 1.0},
       {"max_stop_jerk_threshold", 1.0}});
    return options;
  }

  std::shared_ptr<TrafficLightModule> module_;
  std::shared_ptr<PlannerData> planner_data_;
  TrafficLightModule::PlannerParam planner_param_;
  rclcpp::Logger logger_{rclcpp::get_logger("test")};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  lanelet::TrafficLight::Ptr traffic_light_ptr_;

  // Helper to set traffic signal
  void setTrafficSignal(const autoware_perception_msgs::msg::TrafficLightGroup & signal)
  {
    TrafficSignalStamped signal_stamped;
    signal_stamped.stamp = clock_->now();
    signal_stamped.signal = signal;
    planner_data_->traffic_light_id_map_raw_[12345] = signal_stamped;
  }

  autoware_perception_msgs::msg::TrafficLightElement createElement(
    uint8_t color, uint8_t shape, uint8_t status)
  {
    autoware_perception_msgs::msg::TrafficLightElement element;
    element.color = color;
    element.shape = shape;
    element.status = status;
    element.confidence = 1.0;
    return element;
  }

  using YellowState = TrafficLightModule::YellowState;

  YellowState getYellowTransitionState() { return module_->yellow_transition_state_; }

  void setPlannerData(const std::shared_ptr<PlannerData> & planner_data)
  {
    module_->setPlannerData(planner_data);
  }

  bool callIsStopSignal() { return module_->isStopSignal(); }

  void verifyTransition(
    const autoware_perception_msgs::msg::TrafficLightGroup & first_signal,
    const autoware_perception_msgs::msg::TrafficLightGroup & second_signal,
    YellowState expected_state, bool expected_stop)
  {
    setTrafficSignal(first_signal);
    callIsStopSignal();

    setTrafficSignal(second_signal);
    bool stop = callIsStopSignal();

    EXPECT_EQ(getYellowTransitionState(), expected_state);
    if (expected_stop) {
      EXPECT_TRUE(stop);
    } else {
      EXPECT_FALSE(stop);
    }
  }
};

TEST_F(TrafficLightModuleTest, TransitionToYellowFromGreen)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // 1. Set Green state
  autoware_perception_msgs::msg::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));

  // 2. Transition to Yellow
  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // Should PASS (return false for stop signal) because it's a turn lane with static arrow and came
  // from Green
  verifyTransition(green_signal, yellow_signal, YellowState::kFromGreen, false);
}

TEST_F(TrafficLightModuleTest, TransitionToYellowFromRed)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // 1. Set Red state (or Red Arrow)
  autoware_perception_msgs::msg::TrafficLightGroup red_signal;
  red_signal.elements.push_back(createElement(Element::RED, Element::CIRCLE, Element::SOLID_ON));

  // 2. Transition to Yellow
  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // Should STOP (return true for stop signal) because it did NOT come from Green
  verifyTransition(red_signal, yellow_signal, YellowState::kFromNonGreen, true);
}

TEST_F(TrafficLightModuleTest, StateResetWhenYellowEnds)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // 1. Establish Yellow From Green state
  autoware_perception_msgs::msg::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(green_signal);
  callIsStopSignal();

  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(yellow_signal);
  callIsStopSignal();

  EXPECT_EQ(getYellowTransitionState(), YellowState::kFromGreen);

  // 2. Transition to Red
  autoware_perception_msgs::msg::TrafficLightGroup red_signal;
  red_signal.elements.push_back(createElement(Element::RED, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(red_signal);

  callIsStopSignal();

  // Verify state reset
  EXPECT_EQ(getYellowTransitionState(), YellowState::kNotYellow);
}

TEST_F(TrafficLightModuleTest, NotTurnLane)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // Re-create module with is_turn_lane = false
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
  lanelet::LineString3d ls(lanelet::utils::getId(), points);

  auto traffic_light_ptr = lanelet::TrafficLight::make(12345, {}, {ls});
  const auto & traffic_light_reg_elem = *traffic_light_ptr;
  lanelet::Lanelet lane(100);
  lanelet::LineString3d stop_line(200);
  module_ = std::make_shared<TrafficLightModule>(
    100, traffic_light_reg_elem, lane, stop_line, false /* is_turn_lane */,
    true /* has_static_arrow */, planner_param_, logger_, clock_, time_keeper_,
    planning_factor_interface_);

  auto node = std::make_shared<rclcpp::Node>("test_node", get_node_options());

  planner_data_ = std::make_shared<PlannerData>(*node);
  planner_data_->current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
  planner_data_->current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  planner_data_->current_acceleration =
    std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  planner_data_->is_simulation = false;
  setPlannerData(planner_data_);

  // 1. Green -> Yellow
  autoware_perception_msgs::msg::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));

  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // State is tracked (kFromGreen) but should STOP because it is not a turn lane
  verifyTransition(green_signal, yellow_signal, YellowState::kFromGreen, true);
}

TEST_F(TrafficLightModuleTest, NoStaticArrow)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // Re-create module with has_static_arrow = false
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
  lanelet::LineString3d ls(lanelet::utils::getId(), points);

  auto traffic_light_ptr = lanelet::TrafficLight::make(12345, {}, {ls});
  const auto & traffic_light_reg_elem = *traffic_light_ptr;
  lanelet::Lanelet lane(100);
  lanelet::LineString3d stop_line(200);
  module_ = std::make_shared<TrafficLightModule>(
    100, traffic_light_reg_elem, lane, stop_line, true /* is_turn_lane */,
    false /* has_static_arrow */, planner_param_, logger_, clock_, time_keeper_,
    planning_factor_interface_);

  auto node = std::make_shared<rclcpp::Node>("test_node", get_node_options());
  planner_data_ = std::make_shared<PlannerData>(*node);
  planner_data_->current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
  planner_data_->current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  planner_data_->current_acceleration =
    std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  planner_data_->is_simulation = false;
  setPlannerData(planner_data_);

  // 1. Green -> Yellow
  autoware_perception_msgs::msg::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));

  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));

  // State is tracked (kFromGreen) but should STOP because there is no static arrow
  verifyTransition(green_signal, yellow_signal, YellowState::kFromGreen, true);
}

TEST_F(TrafficLightModuleTest, UnknownToYellow)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // 1. Directly set Yellow state (No previous Green)
  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(yellow_signal);

  // Call isStopSignal
  bool stop = callIsStopSignal();

  // Verify internal state remains kNotYellow (default safe state)
  EXPECT_EQ(getYellowTransitionState(), YellowState::kNotYellow);

  // Should STOP because the origin of yellow is unknown (assumed unsafe)
  EXPECT_TRUE(stop);
}

TEST_F(TrafficLightModuleTest, FeatureDisabled)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;

  // Re-create module with enable_arrow_aware_yellow_passing = false
  lanelet::Points3d points;
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0));
  points.push_back(lanelet::Point3d(lanelet::utils::getId(), 1, 0, 0));
  lanelet::LineString3d ls(lanelet::utils::getId(), points);

  auto traffic_light_ptr = lanelet::TrafficLight::make(12345, {}, {ls});
  const auto & traffic_light_reg_elem = *traffic_light_ptr;
  lanelet::Lanelet lane(100);
  lanelet::LineString3d stop_line(200);

  auto disabled_param = planner_param_;
  disabled_param.enable_arrow_aware_yellow_passing = false;  // Disable feature

  module_ = std::make_shared<TrafficLightModule>(
    100, traffic_light_reg_elem, lane, stop_line, true /* is_turn_lane */,
    true /* has_static_arrow */, disabled_param, logger_, clock_, time_keeper_,
    planning_factor_interface_);

  auto node = std::make_shared<rclcpp::Node>("test_node", get_node_options());
  planner_data_ = std::make_shared<PlannerData>(*node);
  planner_data_->current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
  planner_data_->current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  planner_data_->current_acceleration =
    std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  planner_data_->is_simulation = false;
  setPlannerData(planner_data_);

  // 1. Green -> Yellow
  autoware_perception_msgs::msg::TrafficLightGroup green_signal;
  green_signal.elements.push_back(
    createElement(Element::GREEN, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(green_signal);
  callIsStopSignal();

  autoware_perception_msgs::msg::TrafficLightGroup yellow_signal;
  yellow_signal.elements.push_back(
    createElement(Element::AMBER, Element::CIRCLE, Element::SOLID_ON));
  setTrafficSignal(yellow_signal);

  bool stop = callIsStopSignal();

  // State is NOT verified to be tracked because logic is skipped when disabled
  // Internal state should remain kNotYellow
  EXPECT_EQ(getYellowTransitionState(), YellowState::kNotYellow);

  // But should STOP because the feature is disabled
  EXPECT_TRUE(stop);
}

}  // namespace autoware::behavior_velocity_planner
