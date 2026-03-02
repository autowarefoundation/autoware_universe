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

#include "autoware/trajectory_traffic_rule_filter/trajectory_traffic_rule_filter_node.hpp"
#include "test_utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter
{

class TrajectoryTrafficRuleFilterTest : public ::testing::Test
{
protected:
  const lanelet::Id light_id = 1;
  const double stop_line_x = 10.0;
  const double longitudinal_offset = 4.0;  // [m]

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override("use_sim_time", false);
    node_options_.append_parameter_override(
      "filter_names", std::vector<std::string>(
                        {"autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter"}));

    // Set up dummy vehicle info required by many Autoware nodes
    const auto test_pkg_share = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options_, {test_pkg_share + "/config/test_vehicle_info.param.yaml"});

    node_under_test_ = std::make_shared<TrajectoryTrafficRuleFilter>(node_options_);
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node");

    map_pub_ = test_node_->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/trajectory_traffic_rule_filter_node/input/lanelet2_map", rclcpp::QoS{1}.transient_local());

    auto map = utils::create_map(light_id, stop_line_x);
    autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
    map_pub_->publish(experimental::lanelet2_utils::to_autoware_map_msgs(map));

    tl_pub_ = test_node_->create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/trajectory_traffic_rule_filter_node/input/traffic_signals", 1);

    traj_pub_ =
      test_node_->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_traffic_rule_filter_node/input/candidate_trajectories", 1);

    diag_sub_ = test_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1, [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        if (!msg->status.empty()) {
          last_diag_status_ = msg->status.front();
        }
      });
    last_diag_status_.reset();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Utility to spin until a condition is met or timeout
  bool spin_until(
    std::function<bool()> && condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
      if (condition()) return true;
      rclcpp::spin_some(node_under_test_);
      rclcpp::spin_some(test_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  void publish_traffic_light(uint8_t color)
  {
    autoware_perception_msgs::msg::TrafficLightGroupArray msg;
    autoware_perception_msgs::msg::TrafficLightGroup group;
    group.traffic_light_group_id = light_id;
    autoware_perception_msgs::msg::TrafficLightElement element;
    element.color = color;
    element.shape = autoware_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.status = autoware_perception_msgs::msg::TrafficLightElement::SOLID_ON;
    element.confidence = 1.0;
    group.elements.push_back(element);
    msg.traffic_light_groups.push_back(group);
    tl_pub_->publish(msg);
  }

  static void add_trajectory(
    autoware_internal_planning_msgs::msg::CandidateTrajectories & msg, bool is_diffusion,
    double end_x)
  {
    autoware_internal_planning_msgs::msg::CandidateTrajectory traj;
    autoware_internal_planning_msgs::msg::GeneratorInfo info;

    info.generator_name.data = is_diffusion ? "DiffusionPlanner" : "OtherPlanner";
    info.generator_id = autoware_utils_uuid::generate_uuid();
    traj.generator_id = info.generator_id;

    autoware_planning_msgs::msg::TrajectoryPoint p1;
    autoware_planning_msgs::msg::TrajectoryPoint p2;
    p1.longitudinal_velocity_mps = 1.0;
    p1.pose.position.x = 0.0;
    p1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    p2.longitudinal_velocity_mps = 1.0;
    p2.pose.position.x = end_x;
    p2.time_from_start = rclcpp::Duration::from_seconds(end_x);
    traj.points.push_back(p1);
    traj.points.push_back(p2);

    msg.candidate_trajectories.push_back(traj);
    msg.generator_info.push_back(info);
  }

  rclcpp::NodeOptions node_options_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<TrajectoryTrafficRuleFilter> node_under_test_;
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr tl_pub_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    traj_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  std::optional<diagnostic_msgs::msg::DiagnosticStatus> last_diag_status_;
};

TEST_F(TrajectoryTrafficRuleFilterTest, TestOkStatus)
{
  publish_traffic_light(autoware_perception_msgs::msg::TrafficLightElement::GREEN);

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, true, stop_line_x + 1.0);   // Diffusion, feasible
  add_trajectory(msg, false, stop_line_x + 1.0);  // Other, feasible
  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until([this] { return last_diag_status_.has_value(); }));
  EXPECT_EQ(last_diag_status_->level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(TrajectoryTrafficRuleFilterTest, TestWarnStatus)
{
  publish_traffic_light(autoware_perception_msgs::msg::TrafficLightElement::RED);

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, true, stop_line_x + 1.0);                   // Diffusion, infeasible
  add_trajectory(msg, false, stop_line_x - longitudinal_offset);  // Other, feasible
  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until([this] { return last_diag_status_.has_value(); }));
  EXPECT_EQ(last_diag_status_->level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(last_diag_status_->message, "All diffusion planner trajectories are infeasible");
}

TEST_F(TrajectoryTrafficRuleFilterTest, TestErrorStatus)
{
  publish_traffic_light(autoware_perception_msgs::msg::TrafficLightElement::RED);

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, true, stop_line_x + 1.0);   // Diffusion, infeasible
  add_trajectory(msg, false, stop_line_x + 1.0);  // Other, infeasible
  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until([this] { return last_diag_status_.has_value(); }));
  EXPECT_EQ(last_diag_status_->level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(last_diag_status_->message, "No feasible trajectories found");
}

}  // namespace autoware::trajectory_traffic_rule_filter
