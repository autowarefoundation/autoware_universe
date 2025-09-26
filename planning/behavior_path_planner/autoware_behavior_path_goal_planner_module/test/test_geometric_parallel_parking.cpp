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

#include "autoware/behavior_path_goal_planner_module/goal_candidate.hpp"
#include "autoware/behavior_path_goal_planner_module/manager.hpp"
#include "goal_planner_test_helper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_planner/geometric_pull_over.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::testing::GoalPlannerTestHelper;

namespace autoware::behavior_path_planner
{

class TestGeometricPullOver : public ::testing::Test
{
public:
  std::optional<PullOverPath> call_plan(
    const GoalCandidate & candidate, const size_t id,
    const std::shared_ptr<const PlannerData> & planner_data,
    BehaviorModuleOutput & upstream_module_output, const bool & is_forward,
    const bool & use_clothoid)
  {
    initialize_geometric_pull_over_planner(is_forward, use_clothoid);
    return geometric_pull_over_->plan(candidate, id, planner_data, upstream_module_output);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ =
      rclcpp::Node::make_shared("geometric_pull_over", GoalPlannerTestHelper::make_node_options());
  }
  void TearDown() override { rclcpp::shutdown(); }

  // Member variables
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<GeometricPullOver> geometric_pull_over_;

private:
  void initialize_geometric_pull_over_planner(const bool & is_forward, const bool & use_clothoid)
  {
    auto parameters =
      GoalPlannerModuleManager::initGoalPlannerParameters(node_.get(), "goal_planner.");
    // parameters.lane_departure_check_expansion_margin = 0.0;

    geometric_pull_over_ =
      std::make_shared<GeometricPullOver>(*node_, parameters, is_forward, use_clothoid);
  }
};

TEST_F(TestGeometricPullOver, GenerateValidClothoidPullOverPath)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.8756).y(387.394).z(0.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.7133).w(0.7008));

  const auto goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(363.228).y(421.651).z(0.0))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.7106).w(0.7036));
  auto candidate = GoalCandidate{goal_pose};
  const bool is_forward = true;
  const bool use_clothoid = true;

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node_);
  GoalPlannerTestHelper::set_odometry(planner_data, start_pose);
  GoalPlannerTestHelper::set_route(planner_data, 8930, 4619);

  // Plan the pull over path
  BehaviorModuleOutput upstream_module_output;
  const size_t id = 0;
  auto result =
    call_plan(candidate, id, planner_data, upstream_module_output, is_forward, use_clothoid);
  const auto & partial_paths = result->partial_paths();
  // Assert that a valid geometric pull over path is generated
  ASSERT_TRUE(result.has_value()) << "Geometric pull over path generation failed.";
  EXPECT_EQ(partial_paths.size(), 1UL)
    << "Generated geometric pull over path does not have the expected number of partial paths.";
};
}  // namespace autoware::behavior_path_planner
