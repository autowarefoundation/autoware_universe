#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <autoware/scenario_selector_base.hpp>
#include <autoware/selectors/default_selector.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

using autoware::scenario_selector::ScenarioSelectorPlugin;
using autoware::scenario_selector::DefaultScenarioSelector;

class RclGuard : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(RclGuard, PluginLoadAndInitialize_Succeeds)
{
  auto node = std::make_shared<rclcpp::Node>("default_selector_test_node");

  auto plugin = std::make_shared<DefaultScenarioSelector>();

  EXPECT_NO_THROW(plugin->initialize(node.get()));

  EXPECT_FALSE(plugin->ready());
}

TEST_F(RclGuard, PublishTrajectory_WithinDelay_Publishes)
{
  auto node = std::make_shared<rclcpp::Node>("publish_ok_node");
  auto plugin = std::make_shared<DefaultScenarioSelector>();
  plugin->initialize(node.get());

  std::atomic<int> recv_count{0};
  auto sub = node->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1},
    [&recv_count](const autoware_planning_msgs::msg::Trajectory::SharedPtr) {
      ++recv_count;
    });

  auto msg = std::make_shared<autoware_planning_msgs::msg::Trajectory>();
  msg->header.stamp = node->now();  
  msg->points.resize(2);            

  
  plugin->publishTrajectory(msg);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin_some();

  EXPECT_EQ(recv_count.load(), 1);
}

TEST_F(RclGuard, PublishTrajectory_TooOld_Dropped)
{
  auto node = std::make_shared<rclcpp::Node>("publish_drop_node");
  auto plugin = std::make_shared<DefaultScenarioSelector>();
  plugin->initialize(node.get());

  std::atomic<int> recv_count{0};
  auto sub = node->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1},
    [&recv_count](const autoware_planning_msgs::msg::Trajectory::SharedPtr) {
      ++recv_count;
    });

  auto msg = std::make_shared<autoware_planning_msgs::msg::Trajectory>();
  msg->header.stamp = node->now() - rclcpp::Duration::from_seconds(5.0);
  msg->points.resize(2);

  plugin->publishTrajectory(msg);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin_some();

  EXPECT_EQ(recv_count.load(), 0);
}

TEST_F(RclGuard, GetScenarioTrajectory_ReturnsExpectedPointers)
{
  auto node = std::make_shared<rclcpp::Node>("get_traj_node");
  auto plugin = std::make_shared<DefaultScenarioSelector>();
  plugin->initialize(node.get());

  using autoware_internal_planning_msgs::msg::Scenario;

  auto lane_traj = std::make_shared<autoware_planning_msgs::msg::Trajectory>();
  lane_traj->points.resize(3);
  plugin->onLaneDrivingTrajectory(lane_traj);

  auto park_traj = std::make_shared<autoware_planning_msgs::msg::Trajectory>();
  park_traj->points.resize(4);
  plugin->onParkingTrajectory(park_traj);

  auto got_lane = plugin->getScenarioTrajectory(Scenario::LANEDRIVING);
  auto got_park = plugin->getScenarioTrajectory(Scenario::PARKING);

  ASSERT_TRUE(got_lane);
  ASSERT_TRUE(got_park);
  EXPECT_EQ(got_lane.get(), lane_traj.get());
  EXPECT_EQ(got_park.get(), park_traj.get());

  auto got_invalid = plugin->getScenarioTrajectory("NOT_A_SCENARIO");
  ASSERT_TRUE(got_invalid);
  EXPECT_EQ(got_invalid.get(), lane_traj.get());
}
