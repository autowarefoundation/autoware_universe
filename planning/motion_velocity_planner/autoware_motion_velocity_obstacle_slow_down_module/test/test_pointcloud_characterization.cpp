// Copyright 2026 The Autoware Contributors
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

// Characterization tests that pin the slow down intervals which obstacle_slow_down plans for a
// point cloud. They go through the public interface (preprocess_pointcloud() and plan()), so they
// also cover the point filtering inside the module. The expected values were taken from the code
// before the point filtering was optimized, so the tests pass both before and after the
// optimization as long as the output does not change.

#include "obstacle_slow_down_module.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::ObstacleSlowDownModule;
using autoware::motion_velocity_planner::PlannerData;
using autoware_planning_msgs::msg::TrajectoryPoint;

struct Interval
{
  double from_x;
  double from_y;
  double to_x;
  double to_y;
  double velocity;
};

TrajectoryPoint make_trajectory_point(const double x, const double y, const double yaw)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.z = std::sin(yaw / 2.0);
  point.pose.orientation.w = std::cos(yaw / 2.0);
  point.longitudinal_velocity_mps = 10.0;
  return point;
}

// a straight trajectory along the x axis
std::vector<TrajectoryPoint> make_straight_trajectory()
{
  std::vector<TrajectoryPoint> points;
  for (double x = 0.0; x <= 100.0; x += 1.0) {
    points.push_back(make_trajectory_point(x, 0.0, 0.0));
  }
  return points;
}

// a trajectory that curves like y = 6 sin(x / 25), so the polygons are rotated against the axes
std::vector<TrajectoryPoint> make_curved_trajectory()
{
  std::vector<TrajectoryPoint> points;
  for (double x = 0.0; x <= 100.0; x += 1.0) {
    const double slope = 6.0 / 25.0 * std::cos(x / 25.0);
    points.push_back(make_trajectory_point(x, 6.0 * std::sin(x / 25.0), std::atan(slope)));
  }
  return points;
}

// 3 x 3 points with 0.1 m spacing, so that they become one cluster
void add_cluster(pcl::PointCloud<pcl::PointXYZ> & cloud, const double x, const double y)
{
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      cloud.push_back(
        pcl::PointXYZ(static_cast<float>(x + 0.1 * i), static_cast<float>(y + 0.1 * j), 0.5F));
    }
  }
}
}  // namespace

class PointCloudCharacterizationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    std::vector<std::string> yaml_files = {
      autoware_test_utils_dir + "/config/test_motion_velocity_planner.param.yaml",
      autoware_test_utils_dir + "/config/test_common.param.yaml",
      autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
      autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
      ament_index_cpp::get_package_share_directory(
        "autoware_motion_velocity_obstacle_slow_down_module") +
        "/config/obstacle_slow_down.param.yaml"};
    autoware::test_utils::updateNodeOptions(node_options, yaml_files);
    // slow down for point clouds (disabled by default)
    node_options.append_parameter_override(
      "obstacle_slow_down.obstacle_filtering.object_type.pointcloud", true);

    node_ = std::make_shared<rclcpp::Node>("test_node", node_options);
    module_ = std::make_unique<ObstacleSlowDownModule>();
    module_->init(*node_, "obstacle_slow_down");
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Plans repeatedly until the slow down condition is satisfied, and returns the last intervals.
  // When enable_multi_polygon_filtering is false, the point cloud preprocessing does not remove the
  // points far from the trajectory, so that the filtering of the module is exercised.
  std::vector<Interval> plan_for_point_cloud(
    const std::vector<TrajectoryPoint> & trajectory, pcl::PointCloud<pcl::PointXYZ> cloud,
    const bool enable_multi_polygon_filtering)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.header.stamp = node_->get_clock()->now();
    odometry.pose.pose = trajectory.front().pose;

    auto planner_data = std::make_shared<PlannerData>(*node_);
    planner_data->current_odometry = odometry;
    auto & pointcloud = planner_data->no_ground_pointcloud;
    pointcloud.preprocess_params_.filter_by_trajectory_polygon.enable_monolithic_crop_box = false;
    pointcloud.preprocess_params_.filter_by_trajectory_polygon.enable_multi_polygon_filtering =
      enable_multi_polygon_filtering;
    pointcloud.preprocess_pointcloud(
      std::move(cloud), trajectory, odometry, 50.0, planner_data->vehicle_info_,
      planner_data->trajectory_polygon_collision_check, planner_data->ego_nearest_dist_threshold,
      planner_data->ego_nearest_yaw_threshold);

    const auto count_param_str =
      "obstacle_slow_down.obstacle_filtering.successive_num_to_entry_slow_down_condition";
    const size_t count = node_->get_parameter(count_param_str).as_int();
    autoware::motion_velocity_planner::VelocityPlanningResult result;
    for (size_t i = 0; i < count + 5; ++i) {
      result = module_->plan(trajectory, trajectory, planner_data);
    }

    std::vector<Interval> intervals;
    for (const auto & interval : result.slowdown_intervals) {
      intervals.push_back(
        {interval.from.x, interval.from.y, interval.to.x, interval.to.y, interval.velocity});
    }
    return intervals;
  }

  // The order of the intervals follows the order of the clusters, so compare them after sorting.
  static void expect_same_intervals(std::vector<Interval> actual, std::vector<Interval> expected)
  {
    const auto less = [](const Interval & a, const Interval & b) {
      return std::tie(a.from_x, a.from_y, a.to_x, a.to_y, a.velocity) <
             std::tie(b.from_x, b.from_y, b.to_x, b.to_y, b.velocity);
    };
    std::sort(actual.begin(), actual.end(), less);
    std::sort(expected.begin(), expected.end(), less);
    ASSERT_EQ(actual.size(), expected.size());
    constexpr double tolerance = 1e-9;
    for (size_t i = 0; i < actual.size(); ++i) {
      EXPECT_NEAR(actual[i].from_x, expected[i].from_x, tolerance) << "interval " << i;
      EXPECT_NEAR(actual[i].from_y, expected[i].from_y, tolerance) << "interval " << i;
      EXPECT_NEAR(actual[i].to_x, expected[i].to_x, tolerance) << "interval " << i;
      EXPECT_NEAR(actual[i].to_y, expected[i].to_y, tolerance) << "interval " << i;
      EXPECT_NEAR(actual[i].velocity, expected[i].velocity, tolerance) << "interval " << i;
    }
  }

  std::unique_ptr<ObstacleSlowDownModule> module_;
  std::shared_ptr<rclcpp::Node> node_;
};

namespace
{
pcl::PointCloud<pcl::PointXYZ> make_clusters()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // (x, y): near the trajectory, on both sides, and far from it
  const std::vector<std::pair<double, double>> centers = {
    {20.0, 0.0}, {30.0, 1.0}, {40.0, -1.5}, {50.0, 2.5}, {60.0, 4.0}, {70.0, -8.0}, {80.0, 0.5}};
  for (const auto & [x, y] : centers) {
    add_cluster(cloud, x, y);
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ> make_random_points()
{
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> dist_x(0.0, 100.0);
  std::uniform_real_distribution<double> dist_y(-14.0, 14.0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < 200; ++i) {
    // The order of the two draws is fixed (y, then x) because the expected values depend on it.
    const double y = dist_y(rng);
    const double x = dist_x(rng);
    cloud.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), 0.5F));
  }
  return cloud;
}
}  // namespace

namespace
{
// The slow down intervals planned by the code before the point filtering was optimized
// (columns: from.x, from.y, to.x, to.y, velocity). Four of the seven clusters are inside the
// trajectory polygons.
const std::vector<Interval> kExpectedForClusters = {
  {18.1100009537, 0, 30.21, 0, 8},
  {21.4475009, 0, 33.21, 0, 7.92500001192},
  {24.1100015259, 0, 36.21, 0, 8},
  {36.1100015259, 0, 48.21, 0, 8},
};

// 200 random points around the curved trajectory
const std::vector<Interval> kExpectedForRandomPoints = {
  {37.3043482638, 5.98045162146, 46.4903739467, 5.75033638617, 6.06469746818},
  {39.7556755568, 5.99798125841, 47.5796872379, 5.67048252343, 5.76265099207},
  {40.8297615361, 5.98764766874, 48.6501349915, 5.58156873504, 5.76265099207},
  {41.1985339689, 5.98139381536, 49.0635527257, 5.54520082044, 5.772914706},
  {42.3333311248, 5.95395240909, 51.7582456778, 5.26606148591, 6.1227001541},
  {42.7508612457, 5.94104212007, 51.3710730258, 5.30972450567, 5.94328758275},
  {43.155785701, 5.92704287629, 50.8947981922, 5.36253526545, 5.77946152769},
  {44.670023752, 5.85953184026, 52.4270608231, 5.18705732827, 5.80322536351},
  {47.3971598315, 5.68464378877, 56.4152929272, 4.64251715642, 6.03989952179},
  {49.0475852217, 5.54672547059, 58.0768231878, 4.38056502503, 6.04564820101},
  {52.1206492112, 5.22418062335, 60.062758675, 4.04147190973, 5.76265099207},
  {52.2126969284, 5.2130285833, 60.7319013598, 3.92091307892, 5.93717120394},
  {52.6294708053, 5.16253434935, 61.0882742095, 3.8561588161, 5.92436848965},
  {54.1155629214, 4.97243139368, 63.5076747095, 3.39353980677, 6.13882133148},
  {54.3586600502, 4.93904872668, 62.1620942612, 3.65508103677, 5.77967778661},
  {54.7571516806, 4.88432692283, 62.4818488666, 3.59360448885, 5.76265099207},
  {58.9223420935, 4.23942756841, 66.6124184862, 2.75468173553, 5.76265099207},
  {59.0243182382, 4.22224086898, 67.4813968672, 2.56780263134, 5.9585011023},
  {59.0614128861, 4.2157980661, 67.2373294439, 2.6207561949, 5.87332594611},
  {60.454497787, 3.97089262912, 68.8630373859, 2.26464801542, 5.92883745222},
  {62.0273460155, 3.68098795677, 69.8235883528, 2.04952720052, 5.76270107827},
  {62.2885517542, 3.63076811056, 70.1885680939, 1.96699556777, 5.81621557099},
  {62.9132060407, 3.51067104693, 71.3418943596, 1.70343051396, 5.93776337185},
  {63.3452483957, 3.42567606063, 71.003099229, 1.78153363636, 5.76265099207},
  {63.4929690577, 3.39644933878, 72.3067820152, 1.48023146322, 6.02653193604},
  {63.6129966919, 3.37270171789, 71.7197490268, 1.61632291809, 5.86543053898},
  {63.6843518421, 3.35858401018, 71.3399412156, 1.70388077618, 5.76265099207},
  {64.4027462584, 3.21432503146, 72.6820453989, 1.39278968179, 5.9059732502},
  {66.649883422, 2.74671339991, 74.8322015266, 0.886464385171, 5.88679123424},
  {67.1373724428, 2.64244315065, 74.7737053775, 0.900319640251, 5.76265099207},
};
}  // namespace

TEST_F(PointCloudCharacterizationTest, ClustersOnStraightTrajectoryWithPreprocessingFilter)
{
  expect_same_intervals(
    plan_for_point_cloud(make_straight_trajectory(), make_clusters(), true), kExpectedForClusters);
}

TEST_F(PointCloudCharacterizationTest, ClustersOnStraightTrajectoryWithoutPreprocessingFilter)
{
  expect_same_intervals(
    plan_for_point_cloud(make_straight_trajectory(), make_clusters(), false), kExpectedForClusters);
}

TEST_F(PointCloudCharacterizationTest, RandomPointsOnCurvedTrajectoryWithPreprocessingFilter)
{
  expect_same_intervals(
    plan_for_point_cloud(make_curved_trajectory(), make_random_points(), true),
    kExpectedForRandomPoints);
}

TEST_F(PointCloudCharacterizationTest, RandomPointsOnCurvedTrajectoryWithoutPreprocessingFilter)
{
  expect_same_intervals(
    plan_for_point_cloud(make_curved_trajectory(), make_random_points(), false),
    kExpectedForRandomPoints);
}
