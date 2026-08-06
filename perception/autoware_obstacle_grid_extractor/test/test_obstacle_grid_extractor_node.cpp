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
//
// Node-wrapper contract tests. These drive the real subscription callback through an in-process
// executor (no live sensor / no external DDS peers) to exercise the branches the wrapper OWNS:
//  - the cloud is ALREADY in base_link -> published with no TF involvement at all (the production
//    path: the concatenation node emits the no-ground cloud in base_link).
//  - the cloud is in another frame and TF resolves -> the points are transformed into base_link
//    and the grid still carries the input cloud's own stamp.
//  - TF lookup FAILS  -> nothing is published (the safety-critical "never emit a wrong-frame grid"
//    guard); consumers must treat a stale stamp as "unavailable", never as "clear".
#include "make_point_cloud.hpp"
#include "obstacle_grid_extractor_node.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace
{
using autoware::obstacle_grid_extractor::ObstacleGridExtractorNode;
using autoware::obstacle_grid_extractor::test::makePointCloud;
using std::chrono::steady_clock;

constexpr char kInput[] = "/obstacle_grid_extractor/input/pointcloud";
constexpr char kOutput[] = "/obstacle_grid_extractor/output/obstacle_grid";

// A deliberately small ROI: these are wrapper tests (frame handling, publish/drop), so the grid is
// kept cheap. The production z-band is used as-is because the transform assertions depend on it.
rclcpp::NodeOptions nodeOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("roi.length_x", 60.0);
  options.append_parameter_override("roi.length_y", 40.0);
  options.append_parameter_override("roi.offset_x", 20.0);
  options.append_parameter_override("resolution", 0.2);
  options.append_parameter_override("crop.z_min", -2.5);
  options.append_parameter_override("crop.z_max", 3.5);
  options.append_parameter_override("overhead_split", 2.5);
  return options;
}

// single XYZ point cloud in the given frame, stamped at `stamp`
sensor_msgs::msg::PointCloud2 makeSinglePointCloud(
  const rclcpp::Time & stamp, const std::string & frame, float x, float y, float z)
{
  auto cloud = makePointCloud(frame, {{x, y, z}});
  cloud.header.stamp = stamp;
  return cloud;
}

// spin the executor until `pred()` holds or `timeout` elapses; returns pred() at exit
template <typename Pred>
bool spinUntil(rclcpp::Executor & exec, Pred pred, std::chrono::milliseconds timeout)
{
  const auto deadline = steady_clock::now() + timeout;
  while (steady_clock::now() < deadline) {
    if (pred()) return true;
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}
}  // namespace

TEST(ObstacleGridExtractorNode, PublishesBaseLinkCloudWithoutTransforming)
{
  // Arrange: the node plus a helper node holding the input publisher and the output subscription.
  // NO transform is ever broadcast: a cloud that already declares base_link must not depend on TF.
  auto node = std::make_shared<ObstacleGridExtractorNode>(nodeOptions());
  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInput, rclcpp::SensorDataQoS().keep_last(1));

  grid_map_msgs::msg::GridMap::ConstSharedPtr received;
  auto sub = helper->create_subscription<grid_map_msgs::msg::GridMap>(
    kOutput, rclcpp::QoS(1).reliable(),
    [&received](grid_map_msgs::msg::GridMap::ConstSharedPtr m) { received = m; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  // Act: publish a base_link cloud, which takes the no-transform path. Re-publishing absorbs
  // discovery latency (the extractor is stateless, so repeated frames are idempotent); the last
  // published stamp is the one asserted on.
  rclcpp::Time published_stamp;
  const bool got = spinUntil(
    exec,
    [&]() {
      if (!received) published_stamp = helper->now();
      pub->publish(makeSinglePointCloud(published_stamp, "base_link", 8.05f, 0.05f, 0.5f));
      return static_cast<bool>(received);
    },
    std::chrono::seconds(10));

  // Assert: a base_link grid carrying the input cloud's stamp and the input point's cell, produced
  // with an empty TF buffer.
  ASSERT_TRUE(got) << "no grid published for a cloud already in base_link";
  EXPECT_EQ(received->header.frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(received->header.stamp), published_stamp)
    << "the grid must carry the sensing time, not the TF lookup time";
  grid_map::GridMap grid;
  grid_map::GridMapRosConverter::fromMessage(*received, grid);
  EXPECT_EQ(grid.getLayers().size(), 4u);
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.05, 0.05), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 1.0f);
}

TEST(ObstacleGridExtractorNode, TransformsNonBaseLinkCloudWhenTfResolves)
{
  // Arrange: same wiring, plus a static base_link -> sensor_frame extrinsic of (+1.0, 0, +2.0) so a
  // cloud in sensor_frame must actually be moved to land where the assertion expects it.
  auto node = std::make_shared<ObstacleGridExtractorNode>(nodeOptions());
  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInput, rclcpp::SensorDataQoS().keep_last(1));

  tf2_ros::StaticTransformBroadcaster tf_broadcaster(helper);
  geometry_msgs::msg::TransformStamped extrinsic;
  extrinsic.header.stamp = helper->now();
  extrinsic.header.frame_id = "base_link";
  extrinsic.child_frame_id = "sensor_frame";
  extrinsic.transform.translation.x = 1.0;
  extrinsic.transform.translation.z = 2.0;
  extrinsic.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(extrinsic);

  grid_map_msgs::msg::GridMap::ConstSharedPtr received;
  auto sub = helper->create_subscription<grid_map_msgs::msg::GridMap>(
    kOutput, rclcpp::QoS(1).reliable(),
    [&received](grid_map_msgs::msg::GridMap::ConstSharedPtr m) { received = m; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  // Act: publish in sensor_frame. (7.05, 0.05, -3.0) there is (8.05, 0.05, -1.0) in base_link. The
  // sensor-frame z is below crop.z_min, so an untransformed point would be cropped away entirely —
  // an occupied cell can only exist if the transform ran, and only at the transformed position.
  rclcpp::Time published_stamp;
  const bool got = spinUntil(
    exec,
    [&]() {
      if (!received) published_stamp = helper->now();
      pub->publish(makeSinglePointCloud(published_stamp, "sensor_frame", 7.05f, 0.05f, -3.0f));
      return static_cast<bool>(received);
    },
    std::chrono::seconds(10));

  // Assert: the grid is in base_link, keeps the cloud's stamp, and holds the TRANSFORMED position.
  ASSERT_TRUE(got) << "no grid published for a resolvable sensor_frame cloud";
  EXPECT_EQ(received->header.frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(received->header.stamp), published_stamp)
    << "doTransform() overwrites the header; the cloud's own stamp must be restored";
  grid_map::GridMap grid;
  grid_map::GridMapRosConverter::fromMessage(*received, grid);
  grid_map::Index transformed_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.05, 0.05), transformed_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", transformed_cell), 1.0f);
  EXPECT_FLOAT_EQ(grid.at("max_height", transformed_cell), -1.0f);
  grid_map::Index untransformed_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(7.05, 0.05), untransformed_cell));
  EXPECT_TRUE(std::isnan(grid.at("point_count", untransformed_cell)))
    << "the point was rasterized at its sensor-frame position, i.e. the transform was skipped";
}

TEST(ObstacleGridExtractorNode, DropsCloudWhenTfLookupFails)
{
  // Arrange: same wiring, and wait for discovery first so that a later "nothing arrived" means the
  // callback dropped the cloud rather than that the link was never established.
  auto node = std::make_shared<ObstacleGridExtractorNode>(nodeOptions());
  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInput, rclcpp::SensorDataQoS().keep_last(1));

  grid_map_msgs::msg::GridMap::ConstSharedPtr received;
  auto sub = helper->create_subscription<grid_map_msgs::msg::GridMap>(
    kOutput, rclcpp::QoS(1).reliable(),
    [&received](grid_map_msgs::msg::GridMap::ConstSharedPtr m) { received = m; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);
  ASSERT_TRUE(spinUntil(
    exec, [&]() { return pub->get_subscription_count() > 0; }, std::chrono::seconds(10)))
    << "input subscription never discovered";

  // Act: publish clouds in an UNKNOWN frame. lookupTransform to base_link has no data and throws
  // after its timeout, so onCloud takes the catch branch.
  const auto deadline = steady_clock::now() + std::chrono::milliseconds(1500);
  while (steady_clock::now() < deadline) {
    pub->publish(
      makeSinglePointCloud(helper->now(), "lidar_top_that_has_no_tf", 8.05f, 0.05f, 0.5f));
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Assert: not even the all-NaN heartbeat is emitted on a wrong-frame cloud.
  EXPECT_EQ(received, nullptr) << "a grid was published despite the TF lookup failing";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
