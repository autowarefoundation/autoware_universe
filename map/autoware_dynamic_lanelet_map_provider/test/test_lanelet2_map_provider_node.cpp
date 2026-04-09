// Copyright 2024 The Autoware Contributors
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

#include "../src/lanelet2_map_provider_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_cell_meta_data.hpp>
#include <autoware_map_msgs/msg/lanelet_map_meta_data.hpp>
#include <autoware_map_msgs/srv/get_selected_lanelet2_map.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware::dynamic_lanelet_map_provider::Lanelet2MapProviderNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_map_msgs::msg::LaneletMapCellMetaData;
using autoware_map_msgs::msg::LaneletMapMetaData;
using autoware_map_msgs::srv::GetSelectedLanelet2Map;
using nav_msgs::msg::Odometry;

// ---------------------------------------------------------------------------
// Unit tests for the AABB-vs-circle overlap logic (white-box via a test
// subclass that exposes the static helper).
// ---------------------------------------------------------------------------

class Lanelet2MapProviderNodeTestable : public Lanelet2MapProviderNode
{
public:
  using Lanelet2MapProviderNode::cell_overlaps_circle;
  explicit Lanelet2MapProviderNodeTestable(const rclcpp::NodeOptions & opts)
  : Lanelet2MapProviderNode(opts)
  {
  }
};

static LaneletMapCellMetaData make_cell(
  const std::string & id, double min_x, double min_y, double max_x, double max_y)
{
  LaneletMapCellMetaData cell;
  cell.cell_id = id;
  cell.min_x = min_x;
  cell.min_y = min_y;
  cell.max_x = max_x;
  cell.max_y = max_y;
  return cell;
}

class TestCellOverlapsCircle : public ::testing::Test
{
protected:
  // Cell centred at origin, ±10 m.
  LaneletMapCellMetaData cell = make_cell("test", -10.0, -10.0, 10.0, 10.0);
};

TEST_F(TestCellOverlapsCircle, CenterInsideCellAlwaysOverlaps)
{
  EXPECT_TRUE(Lanelet2MapProviderNode::cell_overlaps_circle(cell, 0.0, 0.0, 1.0));
}

TEST_F(TestCellOverlapsCircle, CornerJustInsideRadius)
{
  // Corner at (10, 10); circle center at (10 + r*cos45, 10 + r*sin45) just touches.
  const double r = 5.0;
  const double offset = r - 0.01;  // just inside
  EXPECT_TRUE(
    Lanelet2MapProviderNode::cell_overlaps_circle(cell, 10.0 + offset * 0.707, 10.0 + offset * 0.707, r));
}

TEST_F(TestCellOverlapsCircle, CornerJustOutsideRadius)
{
  const double r = 5.0;
  const double offset = r + 0.01;  // just outside
  EXPECT_FALSE(
    Lanelet2MapProviderNode::cell_overlaps_circle(cell, 10.0 + offset * 0.707, 10.0 + offset * 0.707, r));
}

TEST_F(TestCellOverlapsCircle, CircleOutsideCellNoOverlap)
{
  // Circle far away in X direction.
  EXPECT_FALSE(Lanelet2MapProviderNode::cell_overlaps_circle(cell, 100.0, 0.0, 5.0));
}

TEST_F(TestCellOverlapsCircle, CircleEdgeTouchesCell)
{
  // Circle touching the right edge of the cell exactly.
  EXPECT_TRUE(Lanelet2MapProviderNode::cell_overlaps_circle(cell, 15.0, 0.0, 5.0));
}

// ---------------------------------------------------------------------------
// Integration test: node construction, metadata subscription, service stub.
// ---------------------------------------------------------------------------

class TestLanelet2MapProviderNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions opts;
    opts.append_parameter_override("map_radius", 150.0);
    opts.append_parameter_override("update_distance_threshold", 50.0);
    node_ = std::make_shared<Lanelet2MapProviderNodeTestable>(opts);

    helper_ = std::make_shared<rclcpp::Node>("test_helper");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<Lanelet2MapProviderNodeTestable> node_;
  std::shared_ptr<rclcpp::Node> helper_;
};

TEST_F(TestLanelet2MapProviderNode, NodeConstructsWithoutError)
{
  SUCCEED();
}

TEST_F(TestLanelet2MapProviderNode, SubmapPublishedAfterServiceResponse)
{
  // Publish fake map metadata.
  auto meta_pub = helper_->create_publisher<LaneletMapMetaData>(
    "input/lanelet2_map_metadata", rclcpp::QoS{1}.transient_local());

  LaneletMapMetaData meta;
  meta.header.frame_id = "map";
  meta.metadata_list.push_back(make_cell("/fake/map.osm", -200.0, -200.0, 200.0, 200.0));
  meta_pub->publish(meta);

  // Publish a fake odometry at the origin.
  auto odom_pub = helper_->create_publisher<Odometry>("input/odometry", 1);
  Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom_pub->publish(odom);

  // Stub out the GetSelectedLanelet2Map service so the node doesn't hang.
  auto service = helper_->create_service<GetSelectedLanelet2Map>(
    "service/get_selected_lanelet2_map",
    [](
      const GetSelectedLanelet2Map::Request::SharedPtr /*req*/,
      GetSelectedLanelet2Map::Response::SharedPtr res) {
      // Return a non-empty (but minimal) LaneletMapBin.
      res->header.frame_id = "map";
      res->lanelet2_cells.data = {0x01};  // arbitrary non-empty payload
    });

  bool submap_received = false;
  auto sub = helper_->create_subscription<LaneletMapBin>(
    "output/lanelet2_submap", rclcpp::QoS{1}.transient_local(),
    [&submap_received](const LaneletMapBin::SharedPtr) { submap_received = true; });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(helper_);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!submap_received && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(50));
  }

  EXPECT_TRUE(submap_received) << "output/lanelet2_submap was not published within 5 s";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
