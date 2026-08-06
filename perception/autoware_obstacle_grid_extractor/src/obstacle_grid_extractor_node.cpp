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
#include "obstacle_grid_extractor_node.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2/exceptions.h>

#include <functional>
#include <memory>

namespace autoware::obstacle_grid_extractor
{
namespace
{
// The frame the ROI is configured in. The extractor rasterizes a cloud in whatever frame it arrives
// in, so the node owns the guarantee that it arrives in this one.
constexpr char kTargetFrame[] = "base_link";
}  // namespace

ObstacleGridExtractorNode::ObstacleGridExtractorNode(const rclcpp::NodeOptions & options)
: Node("obstacle_grid_extractor", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  ExtractorParams p;
  p.roi_length_x = declare_parameter<double>("roi.length_x");
  p.roi_length_y = declare_parameter<double>("roi.length_y");
  p.roi_offset_x = declare_parameter<double>("roi.offset_x");
  p.resolution = declare_parameter<double>("resolution");
  p.crop_z_min = static_cast<float>(declare_parameter<double>("crop.z_min"));
  p.crop_z_max = static_cast<float>(declare_parameter<double>("crop.z_max"));
  p.overhead_split = static_cast<float>(declare_parameter<double>("overhead_split"));
  extractor_ = std::make_unique<ObstacleGridExtractor>(p);

  using std::placeholders::_1;
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ObstacleGridExtractorNode::onCloud, this, _1));
  // RELIABLE KEEP_LAST(1) on the last-resort path. The grid is fixed-size but NOT small: at the
  // default ROI it is 450 k cells x 4 float32 layers = 7.2 MB per message. KEEP_LAST(1) bounds the
  // queue so a slow consumer drops frames (detectable as a stale stamp) instead of queueing them.
  pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
    "~/output/obstacle_grid", rclcpp::QoS(1).reliable());
}

void ObstacleGridExtractorNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // The production pipeline already publishes the no-ground cloud in base_link (the concatenation
  // node's `output_frame`), so this is the common path, not a special case: doTransform() would
  // otherwise copy every point of a >100 k-point cloud only to reproduce it unchanged.
  if (msg->header.frame_id == kTargetFrame) {
    pub_->publish(extractor_->extract(*msg));  // empty cloud -> heartbeat
    return;
  }

  sensor_msgs::msg::PointCloud2 in_base_link;
  try {
    const auto tf = tf_buffer_.lookupTransform(
      kTargetFrame, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(*msg, in_base_link, tf);  // static extrinsic only
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF to base_link failed: %s", e.what());
    // Never publish a wrong-frame grid. Nothing (not even the all-NaN heartbeat) is published on
    // this path, so consumers MUST treat a stale grid stamp as "unavailable", never as "clear".
    return;
  }
  // doTransform() replaces the whole header with the transform's, so restore the cloud's own stamp
  // rather than depend on the lookup echoing it back. The grid must carry the exact sensing time.
  in_base_link.header.stamp = msg->header.stamp;
  pub_->publish(extractor_->extract(in_base_link));  // empty cloud -> heartbeat
}
}  // namespace autoware::obstacle_grid_extractor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::obstacle_grid_extractor::ObstacleGridExtractorNode)
