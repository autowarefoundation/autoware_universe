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

#include "lanelet2_map_provider_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::dynamic_lanelet_map_provider
{

Lanelet2MapProviderNode::Lanelet2MapProviderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_provider", options)
{
  map_radius_ = declare_parameter<double>("map_radius");
  update_distance_threshold_ = declare_parameter<double>("update_distance_threshold");

  cb_group_timer_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_client_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_timer_;

  sub_odometry_ = create_subscription<Odometry>(
    "input/odometry", rclcpp::QoS{1},
    [this](const Odometry::ConstSharedPtr msg) { on_odometry(msg); }, sub_opts);

  sub_map_metadata_ = create_subscription<LaneletMapMetaData>(
    "input/lanelet2_map_metadata", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapMetaData::ConstSharedPtr msg) { on_map_metadata(msg); }, sub_opts);

  pub_submap_ = create_publisher<LaneletMapBin>(
    "output/lanelet2_submap", rclcpp::QoS{1}.transient_local());

  client_ = create_client<GetSelectedLanelet2Map>(
    "service/get_selected_lanelet2_map", rmw_qos_profile_services_default, cb_group_client_);

  timer_ = create_wall_timer(
    std::chrono::seconds(1), [this] { on_timer(); }, cb_group_timer_);
}

void Lanelet2MapProviderNode::on_odometry(const Odometry::ConstSharedPtr msg)
{
  latest_odometry_ = *msg;
}

void Lanelet2MapProviderNode::on_map_metadata(const LaneletMapMetaData::ConstSharedPtr msg)
{
  map_metadata_ = *msg;
  RCLCPP_INFO(
    get_logger(), "Received lanelet2 map metadata with %zu cells.",
    msg->metadata_list.size());
}

void Lanelet2MapProviderNode::on_timer()
{
  if (!latest_odometry_ || !map_metadata_ || request_in_flight_) {
    return;
  }

  const double cx = latest_odometry_->pose.pose.position.x;
  const double cy = latest_odometry_->pose.pose.position.y;

  const double moved = std::hypot(cx - last_update_x_, cy - last_update_y_);
  if (!first_update_ && moved < update_distance_threshold_) {
    return;
  }

  const auto cell_ids = select_cells(cx, cy, map_radius_);
  if (cell_ids.empty()) {
    RCLCPP_WARN(get_logger(), "No map cells overlap the ego position (radius=%.1f m).", map_radius_);
    return;
  }

  if (!client_->wait_for_service(std::chrono::seconds(0))) {
    RCLCPP_WARN(get_logger(), "GetSelectedLanelet2Map service not yet available.");
    return;
  }

  auto request = std::make_shared<GetSelectedLanelet2Map::Request>();
  request->cell_ids = cell_ids;

  request_in_flight_ = true;
  last_update_x_ = cx;
  last_update_y_ = cy;
  first_update_ = false;

  client_->async_send_request(
    request,
    [this](rclcpp::Client<GetSelectedLanelet2Map>::SharedFuture future) {
      request_in_flight_ = false;
      const auto response = future.get();
      if (response->lanelet2_cells.data.empty()) {
        RCLCPP_ERROR(get_logger(), "GetSelectedLanelet2Map returned an empty map.");
        return;
      }
      pub_submap_->publish(response->lanelet2_cells);
      RCLCPP_DEBUG(
        get_logger(), "Published lanelet2 submap (%zu bytes).",
        response->lanelet2_cells.data.size());
    });
}

bool Lanelet2MapProviderNode::cell_overlaps_circle(
  const LaneletMapCellMetaData & cell, const double cx, const double cy, const double radius)
{
  const double px = std::clamp(cx, cell.min_x, cell.max_x);
  const double py = std::clamp(cy, cell.min_y, cell.max_y);
  return std::hypot(cx - px, cy - py) <= radius;
}

std::vector<std::string> Lanelet2MapProviderNode::select_cells(
  const double cx, const double cy, const double radius) const
{
  std::vector<std::string> ids;
  for (const auto & cell : map_metadata_->metadata_list) {
    if (cell_overlaps_circle(cell, cx, cy, radius)) {
      ids.push_back(cell.cell_id);
    }
  }
  return ids;
}

}  // namespace autoware::dynamic_lanelet_map_provider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::dynamic_lanelet_map_provider::Lanelet2MapProviderNode)
