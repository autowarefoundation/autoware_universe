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

#ifndef LANELET2_MAP_PROVIDER_NODE_HPP_
#define LANELET2_MAP_PROVIDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/lanelet_map_meta_data.hpp>
#include <autoware_map_msgs/srv/get_selected_lanelet2_map.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autoware::dynamic_lanelet_map_provider
{

class Lanelet2MapProviderNode : public rclcpp::Node
{
public:
  using LaneletMapCellMetaData = autoware_map_msgs::msg::LaneletMapCellMetaData;

  explicit Lanelet2MapProviderNode(const rclcpp::NodeOptions & options);

  /// @brief AABB-vs-circle overlap test.
  /// Clamps the circle center to the cell's bounding box and checks if the
  /// closest point lies within the radius.
  static bool cell_overlaps_circle(
    const LaneletMapCellMetaData & cell, double cx, double cy, double radius);

private:
  using GetSelectedLanelet2Map = autoware_map_msgs::srv::GetSelectedLanelet2Map;
  using LaneletMapMetaData = autoware_map_msgs::msg::LaneletMapMetaData;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using Odometry = nav_msgs::msg::Odometry;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_map_metadata(const LaneletMapMetaData::ConstSharedPtr msg);
  void on_timer();

  std::vector<std::string> select_cells(double cx, double cy, double radius) const;

  double map_radius_;
  double update_distance_threshold_;

  std::optional<Odometry> latest_odometry_;
  std::optional<LaneletMapMetaData> map_metadata_;

  double last_update_x_{0.0};
  double last_update_y_{0.0};
  bool first_update_{true};
  bool request_in_flight_{false};

  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_client_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaneletMapMetaData>::SharedPtr sub_map_metadata_;
  rclcpp::Publisher<LaneletMapBin>::SharedPtr pub_submap_;
  rclcpp::Client<GetSelectedLanelet2Map>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::dynamic_lanelet_map_provider

#endif  // LANELET2_MAP_PROVIDER_NODE_HPP_
