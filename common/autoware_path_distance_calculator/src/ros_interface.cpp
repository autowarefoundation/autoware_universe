// Copyright 2021 Tier IV, Inc.
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

#include "ros_interface.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <chrono>
#include <utility>

namespace autoware::path_distance_calculator
{
namespace
{
// The polling subscriber keeps returning the latest sample every tick even if nothing new has
// arrived. This returns data only the first time it is observed, and remembers it in `last` so
// the same sample is not handed to the caller (and recomputed) again.
template <typename T>
std::shared_ptr<const T> poll_new_data(
  autoware::agnocast_wrapper::polling::PollingSubscriber<T> & subscriber,
  std::shared_ptr<const T> & last)
{
  const auto data = subscriber.take_data();
  if (!data || data == last) {
    return nullptr;
  }
  last = data;
  return data;
}
}  // namespace

PathDistanceCalculator::PathDistanceCalculator(const rclcpp::NodeOptions & options)
: Node("path_distance_calculator", options)
{
  pub_dist_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/output/distance", rclcpp::QoS(1));

  using std::chrono_literals::operator""s;
  timer_ =
    autoware::agnocast_wrapper::create_timer(this, get_clock(), 1s, [this]() { on_timer(); });
}

void PathDistanceCalculator::on_timer()
{
  if (const auto map = poll_new_data(*sub_map_, last_map_)) {
    calculator_.set_map(*map);
  }
  if (const auto route = poll_new_data(*sub_route_, last_route_)) {
    calculator_.set_route(*route);
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no pose");
    return;
  }
  const auto pose = autoware_utils::transform2pose(transform);

  const auto distance = calculator_.calculate_remaining_distance(pose.pose);
  if (!distance) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no route");
    return;
  }

  auto msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_dist_);
  msg->stamp = pose.header.stamp;
  msg->data = distance.value();
  pub_dist_->publish(std::move(msg));
}

}  // namespace autoware::path_distance_calculator
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_distance_calculator::PathDistanceCalculator)
