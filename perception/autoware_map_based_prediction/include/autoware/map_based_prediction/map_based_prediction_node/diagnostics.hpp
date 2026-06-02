// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__DIAGNOSTICS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__DIAGNOSTICS_HPP_

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>

namespace autoware::map_based_prediction
{

class Diagnostics
{
public:
  struct Params
  {
    double processing_time_tolerance_ms{};
    double processing_time_consecutive_excess_tolerance_ms{};
  };

  explicit Diagnostics(rclcpp::Node * node);
  void setParams(const Params & params);
  void setPublishedTimePublisher(std::unique_ptr<autoware_utils::PublishedTimePublisher> publisher);
  void setProcessingTimePublisher(std::unique_ptr<autoware_utils::DebugPublisher> publisher);

  // Called from ObjectsCallback after pub_objects_->publish() to record message publish time.
  template <typename T>
  void publishIfSubscribed(
    const typename rclcpp::Publisher<T>::SharedPtr & pub, const rclcpp::Time & stamp)
  {
    if (published_time_publisher_) published_time_publisher_->publish_if_subscribed(pub, stamp);
  }

  // Runs diagnostics checks and publishes processing-time debug topics.
  void update(const rclcpp::Time & timestamp, double processing_time_ms, double cyclic_time_ms);

private:
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_ptr_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;
  std::unique_ptr<autoware_utils::DebugPublisher> processing_time_publisher_;

  Params params_{};
  std::optional<rclcpp::Time> last_in_time_processing_timestamp_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__MAP_BASED_PREDICTION_NODE__DIAGNOSTICS_HPP_
