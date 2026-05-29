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

#include "detected_object_feature_remover_node.hpp"

#include <algorithm>
#include <memory>
#include <vector>

namespace autoware::detected_object_feature_remover
{
DetectedObjectFeatureRemover::DetectedObjectFeatureRemover(const rclcpp::NodeOptions & node_options)
: Node("detected_object_feature_remover", node_options)
{
  pub_ = create_publisher<DetectedObjects>("~/output", rclcpp::QoS(1));
  sub_ = create_subscription<DetectedObjectsWithFeature>(
    "~/input", rclcpp::QoS{1},
    [this](const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DetectedObjectsWithFeature) & input) {
      this->objectCallback(input);
    });
  convert_params_.run_convex_hull_conversion =
    this->declare_parameter<bool>("run_convex_hull_conversion", false);
  published_time_publisher_ =
    std::make_unique<autoware_utils::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>(
      this);
}

void DetectedObjectFeatureRemover::objectCallback(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DetectedObjectsWithFeature) & input)
{
  auto output = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_);
  convert::convertToDetectedObjects(*input, *output, convert_params_);
  const auto header_stamp = output->header.stamp;
  pub_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_, header_stamp);
}

}  // namespace autoware::detected_object_feature_remover

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_feature_remover::DetectedObjectFeatureRemover)
