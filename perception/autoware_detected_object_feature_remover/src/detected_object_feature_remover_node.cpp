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
  using std::placeholders::_1;
  pub_ = this->create_publisher<DetectedObjects>("~/output", rclcpp::QoS(1));
  sub_ = this->create_subscription<DetectedObjectsWithFeature>(
    "~/input", 1, std::bind(&DetectedObjectFeatureRemover::objectCallback, this, _1));
  convert_params_.run_convex_hull_conversion =
    this->declare_parameter<bool>("run_convex_hull_conversion", false);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void DetectedObjectFeatureRemover::objectCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr input)
{
  DetectedObjects output;
  convert::convertToDetectedObjects(*input, output, convert_params_);
  pub_->publish(output);
  published_time_publisher_->publish_if_subscribed(pub_, output.header.stamp);
}

}  // namespace autoware::detected_object_feature_remover

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_feature_remover::DetectedObjectFeatureRemover)
