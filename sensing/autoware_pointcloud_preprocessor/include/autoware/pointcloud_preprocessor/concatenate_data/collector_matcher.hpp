// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include "autoware/pointcloud_preprocessor/concatenate_data/matching_policy.hpp"

#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

// Forward declaration of templated class
template <typename MsgTraits>
class CloudCollector;

template <typename MsgTraits>
class CollectorMatcher
{
public:
  virtual ~CollectorMatcher() = default;

  [[nodiscard]] virtual std::optional<std::shared_ptr<CloudCollector<MsgTraits>>>
  match_cloud_to_collector(
    const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
    const IncomingCloudInfo & incoming_cloud_info) const = 0;
  virtual void set_collector_info(
    std::shared_ptr<CloudCollector<MsgTraits>> & collector,
    const IncomingCloudInfo & incoming_cloud_info) = 0;
};

template <typename MsgTraits>
class NaiveCollectorMatcher : public CollectorMatcher<MsgTraits>
{
public:
  explicit NaiveCollectorMatcher(rclcpp::Node & node);
  [[nodiscard]] std::optional<std::shared_ptr<CloudCollector<MsgTraits>>> match_cloud_to_collector(
    const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
    const IncomingCloudInfo & incoming_cloud_info) const override;
  void set_collector_info(
    std::shared_ptr<CloudCollector<MsgTraits>> & collector,
    const IncomingCloudInfo & incoming_cloud_info) override;

private:
  NaiveMatchingPolicy core_;
};

template <typename MsgTraits>
class AdvancedCollectorMatcher : public CollectorMatcher<MsgTraits>
{
public:
  explicit AdvancedCollectorMatcher(rclcpp::Node & node, std::vector<std::string> input_topics);

  [[nodiscard]] std::optional<std::shared_ptr<CloudCollector<MsgTraits>>> match_cloud_to_collector(
    const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
    const IncomingCloudInfo & incoming_cloud_info) const override;
  void set_collector_info(
    std::shared_ptr<CloudCollector<MsgTraits>> & collector,
    const IncomingCloudInfo & incoming_cloud_info) override;

private:
  AdvancedMatchingPolicy core_;
};

}  // namespace autoware::pointcloud_preprocessor

#include "autoware/pointcloud_preprocessor/concatenate_data/collector_matcher.ipp"
