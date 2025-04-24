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

#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

// Forward declaration of templated class
template <typename MsgTraits>
class CloudCollector;

struct MatchingParams
{
  std::string topic_name;
  double cloud_timestamp;
  double cloud_arrival_time;
};

template <typename MsgTraits>
class CollectorMatchingStrategy
{
public:
  virtual ~CollectorMatchingStrategy() = default;

  [[nodiscard]] virtual std::optional<std::shared_ptr<CloudCollector<MsgTraits>>>
  match_cloud_to_collector(
    const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
    const MatchingParams & params) const = 0;
  virtual void set_collector_info(
    std::shared_ptr<CloudCollector<MsgTraits>> & collector,
    const MatchingParams & matching_params) = 0;
};

template <typename MsgTraits>
class NaiveMatchingStrategy : public CollectorMatchingStrategy<MsgTraits>
{
public:
  explicit NaiveMatchingStrategy(rclcpp::Node & node);
  [[nodiscard]] std::optional<std::shared_ptr<CloudCollector<MsgTraits>>> match_cloud_to_collector(
    const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
    const MatchingParams & params) const override;
  void set_collector_info(
    std::shared_ptr<CloudCollector<MsgTraits>> & collector,
    const MatchingParams & matching_params) override;
};

template <typename MsgTraits>
class AdvancedMatchingStrategy : public CollectorMatchingStrategy<MsgTraits>
{
public:
  explicit AdvancedMatchingStrategy(rclcpp::Node & node, std::vector<std::string> input_topics);

  [[nodiscard]] std::optional<std::shared_ptr<CloudCollector<MsgTraits>>> match_cloud_to_collector(
    const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
    const MatchingParams & params) const override;
  void set_collector_info(
    std::shared_ptr<CloudCollector<MsgTraits>> & collector,
    const MatchingParams & matching_params) override;

private:
  std::unordered_map<std::string, double> topic_to_offset_map_;
  std::unordered_map<std::string, double> topic_to_noise_window_map_;
};

}  // namespace autoware::pointcloud_preprocessor

#include "autoware/pointcloud_preprocessor/concatenate_data/collector_matching_strategy.ipp"
