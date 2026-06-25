// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/matching_policy.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/traits.hpp"

#include <rclcpp/rclcpp.hpp>

#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
NaiveCollectorMatcher<MsgTraits>::NaiveCollectorMatcher(rclcpp::Node & node)
{
  RCLCPP_INFO(node.get_logger(), "Utilize naive matching strategy");
}

template <typename MsgTraits>
std::optional<std::shared_ptr<CloudCollector<MsgTraits>>>
NaiveCollectorMatcher<MsgTraits>::match_cloud_to_collector(
  const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
  const IncomingCloudInfo & incoming_cloud_info) const
{
  std::vector<CandidateCollectorState> collector_states;
  std::vector<std::shared_ptr<CloudCollector<MsgTraits>>> candidates;
  collector_states.reserve(cloud_collectors.size());
  candidates.reserve(cloud_collectors.size());

  for (const auto & cloud_collector : cloud_collectors) {
    auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(cloud_collector->get_info());
    if (!naive_info) continue;
    collector_states.push_back(
      CandidateCollectorState{
        naive_info->timestamp, 0.0, cloud_collector->topic_exists(incoming_cloud_info.topic_name)});
    candidates.push_back(cloud_collector);
  }

  const auto matched = core_.match(collector_states, incoming_cloud_info);
  if (matched) return candidates[*matched];
  return std::nullopt;
}

template <typename MsgTraits>
void NaiveCollectorMatcher<MsgTraits>::set_collector_info(
  std::shared_ptr<CloudCollector<MsgTraits>> & collector,
  const IncomingCloudInfo & incoming_cloud_info)
{
  const auto reference = core_.reference_for(incoming_cloud_info);
  collector->set_info(std::make_shared<NaiveCollectorInfo>(reference.reference_time));
}

template <typename MsgTraits>
AdvancedCollectorMatcher<MsgTraits>::AdvancedCollectorMatcher(
  rclcpp::Node & node, std::vector<std::string> input_topics)
: core_(
    input_topics,
    node.declare_parameter<std::vector<double>>("matching_strategy.lidar_timestamp_offsets"),
    node.declare_parameter<std::vector<double>>("matching_strategy.lidar_timestamp_noise_window"))
{
  RCLCPP_INFO(node.get_logger(), "Utilize advanced matching strategy");
}

template <typename MsgTraits>
std::optional<std::shared_ptr<CloudCollector<MsgTraits>>>
AdvancedCollectorMatcher<MsgTraits>::match_cloud_to_collector(
  const std::list<std::shared_ptr<CloudCollector<MsgTraits>>> & cloud_collectors,
  const IncomingCloudInfo & incoming_cloud_info) const
{
  std::vector<CandidateCollectorState> collector_states;
  std::vector<std::shared_ptr<CloudCollector<MsgTraits>>> candidates;
  collector_states.reserve(cloud_collectors.size());
  candidates.reserve(cloud_collectors.size());

  for (const auto & cloud_collector : cloud_collectors) {
    auto advanced_info =
      std::dynamic_pointer_cast<AdvancedCollectorInfo>(cloud_collector->get_info());
    if (!advanced_info) continue;
    collector_states.push_back(
      CandidateCollectorState{
        advanced_info->timestamp, advanced_info->noise_window,
        cloud_collector->topic_exists(incoming_cloud_info.topic_name)});
    candidates.push_back(cloud_collector);
  }

  const auto matched = core_.match(collector_states, incoming_cloud_info);
  if (matched) return candidates[*matched];
  return std::nullopt;
}

template <typename MsgTraits>
void AdvancedCollectorMatcher<MsgTraits>::set_collector_info(
  std::shared_ptr<CloudCollector<MsgTraits>> & collector,
  const IncomingCloudInfo & incoming_cloud_info)
{
  const auto reference = core_.reference_for(incoming_cloud_info);
  collector->set_info(
    std::make_shared<AdvancedCollectorInfo>(reference.reference_time, reference.noise_window));
}

}  // namespace autoware::pointcloud_preprocessor
