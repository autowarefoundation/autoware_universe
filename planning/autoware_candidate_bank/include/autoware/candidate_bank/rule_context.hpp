// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__CANDIDATE_BANK__RULE_CONTEXT_HPP_
#define AUTOWARE__CANDIDATE_BANK__RULE_CONTEXT_HPP_

#include "autoware/candidate_bank/candidate/trajectory_candidate_manager.hpp"
#include "autoware/candidate_bank/preprocessing/lane_segments.hpp"

#include <map>
#include <memory>
namespace autoware::candidate_bank
{
struct RuleConfig
{
  bool stop_on_red{true}, stop_on_amber{true}, stop_on_unknown{true};
  double stop_margin_m{2.0}, max_stop_line_lateral_distance_m{5.0}, signal_timeout_sec{1.0};
};
struct RuleSnapshot
{
  std::optional<autoware_planning_msgs::msg::Trajectory> route;
  CandidateMapConflictContext conflicts;
  CandidateTrafficRuleContext rules;
};
class RuleContext
{
public:
  RuleConfig rules_;
  void set_map(const std::shared_ptr<const lanelet::LaneletMap> & map)
  {
    lanes_ = std::make_unique<preprocess::LaneSegmentContext>(map);
  }
  void clear_map() { lanes_.reset(); }
  bool ready() const { return static_cast<bool>(lanes_); }
  void clear_signals() { signals_.clear(); }
  void signals(
    autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg,
    const rclcpp::Time & now)
  {
    preprocess::process_traffic_signals({msg}, signals_, now, rules_.signal_timeout_sec);
  }
  RuleSnapshot build(
    const nav_msgs::msg::Odometry &, const autoware_planning_msgs::msg::LaneletRoute &,
    const CandidateBankConfig &, const rclcpp::Time &);

private:
  std::unique_ptr<preprocess::LaneSegmentContext> lanes_;
  std::map<lanelet::Id, preprocess::TrafficSignalStamped> signals_;
};
}  // namespace autoware::candidate_bank

#endif  // AUTOWARE__CANDIDATE_BANK__RULE_CONTEXT_HPP_
