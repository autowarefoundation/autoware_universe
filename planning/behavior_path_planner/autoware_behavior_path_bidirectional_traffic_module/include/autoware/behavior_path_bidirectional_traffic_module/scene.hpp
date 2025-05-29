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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__SCENE_HPP_

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/give_way.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

class BidirectionalTrafficModule : public SceneModuleInterface
{
public:
  BidirectionalTrafficModule(
    std::string_view name, rclcpp::Node & node,
    const std::shared_ptr<BidirectionalTrafficModuleParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface);
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void updateData() override;
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;
  void updateModuleParams(const std::any & parameters) override;
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override;
  MarkerArray getModuleVirtualWall() override { return MarkerArray{}; }

  void initialize_all_member_variables();

  std::vector<ConnectedBidirectionalLanelets::SharedConstPtr> get_all_bidirectional_lanes_in_map()
    const;

  lanelet::Ids get_all_bidirectional_lane_ids_in_map() const;

private:
  const std::shared_ptr<const BidirectionalTrafficModuleParameters> parameters_;

  mutable std::optional<std::vector<ConnectedBidirectionalLanelets::SharedConstPtr>>
    all_bidirectional_lanes_in_map_;

  std::optional<ConnectedBidirectionalLanelets::SharedConstPtr>
    bidirectional_lane_where_ego_was_;  //!< current bidirectional lane

  bool has_trajectory_bidirectional_lane_overlap_;

  std::optional<GiveWay> give_way_;  //!< give way state machine

  std::optional<OncomingCars> oncoming_cars_;  //!< circumstance of the ego vehicle
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__SCENE_HPP_
