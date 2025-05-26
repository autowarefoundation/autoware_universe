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
#include "autoware/behavior_path_bidirectional_traffic_module/scene.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/give_way.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/keep_left.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"
#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
namespace autoware::behavior_path_planner
{

BidirectionalTrafficModule::BidirectionalTrafficModule(
  std::string_view name, rclcpp::Node & node,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface{
    name.data(), node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface}
{
}

autoware::universe_utils::Polygon2d BidirectionalTrafficModule::get_ego_polygon(
  const geometry_msgs::msg::Pose & ego_pose) const
{
  return autoware::universe_utils::toFootprint(
    ego_pose, planner_data_->parameters.base_link2front, planner_data_->parameters.base_link2rear,
    planner_data_->parameters.vehicle_width);
}

autoware::universe_utils::Polygon2d BidirectionalTrafficModule::get_ego_polygon() const
{
  return get_ego_polygon(getEgoPose());
}

CandidateOutput BidirectionalTrafficModule::planCandidate() const
{
  // Implementation will be added here in the future.
  return CandidateOutput{};
}

BehaviorModuleOutput BidirectionalTrafficModule::plan()
{
  using tier4_planning_msgs::msg::PathWithLaneId;

  BehaviorModuleOutput module_output = getPreviousModuleOutput();

  PathWithLaneId previous_path = module_output.path;

  auto trajectory =
    trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>::Builder{}.build(
      previous_path.points);

  if (!trajectory) {
    RCLCPP_ERROR(getLogger(), "Failed to build trajectory in BidirectionalTrafficModule::plan");
    return module_output;
  }

  if (bidirectional_lane_intervals_in_current_trajectory_.empty()) {
    return module_output;
  }

  *trajectory = shift_trajectory_for_keep_left(
    *trajectory, bidirectional_lane_intervals_in_current_trajectory_, 0.5, 10.0, 10.0);

  if (give_way_) {
    const double speed = planner_data_->self_odometry->twist.twist.linear.x;
    *trajectory =
      give_way_->modify_trajectory(*trajectory, oncoming_cars_, getEgoPose(), speed < 0.01);
    if (give_way_->is_stop_required()) {
      PoseWithDetail stop_pose(*give_way_->get_stop_pose(), "");
      stop_pose_ = stop_pose;
    } else {
      stop_pose_ = std::nullopt;
    }
  }

  module_output.path.points = trajectory->restore();

  return module_output;
}

BehaviorModuleOutput BidirectionalTrafficModule::planWaitingApproval()
{
  // Implementation will be added here in the future.
  return BehaviorModuleOutput{};
}

bool BidirectionalTrafficModule::isExecutionRequested() const
{
  return !bidirectional_lane_intervals_in_current_trajectory_.empty();
}

bool BidirectionalTrafficModule::isExecutionReady() const
{
  return true;
}

void BidirectionalTrafficModule::processOnEntry()
{
  // Implementation will be added here in the future.
}

void BidirectionalTrafficModule::processOnExit()
{
  // Implementation will be added here in the future.
}

void BidirectionalTrafficModule::updateData()
{
  PathWithLaneId previous_path = getPreviousModuleOutput().path;

  auto trajectory =
    trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>::Builder{}.build(
      previous_path.points);

  if (!trajectory) {
    RCLCPP_ERROR(
      getLogger(), "Failed to build trajectory in BidirectionalTrafficModule::updateData");
    return;
  }

  // Update bidirectional lane intervals in trajectory
  bidirectional_lane_intervals_in_current_trajectory_.clear();
  for (auto & bidirectional_lanes : get_all_bidirectional_lanes_in_map()) {
    std::optional<trajectory::Interval> interval =
      bidirectional_lanes.get_overlap_interval(*trajectory);
    if (interval.has_value()) {
      bidirectional_lane_intervals_in_current_trajectory_.emplace_back(*interval);
    }
  }

  // Check if the ego vehicle is running on the bidirectional lane
  auto new_current_bidirectional_lane = get_current_bidirectional_lane(
    getEgoPose(), get_ego_polygon(), get_all_bidirectional_lanes_in_map());

  if (!current_bidirectional_lane_.has_value() && new_current_bidirectional_lane.has_value()) {
    give_way_.emplace(
      *new_current_bidirectional_lane,          //
      planner_data_->parameters.vehicle_width,  //
      20.0,                                     //
      10.0,                                     //
      0.0);

    RCLCPP_INFO(getLogger(), "Entered Bidirectional Lane");
  }
  if (current_bidirectional_lane_.has_value() && !new_current_bidirectional_lane.has_value()) {
    give_way_.reset();
    RCLCPP_INFO(getLogger(), "Exited Bidirectional Lane");
  }
  current_bidirectional_lane_ = new_current_bidirectional_lane;

  // Update Oncoming Cars
  oncoming_cars_ = OncomingCar::update_oncoming_cars_in_bidirectional_lane(
    oncoming_cars_, *planner_data_->dynamic_object, *current_bidirectional_lane_, getEgoPose());
}

std::vector<ConnectedBidirectionalLanelets>
BidirectionalTrafficModule::get_all_bidirectional_lanes_in_map() const
{
  if (!all_bidirectional_lanes_in_map_.has_value()) {
    const lanelet::LaneletMap & map = *planner_data_->route_handler->getLaneletMapPtr();
    auto get_next_lanelets = [this](const lanelet::ConstLanelet & lanelet) {
      return planner_data_->route_handler->getNextLanelets(lanelet);
    };
    auto get_previous_lanelets = [this](const lanelet::ConstLanelet & lanelet) {
      return planner_data_->route_handler->getPreviousLanelets(lanelet);
    };
    all_bidirectional_lanes_in_map_ =
      ConnectedBidirectionalLanelets::search_bidirectional_lanes_on_map(
        map, get_next_lanelets, get_previous_lanelets);
    RCLCPP_INFO(
      getLogger(), "Found %lu bidirectional lanes", all_bidirectional_lanes_in_map_->size() / 2);
  }
  return all_bidirectional_lanes_in_map_.value();
}

void BidirectionalTrafficModule::acceptVisitor(
  const std::shared_ptr<SceneModuleVisitor> & /* visitor */) const
{
  // Implementation will be added here in the future.
}

void BidirectionalTrafficModule::updateModuleParams(const std::any & /* parameters */)
{
  // Implementation will be added here in the future.
}

bool BidirectionalTrafficModule::canTransitSuccessState()
{
  // Implementation will be added here in the
  return false;
}

bool BidirectionalTrafficModule::canTransitFailureState()
{
  // Implementation will be added here in the
  return false;
}

}  // namespace autoware::behavior_path_planner
