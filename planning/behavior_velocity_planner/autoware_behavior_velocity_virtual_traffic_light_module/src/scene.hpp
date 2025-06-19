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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware::behavior_velocity_planner
{
class VirtualTrafficLightModule : public SceneModuleInterface
{
public:
  enum class State : uint8_t {
    NONE = 0,
    REQUESTING = 1,
    PASSING = 2,
    FINALIZING = 3,
    FINALIZED = 4,
  };

  struct MapData
  {
    std::string instrument_type{};
    std::string instrument_id{};
    std::vector<tier4_v2x_msgs::msg::KeyValue> custom_tags{};
    autoware_utils::Point3d instrument_center{};
    std::optional<autoware_utils::LineString3d> stop_line{};
    autoware_utils::LineString3d start_line{};
    std::vector<autoware_utils::LineString3d> end_lines{};
    std::string stop_line_id_for_log{};
  };

  struct ModuleData
  {
    geometry_msgs::msg::Pose head_pose{};
    autoware_internal_planning_msgs::msg::PathWithLaneId path{};
    std::optional<geometry_msgs::msg::Pose> stop_head_pose_at_stop_line;
    std::optional<geometry_msgs::msg::Pose> stop_head_pose_at_end_line;
  };

  struct PlannerParam
  {
    double max_delay_sec;
    double near_line_distance;
    double dead_line_margin;
    double hold_stop_margin_distance;
    double max_yaw_deviation_rad;
    bool check_timeout_after_stop_line;
  };

public:
  VirtualTrafficLightModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::VirtualTrafficLight & reg_elem, lanelet::ConstLanelet lane,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(PathWithLaneId * path) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

  std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> getInfrastructureCommand() const;
  void setInfrastructureCommand(
    const std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command);

  void setCorrespondingVirtualTrafficLightState(
    const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr
      virtual_traffic_light_states);

  void updateLoggerWithState();

  std::vector<int64_t> getRegulatoryElementIds() const override { return {reg_elem_.id()}; }
  std::vector<int64_t> getLaneletIds() const override { return {lane_id_}; }
  std::vector<int64_t> getLineIds() const override
  {
    std::vector<int64_t> line_ids;

    line_ids.push_back(reg_elem_.getStartLine().id());

    if (reg_elem_.getStopLine()) {
      line_ids.push_back(reg_elem_.getStopLine()->id());
    }

    for (const auto & end_line : reg_elem_.getEndLines()) {
      line_ids.push_back(end_line.id());
    }

    return line_ids;
  }

private:
  const int64_t lane_id_;
  const lanelet::autoware::VirtualTrafficLight & reg_elem_;
  const lanelet::ConstLanelet lane_;
  const PlannerParam planner_param_;
  std::optional<tier4_v2x_msgs::msg::VirtualTrafficLightState> virtual_traffic_light_state_;
  State state_{State::NONE};
  tier4_v2x_msgs::msg::InfrastructureCommand command_;
  std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> infrastructure_command_;
  MapData map_data_;
  ModuleData module_data_;
  rclcpp::Logger base_logger_;

  void setModuleState(
    const State new_state, const std::optional<int64_t> end_line_id = std::nullopt);

  template <State StateValue>
  void setModuleState()
  {
    static_assert(
      StateValue != State::FINALIZING && StateValue != State::FINALIZED,
      "FINALIZING and FINALIZED states require end_line_id parameter");
    setModuleState(StateValue);
  }

  template <State StateValue>
  void setModuleState(const int64_t end_line_id)
  {
    static_assert(
      StateValue == State::FINALIZING || StateValue == State::FINALIZED,
      "This overload is only for FINALIZING and FINALIZED states");
    setModuleState(StateValue, end_line_id);
  }

  void updateInfrastructureCommand();

  std::optional<std::pair<size_t, int64_t>> getPathIndexOfFirstEndLine();

  bool isBeforeStartLine(const size_t end_line_idx);

  bool isBeforeStopLine(const size_t end_line_idx);

  bool isAfterAnyEndLine(const size_t end_line_idx);

  bool isNearAnyEndLine(const size_t end_line_idx);

  bool isStateTimeout(const tier4_v2x_msgs::msg::VirtualTrafficLightState & state);

  bool hasRightOfWay(const tier4_v2x_msgs::msg::VirtualTrafficLightState & state);

  void insertStopVelocityAtStopLine(
    autoware_internal_planning_msgs::msg::PathWithLaneId * path, const size_t end_line_idx);

  void insertStopVelocityAtEndLine(
    autoware_internal_planning_msgs::msg::PathWithLaneId * path, const size_t end_line_idx);

  std::string stateToString(State state) const;
};
}  // namespace autoware::behavior_velocity_planner
#endif  // SCENE_HPP_
