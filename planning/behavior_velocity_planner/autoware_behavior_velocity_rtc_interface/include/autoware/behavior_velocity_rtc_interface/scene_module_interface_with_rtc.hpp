// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_RTC_INTERFACE__SCENE_MODULE_INTERFACE_WITH_RTC_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_RTC_INTERFACE__SCENE_MODULE_INTERFACE_WITH_RTC_HPP_

#include <autoware/behavior_velocity_planner_common/experimental/scene_module_interface.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{

using autoware::rtc_interface::RTCInterface;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::msg::State;

class SceneModuleInterfaceWithRTC : public experimental::SceneModuleInterface
{
public:
  explicit SceneModuleInterfaceWithRTC(
    const int64_t module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);
  virtual ~SceneModuleInterfaceWithRTC() = default;

  void setActivation(const bool activated) { activated_ = activated; }
  void setRTCEnabled(const bool enable_rtc) { rtc_enabled_ = enable_rtc; }
  bool isActivated() const { return activated_; }
  bool isSafe() const { return safe_; }
  double getDistance() const { return distance_; }

protected:
  bool activated_;
  bool safe_;
  bool rtc_enabled_;
  double distance_;

  void setSafe(const bool safe)
  {
    safe_ = safe;
    if (!rtc_enabled_) {
      syncActivation();
    }
  }
  void setDistance(const double distance) { distance_ = distance; }
  void syncActivation() { setActivation(isSafe()); }
};

class SceneModuleManagerInterfaceWithRTC
: public experimental::SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>
{
public:
  SceneModuleManagerInterfaceWithRTC(
    rclcpp::Node & node, const char * module_name, const bool enable_rtc = true);

  void plan(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

protected:
  RTCInterface rtc_interface_;
  std::unordered_map<int64_t, UUID> map_uuid_;

  ObjectsOfInterestMarkerInterface objects_of_interest_marker_interface_;

  virtual void sendRTC(const Time & stamp);

  virtual void setActivation();

  void updateRTCStatus(
    const UUID & uuid, const bool safe, const uint8_t state, const double distance,
    const Time & stamp, const std::optional<bool> override_rtc_auto_mode = std::nullopt)
  {
    rtc_interface_.updateCooperateStatus(
      uuid, safe, state, distance, distance, stamp, false, override_rtc_auto_mode);
  }

  void removeRTCStatus(const UUID & uuid) { rtc_interface_.removeCooperateStatus(uuid); }

  void publishRTCStatus(const Time & stamp)
  {
    rtc_interface_.removeExpiredCooperateStatus();
    rtc_interface_.publishCooperateStatus(stamp);
  }

  UUID getUUID(const int64_t & module_id) const;

  void generate_uuid(const int64_t & module_id);

  void removeUUID(const int64_t & module_id);

  void publishObjectsOfInterestMarker();

  void deleteExpiredModules(const Trajectory & path, const PlannerData & planner_data) override;

  static bool getEnableRTC(rclcpp::Node & node, const std::string & param_name)
  {
    bool enable_rtc = true;

    try {
      enable_rtc = get_or_declare_parameter<bool>(node, "enable_all_modules_auto_mode")
                     ? false
                     : get_or_declare_parameter<bool>(node, param_name);
    } catch (const std::exception & e) {
      enable_rtc = get_or_declare_parameter<bool>(node, param_name);
    }

    return enable_rtc;
  }
};

extern template void
experimental::SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::updateSceneModuleInstances(
  const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data);
extern template void
experimental::SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::modifyPathVelocity(
  Trajectory & path, const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data);
extern template void
experimental::SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::registerModule(
  const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module,
  const PlannerData & planner_data);
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_RTC_INTERFACE__SCENE_MODULE_INTERFACE_WITH_RTC_HPP_
