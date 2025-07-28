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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_

#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "data_structs.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <magic_enum.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::boundary_departure_checker
{

struct FootprintMargin
{
  double lon_m{0.0};
  double lat_m{0.0};

  FootprintMargin operator+(const FootprintMargin & other) const
  {
    return FootprintMargin{lon_m + other.lon_m, lat_m + other.lat_m};
  }
};

struct Input
{
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom;
  lanelet::LaneletMapPtr lanelet_map;
  LaneletRoute::ConstSharedPtr route;
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets shoulder_lanelets;
  Trajectory::ConstSharedPtr reference_trajectory;
  Trajectory::ConstSharedPtr predicted_trajectory;
  std::vector<std::string> boundary_types_to_detect;
};

struct NormalConfig
{
  FootprintMargin footprint_envelop;
};

struct LocalizationConfig : NormalConfig
{
};

struct LongitudinalConfig : NormalConfig
{
  struct LonTracking
  {
    double scale{1.0};
    double extra_margin_m{0.25};
  };
  LonTracking lon_tracking;
};

struct SteeringConfig
{
  double steering_rate_rps{1.0};
};

using AbnormalityConfig =
  std::variant<NormalConfig, LocalizationConfig, LongitudinalConfig, SteeringConfig>;
using AbnormalitiesConfigs = std::unordered_map<AbnormalityType, AbnormalityConfig>;

struct TriggerThreshold
{
  struct MinMax
  {
    double min{-1.0};
    double max{-1.0};
  };
  double brake_delay_s{1.0};
  double dist_error_m{0.25};
  MinMax th_vel_mps{autoware_utils_math::kmph2mps(5.0), autoware_utils_math::kmph2mps(30.0)};
  MinMax th_acc_mps2{-1.0, -2.5};
  MinMax th_jerk_mps3{-1.0, -1.5};
  Side<MinMax> th_dist_to_boundary_m{MinMax{0.001, 5.0}, MinMax{0.001, 5.0}};
};

struct Output
{
  std::map<std::string, double> processing_time_map;
  bool will_leave_lane{};
  bool is_out_of_lane{};
  bool will_cross_boundary{};
  lanelet::ConstLanelets candidate_lanelets;
  TrajectoryPoints resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> vehicle_passing_areas;
};

struct Param
{
  // Old params
  double footprint_margin_scale{};
  double resample_interval{};
  double max_deceleration{};
  double delay_time{};
  double min_braking_distance{};
  // nearest search to ego
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};

  // New params
  TriggerThreshold th_trigger;
  int th_max_lateral_query_num{5};
  double th_point_merge_distance_m{1.0};
  double footprint_extra_margin{0.0};
  double th_cutoff_time_predicted_path_s{4.0};
  double th_cutoff_time_departure_s{2.0};
  double th_cutoff_time_near_boundary_s{3.5};
  AbnormalitiesConfigs abnormality_configs;
  std::vector<std::string> boundary_types_to_detect;
  std::vector<AbnormalityType> abnormality_types_to_compensate;

  template <typename ConfigType>
  tl::expected<std::reference_wrapper<const ConfigType>, std::string> get_abnormality_config(
    const AbnormalityType type) const
  {
    auto it = abnormality_configs.find(type);
    if (it == abnormality_configs.end()) {
      return tl::make_unexpected(std::string(magic_enum::enum_name(type)) + " not exist");
    }

    if (auto ptr = std::get_if<ConfigType>(&it->second)) {
      return *ptr;
    }

    return tl::make_unexpected(
      "Couldn't find the config " + std::string(magic_enum::enum_name(type)));
  }
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
