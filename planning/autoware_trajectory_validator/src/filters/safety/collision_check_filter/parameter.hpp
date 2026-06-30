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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using autoware_perception_msgs::msg::ObjectClassification;

inline constexpr const char * kCollisionCheckParamBaseKey = "base";
// Keep in sync with `parameter_struct.yaml`.
inline constexpr std::pair<uint8_t, std::string_view> kObjectClassifications[] = {
  {ObjectClassification::CAR, "car"},
  {ObjectClassification::TRUCK, "truck"},
  {ObjectClassification::BUS, "bus"},
  {ObjectClassification::TRAILER, "trailer"},
  {ObjectClassification::MOTORCYCLE, "motorcycle"},
  {ObjectClassification::BICYCLE, "bicycle"},
  {ObjectClassification::PEDESTRIAN, "pedestrian"},
  {ObjectClassification::UNKNOWN, "unknown"},
  {ObjectClassification::ANIMAL, "animal"},
  {ObjectClassification::HAZARD, "hazard"},
  {ObjectClassification::OVER_DRIVABLE, "over_drivable"},
  {ObjectClassification::UNDER_DRIVABLE, "under_drivable"}};

constexpr std::string_view to_type_string(const uint8_t label)
{
  for (const auto & [label_val, class_name] : kObjectClassifications) {
    if (label_val == label) {
      return class_name;
    }
  }
  throw std::invalid_argument("Unsupported label: " + std::to_string(label));
}

constexpr std::string_view to_type_string(const ObjectClassification & obj)
{
  return to_type_string(obj.label);
}

inline std::string_view to_type_string(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & obj)
{
  return to_type_string(autoware::object_recognition_utils::getHighestProbLabel(obj));
}

template <typename OutT, typename ParamStruct>
OutT extract_labeled_param(const ParamStruct & params_struct, const std::string_view key)
{
  if constexpr (std::is_aggregate_v<ParamStruct>) {
    if (key == kCollisionCheckParamBaseKey) {
      return static_cast<OutT>(params_struct.base);
    }

    using MemberPtr = OutT ParamStruct::*;

    static const std::unordered_map<std::string_view, MemberPtr> mappings = {
      {"car", &ParamStruct::car},
      {"truck", &ParamStruct::truck},
      {"bus", &ParamStruct::bus},
      {"trailer", &ParamStruct::trailer},
      {"motorcycle", &ParamStruct::motorcycle},
      {"bicycle", &ParamStruct::bicycle},
      {"pedestrian", &ParamStruct::pedestrian},
      {"animal", &ParamStruct::animal},
      {"hazard", &ParamStruct::hazard},
      {"over_drivable", &ParamStruct::over_drivable},
      {"under_drivable", &ParamStruct::under_drivable},
      {"unknown", &ParamStruct::unknown}};

    auto it = mappings.find(key);
    if (it == mappings.end()) {
      throw std::invalid_argument("Unknown label key: " + std::string(key));
    }

    auto label_value = params_struct.*(it->second);
    if constexpr (std::is_floating_point_v<OutT>) {
      return static_cast<OutT>(std::isnan(label_value) ? params_struct.base : label_value);
    } else if constexpr (std::is_same_v<OutT, std::string>) {
      return static_cast<OutT>(label_value.empty() ? params_struct.base : label_value);
    } else {
      return static_cast<OutT>(label_value);
    }

  } else {
    return static_cast<OutT>(params_struct);
  }
}

struct EgoFootprintMargin
{
  double lateral{0.0};
  double front{0.0};
  double rear{0.0};
};

struct GlobalParams
{
  double time_resolution{0.1};

  GlobalParams() = default;
  explicit GlobalParams(const validator::Params::CollisionCheck::GlobalSetting & params)
  {
    time_resolution = params.time_resolution;
  }
};

struct PetThreshold
{
  double ego_first_passing_time_gap{1.0};
  double object_first_passing_time_gap{1.0};
};

struct AssessmentTrajectories
{
  bool map_based{true};
  bool constant_curvature{true};
  bool diffusion_based{true};
};

struct DracParams
{
  struct Threshold
  {
    double ego_acceleration{-4.0};
  };

  DracParams() = default;
  DracParams(const validator::Params & node_params, const std::string_view key)
  {
    const auto & drac = node_params.collision_check.drac;
    enable_assessment = extract_labeled_param<bool>(drac.enable_assessment, key);
    assessment_trajectories.map_based =
      enable_assessment && extract_labeled_param<bool>(drac.assessment_trajectories.map_based, key);
    assessment_trajectories.constant_curvature =
      enable_assessment &&
      extract_labeled_param<bool>(drac.assessment_trajectories.constant_curvature, key);
    assessment_trajectories.diffusion_based =
      enable_assessment &&
      extract_labeled_param<bool>(drac.assessment_trajectories.diffusion_based, key);
    ego_total_braking_delay = extract_labeled_param<double>(drac.ego_total_braking_delay, key);
    ego_footprint_margin.lateral = drac.ego_footprint_margin.lateral;
    ego_footprint_margin.front = drac.ego_footprint_margin.front;
    ego_footprint_margin.rear = drac.ego_footprint_margin.rear;
    warn_threshold.ego_acceleration =
      extract_labeled_param<double>(drac.warn_threshold.ego_acceleration, key);
    error_threshold.ego_acceleration =
      extract_labeled_param<double>(drac.error_threshold.ego_acceleration, key);
  }

  bool enable_assessment{false};
  AssessmentTrajectories assessment_trajectories{};
  double ego_total_braking_delay{0.4};
  EgoFootprintMargin ego_footprint_margin{};
  Threshold warn_threshold{-2.0};
  Threshold error_threshold{};
};

struct PetParams
{
  bool enable_assessment{true};
  AssessmentTrajectories assessment_trajectories{};
  double ego_total_braking_delay{0.4};
  EgoFootprintMargin ego_footprint_margin{};
  double ego_assumed_acceleration{-5.0};
  PetThreshold warn_threshold{};
  PetThreshold error_threshold{0.6, 0.3};

  PetParams() = default;
  PetParams(const validator::Params & node_params, const std::string_view key)
  {
    const auto & pet = node_params.collision_check.pet_collision;
    enable_assessment = extract_labeled_param<bool>(pet.enable_assessment, key);
    assessment_trajectories.map_based =
      enable_assessment && extract_labeled_param<bool>(pet.assessment_trajectories.map_based, key);
    assessment_trajectories.constant_curvature =
      enable_assessment &&
      extract_labeled_param<bool>(pet.assessment_trajectories.constant_curvature, key);
    assessment_trajectories.diffusion_based =
      enable_assessment &&
      extract_labeled_param<bool>(pet.assessment_trajectories.diffusion_based, key);
    ego_total_braking_delay = extract_labeled_param<double>(pet.ego_total_braking_delay, key);
    ego_footprint_margin.lateral = pet.ego_footprint_margin.lateral;
    ego_footprint_margin.front = pet.ego_footprint_margin.front;
    ego_footprint_margin.rear = pet.ego_footprint_margin.rear;
    ego_assumed_acceleration = extract_labeled_param<double>(pet.ego_assumed_acceleration, key);

    warn_threshold.ego_first_passing_time_gap =
      extract_labeled_param<double>(pet.warn_threshold.ego_first_passing_time_gap, key);
    warn_threshold.object_first_passing_time_gap =
      extract_labeled_param<double>(pet.warn_threshold.object_first_passing_time_gap, key);
    error_threshold.ego_first_passing_time_gap =
      extract_labeled_param<double>(pet.error_threshold.ego_first_passing_time_gap, key);
    error_threshold.object_first_passing_time_gap =
      extract_labeled_param<double>(pet.error_threshold.object_first_passing_time_gap, key);
  }
};

struct RssParams
{
  struct ErrorThreshold
  {
    double ego_acceleration{-4.0};
  };

  RssParams() = default;
  RssParams(const validator::Params & node_params, const std::string_view key)
  {
    const auto & rss = node_params.collision_check.rss;
    enable_assessment = extract_labeled_param<bool>(rss.enable_assessment, key);
    stop_distance_margin = extract_labeled_param<double>(rss.stop_distance_margin, key);
    ego_total_braking_delay = extract_labeled_param<double>(rss.ego_total_braking_delay, key);
    ego_footprint_margin.lateral = rss.ego_footprint_margin.lateral;
    ego_footprint_margin.front = rss.ego_footprint_margin.front;
    ego_footprint_margin.rear = rss.ego_footprint_margin.rear;
    object_assumed_acceleration =
      extract_labeled_param<double>(rss.object_assumed_acceleration, key);
    error_threshold.ego_acceleration =
      extract_labeled_param<double>(rss.error_threshold.ego_acceleration, key);
  }

  bool enable_assessment{true};
  double stop_distance_margin{2.0};
  double ego_total_braking_delay{0.4};
  EgoFootprintMargin ego_footprint_margin{};
  double object_assumed_acceleration{-1.0};
  ErrorThreshold error_threshold{};
};

template <typename PluginParam>
std::map<std::string_view, PluginParam> create_param_map_per_object(
  const validator::Params & node_params)
{
  std::map<std::string_view, PluginParam> param_map;
  for (const auto & [label_val, class_name] : kObjectClassifications) {
    param_map[class_name] = PluginParam(node_params, class_name);
  }

  param_map[kCollisionCheckParamBaseKey] = PluginParam(node_params, kCollisionCheckParamBaseKey);

  return param_map;
}

using DracParamMap = std::map<std::string_view, DracParams>;
using PetParamMap = std::map<std::string_view, PetParams>;
using RssParamMap = std::map<std::string_view, RssParams>;

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
