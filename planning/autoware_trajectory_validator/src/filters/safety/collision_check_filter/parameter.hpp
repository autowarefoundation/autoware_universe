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

struct DracParams
{
  struct PetMargin
  {
    double ego_earlier{1.0};
    double object_earlier{1.0};
  };

  struct EgoReactionBrakingDelay
  {
    double nominal{0.4};
    double departure{1.0};
  };

  struct EgoDracAcceleration
  {
    double safe_limit{-1.5};
    double danger_limit{-3.0};
    double fatal_limit{-6.0};
    bool enable_abandon{false};
  };

  struct DracAssessment
  {
    bool enable_assessment{true};
    EgoDracAcceleration ego_drac_assessment{};
  };

  struct ConstantCurvature
  {
    bool enable_assessment{true};
    double object_time_horizon{1.0};
    DracAssessment ego_earlier{};
    DracAssessment object_earlier{};
  };

  struct MapBased
  {
    bool enable_assessment{};
    DracAssessment ego_prioritized_ego_earlier{};
    DracAssessment ego_prioritized_object_earlier{};
    DracAssessment object_prioritized_ego_earlier{};
    DracAssessment object_prioritized_object_earlier{};
  };

  DracParams() = default;
  DracParams(const validator::Params & node_params, const std::string_view key)
  {
    const auto & drac = node_params.collision_check.drac;

    const auto parse_assessment = [&key](const auto & input, DracAssessment & output) {
      output.enable_assessment = extract_labeled_param<bool>(input.enable_assessment, key);

      const auto & input_acceleration = input.ego_drac_acceleration;
      auto & output_acceleration = output.ego_drac_assessment;
      output_acceleration.safe_limit =
        extract_labeled_param<double>(input_acceleration.safe_limit, key);
      output_acceleration.danger_limit =
        extract_labeled_param<double>(input_acceleration.danger_limit, key);
      output_acceleration.fatal_limit =
        extract_labeled_param<double>(input_acceleration.fatal_limit, key);
      output_acceleration.enable_abandon =
        extract_labeled_param<bool>(input_acceleration.enable_abandon, key);
    };

    enable_assessment = extract_labeled_param<bool>(drac.enable_assessment, key);
    pet_margin.ego_earlier = extract_labeled_param<double>(drac.pet_margin.ego_earlier, key);
    pet_margin.object_earlier = extract_labeled_param<double>(drac.pet_margin.object_earlier, key);
    ego_footprint_margin.lateral =
      extract_labeled_param<double>(drac.ego_footprint_margin.lateral, key);
    ego_footprint_margin.front =
      extract_labeled_param<double>(drac.ego_footprint_margin.front, key);
    ego_footprint_margin.rear = extract_labeled_param<double>(drac.ego_footprint_margin.rear, key);
    ego_reaction_braking_delay.nominal =
      extract_labeled_param<double>(drac.ego_reaction_braking_delay.nominal, key);
    ego_reaction_braking_delay.departure =
      extract_labeled_param<double>(drac.ego_reaction_braking_delay.departure, key);

    constant_curvature.enable_assessment =
      extract_labeled_param<bool>(drac.constant_curvature.enable_assessment, key);
    constant_curvature.object_time_horizon =
      extract_labeled_param<double>(drac.constant_curvature.object_time_horizon, key);
    parse_assessment(drac.constant_curvature.ego_earlier, constant_curvature.ego_earlier);
    parse_assessment(drac.constant_curvature.object_earlier, constant_curvature.object_earlier);

    map_based.enable_assessment =
      extract_labeled_param<bool>(drac.map_based.enable_assessment, key);
    parse_assessment(
      drac.map_based.ego_prioritized_ego_earlier, map_based.ego_prioritized_ego_earlier);
    parse_assessment(
      drac.map_based.ego_prioritized_object_earlier, map_based.ego_prioritized_object_earlier);
    parse_assessment(
      drac.map_based.object_prioritized_ego_earlier, map_based.object_prioritized_ego_earlier);
    parse_assessment(
      drac.map_based.object_prioritized_object_earlier,
      map_based.object_prioritized_object_earlier);
  }

  bool enable_assessment{true};
  PetMargin pet_margin{};
  EgoFootprintMargin ego_footprint_margin{};
  EgoReactionBrakingDelay ego_reaction_braking_delay{};
  ConstantCurvature constant_curvature{};
  MapBased map_based{};
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
using RssParamMap = std::map<std::string_view, RssParams>;

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
