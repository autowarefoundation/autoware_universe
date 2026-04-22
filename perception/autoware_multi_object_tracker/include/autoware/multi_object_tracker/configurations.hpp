// Copyright 2020 TIER IV, Inc.
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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__CONFIGURATIONS_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__CONFIGURATIONS_HPP_

#include "autoware/multi_object_tracker/types.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>

namespace autoware::multi_object_tracker
{

//// Online association (measurement <-> tracker matching)

struct AssociatorConfig
{
  struct EnumClassHash
  {
    template <typename T>
    std::size_t operator()(const T value) const
    {
      return static_cast<std::size_t>(value);
    }
  };

  struct TrackerAssociationParameters
  {
    double max_dist_sq;
    double max_area;
    double min_area;
    double min_iou;
  };

  using TrackerAssociationParametersMap =
    std::unordered_map<types::TrackerType, TrackerAssociationParameters, EnumClassHash>;
  using LabelDoubleMap = std::unordered_map<classes::Label, double, EnumClassHash>;
  using LabelToTrackerAssociationParametersMap =
    std::unordered_map<classes::Label, TrackerAssociationParametersMap, EnumClassHash>;

  // Effective association parameters (per measurement label -> tracker type).
  LabelToTrackerAssociationParametersMap association_params_map;

  double unknown_association_giou_threshold;
};

//// Helper: per-label threshold table

struct TrackedLabelThresholds
{
  double unknown;
  double car;
  double truck;
  double bus;
  double trailer;
  double motorcycle;
  double bicycle;
  double pedestrian;

  [[nodiscard]] AssociatorConfig::LabelDoubleMap to_label_map() const
  {
    using Label = classes::Label;
    return {{Label::UNKNOWN, unknown}, {Label::CAR, car},
            {Label::TRUCK, truck},     {Label::BUS, bus},
            {Label::TRAILER, trailer}, {Label::MOTORCYCLE, motorcycle},
            {Label::BICYCLE, bicycle}, {Label::PEDESTRIAN, pedestrian}};
  }
};

//// Tracker overlap manager (tracker-to-tracker layer: remove spatially redundant trackers)

struct TrackerOverlapManagerConfig
{
  float min_known_object_removal_iou;    // ratio [0, 1]
  float min_unknown_object_removal_iou;  // ratio [0, 1]
  AssociatorConfig::LabelDoubleMap pruning_giou_thresholds;
  AssociatorConfig::LabelDoubleMap pruning_distance_thresholds;     // [m]
  AssociatorConfig::LabelDoubleMap pruning_distance_thresholds_sq;  // [m^2]
  double pruning_static_object_speed;                               // [m/s]
  double pruning_moving_object_speed;                               // [m/s]
  double pruning_static_iou_threshold;                              // [ratio]
};

//// Tracker creation (spawning, type mapping)

struct TrackerCreationConfig
{
  using LabelToTrackerTypeMap =
    std::unordered_map<classes::Label, types::TrackerType, AssociatorConfig::EnumClassHash>;

  LabelToTrackerTypeMap tracker_map;
  bool enable_unknown_object_velocity_estimation;
  bool enable_unknown_object_motion_output;
};

//// Utility: safe map lookup

template <typename Map, typename Key>
auto get_map_value_if_exists(const Map & map, const Key & key)
  -> std::optional<std::reference_wrapper<const typename Map::mapped_type>>
{
  const auto it = map.find(key);
  if (it == map.end()) {
    return std::nullopt;
  }
  return std::cref(it->second);
}

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__CONFIGURATIONS_HPP_
