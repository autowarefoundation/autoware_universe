// Copyright 2024 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vru/traffic_signal.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

void PredictorVru::setTrafficSignal(const TrafficLightGroupArray & traffic_signal)
{
  traffic_signal_id_map_.clear();
  for (const auto & signal : traffic_signal.traffic_light_groups) {
    traffic_signal_id_map_[signal.traffic_light_group_id] = signal;
  }
}

std::optional<lanelet::Id> PredictorVru::getTrafficSignalId(
  const lanelet::ConstLanelet & way_lanelet)
{
  const auto traffic_light_reg_elems =
    way_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
  if (traffic_light_reg_elems.empty()) {
    return std::nullopt;
  } else if (traffic_light_reg_elems.size() > 1) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Map Based Prediction]: "
      "Multiple regulatory elements as TrafficLight are defined to one lanelet object.");
  }
  return traffic_light_reg_elems.front()->id();
}

bool PredictorVru::calcIntentionToCrossWithTrafficSignal(
  const TrackedObject & object, const lanelet::ConstLanelet & crosswalk,
  const lanelet::Id & signal_id)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto signal_color = [&] {
    const auto elem_opt = getTrafficSignalElement(signal_id);
    return elem_opt ? elem_opt.value().color : TrafficLightElement::UNKNOWN;
  }();

  const auto key = std::make_pair(autoware_utils::to_hex_string(object.object_id), signal_id);
  if (
    signal_color == TrafficLightElement::GREEN &&
    autoware_utils::calc_norm(object.kinematics.twist_with_covariance.twist.linear) <
      threshold_velocity_assumed_as_stopping_) {
    stopped_times_against_green_.try_emplace(key, node_.get_clock()->now());

    const auto timeout_no_intention_to_walk = [&]() {
      auto InterpolateMap = [](
                              const std::vector<double> & key_set,
                              const std::vector<double> & value_set, const double query) {
        if (query <= key_set.front()) {
          return value_set.front();
        } else if (query >= key_set.back()) {
          return value_set.back();
        }
        for (size_t i = 0; i < key_set.size() - 1; ++i) {
          if (key_set.at(i) <= query && query <= key_set.at(i + 1)) {
            auto ratio =
              (query - key_set.at(i)) / std::max(key_set.at(i + 1) - key_set.at(i), 1.0e-5);
            ratio = std::clamp(ratio, 0.0, 1.0);
            return value_set.at(i) + ratio * (value_set.at(i + 1) - value_set.at(i));
          }
        }
        return value_set.back();
      };

      const auto obj_position = object.kinematics.pose_with_covariance.pose.position;
      const double distance_to_crosswalk = boost::geometry::distance(
        crosswalk.polygon2d().basicPolygon(),
        lanelet::BasicPoint2d(obj_position.x, obj_position.y));
      return InterpolateMap(
        distance_set_for_no_intention_to_walk_, timeout_set_for_no_intention_to_walk_,
        distance_to_crosswalk);
    }();

    if (
      (node_.get_clock()->now() - stopped_times_against_green_.at(key)).seconds() >
      timeout_no_intention_to_walk) {
      return false;
    }

  } else {
    stopped_times_against_green_.erase(key);
    // If the pedestrian disappears, another function erases the old data.
  }

  if (signal_color == TrafficLightElement::RED) {
    return false;
  }

  return true;
}

std::optional<TrafficLightElement> PredictorVru::getTrafficSignalElement(const lanelet::Id & id)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (traffic_signal_id_map_.count(id) != 0) {
    const auto & signal_elements = traffic_signal_id_map_.at(id).elements;
    if (signal_elements.size() > 1) {
      RCLCPP_ERROR(
        node_.get_logger(), "[Map Based Prediction]: Multiple TrafficSignalElement_ are received.");
    } else if (!signal_elements.empty()) {
      return signal_elements.front();
    }
  }
  return std::nullopt;
}

}  // namespace autoware::map_based_prediction
