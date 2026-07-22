// Copyright 2026 The Autoware Foundation
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__RELEVANCE_CLASSIFIER_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__RELEVANCE_CLASSIFIER_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::map_based_prediction
{

/**
 * @brief Relevance of an object to the ego vehicle's planned trajectory.
 *
 * HIGH relevance objects receive full-fidelity prediction (lanelet search, maneuver
 * detection, multi-mode paths). LOW relevance objects receive a lightweight
 * constant-velocity prediction with a shorter time horizon. Nothing is dropped.
 */
enum class Relevance { HIGH, LOW };

struct RelevanceParams
{
  bool enable{false};
  double ego_trajectory_timeout{1.0};      //!< [s] corridor older than this is ignored (fail-safe)
  double always_relevant_radius{20.0};     //!< [m] objects within this distance of ego are HIGH
  double lateral_margin_base{3.0};         //!< [m] corridor half width at the ego position
  double lateral_margin_rate{0.1};         //!< [m/m] corridor half width growth per arc length
  double time_to_corridor_threshold{5.0};  //!< [s] approaching objects closer in time are HIGH
  int demote_frame_count{5};               //!< [frames] consecutive LOW frames required to demote
  double low_fidelity_time_horizon{3.0};   //!< [s] prediction horizon for LOW relevance objects
};

/**
 * @brief Distance of a point to a corridor polyline.
 */
struct CorridorDistance
{
  double lateral_distance;  //!< [m] distance from the point to the nearest polyline point
  double arc_length;        //!< [m] arc length of the nearest polyline point from the start
  geometry_msgs::msg::Point nearest_point;  //!< nearest foot point on the polyline
};

/**
 * @brief Compute the lateral distance and arc length of a point relative to a polyline.
 *
 * The nearest point is searched over all segments; queries beyond the polyline ends are
 * clamped to the end points.
 *
 * @param polyline corridor center line in the map frame (at least 2 points)
 * @param point query position in the map frame
 * @return distance decomposition, or std::nullopt if the polyline has fewer than 2 points
 */
std::optional<CorridorDistance> computeCorridorDistance(
  const std::vector<geometry_msgs::msg::Point> & polyline, const geometry_msgs::msg::Point & point);

/**
 * @brief Classifies tracked objects by their relevance to the ego planned trajectory.
 *
 * This class embodies a "perception in plan" design: the planning intent (the previous
 * planning cycle's trajectory) is fed back to perception so that computational resources
 * are concentrated on the traffic participants that matter for the plan.
 *
 * Safety design:
 * - Fail-safe: without a fresh ego trajectory every object is HIGH (identical to the
 *   behavior when the feature is disabled).
 * - Hysteresis: promotion to HIGH is immediate; demotion to LOW requires
 *   demote_frame_count consecutive LOW classifications.
 * - Approach detection: objects moving toward the corridor are promoted before they
 *   enter it (cut-in and crossing precaution).
 */
class RelevanceClassifier
{
public:
  explicit RelevanceClassifier(const RelevanceParams & params);

  void setParams(const RelevanceParams & params) { params_ = params; }
  const RelevanceParams & getParams() const { return params_; }

  /**
   * @brief Set the ego data used as the planning prior for the current cycle.
   * @param trajectory_points planned trajectory positions in the map frame
   * @param ego_position current ego position in the map frame
   * @param trajectory_time [s] stamp of the trajectory message
   */
  void setEgoData(
    const std::vector<geometry_msgs::msg::Point> & trajectory_points,
    const geometry_msgs::msg::Point & ego_position, const double trajectory_time);

  /**
   * @brief Classify one object and update its hysteresis state.
   * @param object tracked object (pose in the map frame)
   * @param current_time [s] stamp of the tracked objects message
   */
  Relevance classify(const TrackedObject & object, const double current_time);

  /**
   * @brief Remove hysteresis entries not observed within buffer_time.
   */
  void removeOldHistory(const double current_time, const double buffer_time);

private:
  struct ObjectState
  {
    Relevance relevance{Relevance::HIGH};
    int consecutive_low_count{0};
    double last_seen_time{0.0};
  };

  /**
   * @brief Stateless single-frame classification (no hysteresis).
   */
  Relevance classifyOnce(const TrackedObject & object) const;

  RelevanceParams params_;
  std::vector<geometry_msgs::msg::Point> trajectory_points_;
  geometry_msgs::msg::Point ego_position_;
  double trajectory_time_{0.0};
  bool has_ego_data_{false};
  std::unordered_map<std::string, ObjectState> object_states_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__RELEVANCE_CLASSIFIER_HPP_
