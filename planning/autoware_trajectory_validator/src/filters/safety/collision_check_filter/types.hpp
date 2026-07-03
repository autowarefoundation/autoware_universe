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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__TYPES_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__TYPES_HPP_

#include "parameter.hpp"

#include <autoware_trajectory_validator/msg/risk_level.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using autoware_trajectory_validator::msg::RiskLevel;
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

static constexpr double TIME_INDEX_EPSILON = 1e-3;

struct TrajectoryIdentification
{
  const std::string classification;
  const builtin_interfaces::msg::Time stamp{};
  const unique_identifier_msgs::msg::UUID uuid{};
  const std::string trajectory_type{};
  const double acceleration{};

  TrajectoryIdentification() = default;
  explicit TrajectoryIdentification(std::string classification)
  : classification(std::move(classification))
  {
  }

  TrajectoryIdentification(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const builtin_interfaces::msg::Time stamp, std::string trajectory_type = {},
    double acceleration = 0.0)
  : classification(to_type_string(object.classification)),
    stamp(stamp),
    uuid(object.object_id),
    trajectory_type(std::move(trajectory_type)),
    acceleration(acceleration)
  {
  }

  std::string object_id_string() const { return autoware_utils_uuid::to_hex_string(uuid); }
  std::string trajectory_id_string() const
  {
    return object_id_string() + "_" + trajectory_type + " acc: " + std::to_string(acceleration);
  }
};

struct CollisionTiming
{
  double ttc;
  double pet;
};

struct CollisionDetail
{
  TrajectoryIdentification object_identification;
  CollisionTiming first_collision_timing;
  CollisionTiming worst_pet_timing;
  std::vector<geometry_msgs::msg::Pose> ego_trajectory;
  std::vector<geometry_msgs::msg::Pose> object_trajectory;
  Polygon2d ego_hull;
  Polygon2d object_hull;
};

struct CollisionEvaluation
{
  RiskLevel::_level_type risk;
  CollisionDetail detail;
};
struct RssDetail
{
  TrajectoryIdentification object_identification;
  double rss_acceleration;
};

struct RssEvaluation
{
  RiskLevel::_level_type risk;
  RssDetail detail;
};

struct DracArtifact
{
  RiskLevel::_level_type risk{RiskLevel::SAFE};
  std::optional<double> required_acceleration;
  std::vector<CollisionEvaluation> object_evaluations;
};

struct RssArtifact
{
  RiskLevel::_level_type risk{RiskLevel::SAFE};
  std::vector<RssEvaluation> object_evaluations;
};

template <typename Container>
RiskLevel::_level_type calc_worst_risk(const Container & evaluations)
{
  RiskLevel::_level_type worst = RiskLevel::SAFE;

  for (const auto & eval : evaluations) {
    if (eval.risk > worst) {
      worst = eval.risk;
    }
    if (worst == RiskLevel::DANGER) {
      break;
    }
  }
  return worst;
}

inline RiskLevel::_level_type calc_worst_risk(std::initializer_list<RiskLevel::_level_type> risks)
{
  RiskLevel::_level_type worst = RiskLevel::SAFE;

  for (const auto & risk : risks) {
    if (risk > worst) {
      worst = risk;
    }
    if (worst == RiskLevel::DANGER) {
      break;
    }
  }
  return worst;
}

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__TYPES_HPP_
