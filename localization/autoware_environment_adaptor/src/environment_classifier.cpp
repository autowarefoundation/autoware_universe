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

#include "environment_classifier.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

namespace autoware::environment_adaptor
{

void EnvironmentClassifier::load_map(const autoware_map_msgs::msg::LaneletMapBin & msg)
{
  const auto lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg));
  is_map_ready_ = true;

  areas_.clear();
  constexpr auto kAreaType = "feature_environment_specify";
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    if (std::string{polygon.attributeOr(lanelet::AttributeName::Type, "none")} != kAreaType) {
      continue;
    }

    EnvironmentArea area;
    area.subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");

    for (const lanelet::ConstPoint3d & p : polygon) {
      area.polygon.outer().push_back(BoostPoint(p.x(), p.y()));
    }
    if (!area.polygon.outer().empty()) {
      area.polygon.outer().push_back(area.polygon.outer().front());
    }
    boost::geometry::correct(area.polygon);

    const std::string factor_str =
      polygon.attributeOr(param_.map_longitudinal_scale_factor_attribute, "");
    if (!factor_str.empty()) {
      try {
        area.map_longitudinal_scale_factor = std::stod(factor_str);
      } catch (const std::exception &) {
        // invalid attribute value; fall back to parameter
      }
    }

    areas_.push_back(area);
  }
}

EnvironmentClassifier::AreaClassification EnvironmentClassifier::classify(
  const geometry_msgs::msg::Point & point) const
{
  if (!is_map_ready_) {
    AreaClassification result;
    result.environment_id = param_.default_environment_id;
    result.longitudinal_scale_factor = param_.default_longitudinal_scale_factor;
    return result;
  }

  const BoostPoint bp(point.x, point.y);
  for (const auto & area : areas_) {
    if (!boost::geometry::within(bp, area.polygon)) {
      continue;
    }

    AreaClassification result;
    result.environment_id = get_environment_id_for_subtype(area.subtype);
    if (area.map_longitudinal_scale_factor.has_value()) {
      result.longitudinal_scale_factor = area.map_longitudinal_scale_factor.value();
    } else {
      result.longitudinal_scale_factor =
        get_longitudinal_scale_factor_for_env_id(result.environment_id);
    }
    return result;
  }

  AreaClassification result;
  result.environment_id = param_.default_environment_id;
  result.longitudinal_scale_factor = param_.default_longitudinal_scale_factor;
  return result;
}

int32_t EnvironmentClassifier::get_environment_id_for_subtype(const std::string & subtype) const
{
  const auto it = param_.area_subtype_to_environment_id.find(subtype);
  if (it != param_.area_subtype_to_environment_id.end()) {
    return it->second;
  }
  return param_.default_environment_id;
}

double EnvironmentClassifier::get_longitudinal_scale_factor_for_env_id(int32_t env_id) const
{
  const auto it = param_.environment_longitudinal_scale_factor_map.find(env_id);
  if (it != param_.environment_longitudinal_scale_factor_map.end()) {
    return it->second;
  }
  return param_.default_longitudinal_scale_factor;
}

}  // namespace autoware::environment_adaptor
