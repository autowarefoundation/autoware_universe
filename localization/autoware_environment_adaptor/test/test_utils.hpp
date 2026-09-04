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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <autoware/lanelet2_utils/conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::environment_adaptor::test_utils
{

inline autoware_map_msgs::msg::LaneletMapBin make_map_bin(
  const std::string & subtype,
  const std::optional<double> map_longitudinal_scale_factor = std::nullopt)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Polygon3d poly(
    lanelet::utils::getId(), {
                               lanelet::Point3d(lanelet::utils::getId(), -1.0, -1.0, 0.0),
                               lanelet::Point3d(lanelet::utils::getId(), 1.0, -1.0, 0.0),
                               lanelet::Point3d(lanelet::utils::getId(), 1.0, 1.0, 0.0),
                               lanelet::Point3d(lanelet::utils::getId(), -1.0, 1.0, 0.0),
                             });
  poly.setAttribute(lanelet::AttributeName::Type, "degenerate_area");
  poly.setAttribute(lanelet::AttributeName::Subtype, subtype);
  if (map_longitudinal_scale_factor.has_value()) {
    poly.setAttribute(
      "longitudinal_scale_factor", std::to_string(map_longitudinal_scale_factor.value()));
  }
  map->add(poly);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

inline std::vector<double> diag_cov(double v)
{
  std::vector<double> c(36, 0.0);
  for (int i = 0; i < 6; ++i) {
    c[i * 6 + i] = v;
  }
  return c;
}

}  // namespace autoware::environment_adaptor::test_utils

#endif  // TEST_UTILS_HPP_
