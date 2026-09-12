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

#ifndef ENVIRONMENT_CLASSIFIER_HPP_
#define ENVIRONMENT_CLASSIFIER_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/geometry.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::environment_adaptor
{

class EnvironmentClassifier
{
public:
  using BoostPoint = boost::geometry::model::d2::point_xy<double>;
  using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

  struct EnvironmentArea
  {
    BoostPolygon polygon;
    std::string subtype;
    std::optional<double> map_longitudinal_scale_factor;
  };

  struct AreaClassification
  {
    int32_t environment_id{0};
    double longitudinal_scale_factor{1.0};
  };

  struct Param
  {
    std::unordered_map<std::string, int32_t> area_subtype_to_environment_id;
    int32_t default_environment_id{0};
    double default_longitudinal_scale_factor{1.0};
    std::string map_longitudinal_scale_factor_attribute{"longitudinal_scale_factor"};
  };

  void set_param(const Param & param) { param_ = param; }

  void load_map(const autoware_map_msgs::msg::LaneletMapBin & msg);

  bool is_map_ready() const { return is_map_ready_; }

  AreaClassification classify(const geometry_msgs::msg::Point & point) const;

private:
  int32_t get_environment_id_for_subtype(const std::string & subtype) const;

  Param param_;
  bool is_map_ready_{false};
  std::vector<EnvironmentArea> areas_;
};

}  // namespace autoware::environment_adaptor

#endif  // ENVIRONMENT_CLASSIFIER_HPP_
