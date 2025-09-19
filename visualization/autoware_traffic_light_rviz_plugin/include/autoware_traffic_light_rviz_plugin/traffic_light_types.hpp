//
//  Copyright 2025 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE_TRAFFIC_LIGHT_RVIZ_PLUGIN__TRAFFIC_LIGHT_TYPES_HPP_
#define AUTOWARE_TRAFFIC_LIGHT_RVIZ_PLUGIN__TRAFFIC_LIGHT_TYPES_HPP_

#include <lanelet2_core/primitives/Point.h>

#include <string>
#include <vector>

namespace autoware_traffic_light_rviz_plugin
{

struct Point3d
{
  double x;
  double y;
  double z;
};

struct TrafficLightBulbInfo
{
  lanelet::Id id;
  std::string color;
  std::string shape;
  Point3d position;
};

struct TrafficLightInfo
{
  lanelet::Id id;
  Point3d linestring_center;
  std::vector<TrafficLightBulbInfo> bulbs;
};

}  // namespace autoware_traffic_light_rviz_plugin

#endif  // AUTOWARE_TRAFFIC_LIGHT_RVIZ_PLUGIN__TRAFFIC_LIGHT_TYPES_HPP_
