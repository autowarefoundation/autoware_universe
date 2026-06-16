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

#ifndef PERCEPTION_UTILS__IOU_BEV_NMS_HPP_
#define PERCEPTION_UTILS__IOU_BEV_NMS_HPP_

#include <autoware_perception_msgs/msg/detected_object.hpp>

#include <vector>

namespace perception_utils
{

struct IouBevNmsParams
{
  double search_distance_2d{};
  double iou_threshold{};
};

class IouBevNms
{
public:
  void setParameters(const IouBevNmsParams & params);

  std::vector<autoware_perception_msgs::msg::DetectedObject> apply(
    const std::vector<autoware_perception_msgs::msg::DetectedObject> & input_objects,
    bool sort = false) const;

private:
  bool isTargetPairObject(
    const autoware_perception_msgs::msg::DetectedObject & object1,
    const autoware_perception_msgs::msg::DetectedObject & object2) const;

  IouBevNmsParams params_{};
  double search_distance_2d_sq_{};
};

}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__IOU_BEV_NMS_HPP_
