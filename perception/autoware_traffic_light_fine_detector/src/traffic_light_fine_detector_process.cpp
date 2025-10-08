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

#include "traffic_light_fine_detector_process.hpp"

#include <algorithm>

namespace autoware::traffic_light
{

namespace utils
{

float calWeightedIou(
  const sensor_msgs::msg::RegionOfInterest & bbox1, const autoware::tensorrt_yolox::Object & bbox2)
{
  int x1 = std::max(static_cast<int>(bbox1.x_offset), bbox2.x_offset);
  int x2 = std::min(static_cast<int>(bbox1.x_offset + bbox1.width), bbox2.x_offset + bbox2.width);
  int y1 = std::max(static_cast<int>(bbox1.y_offset), bbox2.y_offset);
  int y2 = std::min(static_cast<int>(bbox1.y_offset + bbox1.height), bbox2.y_offset + bbox2.height);
  int area1 = std::max(x2 - x1, 0) * std::max(y2 - y1, 0);
  int area2 = bbox1.width * bbox1.height + bbox2.width * bbox2.height - area1;
  if (area2 == 0) {
    return 0.0;
  }
  return bbox2.score * area1 / area2;
}

bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size)
{
  const int width = static_cast<int>(size.width);
  const int height = static_cast<int>(size.height);
  {
    const int x_min = 0, x_max = width - 2;
    const int y_min = 0, y_max = height - 2;
    lt.x = std::min(std::max(lt.x, x_min), x_max);
    lt.y = std::min(std::max(lt.y, y_min), y_max);
  }
  {
    const int x_min = lt.x + 1, x_max = width - 1;
    const int y_min = lt.y + 1, y_max = height - 1;
    rb.x = std::min(std::max(rb.x, x_min), x_max);
    rb.y = std::min(std::max(rb.y, y_min), y_max);
  }

  return true;
}

}  // namespace utils

}  // namespace autoware::traffic_light
