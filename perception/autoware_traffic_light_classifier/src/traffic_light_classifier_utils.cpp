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

#include "traffic_light_classifier_utils.hpp"

namespace autoware::traffic_light
{
namespace utils
{

bool is_harsh_backlight(const cv::Mat & img, const double backlight_threshold)
{
  if (img.empty()) {
    return false;
  }
  cv::Mat y_cr_cb;
  cv::cvtColor(img, y_cr_cb, cv::COLOR_RGB2YCrCb);

  const cv::Scalar mean_values = cv::mean(y_cr_cb);
  const double intensity = (mean_values[0] - 112.5) / 112.5;

  return backlight_threshold <= intensity;
}

}  // namespace utils
}  // namespace autoware::traffic_light
