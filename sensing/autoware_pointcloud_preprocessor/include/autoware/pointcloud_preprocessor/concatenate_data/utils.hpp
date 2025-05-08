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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_

#include <string>

namespace autoware::pointcloud_preprocessor
{

/**
 * @brief Format a timestamp to a string with 9 decimal places.
 * @param timestamp The timestamp to format.
 * @return A string representation of the timestamp.
 */
std::string format_timestamp(double timestamp);

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__UTILS_HPP_
