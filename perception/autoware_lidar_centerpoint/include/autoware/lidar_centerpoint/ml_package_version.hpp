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

#ifndef AUTOWARE__LIDAR_CENTERPOINT__ML_PACKAGE_VERSION_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__ML_PACKAGE_VERSION_HPP_

#include <string>

namespace autoware::lidar_centerpoint
{
// Manifest layout this node reads: the major version it is written against, and the minor version
// from which the entries it requires are present.
constexpr int SUPPORTED_ML_PACKAGE_MAJOR_VERSION = 4;
constexpr int MINIMUM_ML_PACKAGE_MINOR_VERSION = 1;

// Accepts "<major>.<minor>" with an optional leading "v"; throws std::invalid_argument when the
// version is unreadable, of another major version, or below the minimum minor version. model_path
// names the bundle in the message, and may be empty.
void check_ml_package_version(const std::string & version, const std::string & model_path);

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__ML_PACKAGE_VERSION_HPP_
