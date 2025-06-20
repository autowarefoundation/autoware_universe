// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TIER4_PERCEPTION_RVIZ_PLUGIN__VISIBILITY_CONTROL_HPP_
#define TIER4_PERCEPTION_RVIZ_PLUGIN__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
#if defined(TIER4_PERCEPTION_RVIZ_PLUGIN_BUILDING_DLL) || \
  defined(TIER4_PERCEPTION_RVIZ_PLUGIN_EXPORTS)
#define TIER4_PERCEPTION_RVIZ_PLUGIN_PUBLIC __declspec(dllexport)
#define TIER4_PERCEPTION_RVIZ_PLUGIN_LOCAL
// defined(TIER4_PERCEPTION_RVIZ_PLUGIN_BUILDING_DLL) ||
// defined(TIER4_PERCEPTION_RVIZ_PLUGIN_EXPORTS)
#else
#define TIER4_PERCEPTION_RVIZ_PLUGIN_PUBLIC __declspec(dllimport)
#define TIER4_PERCEPTION_RVIZ_PLUGIN_LOCAL
// defined(TIER4_PERCEPTION_RVIZ_PLUGIN_BUILDING_DLL) ||
// defined(TIER4_PERCEPTION_RVIZ_PLUGIN_EXPORTS)
#endif
#elif defined(__linux__)
#define TIER4_PERCEPTION_RVIZ_PLUGIN_PUBLIC __attribute__((visibility("default")))
#define TIER4_PERCEPTION_RVIZ_PLUGIN_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define TIER4_PERCEPTION_RVIZ_PLUGIN_PUBLIC __attribute__((visibility("default")))
#define TIER4_PERCEPTION_RVIZ_PLUGIN_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
#error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // TIER4_PERCEPTION_RVIZ_PLUGIN__VISIBILITY_CONTROL_HPP_
