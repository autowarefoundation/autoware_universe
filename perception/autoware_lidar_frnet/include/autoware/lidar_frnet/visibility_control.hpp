// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_FRNET__VISIBILITY_CONTROL_HPP_
#define AUTOWARE__LIDAR_FRNET__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(LIDAR_FRNET_BUILDING_DLL) || defined(LIDAR_FRNET_EXPORTS)
#define LIDAR_FRNET_PUBLIC __declspec(dllexport)
#define LIDAR_FRNET_LOCAL
#else  // defined(LIDAR_FRNET_BUILDING_DLL) || defined(LIDAR_FRNET_EXPORTS)
#define LIDAR_FRNET_PUBLIC __declspec(dllimport)
#define LIDAR_FRNET_LOCAL
#endif  // defined(LIDAR_FRNET_BUILDING_DLL) || defined(LIDAR_FRNET_EXPORTS)
#elif defined(__linux__)
#define LIDAR_FRNET_PUBLIC __attribute__((visibility("default")))
#define LIDAR_FRNET_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define LIDAR_FRNET_PUBLIC __attribute__((visibility("default")))
#define LIDAR_FRNET_LOCAL __attribute__((visibility("hidden")))
#else
#error "Unsupported Build Configuration"
#endif

#endif  // AUTOWARE__LIDAR_FRNET__VISIBILITY_CONTROL_HPP_
