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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__VISIBILITY_CONTROL_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CALIBRATION_STATUS_EXPORT __attribute__((dllexport))
#define CALIBRATION_STATUS_IMPORT __attribute__((dllimport))
#else
#define CALIBRATION_STATUS_EXPORT __declspec(dllexport)
#define CALIBRATION_STATUS_IMPORT __declspec(dllimport)
#endif
#ifdef CALIBRATION_STATUS_BUILDING_LIBRARY
#define CALIBRATION_STATUS_PUBLIC CALIBRATION_STATUS_EXPORT
#else
#define CALIBRATION_STATUS_PUBLIC CALIBRATION_STATUS_IMPORT
#endif
#define CALIBRATION_STATUS_PUBLIC_TYPE CALIBRATION_STATUS_PUBLIC
#define CALIBRATION_STATUS_LOCAL
#else
#define CALIBRATION_STATUS_EXPORT __attribute__((visibility("default")))
#define CALIBRATION_STATUS_IMPORT
#if __GNUC__ >= 4
#define CALIBRATION_STATUS_PUBLIC __attribute__((visibility("default")))
#define CALIBRATION_STATUS_LOCAL __attribute__((visibility("hidden")))
#else
#define CALIBRATION_STATUS_PUBLIC
#define CALIBRATION_STATUS_LOCAL
#endif
#define CALIBRATION_STATUS_PUBLIC_TYPE
#endif

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__VISIBILITY_CONTROL_HPP_
