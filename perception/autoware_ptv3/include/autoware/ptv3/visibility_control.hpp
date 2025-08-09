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

#ifndef AUTOWARE__PTV3__VISIBILITY_CONTROL_HPP_
#define AUTOWARE__PTV3__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(PTV3_BUILDING_DLL) || defined(PTV3_EXPORTS)
#define PTV3_PUBLIC __declspec(dllexport)
#define PTV3_LOCAL
#else  // defined(PTV3_BUILDING_DLL) || defined(PTV3_EXPORTS)
#define PTV3_PUBLIC __declspec(dllimport)
#define PTV3_LOCAL
#endif  // defined(PTV3_BUILDING_DLL) || defined(PTV3_EXPORTS)
#elif defined(__linux__)
#define PTV3_PUBLIC __attribute__((visibility("default")))
#define PTV3_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define PTV3_PUBLIC __attribute__((visibility("default")))
#define PTV3_LOCAL __attribute__((visibility("hidden")))
#else
#error "Unsupported Build Configuration"
#endif

#endif  // AUTOWARE__PTV3__VISIBILITY_CONTROL_HPP_
