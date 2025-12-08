// Copyright 2025 TIER IV.
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

#ifndef UTILS__CONSTANTS_HPP_
#define UTILS__CONSTANTS_HPP_

namespace autoware::tensorrt_vad::utils::constants
{

// SUPPORTED_MAJOR_VERSION should match the major version in vad-carla-tiny.param.json
// Models with different major versions are NOT compatible with this node
constexpr int SUPPORTED_MAJOR_VERSION = 0;

}  // namespace autoware::tensorrt_vad::utils::constants

#endif  // UTILS__CONSTANTS_HPP_
