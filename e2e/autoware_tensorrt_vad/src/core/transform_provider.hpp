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

#ifndef CORE__TRANSFORM_PROVIDER_HPP_
#define CORE__TRANSFORM_PROVIDER_HPP_

#include <Eigen/Dense>

#include <optional>
#include <string>

namespace autoware::tensorrt_vad
{

class TransformProvider
{
public:
  virtual ~TransformProvider() = default;

  virtual std::optional<Eigen::Matrix4f> lookup_base2cam(
    const std::string & source_frame) const = 0;
};

}  // namespace autoware::tensorrt_vad

#endif  // CORE__TRANSFORM_PROVIDER_HPP_
