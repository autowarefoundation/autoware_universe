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

#pragma once

namespace autoware::pointcloud_preprocessor
{

struct CollectorInfoBase
{
  virtual ~CollectorInfoBase() = default;
};

struct NaiveCollectorInfo : public CollectorInfoBase
{
  double timestamp;

  explicit NaiveCollectorInfo(double timestamp = 0.0) : timestamp(timestamp) {}
};

struct AdvancedCollectorInfo : public CollectorInfoBase
{
  double timestamp;
  double noise_window;

  explicit AdvancedCollectorInfo(double timestamp = 0.0, double noise_window = 0.0)
  : timestamp(timestamp), noise_window(noise_window)
  {
  }
};

}  // namespace autoware::pointcloud_preprocessor
