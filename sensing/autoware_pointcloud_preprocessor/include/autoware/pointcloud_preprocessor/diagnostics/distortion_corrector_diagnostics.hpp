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

#include <autoware/pointcloud_preprocessor/diagnostics/diagnostics_base.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>

namespace autoware::pointcloud_preprocessor
{

class DistortionCorrectorDiagnostics : public DiagnosticsBase
{
public:
  DistortionCorrectorDiagnostics(
    int mismatch_count, float mismatch_fraction, bool use_3d_distortion_correction,
    bool update_azimuth_and_distance)
  : mismatch_count(mismatch_count),
    mismatch_fraction(mismatch_fraction),
    use_3d_distortion_correction(use_3d_distortion_correction),
    update_azimuth_and_distance(update_azimuth_and_distance)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value("mismatch_count", mismatch_count);
    interface.add_key_value("mismatch_fraction", std::round(mismatch_fraction * 100.0) / 100.0);
    interface.add_key_value("use_3d_distortion_correction", use_3d_distortion_correction);
    interface.add_key_value("update_azimuth_and_distance", update_azimuth_and_distance);
  }

  int mismatch_count;
  float mismatch_fraction;
  bool use_3d_distortion_correction;
  bool update_azimuth_and_distance;
};

}  // namespace autoware::pointcloud_preprocessor
