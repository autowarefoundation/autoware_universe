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

#include <string>

namespace autoware::pointcloud_preprocessor
{

class DistortionCorrectorDiagnostics : public DiagnosticsBase
{
public:
  DistortionCorrectorDiagnostics(
    int timestamp_mismatch_count, float timestamp_mismatch_fraction,
    bool use_3d_distortion_correction, bool update_azimuth_and_distance,
    float timestamp_mismatch_fraction_threshold)
  : timestamp_mismatch_count_(timestamp_mismatch_count),
    timestamp_mismatch_fraction_(timestamp_mismatch_fraction),
    timestamp_mismatch_fraction_threshold_(timestamp_mismatch_fraction_threshold),
    use_3d_distortion_correction_(use_3d_distortion_correction),
    update_azimuth_and_distance_(update_azimuth_and_distance)
  {
  }

  void add_to_interface(autoware_utils::DiagnosticsInterface & interface) const override
  {
    interface.add_key_value("Timestamp mismatch count", timestamp_mismatch_count_);
    interface.add_key_value(
      "Timestamp mismatch fraction", std::round(timestamp_mismatch_fraction_ * 100.0) / 100.0);
    interface.add_key_value("Use 3D distortion correction", use_3d_distortion_correction_);
    interface.add_key_value("Update azimuth and distance", update_azimuth_and_distance_);
  }

  [[nodiscard]] std::optional<std::pair<int, std::string>> evaluate_status() const override
  {
    if (timestamp_mismatch_fraction_ > timestamp_mismatch_fraction_threshold_) {
      return std::make_pair(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "[ERROR]: timestamp mismatch fraction " + std::to_string(timestamp_mismatch_fraction_) +
          " exceeded threshold of " + std::to_string(timestamp_mismatch_fraction_threshold_));
    }
    return std::nullopt;
  }

private:
  int timestamp_mismatch_count_;
  float timestamp_mismatch_fraction_;
  float timestamp_mismatch_fraction_threshold_;
  bool use_3d_distortion_correction_;
  bool update_azimuth_and_distance_;
};

}  // namespace autoware::pointcloud_preprocessor
