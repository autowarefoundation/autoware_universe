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

template <typename NodeT = rclcpp::Node>
class GenericCropBoxDiagnostics : public GenericDiagnosticsBase<NodeT>
{
public:
  explicit GenericCropBoxDiagnostics(int skipped_count) : skipped_count_(skipped_count) {}

  void add_to_interface(
    typename GenericDiagnosticsBase<NodeT>::DiagnosticsInterfaceT & interface) const override
  {
    interface.add_key_value("Skipped NaN point count", skipped_count_);
  }

private:
  int skipped_count_;
};

using CropBoxDiagnostics = GenericCropBoxDiagnostics<rclcpp::Node>;

}  // namespace autoware::pointcloud_preprocessor
