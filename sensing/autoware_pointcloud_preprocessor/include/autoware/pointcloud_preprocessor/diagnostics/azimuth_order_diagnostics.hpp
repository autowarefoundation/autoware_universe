// Copyright 2026 TIER IV, Inc.
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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <optional>
#include <string>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

/**
 * @brief Reports rings whose points are not in scan order.
 *
 * Filters that compare consecutive points of a ring need the points in scan order (the azimuth may
 * increase or decrease). Out-of-order input does not stop the filter, but it can remove valid
 * points, so it is reported as a warning.
 */
template <typename NodeT = rclcpp::Node>
class GenericAzimuthOrderDiagnostics : public GenericDiagnosticsBase<NodeT>
{
public:
  GenericAzimuthOrderDiagnostics(int checked_ring_count, int unordered_ring_count)
  : checked_ring_count_(checked_ring_count), unordered_ring_count_(unordered_ring_count)
  {
  }

  void add_to_interface(
    typename GenericDiagnosticsBase<NodeT>::DiagnosticsInterfaceT & interface) const override
  {
    interface.add_key_value("Checked ring count", checked_ring_count_);
    interface.add_key_value("Ring count not in scan order", unordered_ring_count_);
  }

  [[nodiscard]] std::optional<std::pair<int, std::string>> evaluate_status() const override
  {
    if (unordered_ring_count_ > 0) {
      return std::make_pair(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "[WARN]: " + std::to_string(unordered_ring_count_) +
          " ring(s) are not in scan order (azimuth), valid points may be removed");
    }
    return std::nullopt;
  }

private:
  int checked_ring_count_;
  int unordered_ring_count_;
};

using AzimuthOrderDiagnostics = GenericAzimuthOrderDiagnostics<rclcpp::Node>;

}  // namespace autoware::pointcloud_preprocessor
