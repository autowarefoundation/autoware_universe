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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__VALIDATOR_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__VALIDATOR_INTERFACE_HPP_

#include "autoware/trajectory_validator/filter_context.hpp"

#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>
#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using VehicleInfo = autoware::vehicle_info_utils::VehicleInfo;
using autoware_trajectory_validator::msg::MetricReport;

/**
 * @brief Result of a validation operation, including feasibility and a list of metrics.
 * @note Default-constructed ValidationResult is feasible with no metrics.
 */
struct ValidationResult
{
  bool is_feasible{true};
  std::vector<MetricReport> metrics{};
};

class ValidatorInterface
{
public:
  using result_t = tl::expected<ValidationResult, std::string>;

  explicit ValidatorInterface(std::string name) : name_(std::move(name)) {}

  virtual ~ValidatorInterface() = default;
  ValidatorInterface(const ValidatorInterface &) = delete;
  ValidatorInterface & operator=(const ValidatorInterface &) = delete;
  ValidatorInterface(ValidatorInterface &&) = delete;
  ValidatorInterface & operator=(ValidatorInterface &&) = delete;

  // Main filter method with context for plugin-specific data
  virtual result_t is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) = 0;

  virtual void update_parameters(const validator::Params & params) = 0;

  // Set vehicle info
  virtual void set_vehicle_info(const VehicleInfo & vehicle_info)
  {
    vehicle_info_ptr_ = std::make_shared<VehicleInfo>(vehicle_info);
  }

  void set_shadow_mode(const bool is_shadow_mode) { is_shadow_mode_ = is_shadow_mode; }
  void set_category(const std::string & category) { category_ = category; }

  [[nodiscard]] std::string category() const { return category_; }

  [[nodiscard]] std::string get_name() const { return name_; }
  [[nodiscard]] bool is_shadow_mode() const { return is_shadow_mode_; }

  /**
   * @brief Atomically retrieve and clear the plugin's accumulated debug markers.
   */
  [[nodiscard]] visualization_msgs::msg::MarkerArray take_debug_markers()
  {
    visualization_msgs::msg::MarkerArray output_markers;
    output_markers.markers.reserve(debug_markers_.markers.size() + 1);

    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    output_markers.markers.push_back(delete_all_marker);

    std::move(
      debug_markers_.markers.begin(), debug_markers_.markers.end(),
      std::back_inserter(output_markers.markers));

    debug_markers_.markers.clear();

    return output_markers;
  }

protected:
  std::string name_;
  bool is_shadow_mode_{false};
  std::string category_;
  std::shared_ptr<VehicleInfo> vehicle_info_ptr_;
  visualization_msgs::msg::MarkerArray debug_markers_;
};
}  // namespace autoware::trajectory_validator::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__VALIDATOR_INTERFACE_HPP_
