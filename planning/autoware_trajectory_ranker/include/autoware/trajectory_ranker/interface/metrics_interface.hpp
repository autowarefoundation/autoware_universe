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

#ifndef AUTOWARE__TRAJECTORY_RANKER__INTERFACE__METRICS_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__INTERFACE__METRICS_INTERFACE_HPP_

#include "autoware/trajectory_ranker/interface/data_interface.hpp"

#include <autoware_trajectory_ranker/autoware_trajectory_ranker_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

using vehicle_info_utils::VehicleInfo;

/**
 * @brief Base interface class for trajectory evaluation metrics
 *
 * This abstract class defines the interface for all trajectory evaluation metrics.
 * Each metric plugin must inherit from this class and implement the evaluation logic.
 */
class MetricInterface
{
public:
  MetricInterface(const MetricInterface &) = delete;
  MetricInterface(MetricInterface &&) = delete;
  MetricInterface & operator=(const MetricInterface &) = delete;
  MetricInterface & operator=(MetricInterface &&) = delete;
  explicit MetricInterface(std::string name) : name_{std::move(name)} {}

  virtual ~MetricInterface() = default;

  /**
   * @brief Evaluates a trajectory using this metric
   * @param result Trajectory data interface to evaluate
   * @param max_value Maximum value for normalization
   */
  virtual void evaluate(const std::shared_ptr<DataInterface> & result) const = 0;

  /**
   * @brief Checks if this metric measures deviation from ideal
   * @return true if metric measures deviation (lower is better), false otherwise
   */
  virtual bool is_deviation() const = 0;

  /**
   * @brief Initializes the metric with vehicle info and time resolution
   * @param vehicle_info Vehicle parameters (dimensions, wheelbase, etc.)
   * @param node Optional node pointer for parameter access
   */
  void init(
    const std::shared_ptr<VehicleInfo> & vehicle_info,
    const trajectory_ranker_params::Params::Evaluation & params, rclcpp::Node * node = nullptr)
  {
    vehicle_info_ = vehicle_info;
    node_ptr_ = node;
    resolution_ = static_cast<float>(params.sampling_resolution);
    setup_parameters(params);
  }

  /**
   * @brief Setup metric-specific parameters (optional override)
   */
  virtual void setup_parameters(
    [[maybe_unused]] const trajectory_ranker_params::Params::Evaluation & params)
  {
  }

  /**
   * @brief Sets the metric index for storage in result arrays
   * @param index Index position for this metric's results
   */
  void set_index(const size_t index) { index_ = index; }

  /**
   * @brief Gets the metric name
   * @return Metric name string
   */
  std::string name() const { return name_; }

  /**
   * @brief Gets the metric index
   * @return Index position for this metric
   */
  size_t index() const { return index_; }

  virtual double weight() const = 0;

  virtual std::vector<double> decay_weights() const = 0;

protected:
  /**
   * @brief Gets vehicle info
   * @return Shared pointer to vehicle info
   */
  std::shared_ptr<VehicleInfo> vehicle_info() const { return vehicle_info_; }

  /**
   * @brief Gets time resolution
   * @return Time resolution [s]
   */
  float resolution() const { return resolution_; }

  /**
   * @brief Gets node pointer (for parameter access)
   * @return Node pointer
   */
  rclcpp::Node * node() const { return node_ptr_; }

private:
  std::shared_ptr<VehicleInfo> vehicle_info_;

  std::string name_;

  size_t index_;

  float resolution_;

  rclcpp::Node * node_ptr_{nullptr};
};

}  // namespace autoware::trajectory_ranker::metrics

#endif  // AUTOWARE__TRAJECTORY_RANKER__INTERFACE__METRICS_INTERFACE_HPP_
