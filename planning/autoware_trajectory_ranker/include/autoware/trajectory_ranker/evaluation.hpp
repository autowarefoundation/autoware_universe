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

#ifndef AUTOWARE__TRAJECTORY_RANKER__EVALUATION_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__EVALUATION_HPP_

#include "autoware/trajectory_ranker/data_structs.hpp"
#include "autoware/trajectory_ranker/interface/data_interface.hpp"
#include "autoware/trajectory_ranker/interface/metrics_interface.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_ranker
{

using route_handler::RouteHandler;
using vehicle_info_utils::VehicleInfo;

/**
 * @brief Trajectory evaluation class that scores and ranks trajectories using multiple metrics
 */
class Evaluator
{
public:
  explicit Evaluator(
    const std::shared_ptr<RouteHandler> & route_handler,
    const std::shared_ptr<VehicleInfo> & vehicle_info, const rclcpp::Logger & logger,
    rclcpp::Node * node = nullptr)
  : plugin_loader_(
      "autoware_trajectory_ranker", "autoware::trajectory_ranker::metrics::MetricInterface"),
    route_handler_{route_handler},
    vehicle_info_{vehicle_info},
    logger_{logger},
    node_ptr_{node}
  {
  }

  /**
   * @brief Dynamically loads a metric plugin
   * @param name Metric plugin name to load
   * @param index Index for this metric in the evaluation
   * @param time_resolution Time resolution for metric evaluation [s]
   */
  void load_metric(const std::string & name, const size_t index, const double time_resolution);

  /**
   * @brief Unloads a metric plugin
   * @param name Metric plugin name to unload
   */
  void unload_metric(const std::string & name);

  /**
   * @brief Adds trajectory data for evaluation
   * @param core_data Core trajectory data to evaluate
   */
  void add(const std::shared_ptr<CoreData> & core_data);

  /**
   * @brief Sets up evaluation with previous trajectory
   * @param previous_points Previous trajectory points for comparison metrics
   */
  void setup(const std::shared_ptr<TrajectoryPoints> & previous_points);

  /**
   * @brief Evaluates all trajectories and returns the best one
   * @param parameters Evaluation parameters (weights, max values, etc.)
   * @param exclude Tag of trajectory to exclude from selection
   * @return Best scoring trajectory interface
   */
  std::shared_ptr<DataInterface> best(
    const std::shared_ptr<EvaluatorParameters> & parameters, const std::string & exclude = "");

  /**
   * @brief Clears all evaluation results
   */
  void clear() { results_.clear(); }

  /**
   * @brief Gets all evaluation results
   * @return Vector of evaluated trajectory interfaces
   */
  std::vector<std::shared_ptr<DataInterface>> results() const { return results_; }

  /**
   * @brief Gets a specific trajectory result by tag
   * @param tag Tag identifier for the trajectory
   * @return Trajectory interface if found, nullptr otherwise
   */
  std::shared_ptr<DataInterface> get(const std::string & tag) const;

protected:
  /**
   * @brief Evaluates all trajectories using loaded metrics
   * @param max_value Maximum values for normalization per metric
   */
  void evaluate(const std::vector<float> & max_value);

  /**
   * @brief Compresses multi-dimensional metric scores
   * @param weight Weight matrix for compression
   */
  void compress(const std::vector<std::vector<float>> & weight);

  /**
   * @brief Normalizes metric scores
   * @param weight Weight matrix for normalization
   */
  void normalize(const std::vector<std::vector<float>> & weight);

  /**
   * @brief Applies final weights to compressed scores
   * @param weight Weight vector for final scoring
   */
  void weighting(const std::vector<float> & weight);

  /**
   * @brief Selects best trajectory from evaluated results
   * @param exclude Tag of trajectory to exclude
   * @return Best scoring trajectory interface
   */
  std::shared_ptr<DataInterface> best(const std::string & exclude = "") const;

  /**
   * @brief Gets route handler
   * @return Shared pointer to route handler
   */
  std::shared_ptr<RouteHandler> route_handler() const { return route_handler_; }

  /**
   * @brief Gets vehicle info
   * @return Shared pointer to vehicle info
   */
  std::shared_ptr<VehicleInfo> vehicle_info() const { return vehicle_info_; }

private:
  pluginlib::ClassLoader<metrics::MetricInterface> plugin_loader_;

  std::vector<std::shared_ptr<metrics::MetricInterface>> plugins_;

  std::vector<std::shared_ptr<DataInterface>> results_;

  std::shared_ptr<RouteHandler> route_handler_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

  rclcpp::Logger logger_;

  rclcpp::Node * node_ptr_{nullptr};
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__EVALUATION_HPP_
