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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__COMPOSABLE_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__COMPOSABLE_HPP_

#include "autoware/predicted_path_postprocessor/processor/interface.hpp"
#include "autoware/predicted_path_postprocessor/processor/result.hpp"

#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief A struct representing intermediate report for a processor.
 */
struct IntermediateReport
{
  double processing_time_ms;                                //!< Processing time in milliseconds.
  autoware_perception_msgs::msg::PredictedObjects objects;  //!< Processed predicted objects.
};

/**
 * @brief A class composing multiple processors to process predicted objects.
 */
class ComposableProcessor
{
public:
  using target_type = autoware_perception_msgs::msg::PredictedObjects;
  using report_type = std::unordered_map<std::string, IntermediateReport>;
  using output_with_report_type = std::pair<target_type, report_type>;
  using error_type = ProcessorInterface::error_type;
  using result_type = Result<target_type, error_type>;
  using result_with_report_type = Result<output_with_report_type, error_type>;

  /**
   * @brief Constructor for ComposableProcessor.
   *
   * @param node_ptr Pointer to the ROS node.
   * @param processor_names Vector of processor names.
   */
  explicit ComposableProcessor(
    rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names);

  /**
   * @brief Process predicted objects using a composite filter.
   *
   * @param objects Shared pointer to the input predicted objects.
   * @param context Context information for processing.
   * @return Processed predicted objects.
   */
  result_type process(const target_type::SharedPtr & objects, const Context & context) const;

  /**
   * @brief Process predicted objects and also return intermediate results of each processor.
   * @param objects Predicted objects to process.
   * @param context Context information for processing.
   * @return Processed predicted objects and intermediate reports.
   */
  result_with_report_type process_with_reports(
    const target_type::SharedPtr & objects, const Context & context) const;

private:
  /**
   * @brief Internal helper function that performs the actual processing with optional debug
   * collection.
   * @param objects Predicted objects to process.
   * @param context Context information for processing.
   * @param collect_intermediate Whether to collect intermediate results.
   * @return Processed predicted objects and optionally intermediate reports.
   */
  result_with_report_type process_internal(
    const target_type::SharedPtr & objects, const Context & context,
    bool collect_intermediate) const;

  std::vector<ProcessorInterface::UniquePtr> processors_;  //!< Set of processors.
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>>
    stopwatch_;  //!< Stopwatch for timing.
};
}  // namespace autoware::predicted_path_postprocessor::processor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__COMPOSABLE_HPP_
