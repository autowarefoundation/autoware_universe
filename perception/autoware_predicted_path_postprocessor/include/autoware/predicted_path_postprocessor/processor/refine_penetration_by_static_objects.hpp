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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__REFINE_PENETRATION_BY_STATIC_OBJECTS_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__REFINE_PENETRATION_BY_STATIC_OBJECTS_HPP_

#include "autoware/predicted_path_postprocessor/processor/interface.hpp"
#include "autoware/predicted_path_postprocessor/processor/interpolation.hpp"

#include <limits>
#include <string>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief Processor to refine predicted paths that penetrate static objects.
 */
class RefinePenetrationByStaticObjects final : public ProcessorInterface
{
public:
  /**
   * @brief Constructor for RefinePenetrationByStaticObjects.
   * @param node_ptr Pointer to the ROS 2 node.
   * @param processor_name Name of the processor.
   */
  RefinePenetrationByStaticObjects(rclcpp::Node * node_ptr, const std::string & processor_name);

private:
  /**
   * @brief Check if the context contains objects.
   *
   * @param context The context to be checked.
   * @return result_type Valid result if the context is valid.
   */
  result_type check_context(const Context & context) const noexcept override;

  /**
   * @brief Process the predicted path to refine it by removing penetration with static objects.
   *
   * @param target The predicted path to be processed.
   * @param context The context for the processing.
   * @return The processed predicted path.
   */
  result_type process(target_type & target, const Context & context) override;

  double speed_threshold_;         //!< Speed threshold to determine static objects.
  interpolation_fn interpolator_;  //!< Interpolation function for path refinement.
};
}  // namespace autoware::predicted_path_postprocessor::processor
// clang-format off
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__REFINE_PENETRATION_BY_STATIC_OBJECTS_HPP_  // NOLINT
// clang-format on
