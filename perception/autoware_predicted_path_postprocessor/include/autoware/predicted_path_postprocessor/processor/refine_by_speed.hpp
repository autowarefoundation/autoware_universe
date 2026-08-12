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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__REFINE_BY_SPEED_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__REFINE_BY_SPEED_HPP_

#include "autoware/predicted_path_postprocessor/processor/interface.hpp"
#include "autoware/predicted_path_postprocessor/processor/interpolation.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief Processor to refine predicted paths based on object speed.
 */
class RefineBySpeed final : public ProcessorInterface
{
public:
  /**
   * @brief Constructor for RefineBySpeed processor.
   * @param node_ptr Pointer to the ROS 2 node.
   * @param processor_name Name of the processor.
   */
  RefineBySpeed(rclcpp::Node * node_ptr, const std::string & processor_name);

private:
  /**
   * @brief Process the predicted object and refine its path based on speed.
   *
   * @param target The predicted object to process.
   * @param context The context for processing.
   * @return The result of the processing.
   */
  result_type process(target_type & target, const Context & context) override;

  double speed_threshold_;         //!< Speed threshold for refinement
  interpolation_fn interpolator_;  //!< Interpolator function for path refinement
};
}  // namespace autoware::predicted_path_postprocessor::processor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__REFINE_BY_SPEED_HPP_
