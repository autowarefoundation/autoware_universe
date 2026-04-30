#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__PIPELINE_REPORT_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__PIPELINE_REPORT_HPP_

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator
{

using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_trajectory_validator::msg::ValidationReport;

struct PluginEvaluation
{
  std::string plugin_name;
  bool is_feasible{true};
  bool is_shadow_mode{false};
  std::string reason;
};

struct EvaluationTable
{
  std::string generator_id;
  std::unordered_map<std::string, std::vector<PluginEvaluation>> evaluations;

  [[nodiscard]] bool all_acceptable() const
  {
    return all_evaluations([](const auto & e) { return e.is_feasible || e.is_shadow_mode; });
  }

  [[nodiscard]] bool all_feasible() const
  {
    return all_evaluations([](const auto & e) { return e.is_feasible; });
  }

private:
  template <typename Pred>
  bool all_evaluations(Pred && pred) const
  {
    return std::all_of(evaluations.begin(), evaluations.end(), [&](const auto & pair) {
      return std::all_of(pair.second.begin(), pair.second.end(), pred);
    });
  }
};

/**
 * @brief Final opaque result structure returned by the pipeline.
 * CONTRACT FOR ROS NODE ADAPTER:
 * The Node MUST iterate over `evaluation_tables` post-process to:
 * 1. Emit `RCLCPP_WARN_THROTTLE` for any `!evaluation.is_feasible`.
 * 2. Update the `diagnostics_interface_` per plugin using the final evaluation states.
 */
struct PipelineReport
{
  CandidateTrajectories valid_trajectories;
  std::vector<EvaluationTable> evaluation_tables;
  std::vector<ValidationReport> validation_reports; // Published ROS artifact
  size_t num_feasible_trajectories{0};
  
  // Observability / Timing
  std::unordered_map<std::string, double> processing_time_ms;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__PIPELINE_REPORT_HPP_
