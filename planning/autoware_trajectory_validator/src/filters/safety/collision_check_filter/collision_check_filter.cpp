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

#include "collision_check_filter.hpp"

#include "assessment.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
void CollisionCheckFilter::update_parameters(const validator::Params & node_params)
{
  global_params_ = GlobalParams(node_params.collision_check.global_setting);

  drac_param_map_ = create_param_map_per_object<DracParams>(node_params);
  rss_param_map_ = create_param_map_per_object<RssParams>(node_params);
}

void CollisionCheckFilter::clear_detection_times()
{
  rss_continuous_times_.clear();
  drac_continuous_times_.clear();
}

std::vector<MetricReport> CollisionCheckFilter::generate_metric_reports(
  const DracArtifact & drac_artifact, const RssArtifact & rss_artifact) const
{
  std::vector<MetricReport> reports;

  const auto add_report = [&](
                            const std::string_view metric_name, double metric_value,
                            RiskLevel::_level_type risk_level) {
    RiskLevel risk;
    risk.level = risk_level;
    reports.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(get_name())
        .validator_category(category())
        .metric_name(std::string(metric_name))
        .metric_value(metric_value)
        .risk(risk));
  };

  static constexpr std::array<const char *, 2> kCanonicalTrajectoryTypes = {
    "map_based_predicted_path",
    "constant_curvature_path",
  };

  // DRAC
  for (const auto * type : kCanonicalTrajectoryTypes) {
    bool has_finding = false;
    for (const auto & evaluation : drac_artifact.object_evaluations) {
      if (evaluation.detail.object_identification.trajectory_type.find(type) != std::string::npos) {
        has_finding = true;
        break;
      }
    }
    const double drac_val = has_finding && drac_artifact.required_acceleration.has_value()
                              ? drac_artifact.required_acceleration.value()
                              : std::numeric_limits<double>::quiet_NaN();
    const RiskLevel::_level_type drac_risk = has_finding ? drac_artifact.risk : RiskLevel::SAFE;
    add_report(fmt::format("DRAC_{}", type), drac_val, drac_risk);
  }

  // RSS
  const auto rss_val = [&rss_artifact]() {
    double min_rss = 0.0;
    for (const auto & evaluation : rss_artifact.object_evaluations) {
      const double rss = evaluation.detail.rss_acceleration;
      if (rss < min_rss) {
        min_rss = rss;
      }
    }
    return min_rss;
  }();
  add_report("RSS", rss_val, rss_artifact.risk);

  return reports;
}

CollisionCheckFilter::result_t CollisionCheckFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  const auto & traj_points = candidate_trajectory.points;
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    clear_detection_times();
    return {};  // No objects to check collision with
  }

  if (traj_points.empty()) {
    clear_detection_times();
    return {};  // No trajectory to check
  }

  trajectory::EgoTrajectoryCache ego_trajectory_cache(
    candidate_trajectory, rclcpp::Time(context.predicted_objects->header.stamp),
    rclcpp::Time(context.odometry->header.stamp), global_params_.time_resolution);

  const auto drac_artifact = collision_timing_assessment::assess(
    ego_trajectory_cache, context, drac_param_map_, global_params_, *vehicle_info_ptr_);
  const auto rss_artifact = rss_deceleration::assess(
    ego_trajectory_cache, context, rss_param_map_, global_params_, *vehicle_info_ptr_);

  auto planning_factors = reporter::process_collision_artifacts(
    *context.odometry, drac_artifact, drac_continuous_times_, rss_artifact, rss_continuous_times_,
    debug_markers_, global_params_.time_resolution);

  return ValidationResult{
    calc_worst_risk({drac_artifact.risk, rss_artifact.risk}) != RiskLevel::DANGER,
    generate_metric_reports(drac_artifact, rss_artifact), std::move(planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
