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
  pet_param_map_ = create_param_map_per_object<PetParams>(node_params);
  rss_param_map_ = create_param_map_per_object<RssParams>(node_params);
}

void CollisionCheckFilter::clear_detection_times()
{
  pet_continuous_times_.clear();
  rss_continuous_times_.clear();
  drac_continuous_times_.clear();
}

std::vector<MetricReport> CollisionCheckFilter::generate_metric_reports(
  const DracArtifact & drac_artifact, const PetArtifact & pet_artifact,
  const RssArtifact & rss_artifact) const
{
  std::vector<MetricReport> reports;

  const auto convert_metrics_level = [](const RiskLevel risk_level) {
    switch (risk_level) {
      case RiskLevel::SAFE:
        return MetricReport::OK;
      case RiskLevel::WARN:
        return MetricReport::WARN;
      case RiskLevel::ERROR:
        return MetricReport::ERROR;
      default:
        throw std::runtime_error("invalid argument");
    }
  };

  const auto add_report =
    [&](const std::string_view metric_name, double metric_value, RiskLevel risk) {
      reports.push_back(
        autoware_trajectory_validator::build<MetricReport>()
          .validator_name(get_name())
          .validator_category(category())
          .metric_name(std::string(metric_name))
          .metric_value(metric_value)
          .level(convert_metrics_level(risk)));
    };

  static constexpr std::array<const char *, 3> kCanonicalTrajectoryTypes = {
    "map_based_predicted_path",
    "constant_curvature_path",
    "diffusion_based_trajectory",
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
    const RiskLevel drac_risk = has_finding ? drac_artifact.risk : RiskLevel::SAFE;
    add_report(fmt::format("DRAC_{}", type), drac_val, drac_risk);
  }

  // PET
  for (const auto * type : kCanonicalTrajectoryTypes) {
    double pet_val = std::numeric_limits<double>::quiet_NaN();
    RiskLevel pet_risk = RiskLevel::SAFE;
    for (const auto & evaluation : pet_artifact.object_evaluations) {
      if (evaluation.detail.object_identification.trajectory_type.find(type) == std::string::npos) {
        continue;
      }
      if (std::isnan(pet_val) || std::abs(evaluation.detail.pet) < std::abs(pet_val)) {
        pet_val = evaluation.detail.pet;
        pet_risk = evaluation.risk;
      }
    }
    add_report(fmt::format("PET_{}", type), pet_val, pet_risk);
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
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (
    (!context.predicted_objects || context.predicted_objects->objects.empty()) &&
    (!context.neural_network_predicted_objects ||
     context.neural_network_predicted_objects->objects.empty())) {
    clear_detection_times();
    return {};  // No objects to check collision with
  }

  if (traj_points.empty()) {
    clear_detection_times();
    return {};  // No trajectory to check
  }

  const auto [pet_artifact, drac_artifact] = collision_timing_assessment::assess(
    traj_points, context, pet_param_map_, drac_param_map_, global_params_, *vehicle_info_ptr_);
  const auto rss_artifact = rss_deceleration::assess(
    traj_points, context, rss_param_map_, global_params_, *vehicle_info_ptr_);

  auto planning_factors = reporter::process_collision_artifacts(
    *context.odometry, pet_artifact, pet_continuous_times_, drac_artifact, drac_continuous_times_,
    rss_artifact, rss_continuous_times_, debug_markers_, global_params_.time_resolution);

  return ValidationResult{
    calc_worst_risk({pet_artifact.risk, drac_artifact.risk, rss_artifact.risk}) != RiskLevel::ERROR,
    generate_metric_reports(drac_artifact, pet_artifact, rss_artifact),
    std::move(planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
