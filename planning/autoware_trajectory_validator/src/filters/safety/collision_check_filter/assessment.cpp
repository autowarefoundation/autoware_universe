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

#include "assessment.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment
{
std::optional<CollisionDetail> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  PetThreshold pet_find_range, double time_resolution)
{
  const double max_pet_threshold = std::max(
    pet_find_range.ego_first_passing_time_gap, pet_find_range.object_first_passing_time_gap);

  const auto overall_test_index_range = test_trajectory.resolve_covering_index_range(
    {ref_trajectory.getTimes().front() - pet_find_range.object_first_passing_time_gap,
     ref_trajectory.getTimes().back() + pet_find_range.ego_first_passing_time_gap});
  if (!overall_test_index_range) {
    return std::nullopt;
  }

  if (!boost::geometry::intersects(
        ref_trajectory.get_or_compute_overall_envelope(),
        test_trajectory.get_or_compute_envelope(*overall_test_index_range))) {
    return std::nullopt;
  }

  struct CandidateFinding
  {
    double ttc;
    double pet;
    IndexRange ref_index_range;
    IndexRange test_index_range;
  };

  const auto make_collision_detail =
    [&](
      const CandidateFinding & worst_pet,
      const CandidateFinding & first_collision) -> CollisionDetail {
    return CollisionDetail{
      test_trajectory.getObjectIdentification(),
      CollisionTiming{first_collision.ttc, first_collision.pet},
      CollisionTiming{worst_pet.ttc, worst_pet.pet},
      ref_trajectory.getPoses(),
      test_trajectory.getPoses(),
      ref_trajectory.get_or_compute_convex(worst_pet.ref_index_range),
      test_trajectory.get_or_compute_convex(worst_pet.test_index_range)};
  };

  std::optional<CandidateFinding> first_collision_timing{};
  std::optional<CandidateFinding> worst_pet_timing{};
  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    size_t prev_i = (i == 0) ? 0 : i - 1;
    const double ref_start_time = ref_trajectory.getTimes().at(prev_i);
    const double ref_end_time = ref_trajectory.getTimes().at(i);

    const IndexRange ref_index_range{prev_i, i};
    const Box2d & ref_envelope = ref_trajectory.get_or_compute_envelope(ref_index_range);
    const Polygon2d & ref_convex = ref_trajectory.get_or_compute_convex(ref_index_range);

    const double current_pet_limit =
      worst_pet_timing.has_value() ? std::abs(worst_pet_timing->pet) : max_pet_threshold;

    const auto rough_test_index_range = test_trajectory.resolve_covering_index_range(
      {ref_start_time - current_pet_limit, ref_end_time + current_pet_limit});
    if (!rough_test_index_range) {
      continue;
    }

    if (!boost::geometry::intersects(
          ref_envelope, test_trajectory.get_or_compute_envelope(rough_test_index_range.value()))) {
      continue;
    }

    const auto has_intersects = [&](const IndexRange & index_range) -> bool {
      if (!boost::geometry::intersects(
            ref_envelope, test_trajectory.get_or_compute_envelope(index_range))) {
        return false;
      }

      return geometry::intersects_sat(
        ref_convex, test_trajectory.get_or_compute_convex(index_range));
    };

    const auto find_candidate = [&](const double pet_range) -> std::optional<CandidateFinding> {
      for (const double pet : {-pet_range, pet_range}) {
        const auto test_index_range =
          test_trajectory.resolve_covering_index_range({ref_start_time + pet, ref_end_time + pet});
        if (!test_index_range || !has_intersects(test_index_range.value())) {
          continue;
        }

        return CandidateFinding{ref_start_time, pet, ref_index_range, test_index_range.value()};
      }
      return std::nullopt;
    };

    for (double pet_range = 0.0; pet_range < current_pet_limit; pet_range += time_resolution) {
      const auto candidate = find_candidate(pet_range);
      if (!candidate) {
        continue;
      }

      worst_pet_timing = *candidate;
      if (!first_collision_timing.has_value()) {
        first_collision_timing = worst_pet_timing;
      }
      break;
    }
    if (worst_pet_timing.has_value() && worst_pet_timing->pet == 0.0) {
      return make_collision_detail(worst_pet_timing.value(), first_collision_timing.value());
    }
  }

  if (!worst_pet_timing.has_value()) {
    return std::nullopt;
  }

  return make_collision_detail(worst_pet_timing.value(), first_collision_timing.value());
}

struct DracRiskTable
{
  double safe_acceleration_limit{-1.5};
  double danger_acceleration_limit{-3.0};
  bool enable_abandon{false};
};

RiskLevel::_level_type identify_risk_level(
  const std::optional<double> & required_acceleration, const DracRiskTable & risk_table)
{
  if (!required_acceleration.has_value()) {
    return risk_table.enable_abandon ? RiskLevel::HIGH_CAUTION : RiskLevel::FATAL;
  }

  if (required_acceleration >= risk_table.safe_acceleration_limit) {
    return RiskLevel::SAFE;
  } else if (required_acceleration >= risk_table.danger_acceleration_limit) {
    return RiskLevel::DANGER;
  } else {
    return RiskLevel::FATAL;
  }
}

std::pair<std::optional<double>, std::optional<CollisionDetail>> assess_ego_drac(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_trajectory, const DracParams & drac_params,
  const GlobalParams & global_params)
{
  const std::vector<double> ego_acceleration_list{-1.0, -2.0, -3.0, -4.0, -5.0, -6.0};

  std::optional<CollisionDetail> last_detected_collision{};
  for (auto ego_acc : ego_acceleration_list) {
    trajectory::EgoTrajectoryGenerationParams ego_traj_params{
      drac_params.ego_total_braking_delay, ego_acc, drac_params.ego_footprint_margin};
    auto & ego_trajectory = ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    auto detected_collision = find_collision_timing(
      ego_trajectory, object_trajectory, PetThreshold{0.3, 0.3}, global_params.time_resolution);
    if (!detected_collision.has_value()) {
      return {ego_acc, last_detected_collision};
    }
    last_detected_collision = std::move(detected_collision);
  }

  trajectory::EgoTrajectoryGenerationParams limit_ego_traj_params{
    0.0, ego_acceleration_list.back(), drac_params.ego_footprint_margin};
  auto & limit_ego_trajectory =
    ego_trajectory_cache.get_or_compute_trajectory_data(limit_ego_traj_params);
  auto collision_result = find_collision_timing(
    limit_ego_trajectory, object_trajectory, PetThreshold{0.3, 0.3}, global_params.time_resolution);
  if (!collision_result.has_value()) {
    return {ego_acceleration_list.back(), last_detected_collision};
  }

  return {std::nullopt, collision_result};
}

DracEvaluation assess_drac_constant_curvature_ego_first(CollisionDetail && nominal_collision_result)
{
  DracEvaluation evaluation{};
  evaluation.method = "constant_curvature, ego earlier";
  evaluation.risk = RiskLevel::SAFE;
  evaluation.ego_drac_acceleration = std::nullopt;
  evaluation.detail = std::move(nominal_collision_result);
  return evaluation;
}

DracEvaluation assess_drac_constant_curvature_object_first(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_constant_curvature_trajectory, const DracParams & drac_params,
  const GlobalParams & global_params, CollisionDetail && nominal_collision_result)
{
  auto [required_acceleration, last_collision] = assess_ego_drac(
    ego_trajectory_cache, object_constant_curvature_trajectory, drac_params, global_params);

  DracEvaluation evaluation{};
  evaluation.method = "constant_curvature, object earlier";
  evaluation.risk = identify_risk_level(required_acceleration, DracRiskTable{});
  evaluation.ego_drac_acceleration = required_acceleration;
  evaluation.detail = std::move(last_collision).value_or(std::move(nominal_collision_result));
  return evaluation;
}

DracArtifact assess_constant_curvature(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const autoware_perception_msgs::msg::PredictedObject & object, const DracParams & drac_params,
  const GlobalParams & global_params)
{
  DracArtifact drac_artifact{};

  trajectory::EgoTrajectoryGenerationParams ego_traj_params{
    drac_params.ego_total_braking_delay, 0.0, drac_params.ego_footprint_margin};

  const auto & ego_nominal_trajectory =
    ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

  const auto object_constant_curvature_trajectory =
    trajectory::generate_constant_curvature_trajectory(
      object, 0.0, 0.0, rclcpp::Duration::from_seconds(0.0), 1.0, rclcpp::Time{},
      global_params.time_resolution);

  auto nominal_collision_result = find_collision_timing(
    ego_nominal_trajectory, object_constant_curvature_trajectory, PetThreshold{0.3, 0.3},
    global_params.time_resolution);
  if (!nominal_collision_result.has_value()) {
    return drac_artifact;
  }

  if (nominal_collision_result.value().first_collision_timing.pet > 0.0) {
    drac_artifact.merge(
      assess_drac_constant_curvature_ego_first(std::move(nominal_collision_result.value())));
  } else {
    drac_artifact.merge(assess_drac_constant_curvature_object_first(
      ego_trajectory_cache, object_constant_curvature_trajectory, drac_params, global_params,
      std::move(nominal_collision_result.value())));
  }
  return drac_artifact;
}

DracEvaluation assess_drac_ego_prioritized_ego_earlier(CollisionDetail && nominal_collision_result)
{
  DracEvaluation evaluation{};
  evaluation.method = "map_based, ego prioritized, ego earlier";
  evaluation.risk = RiskLevel::SAFE;
  evaluation.ego_drac_acceleration = std::nullopt;
  evaluation.detail = std::move(nominal_collision_result);
  return evaluation;
}

DracEvaluation assess_drac_ego_prioritized_object_earlier(
  CollisionDetail && nominal_collision_result)
{
  DracEvaluation evaluation{};
  evaluation.method = "map_based, ego prioritized, object earlier";
  evaluation.risk = RiskLevel::SAFE;
  evaluation.ego_drac_acceleration = std::nullopt;
  evaluation.detail = std::move(nominal_collision_result);
  return evaluation;
}

DracEvaluation assess_drac_object_prioritized_ego_earlier(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_map_based_trajectory, const DracParams & drac_params,
  const GlobalParams & global_params, CollisionDetail && nominal_collision_result)
{
  // todo(takagi): add object deceleration.

  auto [required_acceleration, last_collision] =
    assess_ego_drac(ego_trajectory_cache, object_map_based_trajectory, drac_params, global_params);

  DracEvaluation evaluation{};
  evaluation.method = "map_based, object prioritized, ego earlier";
  DracRiskTable risk_table;
  risk_table.enable_abandon = true;
  evaluation.risk = identify_risk_level(required_acceleration, risk_table);
  evaluation.ego_drac_acceleration = required_acceleration;
  evaluation.detail = std::move(last_collision).value_or(std::move(nominal_collision_result));
  return evaluation;
}

DracEvaluation assess_drac_object_prioritized_object_earlier(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_map_based_trajectory, const DracParams & drac_params,
  const GlobalParams & global_params, CollisionDetail && nominal_collision_result)
{
  auto [required_acceleration, last_collision] =
    assess_ego_drac(ego_trajectory_cache, object_map_based_trajectory, drac_params, global_params);

  DracEvaluation evaluation{};
  evaluation.method = "map_based, object prioritized, object earlier";
  evaluation.risk = identify_risk_level(required_acceleration, DracRiskTable{});
  evaluation.ego_drac_acceleration = required_acceleration;
  evaluation.detail = std::move(last_collision).value_or(std::move(nominal_collision_result));
  return evaluation;
}

DracArtifact assess_map_based(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & ego_turn_indicator,
  const autoware_perception_msgs::msg::PredictedObject & object, const DracParams & drac_params,
  const GlobalParams & global_params)
{
  DracArtifact drac_artifact{};

  trajectory::EgoTrajectoryGenerationParams ego_traj_params{
    drac_params.ego_total_braking_delay, 0.0, drac_params.ego_footprint_margin};

  for (const auto & obj_predicted_path : object.kinematics.predicted_paths) {
    const auto predicted_path_nominal_trajectory = trajectory::generate_predicted_path_trajectory(
      object, obj_predicted_path, 0.0, 0.0, rclcpp::Duration::from_seconds(0.0), 8.0,
      rclcpp::Time(), global_params.time_resolution);

    const auto & ego_nominal_trajectory =
      ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    auto nominal_collision_result = find_collision_timing(
      ego_nominal_trajectory, predicted_path_nominal_trajectory, PetThreshold{0.3, 0.3},
      global_params.time_resolution);
    if (!nominal_collision_result.has_value()) {
      continue;
    }

    if (ego_turn_indicator.command == autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE) {
      if (nominal_collision_result.value().first_collision_timing.pet > 0.0) {
        drac_artifact.merge(
          assess_drac_ego_prioritized_ego_earlier(std::move(nominal_collision_result.value())));
      } else {
        drac_artifact.merge(
          assess_drac_ego_prioritized_object_earlier(std::move(nominal_collision_result.value())));
      }
    } else {
      if (nominal_collision_result.value().first_collision_timing.pet > 0.0) {
        drac_artifact.merge(assess_drac_object_prioritized_ego_earlier(
          ego_trajectory_cache, predicted_path_nominal_trajectory, drac_params, global_params,
          std::move(nominal_collision_result.value())));
      } else {
        drac_artifact.merge(assess_drac_object_prioritized_object_earlier(
          ego_trajectory_cache, predicted_path_nominal_trajectory, drac_params, global_params,
          std::move(nominal_collision_result.value())));
      }
    }
  }
  return drac_artifact;
}

DracArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & ego_turn_indicator,
  const FilterContext & context, const DracParamMap & drac_param_map,
  const GlobalParams & global_params)
{
  DracArtifact drac_artifact{};

  for (const auto & predicted_object : context.predicted_objects->objects) {
    const auto & drac_params = drac_param_map.at(to_type_string(predicted_object.classification));
    if (!drac_params.enable_assessment) {
      continue;
    }

    if (drac_params.assessment_trajectories.constant_curvature) {
      drac_artifact.merge(assess_constant_curvature(
        ego_trajectory_cache, predicted_object, drac_params, global_params));
    }

    if (drac_params.assessment_trajectories.map_based) {
      drac_artifact.merge(assess_map_based(
        ego_trajectory_cache, ego_turn_indicator, predicted_object, drac_params, global_params));
    }
  }
  return drac_artifact;
}

}  // namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment

namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
{
std::optional<double> compute_distance_to_collision(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto object_footprint =
    geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
  const auto object_envelope = boost::geometry::return_envelope<Box2d>(object_footprint);

  if (!boost::geometry::intersects(
        ego_trajectory.get_or_compute_overall_envelope(), object_envelope)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < ego_trajectory.size(); ++i) {
    const auto prev_i = (i == 0) ? 0 : (i - 1);
    const auto & ego_footprint = ego_trajectory.get_or_compute_convex(IndexRange{prev_i, i});
    if (geometry::intersects_sat(ego_footprint, object_footprint)) {
      return ego_trajectory.getDistances().at(i);
    }
  }

  return std::nullopt;
}

RssDetail assess_required_acceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object, const RssParams & rss_params,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return RssDetail{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const auto distance_to_collision = compute_distance_to_collision(ego_trajectory, object);
  if (!distance_to_collision.has_value()) {
    return RssDetail{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const double obj_long_vel =
    std::clamp(compute_longitudinal_velocity(ego_trajectory.getPoses(), object), 0.0, 30.0);
  const double safe_distance =
    distance_to_collision.value() - rss_params.stop_distance_margin +
    obj_long_vel * obj_long_vel * 0.5 / -rss_params.object_assumed_acceleration -
    ego_long_vel * rss_params.ego_total_braking_delay;

  const double required_deceleration = safe_distance <= 0.0
                                         ? std::numeric_limits<double>::infinity()
                                         : ego_long_vel * ego_long_vel * 0.5 / safe_distance;

  return RssDetail{TrajectoryIdentification{object, stamp}, -required_deceleration};
}

RssArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache, const FilterContext & context,
  const RssParamMap & rss_param_map)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};
  }

  std::vector<RssEvaluation> rss_evaluations{};
  rss_evaluations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto & rss_params = rss_param_map.at(to_type_string(object.classification));
    if (!rss_params.enable_assessment) {
      continue;
    }

    trajectory::EgoTrajectoryGenerationParams ego_traj_params{
      0.0, 0.0, rss_params.ego_footprint_margin};
    const auto & ego_trajectory =
      ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    const auto rss_detail = assess_required_acceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);
    // todo(takagi): fix risk level
    const auto risk_level =
      rss_detail.rss_acceleration < rss_params.error_threshold.ego_acceleration ? RiskLevel::DANGER
                                                                                : RiskLevel::SAFE;
    rss_evaluations.push_back(RssEvaluation{risk_level, rss_detail});
  }

  return RssArtifact{calc_worst_risk(rss_evaluations), std::move(rss_evaluations)};
}
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
