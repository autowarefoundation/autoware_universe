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
bool is_target_trajectory_type(
  const AssessmentTrajectories & options, const std::string & trajectory_type)
{
  if (trajectory_type.find("diffusion_based_trajectory") != std::string::npos) {
    return options.diffusion_based;
  }
  if (trajectory_type.find("constant_curvature_path") != std::string::npos) {
    return options.constant_curvature;
  }
  if (trajectory_type.find("map_based_predicted_path") != std::string::npos) {
    return options.map_based;
  }
  return false;
}

RiskLevel to_pet_risk_level(double pet, const PetThreshold & error_th, const PetThreshold & warn_th)
{
  const bool is_error =
    pet <= error_th.ego_first_passing_time_gap && pet >= -error_th.object_first_passing_time_gap;
  if (is_error) {
    return RiskLevel::ERROR;
  }

  const bool is_warn =
    pet <= warn_th.ego_first_passing_time_gap && pet >= -warn_th.object_first_passing_time_gap;
  return is_warn ? RiskLevel::WARN : RiskLevel::SAFE;
}

RiskLevel to_drac_risk_level(const std::optional<double> & acc, const DracParams & drac_params)
{
  if (!acc.has_value() || acc.value() < drac_params.error_threshold.ego_acceleration) {
    return RiskLevel::ERROR;
  }
  if (acc.value() < drac_params.warn_threshold.ego_acceleration) {
    return RiskLevel::WARN;
  }
  return RiskLevel::SAFE;
}

trajectory::footprint::EgoDimensions make_ego_dimensions(
  const VehicleInfo & vehicle_info, const EgoFootprintMargin & ego_footprint_margin)
{
  return trajectory::footprint::EgoDimensions{
    vehicle_info.max_longitudinal_offset_m + ego_footprint_margin.front,
    -vehicle_info.min_longitudinal_offset_m + ego_footprint_margin.rear,
    vehicle_info.vehicle_width_m + 2.0 * ego_footprint_margin.lateral};
}

std::optional<CollisionDetail> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  PetThreshold pet_find_range, double time_resolution)
{
  const double max_pet_threshold = std::max(
    pet_find_range.ego_first_passing_time_gap, pet_find_range.object_first_passing_time_gap);

  if (!boost::geometry::intersects(
        ref_trajectory.get_or_compute_overall_envelope(),
        test_trajectory.get_or_compute_envelope(
          TimeRange{
            ref_trajectory.getTimes().front() - pet_find_range.object_first_passing_time_gap,
            ref_trajectory.getTimes().back() + pet_find_range.ego_first_passing_time_gap}))) {
    return std::nullopt;
  }

  struct CandidateFinding
  {
    double ttc;
    double pet;
    IndexRange ref_index_range;
    TimeRange test_time_range;
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
      test_trajectory.get_or_compute_convex(worst_pet.test_time_range)};
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

    if (!boost::geometry::intersects(
          ref_envelope, test_trajectory.get_or_compute_envelope(
                          TimeRange{
                            ref_start_time - current_pet_limit,
                            ref_trajectory.getTimes().back() + current_pet_limit}))) {
      continue;
    }

    const auto has_intersects = [&](const TimeRange & time_range) -> bool {
      if (!boost::geometry::intersects(
            ref_envelope, test_trajectory.get_or_compute_envelope(time_range))) {
        return false;
      }

      return geometry::intersects_sat(
        ref_convex, test_trajectory.get_or_compute_convex(time_range));
    };

    for (double pet_range = 0.0; pet_range < current_pet_limit; pet_range += time_resolution) {
      const TimeRange test_time_range_before{ref_start_time - pet_range, ref_end_time - pet_range};
      const bool has_intersects_before = has_intersects(test_time_range_before);

      const TimeRange test_time_range_after{ref_start_time + pet_range, ref_end_time + pet_range};
      const bool has_intersects_after = has_intersects(test_time_range_after);

      if (!has_intersects_before && !has_intersects_after) {
        continue;
      }

      const double pet = has_intersects_before ? -pet_range : pet_range;
      const TimeRange test_time_range =
        has_intersects_before ? test_time_range_before : test_time_range_after;

      worst_pet_timing = CandidateFinding{ref_start_time, pet, ref_index_range, test_time_range};
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

PetArtifact assess_planned_speed_collision_timing(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetParamMap & pet_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info, const std::vector<TrajectoryData> & object_trajectories)
{
  // todo (takagi): ego_trajectory options can not set for each object type because cache structure
  // is not implemented yet.
  const auto & ego_pet_params = pet_param_map.at(kCollisionCheckParamBaseKey);
  const auto ego_dimensions = collision_timing_assessment::make_ego_dimensions(
    vehicle_info, ego_pet_params.ego_footprint_margin);
  const double ego_time_horizon_for_pet = std::abs(context.odometry->twist.twist.linear.x) * 0.5 /
                                            -ego_pet_params.ego_assumed_acceleration +
                                          ego_pet_params.ego_total_braking_delay;
  auto ego_trajectory = trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_pet, global_params.time_resolution, ego_dimensions);

  std::vector<CollisionEvaluation> collision_evaluations{};
  for (const auto & object_trajectory : object_trajectories) {
    const auto & pet_params =
      pet_param_map.at(object_trajectory.getObjectIdentification().classification);
    if (!is_target_trajectory_type(
          pet_params.assessment_trajectories,
          object_trajectory.getObjectIdentification().trajectory_type)) {
      continue;
    }

    auto collision = find_collision_timing(
      ego_trajectory, object_trajectory, pet_params.warn_threshold, global_params.time_resolution);

    if (collision.has_value()) {
      const auto risk_level = to_pet_risk_level(
        collision->worst_pet_timing.pet, pet_params.error_threshold, pet_params.warn_threshold);
      if (risk_level == RiskLevel::SAFE) {
        continue;
      }
      collision_evaluations.push_back(
        CollisionEvaluation{risk_level, std::move(collision.value())});
    }
  }

  return {calc_worst_risk(collision_evaluations), std::move(collision_evaluations)};
}

DracArtifact assess_drac(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const DracParamMap & drac_param_map, const VehicleInfo & vehicle_info,
  const std::vector<TrajectoryData> & object_trajectories, const GlobalParams & global_params)
{
  const double ego_time_horizon = rclcpp::Duration(traj_points.back().time_from_start).seconds();

  // todo (takagi): ego_trajectory options can not set for each object type because cache structure
  // is not implemented yet.
  const auto & ego_drac_params = drac_param_map.at(kCollisionCheckParamBaseKey);
  const auto ego_dimensions =
    make_ego_dimensions(vehicle_info, ego_drac_params.ego_footprint_margin);
  constexpr double default_ego_deceleration_step = 1.0;
  constexpr double default_max_ego_deceleration = 6.0;
  constexpr PetThreshold error_pet_th{0.6, 0.3};

  std::vector<CollisionEvaluation> last_collision_evaluations{};

  for (double ego_dec = 0.0; ego_dec < default_max_ego_deceleration + 1e-3;
       ego_dec += default_ego_deceleration_step) {
    const auto ego_deceleration_trajectory = [&]() {
      if (ego_dec == 0.0) {
        return trajectory::generate_ego_trajectory(
          traj_points, context, ego_time_horizon, global_params.time_resolution, ego_dimensions);
      } else if (ego_dec > default_max_ego_deceleration - 1e-3) {
        return trajectory::generate_ego_trajectory(
          context.odometry->twist.twist, 0.0, -ego_dec, ego_time_horizon,
          global_params.time_resolution, traj_points, ego_dimensions);
      }
      return trajectory::generate_ego_trajectory(
        context.odometry->twist.twist, ego_drac_params.ego_total_braking_delay, -ego_dec,
        ego_time_horizon, global_params.time_resolution, traj_points, ego_dimensions);
    }();

    std::vector<CollisionEvaluation> collision_evaluations{};
    for (const auto & object_trajectory : object_trajectories) {
      const auto & drac_params =
        drac_param_map.at(object_trajectory.getObjectIdentification().classification);
      if (!is_target_trajectory_type(
            drac_params.assessment_trajectories,
            object_trajectory.getObjectIdentification().trajectory_type)) {
        continue;
      }

      constexpr double drac_params_collision_time_threshold = 1.0;
      auto finding_nominal_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_trajectory, error_pet_th,
        global_params.time_resolution);

      const RiskLevel nominal_motion_risk_level =
        finding_nominal_object_motion.has_value()
          ? to_pet_risk_level(
              finding_nominal_object_motion->worst_pet_timing.pet, error_pet_th, error_pet_th)
          : RiskLevel::SAFE;
      if (nominal_motion_risk_level != RiskLevel::ERROR) {
        continue;
      }

      const auto & traj_type_str = object_trajectory.getObjectIdentification().trajectory_type;
      const auto & object_id = object_trajectory.getObjectIdentification().uuid;

      const auto object_deceleration_trajectory = trajectory::generate_object_trajectory(
        context, object_id, traj_type_str, -ego_dec, global_params.time_resolution,
        ego_time_horizon + drac_params_collision_time_threshold);

      auto finding_dec_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_deceleration_trajectory, error_pet_th,
        global_params.time_resolution);

      const RiskLevel dec_motion_risk_level =
        finding_dec_object_motion.has_value()
          ? to_pet_risk_level(
              finding_dec_object_motion->worst_pet_timing.pet, error_pet_th, error_pet_th)
          : RiskLevel::SAFE;

      if (dec_motion_risk_level != RiskLevel::ERROR) {
        continue;
      }

      collision_evaluations.push_back(
        CollisionEvaluation{
          nominal_motion_risk_level, std::move(finding_nominal_object_motion.value())});
      collision_evaluations.push_back(
        CollisionEvaluation{dec_motion_risk_level, std::move(finding_dec_object_motion.value())});
    }
    if (collision_evaluations.empty()) {
      return DracArtifact{
        to_drac_risk_level(-ego_dec, ego_drac_params), -ego_dec,
        std::move(last_collision_evaluations)};
    }

    last_collision_evaluations = std::move(collision_evaluations);
  }

  return DracArtifact{
    to_drac_risk_level(std::nullopt, ego_drac_params), std::nullopt,
    std::move(last_collision_evaluations)};
}

std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration,
  double time_resolution, const DracParamMap & drac_param_map, const PetParamMap & pet_param_map)
{
  std::vector<TrajectoryData> object_trajectories{};

  if (context.predicted_objects) {
    // const auto trajectory_num_per_object =
    //   static_cast<size_t>(options.predicted_path_trajectory) +
    //   static_cast<size_t>(options.constant_curvature_trajectory);
    // object_trajectories.reserve(
    //   object_trajectories.size() +
    //   context.predicted_objects->objects.size() * trajectory_num_per_object);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.predicted_objects->objects) {
      const auto & drac_param = drac_param_map.at(to_type_string(object.classification));
      const auto & pet_param = pet_param_map.at(to_type_string(object.classification));
      const bool is_require_map_based =
        drac_param.assessment_trajectories.map_based || pet_param.assessment_trajectories.map_based;
      if (is_require_map_based && !object.kinematics.predicted_paths.empty()) {
        object_trajectories.push_back(
          trajectory::generate_predicted_path_trajectory(
            object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
            context.predicted_objects->header.stamp, time_resolution));
      }

      if (
        drac_param.assessment_trajectories.constant_curvature ||
        pet_param.assessment_trajectories.constant_curvature) {
        object_trajectories.push_back(
          trajectory::generate_constant_curvature_trajectory(
            object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
            context.predicted_objects->header.stamp, time_resolution));
      }
    }
  }

  if (context.neural_network_predicted_objects) {
    // object_trajectories.reserve(
    //   object_trajectories.size() + context.neural_network_predicted_objects->objects.size());
    const rclcpp::Duration neural_network_objects_reference_time =
      rclcpp::Time(context.neural_network_predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.neural_network_predicted_objects->objects) {
      if (object.kinematics.predicted_paths.empty()) {
        continue;
      }
      const auto & drac_param = drac_param_map.at(to_type_string(object.classification));
      const auto & pet_param = pet_param_map.at(to_type_string(object.classification));
      const bool is_require_diffusion_based = drac_param.assessment_trajectories.diffusion_based ||
                                              pet_param.assessment_trajectories.diffusion_based;
      if (is_require_diffusion_based) {
        object_trajectories.push_back(
          trajectory::generate_diffusion_based_trajectory(
            object, neural_network_objects_reference_time, required_time_horizon,
            context.neural_network_predicted_objects->header.stamp, time_resolution));
      }
    }
  }
  return object_trajectories;
}

std::pair<PetArtifact, DracArtifact> assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetParamMap & pet_param_map, const DracParamMap & drac_param_map,
  const GlobalParams & global_params, const VehicleInfo & vehicle_info)
{
  const double required_time_horizon =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();
  const auto nominal_speed_object_trajectories = generate_object_trajectories(
    context, required_time_horizon, -1.0, global_params.time_resolution, drac_param_map,
    pet_param_map);

  PetArtifact pet_artifact{};
  pet_artifact = assess_planned_speed_collision_timing(
    traj_points, context, pet_param_map, global_params, vehicle_info,
    nominal_speed_object_trajectories);

  DracArtifact drac_artifact{};
  drac_artifact = assess_drac(
    traj_points, context, drac_param_map, vehicle_info, nominal_speed_object_trajectories,
    global_params);
  return {pet_artifact, drac_artifact};
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

TrajectoryData generate_rss_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, const RssParams & rss_params,
  const GlobalParams & global_params, const VehicleInfo & vehicle_info)
{
  const double ego_time_horizon_for_rss =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();
  const auto ego_dimensions =
    collision_timing_assessment::make_ego_dimensions(vehicle_info, rss_params.ego_footprint_margin);

  return trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_rss, global_params.time_resolution, ego_dimensions);
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
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const RssParamMap & rss_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};
  }

  const auto & ego_rss_params = rss_param_map.at(kCollisionCheckParamBaseKey);
  const auto ego_trajectory =
    generate_rss_ego_trajectory(traj_points, context, ego_rss_params, global_params, vehicle_info);

  std::vector<RssEvaluation> rss_evaluations{};
  rss_evaluations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto & rss_params = rss_param_map.at(to_type_string(object.classification));
    if (!rss_params.enable_assessment) {
      continue;
    }
    const auto rss_detail = assess_required_acceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);
    const auto risk_level =
      rss_detail.rss_acceleration < rss_params.error_threshold.ego_acceleration ? RiskLevel::ERROR
                                                                                : RiskLevel::SAFE;
    rss_evaluations.push_back(RssEvaluation{risk_level, rss_detail});
  }

  return RssArtifact{calc_worst_risk(rss_evaluations), std::move(rss_evaluations)};
}
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
