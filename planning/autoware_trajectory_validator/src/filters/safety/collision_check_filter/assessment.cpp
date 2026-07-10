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
  if (trajectory_type.find("constant_curvature_path") != std::string::npos) {
    return options.constant_curvature;
  }
  if (trajectory_type.find("map_based_predicted_path") != std::string::npos) {
    return options.map_based;
  }
  return false;
}

RiskLevel::_level_type to_pet_risk_level(
  double pet, const PetThreshold & error_th, const PetThreshold & warn_th)
{
  const bool is_error =
    pet <= error_th.ego_first_passing_time_gap && pet >= -error_th.object_first_passing_time_gap;
  if (is_error) {
    return RiskLevel::DANGER;
  }

  const bool is_warn =
    pet <= warn_th.ego_first_passing_time_gap && pet >= -warn_th.object_first_passing_time_gap;
  return is_warn ? RiskLevel::HIGH_CAUTION : RiskLevel::SAFE;
}

RiskLevel::_level_type to_drac_risk_level(
  const std::optional<double> & acc, const DracParams & drac_params)
{
  if (!acc.has_value() || acc.value() < drac_params.error_threshold.ego_acceleration) {
    return RiskLevel::DANGER;
  }
  if (acc.value() < drac_params.warn_threshold.ego_acceleration) {
    return RiskLevel::HIGH_CAUTION;
  }
  return RiskLevel::SAFE;
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

DracArtifact assess_drac(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache, const FilterContext & context,
  const DracParamMap & drac_param_map, const VehicleInfo & vehicle_info,
  const std::vector<TrajectoryData> & object_trajectories, const GlobalParams & global_params)
{
  // todo (takagi): ego_trajectory options can not set for each object type because cache structure
  // is not implemented yet.
  const auto & ego_drac_params = drac_param_map.at(kCollisionCheckParamBaseKey);
  const auto ego_dimensions =
    trajectory::footprint::make_ego_dimensions(vehicle_info, ego_drac_params.ego_footprint_margin);
  constexpr double default_ego_deceleration_step = 1.0;
  constexpr double default_max_ego_deceleration = 6.0;
  constexpr PetThreshold error_pet_th{0.6, 0.3};

  std::vector<CollisionEvaluation> last_collision_evaluations{};

  for (double ego_dec = 0.0; ego_dec < default_max_ego_deceleration + 1e-3;
       ego_dec += default_ego_deceleration_step) {
    const auto & ego_deceleration_trajectory = ego_trajectory_cache.get_or_compute_trajectory_data(
      trajectory::EgoTrajectoryGenerationParams{0.0, -ego_dec, ego_dimensions});

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

      const RiskLevel::_level_type nominal_motion_risk_level =
        finding_nominal_object_motion.has_value()
          ? to_pet_risk_level(
              finding_nominal_object_motion->worst_pet_timing.pet, error_pet_th, error_pet_th)
          : RiskLevel::SAFE;
      if (nominal_motion_risk_level != RiskLevel::DANGER) {
        continue;
      }

      const auto & traj_type_str = object_trajectory.getObjectIdentification().trajectory_type;
      const auto & object_id = object_trajectory.getObjectIdentification().uuid;

      constexpr double obj_trajectory_time_horizon = 8.0;
      const auto object_deceleration_trajectory = trajectory::generate_object_trajectory(
        context, object_id, traj_type_str, -ego_dec, global_params.time_resolution,
        obj_trajectory_time_horizon + drac_params_collision_time_threshold);

      auto finding_dec_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_deceleration_trajectory, error_pet_th,
        global_params.time_resolution);

      const RiskLevel::_level_type dec_motion_risk_level =
        finding_dec_object_motion.has_value()
          ? to_pet_risk_level(
              finding_dec_object_motion->worst_pet_timing.pet, error_pet_th, error_pet_th)
          : RiskLevel::SAFE;

      if (dec_motion_risk_level != RiskLevel::DANGER) {
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
  double time_resolution, const DracParamMap & drac_param_map)
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
      if (
        drac_param.assessment_trajectories.map_based &&
        !object.kinematics.predicted_paths.empty()) {
        object_trajectories.push_back(
          trajectory::generate_predicted_path_trajectory(
            object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
            context.predicted_objects->header.stamp, time_resolution));
      }

      if (drac_param.assessment_trajectories.constant_curvature) {
        object_trajectories.push_back(
          trajectory::generate_constant_curvature_trajectory(
            object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
            context.predicted_objects->header.stamp, time_resolution));
      }
    }
  }
  return object_trajectories;
}

DracArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache, const FilterContext & context,
  const DracParamMap & drac_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info)
{
  constexpr double obj_trajectory_time_horizon = 8.0;
  constexpr double obj_nominal_acc = -1.0;
  const auto nominal_speed_object_trajectories = generate_object_trajectories(
    context, obj_trajectory_time_horizon, obj_nominal_acc, global_params.time_resolution,
    drac_param_map);

  return assess_drac(
    ego_trajectory_cache, context, drac_param_map, vehicle_info, nominal_speed_object_trajectories,
    global_params);
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
  const RssParamMap & rss_param_map, const VehicleInfo & vehicle_info)
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
      0.0, 0.0,
      trajectory::footprint::make_ego_dimensions(vehicle_info, rss_params.ego_footprint_margin)};
    const auto & ego_trajectory =
      ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    const auto rss_detail = assess_required_acceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);
    const auto risk_level =
      rss_detail.rss_acceleration < rss_params.error_threshold.ego_acceleration ? RiskLevel::DANGER
                                                                                : RiskLevel::SAFE;
    rss_evaluations.push_back(RssEvaluation{risk_level, rss_detail});
  }

  return RssArtifact{calc_worst_risk(rss_evaluations), std::move(rss_evaluations)};
}
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
