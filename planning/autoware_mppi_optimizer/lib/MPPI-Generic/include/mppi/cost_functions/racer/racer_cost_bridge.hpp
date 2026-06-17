/**
 * Host-side helpers to populate RacerCost reference trajectories and obstacles.
 */
#pragma once

#include <mppi/cost_functions/parked_car_obstacles.hpp>
#include <mppi/cost_functions/racer/racer_cost.cuh>
#include <mppi/path/path_reference_generator.hpp>

#include <algorithm>
#include <vector>

namespace mppi
{
namespace cost
{

/** Ego OBB from rear axle: box center offset forward from axle using wheel_base overhang model. */
template <int NUM_TIMESTEPS>
inline void setRacerCostEgoFootprint(RacerCostParams<NUM_TIMESTEPS>& params, const float wheel_base,
                                     const float ego_length, const float ego_width)
{
  params.ego_length = ego_length;
  params.ego_width = ego_width;
  const float rear_overhang = 0.08F * wheel_base;
  params.ego_axle_to_box_center = 0.5F * ego_length - rear_overhang;
}

template <int NUM_TIMESTEPS>
inline void fillRacerCostParkedCars(RacerCost<NUM_TIMESTEPS>& cost, const std::vector<ParkedCarObstacle>& cars)
{
  float obs_x[RacerCost<NUM_TIMESTEPS>::kMaxObstacles] = {};
  float obs_y[RacerCost<NUM_TIMESTEPS>::kMaxObstacles] = {};
  float obs_yaw[RacerCost<NUM_TIMESTEPS>::kMaxObstacles] = {};
  float obs_half_length[RacerCost<NUM_TIMESTEPS>::kMaxObstacles] = {};
  float obs_half_width[RacerCost<NUM_TIMESTEPS>::kMaxObstacles] = {};

  const int n =
      static_cast<int>(std::min(cars.size(), static_cast<size_t>(RacerCost<NUM_TIMESTEPS>::kMaxObstacles)));
  for (int i = 0; i < n; ++i)
  {
    const ParkedCarObstacle& car = cars[static_cast<size_t>(i)];
    obs_x[i] = car.ox;
    obs_y[i] = car.oy;
    obs_yaw[i] = car.yaw;
    obs_half_length[i] = car.length * 0.5F;
    obs_half_width[i] = car.width * 0.5F;
  }

  cost.setOrientedBoxObstacles(obs_x, obs_y, obs_yaw, obs_half_length, obs_half_width, n);
}

template <int NUM_TIMESTEPS>
inline void fillRacerCostFromPathReference(RacerCost<NUM_TIMESTEPS>& cost,
                                           const std::vector<mppi::path::PathReferenceSample>& ref)
{
  float ref_x[NUM_TIMESTEPS];
  float ref_y[NUM_TIMESTEPS];

  for (int t = 0; t < NUM_TIMESTEPS; ++t)
  {
    const size_t idx = ref.empty() ? 0U : static_cast<size_t>(std::min(t, static_cast<int>(ref.size()) - 1));
    ref_x[t] = ref[idx].x;
    ref_y[t] = ref[idx].y;
  }

  cost.setReferenceTrajectory(ref_x, ref_y, NUM_TIMESTEPS);
}

}  // namespace cost
}  // namespace mppi
