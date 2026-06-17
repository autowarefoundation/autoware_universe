/**
 * Constant-velocity vehicles for time-varying OBB collision costs (approach B).
 */
#pragma once

#include <mppi/cost_functions/parked_car_obstacles.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace mppi
{
namespace cost
{

/** Cross-traffic / dynamic vehicle with constant velocity. */
struct MovingCarObstacle
{
  float x0 = 0.0F;
  float y0 = 0.0F;
  float vx = 0.0F;
  float vy = 0.0F;
  float yaw = 0.0F;
  float length = 0.55F * 1.5F;
  float width = 0.28F * 1.5F;
  /** Obstacle pose is active in cost/viz when sim_time >= spawn_time. */
  float spawn_time = 0.0F;

  ParkedCarObstacle poseAt(const float t) const
  {
    ParkedCarObstacle car;
    const float elapsed = t - spawn_time;
    car.ox = x0 + vx * elapsed;
    car.oy = y0 + vy * elapsed;
    car.yaw = yaw;
    car.length = length;
    car.width = width;
    return car;
  }

  bool isActiveAt(const float t) const
  {
    return t >= spawn_time;
  }
};

/** Active vehicles at simulation time sim_time (for visualization). */
inline std::vector<ParkedCarObstacle> movingCarPosesAt(const std::vector<MovingCarObstacle>& cars, const float sim_time)
{
  std::vector<ParkedCarObstacle> out;
  out.reserve(cars.size());
  for (const MovingCarObstacle& car : cars)
  {
    if (car.isActiveAt(sim_time))
    {
      out.push_back(car.poseAt(sim_time));
    }
  }
  return out;
}

/**
 * Fill obstacle-major trajectory buffers for MPPI horizon [0, num_timesteps).
 * Step t uses world time horizon_start_time + t * dt.
 */
inline void buildObstacleTrajectoryBuffers(const std::vector<MovingCarObstacle>& cars, const float horizon_start_time,
                                           const float dt, const int num_timesteps, std::vector<float>& x,
                                           std::vector<float>& y, std::vector<float>& yaw,
                                           std::vector<float>& half_length, std::vector<float>& half_width)
{
  const size_t n = std::min(cars.size(), static_cast<size_t>(64));
  const int nt = std::max(1, num_timesteps);
  x.assign(n * static_cast<size_t>(nt), 0.0F);
  y.assign(n * static_cast<size_t>(nt), 0.0F);
  yaw.assign(n * static_cast<size_t>(nt), 0.0F);
  half_length.assign(n, 0.0F);
  half_width.assign(n, 0.0F);

  std::vector<int> timesteps(static_cast<size_t>(nt));
  std::iota(timesteps.begin(), timesteps.end(), 0);

  size_t i = 0;
  for (const MovingCarObstacle& car : cars)
  {
    if (i >= n)
    {
      break;
    }
    half_length[i] = car.length * 0.5F;
    half_width[i] = car.width * 0.5F;
    for (const int t : timesteps)
    {
      const float world_t = horizon_start_time + static_cast<float>(t) * dt;
      const size_t idx = i * static_cast<size_t>(nt) + static_cast<size_t>(t);
      if (car.isActiveAt(world_t))
      {
        const ParkedCarObstacle pose = car.poseAt(world_t);
        x[idx] = pose.ox;
        y[idx] = pose.oy;
        yaw[idx] = pose.yaw;
      }
      else
      {
        x[idx] = -1.0E4F;
        y[idx] = -1.0E4F;
        yaw[idx] = car.yaw;
      }
    }
    ++i;
  }
}

namespace detail
{
inline MovingCarObstacle makeCrossingCar(const float x_at_spawn, const float y_lane, const float speed,
                                         const float spawn_time, const float length_scale = 1.0F)
{
  MovingCarObstacle car;
  car.x0 = x_at_spawn;
  car.y0 = y_lane;
  car.vx = speed;
  car.vy = 0.0F;
  car.yaw = 0.0F;
  car.spawn_time = spawn_time;
  car.length = 0.55F * 1.5F * length_scale;
  car.width = 0.28F * 1.5F;
  return car;
}

inline void appendLanePlatoon(std::vector<MovingCarObstacle>& cars, const float y_lane, const float speed,
                              const float first_x, const float x_spacing, const float spawn_start,
                              const float spawn_spacing, const int count, const float length_scale = 1.0F)
{
  if (count <= 0)
  {
    return;
  }
  std::vector<int> slots(static_cast<size_t>(count));
  std::iota(slots.begin(), slots.end(), 0);
  for (const int i : slots)
  {
    cars.push_back(makeCrossingCar(first_x - static_cast<float>(i) * x_spacing, y_lane, speed,
                                   spawn_start + static_cast<float>(i) * spawn_spacing, length_scale));
  }
}
}  // namespace detail

/** Dense cross-traffic from the west (up to kMaxObstacles = 64 in first-order Dubins cost). */
inline std::vector<MovingCarObstacle> defaultIntersectionCrossTraffic()
{
  constexpr float kSpeedFast = 5.8F;
  constexpr float kSpeedMid = 4.8F;
  constexpr float kSpeedSlow = 3.6F;
  constexpr float kLaneSpacing = 8.5F;

  std::vector<MovingCarObstacle> cars;
  cars.reserve(64);

  // Six eastbound lanes, seven vehicles each (42)
  detail::appendLanePlatoon(cars, -2.2F, kSpeedFast, -16.0F, kLaneSpacing, 0.0F, 0.82F, 7);
  detail::appendLanePlatoon(cars, -2.9F, kSpeedFast, -18.0F, kLaneSpacing, 0.15F, 0.88F, 7);
  detail::appendLanePlatoon(cars, -3.6F, kSpeedFast, -22.0F, kLaneSpacing, 0.35F, 0.92F, 7);
  detail::appendLanePlatoon(cars, -4.4F, kSpeedMid, -26.0F, kLaneSpacing, 0.55F, 0.98F, 7);
  detail::appendLanePlatoon(cars, -5.2F, kSpeedMid, -30.0F, kLaneSpacing, 0.75F, 1.02F, 7);
  detail::appendLanePlatoon(cars, -6.0F, kSpeedSlow, -34.0F, kLaneSpacing, 0.95F, 1.08F, 7);

  // Five distant waves (20)
  detail::appendLanePlatoon(cars, -2.6F, kSpeedMid, -64.0F, 9.0F, 4.2F, 1.05F, 4);
  detail::appendLanePlatoon(cars, -3.3F, kSpeedMid, -68.0F, 9.0F, 4.6F, 1.1F, 4);
  detail::appendLanePlatoon(cars, -4.1F, kSpeedMid, -72.0F, 9.0F, 5.0F, 1.12F, 4);
  detail::appendLanePlatoon(cars, -5.0F, kSpeedSlow, -76.0F, 9.0F, 5.4F, 1.15F, 4);
  detail::appendLanePlatoon(cars, -5.8F, kSpeedSlow, -80.0F, 9.0F, 5.8F, 1.18F, 4);

  // Late heavy vehicles (2) — fills obstacle budget at 64
  cars.push_back(detail::makeCrossingCar(-92.0F, -3.8F, kSpeedSlow, 6.8F, 1.35F));
  cars.push_back(detail::makeCrossingCar(-98.0F, -5.4F, kSpeedSlow, 8.5F, 1.4F));

  return cars;
}

/**
 * One absurdly large eastbound vehicle timed for the left-turn intersection (~s≈22, ~7–8 s @ 3 m/s).
 * Center pose is rear-axle box center; length is along +x. East edge at t: x0 + vx*t + length/2.
 */
inline std::vector<MovingCarObstacle> intersectionGiantBlocker()
{
  constexpr float kEgoApproachTime = 7.5F;
  constexpr float kGiantSpeed = 3.4F;
  constexpr float kGiantLength = 52.0F;
  constexpr float kCenterAtApproach = -8.0F;

  MovingCarObstacle blocker;
  blocker.x0 = kCenterAtApproach - kGiantSpeed * kEgoApproachTime;
  blocker.y0 = -4.0F;
  blocker.vx = kGiantSpeed;
  blocker.vy = 0.0F;
  blocker.yaw = 0.0F;
  blocker.spawn_time = 0.0F;
  blocker.length = kGiantLength;
  blocker.width = 18.0F;
  return { blocker };
}

/** Shared compact two-lane straight-road layout (+y forward, right lane at +x). */
struct TwoLaneRoadLayout
{
  static constexpr float kRightLaneX = 0.9F;
  static constexpr float kLeftLaneX = -0.9F;
  static constexpr float kLaneHalfWidth = 0.85F;
  static constexpr float kRoadYaw = 1.5707963267948966F;
  static constexpr float kRoadYStart = -10.0F;
  static constexpr float kRoadYEnd = 32.0F;
};

/**
 * Left-lane ego goes around a stopped vehicle intruding from the left curb by nudging into
 * the right lane, while a faster vehicle in the right lane approaches from behind.
 */
inline std::vector<MovingCarObstacle> twoLaneDoubleParkAndRearApproach()
{
  std::vector<MovingCarObstacle> obstacles;
  obstacles.reserve(2);

  MovingCarObstacle double_parked;
  double_parked.x0 = TwoLaneRoadLayout::kLeftLaneX;
  double_parked.y0 = 11.0F;
  double_parked.vx = 0.0F;
  double_parked.vy = 0.0F;
  double_parked.yaw = TwoLaneRoadLayout::kRoadYaw;
  double_parked.spawn_time = 0.0F;
  double_parked.length = 0.55F * 1.5F * 1.08F;
  double_parked.width = 0.28F * 1.5F * 1.12F * 2.0F;

  MovingCarObstacle rear_right_lane;
  rear_right_lane.x0 = TwoLaneRoadLayout::kRightLaneX-0.5F;
  rear_right_lane.y0 = -12.5F;
  rear_right_lane.vx = 0.0F;
  rear_right_lane.vy = 3.5F;
  rear_right_lane.yaw = TwoLaneRoadLayout::kRoadYaw;
  rear_right_lane.spawn_time = 0.0F;
  rear_right_lane.length = 0.55F * 1.5F * 10.0F;
  rear_right_lane.width = 0.28F * 1.5F * 1.12F;

  obstacles.push_back(double_parked);
  obstacles.push_back(rear_right_lane);
  return obstacles;
}

}  // namespace cost
}  // namespace mppi
