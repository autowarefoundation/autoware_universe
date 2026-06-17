/**
 * Parked-car obstacle types and road-shoulder placement (shared by path-tracking cost bridges).
 */
#pragma once

#include <mppi/path/path2d.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace mppi
{
namespace cost
{

/** Parked vehicle on the road shoulder (oriented box for cost and viz). */
struct ParkedCarObstacle
{
  float ox = 0.0F;
  float oy = 0.0F;
  float yaw = 0.0F;
  float length = 0.55F * 1.5F;
  float width = 0.28F * 1.5F;
};

/**
 * Place parked cars along both shoulders near the road boundary, alternating left/right with gaps
 * so the ego vehicle weaves through a traffic corridor. lateral offset is a fraction of road_half_width.
 */
inline std::vector<ParkedCarObstacle> generateParkedCarsAlongRoad(const mppi::path::Path2D& path,
                                                                  const float road_half_width,
                                                                  const unsigned int seed,
                                                                  const float along_spacing = 5.5F,
                                                                  const int max_cars = 48)
{
  std::vector<ParkedCarObstacle> cars;
  if (path.empty() || road_half_width <= 0.0F)
  {
    return cars;
  }

  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist_jitter_s(-0.9F, 0.9F);
  std::uniform_real_distribution<float> dist_gap(0.0F, 1.0F);
  std::uniform_real_distribution<float> dist_lat_frac(0.58F, 0.88F);

  constexpr float kGapProbability = 0.24F;
  float s = along_spacing * 0.35F;
  int side_sign = 1;

  while (s < path.length() && static_cast<int>(cars.size()) < max_cars)
  {
    if (dist_gap(rng) < kGapProbability)
    {
      s += along_spacing * 0.55F;
      continue;
    }

    const float s_wrapped = path.wrapArcLength(s + dist_jitter_s(rng));
    const mppi::path::Pose2D p = path.poseAt(s_wrapped);
    float tx = 0.0F;
    float ty = 0.0F;
    path.tangentAt(s_wrapped, tx, ty);

    const float lateral = road_half_width * dist_lat_frac(rng);
    const float sign = (side_sign > 0) ? 1.0F : -1.0F;

    ParkedCarObstacle car;
    car.ox = p.x - sign * lateral * ty;
    car.oy = p.y + sign * lateral * tx;
    car.yaw = p.yaw;
    cars.push_back(car);

    side_sign = -side_sign;
    s += along_spacing + dist_jitter_s(rng) * 0.35F;
  }

  return cars;
}

}  // namespace cost
}  // namespace mppi
