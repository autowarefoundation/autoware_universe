/**
 * Closed polygons for drivable road surfaces and trajectory deviation corridors.
 */
#pragma once

#include <mppi/cost_functions/moving_car_obstacles.hpp>
#include <mppi/path/path2d.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mppi
{
namespace path
{

/** Closed polygon in world coordinates (vertices in order). */
struct Polygon2D
{
  std::vector<float> x;
  std::vector<float> y;

  bool empty() const { return x.size() < 3U || x.size() != y.size(); }

  size_t size() const { return std::min(x.size(), y.size()); }
};

/** Corridor along a path: left boundary forward, right boundary backward. */
inline Polygon2D pathCorridorPolygon(
  const Path2D & path, const float left_half, const float right_half,
  const float sample_spacing = 0.5F, const int max_vertices = 0)
{
  Polygon2D poly;
  const float path_length = path.length();
  if (path_length <= 0.0F || left_half <= 0.0F || right_half <= 0.0F) {
    return poly;
  }

  float spacing = std::max(sample_spacing, path_length / 256.0F);
  if (max_vertices >= 6) {
    const float max_spacing = path_length / (static_cast<float>(max_vertices) * 0.5F - 2.0F);
    spacing = std::max(spacing, max_spacing);
  }

  std::vector<float> left_x;
  std::vector<float> left_y;
  std::vector<float> right_x;
  std::vector<float> right_y;
  left_x.reserve(static_cast<size_t>(path_length / spacing) + 4U);
  right_x.reserve(left_x.capacity());

  for (float s = 0.0F; s <= path_length; s += spacing) {
    const Pose2D p = path.poseAt(s);
    float tx = 0.0F;
    float ty = 0.0F;
    path.tangentAt(s, tx, ty);
    left_x.push_back(p.x - left_half * ty);
    left_y.push_back(p.y + left_half * tx);
    right_x.push_back(p.x + right_half * ty);
    right_y.push_back(p.y - right_half * tx);
  }

  if (left_x.size() < 2U) {
    return poly;
  }

  poly.x = left_x;
  poly.y = left_y;
  for (int i = static_cast<int>(right_x.size()) - 1; i >= 0; --i) {
    poly.x.push_back(right_x[static_cast<size_t>(i)]);
    poly.y.push_back(right_y[static_cast<size_t>(i)]);
  }
  return poly;
}

inline Polygon2D symmetricPathCorridorPolygon(
  const Path2D & path, const float half_width, const float sample_spacing = 0.5F,
  const int max_vertices = 0)
{
  return pathCorridorPolygon(path, half_width, half_width, sample_spacing, max_vertices);
}

/** Axis-aligned rectangle for a straight road segment. */
inline Polygon2D straightCorridorPolygon(
  const float x0, const float y0, const float x1, const float y1, const float half_width)
{
  Polygon2D poly;
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len = std::sqrt(dx * dx + dy * dy);
  if (len < 1.0E-6F || half_width <= 0.0F) {
    return poly;
  }

  const float tx = dx / len;
  const float ty = dy / len;
  const float nx = -ty;
  const float ny = tx;

  poly.x = {x0 + nx * half_width, x1 + nx * half_width, x1 - nx * half_width, x0 - nx * half_width};
  poly.y = {y0 + ny * half_width, y1 + ny * half_width, y1 - ny * half_width, y0 - ny * half_width};
  return poly;
}

/** Both lanes of TwoLaneRoadLayout (+y forward). */
inline Polygon2D twoLaneRoadPolygon()
{
  using L = mppi::cost::TwoLaneRoadLayout;
  const float x_left = L::kLeftLaneX - L::kLaneHalfWidth;
  const float x_right = L::kRightLaneX + L::kLaneHalfWidth;
  Polygon2D poly;
  poly.x = {x_left, x_right, x_right, x_left};
  poly.y = {L::kRoadYStart, L::kRoadYStart, L::kRoadYEnd, L::kRoadYEnd};
  return poly;
}

}  // namespace path
}  // namespace mppi
