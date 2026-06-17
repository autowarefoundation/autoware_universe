/**
 * ImGui draw-list helpers for 2D world-space path-tracking scenes.
 */
#pragma once

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace mppi
{
namespace viz
{
namespace detail
{

struct Rgb8
{
  int r = 0;
  int g = 0;
  int b = 0;
  int a = 255;

  Rgb8() = default;
  Rgb8(const int r_in, const int g_in, const int b_in, const int a_in = 255) : r(r_in), g(g_in), b(b_in), a(a_in)
  {
  }

  ImU32 toImU32() const
  {
    return IM_COL32(r, g, b, a);
  }
};

struct ViewCamera
{
  float center_x = 0.0F;
  float center_y = 0.0F;
  float pixels_per_meter = 22.0F;
};

inline ImVec2 worldToScreen(const float wx, const float wy, const ViewCamera& camera, const ImVec2& origin,
                            const ImVec2& size)
{
  return ImVec2(origin.x + size.x * 0.5F + (wx - camera.center_x) * camera.pixels_per_meter,
                origin.y + size.y * 0.5F - (wy - camera.center_y) * camera.pixels_per_meter);
}

inline void drawFilledPolygon(ImDrawList* draw_list, const std::vector<float>& x, const std::vector<float>& y,
                              const ViewCamera& camera, const ImVec2& origin, const ImVec2& size, const Rgb8& color)
{
  if (x.size() < 3U || x.size() != y.size())
  {
    return;
  }

  std::vector<ImVec2> pts(x.size());
  for (size_t i = 0; i < x.size(); ++i)
  {
    pts[i] = worldToScreen(x[i], y[i], camera, origin, size);
  }
  draw_list->AddConcavePolyFilled(pts.data(), static_cast<int>(pts.size()), color.toImU32());
}

inline void drawPolylineLoop(ImDrawList* draw_list, const std::vector<float>& x, const std::vector<float>& y,
                             const ViewCamera& camera, const ImVec2& origin, const ImVec2& size, const Rgb8& color,
                             const float thickness)
{
  if (x.size() < 2U || x.size() != y.size())
  {
    return;
  }

  std::vector<ImVec2> pts(x.size());
  for (size_t i = 0; i < x.size(); ++i)
  {
    pts[i] = worldToScreen(x[i], y[i], camera, origin, size);
  }
  draw_list->AddPolyline(pts.data(), static_cast<int>(pts.size()), color.toImU32(), ImDrawFlags_Closed, thickness);
}

inline void drawPolyline(ImDrawList* draw_list, const std::vector<float>& x, const std::vector<float>& y,
                         const ViewCamera& camera, const ImVec2& origin, const ImVec2& size, const Rgb8& color,
                         const float thickness)
{
  if (x.size() < 2U || x.size() != y.size())
  {
    return;
  }

  for (size_t i = 0; i + 1 < x.size(); ++i)
  {
    const ImVec2 p0 = worldToScreen(x[i], y[i], camera, origin, size);
    const ImVec2 p1 = worldToScreen(x[i + 1], y[i + 1], camera, origin, size);
    draw_list->AddLine(p0, p1, color.toImU32(), thickness);
  }
}

inline void drawOrientedBox(ImDrawList* draw_list, const float cx, const float cy, const float yaw, const float length,
                            const float width, const ViewCamera& camera, const ImVec2& origin, const ImVec2& size,
                            const Rgb8& fill, const Rgb8& outline, const float outline_thickness)
{
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  const float hl = length * 0.5F;
  const float hw = width * 0.5F;
  const float corners[4][2] = { { hl, hw }, { hl, -hw }, { -hl, -hw }, { -hl, hw } };

  ImVec2 pts[4];
  for (int i = 0; i < 4; ++i)
  {
    const float bx = corners[i][0];
    const float by = corners[i][1];
    const float wx = cx + c * bx - s * by;
    const float wy = cy + s * bx + c * by;
    pts[i] = worldToScreen(wx, wy, camera, origin, size);
  }

  draw_list->AddConvexPolyFilled(pts, 4, fill.toImU32());
  draw_list->AddPolyline(pts, 4, outline.toImU32(), ImDrawFlags_Closed, outline_thickness);
}

inline void drawEgoAtRearAxle(ImDrawList* draw_list, const float axle_x, const float axle_y, const float yaw,
                              const float length, const float width, const float axle_to_center, const ViewCamera& camera,
                              const ImVec2& origin, const ImVec2& size)
{
  const float cx = axle_x + axle_to_center * std::cos(yaw);
  const float cy = axle_y + axle_to_center * std::sin(yaw);
  drawOrientedBox(draw_list, cx, cy, yaw, length, width, camera, origin, size, Rgb8{ 0, 200, 0, 220 },
                  Rgb8{ 0, 120, 0, 255 }, 2.0F);
}

inline ImU32 rolloutCostColor(const float cost, const float min_cost, const float max_cost)
{
  const Rgb8 k_teal{ 0, 128, 128 };
  const Rgb8 k_purple{ 128, 0, 128 };
  if (max_cost <= min_cost)
  {
    return k_teal.toImU32();
  }
  const float t = std::max(0.0F, std::min(1.0F, (cost - min_cost) / (max_cost - min_cost)));
  return IM_COL32(static_cast<int>(k_teal.r + t * (k_purple.r - k_teal.r)),
                  static_cast<int>(k_teal.g + t * (k_purple.g - k_teal.g)),
                  static_cast<int>(k_teal.b + t * (k_purple.b - k_teal.b)), 255);
}

}  // namespace detail
}  // namespace viz
}  // namespace mppi
