/**
 * OpenCV visualization helpers for 2D path-tracking simulations.
 */
#pragma once

#include <mppi/path/drivable_area.hpp>
#include <mppi/path/path2d.hpp>
#include <mppi/path/path_reference_generator.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace mppi
{
namespace viz
{

inline cv::Mat makeWhiteFrame(const int img_w, const int img_h)
{
  return cv::Mat(img_h, img_w, CV_8UC3, cv::Scalar(255, 255, 255));
}

inline cv::Point2f worldToPixel(const float x, const float y, const int img_w, const int img_h,
                                const float scale = 15.0F)
{
  const float u = static_cast<float>(img_w) / 2.0F + x * scale;
  const float v = static_cast<float>(img_h) / 2.0F - y * scale;
  return cv::Point2f(u, v);
}

inline cv::Scalar lerpBgr(const cv::Scalar& a, const cv::Scalar& b, const float t)
{
  const float s = std::max(0.0F, std::min(1.0F, t));
  return cv::Scalar(a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2]));
}

inline cv::Scalar costGradientColor(const float cost, const float min_cost, const float max_cost)
{
  const cv::Scalar k_teal(128, 128, 0);
  const cv::Scalar k_purple(128, 0, 128);
  if (max_cost <= min_cost)
  {
    return k_teal;
  }
  const float t = (cost - min_cost) / (max_cost - min_cost);
  return lerpBgr(k_teal, k_purple, t);
}

inline void drawCenterline(cv::Mat& img, const path::Path2D& path, const float scale = 15.0F)
{
  cv::Mat overlay = img.clone();
  const auto& anchors = path.anchors();
  for (size_t i = 0; i + 1 < anchors.size(); ++i)
  {
    cv::line(overlay, worldToPixel(anchors[i].x, anchors[i].y, img.cols, img.rows, scale),
             worldToPixel(anchors[i + 1].x, anchors[i + 1].y, img.cols, img.rows, scale), cv::Scalar(200, 200, 200),
             2);
  }
  cv::addWeighted(overlay, 0.85, img, 0.15, 0, img);
}

/** Draw left/right road edges offset from the path centerline (matches RacerCost boundary_threshold). */
inline void drawRoadBoundaries(cv::Mat& img, const path::Path2D& path, const float half_width = 0.8F,
                               const float scale = 15.0F, const bool fill_corridor = true)
{
  const float path_length = path.length();
  if (path_length <= 0.0F || half_width <= 0.0F)
  {
    return;
  }

  constexpr float kSampleSpacing = 0.5F;
  std::vector<cv::Point> left_px;
  std::vector<cv::Point> right_px;
  left_px.reserve(static_cast<size_t>(path_length / kSampleSpacing) + 4U);
  right_px.reserve(left_px.capacity());

  for (float s = 0.0F; s <= path_length; s += kSampleSpacing)
  {
    const path::Pose2D p = path.poseAt(s);
    float tx = 0.0F;
    float ty = 0.0F;
    path.tangentAt(s, tx, ty);
    const cv::Point2f left =
        worldToPixel(p.x - half_width * ty, p.y + half_width * tx, img.cols, img.rows, scale);
    const cv::Point2f right =
        worldToPixel(p.x + half_width * ty, p.y - half_width * tx, img.cols, img.rows, scale);
    left_px.emplace_back(static_cast<int>(left.x + 0.5F), static_cast<int>(left.y + 0.5F));
    right_px.emplace_back(static_cast<int>(right.x + 0.5F), static_cast<int>(right.y + 0.5F));
  }

  if (left_px.size() < 2U)
  {
    return;
  }

  if (fill_corridor)
  {
    std::vector<cv::Point> corridor = left_px;
    corridor.insert(corridor.end(), right_px.rbegin(), right_px.rend());
    cv::Mat overlay = img.clone();
    const cv::Point* pts = corridor.data();
    const int n_pts = static_cast<int>(corridor.size());
    cv::fillPoly(overlay, &pts, &n_pts, 1, cv::Scalar(235, 245, 235));
    cv::addWeighted(overlay, 0.35, img, 0.65, 0, img);
  }

  cv::Mat overlay = img.clone();
  const cv::Scalar edge_color(120, 120, 120);
  for (size_t i = 0; i + 1 < left_px.size(); ++i)
  {
    cv::line(overlay, left_px[i], left_px[i + 1], edge_color, 1, cv::LINE_AA);
    cv::line(overlay, right_px[i], right_px[i + 1], edge_color, 1, cv::LINE_AA);
  }
  cv::addWeighted(overlay, 0.9, img, 0.1, 0, img);
}

/** Draw allowed corridor edges relative to path (positive left/right match FirstOrderDubins off-road limits). */
inline void drawAsymmetricRoadBoundaries(cv::Mat& img, const path::Path2D& path, const float left_half,
                                         const float right_half, const float scale = 15.0F,
                                         const bool fill_corridor = true)
{
  const float path_length = path.length();
  if (path_length <= 0.0F || left_half <= 0.0F || right_half <= 0.0F)
  {
    return;
  }

  constexpr float kSampleSpacing = 0.5F;
  std::vector<cv::Point> left_px;
  std::vector<cv::Point> right_px;
  left_px.reserve(static_cast<size_t>(path_length / kSampleSpacing) + 4U);
  right_px.reserve(left_px.capacity());

  for (float s = 0.0F; s <= path_length; s += kSampleSpacing)
  {
    const path::Pose2D p = path.poseAt(s);
    float tx = 0.0F;
    float ty = 0.0F;
    path.tangentAt(s, tx, ty);
    const cv::Point2f left =
        worldToPixel(p.x - left_half * ty, p.y + left_half * tx, img.cols, img.rows, scale);
    const cv::Point2f right =
        worldToPixel(p.x + right_half * ty, p.y - right_half * tx, img.cols, img.rows, scale);
    left_px.emplace_back(static_cast<int>(left.x + 0.5F), static_cast<int>(left.y + 0.5F));
    right_px.emplace_back(static_cast<int>(right.x + 0.5F), static_cast<int>(right.y + 0.5F));
  }

  if (left_px.size() < 2U)
  {
    return;
  }

  if (fill_corridor)
  {
    std::vector<cv::Point> corridor = left_px;
    corridor.insert(corridor.end(), right_px.rbegin(), right_px.rend());
    cv::Mat overlay = img.clone();
    const cv::Point* pts = corridor.data();
    const int n_pts = static_cast<int>(corridor.size());
    cv::fillPoly(overlay, &pts, &n_pts, 1, cv::Scalar(245, 250, 255));
    cv::addWeighted(overlay, 0.35, img, 0.65, 0, img);
  }

  cv::Mat overlay = img.clone();
  const cv::Scalar edge_color(40, 90, 200);
  for (size_t i = 0; i + 1 < left_px.size(); ++i)
  {
    cv::line(overlay, left_px[i], left_px[i + 1], edge_color, 2, cv::LINE_AA);
    cv::line(overlay, right_px[i], right_px[i + 1], edge_color, 2, cv::LINE_AA);
  }
  cv::addWeighted(overlay, 0.9, img, 0.1, 0, img);
}

inline void drawReferencePath(cv::Mat& img, const std::vector<path::PathReferenceSample>& ref,
                              const float scale = 15.0F)
{
  if (ref.size() < 2)
  {
    return;
  }

  cv::Mat overlay = img.clone();
  for (size_t i = 0; i + 1 < ref.size(); ++i)
  {
    cv::line(overlay, worldToPixel(ref[i].x, ref[i].y, img.cols, img.rows, scale),
             worldToPixel(ref[i + 1].x, ref[i + 1].y, img.cols, img.rows, scale), cv::Scalar(0, 165, 255), 2);
  }
  cv::addWeighted(overlay, 0.7, img, 0.3, 0, img);
}

/** Oriented rectangle at box center (body x = forward, y = left). */
inline void drawOrientedBox(cv::Mat& img, const float cx, const float cy, const float yaw, const float length,
                            const float width, const cv::Scalar& fill, const cv::Scalar& outline,
                            const float scale = 15.0F)
{
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  const float hl = length * 0.5F;
  const float hw = width * 0.5F;

  const float corners[4][2] = { { hl, hw }, { hl, -hw }, { -hl, -hw }, { -hl, hw } };
  std::vector<cv::Point> px(4);
  for (int i = 0; i < 4; ++i)
  {
    const float bx = corners[i][0];
    const float by = corners[i][1];
    const float wx = cx + c * bx - s * by;
    const float wy = cy + s * bx + c * by;
    const cv::Point2f p = worldToPixel(wx, wy, img.cols, img.rows, scale);
    px[static_cast<size_t>(i)] =
        cv::Point(static_cast<int>(p.x + 0.5F), static_cast<int>(p.y + 0.5F));
  }

  const cv::Point* poly = px.data();
  const int n = 4;
  cv::fillPoly(img, &poly, &n, 1, fill);
  cv::polylines(img, px, true, outline, 1, cv::LINE_AA);
}

/** Oriented rectangle parked car (body x = forward, y = left). */
inline void drawParkedCar(cv::Mat& img, const float x, const float y, const float yaw, const float length,
                          const float width, const cv::Scalar& fill, const cv::Scalar& outline,
                          const float scale = 15.0F)
{
  drawOrientedBox(img, x, y, yaw, length, width, fill, outline, scale);
}

/** Ego OBB from rear-axle pose (matches RacerCost egoIntersectsParkedCar footprint). */
inline void drawEgoVehicleAtRearAxle(cv::Mat& img, const float axle_x, const float axle_y, const float yaw,
                                      const float length, const float width, const float axle_to_box_center,
                                      const cv::Scalar& fill = cv::Scalar(0, 200, 0),
                                      const cv::Scalar& outline = cv::Scalar(0, 120, 0), const float scale = 15.0F)
{
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  const float cx = axle_x + axle_to_box_center * c;
  const float cy = axle_y + axle_to_box_center * s;
  drawOrientedBox(img, cx, cy, yaw, length, width, fill, outline, scale);
}

template <typename ParkedCarT>
inline void drawParkedCars(cv::Mat& img, const std::vector<ParkedCarT>& cars,
                           const cv::Scalar& fill = cv::Scalar(40, 40, 180),
                           const cv::Scalar& outline = cv::Scalar(20, 20, 100), const float scale = 15.0F)
{
  for (const ParkedCarT& car : cars)
  {
    drawParkedCar(img, car.ox, car.oy, car.yaw, car.length, car.width, fill, outline, scale);
  }
}

template <typename TrajectoryMatrix>
inline void drawTrajectory(cv::Mat& img, const TrajectoryMatrix& traj, const int x_idx, const int y_idx,
                           const cv::Scalar& color = cv::Scalar(255, 64, 0), const int thickness = 3,
                           const float overlay_alpha = 0.85F, const float scale = 15.0F)
{
  cv::Mat overlay = img.clone();
  for (int i = 0; i + 1 < traj.cols(); ++i)
  {
    cv::line(overlay, worldToPixel(traj(x_idx, i), traj(y_idx, i), img.cols, img.rows, scale),
             worldToPixel(traj(x_idx, i + 1), traj(y_idx, i + 1), img.cols, img.rows, scale), color, thickness);
  }
  cv::addWeighted(overlay, overlay_alpha, img, 1.0F - overlay_alpha, 0, img);
}

/** Default cap on rollouts drawn live; full samples remain in MPPI / CSV dumps. */
constexpr int kDefaultMaxDrawRollouts = 400;

template <typename TrajectoryMatrix>
inline void drawSampledTrajectories(cv::Mat& img, const std::vector<TrajectoryMatrix>& sampled_trajectories,
                                    const int x_idx, const int y_idx, const int num_timesteps,
                                    const std::vector<float>& rollout_costs, const int thickness = 1,
                                    const float scale = 15.0F, const int max_rollouts = kDefaultMaxDrawRollouts)
{
  if (sampled_trajectories.empty() || num_timesteps <= 1 ||
      rollout_costs.size() != sampled_trajectories.size())
  {
    return;
  }

  const int num_valid_cols = num_timesteps - 1;

  std::vector<size_t> draw_order(sampled_trajectories.size());
  std::iota(draw_order.begin(), draw_order.end(), 0U);
  if (max_rollouts > 0 && static_cast<int>(draw_order.size()) > max_rollouts)
  {
    std::partial_sort(draw_order.begin(), draw_order.begin() + static_cast<size_t>(max_rollouts), draw_order.end(),
                      [&rollout_costs](const size_t a, const size_t b) {
                        return rollout_costs[a] < rollout_costs[b];
                      });
    draw_order.resize(static_cast<size_t>(max_rollouts));
  }

  float min_cost = rollout_costs[draw_order.front()];
  float max_cost = rollout_costs[draw_order.front()];
  for (const size_t idx : draw_order)
  {
    min_cost = std::min(min_cost, rollout_costs[idx]);
    max_cost = std::max(max_cost, rollout_costs[idx]);
  }

  std::sort(draw_order.begin(), draw_order.end(),
            [&rollout_costs](const size_t a, const size_t b) { return rollout_costs[a] > rollout_costs[b]; });

  for (const size_t idx : draw_order)
  {
    const TrajectoryMatrix& traj = sampled_trajectories[idx];
    const cv::Scalar color = costGradientColor(rollout_costs[idx], min_cost, max_cost);
    const int cols_to_draw = std::min(num_valid_cols, static_cast<int>(traj.cols()));
    for (int i = 0; i + 1 < cols_to_draw; ++i)
    {
      cv::line(img, worldToPixel(traj(x_idx, i), traj(y_idx, i), img.cols, img.rows, scale),
               worldToPixel(traj(x_idx, i + 1), traj(y_idx, i + 1), img.cols, img.rows, scale), color, thickness);
    }
  }
}

/** Draw a straight cross-street segment (world coordinates). */
inline void drawStraightCorridor(cv::Mat& img, const float x0, const float y0, const float x1, const float y1,
                                 const float half_width, const float scale = 15.0F)
{
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len = std::sqrt(dx * dx + dy * dy);
  if (len < 1.0E-6F)
  {
    return;
  }
  const float tx = dx / len;
  const float ty = dy / len;
  const float nx = -ty;
  const float ny = tx;
  const cv::Point2f c0 = worldToPixel(x0 + nx * half_width, y0 + ny * half_width, img.cols, img.rows, scale);
  const cv::Point2f c1 = worldToPixel(x1 + nx * half_width, y1 + ny * half_width, img.cols, img.rows, scale);
  const cv::Point2f c2 = worldToPixel(x1 - nx * half_width, y1 - ny * half_width, img.cols, img.rows, scale);
  const cv::Point2f c3 = worldToPixel(x0 - nx * half_width, y0 - ny * half_width, img.cols, img.rows, scale);
  std::vector<cv::Point> poly = {
    cv::Point(static_cast<int>(c0.x), static_cast<int>(c0.y)),
    cv::Point(static_cast<int>(c1.x), static_cast<int>(c1.y)),
    cv::Point(static_cast<int>(c2.x), static_cast<int>(c2.y)),
    cv::Point(static_cast<int>(c3.x), static_cast<int>(c3.y)),
  };
  cv::Mat overlay = img.clone();
  const cv::Point* pts = poly.data();
  const int n_pts = 4;
  cv::fillPoly(overlay, &pts, &n_pts, 1, cv::Scalar(235, 245, 235));
  cv::addWeighted(overlay, 0.35, img, 0.65, 0, img);
  cv::line(img, poly[0], poly[1], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
  cv::line(img, poly[2], poly[3], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
}

/** Fill a closed world-frame polygon with semi-transparent color. */
inline void drawFilledPolygon(cv::Mat& img, const path::Polygon2D& poly, const cv::Scalar& fill,
                              const cv::Scalar& outline, const float overlay_alpha = 0.35F,
                              const float scale = 15.0F)
{
  if (poly.empty())
  {
    return;
  }

  std::vector<cv::Point> px(poly.size());
  for (size_t i = 0; i < poly.size(); ++i)
  {
    const cv::Point2f p = worldToPixel(poly.x[i], poly.y[i], img.cols, img.rows, scale);
    px[i] = cv::Point(static_cast<int>(p.x + 0.5F), static_cast<int>(p.y + 0.5F));
  }

  cv::Mat overlay = img.clone();
  const cv::Point* pts = px.data();
  const int n_pts = static_cast<int>(px.size());
  cv::fillPoly(overlay, &pts, &n_pts, 1, fill);
  cv::addWeighted(overlay, overlay_alpha, img, 1.0F - overlay_alpha, 0, img);
  cv::polylines(img, px, true, outline, 1, cv::LINE_AA);
}

/** Full drivable road surface (both lanes, etc.). */
inline void drawDrivableAreaPolygon(cv::Mat& img, const path::Polygon2D& poly, const float scale = 15.0F)
{
  drawFilledPolygon(img, poly, cv::Scalar(235, 245, 235), cv::Scalar(120, 180, 120), 0.4F, scale);
}

/** Soft trajectory deviation corridor (track_cost band around reference path). */
inline void drawTrajectoryCorridorPolygon(cv::Mat& img, const path::Polygon2D& poly, const float scale = 15.0F)
{
  drawFilledPolygon(img, poly, cv::Scalar(245, 250, 255), cv::Scalar(40, 90, 200), 0.35F, scale);
}

/** Layout for stacked time-series strips under the track view. */
struct TimeSeriesPlotLayout
{
  int plot_strip_height = 200;
  int plot_strip_count = 4;
  int plot_margin_left = 76;
  int plot_margin_right = 16;
  int plot_margin_top = 26;
  int plot_margin_bottom = 14;
  float plot_font_scale = 0.58F;
  int plot_line_thickness = 2;
  /** Downscale composite for imshow so the window fits typical displays (video stays full size). */
  int max_display_width = 1440;
  int max_display_height = 960;
};

inline const TimeSeriesPlotLayout& defaultTimeSeriesPlotLayout()
{
  static const TimeSeriesPlotLayout layout{};
  return layout;
}

inline cv::Size compositeFrameSize(const int track_width, const int track_height, const TimeSeriesPlotLayout& layout)
{
  return cv::Size(track_width, track_height + layout.plot_strip_count * layout.plot_strip_height);
}

/** Downscale for on-screen display; returns input unchanged if already within limits. */
inline cv::Mat fitFrameForDisplay(const cv::Mat& frame, const int max_width, const int max_height)
{
  if (frame.empty() || max_width <= 0 || max_height <= 0)
  {
    return frame;
  }
  const double sx = static_cast<double>(max_width) / static_cast<double>(frame.cols);
  const double sy = static_cast<double>(max_height) / static_cast<double>(frame.rows);
  const double scale = std::min({ sx, sy, 1.0 });
  if (scale >= 0.999)
  {
    return frame;
  }
  cv::Mat display;
  cv::resize(frame, display, cv::Size(), scale, scale, cv::INTER_AREA);
  return display;
}

inline void showCompositeFrame(const char* window_name, const cv::Mat& composite, const TimeSeriesPlotLayout& layout)
{
  const cv::Mat display =
      fitFrameForDisplay(composite, layout.max_display_width, layout.max_display_height);
  cv::imshow(window_name, display);
  cv::resizeWindow(window_name, display.cols, display.rows);
  cv::setWindowProperty(window_name, cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
}

/** Rolling signals for strip-chart overlays on path-tracking videos. */
struct RunningTimeSeries
{
  std::vector<float> t;
  std::vector<float> vel;
  std::vector<float> accel;
  std::vector<float> steer_cmd;
  std::vector<float> steer_state;

  void push(const float time, const float velocity, const float acceleration, const float steer_command,
            const float steer_angle, const size_t max_points = 400)
  {
    t.push_back(time);
    vel.push_back(velocity);
    accel.push_back(acceleration);
    steer_cmd.push_back(steer_command);
    steer_state.push_back(steer_angle);
    if (t.size() > max_points)
    {
      const size_t drop = t.size() - max_points;
      t.erase(t.begin(), t.begin() + static_cast<std::ptrdiff_t>(drop));
      vel.erase(vel.begin(), vel.begin() + static_cast<std::ptrdiff_t>(drop));
      accel.erase(accel.begin(), accel.begin() + static_cast<std::ptrdiff_t>(drop));
      steer_cmd.erase(steer_cmd.begin(), steer_cmd.begin() + static_cast<std::ptrdiff_t>(drop));
      steer_state.erase(steer_state.begin(), steer_state.begin() + static_cast<std::ptrdiff_t>(drop));
    }
  }
};

inline void drawTimeSeriesStrip(cv::Mat& strip, const std::vector<float>& t, const std::vector<float>& y,
                                const float y_min, const float y_max, const cv::Scalar& color, const char* label,
                                const TimeSeriesPlotLayout& layout, const float ref_line = NAN,
                                const cv::Scalar& ref_color = cv::Scalar(180, 180, 180))
{
  strip.setTo(cv::Scalar(252, 252, 252));
  const int w = strip.cols;
  const int h = strip.rows;
  const int margin_l = layout.plot_margin_left;
  const int margin_r = layout.plot_margin_right;
  const int margin_t = layout.plot_margin_top;
  const int margin_b = layout.plot_margin_bottom;
  const int plot_w = std::max(1, w - margin_l - margin_r);
  const int plot_h = std::max(1, h - margin_t - margin_b);
  const int axis_thickness = std::max(1, layout.plot_line_thickness - 1);
  const int data_thickness = layout.plot_line_thickness;

  cv::line(strip, cv::Point(margin_l, margin_t), cv::Point(margin_l, margin_t + plot_h), cv::Scalar(200, 200, 200),
           axis_thickness);
  cv::line(strip, cv::Point(margin_l, margin_t + plot_h), cv::Point(margin_l + plot_w, margin_t + plot_h),
           cv::Scalar(200, 200, 200), axis_thickness);
  cv::putText(strip, label, cv::Point(8, margin_t - 8), cv::FONT_HERSHEY_SIMPLEX, layout.plot_font_scale,
              cv::Scalar(30, 30, 30), 1, cv::LINE_AA);

  const float y_span = std::max(y_max - y_min, 1.0E-6F);
  auto to_px = [&](const float tv, const float yv) {
    const float t0 = t.front();
    const float t1 = t.back();
    const float t_span = std::max(t1 - t0, 1.0E-6F);
    const float u = (tv - t0) / t_span;
    const float v = (yv - y_min) / y_span;
    const int px = margin_l + static_cast<int>(u * static_cast<float>(plot_w));
    const int py = margin_t + plot_h - static_cast<int>(v * static_cast<float>(plot_h));
    return cv::Point(px, py);
  };

  if (!std::isnan(ref_line) && ref_line >= y_min && ref_line <= y_max)
  {
    const cv::Point p0 = to_px(t.front(), ref_line);
    const cv::Point p1 = to_px(t.back(), ref_line);
    cv::line(strip, p0, p1, ref_color, data_thickness, cv::LINE_AA);
  }

  if (t.size() >= 2)
  {
    for (size_t i = 1; i < t.size(); ++i)
    {
      cv::line(strip, to_px(t[i - 1], y[i - 1]), to_px(t[i], y[i]), color, data_thickness, cv::LINE_AA);
    }
  }
}

/** Stack track view (top) and four signal strips below (full resolution for video). */
inline cv::Mat composeFrameWithTimeSeriesPlots(const cv::Mat& track_frame, const RunningTimeSeries& history,
                                               const float vel_ref, const float vel_max, const float accel_min,
                                               const float accel_max, const float steer_max,
                                               const TimeSeriesPlotLayout& layout = defaultTimeSeriesPlotLayout())
{
  const int strip_h = layout.plot_strip_height;
  const int total_h = track_frame.rows + layout.plot_strip_count * strip_h;
  cv::Mat out(total_h, track_frame.cols, CV_8UC3, cv::Scalar(255, 255, 255));
  track_frame.copyTo(out(cv::Rect(0, 0, track_frame.cols, track_frame.rows)));

  if (history.t.size() < 2)
  {
    return out;
  }

  const cv::Scalar k_vel(200, 120, 0);
  const cv::Scalar k_accel(180, 60, 60);
  const cv::Scalar k_steer_cmd(60, 60, 200);
  const cv::Scalar k_steer_state(60, 160, 60);

  int y_off = track_frame.rows;
  cv::Mat strip_vel(strip_h, track_frame.cols, CV_8UC3);
  drawTimeSeriesStrip(strip_vel, history.t, history.vel, 0.0F, vel_max, k_vel, "velocity [m/s]", layout, vel_ref);
  strip_vel.copyTo(out(cv::Rect(0, y_off, track_frame.cols, strip_h)));
  y_off += strip_h;

  cv::Mat strip_accel(strip_h, track_frame.cols, CV_8UC3);
  drawTimeSeriesStrip(strip_accel, history.t, history.accel, accel_min, accel_max, k_accel, "accel cmd [m/s^2]", layout);
  strip_accel.copyTo(out(cv::Rect(0, y_off, track_frame.cols, strip_h)));
  y_off += strip_h;

  cv::Mat strip_steer_cmd(strip_h, track_frame.cols, CV_8UC3);
  drawTimeSeriesStrip(strip_steer_cmd, history.t, history.steer_cmd, -steer_max, steer_max, k_steer_cmd,
                      "steer cmd [rad]", layout);
  strip_steer_cmd.copyTo(out(cv::Rect(0, y_off, track_frame.cols, strip_h)));
  y_off += strip_h;

  cv::Mat strip_steer_state(strip_h, track_frame.cols, CV_8UC3);
  drawTimeSeriesStrip(strip_steer_state, history.t, history.steer_state, -steer_max, steer_max, k_steer_state,
                      "steer state [rad]", layout);
  strip_steer_state.copyTo(out(cv::Rect(0, y_off, track_frame.cols, strip_h)));

  return out;
}

}  // namespace viz
}  // namespace mppi
