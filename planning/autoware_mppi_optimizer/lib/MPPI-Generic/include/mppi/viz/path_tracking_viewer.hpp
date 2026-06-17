/**
 * GLFW + ImGui viewer for live MPPI path-tracking simulations.
 */
#pragma once

#include <mppi/cost_functions/parked_car_obstacles.hpp>
#include <mppi/path/drivable_area.hpp>
#include <mppi/path/path2d.hpp>
#include <mppi/path/path_reference_generator.hpp>

#include <string>
#include <vector>

namespace mppi
{
namespace viz
{

constexpr int kDefaultMaxDrawRollouts = 400;

/** Rolling signals for ImGui plot overlays. */
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

struct VizPolyline
{
  std::vector<float> x;
  std::vector<float> y;
};

struct VizRollout : VizPolyline
{
  float cost = 0.0F;
};

struct PathTrackingStaticScene
{
  VizPolyline centerline;
  path::Polygon2D drivable;
  path::Polygon2D trajectory_corridor;
  std::vector<path::Polygon2D> extra_corridors;
  std::vector<cost::ParkedCarObstacle> static_obstacles;
};

struct PathTrackingVizFrame
{
  std::vector<path::PathReferenceSample> ref;
  std::vector<VizRollout> rollouts;
  VizPolyline planned;
  float ego_x = 0.0F;
  float ego_y = 0.0F;
  float ego_yaw = 0.0F;
  std::vector<cost::ParkedCarObstacle> obstacles;
  RunningTimeSeries signals;
  float sim_time = 0.0F;
  float vel = 0.0F;
  float lat_err = 0.0F;
  float baseline_cost = 0.0F;
  int step = 0;
  int total_steps = 0;
};

struct PathTrackingEgoFootprint
{
  float length = 0.825F;
  float width = 0.42F;
  float axle_to_center = 0.39F;

  PathTrackingEgoFootprint() = default;
  PathTrackingEgoFootprint(const float length_in, const float width_in, const float axle_to_center_in)
    : length(length_in), width(width_in), axle_to_center(axle_to_center_in)
  {
  }
};

struct PathTrackingSignalLimits
{
  float target_speed = 3.0F;
  float v_max = 5.0F;
  float accel_min = -3.0F;
  float accel_max = 3.0F;
  float steer_max = 0.5F;

  PathTrackingSignalLimits() = default;
  PathTrackingSignalLimits(const float target_speed_in, const float v_max_in, const float accel_min_in,
                           const float accel_max_in, const float steer_max_in)
    : target_speed(target_speed_in)
    , v_max(v_max_in)
    , accel_min(accel_min_in)
    , accel_max(accel_max_in)
    , steer_max(steer_max_in)
  {
  }
};

inline VizPolyline polylineFromPath(const path::Path2D& path)
{
  VizPolyline poly;
  const auto& anchors = path.anchors();
  poly.x.reserve(anchors.size());
  poly.y.reserve(anchors.size());
  for (const path::Pose2D& pose : anchors)
  {
    poly.x.push_back(pose.x);
    poly.y.push_back(pose.y);
  }
  return poly;
}

template <typename TrajMatrix>
inline void extractPolyline(const TrajMatrix& traj, const int x_idx, const int y_idx, const int num_cols,
                            VizPolyline& out)
{
  out.x.clear();
  out.y.clear();
  const int max_cols = std::min(num_cols, static_cast<int>(traj.cols()));
  if (max_cols <= 0)
  {
    return;
  }

  for (int i = 0; i < max_cols; ++i)
  {
    const float xi = traj(x_idx, i);
    const float yi = traj(y_idx, i);
    if (i > 0 && std::abs(xi) < 1.0E-5F && std::abs(yi) < 1.0E-5F)
    {
      const float px = out.x.back();
      const float py = out.y.back();
      if (px * px + py * py > 1.0F)
      {
        break;
      }
    }
    out.x.push_back(xi);
    out.y.push_back(yi);
  }
}

template <typename TrajMatrix>
inline void fillRollouts(std::vector<VizRollout>& out, const std::vector<TrajMatrix>& sampled, const int x_idx,
                         const int y_idx, const int horizon, const std::vector<float>& costs,
                         const int max_rollouts = kDefaultMaxDrawRollouts)
{
  out.clear();
  if (sampled.empty() || costs.size() != sampled.size())
  {
    return;
  }

  std::vector<size_t> order(sampled.size());
  for (size_t i = 0; i < order.size(); ++i)
  {
    order[i] = i;
  }
  if (max_rollouts > 0 && static_cast<int>(order.size()) > max_rollouts)
  {
    std::partial_sort(order.begin(), order.begin() + static_cast<size_t>(max_rollouts), order.end(),
                      [&costs](const size_t a, const size_t b) { return costs[a] < costs[b]; });
    order.resize(static_cast<size_t>(max_rollouts));
  }

  std::sort(order.begin(), order.end(), [&costs](const size_t a, const size_t b) { return costs[a] > costs[b]; });

  out.reserve(order.size());
  // Sampled output trajectories omit t=0 (see MPPI calculateSampledStateTrajectories memcpy).
  if (horizon <= 1)
  {
    return;
  }
  const int cols = horizon - 1;
  for (const size_t idx : order)
  {
    VizRollout rollout;
    rollout.cost = costs[idx];
    extractPolyline(sampled[idx], x_idx, y_idx, cols, rollout);
    if (rollout.x.size() < 2U)
    {
      continue;
    }
    out.push_back(std::move(rollout));
  }
}

class PathTrackingViewer
{
public:
  PathTrackingViewer() = default;
  ~PathTrackingViewer();

  PathTrackingViewer(const PathTrackingViewer&) = delete;
  PathTrackingViewer& operator=(const PathTrackingViewer&) = delete;

  bool open(const char* title, const PathTrackingStaticScene& scene, const float camera_x, const float camera_y,
            const char* video_path = nullptr, const float video_fps = 10.0F, const float sim_dt = 0.0F);
  /** Returns false when the user closes the window or presses Esc. */
  bool showFrame(const PathTrackingVizFrame& frame, const PathTrackingEgoFootprint& ego,
                 const PathTrackingSignalLimits& limits);
  bool beginRecording(const char* output_path, const float fps);
  void stopRecording();
  bool isRecording() const;
  void close();

private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace viz
}  // namespace mppi
