/**
 * GPU-side rollout polyline extraction for live visualization (avoids full OUTPUT_DIM D2H).
 */
#pragma once

#include <mppi/viz/path_tracking_viewer.hpp>

namespace mppi
{
namespace viz
{

struct RolloutVisDeviceView
{
  const float* outputs_d = nullptr;
  const float* costs_d = nullptr;
  int num_rollouts = 0;
  int num_timesteps = 0;
  int output_dim = 0;
  cudaStream_t stream = nullptr;
};

/** Pack top rollout (x, y) polylines on GPU; one small D2H for ImGui drawing. */
bool fillRolloutsFromDevice(std::vector<VizRollout>& out, const RolloutVisDeviceView& view, const int x_idx,
                            const int y_idx, const int horizon, const int max_rollouts = kDefaultMaxDrawRollouts);

}  // namespace viz
}  // namespace mppi
