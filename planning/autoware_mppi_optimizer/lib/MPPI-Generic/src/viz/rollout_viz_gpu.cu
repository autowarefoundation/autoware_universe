#include <mppi/viz/rollout_viz_gpu.cuh>

#include <mppi/utils/gpu_err_chk.cuh>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

namespace
{
__global__ void sumSampledRolloutCostsKernel(const float* __restrict__ costs_d, const int num_rollouts,
                                             const int num_timesteps, float* __restrict__ totals_d)
{
  const int rollout = blockIdx.x;
  if (rollout >= num_rollouts)
  {
    return;
  }

  // visualizeCostKernel layout: running costs at rollout * num_timesteps + t,
  // terminal at rollout * (num_timesteps + 1) + num_timesteps (not a contiguous T+1 block).
  float sum = 0.0F;
  for (int t = threadIdx.x; t < num_timesteps; t += blockDim.x)
  {
    sum += costs_d[rollout * num_timesteps + t];
  }

  __shared__ float shared_sum[256];
  shared_sum[threadIdx.x] = sum;
  __syncthreads();

  for (int stride = blockDim.x / 2; stride > 0; stride >>= 1)
  {
    if (threadIdx.x < stride)
    {
      shared_sum[threadIdx.x] += shared_sum[threadIdx.x + stride];
    }
    __syncthreads();
  }

  if (threadIdx.x == 0)
  {
    totals_d[rollout] = shared_sum[0] + costs_d[rollout * (num_timesteps + 1) + num_timesteps];
  }
}

__global__ void gatherRolloutXYKernel(const float* __restrict__ outputs_d, const int* __restrict__ rollout_indices,
                                      const int num_selected, const int num_timesteps, const int output_dim,
                                      const int x_idx, const int y_idx, float* __restrict__ xy_out_d)
{
  const int slot = blockIdx.x;
  const int t = threadIdx.x;
  if (slot >= num_selected)
  {
    return;
  }

  const int cols = num_timesteps - 1;
  if (t >= cols)
  {
    return;
  }

  const int rollout = rollout_indices[slot];
  const float* y_out = outputs_d + rollout * num_timesteps * output_dim + t * output_dim;
  const int base = slot * cols * 2;
  xy_out_d[base + t * 2 + 0] = y_out[x_idx];
  xy_out_d[base + t * 2 + 1] = y_out[y_idx];
}

struct RolloutVizGpuCache
{
  float* totals_d = nullptr;
  int* indices_d = nullptr;
  float* xy_d = nullptr;
  int capacity_rollouts = 0;
  int capacity_selected = 0;
  int capacity_cols = 0;

  ~RolloutVizGpuCache()
  {
    if (totals_d != nullptr)
    {
      cudaFree(totals_d);
    }
    if (indices_d != nullptr)
    {
      cudaFree(indices_d);
    }
    if (xy_d != nullptr)
    {
      cudaFree(xy_d);
    }
  }

  void ensure(const int num_rollouts, const int num_selected, const int cols)
  {
    if (num_rollouts > capacity_rollouts)
    {
      if (totals_d != nullptr)
      {
        cudaFree(totals_d);
      }
      HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&totals_d), static_cast<size_t>(num_rollouts) * sizeof(float)));
      capacity_rollouts = num_rollouts;
    }
    if (num_selected > capacity_selected || cols > capacity_cols)
    {
      if (indices_d != nullptr)
      {
        cudaFree(indices_d);
      }
      if (xy_d != nullptr)
      {
        cudaFree(xy_d);
      }
      HANDLE_ERROR(
          cudaMalloc(reinterpret_cast<void**>(&indices_d), static_cast<size_t>(num_selected) * sizeof(int)));
      HANDLE_ERROR(cudaMalloc(reinterpret_cast<void**>(&xy_d),
                              static_cast<size_t>(num_selected) * static_cast<size_t>(cols) * 2U * sizeof(float)));
      capacity_selected = num_selected;
      capacity_cols = cols;
    }
  }
};

RolloutVizGpuCache& rolloutVizCache()
{
  static RolloutVizGpuCache cache;
  return cache;
}
}  // namespace

namespace mppi
{
namespace viz
{

bool fillRolloutsFromDevice(std::vector<VizRollout>& out, const RolloutVisDeviceView& view, const int x_idx,
                            const int y_idx, const int horizon, const int max_rollouts)
{
  out.clear();
  if (view.outputs_d == nullptr || view.costs_d == nullptr || view.num_rollouts <= 0 || view.num_timesteps <= 1 ||
      view.output_dim <= 0 || x_idx < 0 || y_idx < 0 || x_idx >= view.output_dim || y_idx >= view.output_dim)
  {
    return false;
  }

  const int cols = std::min(horizon - 1, view.num_timesteps - 1);
  if (cols <= 0)
  {
    return false;
  }

  const int draw_n =
      (max_rollouts > 0) ? std::min(max_rollouts, view.num_rollouts) : view.num_rollouts;
  if (draw_n <= 0)
  {
    return false;
  }

  RolloutVizGpuCache& cache = rolloutVizCache();
  cache.ensure(view.num_rollouts, draw_n, cols);

  const int block_size = 256;
  sumSampledRolloutCostsKernel<<<view.num_rollouts, block_size, 0, view.stream>>>(
      view.costs_d, view.num_rollouts, view.num_timesteps, cache.totals_d);
  HANDLE_ERROR(cudaGetLastError());

  std::vector<float> totals(static_cast<size_t>(view.num_rollouts));
  HANDLE_ERROR(cudaMemcpyAsync(totals.data(), cache.totals_d, totals.size() * sizeof(float), cudaMemcpyDeviceToHost,
                               view.stream));
  HANDLE_ERROR(cudaStreamSynchronize(view.stream));

  std::vector<int> order(static_cast<size_t>(view.num_rollouts));
  std::iota(order.begin(), order.end(), 0);
  if (draw_n < view.num_rollouts)
  {
    std::partial_sort(order.begin(), order.begin() + draw_n, order.end(),
                      [&totals](const int a, const int b) { return totals[static_cast<size_t>(a)] < totals[static_cast<size_t>(b)]; });
    order.resize(static_cast<size_t>(draw_n));
  }
  std::sort(order.begin(), order.end(),
            [&totals](const int a, const int b) { return totals[static_cast<size_t>(a)] > totals[static_cast<size_t>(b)]; });

  HANDLE_ERROR(cudaMemcpyAsync(cache.indices_d, order.data(), order.size() * sizeof(int), cudaMemcpyHostToDevice,
                               view.stream));

  gatherRolloutXYKernel<<<draw_n, cols, 0, view.stream>>>(view.outputs_d, cache.indices_d, draw_n, view.num_timesteps,
                                                        view.output_dim, x_idx, y_idx, cache.xy_d);
  HANDLE_ERROR(cudaGetLastError());

  std::vector<float> xy_host(static_cast<size_t>(draw_n) * static_cast<size_t>(cols) * 2U);
  HANDLE_ERROR(cudaMemcpyAsync(xy_host.data(), cache.xy_d, xy_host.size() * sizeof(float), cudaMemcpyDeviceToHost,
                               view.stream));
  HANDLE_ERROR(cudaStreamSynchronize(view.stream));

  const float max_seg_sq = 6.0F * 6.0F;
  out.reserve(static_cast<size_t>(draw_n));
  for (int r = 0; r < draw_n; ++r)
  {
    VizRollout rollout;
    rollout.cost = totals[static_cast<size_t>(order[static_cast<size_t>(r)])];
    rollout.x.reserve(static_cast<size_t>(cols));
    rollout.y.reserve(static_cast<size_t>(cols));
    const size_t base = static_cast<size_t>(r) * static_cast<size_t>(cols) * 2U;
    for (int t = 0; t < cols; ++t)
    {
      const float xi = xy_host[base + static_cast<size_t>(t) * 2U + 0U];
      const float yi = xy_host[base + static_cast<size_t>(t) * 2U + 1U];
      if (t > 0 && std::abs(xi) < 1.0E-5F && std::abs(yi) < 1.0E-5F)
      {
        const float px = rollout.x.back();
        const float py = rollout.y.back();
        if (px * px + py * py > 1.0F)
        {
          break;
        }
      }
      if (!rollout.x.empty())
      {
        const float dx = xi - rollout.x.back();
        const float dy = yi - rollout.y.back();
        if (dx * dx + dy * dy > max_seg_sq)
        {
          break;
        }
      }
      rollout.x.push_back(xi);
      rollout.y.push_back(yi);
    }
    if (rollout.x.size() >= 2U)
    {
      out.push_back(std::move(rollout));
    }
  }

  return true;
}

}  // namespace viz
}  // namespace mppi
