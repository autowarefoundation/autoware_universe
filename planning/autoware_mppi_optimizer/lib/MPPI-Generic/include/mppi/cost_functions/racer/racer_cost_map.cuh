/**
 * Costmap-based tracking cost for RacerDubins (GPU texture lookup + optional CPU mirror).
 */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_RACER_COST_MAP_CUH_
#define MPPI_COST_FUNCTIONS_RACER_COST_MAP_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>

#include <opencv2/core.hpp>

struct RacerCostMapParams : public CostParams<2>
{
  float desired_speed = 2.5F;
  float speed_coeff = 20.0F;
  float track_coeff = 500.0F;
  float crash_coeff = 10000.0F;
  float boundary_threshold = 0.8F;
  float steer_coeff = 50.0F;

  float3 r_c1 = make_float3(1, 0, 0);
  float3 r_c2 = make_float3(0, 1, 0);
  float3 trs = make_float3(0, 0, 1);
};

template <class CLASS_T, class PARAMS_T = RacerCostMapParams, class DYN_PARAMS_T = RacerDubinsParams>
class RacerCostMapImpl : public Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>;
  using output_array = typename PARENT_CLASS::output_array;
  using control_array = typename PARENT_CLASS::control_array;

  RacerCostMapImpl(cudaStream_t stream = 0);

  void freeCudaMem();

  void paramsToDevice();

  __host__ __device__ void coorTransform(float x, float y, float* u, float* v, float* w) const;

  __device__ float4 queryTextureTransformed(float x, float y) const;

  __device__ float computeStateCost(float* y, int timestep, float* theta_c, int* crash_status);

  float computeStateCost(const Eigen::Ref<const output_array>& y, int timestep, int* crash_status);

  __device__ float computeControlCost(float* u, int timestep, float* theta_c, int* crash);

  float computeControlCost(const Eigen::Ref<const control_array>& u, int timestep, int* crash);

  __device__ float terminalCost(float* y, float* theta_c);

  void setCpuCostmap(const cv::Mat& costmap);

  void setWorldToCostmapBounds(float x_min, float x_max, float y_min, float y_max);

  void costmapToTexture(int width, int height, float4* host_data);

  void updateCostmapTexture(float4* host_data);

  cudaArray* costmapArray_d_ = nullptr;
  cudaTextureObject_t costmap_tex_d_ = 0;
  cudaChannelFormatDesc channelDesc_{};
  int width_ = 0;
  int height_ = 0;

  cv::Mat cpu_costmap_;
};

class RacerCostMap : public RacerCostMapImpl<RacerCostMap>
{
public:
  RacerCostMap(cudaStream_t stream = 0) : RacerCostMapImpl<RacerCostMap>(stream)
  {
  }
};

#if __CUDACC__
#include "racer_cost_map.cu"
#endif

#endif  // MPPI_COST_FUNCTIONS_RACER_COST_MAP_CUH_
