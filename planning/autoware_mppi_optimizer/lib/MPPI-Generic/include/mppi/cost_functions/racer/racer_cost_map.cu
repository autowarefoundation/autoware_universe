#include <mppi/cost_functions/racer/racer_cost_map.cuh>

#include <algorithm>
#include <cstring>

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::RacerCostMapImpl(cudaStream_t stream)
{
  this->bindToStream(stream);
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    if (costmapArray_d_ != nullptr)
    {
      HANDLE_ERROR(cudaFreeArray(costmapArray_d_));
      costmapArray_d_ = nullptr;
    }
    if (costmap_tex_d_ != 0)
    {
      HANDLE_ERROR(cudaDestroyTextureObject(costmap_tex_d_));
      costmap_tex_d_ = 0;
    }
  }
  PARENT_CLASS::freeCudaMem();
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::paramsToDevice()
{
  HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->params_, &this->params_, sizeof(PARAMS_T), cudaMemcpyHostToDevice,
                               this->stream_));
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::setWorldToCostmapBounds(float x_min, float x_max, float y_min,
                                                                             float y_max)
{
  const float inv_x = 1.0F / (x_max - x_min);
  const float inv_y = 1.0F / (y_max - y_min);
  this->params_.r_c1 = make_float3(inv_x, 0.0F, 0.0F);
  this->params_.r_c2 = make_float3(0.0F, inv_y, 0.0F);
  this->params_.trs = make_float3(-x_min * inv_x, -y_min * inv_y, 1.0F);
  if (this->GPUMemStatus_)
  {
    paramsToDevice();
  }
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::coorTransform(float x, float y, float* u,
                                                                                       float* v, float* w) const
{
  *u = this->params_.r_c1.x * x + this->params_.r_c2.x * y + this->params_.trs.x;
  *v = this->params_.r_c1.y * x + this->params_.r_c2.y * y + this->params_.trs.y;
  *w = this->params_.r_c1.z * x + this->params_.r_c2.z * y + this->params_.trs.z;
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
__device__ float4 RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::queryTextureTransformed(float x, float y) const
{
  float u = 0.0F;
  float v = 0.0F;
  float w = 0.0F;
  coorTransform(x, y, &u, &v, &w);
  return tex2D<float4>(costmap_tex_d_, u / w, v / w);
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::computeStateCost(float* y, int timestep, float* theta_c,
                                                                                int* crash_status)
{
  (void)timestep;
  (void)theta_c;
  (void)crash_status;

  const float x = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
  const float vel = y[static_cast<int>(RacerDubinsParams::OutputIndex::TOTAL_VELOCITY)];

  const float track_val = queryTextureTransformed(x, y_pos).x;

  const float vel_diff = vel - this->params_.desired_speed;
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  float crash_cost = 0.0F;
  if (track_val >= this->params_.boundary_threshold)
  {
    crash_cost = this->params_.crash_coeff;
  }

  return speed_cost + track_cost + crash_cost;
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
float RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::computeStateCost(const Eigen::Ref<const output_array>& y,
                                                                     int timestep, int* crash_status)
{
  (void)timestep;
  if (cpu_costmap_.empty())
  {
    return 0.0F;
  }

  const float x = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
  const float vel = y[static_cast<int>(RacerDubinsParams::OutputIndex::TOTAL_VELOCITY)];

  float u = 0.0F;
  float v = 0.0F;
  float w = 0.0F;
  coorTransform(x, y_pos, &u, &v, &w);

  const float norm_u = u / w;
  const float norm_v = v / w;

  const int pixel_u =
      std::max(0, std::min(static_cast<int>(norm_u * cpu_costmap_.cols), cpu_costmap_.cols - 1));
  const int pixel_v =
      std::max(0, std::min(static_cast<int>(norm_v * cpu_costmap_.rows), cpu_costmap_.rows - 1));

  const float track_val = cpu_costmap_.at<float>(pixel_v, pixel_u);

  const float vel_diff = vel - this->params_.desired_speed;
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  float crash_cost = 0.0F;
  if (track_val >= this->params_.boundary_threshold)
  {
    crash_cost = this->params_.crash_coeff;
    if (crash_status != nullptr)
    {
      *crash_status = 1;
    }
  }

  return speed_cost + track_cost + crash_cost;
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::computeControlCost(float* u, int timestep, float* theta_c,
                                                                                  int* crash)
{
  (void)timestep;
  (void)theta_c;
  (void)crash;
  const float steer = u[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)];
  return this->params_.steer_coeff * (steer * steer);
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
float RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::computeControlCost(const Eigen::Ref<const control_array>& u,
                                                                         int timestep, int* crash)
{
  (void)timestep;
  (void)crash;
  const float steer = u(static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD));
  return this->params_.steer_coeff * (steer * steer);
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
__device__ float RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::terminalCost(float* y, float* theta_c)
{
  (void)theta_c;
  const float x = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
  const float track_val = queryTextureTransformed(x, y_pos).x;
  return this->params_.track_coeff * track_val * 10.0F;
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::setCpuCostmap(const cv::Mat& costmap)
{
  cpu_costmap_ = costmap;
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::costmapToTexture(int width, int height, float4* host_data)
{
  width_ = width;
  height_ = height;
  channelDesc_ = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
  HANDLE_ERROR(cudaMallocArray(&costmapArray_d_, &channelDesc_, width, height));
  HANDLE_ERROR(
      cudaMemcpyToArray(costmapArray_d_, 0, 0, host_data, width * height * sizeof(float4), cudaMemcpyHostToDevice));

  cudaResourceDesc resDesc{};
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = costmapArray_d_;

  cudaTextureDesc texDesc{};
  memset(&texDesc, 0, sizeof(texDesc));
  texDesc.addressMode[0] = cudaAddressModeClamp;
  texDesc.addressMode[1] = cudaAddressModeClamp;
  texDesc.filterMode = cudaFilterModeLinear;
  texDesc.readMode = cudaReadModeElementType;
  texDesc.normalizedCoords = 1;

  HANDLE_ERROR(cudaCreateTextureObject(&costmap_tex_d_, &resDesc, &texDesc, nullptr));

  if (this->cost_d_ != nullptr)
  {
    HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->costmap_tex_d_, &costmap_tex_d_, sizeof(cudaTextureObject_t),
                                 cudaMemcpyHostToDevice, this->stream_));
  }
}

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
void RacerCostMapImpl<CLASS_T, PARAMS_T, DYN_PARAMS_T>::updateCostmapTexture(float4* host_data)
{
  if (costmapArray_d_ == nullptr)
  {
    printf("Error: Costmap array not initialized! Call costmapToTexture first.\n");
    return;
  }

  HANDLE_ERROR(cudaMemcpyToArray(costmapArray_d_, 0, 0, host_data, width_ * height_ * sizeof(float4),
                                 cudaMemcpyHostToDevice));
}
