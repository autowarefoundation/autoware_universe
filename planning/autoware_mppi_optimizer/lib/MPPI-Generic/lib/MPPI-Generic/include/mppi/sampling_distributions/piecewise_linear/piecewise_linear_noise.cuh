#pragma once
/**
 * Created by David, Apr 11, 2021
 */

#include <curand.h>
#include <Eigen/Dense>
#include <mppi/utils/gpu_err_chk.cuh>

#include <algorithm>
#include <iostream>
#include <vector>

__global__ void createPiecewiseLinearNoise(const int num_timesteps, const int num_trajectories, const int control_dim,
                                           const int num_piecewise_segments, const int optimization_stride,
                                           const float* scale_piecewise_noise, const float* frac_add_nominal_traj,
                                           const float* scale_add_nominal_noise, const unsigned int* switch_num,
                                           const float* switch_times, const float* switch_values,
                                           float* nominal_control, const float* control_std_dev, float* output)
{
  int sample_index = blockIdx.x * blockDim.x + threadIdx.x;
  int time_index = blockIdx.y * blockDim.y + threadIdx.y;
  int control_index = blockIdx.z * blockDim.z + threadIdx.z;
  if (sample_index >= num_trajectories || time_index >= num_timesteps || control_index >= control_dim)
  {
    return;
  }

  float output_val = 0.0;

  if (sample_index == 1 || time_index < optimization_stride)
  {
    // if sample index = 1, use nominal control
    // sample_index = 0 is replaced by 0 controls later in the kernel
    output_val = nominal_control[time_index * control_dim + control_index];
  }
  else if (float(sample_index) < frac_add_nominal_traj[0] * float(num_trajectories))
  {
    // randomly vary output_val for frac_add_nominal_traj fraction of the trajectories
    output_val =
        output[(sample_index * num_timesteps + time_index) * control_dim + control_index] * scale_add_nominal_noise[0];
    output_val += nominal_control[time_index * control_dim + control_index];
  }
  else
  {
    // all others, use piecewise linear noise.

    // determine start/stop times and piecewise fraction
    int switch_num_val = switch_num[sample_index * control_dim + control_index];
    switch_num_val = min(switch_num_val, num_piecewise_segments);
    int segment_index = 0;
    float start_time = 0.0f;
    float end_time = 1.0f;
    float time_frac = float(time_index) / float(num_timesteps);

    for (int i = 0; i < switch_num_val; i++)
    {
      float switch_time = switch_times[(sample_index * (num_piecewise_segments + 1) + i) * control_dim + control_index];
      if (switch_time < time_frac)
      {
        segment_index += 1;
        if (start_time < switch_time)
        {
          start_time = switch_time;
        }
      }
      if (switch_time > time_frac)
      {
        if (end_time > switch_time)
        {
          end_time = switch_time;
        }
      }
    }

    // compute noise, interpolated between first and second value
    float first_val = 0;
    if (start_time < float(optimization_stride) / float(num_timesteps))
    {  // first value should always be nominal control at optimization stride start
      first_val = nominal_control[optimization_stride * control_dim + control_index];
      start_time = float(optimization_stride) / float(num_timesteps);
    }
    else
    {
      first_val =
          switch_values[(sample_index * (num_piecewise_segments + 1) + segment_index) * control_dim + control_index];
    }
    float second_val =
        switch_values[(sample_index * (num_piecewise_segments + 1) + segment_index + 1) * control_dim + control_index];
    float frac_interval = (time_frac - start_time) / (end_time - start_time);
    output_val = (1.0f - frac_interval) * first_val + frac_interval * second_val;
    output_val = output_val * 2.0 - 1.0;                             // scale to [-1, 1]
    output_val = output_val * scale_piecewise_noise[control_index];  // scale by scale_piecewise_noise

    if (float(sample_index) < (frac_add_nominal_traj[0] + frac_add_nominal_traj[1]) * float(num_trajectories))
    {
      // randomly vary output_val for frac_add_nominal_traj[1] fraction of the trajectories
      output_val = output_val * scale_add_nominal_noise[1];
      output_val += nominal_control[time_index * control_dim + control_index];
    }
  }

  // control noise output gets scaled by the kernel, and added to nominal control.
  // so we need to undo these operations to get exactly the control trajectory we want.
  output[(sample_index * num_timesteps + time_index) * control_dim + control_index] =
      output_val / control_std_dev[control_index] - nominal_control[time_index * control_dim + control_index];
}

void piecewise_linear_noise(const int num_timesteps, const int num_trajectories, const int control_dim,
                            const int num_piecewise_segments, const int optimization_stride,
                            std::vector<float>& scale_piecewise_noise, std::vector<float>& frac_add_nominal_traj,
                            std::vector<float>& scale_add_nominal_noise, float* control_d, float* control_noise_d,
                            const float* control_std_dev_d, curandGenerator_t& gen, cudaStream_t stream = 0)
{
  // generate piecewise linear random noise, in 2 steps:
  // 1. randomly decide start/stop times of segments (uniformly divide t=0:T)
  // 2. randomly decide start and stop values of segments (uniform within bounds)

  const int BLOCKSIZE_X = 32;
  const int BLOCKSIZE_Y = 32;
  const int BLOCKSIZE_Z = 1;

  // try to sample num_piecewise_segments with reasonable poisson distribution
  const float switch_num_poisson_lambda = 0.5f * float(num_piecewise_segments);

  // create random start/stop times and values
  // curandSetStream(gen, stream);
  unsigned int* switch_num_d;
  float* switch_times_d;
  float* switch_values_d;
  HANDLE_ERROR(cudaMalloc((void**)&switch_num_d, sizeof(unsigned int) * num_trajectories * control_dim));
  HANDLE_ERROR(cudaMalloc((void**)&switch_times_d,
                          sizeof(float) * num_trajectories * (num_piecewise_segments + 1) * control_dim));
  HANDLE_ERROR(cudaMalloc((void**)&switch_values_d,
                          sizeof(float) * num_trajectories * (num_piecewise_segments + 1) * control_dim));
  HANDLE_CURAND_ERROR(curandGeneratePoisson(gen, (unsigned int*)switch_num_d, num_trajectories * control_dim,
                                            switch_num_poisson_lambda));
  HANDLE_CURAND_ERROR(curandGenerateUniform(gen, (float*)switch_times_d,
                                            num_trajectories * (num_piecewise_segments + 1) * control_dim));
  HANDLE_CURAND_ERROR(curandGenerateUniform(gen, (float*)switch_values_d,
                                            num_trajectories * (num_piecewise_segments + 1) * control_dim));

  float* scale_piecewise_noise_d;
  float* frac_add_nominal_traj_d;
  float* scale_add_nominal_noise_d;
  HANDLE_ERROR(cudaMalloc((void**)&scale_piecewise_noise_d, sizeof(float) * control_dim));
  HANDLE_ERROR(cudaMalloc((void**)&frac_add_nominal_traj_d, sizeof(float) * control_dim));
  HANDLE_ERROR(cudaMalloc((void**)&scale_add_nominal_noise_d, sizeof(float) * control_dim));
  HANDLE_ERROR(cudaMemcpy(scale_piecewise_noise_d, scale_piecewise_noise.data(), sizeof(float) * control_dim,
                          cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(frac_add_nominal_traj_d, frac_add_nominal_traj.data(), sizeof(float) * control_dim,
                          cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(scale_add_nominal_noise_d, scale_add_nominal_noise.data(), sizeof(float) * control_dim,
                          cudaMemcpyHostToDevice));

  // create piecewise linear noise
  const int grid_x = (num_trajectories - 1) / BLOCKSIZE_X + 1;
  const int grid_y = (num_timesteps - 1) / BLOCKSIZE_Y + 1;
  const int grid_z = (control_dim - 1) / BLOCKSIZE_Z + 1;
  dim3 grid(grid_x, grid_y, grid_z);
  dim3 block(BLOCKSIZE_X, BLOCKSIZE_Y, BLOCKSIZE_Z);
  createPiecewiseLinearNoise<<<grid, block, 0, stream>>>(
      num_timesteps, num_trajectories, control_dim, num_piecewise_segments, optimization_stride,
      scale_piecewise_noise_d, frac_add_nominal_traj_d, scale_add_nominal_noise_d, switch_num_d, switch_times_d,
      switch_values_d, control_d, control_std_dev_d, control_noise_d);

  // cleanup
  HANDLE_ERROR(cudaGetLastError());
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  HANDLE_ERROR(cudaFree(switch_times_d));
  HANDLE_ERROR(cudaFree(switch_values_d));
  HANDLE_ERROR(cudaFree(scale_piecewise_noise_d));
}
