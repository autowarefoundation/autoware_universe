template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
__host__ __device__ float CEMShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::computeWeight(
    float* traj_cost, float baseline, int global_idx)
{
  if (traj_cost[global_idx] >= baseline)
  {
    return (1.0 / ((int)(NUM_ROLLOUTS) * this->params_.gamma + 1));
  }
  else
  {
    return 0.0;
  }
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void CEMShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::computeWeights(cost_traj& trajectory_costs,
                                                                                     float* trajectory_costs_d,
                                                                                     cudaStream_t stream)
{
  // Copy the costs back to the host
  HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs.data(), trajectory_costs_d, NUM_ROLLOUTS * sizeof(float),
                               cudaMemcpyDeviceToHost, this->stream_));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  std::vector<int> indexes(NUM_ROLLOUTS);
  iota(indexes.begin(), indexes.end(), 0);

  std::nth_element(indexes.begin(), indexes.begin() + (int)(NUM_ROLLOUTS * this->params_.gamma), indexes.end(),
                   [&trajectory_costs](int i1, int i2) { return trajectory_costs(i1) > trajectory_costs(i2); });

  this->baseline_ = trajectory_costs(indexes[(int)NUM_ROLLOUTS * this->params_.gamma]);

  // launch the WeightKernel to calculate the weights
  if (this->params_.gamma == 0)
  {
    trajectory_costs(indexes[0]) = 1.0;
    for (int i = 1; i < NUM_ROLLOUTS; i++)
    {
      trajectory_costs(indexes[i]) = 0.0;
    }
  }
  else
  {
    CLASS_T* derived = static_cast<CLASS_T*>(this);
    derived->launchWeightKernel(trajectory_costs_d, this->baseline_, stream);

    HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs.data(), trajectory_costs_d, NUM_ROLLOUTS * sizeof(float),
                                 cudaMemcpyDeviceToHost, stream));
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}
