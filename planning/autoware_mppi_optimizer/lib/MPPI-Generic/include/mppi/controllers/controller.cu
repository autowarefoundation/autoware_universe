#include <mppi/controllers/controller.cuh>

#define CONTROLLER_TEMPLATE                                                                                            \
  template <class DYN_T, class COST_T, class FB_T, class SAMPLING_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS,              \
            class PARAMS_T>
#define CONTROLLER Controller<DYN_T, COST_T, FB_T, SAMPLING_T, MAX_TIMESTEPS, NUM_ROLLOUTS, PARAMS_T>

CONTROLLER_TEMPLATE
void CONTROLLER::deallocateCUDAMemory()
{
  if (CUDA_mem_init_)
  {
    HANDLE_ERROR(cudaFree(initial_state_d_));
    HANDLE_ERROR(cudaFree(vis_initial_state_d_));
    HANDLE_ERROR(cudaFree(control_d_));
    HANDLE_ERROR(cudaFree(output_d_));
    HANDLE_ERROR(cudaFree(trajectory_costs_d_));
    HANDLE_ERROR(cudaFree(cost_baseline_and_norm_d_));
    CUDA_mem_init_ = false;
  }
  if (sampled_states_CUDA_mem_init_)
  {
    HANDLE_ERROR(cudaFree(sampled_outputs_d_));
    HANDLE_ERROR(cudaFree(sampled_costs_d_));
    sampled_states_CUDA_mem_init_ = false;
  }
}

// CONTROLLER_TEMPLATE
// void CONTROLLER::copyControlStdDevToDevice(bool synchronize)
// {
//   if (!CUDA_mem_init_)
//   {
//     return;
//   }
//   HANDLE_ERROR(cudaMemcpyAsync(control_std_dev_d_, params_.control_std_dev_.data(),
//                                sizeof(float) * params_.control_std_dev_.size(), cudaMemcpyHostToDevice, stream_));
//   if (synchronize)
//   {
//     HANDLE_ERROR(cudaStreamSynchronize(stream_));
//   }
// }

CONTROLLER_TEMPLATE
void CONTROLLER::copyNominalControlToDevice(bool synchronize)
{
  if (!CUDA_mem_init_)
  {
    return;
  }
  this->sampler_->copyImportanceSamplerToDevice(control_.data(), 0, synchronize);
}

CONTROLLER_TEMPLATE
void CONTROLLER::copySampledControlFromDevice(bool synchronize)
{
  // if mem is not inited don't use it
  if (!sampled_states_CUDA_mem_init_)
  {
    return;
  }

  int num_sampled_trajectories = perc_sampled_control_trajectories_ * NUM_ROLLOUTS;
  std::vector<int> samples(num_sampled_trajectories);
  if (perc_sampled_control_trajectories_ > 0.98)
  {
    // if above threshold just do everything
    std::iota(samples.begin(), samples.end(), 0);
  }
  else
  {
    // Create sample list without replacement
    // removes the top 2% since top 1% are complete noise
    samples = mppi::math::sample_without_replacement(num_sampled_trajectories, NUM_ROLLOUTS * 0.98);
  }

  // this explicitly adds the optimized control sequence
  HANDLE_ERROR(cudaMemcpyAsync(this->sampled_outputs_d_, this->output_.data(),
                               sizeof(float) * getNumTimesteps() * DYN_T::OUTPUT_DIM, cudaMemcpyHostToDevice,
                               this->vis_stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
      //  this->sampled_noise_d_,
      this->sampler_->getVisControlSample(0, 0, 0), this->control_d_,
      sizeof(float) * getNumTimesteps() * DYN_T::CONTROL_DIM, cudaMemcpyDeviceToDevice, this->vis_stream_));

  for (int i = 1; i < num_sampled_trajectories; i++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->sampled_outputs_d_ + i * getNumTimesteps() * DYN_T::OUTPUT_DIM,
                                 this->output_d_ + samples[i] * getNumTimesteps() * DYN_T::OUTPUT_DIM,
                                 sizeof(float) * getNumTimesteps() * DYN_T::OUTPUT_DIM, cudaMemcpyDeviceToDevice,
                                 this->vis_stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
        //  this->sampled_noise_d_ + i * getNumTimesteps() * DYN_T::CONTROL_DIM,
        this->sampler_->getVisControlSample(i, 0, 0), this->sampler_->getControlSample(samples[i], 0, 0),
        //  this->control_noise_d_ + samples[i] * getNumTimesteps() * DYN_T::CONTROL_DIM,
        sizeof(float) * getNumTimesteps() * DYN_T::CONTROL_DIM, cudaMemcpyDeviceToDevice, this->vis_stream_));
  }
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->vis_stream_));
  }
}

CONTROLLER_TEMPLATE
std::pair<int, float> CONTROLLER::findMinIndexAndValue(std::vector<int>& temp_list)
{
  if (temp_list.size() == 0)
  {
    return std::make_pair(0, 0.0);
  }
  int min_sample_index = 0;
  float min_sample_value = this->trajectory_costs_[temp_list[min_sample_index]];

  for (int index = 1; index < temp_list.size(); index++)
  {
    if (this->trajectory_costs_[temp_list[index]] < min_sample_value)
    {
      min_sample_value = this->trajectory_costs_[temp_list[index]];
      min_sample_index = index;
    }
  }
  return std::make_pair(min_sample_index, min_sample_value);
}

CONTROLLER_TEMPLATE
void CONTROLLER::copyTopControlFromDevice(bool synchronize)
{
  // if mem is not inited don't use it
  if (!sampled_states_CUDA_mem_init_ || num_top_control_trajectories_ <= 0)
  {
    return;
  }

  // Important note: Highest weighted trajectories are the ones with the lowest cost
  int start_top_control_traj_index = perc_sampled_control_trajectories_ * NUM_ROLLOUTS;
  std::vector<int> samples(num_top_control_trajectories_);
  // Start by filling in the top samples list with the first n in the trajectory
  for (int i = 0; i < num_top_control_trajectories_; i++)
  {
    samples[i] = i;
  }

  // Calculate min weight in the current top samples list
  int min_sample_index = 0;
  float min_sample_value = 0;
  std::tie(min_sample_index, min_sample_value) = findMinIndexAndValue(samples);

  // find top n samples by removing the smallest weights from the list
  for (int i = num_top_control_trajectories_; i < NUM_ROLLOUTS; i++)
  {
    if (trajectory_costs_[i] > min_sample_value)
    {  // Remove the smallest weight in the current list and add the new index
      samples[min_sample_index] = i;
      // recalculate min weight in the current list
      std::tie(min_sample_index, min_sample_value) = findMinIndexAndValue(samples);
    }
  }

  // Copy top n samples to this->sampled_noise_d_ after the randomly sampled trajectories
  top_n_costs_.resize(num_top_control_trajectories_);
  for (int i = 0; i < num_top_control_trajectories_; i++)
  {
    top_n_costs_[i] = trajectory_costs_[samples[i]] / getNormalizerCost();
    HANDLE_ERROR(cudaMemcpyAsync(
        this->sampled_outputs_d_ + (start_top_control_traj_index + i) * getNumTimesteps() * DYN_T::OUTPUT_DIM,
        this->output_d_ + samples[i] * getNumTimesteps() * DYN_T::OUTPUT_DIM,
        sizeof(float) * getNumTimesteps() * DYN_T::OUTPUT_DIM, cudaMemcpyDeviceToDevice, this->vis_stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
        // this->sampled_noise_d_ + (start_top_control_traj_index + i) * getNumTimesteps() * DYN_T::CONTROL_DIM,
        this->sampler_->getVisControlSample(start_top_control_traj_index + i, 0, 0),
        this->sampler_->getControlSample(samples[i], 0, 0),
        // this->control_noise_d_ + samples[i] * getNumTimesteps() * DYN_T::CONTROL_DIM,
        sizeof(float) * getNumTimesteps() * DYN_T::CONTROL_DIM, cudaMemcpyDeviceToDevice, this->vis_stream_));
  }
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(this->vis_stream_));
  }
}

CONTROLLER_TEMPLATE
void CONTROLLER::setCUDAStream(cudaStream_t stream)
{
  stream_ = stream;
  model_->bindToStream(stream);
  cost_->bindToStream(stream);
  fb_controller_->bindToStream(stream);
  sampler_->bindToStream(stream);
  curandSetStream(gen_, stream);  // requires the generator to be created!
}

CONTROLLER_TEMPLATE
void CONTROLLER::createAndSeedCUDARandomNumberGen()
{
  // Seed the PseudoRandomGenerator with the CPU time.
  curandCreateGenerator(&gen_, CURAND_RNG_PSEUDO_DEFAULT);
  setSeedCUDARandomNumberGen(this->params_.seed_);
}

CONTROLLER_TEMPLATE
void CONTROLLER::setSeedCUDARandomNumberGen(unsigned seed)
{
  // Seed the PseudoRandomGenerator with the CPU time.
  curandSetPseudoRandomGeneratorSeed(gen_, seed);
  // Reset the offset so setting the seed multiple times returns the same samples
  curandSetGeneratorOffset(gen_, 0);
}

CONTROLLER_TEMPLATE
void CONTROLLER::allocateCUDAMemoryHelper(int nominal_size, bool allocate_double_noise)
{
  if (nominal_size < 0)
  {
    nominal_size = 1;
    std::cerr << "nominal size cannot be below 0 when allocateCudaMemoryHelper is called" << std::endl;
    std::exit(-1);
  }
  else
  {
    // increment by 1 since actual is not included
    ++nominal_size;
  }
  HANDLE_ERROR(cudaMalloc((void**)&initial_state_d_, sizeof(float) * DYN_T::STATE_DIM * nominal_size));
  HANDLE_ERROR(cudaMalloc((void**)&vis_initial_state_d_, sizeof(float) * DYN_T::STATE_DIM * nominal_size));
  HANDLE_ERROR(cudaMalloc((void**)&control_d_, sizeof(float) * DYN_T::CONTROL_DIM * MAX_TIMESTEPS * nominal_size));
  HANDLE_ERROR(
      cudaMalloc((void**)&output_d_, sizeof(float) * DYN_T::OUTPUT_DIM * MAX_TIMESTEPS * NUM_ROLLOUTS * nominal_size));
  HANDLE_ERROR(cudaMalloc((void**)&trajectory_costs_d_, sizeof(float) * NUM_ROLLOUTS * nominal_size));
  // HANDLE_ERROR(cudaMalloc((void**)&control_std_dev_d_, sizeof(float) * DYN_T::CONTROL_DIM));
  // HANDLE_ERROR(cudaMalloc((void**)&control_noise_d_, sizeof(float) * DYN_T::CONTROL_DIM * MAX_TIMESTEPS *
  // NUM_ROLLOUTS *
  //                                                        (allocate_double_noise ? nominal_size : 1)));
  HANDLE_ERROR(cudaMalloc((void**)&cost_baseline_and_norm_d_, sizeof(float2) * nominal_size));
  cost_baseline_and_norm_.resize(nominal_size, make_float2(0.0, 0.0));
  CUDA_mem_init_ = true;
}

CONTROLLER_TEMPLATE
void CONTROLLER::resizeSampledControlTrajectories(float perc, int multiplier, int top_num)
{
  int num_sampled_trajectories = perc * NUM_ROLLOUTS + top_num;

  if (sampled_states_CUDA_mem_init_)
  {
    cudaFree(sampled_outputs_d_);
    // cudaFree(sampled_noise_d_);
    cudaFree(sampled_costs_d_);
    cudaFree(sampled_crash_status_d_);
    sampled_states_CUDA_mem_init_ = false;
  }
  sampled_trajectories_.resize(num_sampled_trajectories * multiplier, output_trajectory::Zero());
  sampled_costs_.resize(num_sampled_trajectories * multiplier, cost_trajectory::Zero());
  sampled_crash_status_.resize(num_sampled_trajectories * multiplier, crash_status_trajectory::Zero());
  sampler_->setNumVisRollouts(num_sampled_trajectories);
  if (num_sampled_trajectories <= 0)
  {
    return;
  }

  HANDLE_ERROR(cudaMalloc((void**)&sampled_outputs_d_,
                          sizeof(float) * DYN_T::OUTPUT_DIM * MAX_TIMESTEPS * num_sampled_trajectories * multiplier));
  // HANDLE_ERROR(cudaMalloc((void**)&sampled_noise_d_,
  //                         sizeof(float) * DYN_T::CONTROL_DIM * MAX_TIMESTEPS * num_sampled_trajectories *
  //                         multiplier));
  // +1 for terminal cost
  HANDLE_ERROR(cudaMalloc((void**)&sampled_costs_d_,
                          sizeof(float) * (MAX_TIMESTEPS + 1) * num_sampled_trajectories * multiplier));
  HANDLE_ERROR(cudaMalloc((void**)&sampled_crash_status_d_,
                          sizeof(int) * MAX_TIMESTEPS * num_sampled_trajectories * multiplier));
  sampled_states_CUDA_mem_init_ = true;
}

CONTROLLER_TEMPLATE
std::vector<float> CONTROLLER::getSampledNoise()
{
  std::vector<float> vector = std::vector<float>(NUM_ROLLOUTS * getNumTimesteps() * DYN_T::CONTROL_DIM, FLT_MIN);

  HANDLE_ERROR(cudaMemcpyAsync(vector.data(), this->sampler_->getControlSample(0, 0, 0),
                               sizeof(float) * NUM_ROLLOUTS * getNumTimesteps() * DYN_T::CONTROL_DIM,
                               cudaMemcpyDeviceToHost, stream_));
  HANDLE_ERROR(cudaStreamSynchronize(stream_));
  return vector;
}

#undef CONTROLLER_TEMPLATE
#undef CONTROLLER
