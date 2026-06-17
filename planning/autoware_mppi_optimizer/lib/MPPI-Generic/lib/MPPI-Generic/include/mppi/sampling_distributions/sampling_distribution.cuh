#pragma once
/*
Created by Bogdan Vlahov on 3/22/2023
*/

#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/managed.cuh>

#include <string>

namespace mppi
{
namespace sampling_distributions
{
// Auto-align all Parameter structures to float4 so that any data after it in saved memory has full memory alignment
template <int C_DIM>
struct alignas(float4) SamplingParams
{
  static const int CONTROL_DIM = C_DIM;
  bool use_same_noise_for_all_distributions = true;
  int num_rollouts = 1;
  int num_timesteps = 1;
  int num_distributions = 1;
  int num_visualization_rollouts = 0;
  SamplingParams(int num_rollouts, int num_timesteps, int num_distributions = 1)
    : num_rollouts{ num_rollouts }, num_timesteps{ num_timesteps }, num_distributions{ num_distributions }
  {
  }
  SamplingParams() = default;
};

template <class CLASS_T, template <int> typename PARAMS_TEMPLATE, class DYN_PARAMS_T>
class SamplingDistribution : public Managed
{
public:
  /*************************************
   * Setup typedefs and aliases
   *************************************/

  typedef CLASS_T SAMPLING_T;
  using ControlIndex = typename DYN_PARAMS_T::ControlIndex;
  using OutputIndex = typename DYN_PARAMS_T::OutputIndex;
  using TEMPLATED_DYN_PARAMS = DYN_PARAMS_T;

  static const int CONTROL_DIM = C_IND_CLASS(DYN_PARAMS_T, NUM_CONTROLS);
  // static const int CONTROL_DIM = E_INDEX(ControlIndex, NUM_CONTROLS);
  typedef PARAMS_TEMPLATE<CONTROL_DIM> SAMPLING_PARAMS_T;
  typedef Eigen::Matrix<float, CONTROL_DIM, 1> control_array;

  static_assert(std::is_base_of<SamplingParams<CONTROL_DIM>, SAMPLING_PARAMS_T>::value,
                "Sampling Distribution PARAMS_T does not inherit from SamplingParams");

  /*************************************
   * Constructors and Destructors
   *************************************/

  SamplingDistribution(cudaStream_t stream = 0) : Managed(stream)
  {
    this->SHARED_MEM_REQUEST_GRD_BYTES = sizeof(SAMPLING_PARAMS_T);
    this->SHARED_MEM_REQUEST_BLK_BYTES = 0;
  }
  SamplingDistribution(const SAMPLING_PARAMS_T& params, cudaStream_t stream = 0) : params_{ params }, Managed(stream)
  {
    this->SHARED_MEM_REQUEST_GRD_BYTES = sizeof(SAMPLING_PARAMS_T);
    this->SHARED_MEM_REQUEST_BLK_BYTES = 0;
  }

  virtual ~SamplingDistribution()
  {
    freeCudaMem();
  }

  /**
   * @brief Get the Sampling Distribution Name object
   *
   * @return std::string - name of the sampling distribution
   */
  __host__ virtual std::string getSamplingDistributionName() const
  {
    return "Sampling distribution name not set";
  }

  /*************************************
   * DEFAULT CLASS METHODS THAT SHOULD NOT NEED OVERWRITING
   *************************************/

  void GPUSetup();

  /**
   * Updates the sampling distribution parameters
   * @param params
   */
  void setParams(const SAMPLING_PARAMS_T& params, bool synchronize = true)
  {
    bool reallocate_memory = params_.num_timesteps != params.num_timesteps ||
                             params_.num_rollouts != params.num_rollouts ||
                             params_.num_distributions != params.num_distributions;
    bool reallocate_vis_memory = params_.num_timesteps != params.num_timesteps ||
                                 params_.num_visualization_rollouts != params.num_visualization_rollouts ||
                                 params_.num_distributions != params.num_distributions;

    params_ = params;
    if (GPUMemStatus_)
    {
      if (reallocate_memory)
      {
        allocateCUDAMemory(false);
      }
      if (reallocate_vis_memory)
      {
        resizeVisualizationControlTrajectories(true);
      }
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice(synchronize);
    }
  }

  // __host__ __device__ SAMPLING_PARAMS_T getParams() const
  // {
  //   return params_;
  // }

  __host__ __device__ const SAMPLING_PARAMS_T getParams() const
  {
    return params_;
  }

  __host__ __device__ int getNumTimesteps() const
  {
    return this->params_.num_timesteps;
  }

  __host__ __device__ int getNumRollouts() const
  {
    return this->params_.num_rollouts;
  }

  __host__ __device__ int getNumVisRollouts() const
  {
    return this->params_.num_visualization_rollouts;
  }

  __host__ __device__ int getNumDistributions() const
  {
    return this->params_.num_distributions;
  }

  __host__ void setNumTimesteps(const int num_timesteps, bool synchronize = false)
  {
    const bool reallocate_memory = params_.num_timesteps != num_timesteps;
    this->params_.num_timesteps = num_timesteps;
    if (GPUMemStatus_ && reallocate_memory)
    {
      if (reallocate_memory)
      {
        allocateCUDAMemory(false);
        resizeVisualizationControlTrajectories(true);
      }
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice(synchronize);
    }
  }

  __host__ void setNumVisRollouts(const int num_visualization_rollouts, bool synchronize = false)
  {
    const bool reallocate_memory = params_.num_visualization_rollouts != num_visualization_rollouts;
    this->params_.num_visualization_rollouts = num_visualization_rollouts;
    if (GPUMemStatus_ && reallocate_memory)
    {
      if (reallocate_memory)
      {
        resizeVisualizationControlTrajectories(true);
      }
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice(synchronize);
    }
  }

  __host__ void setNumRollouts(const int num_rollouts, bool synchronize = false)
  {
    const bool reallocate_memory = params_.num_rollouts != num_rollouts;
    this->params_.num_rollouts = num_rollouts;
    if (GPUMemStatus_ && reallocate_memory)
    {
      if (reallocate_memory)
      {
        allocateCUDAMemory(false);
      }
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice(synchronize);
    }
  }

  __host__ void setNumDistributions(const int num_distributions, bool synchronize = false)
  {
    const bool reallocate_memory = params_.num_distributions != num_distributions;
    this->params_.num_distributions = num_distributions;
    if (GPUMemStatus_ && reallocate_memory)
    {
      if (reallocate_memory)
      {
        allocateCUDAMemory(false);
        resizeVisualizationControlTrajectories(true);
      }
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice(synchronize);
    }
  }

  __host__ void resizeVisualizationControlTrajectories(bool synchronize = true);

  __host__ void setVisStream(cudaStream_t stream)
  {
    vis_stream_ = stream;
  }

  __host__ void allocateCUDAMemory(bool synchronize = false);

  /**
   * @brief deallocates the allocated cuda memory for the sampling distribution
   */
  __host__ void freeCudaMem();

  /**
   * @brief Get a pointer to a specific control sample. This is useful for plugging into methods like enforceConstraints
   *
   * @param sample_index - sample number out of num_rollouts
   * @param t - timestep out of num_timesteps
   * @param distribution_index - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param output - output pointer for compatibility with a output-based sampling distribution
   * @return float* pointer to the control array that is at [distribution_index][sample_index][t]
   */
  __host__ __device__ float* getControlSample(const int& sample_index, const int& t, const int& distribution_index,
                                              const float* __restrict__ theta_d = nullptr,
                                              const float* __restrict__ output = nullptr);

  /**
   * @brief Get a pointer to a specific visualization control sample.
   *
   * @param sample_index - sample number out of num_visualization_rollouts
   * @param t - timestep out of num_timesteps
   * @param distribution_index - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param output - output pointer for compatibility with a output-based sampling distribution
   * @return float* pointer to the control array that is at [distribution_index][sample_index][t]
   */
  __host__ __device__ float* getVisControlSample(const int& sample_index, const int& t, const int& distribution_index,
                                                 const float* __restrict__ theta_d = nullptr,
                                                 const float* __restrict__ output = nullptr);

  /**
   * @brief Method for starting up any potential work for distributions. By default, it just loads the params into
   * shared memory
   *
   * @param output - initial output
   * @param t_0 - starting time
   * @param dt - step size
   * @param theta_d - shared memory pointer to sampling distribution space
   */
  __device__ void initializeDistributions(const float* __restrict__ output, const float t_0, const float dt,
                                          float* __restrict__ theta_d);

  __host__ void paramsToDevice(bool synchronize = true);

  /**
   * @brief Look up a specific control sample located at [distribution_index][sample_index][t] and put it into the
   * control array
   *
   * @param sample_index - sample number out of num_rollouts
   * @param t - timestep out of num_timesteps
   * @param distribution_index - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param control - pointer to fill with the specific control array
   * @param theta_d - shared memory pointer for passing through params
   * @param block_size - parallelizable step size for the gpu (normally blockDim.y)
   * @param thread_index - parallelizable index for the gpu (normally threadIdx.y)
   * @param output - output pointer for compatibility with a output-based sampling distribution
   */
  __device__ void readControlSample(const int& sample_index, const int& t, const int& distribution_index,
                                    float* __restrict__ control, float* __restrict__ theta_d, const int& block_size = 1,
                                    const int& thread_index = 1, const float* __restrict__ output = nullptr);

  /**
   * @brief Look up a specific visualization control sample located at [distribution_index][sample_index][t] and put it
   * into the control array
   *
   * @param sample_index - sample number out of num_visualization_rollouts
   * @param t - timestep out of num_timesteps
   * @param distribution_index - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param control - pointer to fill with the specific control array
   * @param theta_d - shared memory pointer for passing through params
   * @param block_size - parallelizable step size for the gpu (normally blockDim.y)
   * @param thread_index - parallelizable index for the gpu (normally threadIdx.y)
   * @param output - output pointer for compatibility with a output-based sampling distribution
   */
  __device__ void readVisControlSample(const int& sample_index, const int& t, const int& distribution_index,
                                       float* __restrict__ control, float* __restrict__ theta_d,
                                       const int& block_size = 1, const int& thread_index = 1,
                                       const float* __restrict__ output = nullptr);

  /**
   * @brief Update the distribution according to the weights of each sample. Should only be used if weights only exist
   * on the host side. Otherwise, use updateDistributionParamsFromDevice
   *
   * @param trajectory_weights - vector of size num_rollouts containing the weight of each sample
   * @param normalizer - the sum of all trajectory weights
   * @param distribution_i - which distribution to update
   * @param synchronize - whether or not to run cudaStreamSynchronize
   */
  __host__ void updateDistributionParamsFromHost(const Eigen::Ref<const Eigen::MatrixXf>& trajectory_weights,
                                                 float normalizer, const int& distribution_i, bool synchronize = false);

  /*************************************
   * Methods that need to be overwritten by derived classes
   *************************************/

  /**
   * @brief method for allocating additional CUDA memory in derived
   */
  __host__ void allocateCUDAMemoryHelper();

  __host__ __device__ float computeFeedbackCost(const float* __restrict__ u_fb, float* __restrict__ theta_d,
                                                const int t, const int distribution_idx, const float lambda = 1.0,
                                                const float alpha = 0.0);

  /**
   * @brief host method to calculate the cost for feedback based on the sampling distribution in RMPPI
   *
   * @param u_fb - feedback control
   * @param t - timestep
   * @param distribution_idx - distribution index
   * @param lambda - MPPI temperature parameter
   * @param alpha - coeff to turn off the likelihood cost (set to 1 -> no likelihood cost, set to 0 -> all likelihood
   *
   * @return feedback cost
   */
  __host__ float computeFeedbackCost(const Eigen::Ref<const control_array>& u_fb, const int t,
                                     const int distribution_idx, const float lambda = 1.0, const float alpha = 0.0);

  /**
   * @brief Device method to calculate the likelihood ratio cost for a given sample u
   *
   * @param u - sampled control
   * @param theta_d - shared memory for sampling distribution
   * @param t - timestep
   * @param distribution_idx - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param lambda - MPPI temperature parameter
   * @param alpha - coeff to turn off the likelihood cost (set to 1 -> no likelihood cost, set to 0 -> all likelihood
   * cost)
   */
  __host__ __device__ float computeLikelihoodRatioCost(const float* __restrict__ u, float* __restrict__ theta_d,
                                                       const int sample_index, const int t, const int distribution_idx,
                                                       const float lambda = 1.0, const float alpha = 0.0);

  /**
   * @brief Host-side method to calculate the likelihood ration cost for a given sample u
   *
   * @param u - sampled control
   * @param t - timestep
   * @param distribution_idx - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param lambda - MPPI temperature parameter
   * @param alpha - coeff to turn off the likelihood cost (set to 1 -> no likelihood cost, set to 0 -> all likelihood
   * cost)
   */
  __host__ float computeLikelihoodRatioCost(const Eigen::Ref<const control_array>& u, const int sample_index,
                                            const int t, const int distribution_idx, const float lambda = 1.0,
                                            const float alpha = 0.0);

  /**
   * @brief Get the latest importance sampler from time-shifting on the controller and update the device importance
   * sampler
   *
   * @param importance_sampler - host pointer to a control sequence that is NUM_TIMESTEPS * CONTROL_DIM
   * @param distribution_idx - which distribution is the importance sampler meant for
   * @param synchronize - whether or not to run cudaStreamSynchronize
   */
  __host__ void copyImportanceSamplerToDevice(const float* importance_sampler, const int& distribution_idx,
                                              bool synchronize = true);

  /**
   * @brief Generate control samples that will be on the GPU.
   *
   * @param optimization_stride - timestep to start control samples from
   * @param iteration_num - which iteration of the algorithm we are on. Useful for decaying std_dev
   * @param gen - pseudo-random noise generator
   */
  __host__ void generateSamples(const int& optimization_stride, const int& iteration_num, curandGenerator_t& gen,
                                bool synchronize = true);

  /**
   * @brief Set the Host-side Optimal Control Trajectory
   *
   * @param optimal_control_trajectory - pointer to CPU memory location to store the optimal control
   * @param distribution_idx - which distribution we are looking for the optimal control from (Useful for Tube and
   * RMPPI)
   * @param synchronize - whether or not to run cudaStreamSynchronize
   */
  __host__ void setHostOptimalControlSequence(float* optimal_control_trajectory, const int& distribution_idx,
                                              bool synchronize = true);

  /**
   * @brief takes in the cost of each sample generated and conducts an update of the distribution (For Gaussians, mean
   * update)
   *
   * @param trajectory_weights_d - vector of weights of size num_rollouts located on the GPU
   * @param normalizer - sum of all weights
   * @param distribution_i - which distribution to update
   * @param synchronize - whether or not to run cudaStreamSynchronize
   */
  __host__ virtual void updateDistributionParamsFromDevice(const float* trajectory_weights_d, float normalizer,
                                                           const int& distribution_i, bool synchronize = false) = 0;

  /**
   * @brief Write to a specific control sample located at [distribution_index][sample_index][t] from the
   * control array
   *
   * @param sample_index - sample number out of num_rollouts
   * @param t - timestep out of num_timesteps
   * @param distribution_index - distribution index (if it is larger than num_distributions, it just defaults to first
   * distribution for future compatibility with sampling dynamical systems)
   * @param control - pointer to control array with the desired data
   * @param theta_d - shared memory pointer for passing through params
   * @param block_size - parallelizable step size for the gpu (normally blockDim.y)
   * @param thread_index - parallelizable index for the gpu (normally threadIdx.y)
   * @param output - output pointer for compatibility with a output-based sampling distribution
   */
  __device__ void writeControlSample(const int& sample_index, const int& t, const int& distribution_index,
                                     const float* __restrict__ control, float* __restrict__ theta_d,
                                     const int& block_size = 1, const int& thread_index = 1,
                                     const float* __restrict__ output = nullptr);

  CLASS_T* sampling_d_ = nullptr;
  cudaStream_t vis_stream_ = nullptr;

protected:
  float* control_samples_d_ = nullptr;
  float* vis_control_samples_d_ = nullptr;

  SAMPLING_PARAMS_T params_;
};

#ifdef __CUDACC__
#include "sampling_distribution.cu"
#endif

template <class CLASS_T, template <int> class PARAMS_TEMPLATE, class DYN_PARAMS_T>
const int SamplingDistribution<CLASS_T, PARAMS_TEMPLATE, DYN_PARAMS_T>::CONTROL_DIM;

// template <int C_DIM>
// const int SamplingParams<C_DIM>::CONTROL_DIM;
}  // namespace sampling_distributions
}  // namespace mppi
