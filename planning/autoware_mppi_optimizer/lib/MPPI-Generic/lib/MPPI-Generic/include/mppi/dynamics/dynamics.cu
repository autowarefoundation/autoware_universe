#include <mppi/dynamics/dynamics.cuh>

template <class CLASS_T, class PARAMS_T>
void Dynamics<CLASS_T, PARAMS_T>::paramsToDevice(bool synchronize)
{
  if (GPUMemStatus_)
  {
    HANDLE_ERROR(cudaMemcpyAsync(&model_d_->params_, &params_, sizeof(PARAMS_T), cudaMemcpyHostToDevice, stream_));

    HANDLE_ERROR(cudaMemcpyAsync(&model_d_->control_rngs_, &control_rngs_, CONTROL_DIM * sizeof(float2),
                                 cudaMemcpyHostToDevice, stream_));
    if (synchronize)
    {
      HANDLE_ERROR(cudaStreamSynchronize(stream_));
    }
  }
}

template <class CLASS_T, class PARAMS_T>
void Dynamics<CLASS_T, PARAMS_T>::setDefaultControlRanges()
{
  for (int i = 0; i < CONTROL_DIM; i++)
  {
    control_rngs_[i].x = -FLT_MAX;
    control_rngs_[i].y = FLT_MAX;
  }
}

template <class CLASS_T, class PARAMS_T>
void Dynamics<CLASS_T, PARAMS_T>::setControlRanges(std::array<float2, CONTROL_DIM>& control_rngs, bool synchronize)
{
  for (int i = 0; i < CONTROL_DIM; i++)
  {
    control_rngs_[i].x = control_rngs[i].x;
    control_rngs_[i].y = control_rngs[i].y;
  }
  if (GPUMemStatus_)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->model_d_->control_rngs_, this->control_rngs_, CONTROL_DIM * sizeof(float2),
                                 cudaMemcpyHostToDevice, stream_));
    if (synchronize)
    {
      HANDLE_ERROR(cudaStreamSynchronize(stream_));
    }
  }
}

template <class CLASS_T, class PARAMS_T>
void Dynamics<CLASS_T, PARAMS_T>::setControlDeadbands(std::array<float, CONTROL_DIM>& control_deadband,
                                                      bool synchronize)
{
  for (int i = 0; i < CONTROL_DIM; i++)
  {
    control_deadband_[i] = control_deadband[i];
  }
  if (GPUMemStatus_)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->model_d_->control_deadband_, this->control_deadband_,
                                 CONTROL_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_));
    if (synchronize)
    {
      HANDLE_ERROR(cudaStreamSynchronize(stream_));
    }
  }
}

template <class CLASS_T, class PARAMS_T>
void Dynamics<CLASS_T, PARAMS_T>::GPUSetup()
{
  CLASS_T* derived = static_cast<CLASS_T*>(this);
  if (!GPUMemStatus_)
  {
    model_d_ = Managed::GPUSetup(derived);
  }
  else
  {
    this->logger_->debug("%s: GPU Memory already set\n", derived->getDynamicsModelName().c_str());
  }
  derived->paramsToDevice();
}

template <class CLASS_T, class PARAMS_T>
void Dynamics<CLASS_T, PARAMS_T>::freeCudaMem()
{
  if (GPUMemStatus_)
  {
    HANDLE_ERROR(cudaFree(model_d_));
    GPUMemStatus_ = false;
    model_d_ = nullptr;
  }
}

template <class CLASS_T, class PARAMS_T>
__device__ inline void Dynamics<CLASS_T, PARAMS_T>::computeStateDeriv(float* state, float* control, float* state_der,
                                                                      float* theta_s)
{
  CLASS_T* derived = static_cast<CLASS_T*>(this);
  // only propagate a single state, i.e. thread.y = 0
  // find the change in x,y,theta based off of the rest of the state
  if (threadIdx.y == 0)
  {
    derived->computeKinematics(state, state_der);
  }
  derived->computeDynamics(state, control, state_der, theta_s);
}

template <class CLASS_T, class PARAMS_T>
__device__ void Dynamics<CLASS_T, PARAMS_T>::enforceConstraints(float* state, float* control)
{
  // TODO should control_rngs_ be a constant memory parameter
  int i, p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  // parallelize setting the constraints with y dim
  for (i = p_index; i < CONTROL_DIM; i += step)
  {
    if (fabsf(control[i]) < this->control_deadband_[i])
    {
      control[i] = this->zero_control_[i];
    }
    else
    {
      control[i] += this->control_deadband_[i] * -mppi::math::sign(control[i]);
    }
    control[i] = fminf(fmaxf(this->control_rngs_[i].x, control[i]), this->control_rngs_[i].y);
  }
}

template <class CLASS_T, class PARAMS_T>
__device__ void Dynamics<CLASS_T, PARAMS_T>::updateState(float* state, float* next_state, float* state_der,
                                                         const float dt)
{
  int i, p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  // Add the state derivative time dt to the current state.
  for (i = p_index; i < STATE_DIM; i += step)
  {
    next_state[i] = state[i] + state_der[i] * dt;
  }
}

template <class CLASS_T, class PARAMS_T>
__device__ inline void Dynamics<CLASS_T, PARAMS_T>::step(float* state, float* next_state, float* state_der,
                                                         float* control, float* output, float* theta_s, const float t,
                                                         const float dt)
{
  CLASS_T* derived = static_cast<CLASS_T*>(this);
  derived->computeStateDeriv(state, control, state_der, theta_s);
  __syncthreads();
  derived->updateState(state, next_state, state_der, dt);
  __syncthreads();
  derived->stateToOutput(next_state, output);
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ inline void Dynamics<CLASS_T, PARAMS_T>::stateToOutput(const float* __restrict__ state,
                                                                           float* __restrict__ output)
{
  // TODO this is a hack
  int p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  for (int i = p_index; i < OUTPUT_DIM && i < STATE_DIM; i += step)
  {
    output[i] = state[i];
  }
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ inline void Dynamics<CLASS_T, PARAMS_T>::outputToState(const float* __restrict__ output,
                                                                           float* __restrict__ state)
{
  // TODO this is a hack
  int p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  for (int i = p_index; i < OUTPUT_DIM && i < STATE_DIM; i += step)
  {
    state[i] = output[i];
  }
}

template <class CLASS_T, class PARAMS_T>
Dynamics<CLASS_T, PARAMS_T>::state_array Dynamics<CLASS_T, PARAMS_T>::getZeroState() const
{
  return state_array::Zero();
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ inline void Dynamics<CLASS_T, PARAMS_T>::getZeroState(float* state) const
{
  int p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  for (int i = p_index; i < STATE_DIM; i += step)
  {
    state[i] = 0.0f;
  }
}
