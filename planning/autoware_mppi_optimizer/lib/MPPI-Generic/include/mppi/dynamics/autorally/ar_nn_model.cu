
#include "ar_nn_model.cuh"

template <int S_DIM, int C_DIM, int K_DIM>
NeuralNetModel<S_DIM, C_DIM, K_DIM>::NeuralNetModel(std::array<float2, C_DIM> control_rngs, cudaStream_t stream)
  : PARENT_CLASS(control_rngs, stream)
{
  std::vector<int> args{ 6, 32, 32, 4 };
  helper_ = new FNNHelper<>(args, stream);
}

template <int S_DIM, int C_DIM, int K_DIM>
NeuralNetModel<S_DIM, C_DIM, K_DIM>::NeuralNetModel(cudaStream_t stream) : PARENT_CLASS(stream)
{
  std::vector<int> args{ 6, 32, 32, 4 };
  helper_ = new FNNHelper<>(args, stream);
}

template <int S_DIM, int C_DIM, int K_DIM>
NeuralNetModel<S_DIM, C_DIM, K_DIM>::~NeuralNetModel()
{
  freeCudaMem();
  free(helper_);
}

template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    helper_->freeCudaMem();
  }
  PARENT_CLASS::freeCudaMem();
}

template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::updateModel(const std::vector<int>& description,
                                                      const std::vector<float>& data)
{
  helper_->updateModel(description, data);
}

template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::paramsToDevice()
{
  if (this->GPUMemStatus_)
  {
    helper_->paramsToDevice();
  }
  PARENT_CLASS::paramsToDevice();
}

template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::loadParams(const std::string& model_path)
{
  helper_->loadParams(model_path);
}

template <int S_DIM, int C_DIM, int K_DIM>
bool NeuralNetModel<S_DIM, C_DIM, K_DIM>::computeGrad(const Eigen::Ref<const state_array>& state,
                                                      const Eigen::Ref<const control_array>& control,
                                                      Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B)
{
  Eigen::Matrix<float, S_DIM, S_DIM + C_DIM> jac;
  jac.setZero();

  // Start with the kinematic and physics model derivatives
  jac.row(0) << 0, 0, -sin(state(2)) * state(4) - cos(state(2)) * state(5), 0, cos(state(2)), -sin(state(2)), 0, 0, 0;
  jac.row(1) << 0, 0, cos(state(2)) * state(4) - sin(state(2)) * state(5), 0, sin(state(2)), cos(state(2)), 0, 0, 0;
  jac.row(2) << 0, 0, 0, 0, 0, 0, -1, 0, 0;

  state_array state_der;

  // First do the forward pass
  computeDynamics(state, control, state_der);

  Eigen::MatrixXf ip_delta = helper_->getZeroJacobianMatrix();
  helper_->computeGrad(ip_delta);

  jac.bottomRightCorner(DYNAMICS_DIM, DYNAMICS_DIM + C_DIM) += ip_delta;
  A = jac.leftCols(S_DIM);
  B = jac.rightCols(C_DIM);
  return true;
}

template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::computeKinematics(const Eigen::Ref<const state_array>& state,
                                                            Eigen::Ref<state_array> state_der)
{
  state_der(0) = cosf(state(2)) * state(4) - sinf(state(2)) * state(5);
  state_der(1) = sinf(state(2)) * state(4) + cosf(state(2)) * state(5);
  state_der(2) = -state(6);  // Pose estimate actually gives the negative yaw derivative
}

template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::computeDynamics(const Eigen::Ref<const state_array>& state,
                                                          const Eigen::Ref<const control_array>& control,
                                                          Eigen::Ref<state_array> state_der)
{
  Eigen::VectorXf input = helper_->getZeroInputVector();
  Eigen::VectorXf output = helper_->getZeroOutputVector();
  int i;
  for (i = 0; i < DYNAMICS_DIM; i++)
  {
    input(i) = state(i + (S_DIM - DYNAMICS_DIM));
  }
  for (i = 0; i < C_DIM; i++)
  {
    input(DYNAMICS_DIM + i) = control(i);
  }
  helper_->forward(input, output);
  for (i = 0; i < DYNAMICS_DIM; i++)
  {
    state_der(i + (S_DIM - DYNAMICS_DIM)) = output[i];
  }
}

template <int S_DIM, int C_DIM, int K_DIM>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM>::computeKinematics(float* state, float* state_der)
{
  state_der[0] = cosf(state[2]) * state[4] - sinf(state[2]) * state[5];
  state_der[1] = sinf(state[2]) * state[4] + cosf(state[2]) * state[5];
  state_der[2] = -state[6];  // Pose estimate actually gives the negative yaw derivative
}

template <int S_DIM, int C_DIM, int K_DIM>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM>::computeDynamics(float* state, float* control, float* state_der,
                                                                     float* theta_s)
{
  float* curr_act;
  int tdy = threadIdx.y;
  int i;
  // curr_act = &theta_s[SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float) + 1 + (2 * LARGEST_LAYER) * (blockDim.x * tdz +
  // tdx)];
  curr_act = helper_->getInputLocation(theta_s);
  // iterate through the part of the state that should be an input to the NN
  for (i = tdy; i < DYNAMICS_DIM; i += blockDim.y)
  {
    curr_act[i] = state[i + (S_DIM - DYNAMICS_DIM)];
  }
  // iterate through the control to put into first layer
  for (i = tdy; i < C_DIM; i += blockDim.y)
  {
    curr_act[DYNAMICS_DIM + i] = control[i];
  }
  __syncthreads();

  curr_act = helper_->forward(nullptr, theta_s);

  // copies results back into state derivative
  for (i = tdy; i < DYNAMICS_DIM; i += blockDim.y)
  {
    state_der[i + (S_DIM - DYNAMICS_DIM)] = curr_act[i];
  }
  __syncthreads();
}
template <int S_DIM, int C_DIM, int K_DIM>
void NeuralNetModel<S_DIM, C_DIM, K_DIM>::GPUSetup()
{
  PARENT_CLASS* derived = static_cast<PARENT_CLASS*>(this);
  helper_->GPUSetup();
  derived->GPUSetup();

  HANDLE_ERROR(cudaMemcpyAsync(&(this->model_d_->helper_), &(this->helper_->network_d_), sizeof(FNNHelper<>*),
                               cudaMemcpyHostToDevice, this->stream_));
}

template <int S_DIM, int C_DIM, int K_DIM>
__device__ void NeuralNetModel<S_DIM, C_DIM, K_DIM>::initializeDynamics(float* state, float* control, float* output,
                                                                        float* theta_s, float t_0, float dt)
{
  if (output)
  {
    for (int i = 0; i < this->OUTPUT_DIM && i < this->STATE_DIM; i++)
    {
      output[i] = state[i];
    }
  }
  helper_->initialize(theta_s);
}

template <int S_DIM, int C_DIM, int K_DIM>
NeuralNetModel<S_DIM, C_DIM, K_DIM>::state_array
NeuralNetModel<S_DIM, C_DIM, K_DIM>::stateFromMap(const std::map<std::string, float>& map)
{
  state_array s;
  s(S_INDEX(POS_X)) = map.at("POS_X");
  s(S_INDEX(POS_Y)) = map.at("POS_Y");
  s(S_INDEX(YAW)) = map.at("YAW");
  s(S_INDEX(ROLL)) = map.at("ROLL");
  s(S_INDEX(BODY_VEL_X)) = map.at("VEL_X");
  s(S_INDEX(BODY_VEL_Y)) = map.at("VEL_Y");
  s(S_INDEX(YAW_RATE)) = map.at("OMEGA_Z");
  return s;
}

template <int S_DIM, int C_DIM, int K_DIM>
__host__ __device__ int NeuralNetModel<S_DIM, C_DIM, K_DIM>::getGrdSharedSizeBytes() const
{
  return helper_->getGrdSharedSizeBytes();
}

template <int S_DIM, int C_DIM, int K_DIM>
__host__ __device__ int NeuralNetModel<S_DIM, C_DIM, K_DIM>::getBlkSharedSizeBytes() const
{
  return helper_->getBlkSharedSizeBytes();
}
