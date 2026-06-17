#include <gtest/gtest.h>
#include <mppi_test/mock_classes/mock_dynamics.h>
#include <mppi/feedback_controllers/feedback.cuh>

template <int STATE_DIM = 1, int CONTROL_DIM = 1, int OUTPUT_DIM = 1>
struct DynamicsTesterParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    NUM_STATES = STATE_DIM
  };
  enum class ControlIndex : int
  {
    NUM_CONTROLS = CONTROL_DIM
  };
  enum class OutputIndex : int
  {
    NUM_OUTPUTS = OUTPUT_DIM
  };
  int var_1 = 1;
  int var_2 = 2;
  float4 var_4;
};

struct FeedbackParams
{
  int var_10 = 10;
  int var_20 = 20;
  float var_30 = 3.14;
};

template <int STATE_DIM = 1, int CONTROL_DIM = 1>
class DynamicsTester
  : public MPPI_internal::Dynamics<DynamicsTester<STATE_DIM, CONTROL_DIM>, DynamicsTesterParams<STATE_DIM, CONTROL_DIM>>
{
public:
  using PARENT_CLASS =
      MPPI_internal::Dynamics<DynamicsTester<STATE_DIM, CONTROL_DIM>, DynamicsTesterParams<STATE_DIM, CONTROL_DIM>>;
  using state_array = typename PARENT_CLASS::state_array;
  using control_array = typename PARENT_CLASS::control_array;

  DynamicsTester(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }

  DynamicsTester(std::array<float2, CONTROL_DIM> control_rngs, cudaStream_t stream = 0)
    : PARENT_CLASS(control_rngs, stream)
  {
  }

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der)
  {
    state_der(1) = control(0);
  }

  void computeKinematics(const Eigen::Ref<const state_array>& state, Eigen::Ref<state_array> s_der)
  {
    s_der(0) = state(0) + state(1);
  };

  // TODO must be properly parallelized
  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta_s = nullptr)
  {
    state_der[1] = control[0];
  }

  // TODO must be properly parallelized
  __device__ void computeKinematics(float* state, float* state_der)
  {
    state_der[0] = state[0] + state[1];
  }
};

struct TestGPUState : GPUState
{
  int testing = 5;
};

class TestGPUFeedbackController
  : public GPUFeedbackController<TestGPUFeedbackController, DynamicsTester<>, TestGPUState>
{
public:
  typedef MockDynamics DYN_T;
  typedef GPUFeedbackController<TestGPUFeedbackController, DynamicsTester<>, TestGPUState> PARENT_CLASS;

  TestGPUFeedbackController(cudaStream_t stream = 0) : PARENT_CLASS(stream)
  {
  }

  void allocateCudaMemory()
  {
  }

  void deallocateCUDAMemory()
  {
  }

  void copyToDevice()
  {
  }

  void copyFromDevice()
  {
  }
};

class TestFeedbackController : public FeedbackController<TestGPUFeedbackController, FeedbackParams, 10>
{
public:
  typedef FeedbackController<TestGPUFeedbackController, FeedbackParams, 10> PARENT_CLASS;
  using INTERNAL_STATE_T = typename PARENT_CLASS::TEMPLATED_FEEDBACK_STATE;

  TestFeedbackController(float dt = 0.01, int num_timesteps = 10, cudaStream_t stream = 0)
    : PARENT_CLASS(dt, num_timesteps, stream)
  {
  }

  control_array k_(const Eigen::Ref<const state_array>& x_act, const Eigen::Ref<const state_array>& x_goal, int t,
                   INTERNAL_STATE_T& fb_state) override
  {
  }

  // might not be a needed method
  void computeFeedback(const Eigen::Ref<const state_array>& init_state,
                       const Eigen::Ref<const state_trajectory>& goal_traj,
                       const Eigen::Ref<const control_trajectory>& control_traj) override
  {
  }
  void initTrackingController() override
  {
  }
};

TEST(FeedbackController, Constructor)
{
  cudaStream_t stream;

  HANDLE_ERROR(cudaStreamCreate(&stream));

  TestFeedbackController feedbackController(0.01, 10, stream);
  feedbackController.GPUSetup();

  EXPECT_EQ(feedbackController.getHostPointer()->stream_, stream) << "Stream binding failure.";
  EXPECT_EQ(feedbackController.getHostPointer()->GPUMemStatus_, true) << "GPU not set up";
  EXPECT_NE(feedbackController.getDevicePointer(), nullptr) << "GPU is not in device pointer";

  HANDLE_ERROR(cudaStreamDestroy(stream));
}

TEST(FeedbackController, test)
{
}
