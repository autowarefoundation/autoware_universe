/**
 * Created by jason on 10/30/19.
 * Creates the API for interfacing with an MPPI controller
 * should define a compute_control based on state as well
 * as return timing info
 **/

#ifndef MPPIGENERIC_COLORED_MPPI_CONTROLLER_CUH
#define MPPIGENERIC_COLORED_MPPI_CONTROLLER_CUH

#include <mppi/controllers/controller.cuh>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>

#include <vector>

template <int S_DIM, int C_DIM, int MAX_TIMESTEPS>
struct ColoredMPPIParams : public ControllerParams<S_DIM, C_DIM, MAX_TIMESTEPS>
{
  float r = 2.0;
  float gamma = 0;
  Eigen::Matrix<float, S_DIM, 1> state_leash_dist_ = Eigen::Matrix<float, S_DIM, 1>::Zero();

  ColoredMPPIParams() = default;
  ColoredMPPIParams(const ColoredMPPIParams<S_DIM, C_DIM, MAX_TIMESTEPS>& other)
  {
    typedef ControllerParams<S_DIM, C_DIM, MAX_TIMESTEPS> BASE;
    const BASE& other_item_ref = other;
    *(static_cast<BASE*>(this)) = other_item_ref;
    // this->colored_noise_exponents_ = other.colored_noise_exponents_;
  }

  ColoredMPPIParams(ColoredMPPIParams<S_DIM, C_DIM, MAX_TIMESTEPS>& other)
  {
    typedef ControllerParams<S_DIM, C_DIM, MAX_TIMESTEPS> BASE;
    BASE& other_item_ref = other;
    *(static_cast<BASE*>(this)) = other_item_ref;
    // this->colored_noise_exponents_ = other.colored_noise_exponents_;
  }
};

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS,
          class SAMPLING_T = ::mppi::sampling_distributions::ColoredNoiseDistribution<typename DYN_T::DYN_PARAMS_T>,
          class PARAMS_T = ColoredMPPIParams<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM, MAX_TIMESTEPS>>
class ColoredMPPIController : public Controller<DYN_T, COST_T, FB_T, SAMPLING_T, MAX_TIMESTEPS, NUM_ROLLOUTS, PARAMS_T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Controller<DYN_T, COST_T, FB_T, SAMPLING_T, MAX_TIMESTEPS, NUM_ROLLOUTS, PARAMS_T> PARENT_CLASS;
  // need control_array = ... so that we can initialize
  // Eigen::Matrix with Eigen::Matrix::Zero();
  using control_array = typename PARENT_CLASS::control_array;
  using control_trajectory = typename PARENT_CLASS::control_trajectory;
  using state_trajectory = typename PARENT_CLASS::state_trajectory;
  using state_array = typename PARENT_CLASS::state_array;
  using sampled_cost_traj = typename PARENT_CLASS::sampled_cost_traj;
  using FEEDBACK_GPU = typename PARENT_CLASS::TEMPLATED_FEEDBACK_GPU;

  /**
   *
   * Public member functions
   */
  // Constructor
  ColoredMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, float dt, int max_iter,
                        float lambda, float alpha, int num_timesteps = MAX_TIMESTEPS,
                        const Eigen::Ref<const control_trajectory>& init_control_traj = control_trajectory::Zero(),
                        cudaStream_t stream = nullptr);

  ColoredMPPIController(DYN_T* model, COST_T* cost, FB_T* fb_controller, SAMPLING_T* sampler, PARAMS_T& params,
                        cudaStream_t stream = nullptr);

  // Destructor
  ~ColoredMPPIController();

  std::string getControllerName() const override
  {
    return "Colored MPPI";
  };

  /**
   * computes a new control sequence
   * @param state starting position
   */
  void computeControl(const Eigen::Ref<const state_array>& state, int optimization_stride = 1) override;

  /**
   * Slide the control sequence back n steps
   */
  void slideControlSequence(int optimization_stride) override;

  void setPercentageSampledControlTrajectories(float new_perc)
  {
    this->setPercentageSampledControlTrajectoriesHelper(new_perc, 1);
  }

  void setGamma(float gamma)
  {
    this->params_.gamma = gamma;
  }

  float getGamma()
  {
    return this->params_.gamma;
  }

  void setRExp(float r)
  {
    this->params_.r = r;
  }

  float getRExp()
  {
    return this->params_.r;
  }

  void setOffsetDecayRate(float decay_rate)
  {
    this->sampler_->setOffsetDecayRate(decay_rate);
  }

  float getOffsetDecayRate()
  {
    return this->sampler_->getOffsetDecayRate();
  }

  void setColoredNoiseExponents(std::vector<float>& new_exponents)
  {
    auto sampler_params = this->sampler_->getParams();
    for (int i = 0; i < new_exponents.size(); i++)
    {
      sampler_params.exponents[i] = new_exponents[i];
    }
    this->sampler_->setParams(sampler_params);
  }

  float getColoredNoiseExponent(int index)
  {
    auto sampler_params = this->sampler_->getParams();
    return sampler_params.exponents[index];
  }

  std::vector<float> getColoredNoiseExponents()
  {
    std::vector<float> exponents;
    auto sampler_params = this->sampler_->getParams();
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      exponents.push_back(sampler_params.exponents[i]);
    }
    return exponents;
  }

  void setNoiseDecay(float new_noise_decay)
  {
    auto sampler_params = this->sampler_->getParams();
    sampler_params.std_dev_decay = new_noise_decay;
    this->sampler_->setParams(sampler_params);
  }

  void setStateLeashLength(float new_state_leash, int index = 0)
  {
    this->params_.state_leash_dist_[index] = new_state_leash;
  }

  float getStateLeashLength(int index)
  {
    return this->params_.state_leash_dist_[index];
  }

  bool getLeashActive()
  {
    return leash_active_;
  }

  void setLeashActive(bool new_leash_active)
  {
    leash_active_ = new_leash_active;
  }

  void calculateSampledStateTrajectories() override;

  void chooseAppropriateKernel() override;

protected:
  std::vector<float>& getColoredNoiseExponentsLValue()
  {
    return this->params_.colored_noise_exponents_;
  }

  void computeStateTrajectory(const Eigen::Ref<const state_array>& x0);

  void smoothControlTrajectory();
  int leash_jump_ = 1;
  bool leash_active_ = false;
  float control_std_dev_decay_ = 1.0;

private:
  // ======== MUST BE OVERWRITTEN =========
  void allocateCUDAMemory();
  // ======== END MUST BE OVERWRITTEN =====
};

#if __CUDACC__
#include "colored_mppi_controller.cu"
#endif

#endif  // MPPIGENERIC_COLORED_MPPI_CONTROLLER_CUH
