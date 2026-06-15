/**
 * Created by david fan on 04/11/22.
 * Creates the API for interfacing with an MPPI controller
 * should define a compute_control based on state as well
 * as return timing info
 **/

#ifndef MPPIGENERIC_PRIMITIVES_CONTROLLER_CUH
#define MPPIGENERIC_PRIMITIVES_CONTROLLER_CUH

#include <mppi/controllers/controller.cuh>

// this is needed to extend the coloredMPPI params and ensure that
// dynamic reconfigure in mppi_generic_racer_plant.cpp works properly
#include <mppi/controllers/ColoredMPPI/colored_mppi_controller.cuh>

#include <vector>

template <int S_DIM, int C_DIM, int MAX_TIMESTEPS>
struct PrimitivesParams : public ColoredMPPIParams<S_DIM, C_DIM, MAX_TIMESTEPS>
{
  int num_primitive_iters_;
  int num_piecewise_segments_ = 5;
  std::vector<float> scale_piecewise_noise_;
  std::vector<float> frac_add_nominal_traj_;
  std::vector<float> scale_add_nominal_noise_;
  float stopping_cost_threshold_ = 1.0e8;
  float hysteresis_cost_threshold_ = 0.0;
  bool visualize_primitives_ = false;

  PrimitivesParams<S_DIM, C_DIM, MAX_TIMESTEPS>() = default;
  PrimitivesParams<S_DIM, C_DIM, MAX_TIMESTEPS>(const PrimitivesParams<S_DIM, C_DIM, MAX_TIMESTEPS>& other)
  {
    typedef ColoredMPPIParams<S_DIM, C_DIM, MAX_TIMESTEPS> BASE;
    const BASE& other_item_ref = other;
    *(static_cast<BASE*>(this)) = other_item_ref;
    this->scale_piecewise_noise_ = other.scale_piecewise_noise_;
    this->frac_add_nominal_traj_ = other.frac_add_nominal_traj_;
    this->scale_add_nominal_noise_ = other.scale_add_nominal_noise_;
  }

  PrimitivesParams<S_DIM, C_DIM, MAX_TIMESTEPS>(PrimitivesParams<S_DIM, C_DIM, MAX_TIMESTEPS>& other)
  {
    typedef ColoredMPPIParams<S_DIM, C_DIM, MAX_TIMESTEPS> BASE;
    BASE& other_item_ref = other;
    *(static_cast<BASE*>(this)) = other_item_ref;
    this->scale_piecewise_noise_ = other.scale_piecewise_noise_;
    this->frac_add_nominal_traj_ = other.frac_add_nominal_traj_;
    this->scale_add_nominal_noise_ = other.scale_add_nominal_noise_;
  }
};

template <class DYN_T, class COST_T, class FB_T, int MAX_TIMESTEPS, int NUM_ROLLOUTS, int BDIM_X, int BDIM_Y,
          int COST_B_X = 64, int COST_B_Y = 2,
          class PARAMS_T = PrimitivesParams<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM, MAX_TIMESTEPS>>
class PrimitivesController
  : public Controller<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, BDIM_X, BDIM_Y, PARAMS_T>
{
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // need control_array = ... so that we can initialize
  // Eigen::Matrix with Eigen::Matrix::Zero();
  typedef Controller<DYN_T, COST_T, FB_T, MAX_TIMESTEPS, NUM_ROLLOUTS, BDIM_X, BDIM_Y, PARAMS_T> PARENT_CLASS;
  using control_array = typename PARENT_CLASS::control_array;
  using control_trajectory = typename PARENT_CLASS::control_trajectory;
  using state_trajectory = typename PARENT_CLASS::state_trajectory;
  using state_array = typename PARENT_CLASS::state_array;
  using output_array = typename PARENT_CLASS::output_array;
  using sampled_cost_traj = typename PARENT_CLASS::sampled_cost_traj;
  using FEEDBACK_GPU = typename PARENT_CLASS::TEMPLATED_FEEDBACK_GPU;

  /**
   *
   * Public member functions
   */
  // Constructor
  PrimitivesController(DYN_T* model, COST_T* cost, FB_T* fb_controller, float dt, int max_iter, float lambda,
                       float alpha, const Eigen::Ref<const control_array>& control_std_dev,
                       int num_timesteps = MAX_TIMESTEPS,
                       const Eigen::Ref<const control_trajectory>& init_control_traj = control_trajectory::Zero(),
                       cudaStream_t stream = nullptr);

  PrimitivesController(DYN_T* model, COST_T* cost, FB_T* fb_controller, PARAMS_T& params,
                       cudaStream_t stream = nullptr);

  // Destructor
  ~PrimitivesController();

  std::string getControllerName()
  {
    return "Primitives";
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

  void setNumPrimitiveIterations(int new_num_iter)
  {
    this->params_.num_primitive_iters_ = new_num_iter;
  }

  int getNumPrimitiveIterations()
  {
    return this->params_.num_primitive_iters_;
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

  void setOffsetDecayRate(float offset_decay_rate)
  {
    this->params_.offset_decay_rate = offset_decay_rate;
  }

  float getOffsetDecayRate()
  {
    return this->params_.offset_decay_rate;
  }

  void setColoredNoiseExponents(std::vector<float>& new_exponents)
  {
    this->params_.colored_noise_exponents_ = new_exponents;
  }

  float getColoredNoiseExponent(int index) const
  {
    return this->params_.colored_noise_exponents_[index];
  }

  void setPiecewiseSegments(int segments)
  {
    this->params_.num_piecewise_segments_ = segments;
  }

  int getPiecewiseSegments()
  {
    return this->params_.num_piecewise_segments_;
  }

  void setScalePiecewiseNoise(std::vector<float>& new_scale)
  {
    this->params_.scale_piecewise_noise_ = new_scale;
  }

  std::vector<float> getScalePiecewiseNoise()
  {
    return this->params_.scale_piecewise_noise_;
  }

  void setFracRandomNoiseTraj(std::vector<float> frac_add_nominal_traj)
  {
    this->params_.frac_add_nominal_traj_ = frac_add_nominal_traj;
  }
  std::vector<float> getFracRandomNoiseTraj()
  {
    return this->params_.frac_add_nominal_traj_;
  }

  void setScaleAddNominalNoise(std::vector<float> scale_add_nominal_noise)
  {
    this->params_.scale_add_nominal_noise_ = scale_add_nominal_noise;
  }

  std::vector<float> getScaleAddNominalNoise()
  {
    return this->params_.scale_add_nominal_noise_;
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

  void setStoppingCostThreshold(float new_stopping_cost_threshold)
  {
    this->params_.stopping_cost_threshold_ = new_stopping_cost_threshold;
  }

  float getStoppingCostThreshold()
  {
    return this->params_.stopping_cost_threshold_;
  }

  void setHysteresisCostThreshold(float new_hysteresis_cost_threshold)
  {
    this->params_.hysteresis_cost_threshold_ = new_hysteresis_cost_threshold;
  }

  float getHysteresisCostThreshold()
  {
    return this->params_.hysteresis_cost_threshold_;
  }

  void setVisualizePrimitives(bool visualize_primitives)
  {
    this->params_.visualize_primitives_ = visualize_primitives;
  }

  bool getVisualizePrimitives()
  {
    return this->params_.visualize_primitives_;
  }

  void calculateSampledStateTrajectories() override;

protected:
  std::vector<float>& getScaleAddNominalNoiseLValue()
  {
    return this->params_.scale_add_nominal_noise_;
  }

  std::vector<float>& getScalePiecewiseNoiseLValue()
  {
    return this->params_.scale_piecewise_noise_;
  }

  std::vector<float>& getFracRandomNoiseTrajLValue()
  {
    return this->params_.frac_add_nominal_traj_;
  }

  std::vector<float>& getColoredNoiseExponentsLValue()
  {
    return this->params_.colored_noise_exponents_;
  }

  void computeStateTrajectory(const Eigen::Ref<const state_array>& x0);
  void copyMPPIControlToDevice(bool synchronize = true);

  void computeStoppingTrajectory(const Eigen::Ref<const state_array>& x0);
  void smoothControlTrajectory();

  int leash_jump_ = 1;
  bool leash_active_ = false;

  float* control_mppi_d_;                                         // Array of size DYN_T::CONTROL_DIM*NUM_TIMESTEPS
  control_trajectory control_mppi_ = control_trajectory::Zero();  // host side mppi control trajectory
  Eigen::Matrix<float, DYN_T::CONTROL_DIM, 2> control_mppi_history_;

private:
  // ======== MUST BE OVERWRITTEN =========
  void allocateCUDAMemory();
  // ======== END MUST BE OVERWRITTEN =====
};

#if __CUDACC__
#include "primitives_controller.cu"
#endif

#endif  // MPPIGENERIC_PRIMITIVES_CONTROLLER_CUH
