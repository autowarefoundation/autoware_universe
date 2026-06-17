#include <mppi/cost_functions/quadrotor/quadrotor_quadratic_cost.cuh>

QuadrotorQuadraticCost::QuadrotorQuadraticCost(cudaStream_t stream)
{
  bindToStream(stream);
}

/**
 * Host Functions
 */
float QuadrotorQuadraticCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status)
{
  Eigen::Vector3f x, v, w;
  // Eigen::Vector4f q;

  Eigen::Map<const Eigen::Vector3f> x_g(this->params_.x_goal());
  Eigen::Map<const Eigen::Vector3f> v_g(this->params_.v_goal());
  Eigen::Map<const Eigen::Vector3f> w_g(this->params_.w_goal());

  x = s.block<3, 1>(E_INDEX(OutputIndex, POS_X), 0);
  v = s.block<3, 1>(E_INDEX(OutputIndex, VEL_X), 0);
  w = s.block<3, 1>(E_INDEX(OutputIndex, ANG_VEL_X), 0);

  // Quaternion/Angle Costs
  Eigen::Quaternionf q(s[6], s[7], s[8], s[9]);
  Eigen::Quaternionf q_g(this->params_.q_goal()[0], this->params_.q_goal()[1], this->params_.q_goal()[2],
                         this->params_.q_goal()[3]);
  Eigen::Quaternionf q_diff;
  mppi::math::QuatSubtract(q, q_g, q_diff);
  Eigen::VectorXf rot_diff;
  Eigen::ArrayXf rot_coeff;
  if (this->params_.use_euler)
  {
    float r_diff, p_diff, y_diff;
    mppi::math::Quat2EulerNWU(q_diff, r_diff, p_diff, y_diff);

    Eigen::Vector3f angle_diff;
    angle_diff << r_diff, p_diff, y_diff;
    Eigen::Array3f angle_coeff;
    angle_coeff << this->params_.roll_coeff, this->params_.pitch_coeff, this->params_.yaw_coeff;
    rot_diff = angle_diff;
    rot_coeff = angle_coeff;
  }
  else
  {
    Eigen::Vector4f q_vec;
    q_vec << q_diff.w(), q_diff.x(), q_diff.y(), q_diff.z();
    Eigen::Array4f q_coeff;
    q_coeff << this->params_.q_coeff, this->params_.q_coeff, this->params_.q_coeff, this->params_.q_coeff;
    rot_diff = q_vec;
    rot_coeff = q_coeff;
  }

  Eigen::Vector3f x_cost = this->params_.x_coeff * (x - x_g).array().square();
  Eigen::Vector3f v_cost = this->params_.v_coeff * (v - v_g).array().square();
  Eigen::Vector3f q_cost = rot_coeff * rot_diff.array().square();
  Eigen::Vector3f w_cost = this->params_.w_coeff * (w - w_g).array().square();

  return x_cost.sum() + v_cost.sum() + q_cost.sum() + w_cost.sum();
}

float QuadrotorQuadraticCost::terminalCost(const Eigen::Ref<const output_array> s)
{
  return this->params_.terminal_cost_coeff * computeStateCost(s);
}

/**
 * Device Functions
 */
__device__ float QuadrotorQuadraticCost::computeStateCost(float* s, int timestep, float* theta_c, int* crash_status)
{
  float s_diff[OUTPUT_DIM];
  int i;
  float sum = 0;

  for (i = 0; i < OUTPUT_DIM; i++)
  {
    s_diff[i] = powf(s[i] - this->params_.s_goal[i], 2);
  }
  float q_diff[4];
  mppi::math::QuatSubtract(s + 6, this->params_.s_goal + 6, q_diff);

  for (i = 0; i < 3; i++)
  {
    s_diff[i] *= this->params_.x_coeff;
  }

  for (i = 3; i < 6; i++)
  {
    s_diff[i] *= this->params_.v_coeff;
  }

  if (!this->params_.use_euler)
  {
    for (i = 6; i < 10; i++)
    {
      s_diff[i] = this->params_.q_coeff * q_diff[i - 6];
    }
  }
  if (this->params_.use_euler)
  {
    // Zero out quaternion portion for later
    for (i = 6; i < 10; i++)
    {
      s_diff[i] = 0;
    }
    // Get Euler Angles
    float r_diff, p_diff, y_diff;
    mppi::math::Quat2EulerNWU(q_diff, r_diff, p_diff, y_diff);
    sum += this->params_.roll_coeff * SQ(r_diff);
    sum += this->params_.pitch_coeff * SQ(p_diff);
    sum += this->params_.yaw_coeff * SQ(y_diff);
  }

  for (i = 10; i < 13; i++)
  {
    s_diff[i] *= this->params_.w_coeff;
  }

  for (i = 0; i < OUTPUT_DIM; i++)
  {
    sum += s_diff[i];
  }

  // do a final nan check
  return sum * (1 - isnan(sum)) + isnan(sum) * MAX_COST_VALUE;
}

__device__ float QuadrotorQuadraticCost::terminalCost(float* s, float* theta_c)
{
  float cost = this->params_.terminal_cost_coeff * computeStateCost(s);
  return cost * (1 - isnan(cost)) + isnan(cost) * MAX_COST_VALUE;
}
