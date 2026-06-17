//
// Created by mgandhi3 on 7/16/21.
//

#ifndef MPPIGENERIC_NUMERICAL_INTEGRATION_H
#define MPPIGENERIC_NUMERICAL_INTEGRATION_H

#include <Eigen/Dense>

template <class DYN>
void rk4integrate(DYN* dynamics, float dt, const Eigen::Ref<typename DYN::state_array>& x_k,
                  const Eigen::Ref<typename DYN::control_array>& u_k, Eigen::Ref<typename DYN::state_array> x_kp1)
{
  // Assume a zero order hold on the control
  typename DYN::state_array k1, k2, k3, k4, x_next1, x_next2, x_next3, x_next4;
  typename DYN::output_array output;
  dynamics->step(x_k, x_next1, k1, u_k, output, 0, dt / 2);
  dynamics->step(x_next1, x_next2, k2, u_k, output, 0, dt / 2);
  dynamics->step(x_next2, x_next3, k3, u_k, output, 0, dt);
  dynamics->step(x_next3, x_next4, k4, u_k, output, 0, dt / 2);
  // dynamics->computeStateDeriv(x_k, u_k, k1);
  // dynamics->computeStateDeriv(x_k + k1 * dt / 2, u_k, k2);
  // dynamics->computeStateDeriv(x_k + k2 * dt / 2, u_k, k3);
  // dynamics->computeStateDeriv(x_k + k3 * dt, u_k, k4);
  x_kp1 = x_k + (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6;
}

#endif  // MPPIGENERIC_NUMERICAL_INTEGRATION_H
