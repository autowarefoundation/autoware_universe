# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import types

from casadi import SX
from casadi import atan
from casadi import cos
from casadi import sin
from casadi import tan
from casadi import vertcat


def bicycle_model_temporal():
    """Define a kinematic bicycle in time.

    States: x, y, psi, v (world pose and speed).
    Controls: a (longitudinal acceleration), delta (front steering angle [rad]).
    Steering is applied directly in the bicycle kinematics (no steering lag state).
    Parameters: lf, lr (wheelbase split).
    """
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "kinematic_bicycle_temporal"

    x_pos = SX.sym("x")
    y_pos = SX.sym("y")
    psi = SX.sym("psi")
    v = SX.sym("v")

    x = vertcat(x_pos, y_pos, psi, v)

    lf = SX.sym("lf")
    lr = SX.sym("lr")

    p = vertcat(lf, lr)

    a_long = SX.sym("a")
    delta = SX.sym("delta")
    u = vertcat(a_long, delta)

    xdot_sym = SX.sym("xdot")
    ydot_sym = SX.sym("ydot")
    psidot_sym = SX.sym("psidot")
    vdot_sym = SX.sym("vdot")
    xdot = vertcat(xdot_sym, ydot_sym, psidot_sym, vdot_sym)

    L = lf + lr
    beta_slip = atan(lr * tan(delta) / L)
    # Curvature-like term from yaw rate: psi_dot = v * K  =>  a_lat = v * psi_dot = v^2 * K
    K = cos(beta_slip) * tan(delta) / L
    a_lat = v * v * K

    x_dot = v * cos(psi + beta_slip)
    y_dot = v * sin(psi + beta_slip)
    psi_dot = v * cos(beta_slip) * tan(delta) / L
    v_dot = a_long

    f_expl = vertcat(x_dot, y_dot, psi_dot, v_dot)

    model.con_h_expr = vertcat(a_lat)

    model.a_min = -3.5
    model.a_max = 3.5
    model.delta_cmd_min = -0.7
    model.delta_cmd_max = 0.7
    model.delta_min = -0.7
    model.delta_max = 0.7

    params = types.SimpleNamespace()
    params.lf = lf
    params.lr = lr
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = model_name
    model.params = params
    return model, constraint
