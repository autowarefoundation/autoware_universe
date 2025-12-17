# author: Arjun Jagdish Ram
import os
import sys
import time

import numpy as np
import scipy.linalg

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from acados_template import AcadosModel
from acados_template import AcadosOcp
from acados_template import AcadosOcpSolver
from bicycle_model_spatial_with_body_points import bicycle_model_spatial_with_body_points
from utils.clothoid_spline import ClothoidSpline


class PathTrackingMPCSpatialWithBodyPoints:
    def __init__(self, Tf, N, n_points, num_body_points, build=True, generate=True):
        self.Tf = Tf
        self.N = N
        self.n_points = n_points
        self.num_body_points = num_body_points

        self.constraint, self.model, self.acados_solver = self.acados_settings(build, generate)

    def acados_settings(self, build=True, generate=True):
        # create render arguments
        ocp = AcadosOcp()

        # export model
        model, constraint = bicycle_model_spatial_with_body_points(
            self.n_points, self.num_body_points
        )

        # define acados ODE
        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.f_impl_expr
        model_ac.f_expl_expr = model.f_expl_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.p = model.p
        model_ac.name = model.name
        ocp.model = model_ac

        # dimensions
        nx = model.x.rows()
        nu = model.u.rows()
        ny = nx + nu
        ny_e = nx

        # discretization
        ocp.solver_options.N_horizon = self.N

        # set cost
        Q = np.diag([1e-2, 1e-1] + [0.0] * self.num_body_points + [0.0] * self.num_body_points)

        R = np.eye(nu)
        R[0, 0] = 2e-1

        Qe = 5 * Q

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        unscale = self.N / self.Tf

        ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Qe / unscale

        Vx = np.zeros((ny, nx))
        Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        Vu = np.zeros((ny, nu))
        Vu[ny - 1, 0] = 1.0
        ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        # set initial references
        ocp.cost.yref = np.array(
            [
                0,
                0,
            ]
            + [0.0] * self.num_body_points
            + [0.0] * self.num_body_points
            + [0.0]
        )
        ocp.cost.yref_e = np.array(
            [0, 0] + [0.0] * self.num_body_points + [0.0] * self.num_body_points
        )

        # setting constraints
        idxbx_eY = [0]  # main eY
        idxbx_eY += list(range(2 + self.num_body_points, 2 + 2 * self.num_body_points))
        idxbx_ePsi = [1]

        ocp.constraints.idxbx = np.array(np.concatenate((idxbx_eY, idxbx_ePsi)))

        ocp.constraints.lbx = np.array(
            np.concatenate(([model.eY_min] * len(idxbx_eY), [model.ePsi_min] * len(idxbx_ePsi)))
        )
        ocp.constraints.ubx = np.array(
            np.concatenate(([model.eY_max] * len(idxbx_eY), [model.ePsi_max] * len(idxbx_ePsi)))
        )

        ocp.constraints.lbu = np.array(
            [
                model.delta_min,
            ]
        )
        ocp.constraints.ubu = np.array(
            [
                model.delta_max,
            ]
        )
        ocp.constraints.idxbu = np.array([0])

        # set initial condition
        ocp.constraints.x0 = np.zeros(nx)
        ocp.parameter_values = np.zeros(model.p.shape[0])

        # set QP solver and integration
        ocp.solver_options.tf = self.Tf
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.num_steps = 1
        ocp.solver_options.nlp_solver_max_iter = 20
        ocp.solver_options.tol = 1e-4

        # create solver
        print("Creating acados solver...")
        acados_solver = AcadosOcpSolver(
            ocp, json_file="acados_ocp.json", build=build, generate=generate
        )

        return constraint, model, acados_solver

    def get_optimised_steering(
        self,
        x0,
        body_points_array,
        x_ref_sub_knots,
        x_ref_sub_coefficients,
        y_ref_sub_knots,
        y_ref_sub_coefficients,
        clothoid_spline: ClothoidSpline,
        lf,
        lr,
    ):
        # load model
        Sf = clothoid_spline.pathlength
        constraint, model, acados_solver = self.constraint, self.model, self.acados_solver

        # dimensions
        nx = model.x.rows()
        nu = model.u.rows()
        ny = nx + nu

        # initialize data structs
        simX = np.zeros((self.N, nx))
        simU = np.zeros((self.N, nu))
        s0 = 0
        tcomp_sum = 0
        tcomp_max = 0

        acados_solver.set(0, "lbx", x0)
        acados_solver.set(0, "ubx", x0)

        # Extract sub-spline for the current position
        clothoid_sub_knots, clothoid_sub_coefficients = (
            clothoid_spline.get_sub_spline_knots_and_coefficients_from_window_size(
                s0, self.n_points
            )
        )

        sref = clothoid_spline.pathlength
        for j in range(self.N):
            # Ensure all arrays are properly shaped before concatenation
            s_interp = np.array([s0 + (sref - s0) * j / self.N])  # Convert scalar to 1D array
            parameters = np.concatenate(
                (
                    s_interp,
                    x_ref_sub_knots,
                    x_ref_sub_coefficients.flatten(),
                    y_ref_sub_knots,
                    y_ref_sub_coefficients.flatten(),
                    clothoid_sub_knots,
                    clothoid_sub_coefficients.flatten(),
                    body_points_array,
                ),
                axis=0,
            )
            parameters = np.concatenate((parameters, [lf, lr]), axis=0)

            print("s_interp: ", len(s_interp))
            print("x_ref_sub_knots: ", len(x_ref_sub_knots))
            print("x_ref_sub: ", len(x_ref_sub_coefficients.flatten()))
            print("y_ref_sub: ", len(y_ref_sub_coefficients.flatten()))
            print(
                "clothoid_sub: ",
                len(np.concatenate((clothoid_sub_knots, clothoid_sub_coefficients.flatten()))),
            )
            print("body_points_array: ", len(body_points_array))

            yref = np.array(
                [0, 0] + [0.0] * self.num_body_points + [0.0] * self.num_body_points + [0.0]
            )
            acados_solver.set(j, "yref", yref)
            acados_solver.set(j, "p", parameters)
        yref_N = np.array([0, 0] + [0.0] * self.num_body_points + [0.0] * self.num_body_points)
        acados_solver.set(self.N, "yref", yref_N)

        # solve ocp
        t = time.time()

        status = acados_solver.solve()
        if status != 0:
            print("acados returned status {} in closed loop iteration.".format(status))

        elapsed = time.time() - t

        # manage timings
        tcomp_sum += elapsed
        if elapsed > tcomp_max:
            tcomp_max = elapsed

        # update initial condition
        x0 = acados_solver.get(1, "x")
        acados_solver.set(0, "lbx", x0)
        acados_solver.set(0, "ubx", x0)

        for idx in range(0, self.N):
            x = acados_solver.get(idx, "x")
            u = acados_solver.get(idx, "u")

            simX[idx, :] = x
            simU[idx, :] = u

        final_idx = int(clothoid_spline.pathlength / (Sf / self.N))

        simX = simX[:final_idx, :]
        simU = simU[:final_idx, :]

        return simX, simU, Sf, elapsed


def main():

    N = 100
    Sf = 100
    num_body_points = 6

    _ = PathTrackingMPCSpatialWithBodyPoints(Sf, N, N, num_body_points)


if __name__ == "__main__":
    main()
