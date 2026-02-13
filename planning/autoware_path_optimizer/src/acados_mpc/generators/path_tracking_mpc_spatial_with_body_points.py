# author: Arjun Jagdish Ram
from acados_template import AcadosModel
from acados_template import AcadosOcp
from acados_template import AcadosOcpSolver
from generators.bicycle_model_spatial_with_body_points import bicycle_model_spatial_with_body_points
import numpy as np


class PathTrackingMPCSpatialWithBodyPoints:
    def __init__(self, Tf, N, n_points, n_circles=0, build=True, generate=True):
        self.Tf = Tf
        self.N = N
        self.n_points = n_points
        self.n_circles = n_circles

        self.constraint, self.model, self.acados_solver = self.acados_settings(build, generate)

    def acados_settings(self, build=True, generate=True):
        # create render arguments
        ocp = AcadosOcp()

        # export model
        model, constraint = bicycle_model_spatial_with_body_points(self.n_points, self.n_circles)

        # define acados ODE
        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.f_impl_expr
        model_ac.f_expl_expr = model.f_expl_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.p = model.p
        if hasattr(model, "con_h_expr"):
            model_ac.con_h_expr = model.con_h_expr
        model_ac.name = model.name
        ocp.model = model_ac

        # Set solver options to skip heavy CasADi simplifications that might hang
        ocp.code_export_directory = "c_generated_code"

        # dimensions
        nx = model.x.rows()
        nu = model.u.rows()
        ny = nx + nu
        ny_e = nx

        # discretization
        ocp.solver_options.N_horizon = self.N

        # set cost
        Q = np.diag([1e-2, 1e-1])

        R = np.eye(nu)
        R[0, 0] = 2e-1

        Qe = 5 * Q

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        unscale = self.N / self.Tf

        ocp.cost.W = unscale * np.block(
            [[Q, np.zeros((Q.shape[0], R.shape[1]))], [np.zeros((R.shape[0], Q.shape[1])), R]]
        )
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
        ocp.cost.yref = np.array([0.0, 0.0, 0.0])
        ocp.cost.yref_e = np.array([0.0, 0.0])

        # setting constraints
        # No global box constraints on states; only fix x0 at stage 0.

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

        # MPT-style rotated footprint hard constraints (one per circle): lh <= h(x,p) <= uh
        if self.n_circles > 0:
            ocp.constraints.lh = np.zeros(self.n_circles)
            ocp.constraints.uh = np.zeros(self.n_circles)
            # Soft constraints via slack variables on h:
            # lh - s_l <= h(x,p) <= uh + s_u, with s_l >= 0, s_u >= 0
            # and cost: zl^T s_l + zu^T s_u (quadratic terms set to 0 here).
            ocp.constraints.idxsh = np.arange(self.n_circles)
            ocp.constraints.lsh = np.zeros(self.n_circles)
            ocp.constraints.ush = np.zeros(self.n_circles)
            ocp.cost.zl = np.ones(self.n_circles)
            ocp.cost.zu = np.ones(self.n_circles)
            # NOTE: acados_template expects Zl/Zu as 1D arrays (diagonal entries).
            ocp.cost.Zl = np.zeros(self.n_circles)
            ocp.cost.Zu = np.zeros(self.n_circles)
        ocp.parameter_values = np.zeros(model.p.shape[0])

        # set QP solver and integration
        ocp.solver_options.tf = self.Tf
        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.num_steps = 1
        ocp.solver_options.nlp_solver_max_iter = 20
        ocp.solver_options.tol = 1e-4

        # create solver
        if not build:
            # If we only want to generate code, use the static method to avoid loading the library
            AcadosOcpSolver.generate(ocp, json_file="acados_ocp.json")
            return constraint, model, None

        acados_solver = AcadosOcpSolver(
            ocp, json_file="acados_ocp.json", build=build, generate=generate
        )

        return constraint, model, acados_solver


def main():

    N = 100
    Sf = 100
    n_circles = 6

    # Set build=False and generate=True to only generate the C code
    # without trying to compile the solver inside the Python process.
    _ = PathTrackingMPCSpatialWithBodyPoints(
        Sf, N, N, n_circles=n_circles, build=False, generate=True
    )


if __name__ == "__main__":
    main()
