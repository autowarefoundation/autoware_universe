import types

from casadi import SX
from casadi import atan
from casadi import cos
from casadi import tan
from casadi import vertcat
from utils.symbolic_cubic_spline import SymbolicCubicSpline


def bicycle_model_spatial_with_body_points(n_points: int, n_circles: int = 0):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "curvilinear_bicycle_model_spatial"

    # CasADi Model
    # set up states & controls
    eY = SX.sym("eY")
    eψ = SX.sym("eψ")

    x = vertcat(
        eY,
        eψ,
    )

    s_sym = SX.sym("s")  # symbolic independent variable
    kappa_ref_s_symbolic_curvature_cubic_spline = SymbolicCubicSpline(n_points=n_points, u=s_sym)
    kappa_ref_s = kappa_ref_s_symbolic_curvature_cubic_spline.get_symbolic_spline()
    lf = SX.sym("lf")
    lr = SX.sym("lr")

    print("sym shape: ", s_sym.shape)
    print("kappa_ref_s shape: ", kappa_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)

    p = vertcat(
        s_sym,
        kappa_ref_s_symbolic_curvature_cubic_spline.get_parameters(),
    )
    p = vertcat(p, lf, lr)

    # Parameters for MPT-style rotated footprint constraints (one per circle)
    # We pass cos(beta) and sin(beta) (beta is yaw difference at lon offset) and lon_offset.
    if n_circles > 0:
        cos_beta = SX.sym("cos_beta", n_circles)
        sin_beta = SX.sym("sin_beta", n_circles)
        lon_offset = SX.sym("lon_offset", n_circles)
        p = vertcat(p, cos_beta, sin_beta, lon_offset)
    else:
        cos_beta = None
        sin_beta = None
        lon_offset = None

    # controls
    delta = SX.sym("delta")
    u = vertcat(delta)

    # xdot
    eYdot = SX.sym("eYdot")
    eψdot = SX.sym("eψdot")

    xdot = vertcat(
        eYdot,
        eψdot,
    )

    beta = atan(lr * tan(delta) / (lf + lr))
    kappa = cos(beta) * tan(delta) / (lf + lr)

    # dynamics
    deY_ds = tan(eψ + beta) * (1 - kappa_ref_s * eY)
    deψ_ds = kappa * (1 - kappa_ref_s * eY) / cos(eψ) - kappa_ref_s

    f_expl = vertcat(
        deY_ds,
        deψ_ds,
    )

    # MPT-style hard inequality per circle:
    # In MPT (OSQP) hard constraints are implemented as:
    #   lb_i - C_vec <= C_mat * x <= ub_i - C_vec, where C_vec = lon_i*sin(beta_i)
    # which is equivalent to:
    #   lb_i <= C_mat * x + C_vec <= ub_i
    #
    # Therefore our h(x,p) should be:
    #   h_i = cos(beta_i)*(eY + lon_i*eψ) + lon_i*sin(beta_i)
    # and bounds are simply lh=lb_i, uh=ub_i.
    # Bounds (lb/ub) are provided per-stage via lh/uh.
    if n_circles > 0:
        h_expr = []
        for i in range(n_circles):
            h_expr.append(cos_beta[i] * (eY + lon_offset[i] * eψ) + lon_offset[i] * sin_beta[i])
        model.con_h_expr = vertcat(*h_expr)

    # Model bounds
    model.eY_min = -1.5  # width of the track [m]
    model.eY_max = 1.5  # width of the track [m]
    # NOTE: Don't over-bound yaw error (eψ) with a tight box constraint; rely on cost + input bounds.

    # input bounds
    model.delta_min = -0.7  # minimum steering angle [rad]
    model.delta_max = 0.7  # maximum steering angle [rad]

    # Define model struct
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
