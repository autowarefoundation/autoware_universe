from casadi import *
from utils.symbolic_cubic_spline import SymbolicCubicSpline


def bicycle_model_spatial_with_body_points(n_points, num_body_points):
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "curvilinear_bicycle_model_spatial"

    ## CasADi Model
    # set up states & controls
    eY = SX.sym("eY")
    eψ = SX.sym("eψ")

    s_body_points = SX.sym("s_body_points", num_body_points)
    eY_body_points = SX.sym("eY_body_points", num_body_points)

    x = vertcat(
        eY,
        eψ,
        s_body_points,
        eY_body_points,
    )

    x_body_points = SX.sym("x_body_points", num_body_points)
    y_body_points = SX.sym("y_body_points", num_body_points)

    s_sym = SX.sym("s")  # symbolic independent variable
    x_ref_s_symbolic_curvature_cubic_spline = SymbolicCubicSpline(n_points=n_points, u=s_sym)
    x_ref_s = x_ref_s_symbolic_curvature_cubic_spline.get_symbolic_spline()
    y_ref_s_symbolic_curvature_cubic_spline = SymbolicCubicSpline(n_points=n_points, u=s_sym)
    y_ref_s = y_ref_s_symbolic_curvature_cubic_spline.get_symbolic_spline()
    kappa_ref_s_symbolic_curvature_cubic_spline = SymbolicCubicSpline(n_points=n_points, u=s_sym)
    kappa_ref_s = kappa_ref_s_symbolic_curvature_cubic_spline.get_symbolic_spline()
    lf = SX.sym("lf")
    lr = SX.sym("lr")
    L = lf + lr

    print("sym shape: ", s_sym.shape)
    print("x_ref_s shape: ", x_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)
    print("y_ref_s shape: ", y_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)
    print("kappa_ref_s shape: ", kappa_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)
    print("x_body_points shape: ", x_body_points.shape)
    print("y_body_points shape: ", y_body_points.shape)

    p = vertcat(
        s_sym,
        x_ref_s_symbolic_curvature_cubic_spline.get_parameters(),
        y_ref_s_symbolic_curvature_cubic_spline.get_parameters(),
        kappa_ref_s_symbolic_curvature_cubic_spline.get_parameters(),
    )
    # p = vertcat(p, x_body_points, y_body_points)
    p = vertcat(p, lf, lr)

    # import pdb; pdb.set_trace()
    print("n_points: ", n_points)
    print("len s_sym: ", s_sym)
    print("len x_ref_s: ", x_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)
    print("len y_ref_s: ", y_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)
    print("len kappa_ref_s: ", kappa_ref_s_symbolic_curvature_cubic_spline.get_parameters().shape)
    print("len x_body_points: ", x_body_points.shape)
    print("len y_body_points: ", y_body_points.shape)

    # controls
    delta = SX.sym("delta")
    u = vertcat(delta)

    # xdot
    eYdot = SX.sym("eYdot")
    eψdot = SX.sym("eψdot")

    sdot_body_points = SX.sym("sdot_body_points", num_body_points)
    eYdot_body_points = SX.sym("eYdot_body_points", num_body_points)

    xdot = vertcat(
        eYdot,
        eψdot,
        sdot_body_points,
        eYdot_body_points,
    )

    beta = atan(lr * tan(delta) / (lf + lr))
    kappa = cos(beta) * tan(delta) / (lf + lr)
    psi_ref_s = atan2(jacobian(y_ref_s, s_sym), jacobian(x_ref_s, s_sym))

    # dynamics
    deY_ds = tan(eψ + beta) * (1 - kappa_ref_s * eY)
    deψ_ds = kappa * (1 - kappa_ref_s * eY) / cos(eψ) - kappa_ref_s

    ds_body_points_ds = []
    deY_body_points_ds = []

    for i in range(num_body_points):
        # s_i = s_body_points[i]
        # eY_i = eY_body_points[i]

        # x_centre = x_ref_s + eY * (-sin(psi_ref_s))
        # y_centre = y_ref_s + eY * (cos(psi_ref_s))
        # psi = psi_ref_s + eψ

        # x_body = x_body_points[i]
        # y_body = y_body_points[i]

        # dx = x_body - x_centre
        # dy = y_body - y_centre

        # evaluate curvature spline at body point s position
        # x_ref_body_s_i = substitute(x_ref_s, s_sym, s_i)
        # y_ref_body_s_i = substitute(y_ref_s, s_sym, s_i)
        # psi_ref_body_s_i = substitute(psi_ref_s, s_sym, s_i)
        # kappa_ref_s_i = substitute(kappa_ref_s, s_sym, s_i)

        # dynamics for body point s position
        ds_i_ds = 1  # -(kappa*(dx*sin(psi_ref_s-psi_ref_body_s_i) - dy*cos(psi_ref_s-psi_ref_body_s_i)) + cos(beta + psi-psi_ref_body_s_i))*(kappa_ref_s*eY - 1)/((kappa_ref_s_i*eY_i - 1)*cos(beta + eψ))
        ds_body_points_ds.append(ds_i_ds)
        # dynamics for body point eY position
        deY_i_ds = deY_ds  # (kappa*(dx*cos(psi_ref_s-psi_ref_body_s_i) + dy*sin(psi_ref_s-psi_ref_body_s_i)) + sin(beta + psi-psi_ref_body_s_i))*(1-kappa_ref_s*eY)/cos(beta + eψ)
        deY_body_points_ds.append(deY_i_ds)

    f_expl = vertcat(
        deY_ds,
        deψ_ds,
        *ds_body_points_ds,
        *deY_body_points_ds,
    )

    # Model bounds
    model.eY_min = -1.5 # width of the track [m]
    model.eY_max =  1.5  # width of the track [m]
    model.ePsi_min = -0.1 # maximum steering angle [rad]
    model.ePsi_max = 0.1  # maximum steering angle [rad]

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
