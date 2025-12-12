# ============================================================
# curvilinear_dynamics_symbolic.py
# Curvilinear dynamics with kinematic bicycle (slip β)
# Correct definitions displayed, symbolic derivatives, working lambdify
# ============================================================

import sympy as sp

# =============================================================
# 0. Key symbolic definitions (display only)
# =============================================================
x, y, psi = sp.symbols("x y psi", real=True)
s = sp.symbols("s", real=True)

x_r = sp.Function("x_r")(s)
y_r = sp.Function("y_r")(s)
psi_r = sp.Function("psi_r")(s)
beta, delta, L = sp.symbols("beta delta L", real=True)

# Curvilinear deviations (for display only)
eY_expr = -(x - x_r) * sp.sin(psi_r) + (y - y_r) * sp.cos(psi_r)
ePsi_expr = psi - psi_r

K_expr = sp.cos(beta) * sp.tan(delta) / L  # slip-modified curvature

print("=== Key symbolic definitions (display only) ===")
print("eY   = ")
sp.pprint(eY_expr)
print("ePsi = ")
sp.pprint(ePsi_expr)
print("K    = ")
sp.pprint(K_expr)
print("==============================================\n")

# =============================================================
# 1. Symbolic variables for computation
# =============================================================
v, l_f, l_r, K_ref = sp.symbols("v l_f l_r K_ref", real=True)
eY, ePsi = sp.symbols("eY ePsi", real=True)  # symbolic deviations
K = sp.symbols("K")  # symbolic curvature

# =============================================================
# 2. Kinematic bicycle velocities using ePsi + psi_r
# =============================================================
psi_subbed = ePsi + psi_r  # replace psi - psi_r with ePsi
xdot = v * sp.cos(psi_subbed + beta)
ydot = v * sp.sin(psi_subbed + beta)
psidot = v * K

# =============================================================
# 3. General geometric relations
# =============================================================
t_r = sp.Matrix([sp.cos(psi_r), sp.sin(psi_r)])  # tangent
n_r = sp.Matrix([-sp.sin(psi_r), sp.cos(psi_r)])  # normal
v_global = sp.Matrix([xdot, ydot])

# =============================================================
# 4. Time derivatives in curvilinear coordinates
# =============================================================
s_dot = sp.simplify((v_global.dot(t_r)) / (1 - K_ref * eY))
eY_dot = sp.simplify(v_global.dot(n_r))
ePsi_dot = sp.simplify(psidot - K_ref * s_dot)

# =============================================================
# 5. Spatial derivatives d()/ds
# =============================================================
eY_prime = sp.simplify(eY_dot / s_dot)
ePsi_prime = K * (1 - K_ref * eY) / sp.cos(ePsi) - K_ref  # factorized for clarity

# =============================================================
# 6. Display derivatives
# =============================================================
print("=== TIME DERIVATIVES (slip β) ===")
print("ṡ =")
sp.pprint(s_dot)
print("\nėY =")
sp.pprint(eY_dot)
print("\nePsi̇ =")
sp.pprint(ePsi_dot)

print("\n=== SPATIAL DERIVATIVES (d/ds) ===")
print("eY' =")
sp.pprint(eY_prime)
print("\nePsi' =")
sp.pprint(ePsi_prime)

# =============================================================
# 7. Lambdify for numerical evaluation (symbolic eY, ePsi)
# =============================================================
curvilinear_dynamics_time = sp.lambdify(
    (v, K, eY, ePsi, s, K_ref, psi_r), (s_dot, eY_dot, ePsi_dot), "numpy"
)

curvilinear_dynamics_spatial = sp.lambdify(
    (v, K, eY, ePsi, s, K_ref, psi_r), (eY_prime, ePsi_prime), "numpy"
)

# =============================================================
# 8. Arbitrary body points in curvilinear coordinates
# =============================================================
N_points = 1  # number of body points (e.g., corners)
s_i = sp.symbols(f"s_i0:{N_points}", real=True)
eY_i = sp.symbols(f"eY_i0:{N_points}", real=True)
K_ref_i = sp.symbols(f"K_ref_i0:{N_points}", real=True)  # reference curvature at each point

# =============================================================
# 9. Body points time derivatives (fully derived)
# =============================================================
s_dot_points = []
eY_dot_points = []

for k in range(N_points):
    x_r_body = [sp.Function(f"x_r_body_{k}")(s_i[k]) for k in range(N_points)]
    y_r_body = [sp.Function(f"y_r_body_{k}")(s_i[k]) for k in range(N_points)]
    psi_r_body = [sp.Function(f"psi_r_body_{k}")(s_i[k]) for k in range(N_points)]

    # Offsets relative to body point
    dX = sp.symbols("dX", real=True)
    dY = sp.symbols("dY", real=True)
    # Velocity in global frame
    xdot_body = v * sp.cos(psi + beta) + psidot * (-dY * sp.cos(psi) - dX * sp.sin(psi))
    ydot_body = v * sp.sin(psi + beta) + psidot * (-dY * sp.sin(psi) + dX * sp.cos(psi))

    # Curvilinear projection along the body-point tangent
    s_dot_i = (xdot_body * sp.cos(psi_r_body[k]) + ydot_body * sp.sin(psi_r_body[k])) / (
        1 - K_ref_i[k] * eY_i[k]
    )
    # s_dot_i = -(kappa*(dx*sin(psi_ref_s-psi_ref_body_s_i) - dy*cos(psi_ref_s-psi_ref_body_s_i)) + cos(beta + psi-psi_ref_body_s_i))*(kappa_ref_s*eY - 1)/((kappa_ref_s_i*eY_i - 1)*cos(beta + eψ))
    eY_dot_i = -xdot_body * sp.sin(psi_r_body[k]) + ydot_body * sp.cos(psi_r_body[k])
    # eY_dot_i = (kappa*(dx*cos(psi_ref_s-psi_ref_body_s_i) + dy*sin(psi_ref_s-psi_ref_body_s_i)) + sin(beta + psi-psi_ref_body_s_i))*(1-kappa_ref_s*eY)/cos(beta + eψ)

    # Simplify expressions
    s_dot_i = sp.simplify(s_dot_i)
    eY_dot_i = sp.simplify(eY_dot_i)

    s_dot_points.append(s_dot_i)
    eY_dot_points.append(eY_dot_i)

# Spatial derivatives wrt vehicle center s
s_prime_points = [sp.simplify(s_dot_points[k] / s_dot) for k in range(N_points)]
eY_prime_points = [sp.simplify(eY_dot_points[k] / s_dot) for k in range(N_points)]

s_prime_points = [
    sp.collect(
        s_prime_points[k],
        [K, sp.sin(ePsi + psi_r), sp.cos(ePsi + psi_r), sp.sin(ePsi), sp.cos(ePsi)],
    )
    for k in range(N_points)
]
eY_prime_points = [
    sp.collect(
        eY_prime_points[k],
        [K, sp.sin(ePsi + psi_r), sp.cos(ePsi + psi_r), sp.sin(ePsi), sp.cos(ePsi)],
    )
    for k in range(N_points)
]

s_prime_points = [sp.trigsimp(sp.simplify(s_prime_points[k])) for k in range(N_points)]
eY_prime_points = [sp.trigsimp(sp.simplify(eY_prime_points[k])) for k in range(N_points)]

# =============================================================
# 10. Display body point derivatives
# =============================================================
print("\n=== BODY POINTS TIME DERIVATIVES ===")
for k in range(N_points):
    print(f"Point {k}:")
    print("\tṡ_i = ")
    sp.pprint(s_dot_points[k])
    print("\teẎ_i = ")
    sp.pprint(eY_dot_points[k])

print("\n=== BODY POINTS SPATIAL DERIVATIVES wrt vehicle s ===")
for k in range(N_points):
    print(f"Point {k}:")
    print("\ts_i' = ")
    print(s_prime_points[k])
    print("\teY_i' = ")
    print(eY_prime_points[k])

# =============================================================
# 11. Lambdify for numerical evaluation (body points)
# =============================================================
curvilinear_points_time = [
    sp.lambdify((v, ePsi, eY_i[k], K_ref_i[k]), (s_dot_points[k], eY_dot_points[k]), "numpy")
    for k in range(N_points)
]

curvilinear_points_spatial = [
    sp.lambdify((v, ePsi, eY_i[k], K_ref_i[k]), (s_prime_points[k], eY_prime_points[k]), "numpy")
    for k in range(N_points)
]
