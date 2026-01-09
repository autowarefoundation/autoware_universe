"""
Generic symbolic cubic spline for optimization applications.
"""

import casadi as ca
import numpy as np


class SymbolicCubicSpline:
    """Simple symbolic cubic spline class."""

    def __init__(self, n_points: int, u: ca.SX.sym("u")):
        """Initialize spline with n_points."""
        self.n_points = n_points
        self.n_segments = n_points - 1

        self.knots = ca.SX.sym("knots", n_points)
        self.a = ca.SX.sym("a", n_points - 1)
        self.b = ca.SX.sym("b", n_points - 1)
        self.c = ca.SX.sym("c", n_points - 1)
        self.d = ca.SX.sym("d", n_points - 1)

        self.coefficients = ca.vertcat(self.a, self.b, self.c, self.d)

        self.x_val = 0
        self.dx_du = 0
        self.d2x_du2 = 0

        self.u = u

        # Create piecewise symbolic spline evaluation
        for i in range(self.n_points - 1):
            # Condition: knots[i] <= u < knots[i+1] (or u <= knots[i+1] for last segment)
            if i == 0:
                # For first segment, include u = knots[0]
                condition = ca.logic_and(self.u >= self.knots[i], self.u < self.knots[i + 1])
            elif i == self.n_segments - 1:
                # For last segment, include u = knots[-1]
                condition = ca.logic_and(self.u >= self.knots[i], self.u <= self.knots[i + 1])
            else:
                condition = ca.logic_and(self.u >= self.knots[i], self.u < self.knots[i + 1])

            # Local parameter within segment
            t = self.u - self.knots[i]

            # Cubic polynomial: f(t) = a*t^3 + b*t^2 + c*t + d
            # Note: scipy stores coefficients in reverse order [d, c, b, a]
            d, c, b, a = self.d[i], self.c[i], self.b[i], self.a[i]

            # Position
            x_seg = a * t**3 + b * t**2 + c * t + d

            # First derivatives: f'(t) = 3*a*t^2 + 2*b*t + c
            dx_seg = 3 * a * t**2 + 2 * b * t + c

            # Second derivatives: f''(t) = 6*a*t + 2*b
            d2x_seg = 6 * a * t + 2 * b

            # Use conditional assignment
            self.x_val = ca.if_else(condition, x_seg, self.x_val)
            self.dx_du = ca.if_else(condition, dx_seg, self.dx_du)
            self.d2x_du2 = ca.if_else(condition, d2x_seg, self.d2x_du2)

    def get_symbolic_spline(self):
        """Get the symbolic spline."""
        return self.x_val

    def get_symbolic_derivatives(self):
        """Get the symbolic derivatives."""
        return self.dx_du

    def get_symbolic_second_derivatives(self):
        """Get the symbolic second derivatives."""
        return self.d2x_du2

    def get_parameters(self):
        """Get the parameters of the spline."""
        return ca.vertcat(self.knots, ca.reshape(self.coefficients, -1, 1))
