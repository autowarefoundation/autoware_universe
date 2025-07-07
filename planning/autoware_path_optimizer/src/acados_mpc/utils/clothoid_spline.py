import numpy as np
from scipy.interpolate import CubicSpline


class ClothoidSpline:
    def __init__(self, s0, kapparef, num_points=None):
        self.s0 = s0
        self.kapparef = kapparef

        # Resample s0 and kapparef to have only a fixed number of points
        if num_points != None:
            s0_resampled = np.linspace(s0[0], s0[-1], num_points)
            kapparef_resampled = np.interp(s0_resampled, s0, kapparef)
            s0 = s0_resampled
            kapparef = kapparef_resampled

        self.spline = CubicSpline(s0, kapparef)
        self.knots = self.spline.x
        self.coefficients = self.spline.c

        self.pathlength = s0[-1]

    def get_spline(self):
        return self.spline

    def get_knots(self):
        return self.knots

    def get_coefficients(self):
        return self.coefficients

    def get_sub_spline_knots_and_coefficients_from_window_size(self, s, window_size):
        closest_knot = np.argmin(np.abs(self.knots - s))

        sub_knots = self.knots[closest_knot : min(closest_knot + window_size, len(self.knots))]
        sub_coefficients = self.coefficients[
            :, closest_knot : min(closest_knot + window_size - 1, np.shape(self.coefficients)[1])
        ]

        if len(sub_knots) < window_size:
            sub_knots = np.append(
                sub_knots,
                np.linspace(sub_knots[-1], sub_knots[-1] + 100.0, window_size - len(sub_knots)),
            )
            sub_coefficients = np.append(
                sub_coefficients,
                np.zeros((4, window_size - 1 - np.shape(sub_coefficients)[1])),
                axis=1,
            )

        return sub_knots, sub_coefficients
