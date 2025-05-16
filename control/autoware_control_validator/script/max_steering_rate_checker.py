# Copyright 2025 TIER IV, Inc.
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

import argparse

import matplotlib.pyplot as plt
import numpy as np


def calculate_max_steering_rate(velocity, wheel_base, max_lateral_jerk):
    """Calculate maximum allowable steering rate based on comfort criteria (max lateral jerk).

    Uses simplified lateral jerk formula with approximations: tan(δ) ≈ δ and (1 + tan²(δ)) ≈ 1

    Formula: dδ/dt = (j_y · L) / V²

    Args:
        velocity: Longitudinal velocity [m/s]
        wheel_base: Vehicle wheelbase [m]
        max_lateral_jerk: Maximum allowable lateral jerk [m/s³]

    Returns:
        Maximum allowable steering rate [rad/s]
    """
    # Ensure velocity is at least 0.1 to avoid division by zero
    safe_velocity = np.maximum(velocity, 0.1)

    # Calculate max steering rate (simplified formula)
    max_steering_rate = (max_lateral_jerk * wheel_base) / (safe_velocity**2)

    return max_steering_rate


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Calculate maximum allowable steering rate based on comfort criteria."
    )
    parser.add_argument(
        "--wheel_base",
        type=float,
        default=2.74,
        help="Wheel base [m]",
    )
    parser.add_argument(
        "--max_velocity",
        type=float,
        default=35.0,
        help="Maximum velocity [m/s]",
    )
    parser.add_argument(
        "--max_lateral_jerk",
        type=float,
        default=5.0,
        help="Maximum allowable lateral jerk [m/s³]",
    )

    # Parse arguments
    args = parser.parse_args()

    # Extract parameters
    wheel_base = args.wheel_base
    max_velocity = args.max_velocity
    max_lateral_jerk = args.max_lateral_jerk

    # Create velocity range for curve
    velocities = np.linspace(0.1, max_velocity, 100)

    # Calculate max steering rates
    steer_rates = calculate_max_steering_rate(velocities, wheel_base, max_lateral_jerk)
    steer_rates_deg = np.degrees(steer_rates)

    # Generate sample velocities automatically (6 points evenly spaced)
    num_samples = 6
    sample_velocities = np.linspace(max(1.0, max_velocity * 0.1), max_velocity * 0.9, num_samples)
    sample_velocities = np.round(
        sample_velocities, 1
    )  # Round to 1 decimal point for cleaner display

    # Calculate rates for sample points
    sample_rates = [
        np.degrees(calculate_max_steering_rate(v, wheel_base, max_lateral_jerk))
        for v in sample_velocities
    ]

    # Create the plot
    plt.figure(figsize=(10, 6))

    # Plot the curve
    plt.plot(velocities, steer_rates_deg, "b-", linewidth=2, label="Max Steering Rate")

    # Add sample points
    plt.plot(sample_velocities, sample_rates, "ro", markersize=8, label="Reference Points")

    # Add labels for sample points - adjust vertical position based on value
    for v, r in zip(sample_velocities, sample_rates):
        # Adjust text vertical position to avoid overlap
        offset = 1.5 if r < 20 else -2.0  # Place text above or below point based on value
        plt.text(v, r + offset, f"{r:.1f} deg/s", ha="center")

    # Configure plot
    plt.title(
        f"Maximum Allowable Steering Rate vs. Velocity\nWheel Base: {wheel_base}m, Max Lateral Jerk: {max_lateral_jerk} m/s³"
    )
    plt.xlabel("Velocity [m/s]")
    plt.ylabel("Max Steering Rate [deg/s]")
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.xlim(0, max_velocity)

    max_rate = np.max(steer_rates_deg)
    plt.ylim(
        0, min(max_rate * 1.2, 250)
    )  # Set upper limit to 20% above max value or 250, whichever is smaller

    plt.legend(loc="upper right")

    # Adjust layout to leave room for the formula text
    plt.subplots_adjust(bottom=0.15)

    # Add formula text
    # cSpell:ignore figtext boxstyle
    plt.figtext(
        0.5,
        0.02,
        "Formula: dδ/dt = (j_y · L) / V² (where j_y = 5.0 m/s³)",
        ha="center",
        fontsize=12,
        bbox={"facecolor": "white", "alpha": 0.8, "boxstyle": "round"},
    )

    # Print reference table
    print("\nMax Steering Rate Reference Table:")
    print("-" * 50)
    print("Velocity (m/s) | Max Steering Rate (deg/s)")
    print("-" * 50)

    for v, r in zip(sample_velocities, sample_rates):
        print(f"{v:13.1f} | {r:22.2f}")

    # Show plot in a popup window
    plt.show()


if __name__ == "__main__":
    main()
