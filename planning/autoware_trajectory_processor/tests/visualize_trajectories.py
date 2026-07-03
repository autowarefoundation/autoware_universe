#!/usr/bin/env python3
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

# cspell: ignore xycoords, boxstyle

"""
Visualization script for ContinuousJerkSmoother test trajectories.

Plots velocity profiles from exported CSV files.
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt


def load_trajectory(csv_path):
    """Load trajectory data from CSV file."""
    if not os.path.exists(csv_path):
        return None, None, None, None

    indices = []
    x = []
    velocity_kmh = []
    constraint_kmh = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            indices.append(int(row["index"]))
            x.append(float(row["x"]))
            velocity_kmh.append(float(row["velocity_kmh"]))
            constraint_kmh.append(float(row["constraint_kmh"]))

    return indices, x, velocity_kmh, constraint_kmh


def plot_test(test_name, input_csv, output_csv, title=None):
    """Plot input and output trajectories for a single test."""
    indices, x, vel_input, _ = load_trajectory(input_csv)
    _, _, vel_output, constraint = load_trajectory(output_csv)

    if vel_input is None or vel_output is None:
        print(f"Warning: Missing files for {test_name}")
        return

    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot input trajectory
    ax.plot(x, vel_input, "b-o", markersize=3, label="Input", alpha=0.7)

    # Plot output trajectory
    ax.plot(x, vel_output, "r-s", markersize=3, label="Output (Optimized)", linewidth=2)

    # Plot constraint if present
    if any(c > 0 for c in constraint):
        ax.plot(x, constraint, "g--", linewidth=2, label="Max Velocity Constraint")

    ax.set_xlabel("X Position (m)", fontsize=12)
    ax.set_ylabel("Velocity (km/h)", fontsize=12)
    ax.set_title(title or f"{test_name}: Velocity Profile", fontsize=14)
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    # Add velocity range annotation
    max_vel = max(vel_output)
    min_vel = min(vel_output)
    ax.annotate(
        f"Max: {max_vel:.1f} km/h\nMin: {min_vel:.1f} km/h",
        xy=(0.02, 0.98),
        xycoords="axes fraction",
        fontsize=10,
        verticalalignment="top",
        bbox={"boxstyle": "round", "facecolor": "wheat", "alpha": 0.5},
    )

    plt.tight_layout()
    return fig


def plot_all_tests(export_dir, output_dir=None):
    """Plot all test cases."""
    tests = [
        (
            "Test 1: No Constraint",
            "test1_no_constraint_input.csv",
            "test1_no_constraint_output.csv",
        ),
        (
            "Test 2: Max Velocity Cap (35 km/h)",
            "test2_max_vel_cap_input.csv",
            "test2_max_vel_cap_output.csv",
        ),
        (
            "Test 3: Deceleration to Stop",
            "test3_decel_to_stop_input.csv",
            "test3_decel_to_stop_output.csv",
        ),
        (
            "Test 4: Local Low Speed Constraint",
            "test4_local_low_speed_input.csv",
            "test4_local_low_speed_output.csv",
        ),
    ]

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    for test_name, input_file, output_file in tests:
        fig = plot_test(
            test_name, os.path.join(export_dir, input_file), os.path.join(export_dir, output_file)
        )

        if output_dir and fig:
            safe_name = test_name.split(":")[0].lower().replace(" ", "_")
            fig.savefig(os.path.join(output_dir, f"{safe_name}.png"), dpi=150)
            print(f"Saved: {output_dir}/{safe_name}.png")

    # Create combined overview plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    for idx, (test_name, input_file, output_file) in enumerate(tests):
        ax = axes[idx]

        _, x, vel_input, _ = load_trajectory(os.path.join(export_dir, input_file))
        _, _, vel_output, constraint = load_trajectory(os.path.join(export_dir, output_file))

        if vel_input is None:
            continue

        ax.plot(x, vel_input, "b-o", markersize=2, label="Input", alpha=0.7)
        ax.plot(x, vel_output, "r-s", markersize=2, label="Output", linewidth=1.5)

        if any(c > 0 for c in constraint):
            ax.plot(x, constraint, "g--", linewidth=1.5, label="Constraint")

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Velocity (km/h)")
        ax.set_title(test_name.split(":")[0])
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.suptitle("ContinuousJerkSmoother - All Test Cases", fontsize=14, fontweight="bold")
    plt.tight_layout()

    if output_dir:
        fig.savefig(os.path.join(output_dir, "all_tests_overview.png"), dpi=150)
        print(f"Saved: {output_dir}/all_tests_overview.png")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize trajectory optimizer test results")
    parser.add_argument(
        "--dir",
        "-d",
        default="/tmp/continuous_jerk_smoother",
        help="Directory containing CSV files",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output directory for PNG files (default: display only)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.dir):
        print(f"Error: Directory {args.dir} does not exist")
        print("Run the tests first to generate the CSV files:")
        print("  colcon test --packages-select autoware_trajectory_optimizer")
        return

    print(f"Loading trajectories from: {args.dir}")
    plot_all_tests(args.dir, args.output)


if __name__ == "__main__":
    main()
