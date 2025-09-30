# Autoware Trajectory Optimizer

The `autoware_trajectory_optimizer` package is responsible for generating smooth and feasible trajectories for autonomous vehicles. It takes in a series of waypoints and outputs a continuous trajectory that the vehicle can follow. The interpolation methods for the path include the elastic band smoother and the Akima spline. Additionally, velocity smoothing can be achieved using assets from the `autoware_velocity_smoother` package.

## Features

- Interpolates waypoints to generate smooth trajectories.
- Ensures continuity and feasibility of the generated trajectories.
- Configurable parameters to adjust the interpolation behavior.

## Dependencies

This package depends on the following packages:

- `autoware_velocity_smoother`: Ensures that the velocity profile of the trajectory is smooth and feasible.
- `autoware_path_smoother`: Smooths the path to ensure that the trajectory is continuous and drivable.

## Parameters

{{ json_to_markdown("planning/autoware_trajectory_optimizer/schema/trajectory_optimizer.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.
