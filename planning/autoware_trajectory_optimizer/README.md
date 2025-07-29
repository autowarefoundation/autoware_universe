# Autoware Trajectory Optimizer

The `autoware_trajectory_optimizer` package is responsible for generating smooth and feasible trajectories for autonomous vehicles. It takes in a series of waypoints and outputs a continuous trajectory that the vehicle can follow. The interpolation methods for the path include the elastic band smoother and the Akima spline. Additionally, velocity smoothing can be achieved using assets from the `autoware_velocity_smoother` package.

The `autoware_trajectory_optimizer` package is responsible for generating smooth and feasible trajectories for autonomous vehicles. It takes in a series of waypoints and outputs a continuous trajectory that the vehicle can follow.

## Features

- Interpolates waypoints to generate smooth trajectories.
- Ensures continuity and feasibility of the generated trajectories.
- Configurable parameters to adjust the interpolation behavior.

## Dependencies

This package depends on the following packages:

- `autoware_velocity_smoother`: Ensures that the velocity profile of the trajectory is smooth and feasible.
- `autoware_path_smoother`: Smooths the path to ensure that the trajectory is continuous and drivable.

## Configuration

The behavior of the `autoware_trajectory_optimizer` can be configured using the parameters defined in the `config` directory. Some of the key parameters include:

- `keep_last_trajectory_s`: How long to maintain the previous chosen/best trajectory as a single output for this module. Only used if `keep_last_trajectory` is true.
- `nearest_dist_threshold_m`: constraint used to search for the nearest trajectory pose to the ego vehicle.
- `nearest_yaw_threshold_rad`: constraint used to search for the nearest trajectory pose to the ego vehicle.
- `target_pull_out_speed_mps`: to assure the ego can start moving from a stopped position, this parameter sets a minimum trajectory speed value.
- `target_pull_out_acc_mps2`:to assure the ego can start moving from a stopped position, this parameter sets a minimum trajectory acceleration value.
- `max_speed_mps`: The maximum allowable velocity for the trajectory.
- `spline_interpolation_resolution_m`: Interpolation resolution for Akima spline.
- `backward_trajectory_extension_m`: How long should the ego trajectory extend backward. This backward trajectory is built using the ego's previous poses.
- `use_akima_spline_interpolation`: To use akima spline interpolation to smooth the trajectories.
- `smooth_trajectories`: Flag to indicate if the Elastic Band smoother should be applied on the input trajectories.
- `limit_speed`: Flag to indicate if a `max_speed_mps` speed limit should be applied to the trajectories.
- `fix_invalid_points`: If the module should remove repeated or invalid points, or points that go against the general trajectory direction.
- `smooth_velocities`: Apply velocity smoothing to the input trajectories.
- `publish_last_trajectory`: Publish the previous trajectory selected by the `autoware_trajectory_ranker` package along with the interpolated new trajectories coming from the trajectory generator.
- `keep_last_trajectory`: with this flag on, the module will only publish the previous trajectory selected by the `autoware_trajectory_ranker` for `keep_last_trajectory_s` seconds.
- `extend_trajectory_backward`: flag used to indicate if the ego's trajectory should be extended backward.

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
