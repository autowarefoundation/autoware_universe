# Deviation Estimator

## Overview

The **Deviation Estimator** estimates the standard deviation and bias for velocity and yaw bias, by comparing the velocity and gyro observations with ground truth poses (e.g. from LiDAR-based localization).

Here are some assumptions made for input data:

- The data should have accurate localization results. It doesn't need to be strictly precise, but data that is obviously incorrect should be avoided. In case of NDT in Autoware, it should not have any TP or NVTL warnings from the ndt_scan_matcher.
- The data should cover a reasonable duration of driving. A few minutes of data is sufficient. It is desirable to have a distance of approximately 500 meters. (For example, around 2 minutes at a speed of 15 km/h).
- The data should include sections of driving in a straight line. This is necessary for estimating velocity-related parameters. Having at least one minute of straight line driving is enough.
- The data should cover the expected speed range during operation.
- [Optional] Ideally, the data should be recorded as recently as possible. Especially in cases where IMU or tire replacement has occurred, data recorded before those changes may not provide accurate estimations.
- [Optional] The data should cover various driving behaviors expected during operation, such as right turns, left turns, acceleration, and deceleration.

## Launch

The `autoware_deviation_estimator` can be launched with the following command.

```sh
ros2 launch autoware_deviation_estimator deviation_estimator.launch.xml
ros2 bag play YOUR_BAG # You can also play in a faster rate, e.g. -r 5
```

The parameters and input topic names can be seen in the `deviation_estimator.launch.xml` file.
`YOUR_BAG` should include all the required inputs written below.

## Inputs / Outputs

### Input

| Name                     | Type                                            | Description          |
| ------------------------ | ----------------------------------------------- | -------------------- |
| `in_pose_with_covariance | `geometry_msgs::msg::PoseWithCovarianceStamped` | Input pose           |
| `in_imu`                 | `sensor_msgs::msg::Imu`                         | Input IMU data       |
| `in_wheel_odometry`      | `autoware_vehicle_msgs::msg::VelocityReport`    | Input wheel odometry |

### Output

| Name                                 | Type                        | Description                                      |
| ------------------------------------ | --------------------------- | ------------------------------------------------ |
| `/estimated_stddev_vx`               | `std_msgs::msg::Float64`    | estimated standard deviation of vx               |
| `/estimated_stddev_angular_velocity` | `geometry_msgs/msg/Vector3` | estimated standard deviation of angular velocity |
| `/estimated_coef_vx`                 | `std_msgs::msg::Float64`    | coef of vx                                       |
| `/estimated_bias_angular_velocity`   | `geometry_msgs/msg/Vector3` | bias of angular velocity                         |

## Parameters

| Name                                           | Type   | Description                                                         | Default value |
| ---------------------------------------------- | ------ | ------------------------------------------------------------------- | ------------- |
| show_debug_info                                | bool   | Flag to display debug info                                          | true          |
| t_design                                       | double | Maximum expected duration of dead-reckoning [s]                     | 10.0          |
| x_design                                       | double | Maximum expected trajectory length of dead-reckoning [m]            | 30.0          |
| time_window                                    | double | Estimation period [s]                                               | 4.0           |
| results_dir                                    | string | Text path where the estimated results will be stored                | "$(env HOME)" |
| gyro_estimation.only_use_straight              | bool   | Flag to use only straight sections for gyro estimation              | true          |
| gyro_estimation.only_use_moving                | bool   | Flag to use only moving sections for gyro estimation                | true          |
| gyro_estimation.only_use_constant_velocity     | bool   | Flag to use only constant velocity sections for gyro estimation     | true          |
| velocity_estimation.only_use_straight          | bool   | Flag to use only straight sections for velocity estimation          | true          |
| velocity_estimation.only_use_moving            | bool   | Flag to use only moving sections for velocity estimation            | true          |
| velocity_estimation.only_use_constant_velocity | bool   | Flag to use only constant velocity sections for velocity estimation | true          |

## Functions

### Bias estimation

By assuming that the pose information is a ground truth, the node estimates the bias of velocity and yaw rate.

### Standard deviation estimation

The node also estimates the standard deviation of velocity and yaw rate. This can be used as a parameter in `ekf_localizer`.
Note that the final estimation takes into account the bias.

## Usage

Here you estimate the following parameters using `autoware_deviation_estimator`.

- the standard deviation of velocity and yaw rate
- the bias of velocity and yaw rate

Launch the node with the following command. Make sure you set the correct parameters.

```sh
ros2 launch autoware_deviation_estimator deviation_estimator.launch.xml
```

Then, you need to run either ROS bag or `autoware_launch` to provide `pose` and `twist` to `autoware_deviation_estimator`.

If you are using rosbag, it should contain the following topics:

- Raw IMU data (default: `/sensing/imu/tamagawa/imu_raw`)
- Raw velocity data (default: `/vehicle/status/velocity_status`)
- `/localization/pose_estimator/pose_with_covariance`
- `/clock`
- `/tf_static` (that contains transform from `base_link` to `imu_link`)

NOTE that the pose and twist must be estimated with default parameters.

Play the rosbag in a different terminal:

```sh
ros2 bag play YOUR_BAG # You can also play in a faster rate, e.g. -r 5
```

You can check the results in the following three output files:

1. IMU parameters (default: `$HOME/imu_corrector.param.yaml`)
2. Velocity parameters (default: `$HOME/vehicle_velocity_converter.param.yaml`)
3. Logs (default: `$HOME/output.txt`)

### Sample Output

#### output.txt

```sh
# Validation results
# value: [min, max]
[OK] coef_vx: [0.99538, 0.99593]
[OK] stddev_vx: [0.17192, 0.19161]
[OK] angular_velocity_offset_x: [-0.00742, -0.00727]
[OK] angular_velocity_offset_y: [-0.00119, -0.00115]
[OK] angular_velocity_offset_z: [0.00635, 0.00641]
[OK] angular_velocity_stddev_xx: [0.04151, 0.04258]
[OK] angular_velocity_stddev_yy: [0.04151, 0.04258]
[OK] angular_velocity_stddev_zz: [0.04151, 0.04258]
```

#### imu_corrector.param.yaml

```sh
# Estimated by autoware_deviation_estimator
/**:
  ros__parameters:
    angular_velocity_stddev_xx: 0.01798
    angular_velocity_stddev_yy: 0.01798
    angular_velocity_stddev_zz: 0.01798
    angular_velocity_offset_x: -0.00952
    angular_velocity_offset_y: -0.00095
    angular_velocity_offset_z: 0.00607
```

#### vehicle_velocity_converter.param.yaml

```sh
# Estimated by autoware_deviation_estimator
/**:
  ros__parameters:
    speed_scale_factor: 0.99507
    velocity_stddev_xx: 0.16708
    velocity_stddev_xx: 0.1 # Default value
    frame_id: base_link # Default value
```
