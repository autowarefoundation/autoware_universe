# autoware_imu_corrector

## imu_corrector

`imu_corrector_node` is a node that correct imu data.

1. Correct yaw rate offset $b$ by reading the parameter.
2. Correct yaw rate standard deviation $\sigma$ by reading the parameter.
3. Correct yaw rate scale $s$ using NDT pose as a ground truth.

Mathematically, we assume the following equation:

$$
\tilde{\omega}(t) = s(t) * \omega(t) + b(t) + n(t)
$$

where $\tilde{\omega}$ denotes observed angular velocity, $\omega$ denotes true angular velocity, $b$ denotes an offset, $s$ denotes the scale, and $n$ denotes a gaussian noise.
We also assume that $n\sim\mathcal{N}(0, \sigma^2)$.

<!-- Use the value estimated by [deviation_estimator](https://github.com/autowarefoundation/autoware_tools/tree/main/localization/deviation_estimation_tools) as the parameters for this node. -->

### Input

| Name     | Type                    | Description  |
| -------- | ----------------------- | ------------ |
| `~input` | `sensor_msgs::msg::Imu` | raw imu data |

### Output

| Name      | Type                    | Description        |
| --------- | ----------------------- | ------------------ |
| `~output` | `sensor_msgs::msg::Imu` | corrected imu data |

### Parameters

| Name                         | Type   | Description                                      |
| ---------------------------- | ------ | ------------------------------------------------ |
| `angular_velocity_offset_x`  | double | roll rate offset in imu_link [rad/s]             |
| `angular_velocity_offset_y`  | double | pitch rate offset imu_link [rad/s]               |
| `angular_velocity_offset_z`  | double | yaw rate offset imu_link [rad/s]                 |
| `angular_velocity_stddev_xx` | double | roll rate standard deviation imu_link [rad/s]    |
| `angular_velocity_stddev_yy` | double | pitch rate standard deviation imu_link [rad/s]   |
| `angular_velocity_stddev_zz` | double | yaw rate standard deviation imu_link [rad/s]     |
| `acceleration_stddev`        | double | acceleration standard deviation imu_link [m/s^2] |

**Note:** The angular velocity offset values introduce a fixed compensation that is not considered in the gyro bias estimation. If the `on_off_correction.correct_for_dynamic_bias` flag and the `on_off_correction.correct_for_static_bias` flags are enabled, automatically the `correct_for_static_bias` will be disabled to avoid correction errors.

## gyro_bias_estimator

`gyro_bias_validator` is a node that validates the bias of the gyroscope. It subscribes to the `sensor_msgs::msg::Imu` topic and validate if the bias of the gyroscope is within the specified range.

Note that the node calculates bias from the gyroscope data by averaging the data only when the vehicle is stopped.

### Input

| Name               | Type                                            | Description      |
| ------------------ | ----------------------------------------------- | ---------------- |
| `~/input/imu_raw`  | `sensor_msgs::msg::Imu`                         | **raw** imu data |
| `~/input/pose`     | `geometry_msgs::msg::PoseWithCovarianceStamped` | ndt pose         |
| `~/input/odometry` | `nav_msgs::msg::Odometry`                       | odometry data    |

Note that the input pose is assumed to be accurate enough. For example when using NDT, we assume that the NDT is appropriately converged.

Currently, it is possible to use methods other than NDT as a `pose_source` for Autoware, but less accurate methods are not suitable for IMU bias estimation.

In the future, with careful implementation for pose errors, the IMU bias estimated by NDT could potentially be used not only for validation but also for online calibration.

The Extended Kalman Filter (EKF) is used for scale estimation. The NDT pose is used as ground truth, and we assume it's accurate enough to provide long-term convergence for the correct scale observation.

### Output

| Name                  | Type                                 | Description                                 |
| --------------------- | ------------------------------------ | ------------------------------------------- |
| `~/output/gyro_bias`  | `geometry_msgs::msg::Vector3Stamped` | bias of the gyroscope [rad/s]               |
| `~/output/gyro_scale` | `geometry_msgs::msg::Vector3Stamped` | estimated scale of the gyroscope [no units] |

### Parameters (Bias estimation)

Note that this node also uses `angular_velocity_offset_x`, `angular_velocity_offset_y`, `angular_velocity_offset_z` parameters from `imu_corrector.param.yaml`.

| Name                                  | Type   | Description                                                                                 |
| ------------------------------------- | ------ | ------------------------------------------------------------------------------------------- |
| `gyro_bias_threshold`                 | double | threshold of the bias of the gyroscope [rad/s]                                              |
| `timer_callback_interval_sec`         | double | seconds about the timer callback function [sec]                                             |
| `diagnostics_updater_interval_sec`    | double | period of the diagnostics updater [sec]                                                     |
| `straight_motion_ang_vel_upper_limit` | double | upper limit of yaw angular velocity, beyond which motion is not considered straight [rad/s] |

### Parameters (Scale estimation)

| Name                                  | Type   | Description                                                            |
| ------------------------------------- | ------ | ---------------------------------------------------------------------- |
| `estimate_scale_init`                 | double | Initial value for scale estimation                                     |
| `min_allowed_scale`                   | double | Minimum allowed scale value                                            |
| `max_allowed_scale`                   | double | Maximum allowed scale value                                            |
| `threshold_to_estimate_scale`         | double | Minimum yaw rate required to estimate scale                            |
| `percentage_scale_rate_allow_correct` | double | Allowed percentage change with respect to current scale for correction |
| `alpha`                               | double | Filter coefficient for scale (complementary filter)                    |
| `delay_gyro_ms`                       | int    | Delay applied to gyro data in milliseconds                             |
| `samples_filter_pose_rate`            | int    | Number of samples for pose rate filtering                              |
| `samples_filter_gyro_rate`            | int    | Number of samples for gyro rate filtering                              |
| `alpha_gyro`                          | double | Filter coefficient for gyro rate                                       |
| `buffer_size_gyro`                    | int    | Buffer size for gyro data                                              |
| `alpha_ndt_rate`                      | double | Filter coefficient for NDT rate                                        |
| `ekf_rate.max_variance_p`             | double | Maximum allowed variance for EKF rate estimation                       |
| `ekf_rate.variance_p_after`           | double | Variance after initialization for EKF rate estimation                  |
| `ekf_rate.process_noise_q`            | double | Process noise for EKF rate estimation                                  |
| `ekf_rate.process_noise_q_after`      | double | Process noise after initialization for EKF rate estimation             |
| `ekf_rate.measurement_noise_r`        | double | Measurement noise for EKF rate estimation                              |
| `ekf_rate.measurement_noise_r_after`  | double | Measurement noise after initialization for EKF rate estimation         |
| `ekf_rate.samples_to_init`            | int    | Number of samples to initialize EKF rate estimation                    |
| `ekf_rate.min_covariance`             | double | Minimum covariance for EKF rate estimation                             |
| `ekf_angle.process_noise_q_angle`     | double | Process noise for EKF angle estimation                                 |
| `ekf_angle.variance_p_angle`          | double | Initial variance for EKF angle estimation                              |
| `ekf_angle.measurement_noise_r_angle` | double | Measurement noise for EKF angle estimation                             |
| `ekf_angle.min_covariance_angle`      | double | Minimum covariance for EKF angle estimation                            |
| `ekf_angle.decay_coefficient`         | double | Decay coefficient for EKF angle estimation                             |

## IMU scale/bias injection

In order to test the result of the scale and bias estimation for the gyro, an optional artificial scale and bias can be injected into the raw IMU data using the parameters below. The IMU scale can be observed through an output that can be remapped to be the input of the 'imu_corrector'.

### Output

| Name                  | Type                    | Description                     |
| --------------------- | ----------------------- | ------------------------------- |
| `~/output/imu_scaled` | `sensor_msgs::msg::Imu` | IMU data after scale correction |

### Parameters

| Name               | Type   | Description                                                         |
| ------------------ | ------ | ------------------------------------------------------------------- |
| `modify_imu_scale` | bool   | Enable or disable scale injection                                   |
| `scale_on_purpose` | double | Value to inject as scale                                            |
| `bias_on_purpose`  | double | Value to inject as bias                                             |
| `drift_scale`      | double | Value to add to the scale value every loop, to simulate scale drift |
| `drift_bias`       | double | Value to add to the bias value every loop, to simulate bias drift   |

## IMU Correction control

These parameters control how gyroscope bias and scale are corrected. **Only one bias-correction method should be enabled at a time.**  
If both flags are enabled, **static bias correction is automatically disabled**.

### Static Bias Correction (`correct_for_static_bias`)

- Offset values must be precomputed and stored in the configuration file under the parameter:
  `angular_velocity_offset_[x, y, z]`
- Applies these offsets directly to the raw gyroscope data:
  `~/input/imu_raw`
  -Should be used only if the gyroscope offset doesn't change along the time.

### Dynamic Bias Correction (`correct_for_dynamic_bias`)

- Uses bias estimates published on:
  `~/output/gyro_bias`
- Bias is estimated with the help of the odometry data:
  `~/input/odometry`
- The estimated bias is applied to correct the raw gyroscope data:
  `~/input/imu_raw`
  -Should be used when the gyroscope offset is changing along the time and odometry data is accessible.

### Parameters

| Name                                         | Type | Description                                                |
| -------------------------------------------- | ---- | ---------------------------------------------------------- |
| `on_off_correction.correct_for_static_bias`  | bool | Enable or disable static bias correction (default: true)   |
| `on_off_correction.correct_for_dynamic_bias` | bool | Enable or disable dynamic bias correction (default: false) |
| `on_off_correction.correct_for_scale`        | bool | Enable or disable scale correction (default: false)        |
