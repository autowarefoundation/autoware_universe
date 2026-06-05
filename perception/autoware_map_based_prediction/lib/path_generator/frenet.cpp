// Copyright 2021 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/map_based_prediction/path_generator/frenet.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>

#include <vector>

namespace autoware::map_based_prediction
{

namespace
{

Eigen::Vector3d calcLatCoefficients(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double T)
{
  // Lateral Path Calculation
  // Quintic polynomial for d
  // A = np.array([[T**3, T**4, T**5],
  //               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
  //               [6 * T, 12 * T ** 2, 20 * T ** 3]])
  // A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
  //                    [-15/(T**4), 7/(T**3), -1/(T**2)],
  //                    [6/(T**5), -3/(T**4),  1/(2*T**3)]])
  // b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],
  //                [vxe - self.a1 - 2 * self.a2 * T],
  //                [axe - 2 * self.a2]])
  const auto T2 = T * T;
  const auto T3 = T2 * T;
  const auto T4 = T3 * T;
  const auto T5 = T4 * T;

  Eigen::Matrix3d A_lat_inv;
  A_lat_inv << 10 / T3, -4 / T2, 1 / (2 * T), -15 / T4, 7 / T3, -1 / T2, 6 / T5, -3 / T4,
    1 / (2 * T3);
  Eigen::Vector3d b_lat;
  b_lat[0] = target_point.d - current_point.d - current_point.d_vel * T;
  b_lat[1] = target_point.d_vel - current_point.d_vel;
  b_lat[2] = target_point.d_acc;

  return A_lat_inv * b_lat;
}

Eigen::Vector2d calcLonCoefficients(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double T)
{
  // Longitudinal Path Calculation
  // Quadric polynomial
  // A_inv = np.matrix([[1/(T**2), -1/(3*T)],
  //                         [-1/(2*T**3), 1/(4*T**2)]])
  // b = np.matrix([[vxe - self.a1 - 2 * self.a2 * T],
  //               [axe - 2 * self.a2]])
  const auto T2 = T * T;
  const auto T3 = T2 * T;

  Eigen::Matrix2d A_lon_inv;
  A_lon_inv << 1 / T2, -1 / (3 * T), -1 / (2 * T3), 1 / (4 * T2);
  Eigen::Vector2d b_lon;
  b_lon[0] = target_point.s_vel - current_point.s_vel;
  b_lon[1] = 0.0;
  return A_lon_inv * b_lon;
}

std::vector<double> interpolationLerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys)
{
  // calculate linear interpolation
  // extrapolate the value if the query key is out of the base key range
  std::vector<double> query_values;
  size_t key_index = 0;
  double last_query_key = query_keys.at(0);
  for (const auto query_key : query_keys) {
    // search for the closest key index
    // if current query key is larger than the last query key, search base_keys increasing order
    if (query_key >= last_query_key) {
      while (base_keys.at(key_index + 1) < query_key) {
        if (key_index == base_keys.size() - 2) {
          break;
        }
        ++key_index;
      }
    } else {
      // if current query key is smaller than the last query key, search base_keys decreasing order
      while (base_keys.at(key_index) > query_key) {
        if (key_index == 0) {
          break;
        }
        --key_index;
      }
    }
    last_query_key = query_key;

    const double & src_val = base_values.at(key_index);
    const double & dst_val = base_values.at(key_index + 1);
    const double ratio = (query_key - base_keys.at(key_index)) /
                         (base_keys.at(key_index + 1) - base_keys.at(key_index));

    const double interpolated_val = src_val + (dst_val - src_val) * ratio;
    query_values.push_back(interpolated_val);
  }

  return query_values;
}

std::vector<tf2::Quaternion> interpolationLerp(
  const std::vector<double> & base_keys, const std::vector<tf2::Quaternion> & base_values,
  const std::vector<double> & query_keys)
{
  // calculate linear interpolation
  // extrapolate the value if the query key is out of the base key range
  std::vector<tf2::Quaternion> query_values;
  size_t key_index = 0;
  double last_query_key = query_keys.at(0);
  for (const auto query_key : query_keys) {
    // search for the closest key index
    // if current query key is larger than the last query key, search base_keys increasing order
    if (query_key >= last_query_key) {
      while (base_keys.at(key_index + 1) < query_key) {
        if (key_index == base_keys.size() - 2) {
          break;
        }
        ++key_index;
      }
    } else {
      // if current query key is smaller than the last query key, search base_keys decreasing order
      while (base_keys.at(key_index) > query_key) {
        if (key_index == 0) {
          break;
        }
        --key_index;
      }
    }
    last_query_key = query_key;

    const tf2::Quaternion & src_val = base_values.at(key_index);
    const tf2::Quaternion & dst_val = base_values.at(key_index + 1);
    const double ratio = (query_key - base_keys.at(key_index)) /
                         (base_keys.at(key_index + 1) - base_keys.at(key_index));

    // in case of extrapolation, export the nearest quaternion
    if (ratio < 0.0) {
      query_values.push_back(src_val);
      continue;
    }
    if (ratio > 1.0) {
      query_values.push_back(dst_val);
      continue;
    }
    const auto interpolated_quat = tf2::slerp(src_val, dst_val, ratio);
    query_values.push_back(interpolated_quat);
  }

  return query_values;
}

}  // namespace

FrenetPoint getFrenetPoint(
  const TrackedObject & object, const geometry_msgs::msg::Pose & ref_pose, const double duration,
  const double speed_limit, const bool use_vehicle_acceleration,
  const double acceleration_exponential_half_life)
{
  FrenetPoint frenet_point;

  // 1. Position
  const auto obj_point = object.kinematics.pose_with_covariance.pose.position;
  const float obj_yaw =
    static_cast<float>(tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation));
  const float lane_yaw = static_cast<float>(tf2::getYaw(ref_pose.orientation));
  frenet_point.s = (obj_point.x - ref_pose.position.x) * cos(lane_yaw) +
                   (obj_point.y - ref_pose.position.y) * sin(lane_yaw);
  frenet_point.d = -(obj_point.x - ref_pose.position.x) * sin(lane_yaw) +
                   (obj_point.y - ref_pose.position.y) * cos(lane_yaw);

  // 2. Velocity (adjusted by acceleration)
  const float vx = static_cast<float>(object.kinematics.twist_with_covariance.twist.linear.x);
  const float vy = static_cast<float>(object.kinematics.twist_with_covariance.twist.linear.y);
  const float ax =
    use_vehicle_acceleration
      ? static_cast<float>(object.kinematics.acceleration_with_covariance.accel.linear.x)
      : 0.0;
  const float ay =
    use_vehicle_acceleration
      ? static_cast<float>(object.kinematics.acceleration_with_covariance.accel.linear.y)
      : 0.0;
  const float delta_yaw = obj_yaw - lane_yaw;

  // using a decaying acceleration model. Consult the README for more information about the model.
  const double t_h = duration;
  const float lambda = std::log(2) / acceleration_exponential_half_life;

  auto have_same_sign = [](double a, double b) -> bool {
    return (a >= 0.0 && b >= 0.0) || (a < 0.0 && b < 0.0);
  };

  auto get_acceleration_adjusted_velocity = [&](const double v, const double a) {
    constexpr double epsilon = 1E-5;
    if (std::abs(a) < epsilon) {
      // Assume constant speed
      return v;
    }
    // Get velocity after time horizon
    const auto terminal_velocity = v + a * (1.0 / lambda) * (1 - std::exp(-lambda * t_h));

    const auto lambda_2 = lambda * lambda;

    // If vehicle is decelerating, make sure its speed does not change signs (we assume it will, at
    // most stop, not reverse its direction)
    if (!have_same_sign(terminal_velocity, v)) {
      // we assume a forwards moving vehicle will not decelerate to 0 and then move backwards
      // if the velocities don't have the same sign, calculate when the vehicle reaches 0 speed ->
      // time t_stop

      // 0 = Vo + acc(1/lambda)(1-e^(-lambda t_stop))
      // e^(-lambda t_stop) = 1 - (-Vo* lambda)/acc
      // t_stop = (-1/lambda)*ln(1 - (-Vo* lambda)/acc)
      // t_stop = (-1/lambda)*ln(1 + (Vo* lambda)/acc)
      auto t_stop = (-1.0 / lambda) * std::log1p(v * lambda / a);

      // Calculate the distance traveled until stopping
      auto distance_to_reach_zero_speed =
        v * t_stop + a * t_stop * (1.0 / lambda) + a * (1.0 / lambda_2) * std::expm1(-lambda * t_h);
      // Output an equivalent constant speed
      return distance_to_reach_zero_speed / t_h;
    }

    // if the vehicle speed limit is not surpassed we return an equivalent speed = x(T) / T
    // alternatively, if the vehicle is still accelerating and has surpassed the speed limit.
    // assume it will continue accelerating (reckless driving)
    const bool object_has_surpassed_limit_already = v > speed_limit;
    if (terminal_velocity < speed_limit || object_has_surpassed_limit_already)
      return v + a * (1.0 / lambda) + (a / (t_h * lambda_2)) * std::expm1(-lambda * t_h);

    // It is assumed the vehicle accelerates until final_speed is reached and
    // then continues at constant speed for the rest of the time horizon
    // So, we calculate the time it takes to reach the speed limit and compute how far the vehicle
    // would go if it accelerated until reaching the speed limit, and then continued at a constant
    // speed.
    const double t_f = (-1.0 / lambda) * std::log(1 - ((speed_limit - v) * lambda) / a);
    const double distance_covered =
      // Distance covered while accelerating
      a * (1.0 / lambda) * t_f + a * (1.0 / lambda_2) * std::expm1(-lambda * t_f) + v * t_f +
      // Distance covered at constant speed for the rest of the horizon time
      speed_limit * (t_h - t_f);
    return distance_covered / t_h;
  };

  const float acceleration_adjusted_velocity_x = get_acceleration_adjusted_velocity(vx, ax);
  const float acceleration_adjusted_velocity_y = get_acceleration_adjusted_velocity(vy, ay);
  frenet_point.s_vel = acceleration_adjusted_velocity_x * std::cos(delta_yaw) -
                       acceleration_adjusted_velocity_y * std::sin(delta_yaw);
  frenet_point.d_vel = acceleration_adjusted_velocity_x * std::sin(delta_yaw) +
                       acceleration_adjusted_velocity_y * std::cos(delta_yaw);

  // 3. Acceleration, assuming constant acceleration
  frenet_point.s_acc = 0.0;
  frenet_point.d_acc = 0.0;

  return frenet_point;
}

FrenetPath generateFrenetPath(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double max_length,
  const double duration, const double lateral_duration, const double sampling_time_interval)
{
  FrenetPath path;

  // Compute Lateral and Longitudinal Coefficients to generate the trajectory
  const Eigen::Vector3d lat_coeff =
    calcLatCoefficients(current_point, target_point, lateral_duration);
  const Eigen::Vector2d lon_coeff = calcLonCoefficients(current_point, target_point, duration);

  // Generate the trajectory
  path.reserve(static_cast<size_t>(duration / sampling_time_interval));
  for (double t = 0.0; t <= duration; t += sampling_time_interval) {
    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;
    const auto t5 = t4 * t;
    const double current_acc =
      0.0;  // Currently we assume the object is traveling at a constant speed
    const double d_next_ = current_point.d + current_point.d_vel * t + current_acc * 2.0 * t2 +
                           lat_coeff(0) * t3 + lat_coeff(1) * t4 + lat_coeff(2) * t5;
    // t > lateral_duration: target_point.d, else d_next_
    const double d_next = t > lateral_duration ? target_point.d : d_next_;
    const double s_next = current_point.s + current_point.s_vel * t + 2.0 * current_acc * t2 +
                          lon_coeff(0) * t3 + lon_coeff(1) * t4;

    if (s_next > max_length) {
      break;
    }

    // Fill the FrenetPoint, velocity and acceleration are not used in the path generator
    FrenetPoint point;
    point.s = s_next;
    point.d = d_next;
    path.push_back(point);
  }

  return path;
}

PosePath interpolateReferencePath(
  const PosePath & base_path, const FrenetPath & frenet_predicted_path)
{
  PosePath interpolated_path;
  const size_t interpolate_num = frenet_predicted_path.size();
  if (interpolate_num < 2) {
    interpolated_path.emplace_back(base_path.front());
    return interpolated_path;
  }

  // Prepare base path vectors
  std::vector<double> base_path_x(base_path.size());
  std::vector<double> base_path_y(base_path.size());
  std::vector<double> base_path_z(base_path.size());
  std::vector<tf2::Quaternion> base_path_orientation(base_path.size());
  std::vector<double> base_path_s(base_path.size(), 0.0);
  for (size_t i = 0; i < base_path.size(); ++i) {
    base_path_x.at(i) = base_path.at(i).position.x;
    base_path_y.at(i) = base_path.at(i).position.y;
    base_path_z.at(i) = base_path.at(i).position.z;
    tf2::Quaternion src_tf;
    tf2::fromMsg(base_path.at(i).orientation, src_tf);
    base_path_orientation.at(i) = src_tf;
    if (i > 0) {
      base_path_s.at(i) = base_path_s.at(i - 1) +
                          autoware_utils::calc_distance2d(base_path.at(i - 1), base_path.at(i));
    }
  }

  // Prepare resampled s vector
  std::vector<double> resampled_s(frenet_predicted_path.size());
  for (size_t i = 0; i < frenet_predicted_path.size(); ++i) {
    resampled_s.at(i) = frenet_predicted_path.at(i).s;
  }

  // Linear Interpolation for x, y, z, and orientation
  std::vector<double> lerp_ref_path_x = interpolationLerp(base_path_s, base_path_x, resampled_s);
  std::vector<double> lerp_ref_path_y = interpolationLerp(base_path_s, base_path_y, resampled_s);
  std::vector<double> lerp_ref_path_z = interpolationLerp(base_path_s, base_path_z, resampled_s);
  std::vector<tf2::Quaternion> lerp_ref_path_orientation =
    interpolationLerp(base_path_s, base_path_orientation, resampled_s);

  // Set the interpolated PosePath
  interpolated_path.resize(interpolate_num);
  for (size_t i = 0; i < interpolate_num; ++i) {
    geometry_msgs::msg::Pose interpolated_pose;
    interpolated_pose.position = autoware_utils::create_point(
      lerp_ref_path_x.at(i), lerp_ref_path_y.at(i), lerp_ref_path_z.at(i));
    interpolated_pose.orientation = tf2::toMsg(lerp_ref_path_orientation.at(i));
    interpolated_path.at(i) = interpolated_pose;
  }

  return interpolated_path;
}

PredictedPath convertToPredictedPath(
  const TrackedObject & object, const FrenetPath & frenet_predicted_path, const PosePath & ref_path,
  const double sampling_time_interval)
{
  // Object position
  const auto & object_pose = object.kinematics.pose_with_covariance.pose;
  const double object_height = object.shape.dimensions.z / 2.0;

  // Convert Frenet Path to Cartesian Path
  PredictedPath predicted_path;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval);
  predicted_path.path.resize(ref_path.size());

  // Set the first point as the object's current position
  predicted_path.path.at(0) = object_pose;

  // Convert the rest of the points
  for (size_t i = 1; i < predicted_path.path.size(); ++i) {
    // Reference Point from interpolated reference path
    const auto & ref_pose = ref_path.at(i);

    // Frenet Point from frenet predicted path
    const auto & frenet_point = frenet_predicted_path.at(i);
    double d_offset = frenet_point.d;

    // Converted Pose
    auto predicted_pose = autoware_utils::calc_offset_pose(ref_pose, 0.0, d_offset, 0.0);
    predicted_pose.position.z += object_height;
    const double yaw = autoware_utils::calc_azimuth_angle(
      predicted_path.path.at(i - 1).position, predicted_pose.position);
    predicted_pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);

    predicted_path.path.at(i) = predicted_pose;
  }

  return predicted_path;
}

}  // namespace autoware::map_based_prediction
