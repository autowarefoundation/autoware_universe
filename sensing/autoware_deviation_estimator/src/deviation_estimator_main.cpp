// Copyright 2018-2019 Autoware Foundation
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

#include "autoware/deviation_estimator/deviation_estimator.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <rosbag_path>" << std::endl;
    return 1;
  }
  const std::string rosbag_path = argv[1];

  std::cout << "deviation_estimator_unit_tool" << std::endl;

  // Load parameters
  std::map<std::string, rclcpp::Parameter> param_map;
  {
    const std::string yaml_path =
      ament_index_cpp::get_package_share_directory("deviation_estimator") +
      "/config/deviation_estimator.param.yaml";
    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      std::cerr << "Failed to parse yaml file: " << yaml_path << std::endl;
      std::exit(1);
    }
    const std::vector<rclcpp::Parameter> parameters =
      rclcpp::parameter_map_from(params_st, "").at("");
    rcl_yaml_node_struct_fini(params_st);
    for (const rclcpp::Parameter & param : parameters) {
      param_map[param.get_name()] = param;
    }
  }

  // Prepare rosbag reader
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbag_path;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  // Prepare serialization
  rclcpp::Serialization<autoware_vehicle_msgs::msg::VelocityReport> serialization_velocity_status;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization_tf;
  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_imu;
  rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serialization_pose;

  // Prepare tf_buffer
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer tf_buffer(clock);

  // Prepare variables
  std::vector<autoware::deviation_estimator::TrajectoryData> trajectory_data_list;
  rclcpp::Time first_stamp(std::numeric_limits<int64_t>::max(), RCL_ROS_TIME);
  const double time_window = param_map.at("time_window").as_double();
  std::string imu_frame_id;

  // ----------- //
  // Read rosbag //
  // ----------- //
  while (reader.has_next()) {
    const rosbag2_storage::SerializedBagMessageSharedPtr serialized_message = reader.read_next();
    const std::string topic_name = serialized_message->topic_name;
    const rclcpp::SerializedMessage msg(*serialized_message->serialized_data);

    if (topic_name == "/vehicle/status/velocity_status") {
      autoware_vehicle_msgs::msg::VelocityReport::SharedPtr velocity_status_msg =
        std::make_shared<autoware_vehicle_msgs::msg::VelocityReport>();
      serialization_velocity_status.deserialize_message(&msg, velocity_status_msg.get());
      const rclcpp::Time curr_stamp = velocity_status_msg->header.stamp;
      first_stamp = std::min(first_stamp, curr_stamp);
      const double diff_sec = (curr_stamp - first_stamp).seconds();
      const int64_t index = static_cast<int64_t>(diff_sec / time_window);
      if (index >= static_cast<int64_t>(trajectory_data_list.size())) {
        trajectory_data_list.resize(index + 1);
      }
      autoware_internal_debug_msgs::msg::Float64Stamped vx;
      vx.stamp = velocity_status_msg->header.stamp;
      vx.data = velocity_status_msg->longitudinal_velocity;
      trajectory_data_list[index].vx_list.push_back(vx);

    } else if (topic_name == "/tf_static") {
      tf2_msgs::msg::TFMessage::SharedPtr tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
      serialization_tf.deserialize_message(&msg, tf_msg.get());
      for (const auto & transform : tf_msg->transforms) {
        try {
          tf_buffer.setTransform(transform, "default_authority", false);
        } catch (const tf2::TransformException & ex) {
          std::cerr << "Transform exception: " << ex.what() << std::endl;
          std::exit(1);
        }
      }

    } else if (topic_name == "/sensing/imu/tamagawa/imu_raw") {
      sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
      serialization_imu.deserialize_message(&msg, imu_msg.get());
      imu_frame_id = imu_msg->header.frame_id;
      geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
      try {
        const geometry_msgs::msg::TransformStamped transform =
          tf_buffer.lookupTransform("base_link", imu_msg->header.frame_id, tf2::TimePointZero);
        vec_stamped_transformed.header = imu_msg->header;
        tf2::doTransform(imu_msg->angular_velocity, vec_stamped_transformed.vector, transform);
      } catch (const tf2::TransformException & ex) {
        std::cerr << "Transform exception: " << ex.what() << std::endl;
        continue;
      }
      const rclcpp::Time curr_stamp = imu_msg->header.stamp;
      first_stamp = std::min(first_stamp, curr_stamp);
      const double diff_sec = (curr_stamp - first_stamp).seconds();
      const int64_t index = static_cast<int64_t>(diff_sec / time_window);
      if (index >= static_cast<int64_t>(trajectory_data_list.size())) {
        trajectory_data_list.resize(index + 1);
      }
      trajectory_data_list[index].gyro_list.push_back(vec_stamped_transformed);

    } else if (topic_name == "/localization/pose_estimator/pose_with_covariance") {
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      serialization_pose.deserialize_message(&msg, pose_msg.get());
      const rclcpp::Time curr_stamp = pose_msg->header.stamp;
      first_stamp = std::min(first_stamp, curr_stamp);
      const double diff_sec = (curr_stamp - first_stamp).seconds();
      const int64_t index = static_cast<int64_t>(diff_sec / time_window);
      if (index >= static_cast<int64_t>(trajectory_data_list.size())) {
        trajectory_data_list.resize(index + 1);
      }
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = pose_msg->header;
      pose_stamped.pose = pose_msg->pose.pose;
      trajectory_data_list[index].pose_list.push_back(pose_stamped);

    } else {
    }
  }

  // ------------------ //
  // Estimate deviation //
  // ------------------ //
  const double wz_threshold = param_map.at("wz_threshold").as_double();
  const double vx_threshold = param_map.at("vx_threshold").as_double();
  const double accel_threshold = param_map.at("accel_threshold").as_double();
  const bool gyro_only_use_straight = param_map.at("gyro_estimation.only_use_straight").as_bool();
  const bool gyro_only_use_moving = param_map.at("gyro_estimation.only_use_moving").as_bool();
  const bool gyro_only_use_constant_velocity =
    param_map.at("gyro_estimation.only_use_constant_velocity").as_bool();
  const bool gyro_add_bias_uncertainty =
    param_map.at("gyro_estimation.add_bias_uncertainty").as_bool();
  const bool velocity_only_use_straight =
    param_map.at("velocity_estimation.only_use_straight").as_bool();
  const bool velocity_only_use_moving =
    param_map.at("velocity_estimation.only_use_moving").as_bool();
  const bool velocity_only_use_constant_velocity =
    param_map.at("velocity_estimation.only_use_constant_velocity").as_bool();
  const bool velocity_add_bias_uncertainty =
    param_map.at("velocity_estimation.add_bias_uncertainty").as_bool();
  std::unique_ptr<autoware::deviation_estimator::GyroBiasModule> gyro_bias_module =
    std::make_unique<autoware::deviation_estimator::GyroBiasModule>();
  std::unique_ptr<autoware::deviation_estimator::VelocityCoefModule> vel_coef_module =
    std::make_unique<autoware::deviation_estimator::VelocityCoefModule>();
  std::vector<autoware::deviation_estimator::TrajectoryData> traj_data_list_for_velocity;
  std::vector<autoware::deviation_estimator::TrajectoryData> traj_data_list_for_gyro;

  if (gyro_add_bias_uncertainty) {
    std::cerr << "gyro_add_bias_uncertainty is not supported yet." << std::endl;
    std::exit(1);
  }
  if (velocity_add_bias_uncertainty) {
    std::cerr << "velocity_add_bias_uncertainty is not supported yet." << std::endl;
    std::exit(1);
  }

  std::unique_ptr<autoware::deviation_estimator::ValidationModule> validation_module =
    std::make_unique<autoware::deviation_estimator::ValidationModule>(
      param_map.at("thres_coef_vx").as_double(), param_map.at("thres_stddev_vx").as_double(),
      param_map.at("thres_bias_gyro").as_double(), param_map.at("thres_stddev_gyro").as_double(),
      5);

  autoware::deviation_estimator::Logger results_logger(".");

  for (const autoware::deviation_estimator::TrajectoryData & traj_data : trajectory_data_list) {
    std::cout << "traj_data.pose_list.size(): " << traj_data.pose_list.size() << std::endl;
    std::cout << "traj_data.gyro_list.size(): " << traj_data.gyro_list.size() << std::endl;
    std::cout << "traj_data.vx_list.size(): " << traj_data.vx_list.size() << std::endl;

    // Skip if there is too little data such as terminal data
    if (
      traj_data.pose_list.size() < 2 || traj_data.gyro_list.size() < 2 ||
      traj_data.vx_list.size() < 2) {
      continue;
    }

    const bool is_straight =
      autoware::deviation_estimator::get_mean_abs_wz(traj_data.gyro_list) < wz_threshold;
    const bool is_moving =
      autoware::deviation_estimator::get_mean_abs_vx(traj_data.vx_list) > vx_threshold;
    const bool is_constant_velocity =
      std::abs(autoware::deviation_estimator::get_mean_accel(traj_data.vx_list)) < accel_threshold;

    const bool use_gyro = autoware::deviation_estimator::whether_to_use_data(
      is_straight, is_moving, is_constant_velocity, gyro_only_use_straight, gyro_only_use_moving,
      gyro_only_use_constant_velocity);
    const bool use_velocity = autoware::deviation_estimator::whether_to_use_data(
      is_straight, is_moving, is_constant_velocity, velocity_only_use_straight,
      velocity_only_use_moving, velocity_only_use_constant_velocity);
    if (use_velocity) {
      vel_coef_module->update_coef(traj_data);
      traj_data_list_for_velocity.push_back(traj_data);
    }
    if (use_gyro) {
      gyro_bias_module->update_bias(traj_data);
      traj_data_list_for_gyro.push_back(traj_data);
    }

    double stddev_vx = autoware::deviation_estimator::estimate_stddev_velocity(
      traj_data_list_for_velocity, vel_coef_module->get_coef());

    auto stddev_angvel_base = autoware::deviation_estimator::estimate_stddev_angular_velocity(
      traj_data_list_for_gyro, gyro_bias_module->get_bias_base_link());

    // print
    const geometry_msgs::msg::Vector3 curr_gyro_bias = gyro_bias_module->get_bias_base_link();

    std::cout << std::fixed                                                       //
              << "is_straight=" << is_straight                                    //
              << ", is_moving=" << is_moving                                      //
              << ", is_constant_velocity=" << is_constant_velocity                //
              << ", use_gyro=" << use_gyro                                        //
              << ", use_velocity=" << use_velocity                                //
              << ", vel_coef_module->get_coef()=" << vel_coef_module->get_coef()  //
              << ", curr_gyro_bias.x=" << curr_gyro_bias.x                        //
              << ", curr_gyro_bias.y=" << curr_gyro_bias.y                        //
              << ", curr_gyro_bias.z=" << curr_gyro_bias.z                        //
              << ", stddev_vx=" << stddev_vx                                      //
              << ", stddev_angvel_base.x=" << stddev_angvel_base.x                //
              << ", stddev_angvel_base.y=" << stddev_angvel_base.y                //
              << ", stddev_angvel_base.z=" << stddev_angvel_base.z                //
              << std::endl;

    // For IMU link standard deviation, we use the yaw standard deviation in base_link.
    // This is because the standard deviation estimation of x and y in base_link may not be accurate
    // especially when the data contains a motion when the people are getting on/off the vehicle,
    // which causes the vehicle to tilt in roll and pitch. In this case, we would like to use the
    // standard deviation of yaw axis in base_link.
    double stddev_angvel_imu = stddev_angvel_base.z;
    geometry_msgs::msg::Vector3 stddev_angvel_imu_msg = autoware::universe_utils::createVector3(
      stddev_angvel_imu, stddev_angvel_imu, stddev_angvel_imu);

    const geometry_msgs::msg::TransformStamped transform =
      tf_buffer.lookupTransform(imu_frame_id, "base_link", tf2::TimePointZero);
    geometry_msgs::msg::Vector3 bias_angvel_imu;
    tf2::doTransform(gyro_bias_module->get_bias_base_link(), bias_angvel_imu, transform);

    validation_module->set_velocity_data(vel_coef_module->get_coef(), stddev_vx);
    validation_module->set_gyro_data(bias_angvel_imu, stddev_angvel_imu_msg);

    results_logger.log_estimated_result_section(
      stddev_vx, vel_coef_module->get_coef(), stddev_angvel_imu_msg, bias_angvel_imu);
    results_logger.log_validation_result_section(*validation_module);

    std::cout << "saved to ./" << std::endl;
  }

  std::cout << "count for velocity : " << traj_data_list_for_velocity.size() << std::endl;
  std::cout << "count for gyro : " << traj_data_list_for_gyro.size() << std::endl;
  std::cout << "Finished." << std::endl;
}
