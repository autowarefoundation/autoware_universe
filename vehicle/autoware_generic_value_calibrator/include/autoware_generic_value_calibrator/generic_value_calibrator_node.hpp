//
// Copyright 2025 TIER IV, Inc. All rights reserved.
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
//

#ifndef AUTOWARE_GENERIC_VALUE_CALIBRATOR__GENERIC_VALUE_CALIBRATOR_NODE_HPP_
#define AUTOWARE_GENERIC_VALUE_CALIBRATOR__GENERIC_VALUE_CALIBRATOR_NODE_HPP_

#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "autoware_utils/ros/transform_listener.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

#include <Eigen/Dense>

#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace autoware::generic_value_calibrator
{
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using std_msgs::msg::Float32MultiArray;
using tier4_debug_msgs::msg::Float64Stamped;
using visualization_msgs::msg::MarkerArray;

using Map = std::vector<std::vector<double>>;

struct DataStamped
{
  DataStamped(const double _data, const rclcpp::Time & _data_time)
  : data{_data}, data_time{_data_time}
  {
  }
  double data;
  rclcpp::Time data_time;
};
using DataStampedPtr = std::shared_ptr<DataStamped>;

class GenericValueCalibrator : public rclcpp::Node
{
private:
  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;
  std::string csv_default_map_dir_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr update_suggest_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr current_map_error_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr updated_map_error_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr map_error_ratio_pub_;

  // Debug/Visualization Publishers
  rclcpp::Publisher<OccupancyGrid>::SharedPtr original_map_occ_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr update_map_occ_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_ave_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_std_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_count_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr data_count_with_self_pose_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr index_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr original_map_raw_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr update_map_raw_pub_;

  // Polling Subscribers
  autoware_utils::InterProcessPollingSubscriber<SteeringReport> steer_sub_{this, "~/input/steer"};
  autoware_utils::InterProcessPollingSubscriber<Float64Stamped> input_value_sub_{
    this, "~/input/value"};
  autoware_utils::InterProcessPollingSubscriber<VelocityReport> velocity_sub_{
    this, "~/input/velocity"};

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_output_csv_;

  void init_timer(double period_s);
  void init_output_csv_timer(double period_s);

  // Data storage
  TwistStamped::ConstSharedPtr twist_ptr_;
  std::vector<std::shared_ptr<TwistStamped>> twist_vec_;
  std::vector<DataStampedPtr> input_value_vec_;  // for delayed input values
  SteeringReport::ConstSharedPtr steer_ptr_;
  DataStampedPtr input_value_ptr_;
  DataStampedPtr delayed_input_value_ptr_;

  // Diagnostic Updater
  std::shared_ptr<diagnostic_updater::Updater> updater_ptr_;
  bool is_default_map_ = true;

  // Algorithm parameters
  int get_pitch_method_;
  int update_method_;
  double acceleration_ = 0.0;
  double acceleration_time_;
  double pre_acceleration_ = 0.0;
  double pre_acceleration_time_;
  double jerk_ = 0.0;
  double input_value_speed_ = 0.0;
  double pitch_ = 0.0;
  double update_hz_;
  double velocity_min_threshold_;
  double velocity_diff_threshold_;
  double value_diff_threshold_;
  double max_steer_threshold_;
  double max_pitch_threshold_;
  double max_jerk_threshold_;
  double value_velocity_thresh_;
  double max_accel_;
  double min_accel_;
  double value_to_accel_delay_;
  int precision_;
  std::string csv_calibrated_map_dir_;
  std::string output_map_file_;

  // for calculating differential of msg
  const double dif_twist_time_ = 0.2;   // 200ms
  const double dif_value_time_ = 0.16;  // 160ms
  const std::size_t twist_vec_max_size_ = 100;
  const std::size_t value_vec_max_size_ = 100;
  const double timeout_sec_ = 0.5;
  int max_data_count_;
  bool progress_file_output_ = false;

  // Map storage
  Map value_map_;
  Map update_value_map_;
  Map value_offset_covariance_value_;
  std::vector<double> vel_index_;
  std::vector<double> value_index_;

  bool update_success_;
  int update_success_count_ = 0;
  int update_count_ = 0;
  int lack_of_data_count_ = 0;
  int failed_to_get_pitch_count_ = 0;
  int too_large_pitch_count_ = 0;
  int too_low_speed_count_ = 0;
  int too_large_steer_count_ = 0;
  int too_large_jerk_count_ = 0;
  int too_large_value_spd_count_ = 0;
  int update_fail_count_ = 0;

  // for map update (RLS)
  double map_offset_ = 0.0;
  double covariance_;
  const double forgetting_factor_ = 0.999;

  // for evaluation
  std::vector<double> part_original_mse_que_;
  std::vector<double> new_mse_que_;
  std::size_t part_mse_que_size_ = 3000;
  double part_original_rmse_ = 0.0;
  double new_rmse_ = 0.0;
  double update_suggest_thresh_;

  /* for covariance calculation */
  Eigen::MatrixXd data_mean_mat_;
  Eigen::MatrixXd data_covariance_mat_;
  Eigen::MatrixXd data_num_;

  // output log
  std::ofstream output_log_;

  // Core functions
  void fetch_data();
  bool get_current_pitch_from_tf(double * pitch);
  bool take_data();
  void timer_callback_output_csv();
  void execute_update(const int value_index, const int vel_index);
  bool update_each_val_offset(
    const int value_index, const int vel_index, const double measured_acc, const double map_acc);
  void update_total_map_offset(const double measured_acc, const double map_acc);
  void update_statistics(const int value_index, const int vel_index, const double measured_acc);

  void take_input_value(const Float64Stamped::ConstSharedPtr msg);
  void take_velocity(const VelocityReport::ConstSharedPtr msg);

  static double lowpass(const double original, const double current, const double gain = 0.8);
  static double get_value_speed(
    const DataStampedPtr & prev_value, const DataStampedPtr & current_value,
    const double prev_value_speed);
  double get_accel(
    const TwistStamped::ConstSharedPtr & prev_twist,
    const TwistStamped::ConstSharedPtr & current_twist) const;
  double get_jerk();
  bool index_value_search(
    const std::vector<double> & value_index, const double value, const double value_thresh,
    int * searched_index) const;
  static int nearest_value_search(const std::vector<double> & value_index, const double value);
  void take_consistency_of_value_map();
  bool update_value_map();
  void publish_float64(const std::string & publish_type, const double val);
  void publish_update_suggest_flag();
  double get_pitch_compensated_acceleration() const;
  void execute_evaluation();
  double calculate_value_squared_error(const double value, const double vel);
  double calculate_updated_value_squared_error(const double value, const double vel);

  template <class T>
  void push_data_to_vec(const T data, const std::size_t max_size, std::vector<T> * vec);
  template <class T>
  static T get_nearest_time_data_from_vec(
    const T base_data, const double back_time, const std::vector<T> & vec);
  DataStampedPtr get_nearest_time_data_from_vec(
    DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec);
  static double get_average(const std::vector<double> & vec);
  bool is_timeout(const builtin_interfaces::msg::Time & stamp, const double timeout_sec);
  bool is_timeout(const DataStampedPtr & data_stamped, const double timeout_sec);

  bool write_map_to_csv(
    std::vector<double> vel_index, std::vector<double> value_index, Map value_map,
    std::string filename);
  static void add_index_to_csv(std::ofstream * csv_file);

  /* Diag*/
  void check_update_suggest(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /* Debug/Visualization functions */
  void publish_map(const Map & value_map, const std::string & publish_type);
  void publish_count_map();
  void publish_index();
  OccupancyGrid get_occ_msg(
    const std::string & frame_id, const double height, const double width, const double resolution,
    const std::vector<int8_t> & map_value);
  int nearest_value_index_search() const;

  enum GET_PITCH_METHOD { TF = 0, NONE = 1 };

  enum UPDATE_METHOD {
    UPDATE_OFFSET_EACH_CELL = 0,
    UPDATE_OFFSET_TOTAL = 1,
  };

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

public:
  explicit GenericValueCalibrator(const rclcpp::NodeOptions & node_options);
  ~GenericValueCalibrator();
};

}  // namespace autoware::generic_value_calibrator

#endif  // AUTOWARE_GENERIC_VALUE_CALIBRATOR__GENERIC_VALUE_CALIBRATOR_NODE_HPP_
