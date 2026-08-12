// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CAMERA_LIDAR_INFO_COLLECTOR_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CAMERA_LIDAR_INFO_COLLECTOR_HPP_

#include "autoware/calibration_status_classifier/data_type.hpp"
#include "autoware/calibration_status_classifier/data_type_eigen.hpp"

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rmw/qos_profiles.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

/**
 * @class CameraLidarInfoCollector
 * @brief Utility class that collects camera and lidar information from multiple topics
 *
 * The CameraLidarInfoCollector class subscribes to multiple camera_info, camera image, and lidar
 * point cloud topics. It provides methods to retrieve the most recent sensor information and
 * transformations between camera and lidar frames. It uses a non-executing callback group to
 * enable manual polling of messages without automatic processing.
 */
class CameraLidarInfoCollector
{
  constexpr static auto call_delay = 100ms;
  constexpr static auto waitset_timeout = 5s;

public:
  /**
   * @brief Constructor
   * @param node Pointer to the ROS node
   * @param camera_lidar_in_out_info List of camera and lidar topic information to subscribe to
   *
   * Creates a collector that will subscribe to the specified camera and lidar topics
   * using a non-executing callback group to enable manual polling of messages.
   */
  explicit CameraLidarInfoCollector(
    rclcpp::Node * const node, const std::vector<CameraLidarTopicsInfo> & camera_lidar_in_out_info)
  : node_(node),
    tf_buffer_(node->get_clock()),
    tf_listener_(tf_buffer_),
    camera_lidar_in_out_info_(camera_lidar_in_out_info),
    info_size_(camera_lidar_in_out_info.size())
  {
    // Create the callback group once
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    // Create and store all subscriptions at construction time
    for (const auto & info : camera_lidar_in_out_info_) {
      std::function<void(sensor_msgs::msg::CameraInfo::ConstSharedPtr)> dummy_camera_info_cb =
        []([[maybe_unused]] const auto & msg) { assert(false); };
      camera_info_subs_.push_back(node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        info.camera_info_topic, rclcpp::SensorDataQoS(), dummy_camera_info_cb, options));

      std::function<void(sensor_msgs::msg::PointCloud2::ConstSharedPtr)> dummy_lidar_cb =
        []([[maybe_unused]] const auto & msg) { assert(false); };
      lidar_subs_.push_back(node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        info.lidar_topic, rclcpp::SensorDataQoS(), dummy_lidar_cb, options));

      std::function<void(sensor_msgs::msg::Image::ConstSharedPtr)> dummy_camera_cb =
        []([[maybe_unused]] const auto & msg) { assert(false); };
      camera_subs_.push_back(node_->create_subscription<sensor_msgs::msg::Image>(
        info.camera_topic, rclcpp::SensorDataQoS(), dummy_camera_cb, options));
    }
  }

  /**
   * @brief Retrieves camera and lidar information and transformations
   * @return Vector of CameraLidarInfo containing camera parameters, lidar/camera frame IDs, and
   * transformations
   *
   * This method collects information from all subscribed camera and lidar topics, including
   * camera intrinsics, distortion parameters, frame IDs, and transformations between
   * camera and lidar frames. It blocks until all required information is received.
   */
  std::vector<CameraLidarInfo> get_cameras_lidars_info()
  {
    std::vector<CameraLidarInfo> cameras_lidars_info(info_size_);
    auto cameras_info = get_cameras_info();
    auto cameras_lidars_frame_id = get_lidar_camera_frames_id();
    auto cameras_lidars_tf = get_cameras_lidars_tf(cameras_lidars_frame_id);

    for (std::size_t i = 0; i < cameras_info.size(); ++i) {
      cameras_lidars_info.at(i).height = static_cast<std::size_t>(cameras_info.at(i).height);
      cameras_lidars_info.at(i).width = static_cast<std::size_t>(cameras_info.at(i).width);
      auto d_to_copy = std::min(dist_coeffs_size, cameras_info.at(i).d.size());
      std::copy_n(cameras_info.at(i).d.begin(), d_to_copy, cameras_lidars_info.at(i).d.begin());
      cameras_lidars_info.at(i).k =
        Eigen::Map<const decltype(CameraLidarInfo::k)>(cameras_info.at(i).k.data());
      cameras_lidars_info.at(i).r =
        Eigen::Map<const decltype(CameraLidarInfo::r)>(cameras_info.at(i).r.data());
      cameras_lidars_info.at(i).p =
        Eigen::Map<const decltype(CameraLidarInfo::p)>(cameras_info.at(i).p.data());
      cameras_lidars_info.at(i).lidar_frame_id = cameras_lidars_frame_id.at(i).first;
      cameras_lidars_info.at(i).camera_frame_id = cameras_lidars_frame_id.at(i).second;
      cameras_lidars_info.at(i).tf_camera_to_lidar = cameras_lidars_tf.at(i);
      cameras_lidars_info.at(i).to_undistort = !camera_lidar_in_out_info_.at(i).already_rectified;
    }

    return cameras_lidars_info;
  }

private:
  /**
   * @brief Attempts to take a single message from a persistent subscription.
   * @tparam MsgT The message type.
   * @param sub The subscription to take from.
   * @return Optional containing the message if available, or empty otherwise.
   */
  template <typename MsgT>
  std::optional<MsgT> take_message(const typename rclcpp::Subscription<MsgT>::SharedPtr & sub)
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub);
    auto ret = wait_set.wait(waitset_timeout);

    if (ret.kind() == rclcpp::WaitResultKind::Ready) {
      MsgT msg;
      rclcpp::MessageInfo msg_info;
      if (sub->take(msg, msg_info)) {
        wait_set.remove_subscription(sub);
        return msg;
      }
    }
    wait_set.remove_subscription(sub);
    RCLCPP_WARN(
      node_->get_logger(), "No message received on topic %s within timeout %lf s",
      sub->get_topic_name(), std::chrono::duration<double>(waitset_timeout).count());
    return std::nullopt;
  }

  /**
   * @brief Retrieves camera information from all subscribed topics
   * @return Vector of CameraInfo messages, one for each subscribed topic
   */
  std::vector<sensor_msgs::msg::CameraInfo> get_cameras_info()
  {
    std::vector<sensor_msgs::msg::CameraInfo> camera_infos;
    std::size_t idx{0};

    while (camera_infos.size() < info_size_) {
      if (
        auto camera_info = take_message<sensor_msgs::msg::CameraInfo>(camera_info_subs_.at(idx))) {
        camera_infos.push_back(*camera_info);
        ++idx;
      } else {
        rclcpp::sleep_for(call_delay);
      }
    }
    return camera_infos;
  }

  /**
   * @brief Retrieves frame IDs for all lidar and camera pairs
   * @return Vector of pairs containing lidar and camera frame IDs
   */
  std::vector<std::pair<std::string, std::string>> get_lidar_camera_frames_id()
  {
    std::vector<std::pair<std::string, std::string>> frames_id;
    std::size_t idx{0};
    while (frames_id.size() < info_size_) {
      auto lidar_msg = take_message<sensor_msgs::msg::PointCloud2>(lidar_subs_.at(idx));
      auto camera_msg = take_message<sensor_msgs::msg::Image>(camera_subs_.at(idx));

      if (lidar_msg && camera_msg) {
        frames_id.emplace_back(lidar_msg->header.frame_id, camera_msg->header.frame_id);
        ++idx;
      } else {
        rclcpp::sleep_for(call_delay);
      }
    }
    return frames_id;
  }

  /**
   * @brief Gets the transform from camera to lidar frame
   * @param camera_frame_id The camera frame ID (target frame)
   * @param lidar_frame_id The lidar frame ID (source frame)
   * @return Optional containing the transform if available, or empty if no transform is available
   */
  std::optional<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> get_camera_lidar_tf(
    const std::string & camera_frame_id, const std::string & lidar_frame_id)
  {
    try {
      auto transform =
        tf_buffer_.lookupTransform(camera_frame_id, lidar_frame_id, tf2::TimePointZero);
      auto eigen_matrix = tf2::transformToEigen(transform).matrix();
      return Eigen::Matrix<double, 4, 4, Eigen::RowMajor>(eigen_matrix);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "Could not get transform from %s to %s: %s",
        camera_frame_id.c_str(), lidar_frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }

  /**
   * @brief Gets transforms for all lidar-camera pairs
   * @param frames_id_pairs Vector of pairs containing lidar and camera frame IDs
   * @return Vector of transforms, one for each lidar-camera pair
   */
  std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> get_cameras_lidars_tf(
    const std::vector<std::pair<std::string, std::string>> & frames_id_pairs)
  {
    std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> tf_cameras_lidars;
    std::size_t idx{0};

    while (tf_cameras_lidars.size() < info_size_) {
      auto tf = get_camera_lidar_tf(frames_id_pairs.at(idx).second, frames_id_pairs.at(idx).first);
      if (tf) {
        tf_cameras_lidars.push_back(*tf);
        ++idx;
      } else {
        rclcpp::sleep_for(call_delay);
      }
    }
    return tf_cameras_lidars;
  }

  // Member variables
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Node * node_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  const std::vector<CameraLidarTopicsInfo> camera_lidar_in_out_info_;
  const std::size_t info_size_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_subs_;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CAMERA_LIDAR_INFO_COLLECTOR_HPP_
