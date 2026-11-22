// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_node.hpp"

#include "autoware/point_types/types.hpp"
#include "autoware/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_utils.hpp"

#include <std_msgs/msg/header.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRCAEDT;
using autoware::point_types::ReturnType;
using diagnostic_msgs::msg::DiagnosticStatus;

DualReturnOutlierFilterComponent::DualReturnOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("DualReturnOutlierFilter", options)
{
  // set initial parameters
  {
    x_max_ = declare_parameter<float>("x_max");
    x_min_ = declare_parameter<float>("x_min");
    y_max_ = declare_parameter<float>("y_max");
    y_min_ = declare_parameter<float>("y_min");
    z_max_ = declare_parameter<float>("z_max");
    z_min_ = declare_parameter<float>("z_min");
    min_azimuth_deg_ = declare_parameter<float>("min_azimuth_deg");
    max_azimuth_deg_ = declare_parameter<float>("max_azimuth_deg");
    max_distance_ = declare_parameter<float>("max_distance");
    vertical_bins_ = declare_parameter<int>("vertical_bins");
    max_azimuth_diff_ = declare_parameter<float>("max_azimuth_diff");
    weak_first_distance_ratio_ = declare_parameter<double>("weak_first_distance_ratio");
    general_distance_ratio_ = declare_parameter<double>("general_distance_ratio");

    weak_first_local_noise_threshold_ = declare_parameter<int>("weak_first_local_noise_threshold");
    roi_mode_ = declare_parameter<std::string>("roi_mode");
    visibility_error_threshold_ = declare_parameter<double>("visibility_error_threshold");
    visibility_warn_threshold_ = declare_parameter<double>("visibility_warn_threshold");
  }
  updater_.setHardwareID("dual_return_outlier_filter");
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &DualReturnOutlierFilterComponent::onVisibilityChecker);
  updater_.setPeriod(0.1);

  image_pub_ =
    image_transport::create_publisher(this, "dual_return_outlier_filter/debug/frequency_image");
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "dual_return_outlier_filter/debug/visibility", rclcpp::SensorDataQoS());
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    noise_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "dual_return_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(), pub_options);
  }
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DualReturnOutlierFilterComponent::param_callback, this, _1));
}

void DualReturnOutlierFilterComponent::onVisibilityChecker(DiagnosticStatusWrapper & stat)
{
  // Add values
  stat.add("value", std::to_string(visibility_));

  auto level = DiagnosticStatus::OK;
  std::string msg = "OK";
  if (visibility_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (visibility_ < visibility_error_threshold_) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR: low visibility in dual outlier filter";
  } else if (visibility_ < visibility_warn_threshold_) {
    level = DiagnosticStatus::WARN;
    msg = "WARNING: low visibility in dual outlier filter";
  }
  stat.summary(level, msg);
}

void DualReturnOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  pcl::PointCloud<PointXYZIRCAEDT>::Ptr pcl_input(new pcl::PointCloud<PointXYZIRCAEDT>);
  pcl::fromROSMsg(*input, *pcl_input);

  uint32_t vertical_bins = vertical_bins_;
  uint32_t horizontal_bins = 36;
  float max_azimuth = 2 * M_PI;
  float min_azimuth = 0.0f;
  switch (roi_mode_map_[roi_mode_]) {
    case 2: {
      max_azimuth = max_azimuth_deg_ * (M_PI / 180.0);
      min_azimuth = min_azimuth_deg_ * (M_PI / 180.0);
      break;
    }

    default: {
      max_azimuth = 2 * M_PI;
      min_azimuth = 0.0f;
      break;
    }
  }

  const dual_return_outlier_filter::AzimuthBins azimuth_bins(
    min_azimuth, max_azimuth, horizontal_bins);
  const float max_azimuth_diff = max_azimuth_diff_ * static_cast<float>(M_PI / 180.0);

  // Region where the weak first returns are counted for the visibility estimation
  const auto roi_mode = roi_mode_map_[roi_mode_];
  const auto is_in_roi = [&](const PointXYZIRCAEDT & p) {
    switch (roi_mode) {
      case 1:  // base_link xyz-ROI
        return p.x > x_min_ && p.x < x_max_ && p.y > y_min_ && p.y < y_max_ && p.z > z_min_ &&
               p.z < z_max_;
      case 2:  // azimuth-ROI in the LiDAR frame
        return azimuth_bins.contains(p.azimuth) && p.distance < max_distance_;
      default:
        return true;
    }
  };

  pcl::PointCloud<PointXYZIRCAEDT>::Ptr pcl_output(new pcl::PointCloud<PointXYZIRCAEDT>);
  pcl_output->points.reserve(pcl_input->points.size());

  pcl::PointCloud<PointXYZIRCAEDT>::Ptr noise_output(new pcl::PointCloud<PointXYZIRCAEDT>);
  noise_output->points.reserve(pcl_input->points.size());

  std::vector<std::vector<PointXYZIRCAEDT>> input_rings(vertical_bins);
  std::vector<std::vector<PointXYZIRCAEDT>> weak_first_rings(vertical_bins);
  size_t num_points_with_invalid_ring = 0;
  for (const auto & p : pcl_input->points) {
    if (p.channel >= vertical_bins) {
      ++num_points_with_invalid_ring;
      continue;
    }
    if (p.return_type == ReturnType::DUAL_WEAK_FIRST) {
      weak_first_rings[p.channel].push_back(p);
    } else {
      input_rings[p.channel].push_back(p);
    }
  }
  if (num_points_with_invalid_ring > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipped %zu points whose channel is not less than vertical_bins (%u).",
      num_points_with_invalid_ring, vertical_bins);
  }

  cv::Mat frequency_image(cv::Size(horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  // Weak first returns: points that are not part of a segment are noise. A horizontal bin with
  // at least weak_first_local_noise_threshold noise points in the ROI is noisy, and the segment
  // points in it are removed as well. Bins are computed from each point's own azimuth, so the
  // result does not depend on the order of the points within a ring.
  std::vector<int> noise_frequency(horizontal_bins);
  for (uint32_t ring_id = 0; ring_id < vertical_bins; ++ring_id) {
    const auto & ring = weak_first_rings[ring_id];
    if (ring.empty()) {
      continue;
    }
    const auto in_segment = dual_return_outlier_filter::find_segment_points(
      ring, weak_first_distance_ratio_, max_azimuth_diff);

    std::fill(noise_frequency.begin(), noise_frequency.end(), 0);
    for (size_t i = 0; i < ring.size(); ++i) {
      if (in_segment[i] || !is_in_roi(ring[i])) {
        continue;
      }
      if (const auto bin = azimuth_bins.bin(ring[i].azimuth)) {
        ++noise_frequency[*bin];
      }
    }
    // The decision uses the counts of the isolated points only, so it does not depend on the
    // order in which the segment points are visited.
    const std::vector<int> isolated_noise_frequency = noise_frequency;

    for (size_t i = 0; i < ring.size(); ++i) {
      const auto & p = ring[i];
      if (!in_segment[i]) {
        noise_output->points.push_back(p);
        continue;
      }
      const auto bin = azimuth_bins.bin(p.azimuth);
      if (bin && isolated_noise_frequency[*bin] >= weak_first_local_noise_threshold_) {
        noise_output->points.push_back(p);
        if (is_in_roi(p)) {
          ++noise_frequency[*bin];
        }
      } else {
        pcl_output->points.push_back(p);
      }
    }

    for (uint32_t bin = 0; bin < horizontal_bins; ++bin) {
      frequency_image.at<uchar>(ring_id, bin) =
        static_cast<uchar>(std::min(noise_frequency[bin], 255));
    }
  }

  // Other returns: a ring outlier filter, without the visibility estimation
  for (const auto & ring : input_rings) {
    const auto in_segment = dual_return_outlier_filter::find_segment_points(
      ring, general_distance_ratio_, max_azimuth_diff);
    for (size_t i = 0; i < ring.size(); ++i) {
      if (in_segment[i]) {
        pcl_output->points.push_back(ring[i]);
      } else {
        noise_output->points.push_back(ring[i]);
      }
    }
  }

  // Threshold for diagnostics (tunable)
  cv::Mat binary_image;
  cv::inRange(frequency_image, weak_first_local_noise_threshold_, 255, binary_image);
  int num_pixels = cv::countNonZero(binary_image);
  float filled =
    static_cast<float>(num_pixels) / static_cast<float>(vertical_bins * horizontal_bins);
  visibility_ = 1.0f - filled;
  // Visualization of histogram
  cv::Mat frequency_image_colorized;
  // Multiply bins by four to get pretty colours
  cv::applyColorMap(frequency_image * 4, frequency_image_colorized, cv::COLORMAP_JET);
  sensor_msgs::msg::Image::SharedPtr frequency_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frequency_image_colorized).toImageMsg();
  frequency_image_msg->header = input->header;
  // Publish histogram image
  image_pub_.publish(frequency_image_msg);
  autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
  visibility_msg.data = (1.0f - filled);
  visibility_msg.stamp = now();
  visibility_pub_->publish(visibility_msg);

  // Publish noise points
  sensor_msgs::msg::PointCloud2 noise_output_msg;
  pcl::toROSMsg(*noise_output, noise_output_msg);
  noise_output_msg.header = input->header;
  noise_cloud_pub_->publish(noise_output_msg);

  // Publish filtered pointcloud
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult DualReturnOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "weak_first_distance_ratio", weak_first_distance_ratio_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new weak first distance ratio to: %f.", weak_first_distance_ratio_);
  }
  if (get_param(p, "general_distance_ratio", general_distance_ratio_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new general distance ratio to: %f.", general_distance_ratio_);
  }
  if (get_param(p, "weak_first_local_noise_threshold", weak_first_local_noise_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new weak first local noise threshold to: %d.",
      weak_first_local_noise_threshold_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "max_azimuth_diff", max_azimuth_diff_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max_azimuth_diff to: %f.", max_azimuth_diff_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent)
