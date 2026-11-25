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
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRCAEDT;
using autoware::point_types::ReturnType;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
/** @brief Points of each ring, split by return type. */
struct Rings
{
  std::vector<std::vector<PointXYZIRCAEDT>> weak_first;
  std::vector<std::vector<PointXYZIRCAEDT>> others;
  size_t num_points_with_invalid_channel{0};
};

Rings split_into_rings(
  const pcl::PointCloud<PointXYZIRCAEDT>::VectorType & points, const uint32_t vertical_bins)
{
  Rings rings{
    std::vector<std::vector<PointXYZIRCAEDT>>(vertical_bins),
    std::vector<std::vector<PointXYZIRCAEDT>>(vertical_bins), 0};
  for (const auto & p : points) {
    if (p.channel >= vertical_bins) {
      ++rings.num_points_with_invalid_channel;
      continue;
    }
    auto & target = p.return_type == ReturnType::DUAL_WEAK_FIRST ? rings.weak_first : rings.others;
    target[p.channel].push_back(p);
  }
  return rings;
}

/** @brief The kept points and the removed (noise) points. */
struct FilteredClouds
{
  pcl::PointCloud<PointXYZIRCAEDT> kept;
  pcl::PointCloud<PointXYZIRCAEDT> noise;
};

/** @brief Appends each noise point of the ring to clouds.noise, and the others to clouds.kept. */
void append_by_noise_flag(
  const std::vector<PointXYZIRCAEDT> & ring, const std::vector<bool> & is_noise,
  FilteredClouds & clouds)
{
  for (size_t i = 0; i < ring.size(); ++i) {
    auto & target = is_noise[i] ? clouds.noise : clouds.kept;
    target.points.push_back(ring[i]);
  }
}

/** @brief Region where the weak first returns are counted for the visibility estimation. */
struct VisibilityRoi
{
  uint8_t mode;  // 0: whole cloud, 1: base_link xyz-ROI, 2: azimuth-ROI in the LiDAR frame
  dual_return_outlier_filter::XyzRoi xyz;
  const dual_return_outlier_filter::AzimuthBins & azimuth_bins;
  float max_distance;

  bool operator()(const PointXYZIRCAEDT & p) const
  {
    if (mode == 1) {
      return xyz.contains(p);
    }
    if (mode == 2) {
      return azimuth_bins.contains(p.azimuth) && p.distance < max_distance;
    }
    return true;
  }
};

/** @brief Writes the noise counts of one ring to its row of the frequency image. */
void write_frequency_row(
  cv::Mat & frequency_image, const uint32_t ring_id, const std::vector<int> & noise_frequency)
{
  for (size_t bin = 0; bin < noise_frequency.size(); ++bin) {
    frequency_image.at<uchar>(static_cast<int>(ring_id), static_cast<int>(bin)) =
      static_cast<uchar>(std::min(noise_frequency[bin], 255));
  }
}

/** @brief Applies a ring outlier filter to the non weak first returns. */
void filter_other_rings(
  const std::vector<std::vector<PointXYZIRCAEDT>> & rings, const double distance_ratio,
  const float max_azimuth_diff, FilteredClouds & clouds)
{
  for (const auto & ring : rings) {
    auto is_noise =
      dual_return_outlier_filter::find_segment_points(ring, distance_ratio, max_azimuth_diff);
    is_noise.flip();
    append_by_noise_flag(ring, is_noise, clouds);
  }
}
}  // namespace

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

  if (roi_mode_ == "Fixed_azimuth_ROI" && min_azimuth_deg_ >= max_azimuth_deg_) {
    // An ROI crossing 0 deg is not supported
    throw std::invalid_argument("min_azimuth_deg must be less than max_azimuth_deg.");
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

  const auto vertical_bins = static_cast<uint32_t>(vertical_bins_);
  const auto roi_mode = roi_mode_map_[roi_mode_];
  const auto azimuth_bins = make_azimuth_bins(roi_mode);
  const dual_return_outlier_filter::WeakFirstParams weak_first_params{
    weak_first_distance_ratio_, max_azimuth_diff_ * static_cast<float>(M_PI / 180.0),
    weak_first_local_noise_threshold_};

  const VisibilityRoi is_in_roi{
    roi_mode, {{x_min_, x_max_}, {y_min_, y_max_}, {z_min_, z_max_}}, azimuth_bins, max_distance_};

  const auto rings = split_into_rings(pcl_input->points, vertical_bins);
  if (rings.num_points_with_invalid_channel > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Skipped %zu points whose channel is not less than vertical_bins (%u).",
      rings.num_points_with_invalid_channel, vertical_bins);
  }

  FilteredClouds clouds;
  clouds.kept.points.reserve(pcl_input->points.size());
  clouds.noise.points.reserve(pcl_input->points.size());

  // Weak first returns: isolated points are noise, and so are the segment points in noisy bins.
  // The noise counts of each ring and bin make the frequency image.
  cv::Mat frequency_image(
    cv::Size(static_cast<int>(azimuth_bins.size()), static_cast<int>(vertical_bins)), CV_8UC1,
    cv::Scalar(0));
  for (uint32_t ring_id = 0; ring_id < vertical_bins; ++ring_id) {
    const auto & ring = rings.weak_first[ring_id];
    const auto result = dual_return_outlier_filter::classify_weak_first_ring(
      ring, azimuth_bins, weak_first_params, is_in_roi);
    append_by_noise_flag(ring, result.is_noise, clouds);
    write_frequency_row(frequency_image, ring_id, result.noise_frequency);
  }

  // Other returns: a ring outlier filter, without the visibility estimation
  filter_other_rings(
    rings.others, general_distance_ratio_, weak_first_params.max_azimuth_diff, clouds);

  publish_visibility(frequency_image, input->header);

  // Publish noise points
  sensor_msgs::msg::PointCloud2 noise_output_msg;
  pcl::toROSMsg(clouds.noise, noise_output_msg);
  noise_output_msg.header = input->header;
  noise_cloud_pub_->publish(noise_output_msg);

  // Publish filtered pointcloud
  pcl::toROSMsg(clouds.kept, output);
  output.header = input->header;
}

dual_return_outlier_filter::AzimuthBins DualReturnOutlierFilterComponent::make_azimuth_bins(
  const uint8_t roi_mode) const
{
  constexpr std::size_t horizontal_bins = 36;
  if (roi_mode == 2) {
    return {
      min_azimuth_deg_ * static_cast<float>(M_PI / 180.0),
      max_azimuth_deg_ * static_cast<float>(M_PI / 180.0), horizontal_bins};
  }
  return {0.0F, static_cast<float>(2.0 * M_PI), horizontal_bins};
}

void DualReturnOutlierFilterComponent::publish_visibility(
  const cv::Mat & frequency_image, const std_msgs::msg::Header & header)
{
  // A pixel is filled when its noise count reaches the threshold (tunable)
  cv::Mat binary_image;
  cv::inRange(frequency_image, weak_first_local_noise_threshold_, 255, binary_image);
  const int num_pixels = cv::countNonZero(binary_image);
  const float filled = static_cast<float>(num_pixels) / static_cast<float>(frequency_image.total());
  visibility_ = 1.0f - filled;

  // Visualization of histogram. Multiply bins by four to get pretty colours
  cv::Mat frequency_image_colorized;
  cv::applyColorMap(frequency_image * 4, frequency_image_colorized, cv::COLORMAP_JET);
  sensor_msgs::msg::Image::SharedPtr frequency_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frequency_image_colorized).toImageMsg();
  frequency_image_msg->header = header;
  image_pub_.publish(frequency_image_msg);

  autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
  visibility_msg.data = (1.0f - filled);
  visibility_msg.stamp = now();
  visibility_pub_->publish(visibility_msg);
}

rcl_interfaces::msg::SetParametersResult DualReturnOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  get_param(p, "weak_first_distance_ratio", weak_first_distance_ratio_);
  get_param(p, "general_distance_ratio", general_distance_ratio_);
  get_param(p, "weak_first_local_noise_threshold", weak_first_local_noise_threshold_);
  get_param(p, "vertical_bins", vertical_bins_);
  get_param(p, "max_azimuth_diff", max_azimuth_diff_);
  RCLCPP_DEBUG(
    get_logger(),
    "Parameters: weak_first_distance_ratio=%f, general_distance_ratio=%f, "
    "weak_first_local_noise_threshold=%d, vertical_bins=%d, max_azimuth_diff=%f.",
    weak_first_distance_ratio_, general_distance_ratio_, weak_first_local_noise_threshold_,
    vertical_bins_, max_azimuth_diff_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent)
