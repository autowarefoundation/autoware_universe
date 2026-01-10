// Copyright 2020 Tier IV, Inc.
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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__FILTER_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__FILTER_HPP_

#include "autoware/pointcloud_preprocessor/transform_info.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/model_coefficients.h>
#include <pcl_msgs/msg/point_indices.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Autoware utils
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <managed_transform_buffer/managed_transform_buffer.hpp>

namespace autoware::pointcloud_preprocessor
{
namespace sync_policies = message_filters::sync_policies;

/** \brief For parameter service callback */
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

/** \brief @b Filter represents the base filter class. Some generic 3D operations that are
 * applicable to all filters are defined here as static methods. \author Radu Bogdan Rusu
 */
class Filter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  using PointIndices = pcl_msgs::msg::PointIndices;
  using PointIndicesPtr = PointIndices::SharedPtr;
  using PointIndicesConstPtr = PointIndices::ConstSharedPtr;

  using ModelCoefficients = pcl_msgs::msg::ModelCoefficients;
  using ModelCoefficientsPtr = ModelCoefficients::SharedPtr;
  using ModelCoefficientsConstPtr = ModelCoefficients::ConstSharedPtr;

  using IndicesPtr = pcl::IndicesPtr;
  using IndicesConstPtr = pcl::IndicesConstPtr;

  using ExactTimeSyncPolicy =
    message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices>>;
  using ApproximateTimeSyncPolicy =
    message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices>>;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit Filter(
    const std::string & filter_name = "pointcloud_preprocessor_filter",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  message_filters::Subscriber<PointIndices> sub_indices_filter_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a
   * filter_limit_max_). Default: false. */
  bool filter_limit_negative_;

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_output_frame_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief The diagnostic message */
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_;

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  /** \brief Virtual abstract filter method. To be implemented by every child.
   * \param input the input point cloud dataset.
   * \param indices a pointer to the vector of point indices to use.
   * \param output the resultant filtered PointCloud2
   */
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) = 0;

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API. It's not pure virtual function so that a child class does not have to implement
  // this function.
  virtual void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
    const TransformInfo & transform_info);  // != 0

  /** \brief Lazy transport subscribe routine. */
  virtual void subscribe(const std::string & filter_name);
  virtual void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  virtual void unsubscribe();

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input the input point cloud dataset.
   * \param indices a pointer to the vector of point indices to use.
   */
  virtual void compute_publish(const PointCloud2ConstPtr & input, const IndicesPtr & indices);
  /** \brief PointCloud2 + Indices data callback. */
  virtual void input_indices_callback(
    const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices);
  virtual bool convert_output_costly(std::unique_ptr<PointCloud2> & output);

  //////////////////////
  // from PCLNodelet //
  //////////////////////
  /** \brief Set to true if point indices are used.
   *
   * When receiving a point cloud, if use_indices_ is false, the entire
   * point cloud is processed for the given operation. If use_indices_ is
   * true, then the ~indices topic is read to get the vector of point
   * indices specifying the subset of the point cloud that will be used for
   * the operation. In the case where use_indices_ is true, the ~input and
   * ~indices topics must be synchronised in time, either exact or within a
   * specified jitter. See also @ref latched_indices_ and approximate_sync.
   **/
  bool use_indices_ = false;
  /** \brief Set to true if the indices topic is latched.
   *
   * If use_indices_ is true, the ~input and ~indices topics generally must
   * be synchronised in time. By setting this flag to true, the most recent
   * value from ~indices can be used instead of requiring a synchronised
   * message.
   **/
  bool latched_indices_ = false;

  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;

  /** \brief True if we use an approximate time synchronizer
   * versus an exact one (false by default). */
  bool approximate_sync_ = false;

  std::unique_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_{nullptr};

  /**
   * @brief Validate a sensor_msgs::msg::PointCloud2 message for structural consistency and layout.
   *
   * This function performs a series of lightweight sanity checks to determine whether a
   * PointCloud2 message is safe to consume by algorithms expecting an XYZ-compatible,
   * organized (dense) point cloud.
   *
   * Validation errors and warnings are reported using throttled ROS logging to avoid
   * excessive log spam when invalid data is repeatedly received.
   *
   * @details
   * The following checks are performed in order:
   *
   * 1. **Null pointer check**
   *    - Ensures the input PointCloud2 pointer is valid.
   *
   * 2. **Point step validation**
   *    - Verifies that `point_step` is non-zero.
   *
   * 3. **Dense (organized) cloud requirement**
   *    - Warns if `is_dense` is false.
   *    - Currently does *not* fail validation, but will become a hard error
   *      starting **July 2026**.
   *
   * 4. **Point field layout compatibility**
   *    - Ensures the point data layout is compatible with `PointXYZ`
   *      (fields must begin with `x`, `y`, `z` of type `FLOAT32`).
   *
   * 5. **Row step consistency**
   *    - Validates that `row_step == width * point_step`.
   *    - Currently logs a warning on mismatch.
   *    - Will become a hard error starting **July 2026**.
   *
   * 6. **Data buffer size consistency**
   *    - Ensures `data.size() == height * row_step`.
   *
   * @param cloud
   *   Shared pointer to the input PointCloud2 message to validate.
   *
   * @param logger
   *   ROS 2 logger used for emitting validation warnings.
   *
   * @param clock
   *   ROS 2 clock used for throttled logging.
   *
   * @return true
   *   If the point cloud passes all mandatory validation checks.
   *
   * @return false
   *   If a critical validation failure is detected (e.g., null pointer,
   *   incompatible data layout, or inconsistent buffer size).
   *
   * @note
   * - All warnings are throttled with a fixed duration of 5000 ms.
   * - Some warnings are marked as *future errors* and are expected to
   *   become fatal after July 2026.
   *
   * @warning
   * This function assumes consumers expect an XYZ-only point layout.
   * Additional fields (e.g., intensity, color) are allowed only if they
   * appear *after* the XYZ fields and do not alter the layout compatibility.
   */
  static bool is_valid(
    const PointCloud2ConstPtr & cloud, const rclcpp::Logger & logger, rclcpp::Clock & clock)
  {
    static constexpr rcutils_duration_value_t throttle_duration_ms = 5000;

    if (!cloud) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_duration_ms, "Invalid PointCloud: Null pointer received.");
      return false;
    }

    // Check: Point Step
    if (cloud->point_step == 0) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_duration_ms,
        "Invalid PointCloud: point_step is 0. "
        "Frame: '%s', Stamp: %d.%09u",
        cloud->header.frame_id.c_str(), cloud->header.stamp.sec, cloud->header.stamp.nanosec);
      return false;
    }

    // Check: Only accept organized (dense) point clouds
    if (!cloud->is_dense) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_duration_ms,
        "Invalid PointCloud: is_dense is false. "
        "The point cloud should be organized (dense) and contain only valid points. "
        "This will be an ERROR starting in 2026 July"
        "Frame: '%s', Stamp: %d.%09u",
        cloud->header.frame_id.c_str(), cloud->header.stamp.sec, cloud->header.stamp.nanosec);
      // TODO(mfc): return false; After 2026 July
    }

    // Check: Point field layout compatibility
    if (!utils::is_data_layout_compatible_with_point_xyz(*cloud)) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_duration_ms,
        "The pointcloud layout is not compatible with PointXYZ. Aborting. "
        "Make sure Point fields start with x, y, z as FLOAT32.");
      return false;
    }

    // Check: Row step consistency
    size_t expected_row_step =
      static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->point_step);

    if (expected_row_step != static_cast<size_t>(cloud->row_step)) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_duration_ms,
        "Invalid PointCloud: row_step mismatch. "
        "Expected: %zu (width %u * point_step %u), Got: %u. "
        "Frame: '%s', Stamp: %d.%09u"
        " Please fill in the `cloud->row_step` field accordingly."
        " This will be an ERROR starting in 2026 July",
        expected_row_step, cloud->width, cloud->point_step, cloud->row_step,
        cloud->header.frame_id.c_str(), cloud->header.stamp.sec, cloud->header.stamp.nanosec);
      // TODO(mfc): return false; After 2026 July
    }

    // Check: Data buffer size consistency
    size_t expected_data_size = static_cast<size_t>(cloud->height) * expected_row_step;

    if (expected_data_size != cloud->data.size()) {
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_duration_ms,
        "Invalid PointCloud: data size mismatch. "
        "Expected: %zu (height %u * row_step %u), Got: %zu. "
        "Frame: '%s', Stamp: %d.%09u",
        expected_data_size, cloud->height, cloud->row_step, cloud->data.size(),
        cloud->header.frame_id.c_str(), cloud->header.stamp.sec, cloud->header.stamp.nanosec);
      return false;
    }

    return true;
  }

  /// @brief Validates that point indices are non-null and within the bounds of the cloud size.
  /// @param indices Pointer to the indices to check.
  /// @param cloud_size The total number of points in the target cloud.
  /// @param logger Logger for outputting warnings on failure.
  /// @return True if indices are valid (or empty); false if null or out of bounds.
  static bool is_valid_indices(
    const PointIndicesConstPtr & indices, size_t cloud_size, const rclcpp::Logger & logger)
  {
    if (!indices) {
      RCLCPP_WARN(logger, "PointIndices pointer is null");
      return false;
    }

    // Valid special case: empty indices
    if (indices->indices.empty()) {
      return true;
    }

    // Assumption: Indices represent a subset/filter.
    if (indices->indices.size() > cloud_size) {
      RCLCPP_WARN(
        logger, "Received %zu indices for a cloud with %zu points", indices->indices.size(),
        cloud_size);
      return false;
    }

    for (const std::int32_t idx : indices->indices) {
      // Optimization: Hint to branch predictor that invalid indices are rare.
      // TODO(mfc): Replace __builtin_expect with [[unlikely]] when upgrading to C++20.
      if (__builtin_expect(idx < 0 || static_cast<size_t>(idx) >= cloud_size, 0)) {
        RCLCPP_WARN(logger, "Invalid point index detected: %d (cloud size: %zu)", idx, cloud_size);
        return false;
      }
    }

    return true;
  }

private:
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_filter_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult filter_param_callback(
    const std::vector<rclcpp::Parameter> & p);

  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<ExactTimeSyncPolicy> sync_input_indices_e_;
  std::shared_ptr<ApproximateTimeSyncPolicy> sync_input_indices_a_;

  /** \brief Get a matrix for conversion from the original frame to the target frame */
  bool calculate_transform_matrix(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
    TransformInfo & transform_info /*output*/);

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API.
  void faster_input_indices_callback(
    const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices);

  void setup_tf();
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__FILTER_HPP_
