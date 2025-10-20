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

#include "bbox_object_locator_node.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/region_of_interest.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// cspell: ignore Matx

namespace autoware::image_object_locator
{
using autoware::image_object_locator::grid_pixel_sampler::AdaptiveGridPixelSampler;
using autoware::image_object_locator::grid_pixel_sampler::FixedGridPixelSampler;
using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
using autoware_utils_math::deg2rad;

void transformToRT(const geometry_msgs::msg::TransformStamped & tf, cv::Matx33d & R, cv::Vec3d & t)
{
  tf2::Quaternion q(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);

  tf2::Matrix3x3 m(q);

  R = cv::Matx33d(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);
  t = cv::Vec3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
}

cv::Point2f undistortPoint(cv::Point2f pixel, cv::Matx33d K, cv::Mat D)
{
  cv::Point2f undistorted_pixel;
  std::vector<cv::Point2f> pixels = {pixel};
  std::vector<cv::Point2f> undistorted_pixels;

  cv::undistortPoints(pixels, undistorted_pixels, K, D);

  undistorted_pixel.x = undistorted_pixels[0].x;
  undistorted_pixel.y = undistorted_pixels[0].y;

  return undistorted_pixel;
}

/**
 * @brief Computes the ground intersection point of a pixel ray.
 *
 * Assumes that the ROI covers the entire object, and projects the given image point
 * onto the ground plane (z=0) in target frame coordinate.
 */
cv::Vec3d projectToGround(const cv::Point2f & pixel, const cv::Matx33d & R, const cv::Vec3d & t)
{
  cv::Vec3d ray_cam(pixel.x, pixel.y, 1.0);
  cv::Vec3d ray_world = R * ray_cam;
  cv::Vec3d cam_origin = t;

  // compute the scale factor (lambda) where the ray intersects the ground plane (z=0)
  const double lambda = -cam_origin[2] / ray_world[2];

  return cam_origin + lambda * ray_world;
}

inline cv::Vec3d projectUndistortedToGround(
  const cv::Point2f & px, const cv::Matx33d & K, const cv::Mat & D, const cv::Matx33d & R,
  const cv::Vec3d & t)
{
  return projectToGround(undistortPoint(px, K, D), R, t);
}

/**
 * @brief Computes the height of an ROI in 3D space.
 *
 * Uses the top pixel of the ROI to cast a ray into 3D, and estimates the height
 * by intersecting this ray with the (x, y) location of the projected bottom pixel.
 * Falls back to a pseudo height if no valid intersection is found.
 */
double computeHeight(
  const cv::Point2f & pixel_top, const cv::Matx33d & R, const cv::Vec3d & t,
  const cv::Vec3d & projected_bottom_pixel, const double pseudo_height)
{
  cv::Vec3d ray_cam(pixel_top.x, pixel_top.y, 1.0);
  cv::Vec3d ray_world = R * ray_cam;
  cv::Vec3d cam_origin = t;

  // search for the scale factor (lambda) at which the ray intersects
  // the (x, y) position of the projected bottom pixel
  const double lambda_x = (projected_bottom_pixel[0] - cam_origin[0]) / ray_world[0];
  const double lambda_y = (projected_bottom_pixel[1] - cam_origin[1]) / ray_world[1];

  double lambda;
  // handle cases where the ray points in the opposite direction (negative lambda)
  if (lambda_x >= 0) {
    if (lambda_y >= 0) {
      // average the two solutions for robustness
      // NOTE: a least-squares solution might be more accurate
      lambda = (lambda_x + lambda_y) / 2.0;
    } else {
      lambda = lambda_x;
    }
  } else {
    if (lambda_y >= 0) {
      lambda = lambda_y;
    } else {
      // if both lambdas are negative, fall back to the pseudo height
      lambda = pseudo_height;
    }
  }

  cv::Vec3d projected_top_point = cam_origin + lambda * ray_world;

  return projected_top_point[2];
}

// compute 2x2 covariance over (x,y) of the final object pose center
cv::Matx22d computeCovarianceXY(
  const cv::Point2f & bottom_center_px, const cv::Point2f & bottom_left_px,
  const cv::Point2f & bottom_right_px, const std::vector<cv::Point2f> & sampled_points_for_cov,
  const cv::Matx33d & K, const cv::Mat & D, const cv::Matx33d & R, const cv::Vec3d & t,
  uint8_t label, const cv::Vec3d & camera_optical_axis_world)
{
  const size_t sample_num = sampled_points_for_cov.size();

  if (sample_num < 2) return cv::Matx22d::eye() * 1e-6;

  // accumulators
  double mx = 0.0;
  double my = 0.0;

  // First pass: compute means of sampled (x,y)
  std::vector<cv::Vec2d> pos_xy;
  pos_xy.reserve(sample_num);
  for (size_t i = 0; i < sample_num; ++i) {
    cv::Vec3d pos = projectUndistortedToGround(sampled_points_for_cov[i], K, D, R, t);

    if (label == Label::PEDESTRIAN) {
      // apply same delta to left/right to keep footprint consistent with the footpoint
      const cv::Point2f diff = sampled_points_for_cov[i] - bottom_center_px;
      const cv::Point2f bl = bottom_left_px + diff;
      const cv::Point2f br = bottom_right_px + diff;

      // width for pedestrian bias
      const cv::Vec3d ped_left = projectUndistortedToGround(bl, K, D, R, t);
      const cv::Vec3d ped_right = projectUndistortedToGround(br, K, D, R, t);
      const double width = cv::norm(ped_left - ped_right);

      // bias toward optical axis by 0.5 * width
      cv::Vec3d bias = camera_optical_axis_world * (0.5 * width);
      pos += bias;
    }

    pos_xy.emplace_back(pos[0], pos[1]);
    mx += pos[0];
    my += pos[1];
  }

  mx /= static_cast<double>(sample_num);
  my /= static_cast<double>(sample_num);

  // unbiased covariance
  double cxx = 0.0;
  double cxy = 0.0;
  double cyy = 0.0;
  for (const auto & p : pos_xy) {
    const double dx = p[0] - mx;
    const double dy = p[1] - my;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }

  const double denom = static_cast<double>(sample_num - 1);
  cxx /= denom;
  cxy /= denom;
  cyy /= denom;

  return cv::Matx22d(cxx, cxy, cxy, cyy);
}

// initialize constructor
BboxObjectLocatorNode::BboxObjectLocatorNode(const rclcpp::NodeOptions & node_options)
: Node("roi_3d_projector", node_options),
  target_frame_(declare_parameter<std::string>("target_frame"))
{
  label_settings_.UNKNOWN = declare_parameter<bool>("detection_target_class.UNKNOWN");
  label_settings_.CAR = declare_parameter<bool>("detection_target_class.CAR");
  label_settings_.TRUCK = declare_parameter<bool>("detection_target_class.TRUCK");
  label_settings_.BUS = declare_parameter<bool>("detection_target_class.BUS");
  label_settings_.TRAILER = declare_parameter<bool>("detection_target_class.TRAILER");
  label_settings_.MOTORCYCLE = declare_parameter<bool>("detection_target_class.MOTORCYCLE");
  label_settings_.BICYCLE = declare_parameter<bool>("detection_target_class.BICYCLE");
  label_settings_.PEDESTRIAN = declare_parameter<bool>("detection_target_class.PEDESTRIAN");

  roi_confidence_th_ = declare_parameter<double>("roi_confidence_threshold");

  std::vector<bool> roi_truncation_validation_enable =
    declare_parameter<std::vector<bool>>("roi_truncation_validation.enabled");
  std::vector<bool> roi_truncation_validation_remove_truncated =
    declare_parameter<std::vector<bool>>("roi_truncation_validation.remove_truncated");
  double image_border_truncation_horizontal_margin_ratio = declare_parameter<double>(
    "roi_truncation_validation.image_border_truncation_horizontal_margin_ratio");
  double image_border_truncation_vertical_margin_ratio = declare_parameter<double>(
    "roi_truncation_validation.image_border_truncation_vertical_margin_ratio");

  const double detection_max_range = declare_parameter<double>("detection_max_range");
  detection_max_range_sq_ = detection_max_range * detection_max_range;
  pseudo_height_ = declare_parameter<double>("pseudo_height");

  pedestrian_width_min_ = declare_parameter<double>("pedestrian_detection_config.width_min");
  pedestrian_width_max_ = declare_parameter<double>("pedestrian_detection_config.width_max");

  std::vector<int64_t> rois_ids = declare_parameter<std::vector<int64_t>>("rois_ids");
  size_t rois_number = rois_ids.size();

  if (!isRoiValidationParamValid(
        rois_number, roi_truncation_validation_enable,
        roi_truncation_validation_remove_truncated)) {
    throw std::runtime_error("Parameter roi_truncation_validation is invalid.");
  }

  // create subscriber and publisher
  camera_info_subs_.resize(rois_number);
  roi_subs_.resize(rois_number);
  for (auto rois_id_index = 0u; rois_id_index < rois_number; ++rois_id_index) {
    const int rois_id = rois_ids[rois_id_index];
    const std::string rois_id_str = std::to_string(rois_id);

    is_camera_info_arrived_[rois_id] = false;

    // ROI validator initialization
    RoiValidator roi_validator{};
    roi_validator.enable_validation = roi_truncation_validation_enable[rois_id_index];
    roi_validator.remove_object_might_be_truncated =
      roi_truncation_validation_remove_truncated[rois_id_index];
    roi_validator.image_border_truncation_horizontal_margin_ratio =
      image_border_truncation_horizontal_margin_ratio;
    roi_validator.image_border_truncation_vertical_margin_ratio =
      image_border_truncation_vertical_margin_ratio;
    roi_validator_[rois_id] = roi_validator;

    // sampler to calculate covariance
    if (covariance_control_param_.use_adaptive_sampler) {
      pixel_sampler_[rois_id] = std::make_unique<AdaptiveGridPixelSampler>(
        covariance_control_param_.half_grid_size_, covariance_control_param_.bbox_fraction_);
    } else {
      pixel_sampler_[rois_id] = std::make_unique<FixedGridPixelSampler>(
        covariance_control_param_.half_grid_size_, covariance_control_param_.grid_cell_size_);
    }

    // subscriber: camera info
    const std::string camera_info_topic_name = declare_parameter<std::string>(
      "input/camera_info" + rois_id_str, "/sensing/camera/camera" + rois_id_str + "/camera_info");
    const auto camera_info_qos = rclcpp::QoS{1}.best_effort();

    camera_info_subs_[rois_id_index] = this->create_subscription<CameraInfo>(
      camera_info_topic_name, camera_info_qos,
      [this, rois_id](const CameraInfo::ConstSharedPtr msg) {
        this->cameraInfoCallback(msg, rois_id);
      });

    // subscriber: roi
    const std::string roi_topic_name = declare_parameter<std::string>(
      "input/rois" + rois_id_str, "/perception/object_recognition/detection/rois" + rois_id_str);
    const auto roi_qos = rclcpp::QoS{1}.best_effort();

    roi_subs_[rois_id_index] = this->create_subscription<DetectedObjectsWithFeature>(
      roi_topic_name, roi_qos,
      [this, rois_id](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
        this->roiCallback(msg, rois_id);
      });

    // publisher
    const std::string output_topic_name = declare_parameter<std::string>(
      "output/rois" + rois_id_str + "/objects", "~/rois" + rois_id_str + "/objects");
    objects_pubs_[rois_id] = this->create_publisher<DetectedObjects>(output_topic_name, 1);
  }

  transform_listener_ = std::make_shared<TransformListener>(this);
}

bool BboxObjectLocatorNode::isRoiValidationParamValid(
  const size_t rois_number, const std::vector<bool> & validation_enable,
  const std::vector<bool> & remove_truncated)
{
  if (validation_enable.size() < rois_number) {
    RCLCPP_ERROR(
      get_logger(),
      "roi_truncation_validation.enabled must have the parameters for all ROIs. "
      "number of rois_id: %ld, number of roi_truncation_validation.enabled: %ld",
      rois_number, validation_enable.size());

    return false;
  }

  if (remove_truncated.size() < rois_number) {
    RCLCPP_ERROR(
      get_logger(),
      "roi_truncation_validation.remove_truncated must have the parameters for all ROIs. "
      "number of rois_id: %ld, number of roi_truncation_validation.remove_truncated: %ld",
      rois_number, remove_truncated.size());

    return false;
  }

  return true;
}

void BboxObjectLocatorNode::cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg, int rois_id)
{
  // assuming camera parameter never changes while node is running
  if (!is_camera_info_arrived_[rois_id]) {
    CameraInfo camera_info = *msg;
    camera_info_[rois_id] = camera_info;

    RoiValidator & roi_validator = roi_validator_[rois_id];

    uint32_t truncation_horizontal_margin = static_cast<uint32_t>(
      camera_info.width * roi_validator.image_border_truncation_horizontal_margin_ratio);
    uint32_t truncation_vertical_margin = static_cast<uint32_t>(
      camera_info.height * roi_validator.image_border_truncation_vertical_margin_ratio);

    // determine truncation border
    //
    // image
    //    +------------ image boarder -- ...   --|
    //    |  truncated ROI (bbox) area           | <-- vertical margin
    //    |                                      |
    //    |    --------- margin border -- ...  --|
    //    |    |
    //    |    |  non truncated ROI (bbox) area
    //    ...
    //    |----|
    //      ^
    //      |
    //     horizontal margin
    //
    // if the bottom center point of bounding box is at margin border pixel,
    // it will be considered as truncated ROI.
    //
    roi_validator.pixel_truncated_top = truncation_vertical_margin;
    roi_validator.pixel_truncated_bottom = camera_info.height > (truncation_vertical_margin + 1)
                                             ? camera_info.height - (truncation_vertical_margin + 1)
                                             : 0;
    roi_validator.pixel_truncated_left = truncation_horizontal_margin;
    roi_validator.pixel_truncated_right = camera_info.width > (truncation_horizontal_margin + 1)
                                            ? camera_info.width - (truncation_horizontal_margin + 1)
                                            : 0;

    // add biased pixel points outside the image when the ROI bounding box touches the image edge
    // used to model truncated objects in the covariance
    const float pixel_offset_x =
      camera_info.height * covariance_control_param_.truncation_out_of_bounds_sampling_ratio_width_;
    const float pixel_offset_y =
      camera_info.height *
      covariance_control_param_.truncation_out_of_bounds_sampling_ratio_height_;
    roi_validator.out_of_bounds_object_sampling_top = -pixel_offset_y;
    roi_validator.out_of_bounds_object_sampling_bottom = camera_info.height + pixel_offset_y;
    roi_validator.out_of_bounds_object_sampling_left = -pixel_offset_x;
    roi_validator.out_of_bounds_object_sampling_right = camera_info.width + pixel_offset_x;

    // K is row-major 3x3
    cv::Matx33d K(
      camera_info.k[0], camera_info.k[1], camera_info.k[2], camera_info.k[3], camera_info.k[4],
      camera_info.k[5], camera_info.k[6], camera_info.k[7], camera_info.k[8]);

    cv::Mat D = cv::Mat(camera_info.d.size(), 1, CV_64F);
    for (size_t i = 0; i < camera_info.d.size(); i++) {
      D.at<double>(i, 0) = camera_info.d[i];
    }

    cam_intrinsics_[rois_id] = CameraIntrinsics{K, D};
    is_camera_info_arrived_[rois_id] = true;
  }
}

/**
 * @brief Estimates a 3D object from a 2D ROI.
 *
 * This function primarily targets pedestrian detection, as the generated 3D object
 * is only an approximation and may be inaccurate without depth information.
 */
bool BboxObjectLocatorNode::generateROIBasedObject(
  const sensor_msgs::msg::RegionOfInterest & roi, const int & rois_id,
  const geometry_msgs::msg::TransformStamped & tf, const uint8_t & label, DetectedObject & object)
{
  const uint32_t x_offset = roi.x_offset;
  const uint32_t y_offset = roi.y_offset;
  const uint32_t roi_width = roi.width;
  const uint32_t roi_height = roi.height;

  // use the ROI's bottom center to estimate the 3D position,
  // and the top center to approximate object height
  const uint32_t bottom_pixel_y = y_offset + roi_height;
  const cv::Point2f bottom_center(x_offset + roi_width * 0.5f, bottom_pixel_y);
  const cv::Point2f top_center(x_offset + roi_width * 0.5f, y_offset);
  // bottom left and right points are used to estimate the object's width/diameter
  const cv::Point2f bottom_left(x_offset, bottom_pixel_y);
  const cv::Point2f bottom_right(x_offset + roi_width, bottom_pixel_y);

  std::vector<cv::Point2f> pixel_for_sampling = {bottom_center};
  RoiValidator roi_validator = roi_validator_[rois_id];

  // check ROI is touching the edge with margin
  if (roi_validator.enable_validation) {
    bool roi_truncated = false;

    // check top and bottom sides
    if (bottom_center.y >= roi_validator.pixel_truncated_bottom) {
      pixel_for_sampling.emplace_back(
        bottom_center.x, roi_validator.out_of_bounds_object_sampling_bottom);
      roi_truncated = true;
    } else if (bottom_center.y <= roi_validator.pixel_truncated_top) {
      pixel_for_sampling.emplace_back(
        bottom_center.x, roi_validator.out_of_bounds_object_sampling_top);
      roi_truncated = true;
    }

    // check left and right sides
    if (bottom_left.x <= roi_validator.pixel_truncated_left) {
      pixel_for_sampling.emplace_back(
        roi_validator.out_of_bounds_object_sampling_left, bottom_center.y);
      roi_truncated = true;
    } else if (bottom_right.x >= roi_validator.pixel_truncated_right) {
      pixel_for_sampling.emplace_back(
        roi_validator.out_of_bounds_object_sampling_right, bottom_center.y);
      roi_truncated = true;
    }

    if (roi_validator.remove_object_might_be_truncated && roi_truncated) {
      // truncated ROI removal
      return false;
    }
  }

  CameraIntrinsics cam_intrinsics = cam_intrinsics_[rois_id];
  const cv::Matx33d K = cam_intrinsics.K;
  const cv::Mat D = cam_intrinsics.D;

  cv::Matx33d R;
  cv::Vec3d t;
  transformToRT(tf, R, t);

  // compute object ground point
  const cv::Vec3d bottom_point_in_3d = projectUndistortedToGround(bottom_center, K, D, R, t);

  const double dist_sq =
    bottom_point_in_3d[0] * bottom_point_in_3d[0] + bottom_point_in_3d[1] * bottom_point_in_3d[1];
  if (dist_sq > detection_max_range_sq_) {
    // skip this ROI if it is beyond the maximum detection range
    return false;
  }

  cv::Point2f undistorted_top_center = undistortPoint(top_center, K, D);
  const double height =
    computeHeight(undistorted_top_center, R, t, bottom_point_in_3d, pseudo_height_);

  const cv::Vec3d left_point_in_3d = projectUndistortedToGround(bottom_left, K, D, R, t);
  const cv::Vec3d right_point_in_3d = projectUndistortedToGround(bottom_right, K, D, R, t);

  const double dim_xy = cv::norm(left_point_in_3d - right_point_in_3d);
  const cv::Vec3d world_opt_axis = R * camera_optical_axis_;

  // this function primarily targets pedestrian, we will check the dimensions of pedestrian
  if (label == Label::PEDESTRIAN) {
    const double clamped_dim_xy = std::clamp(dim_xy, pedestrian_width_min_, pedestrian_width_max_);
    object.shape.dimensions.x = clamped_dim_xy;
    object.shape.dimensions.y = clamped_dim_xy;

    // move the cylinder toward the optical axis to improve fitting
    cv::Vec3d bias_vec = world_opt_axis * dim_xy * 0.5;

    object.kinematics.pose_with_covariance.pose.position.x = bottom_point_in_3d[0] + bias_vec[0];
    object.kinematics.pose_with_covariance.pose.position.y = bottom_point_in_3d[1] + bias_vec[1];
    object.kinematics.pose_with_covariance.pose.position.z = height * 0.5;
  } else {
    object.shape.dimensions.x = dim_xy;
    object.shape.dimensions.y = dim_xy;

    object.kinematics.pose_with_covariance.pose.position.x = bottom_point_in_3d[0];
    object.kinematics.pose_with_covariance.pose.position.y = bottom_point_in_3d[1];
    object.kinematics.pose_with_covariance.pose.position.z = height * 0.5;
  }
  object.shape.dimensions.z = height;

  object.shape.type = label_settings_.getLabelShape(label);
  if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const float spacing = static_cast<float>(dim_xy) * 0.5f;

    // add footprint polygon points for visualization
    geometry_msgs::msg::Point32 p1;
    p1.x = -spacing;
    p1.y = -spacing;
    p1.z = 0.0f;
    object.shape.footprint.points.push_back(p1);

    geometry_msgs::msg::Point32 p2;
    p2.x = -spacing;
    p2.y = +spacing;
    p2.z = 0.0f;
    object.shape.footprint.points.push_back(p2);

    geometry_msgs::msg::Point32 p3;
    p3.x = +spacing;
    p3.y = +spacing;
    p3.z = 0.0f;
    object.shape.footprint.points.push_back(p3);

    geometry_msgs::msg::Point32 p4;
    p4.x = +spacing;
    p4.y = -spacing;
    p4.z = 0.0f;
    object.shape.footprint.points.push_back(p4);
  }

  // sample some ground point for calculating a covariance
  std::vector<cv::Point2f> sampled_ground_points = pixel_sampler_[rois_id]->samplePoints(
    pixel_for_sampling, static_cast<float>(roi_width), static_cast<float>(roi_height));

  // sigma_xy from bottom sampling
  const cv::Matx22d sigma_xy = computeCovarianceXY(
    bottom_center, bottom_left, bottom_right, sampled_ground_points, K, D, R, t, label,
    world_opt_axis);

  object.kinematics.has_position_covariance = true;
  object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;
  auto & pwc = object.kinematics.pose_with_covariance;

  // write into PoseWithCovariance (6x6, row-major: x y z roll pitch yaw)
  // add small floors for Symmetric Positive Definite (SPD) matrix
  constexpr double eps = 1e-6;
  pwc.covariance[XYZRPY_COV_IDX::X_X] = std::max(eps, static_cast<double>(sigma_xy(0, 0)));
  pwc.covariance[XYZRPY_COV_IDX::X_Y] = sigma_xy(0, 1);
  pwc.covariance[XYZRPY_COV_IDX::Y_X] = sigma_xy(1, 0);
  pwc.covariance[XYZRPY_COV_IDX::Y_Y] = std::max(eps, static_cast<double>(sigma_xy(1, 1)));
  // keep others small but non-zero (so the matrix stays positive-definite)
  pwc.covariance[XYZRPY_COV_IDX::Z_Z] = eps;
  pwc.covariance[XYZRPY_COV_IDX::ROLL_ROLL] = eps;
  pwc.covariance[XYZRPY_COV_IDX::PITCH_PITCH] = eps;
  pwc.covariance[XYZRPY_COV_IDX::YAW_YAW] = eps;

  return true;
}

void BboxObjectLocatorNode::roiCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & msg, int rois_id)
{
  if (!is_camera_info_arrived_[rois_id]) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info is not received yet");
    return;
  }

  DetectedObjects objects;

  // get transform from camera frame to target frame
  try {
    transform_ = transform_listener_->getTransform(
      target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to get transform: %s", ex.what());
    objects.header = msg->header;
    objects_pubs_[rois_id]->publish(objects);
    return;
  }

  if (!transform_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "getTransform failed. output objects will be empty.");
    objects.header = msg->header;
    objects_pubs_[rois_id]->publish(objects);
    return;
  }

  for (const auto & obj_with_feature : msg->feature_objects) {
    DetectedObject object;

    const auto & label = obj_with_feature.object.classification.front().label;
    if (!label_settings_.isDetectionTargetLabel(label)) {
      continue;
    }

    // check ROI's confidence
    if (obj_with_feature.object.existence_probability < roi_confidence_th_) {
      continue;
    }

    object.classification.push_back(obj_with_feature.object.classification.front());
    object.existence_probability = obj_with_feature.object.existence_probability;

    // if it is in the detection range, we will publish it
    if (generateROIBasedObject(obj_with_feature.feature.roi, rois_id, *transform_, label, object)) {
      objects.objects.push_back(object);
    }
  }

  objects.header = msg->header;
  objects.header.frame_id = target_frame_;
  objects_pubs_[rois_id]->publish(objects);
}

}  // namespace autoware::image_object_locator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_object_locator::BboxObjectLocatorNode)
