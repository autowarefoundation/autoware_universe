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

#include "autoware/multi_object_tracker/odometry.hpp"

#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace autoware::multi_object_tracker
{

namespace
{
// Buffering / interpolation limits for the odometry backend.
constexpr double kMaxOdomBufferAge = 2.0;  // [s] keep at most this much history
constexpr double kMaxExtrapolation = 1.0;  // [s] bounded extrapolation beyond buffer edges

double toSeconds(const rclcpp::Duration & d)
{
  return static_cast<double>(d.nanoseconds()) * 1e-9;
}

// Linear interpolation of the 6x6 covariance arrays.
void lerpCovariance(
  const std::array<double, 36> & a, const std::array<double, 36> & b, double ratio,
  std::array<double, 36> & out)
{
  for (size_t i = 0; i < 36; ++i) {
    out[i] = a[i] + (b[i] - a[i]) * ratio;
  }
}

// Interpolate between two odometry samples at the given ratio in [0, 1].
nav_msgs::msg::Odometry lerpOdometry(
  const nav_msgs::msg::Odometry & a, const nav_msgs::msg::Odometry & b, double ratio,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out = a;
  out.header.stamp = stamp;

  // position: linear
  const auto & pa = a.pose.pose.position;
  const auto & pb = b.pose.pose.position;
  out.pose.pose.position.x = pa.x + (pb.x - pa.x) * ratio;
  out.pose.pose.position.y = pa.y + (pb.y - pa.y) * ratio;
  out.pose.pose.position.z = pa.z + (pb.z - pa.z) * ratio;

  // orientation: SLERP
  tf2::Quaternion qa;
  tf2::Quaternion qb;
  tf2::fromMsg(a.pose.pose.orientation, qa);
  tf2::fromMsg(b.pose.pose.orientation, qb);
  tf2::Quaternion q = qa.slerp(qb, ratio);
  q.normalize();
  out.pose.pose.orientation = tf2::toMsg(q);

  // twist: linear
  const auto & la = a.twist.twist.linear;
  const auto & lb = b.twist.twist.linear;
  out.twist.twist.linear.x = la.x + (lb.x - la.x) * ratio;
  out.twist.twist.linear.y = la.y + (lb.y - la.y) * ratio;
  out.twist.twist.linear.z = la.z + (lb.z - la.z) * ratio;
  const auto & aa = a.twist.twist.angular;
  const auto & ab = b.twist.twist.angular;
  out.twist.twist.angular.x = aa.x + (ab.x - aa.x) * ratio;
  out.twist.twist.angular.y = aa.y + (ab.y - aa.y) * ratio;
  out.twist.twist.angular.z = aa.z + (ab.z - aa.z) * ratio;

  // covariance: linear
  lerpCovariance(a.pose.covariance, b.pose.covariance, ratio, out.pose.covariance);
  lerpCovariance(a.twist.covariance, b.twist.covariance, ratio, out.twist.covariance);

  return out;
}

// Constant-twist forward/backward extrapolation of a single odometry sample by dt seconds.
nav_msgs::msg::Odometry extrapolateOdometry(
  const nav_msgs::msg::Odometry & base, double dt, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out = base;
  out.header.stamp = stamp;

  // integrate planar motion in the body frame using the body twist
  tf2::Quaternion q;
  tf2::fromMsg(base.pose.pose.orientation, q);
  const double yaw = tf2::getYaw(q);
  const double vx = base.twist.twist.linear.x;
  const double vy = base.twist.twist.linear.y;
  const double wz = base.twist.twist.angular.z;

  out.pose.pose.position.x += (std::cos(yaw) * vx - std::sin(yaw) * vy) * dt;
  out.pose.pose.position.y += (std::sin(yaw) * vx + std::cos(yaw) * vy) * dt;

  tf2::Quaternion dq;
  dq.setRPY(0.0, 0.0, wz * dt);
  tf2::Quaternion q_new = q * dq;
  q_new.normalize();
  out.pose.pose.orientation = tf2::toMsg(q_new);

  return out;
}
}  // namespace

EgoSource toEgoSource(const std::string & name)
{
  if (name == "tf") return EgoSource::TF;
  if (name == "odometry") return EgoSource::ODOMETRY;
  throw std::invalid_argument("Invalid ego_source: '" + name + "'. Expected 'tf' or 'odometry'.");
}

DelayReference toDelayReference(const std::string & name)
{
  if (name == "none") return DelayReference::NONE;
  if (name == "publish_delay") return DelayReference::PUBLISH_DELAY;
  if (name == "odometry") return DelayReference::ODOMETRY;
  if (name == "full") return DelayReference::FULL;
  throw std::invalid_argument(
    "Invalid delay_compensation: '" + name +
    "'. Expected 'none', 'publish_delay', 'odometry', or 'full'.");
}

Odometry::Odometry(
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<autoware::agnocast_wrapper::Buffer> tf_buffer, const std::string & world_frame_id,
  const std::string & ego_frame_id, bool enable_odometry_uncertainty, EgoSource ego_source)
: logger_(logger),
  clock_(clock),
  ego_frame_id_(ego_frame_id),
  world_frame_id_(world_frame_id),
  tf_buffer_(tf_buffer),
  tf_listener_(*tf_buffer_),
  ego_source_(ego_source),
  enable_odometry_uncertainty_(enable_odometry_uncertainty)
{
}

void Odometry::updateOdometryBuffer(const nav_msgs::msg::Odometry & odometry)
{
  const rclcpp::Time stamp(odometry.header.stamp);
  odom_buffer_[stamp] = odometry;

  // evict samples older than the newest by more than the max age
  const auto newest = odom_buffer_.rbegin()->first;
  const auto max_age = rclcpp::Duration::from_seconds(kMaxOdomBufferAge);
  for (auto it = odom_buffer_.begin(); it != odom_buffer_.end();) {
    if (newest - it->first > max_age) {
      it = odom_buffer_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<rclcpp::Time> Odometry::getLatestOdometryTime() const
{
  if (odom_buffer_.empty()) {
    return std::nullopt;
  }
  return rclcpp::Time(odom_buffer_.rbegin()->first);
}

std::optional<nav_msgs::msg::Odometry> Odometry::interpolateOdometry(
  const rclcpp::Time & time) const
{
  if (odom_buffer_.empty()) {
    return std::nullopt;
  }

  // exact / bracketing lookup
  const auto upper = odom_buffer_.lower_bound(time);  // first sample with stamp >= time

  if (upper == odom_buffer_.end()) {
    // time is newer than the newest sample: bounded forward extrapolation
    const auto & last = odom_buffer_.rbegin()->second;
    const double dt = toSeconds(time - odom_buffer_.rbegin()->first);
    if (dt > kMaxExtrapolation) {
      return std::nullopt;
    }
    return extrapolateOdometry(last, dt, time);
  }

  if (rclcpp::Time(upper->first) == time) {
    return upper->second;  // exact hit
  }

  if (upper == odom_buffer_.begin()) {
    // time is older than the oldest sample: bounded backward extrapolation
    const double dt = toSeconds(time - upper->first);  // negative
    if (-dt > kMaxExtrapolation) {
      return std::nullopt;
    }
    return extrapolateOdometry(upper->second, dt, time);
  }

  // interpolate between the bracketing samples
  const auto lower = std::prev(upper);
  const rclcpp::Time t0 = lower->first;
  const rclcpp::Time t1 = upper->first;
  const double span = toSeconds(t1 - t0);
  const double ratio = span > 0.0 ? toSeconds(time - t0) / span : 0.0;
  return lerpOdometry(lower->second, upper->second, ratio, time);
}

std::optional<geometry_msgs::msg::Transform> Odometry::getEgoTransformFromOdometry(
  const rclcpp::Time & time) const
{
  const auto odometry = interpolateOdometry(time);
  if (!odometry) {
    return std::nullopt;
  }
  geometry_msgs::msg::Transform transform;
  transform.translation.x = odometry->pose.pose.position.x;
  transform.translation.y = odometry->pose.pose.position.y;
  transform.translation.z = odometry->pose.pose.position.z;
  transform.rotation = odometry->pose.pose.orientation;
  return transform;
}

void Odometry::updateTfCache(
  const rclcpp::Time & time, const geometry_msgs::msg::Transform & tf) const
{
  // update the tf buffer
  tf_cache_.emplace(time, tf);

  // remove too old tf
  const auto max_tf_age = rclcpp::Duration::from_seconds(1.0);
  for (auto it = tf_cache_.begin(); it != tf_cache_.end();) {
    if (time - it->first > max_tf_age) {
      it = tf_cache_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<geometry_msgs::msg::Transform> Odometry::getTransform(const rclcpp::Time & time) const
{
  // odometry backend: interpolate (or bounded-extrapolate) world <- ego at the addressed time
  if (ego_source_ == EgoSource::ODOMETRY) {
    const auto ego_tf = getEgoTransformFromOdometry(time);
    if (ego_tf) {
      return ego_tf;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Odometry buffer miss at %.9f, falling back to TF.", time.seconds());
  }

  // TF backend (and fallback): check cache then look up the transform from tf
  for (const auto & tf : tf_cache_) {
    if (tf.first == time) {
      return std::optional<geometry_msgs::msg::Transform>(tf.second);
    }
  }
  return getTransform(ego_frame_id_, time);
}

std::optional<geometry_msgs::msg::Transform> Odometry::getTransform(
  const std::string & source_frame_id, const rclcpp::Time & time) const
{
  // odometry backend: world <- source = world <- ego(t) [odom] * ego <- source [static TF]
  if (ego_source_ == EgoSource::ODOMETRY) {
    const auto world_to_ego = getEgoTransformFromOdometry(time);
    if (world_to_ego) {
      if (source_frame_id == ego_frame_id_) {
        return world_to_ego;
      }
      try {
        std::string errstr;
        if (tf_buffer_->canTransform(
              ego_frame_id_, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
          const auto ego_to_source =
            tf_buffer_->lookupTransform(ego_frame_id_, source_frame_id, tf2::TimePointZero);
          tf2::Transform tf_world_to_ego;
          tf2::Transform tf_ego_to_source;
          tf2::fromMsg(*world_to_ego, tf_world_to_ego);
          tf2::fromMsg(ego_to_source.transform, tf_ego_to_source);
          geometry_msgs::msg::Transform composed;
          tf2::toMsg(tf_world_to_ego * tf_ego_to_source, composed);
          return composed;
        }
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_STREAM(logger_, ex.what());
      }
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Odometry-based transform unavailable for '%s', falling back to TF.",
      source_frame_id.c_str());
  }

  try {
    // Check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer_->canTransform(
          world_frame_id_, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return std::nullopt;
    }

    // Lookup the transform
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer_->lookupTransform(
      world_frame_id_, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));

    // update the cache
    if (source_frame_id == ego_frame_id_) {
      updateTfCache(time, self_transform_stamped.transform);
    }

    return std::optional<geometry_msgs::msg::Transform>(self_transform_stamped.transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(logger_, ex.what());
    return std::nullopt;
  }
}

std::optional<nav_msgs::msg::Odometry> Odometry::getOdometryFromTf(const rclcpp::Time & time) const
{
  // odometry backend: return the real interpolated odometry (with real twist & covariance)
  if (ego_source_ == EgoSource::ODOMETRY) {
    if (const auto odometry = interpolateOdometry(time)) {
      return odometry;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Odometry buffer miss at %.9f, fabricating from TF.", time.seconds());
  }

  const auto self_transform = getTransform(time);
  if (!self_transform) {
    return std::nullopt;
  }
  const auto current_transform = self_transform.value();

  nav_msgs::msg::Odometry odometry;
  {
    odometry.header.stamp = time + rclcpp::Duration::from_seconds(0.00001);

    // set the odometry pose
    auto & odom_pose = odometry.pose.pose;
    odom_pose.position.x = current_transform.translation.x;
    odom_pose.position.y = current_transform.translation.y;
    odom_pose.position.z = current_transform.translation.z;
    odom_pose.orientation = current_transform.rotation;

    // set odometry twist
    auto & odom_twist = odometry.twist.twist;
    odom_twist.linear.x = 10.0;  // m/s
    odom_twist.linear.y = 0.1;   // m/s
    odom_twist.angular.z = 0.1;  // rad/s

    // model the uncertainty
    auto & odom_pose_cov = odometry.pose.covariance;
    odom_pose_cov[0] = 0.1;      // x-x
    odom_pose_cov[7] = 0.1;      // y-y
    odom_pose_cov[35] = 0.0001;  // yaw-yaw

    auto & odom_twist_cov = odometry.twist.covariance;
    odom_twist_cov[0] = 2.0;     // x-x [m^2/s^2]
    odom_twist_cov[7] = 0.2;     // y-y [m^2/s^2]
    odom_twist_cov[35] = 0.001;  // yaw-yaw [rad^2/s^2]
  }

  return std::optional<nav_msgs::msg::Odometry>(odometry);
}

std::optional<types::DynamicObjectList> Odometry::transformObjects(
  const types::DynamicObjectList & input_objects) const
{
  types::DynamicObjectList output_objects = input_objects;

  // transform to world coordinate
  if (input_objects.header.frame_id != world_frame_id_) {
    output_objects.header.frame_id = world_frame_id_;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(input_objects.header.frame_id, input_objects.header.stamp);
      if (!ros_target2objects_world) {
        return std::nullopt;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_objects.objects) {
      auto & pose = object.pose;
      auto & pose_cov = object.pose_covariance;
      tf2::fromMsg(pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      // transform pose, frame difference and object pose
      tf2::toMsg(tf_target2objects, pose);
      // transform covariance, only the frame difference
      pose_cov = tf2::transformCovariance(pose_cov, tf_target2objects_world);
    }
  }
  // Add the odometry uncertainty to the object uncertainty
  if (enable_odometry_uncertainty_) {
    // Create a modeled odometry message
    const auto odometry = getOdometryFromTf(input_objects.header.stamp);
    if (!odometry) {
      return std::nullopt;
    }
    // Add the odometry uncertainty to the object uncertainty
    uncertainty::addOdometryUncertainty(odometry.value(), output_objects);
  }

  return std::optional<types::DynamicObjectList>(output_objects);
}

}  // namespace autoware::multi_object_tracker
