// Copyright 2026 TIER IV, Inc.
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

#include "autoware/multi_object_tracker/association/association_manager.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
#include <limits>
#include <list>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

namespace
{
// Maximum age of the ego pose relative to the measurement timestamp
constexpr double kMaxEgoPoseAgeSec = 0.08;
}  // namespace

AssociationManager::AssociationManager(
  const AssociatorConfig & bev_config, const std::vector<types::InputChannel> & channels_config)
: channels_config_(channels_config),
  bev_association_(std::make_unique<BevAssociation>(bev_config)),
  polar_association_(std::make_unique<PolarAssociation>(bev_config))
{
}

AssociationBase & AssociationManager::getAssociationForChannel(
  const uint channel_index, const bool polar_viable) const
{
  if (
    polar_viable &&
    channels_config_[channel_index].associator_type == types::AssociationType::POLAR) {
    return *polar_association_;
  }
  return *bev_association_;
}

bool AssociationManager::isPolarViable(const rclcpp::Time & measurement_time) const
{
  if (!ego_pose_.has_value()) return false;
  const rclcpp::Time ego_time{ego_pose_->header.stamp};
  const double dt = std::abs((measurement_time - ego_time).seconds());
  return dt <= kMaxEgoPoseAgeSec;
}

void AssociationManager::setEgoPose(const std::optional<geometry_msgs::msg::PoseStamped> & ego_pose)
{
  ego_pose_ = ego_pose;
  polar_association_->setEgoPose(ego_pose ? std::make_optional(ego_pose->pose) : std::nullopt);
}

types::AssociationResult AssociationManager::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  const rclcpp::Time meas_time{measurements.header.stamp};
  const bool polar_viable = isPolarViable(meas_time);

  const bool channel_wants_polar =
    channels_config_[measurements.channel_index].associator_type == types::AssociationType::POLAR;
  if (channel_wants_polar && !polar_viable) {
    static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    const double dt = ego_pose_ ? (meas_time - rclcpp::Time{ego_pose_->header.stamp}).seconds()
                                : std::numeric_limits<double>::infinity();
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("association_manager"), steady_clock, 5000,
      "AssociationManager: polar channel falling back to BEV — ego pose dt=%.3f s (threshold=%.3f "
      "s)",
      dt, kMaxEgoPoseAgeSec);
  }

  return getAssociationForChannel(measurements.channel_index, polar_viable)
    .associate(measurements, trackers);
}

void AssociationManager::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  polar_association_->setTimeKeeper(time_keeper_ptr);
  bev_association_->setTimeKeeper(std::move(time_keeper_ptr));
}

}  // namespace autoware::multi_object_tracker
