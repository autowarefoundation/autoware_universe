// Copyright 2025 TIER IV.
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

#ifndef SYNCHRONIZATION_STRATEGY_HPP_
#define SYNCHRONIZATION_STRATEGY_HPP_

#include "vad_interface.hpp"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <memory>
#include <optional>

namespace autoware::tensorrt_vad
{

/**
 * @brief Abstract base class for synchronization strategies
 *
 * This class defines the interface for different synchronization strategies
 * that determine when inference should be triggered and how to handle
 * dropped topic data.
 */
class SynchronizationStrategy
{
public:
  virtual ~SynchronizationStrategy() = default;

  /**
   * @brief Check if the VadInputTopicData is ready to trigger inference
   * @param vad_input_topic_data VadInputTopicData in Current frame
   * @return true if ready to trigger inference, false otherwise
   */
  virtual bool is_ready(const VadInputTopicData & vad_input_topic_data) const = 0;

  /**
   * @brief Check if any image topic data is dropped
   * @param vad_input_topic_data VadInputTopicData in Current frame
   * @return true if any image topic data is dropped, false otherwise
   */
  virtual bool is_dropped(const VadInputTopicData & vad_input_topic_data) const = 0;

  /**
   * @brief Fill dropped data with appropriate default values
   * @param current_data VadInputTopicData in Current frame with potentially dropped topics
   * @return Modified frame data with dropped topics filled
   */
  virtual std::optional<VadInputTopicData> fill_dropped_data(
    const VadInputTopicData & current_data) = 0;

  /**
   * @brief Check if the current frame data is synchronized within tolerance
   * @param vad_input_topic_data VadInputTopicData in Current frame
   * @return true if data is synchronized, false otherwise
   */
  virtual bool is_synchronized(const VadInputTopicData & vad_input_topic_data) const = 0;
};

/**
 * @brief Front camera critical synchronization strategy
 *
 * This strategy considers the front camera as the anchor for synchronization.
 * Inference is triggered when front camera data is available and synchronized
 * with localization data.
 */
class FrontCriticalSynchronizationStrategy : public SynchronizationStrategy
{
public:
  /**
   * @brief Constructor
   * @param front_camera_id Camera ID for the front camera (anchor topic)
   * @param sync_tolerance_ms Synchronization tolerance in milliseconds (e.g. 100ms)
   */
  explicit FrontCriticalSynchronizationStrategy(int32_t front_camera_id, double sync_tolerance_ms);

  // Override virtual functions
  bool is_ready(const VadInputTopicData & vad_input_topic_data) const override;
  bool is_dropped(const VadInputTopicData & vad_input_topic_data) const override;

  /**
   * @brief Fill dropped data with black images using front camera dimensions
   * @param current_data VadInputTopicData in Current frame with potentially dropped topics
   * @return Modified frame data with dropped topics filled
   * @note Uses front camera's width, height, and timestamp for all dropped images
   */
  std::optional<VadInputTopicData> fill_dropped_data(
    const VadInputTopicData & current_data) override;

  /**
   * @brief Check if the current frame data is synchronized
   *        when the acceleration and kinematic_state topic timestamps
   *        are within sync_tolerance_ms of the front_camera timestamp for synchronization.
   * @param vad_input_topic_data VadInputTopicData in Current frame
   * @return true if data is synchronized, false otherwise
   */
  bool is_synchronized(const VadInputTopicData & vad_input_topic_data) const override;

private:
  int32_t front_camera_id_;   ///< Front camera ID (anchor camera)
  double sync_tolerance_ms_;  ///< Synchronization tolerance in milliseconds
};

}  // namespace autoware::tensorrt_vad

#endif  // SYNCHRONIZATION_STRATEGY_HPP_
