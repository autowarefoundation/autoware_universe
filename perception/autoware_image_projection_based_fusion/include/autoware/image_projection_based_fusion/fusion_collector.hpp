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

#pragma once

#include "autoware/image_projection_based_fusion/camera_projection.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace autoware::image_projection_based_fusion
{
using autoware::image_projection_based_fusion::CameraProjection;

template <class Msg3D, class Msg2D, class ExportObj>
class FusionNode;

template <class Msg2D>
struct Det2dStatus
{
  // camera index
  std::size_t id{0};
  // camera projection
  std::shared_ptr<CameraProjection> camera_projector_ptr{nullptr};
  bool project_to_unrectified_image{false};
  bool approximate_camera_projection{false};
};

struct FusionCollectorInfoBase
{
  virtual ~FusionCollectorInfoBase() = default;
};

struct NaiveCollectorInfo : public FusionCollectorInfoBase
{
  double timestamp;
  double threshold;

  explicit NaiveCollectorInfo(double timestamp = 0.0, double threshold = 0.0)
  : timestamp(timestamp), threshold(threshold)
  {
  }
};

struct AdvancedCollectorInfo : public FusionCollectorInfoBase
{
  double timestamp;
  double noise_window;

  explicit AdvancedCollectorInfo(double timestamp = 0.0, double noise_window = 0.0)
  : timestamp(timestamp), noise_window(noise_window)
  {
  }
};

enum class CollectorStatus { Idle, Processing, Finished };

template <class Msg3D, class Msg2D, class ExportObj>
class FusionCollector
{
public:
  FusionCollector(
    std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> && ros2_parent_node,
    std::size_t rois_number, const std::vector<Det2dStatus<Msg2D>> & det2d_status_list,
    bool debug_mode);
  void process_msg3d(const typename Msg3D::ConstSharedPtr msg3d, double msg3d_timeout);
  void process_rois(
    const std::size_t & rois_id, const typename Msg2D::ConstSharedPtr rois_msg,
    double rois_timeout);
  void fusion_callback();

  [[nodiscard]] CollectorStatus get_status();

  void set_info(std::shared_ptr<FusionCollectorInfoBase> collector_info);
  [[nodiscard]] std::shared_ptr<FusionCollectorInfoBase> get_info() const;
  bool ready_to_fuse();
  bool rois_exists(const std::size_t & rois_id);
  bool msg3d_exists();
  void add_camera_projection(
    std::size_t rois_id, std::shared_ptr<CameraProjection> camera_projector_ptr);
  void set_period(const std::chrono::nanoseconds period);
  void reset();
  void show_debug_message();

private:
  std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> ros2_parent_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t rois_number_;
  typename Msg3D::ConstSharedPtr msg3d_{nullptr};
  std::vector<Det2dStatus<Msg2D>> det2d_status_list_;
  std::unordered_map<std::size_t, typename Msg2D::ConstSharedPtr> id_to_rois_map_;
  bool is_first_msg3d_{false};
  bool debug_mode_;
  std::mutex fusion_mutex_;
  std::shared_ptr<FusionCollectorInfoBase> fusion_collector_info_;
  CollectorStatus status_;
};

}  // namespace autoware::image_projection_based_fusion
