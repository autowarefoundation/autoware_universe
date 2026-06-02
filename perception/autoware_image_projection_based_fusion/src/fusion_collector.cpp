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

#include "autoware/image_projection_based_fusion/fusion_collector.hpp"

#include "autoware/image_projection_based_fusion/fusion_node.hpp"
#include "autoware/image_projection_based_fusion/fusion_types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::image_projection_based_fusion
{

template <class Msg3D, class Msg2D, class ExportObj>
FusionCollector<Msg3D, Msg2D, ExportObj>::FusionCollector(
  std::shared_ptr<FusionNode<Msg3D, Msg2D, ExportObj>> && parent_node, std::size_t rois_number,
  const std::vector<Det2dStatus<Msg2D>> & det2d_status_list, bool debug_mode)
: parent_node_(std::move(parent_node)),
  rois_number_(rois_number),
  det2d_status_list_(det2d_status_list),
  debug_mode_(debug_mode)
{
  status_ = CollectorStatus::Idle;

  auto init_timeout_sec = 1.0;  // This will be overwritten when input come
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(init_timeout_sec));

  timer_ = autoware::agnocast_wrapper::create_timer(
    parent_node_.get(), parent_node_->get_clock(), period_ns, [this]() {
      std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
      if (status_ == CollectorStatus::Finished) return;
      fusion_callback();
    });
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::set_info(
  std::shared_ptr<FusionCollectorInfoBase> fusion_collector_info)
{
  fusion_collector_info_ = std::move(fusion_collector_info);
}

template <class Msg3D, class Msg2D, class ExportObj>
std::shared_ptr<FusionCollectorInfoBase> FusionCollector<Msg3D, Msg2D, ExportObj>::get_info() const
{
  return fusion_collector_info_;
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::process_msg3d(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Msg3D) msg3d, double msg3d_timeout)
{
  std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);

  if (status_ == CollectorStatus::Idle) {
    // Add msg3d to the collector, restart the timer
    status_ = CollectorStatus::Processing;
    is_first_msg3d_ = true;
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(msg3d_timeout));
    set_period(period_ns);
    timer_->reset();
  } else if (status_ == CollectorStatus::Processing) {
    if (msg3d_) {
      RCLCPP_WARN_STREAM_THROTTLE(
        parent_node_->get_logger(), *parent_node_->get_clock(),
        std::chrono::milliseconds(10000).count(),
        "Msg3d already exists in the collector. Check the timestamp of the msg3d.");
    }

    if (!is_first_msg3d_) {
      const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(msg3d_timeout));
      set_period(period_ns);
      timer_->reset();
    }
  }

  msg3d_ = msg3d;
  if (ready_to_fuse()) {
    fusion_callback();
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::process_rois(
  const std::size_t & rois_id, const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Msg2D) rois_msg,
  double rois_timeout)
{
  std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);

  if (status_ == CollectorStatus::Idle) {
    // Add rois_msg to the collector, restart the timer
    status_ = CollectorStatus::Processing;
    is_first_msg3d_ = false;
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(rois_timeout));
    set_period(period_ns);
    timer_->reset();
  } else if (status_ == CollectorStatus::Processing) {
    if (id_to_rois_map_.find(rois_id) != id_to_rois_map_.end()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        parent_node_->get_logger(), *parent_node_->get_clock(),
        std::chrono::milliseconds(10000).count(),
        "ROIs '" << rois_id
                 << "' already exists in the collector. Check the timestamp of the rois.");
    }
  }

  id_to_rois_map_[rois_id] = rois_msg;
  if (ready_to_fuse()) {
    fusion_callback();
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
bool FusionCollector<Msg3D, Msg2D, ExportObj>::ready_to_fuse()
{
  return id_to_rois_map_.size() == rois_number_ && static_cast<bool>(msg3d_);
}

template <class Msg3D, class Msg2D, class ExportObj>
CollectorStatus FusionCollector<Msg3D, Msg2D, ExportObj>::get_status()
{
  std::lock_guard<std::mutex> fusion_lock(fusion_mutex_);
  return status_;
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::fusion_callback()
{
  if (debug_mode_) {
    show_debug_message();
  }

  // All pointcloud and rois are received or the timer has timed out, cancel the timer and fuse
  // them.
  timer_->cancel();

  std::unordered_map<std::size_t, double> id_to_stamp_map;
  for (const auto & [rois_id, rois_msg] : id_to_rois_map_) {
    id_to_stamp_map[rois_id] = rclcpp::Time(rois_msg->header.stamp).seconds();
  }

  if (!msg3d_) {
    RCLCPP_DEBUG(
      parent_node_->get_logger(),
      "The input 3D message is not in the fusion collector, so the fusion process will be "
      "skipped.");
    status_ = CollectorStatus::Finished;
    parent_node_->show_diagnostic_message(id_to_stamp_map, fusion_collector_info_);
    return;
  }

  typename Msg3D::SharedPtr output_det3d_msg = std::make_shared<Msg3D>(*msg3d_);
  parent_node_->preprocess(*output_det3d_msg);

  for (const auto & [rois_id, rois_msg] : id_to_rois_map_) {
    if (det2d_status_list_[rois_id].camera_projector_ptr == nullptr) {
      RCLCPP_WARN_THROTTLE(
        parent_node_->get_logger(), *parent_node_->get_clock(), 5000,
        "no camera info. id is %zu", rois_id);
      continue;
    }
    parent_node_->fuse_on_single_image(
      *msg3d_, det2d_status_list_[rois_id], *rois_msg, *output_det3d_msg);
  }

  parent_node_->export_process(output_det3d_msg, id_to_stamp_map, fusion_collector_info_);
  status_ = CollectorStatus::Finished;
}

template <class Msg3D, class Msg2D, class ExportObj>
bool FusionCollector<Msg3D, Msg2D, ExportObj>::rois_exists(const std::size_t & rois_id)
{
  return id_to_rois_map_.find(rois_id) != id_to_rois_map_.end();
}

template <class Msg3D, class Msg2D, class ExportObj>
bool FusionCollector<Msg3D, Msg2D, ExportObj>::msg3d_exists()
{
  return static_cast<bool>(msg3d_);
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::add_camera_projection(
  std::size_t rois_id, std::shared_ptr<CameraProjection> camera_projector_ptr)
{
  std::lock_guard<std::mutex> lock(fusion_mutex_);
  det2d_status_list_[rois_id].camera_projector_ptr = camera_projector_ptr;
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::set_period(const std::chrono::nanoseconds period)
{
  try {
    if (!timer_) {
      return;
    }
    autoware::agnocast_wrapper::set_period(timer_, period);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      parent_node_->get_logger(), *parent_node_->get_clock(), 5000, "%s", ex.what());
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::reset()
{
  std::lock_guard<std::mutex> lock(fusion_mutex_);

  status_ = CollectorStatus::Idle;  // Reset status to Idle
  id_to_rois_map_.clear();
  msg3d_ = {};
  fusion_collector_info_ = nullptr;
  is_first_msg3d_ = false;

  if (timer_ && !timer_->is_canceled()) {
    timer_->cancel();
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionCollector<Msg3D, Msg2D, ExportObj>::show_debug_message()
{
  auto time_until_trigger = timer_->time_until_trigger();
  std::stringstream log_stream;
  log_stream << std::fixed << std::setprecision(6);
  log_stream << "Collector's fusion callback time: "
             << parent_node_->get_clock()->now().seconds() << " seconds\n";

  if (
    auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(fusion_collector_info_)) {
    log_stream << "Advanced strategy:\n Fusion collector's reference time min: "
               << advanced_info->timestamp - advanced_info->noise_window
               << " to max: " << advanced_info->timestamp + advanced_info->noise_window
               << " seconds\n";
  } else if (
    auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(fusion_collector_info_)) {
    log_stream << "Naive strategy:\n Fusion collector's timestamp: " << naive_info->timestamp
               << " seconds\n";
  }

  log_stream << "Time until trigger: " << (time_until_trigger.count() / 1e9) << " seconds\n";
  if (msg3d_) {
    log_stream << "Msg3d: [" << rclcpp::Time(msg3d_->header.stamp).seconds() << "]\n";
  } else {
    log_stream << "Msg3d: [Is empty]\n";
  }
  log_stream << "ROIs: [";
  std::string separator;
  for (const auto & [id, rois] : id_to_rois_map_) {
    log_stream << separator;
    log_stream << "[rois " << id << ", " << rclcpp::Time(rois->header.stamp).seconds() << "]";
    separator = ", ";
  }

  log_stream << "]\n";

  const std::string & str = log_stream.str();
  RCLCPP_INFO(parent_node_->get_logger(), "%s", str.c_str());
}

// Explicit instantiation for the supported types

// pointpainting fusion
template class FusionCollector<PointCloudMsgType, RoiMsgType, DetectedObjects>;

// roi cluster fusion
template class FusionCollector<ClusterMsgType, RoiMsgType, ClusterMsgType>;

// roi detected-object fusion
template class FusionCollector<DetectedObjects, RoiMsgType, DetectedObjects>;

// roi pointcloud fusion
template class FusionCollector<PointCloudMsgType, RoiMsgType, ClusterMsgType>;

// segment pointcloud fusion
template class FusionCollector<PointCloudMsgType, Image, PointCloudMsgType>;

}  // namespace autoware::image_projection_based_fusion
