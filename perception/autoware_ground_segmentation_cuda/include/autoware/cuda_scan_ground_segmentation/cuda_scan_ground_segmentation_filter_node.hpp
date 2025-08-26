#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_NODE_HPP_

#include "autoware/cuda_scan_ground_segmentation/cuda_scan_ground_segmentation_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

// Autoware utils
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>

namespace autoware::cuda_ground_segmentation
{
class CudaScanGroundSegmentationFilterNode : public rclcpp::Node
{
public:
  explicit CudaScanGroundSegmentationFilterNode(const rclcpp::NodeOptions & options);

private:
  void cudaPointCloudCallback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & msg);
  // Cuda Sub
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    sub_{};
  // Cuda Pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    pub_{};

  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    pub_gnd_{};
  // Cuda Ground Segmentation Filter
  std::unique_ptr<CudaScanGroundSegmentationFilter> cuda_ground_segmentation_filter_{};
  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
};

}  // namespace autoware::cuda_ground_segmentation

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_NODE_HPP_
