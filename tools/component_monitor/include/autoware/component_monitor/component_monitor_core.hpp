#ifndef AUTOWARE__COMPONENT_MONITOR__COMPONENT_MONITOR_CORE_HPP_
#define AUTOWARE__COMPONENT_MONITOR__COMPONENT_MONITOR_CORE_HPP_
#include "score_loader.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_map_msgs/srv/get_differential_score_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <stdexcept>
#include <unordered_map>

#ifndef MAX_QUEUE_SIZE_
#define MAX_QUEUE_SIZE_ (100)
#endif

namespace autoware::component_monitor
{

template <typename T>
class Queue
{
public:
  Queue()
  {
    head_loc_ = tail_loc_ = ele_num_ = 0;
    data_.resize(MAX_QUEUE_SIZE_);
  }

  Queue(const Queue & other) : data_(other.data_)
  {
    head_loc_ = other.head_loc_;
    tail_loc_ = other.tail_loc_;
    ele_num_ = other.ele_num_;
  }

  Queue(Queue && other) : data_(std::move(other.data_))
  {
    head_loc_ = other.head_loc_;
    tail_loc_ = other.tail_loc_;
    ele_num_ = other.ele_num_;
  }

  Queue & operator=(const Queue & other)
  {
    data_ = other.data_;

    head_loc_ = other.head_loc_;
    tail_loc_ = other.tail_loc_;
    ele_num_ = other.ele_num_;

    return *this;
  }

  Queue & operator=(Queue && other)
  {
    data_ = std::move(other.data_);

    head_loc_ = other.head_loc_;
    tail_loc_ = other.tail_loc_;

    return *this;
  }

  bool is_full() { return (ele_num_ == MAX_QUEUE_SIZE_); }

  bool is_empty() { return (ele_num_ == 0); }

  T & enqueue(const T & ele)
  {
    // Drop the head entry if the queue is already full
    if (is_full()) {
      dequeue();
    }

    data_[tail_loc_] = ele;

    tail_loc_ = (tail_loc_ == MAX_QUEUE_SIZE_ - 1) ? 0 : tail_loc_ + 1;
    ++ele_num_;
  }

  void dequeue()
  {
    head_loc_ = (head_loc_ == MAX_QUEUE_SIZE_ - 1) ? 0 : head_loc_ + 1;
    --ele_num_;
  }

  // Access the entry at index ith from the head_loc
  T & operator[](int i)
  {
    try {
      return data_[idx_convert(i)];
    } catch (const std::exception & e) {
      std::cerr << "Caught runtime error: " << e.what() << std::endl;
      exit(EXIT_FAILURE);
    }
  }

private:
  int idx_convert(int i)
  {
    if (i >= ele_num_) {
      std::istringstream msg;

      msg << "Out-of-bound queue access. Accessing index: " << i
          << " number of elements: " << ele_num_ << std::endl;

      throw std::runtime_error(msg.str());
    }

    return (head_loc + i) % MAX_QUEUE_SIZE_;
  }

  int head_loc_, tail_loc_, ele_num_;
  std::vector<T> data_;
};

class ComponentMonitor : public rclcpp::Node
{
public:
  explicit ComponentMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void callback_loc_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr);
  void callback_loc_tp(
    autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr tp_msg_ptr);
  void callback_loc_nvtl(
    autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr nvtl_msg_ptr);
  void callback_loc_sensor_points(
    geometry_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_ptr);

private:
  std::vector<std::string> get_cached_ids();

  // A queue buffer to cache poses
  Queue<std::pair<rclcpp::Time, geometry_msgs::msg::PoseWithCovarianceStamped>> pose_buffer_;
  // 0 if the pose is normal, and 1 otherwise (sign of decay detected)
  std::vector<int> pose_mark_;

  // A quque buffer to cache sensor points
  Queue<std::pair<rclcpp::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>> sensor_points_buffer_;

  float segment_res_;  // By default, the segment size is 20x20
  float res_;          // By default, 2.0m

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr loc_pose_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr loc_tp_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr loc_nvtl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;

  // Dynamic score loader
  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialScoreMap>::SharedPtr score_loader_client_;
  const float map_radius_ = 50.0;

  // Average TPs of segments
  std::unordered_map<Eigen::Vector2i, std::unordered_map<Eigen::Vector2i, std::pair<int, float>>>
    avg_tp_map_;
};

}  // namespace autoware::component_monitor

// For storing segments in an unordered map
namespace std
{

template <typename PointT>
struct hash<Eigen::Vector2i>
{
public:
  size_t operator()(const Eigen::Vector2i & id) const
  {
    std::size_t seed = 0;

    seed ^= std::hash<int>{}(id(0)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(id(1)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

}  // namespace std

#endif  // AUTOWARE__COMPONENT_MONITOR__COMPONENT_MONITOR_CORE_HPP_
