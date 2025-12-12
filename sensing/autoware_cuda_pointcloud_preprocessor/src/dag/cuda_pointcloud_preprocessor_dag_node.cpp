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

#include "autoware/cuda_pointcloud_preprocessor/dag/cuda_pointcloud_preprocessor_dag_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/dag/dag_config_parser.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/dag_utils.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filter_registrations.hpp"
#include "autoware/cuda_pointcloud_preprocessor/memory.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <stdexcept>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

CudaPointcloudPreprocessorDagNode::CudaPointcloudPreprocessorDagNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_pointcloud_preprocessor_dag", node_options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_)
{
  using std::placeholders::_1;

  CHECK_CUDA_ERROR(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));
  CHECK_CUDA_ERROR(cudaStreamCreate(&cuda_stream_));

  int current_device_id = 0;
  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  cudaMemPoolProps pool_props = {};
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.location.id = current_device_id;
  pool_props.location.type = cudaMemLocationTypeDevice;
  CHECK_CUDA_ERROR(cudaMemPoolCreate(&cuda_memory_pool_, &pool_props));

  constexpr uint64_t DEFAULT_POOL_RELEASE_THRESHOLD = 1ULL << 30;
  uint64_t pool_release_threshold = DEFAULT_POOL_RELEASE_THRESHOLD;
  CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
    cuda_memory_pool_, cudaMemPoolAttrReleaseThreshold,
    static_cast<void *>(&pool_release_threshold)));

  shared_preprocessor_ = std::make_unique<CudaPointcloudPreprocessor>();

  context_.stream = cuda_stream_;
  context_.memory_pool = cuda_memory_pool_;
  context_.tf_buffer = &tf2_buffer_;
  context_.clock = this->get_clock().get();
  context_.shared_preprocessor = shared_preprocessor_.get();

  auto logger = this->get_logger();
  context_.logger = &logger;
  context_.twist_queue = &twist_queue_;
  context_.angular_velocity_queue = &angular_velocity_queue_;

  auto dag_config = parseDagConfig();
  executor_.buildDag(dag_config, context_);

  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;

    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "cuda_pointcloud_preprocessor_dag");
    stop_watch_ptr_->tic("processing_time");
  }

  createDynamicSubscribers();
  createDynamicPublishers();
}

std::vector<DagNodeConfig> CudaPointcloudPreprocessorDagNode::parseDagConfig()
{
  // Get the DAG configuration file path from ROS parameters
  auto dag_config_file = declare_parameter<std::string>("dag_config_file", "");

  if (dag_config_file.empty()) {
    RCLCPP_ERROR(get_logger(), "dag_config_file parameter is not set!");
    throw std::runtime_error("dag_config_file parameter is required");
  }

  RCLCPP_INFO(get_logger(), "Loading DAG configuration from: %s", dag_config_file.c_str());

  try {
    // Parse the YAML configuration file
    DagConfigParser parser;
    DagConfig config = parser.parseFromFile(dag_config_file);

    // Log DAG structure for debugging
    RCLCPP_INFO(
      get_logger(), "Loaded DAG with %zu inputs, %zu nodes, %zu outputs", config.inputs.size(),
      config.nodes.size(), config.outputs.size());

    for (const auto & node : config.nodes) {
      RCLCPP_DEBUG(
        get_logger(), "  Node '%s' (type: %s) with %zu inputs, %zu outputs", node.id.c_str(),
        node.type.c_str(), node.inputs.size(), node.outputs.size());
    }

    // Store input configurations for subscriber creation
    dag_input_configs_ = config.inputs;

    // Store output configurations for publisher creation
    dag_output_configs_ = config.outputs;

    return config.nodes;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse DAG configuration: %s", e.what());
    throw;
  }
}

void CudaPointcloudPreprocessorDagNode::createDynamicSubscribers()
{
  const uint16_t QUEUE_SIZE = 100;

  for (const auto & input_config : dag_input_configs_) {
    if (input_config.name == "pointcloud") {
      // Create CUDA blackboard subscriber for pointcloud
      pointcloud_sub_ = std::make_shared<
        cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
        *this, input_config.topic,
        std::bind(
          &CudaPointcloudPreprocessorDagNode::pointcloudCallback, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "Created pointcloud subscriber for topic: %s", input_config.topic.c_str());
    } else if (input_config.name == "twist") {
      // Create twist subscriber
      twist_sub_ = autoware_utils::InterProcessPollingSubscriber<
        geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::
        create_subscription(this, input_config.topic, rclcpp::QoS(QUEUE_SIZE));
      RCLCPP_INFO(
        get_logger(), "Created twist subscriber for topic: %s", input_config.topic.c_str());
    } else if (input_config.name == "imu") {
      // Create IMU subscriber
      imu_sub_ = autoware_utils::InterProcessPollingSubscriber<
        sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::
        create_subscription(this, input_config.topic, rclcpp::QoS(QUEUE_SIZE));
      RCLCPP_INFO(get_logger(), "Created IMU subscriber for topic: %s", input_config.topic.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "Unknown input type '%s', skipping subscriber creation",
        input_config.name.c_str());
    }
  }
}

void CudaPointcloudPreprocessorDagNode::createDynamicPublishers()
{
  for (const auto & output_config : dag_output_configs_) {
    // Validate output type
    if (output_config.type != "cuda_blackboard::CudaPointCloud2") {
      RCLCPP_WARN(
        get_logger(),
        "Output '%s' has unsupported type '%s', only 'cuda_blackboard::CudaPointCloud2' is "
        "supported. Skipping.",
        output_config.name.c_str(), output_config.type.c_str());
      continue;
    }

    // Create publisher for this output
    auto publisher =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, output_config.topic);

    RCLCPP_INFO(
      get_logger(), "Created publisher '%s' for node '%s.%s' -> topic '%s'",
      output_config.name.c_str(), output_config.from_node.c_str(), output_config.source.c_str(),
      output_config.topic.c_str());

    // Store publisher with the output node.source as the key for easy lookup
    std::string output_key = output_config.from_node + "." + output_config.source;
    publishers_[output_key] = std::move(publisher);
  }

  if (publishers_.empty()) {
    RCLCPP_WARN(
      get_logger(), "No publishers created. DAG outputs may not be configured correctly.");
  }
}

void CudaPointcloudPreprocessorDagNode::pointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  const auto & input_pointcloud_msg = *input_pointcloud_msg_ptr;

  stop_watch_ptr_->toc("processing_time", true);

  // Get first point timestamp (simplified - use header stamp)
  std::uint64_t first_point_stamp = 1'000'000'000ULL * input_pointcloud_msg.header.stamp.sec +
                                    input_pointcloud_msg.header.stamp.nanosec;

  updateTwistQueue(first_point_stamp);
  updateImuQueue(first_point_stamp);

  // STEP 1: INTERNALIZED ORGANIZE - Automatically organize input pointcloud
  // This is always required for CUDA processing, so we do it automatically
  auto organized_unique = shared_preprocessor_->organizePointcloudPublic(input_pointcloud_msg);

  // STEP 2: Create PointcloudProcessingState (ENTRY to zero-copy pipeline)
  // This state points to internal device_organized_points_ (non-owning pointer)
  auto processing_state =
    std::make_shared<autoware::cuda_pointcloud_preprocessor::dag::PointcloudProcessingState>(
      shared_preprocessor_->createProcessingStateFromOrganized(*organized_unique));

  // Prepare inputs with processing state for DAG execution
  std::map<std::string, std::shared_ptr<void>> inputs;
  inputs["pointcloud"] = processing_state;

  try {
    // Execute DAG with publishers for immediate publishing
    // Publishers are passed to execute() so outputs are finalized and published immediately
    // after each node produces them, preventing modification by subsequent filters
    auto outputs = executor_.execute(inputs, publishers_);

    // STEP 3: Publishing is now handled inside execute() for immediate publication
    // No need to iterate here - outputs are already published when produced
    // The outputs map only contains intermediate results for downstream consumers

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error executing DAG: %s", e.what());
  }

  const auto processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", processing_time_ms);
}

void CudaPointcloudPreprocessorDagNode::updateTwistQueue(std::uint64_t first_point_stamp)
{
  // Poll new twist messages and add to queue
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> twist_msgs =
    twist_sub_->take_data();

  for (const auto & msg : twist_msgs) {
    // Clean up old messages
    while (!twist_queue_.empty()) {
      bool backwards_time_jump_detected =
        rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(msg->header.stamp);
      bool is_queue_longer_than_1s =
        rclcpp::Time(twist_queue_.front().header.stamp) <
        rclcpp::Time(msg->header.stamp) - rclcpp::Duration::from_seconds(1.0);

      if (backwards_time_jump_detected) {
        twist_queue_.clear();
      } else if (is_queue_longer_than_1s) {
        twist_queue_.pop_front();
      } else {
        break;
      }
    }

    // Insert in sorted order
    auto it = std::lower_bound(
      twist_queue_.begin(), twist_queue_.end(), msg->header.stamp,
      [](const auto & twist, const auto & stamp) {
        return rclcpp::Time(twist.header.stamp) < stamp;
      });
    twist_queue_.insert(it, *msg);
  }

  // Remove messages older than first_point_stamp
  auto it = std::lower_bound(
    twist_queue_.begin(), twist_queue_.end(), first_point_stamp,
    [](const auto & twist, const std::uint64_t stamp) {
      return rclcpp::Time(twist.header.stamp).nanoseconds() < stamp;
    });

  twist_queue_.erase(twist_queue_.begin(), it);
}

void CudaPointcloudPreprocessorDagNode::updateImuQueue(std::uint64_t first_point_stamp)
{
  // Poll new IMU messages and add to queue
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_msgs = imu_sub_->take_data();

  for (const auto & msg : imu_msgs) {
    // Clean up old messages
    while (!angular_velocity_queue_.empty()) {
      bool backwards_time_jump_detected =
        rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
        rclcpp::Time(msg->header.stamp);
      bool is_queue_longer_than_1s =
        rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
        rclcpp::Time(msg->header.stamp) - rclcpp::Duration::from_seconds(1.0);

      if (backwards_time_jump_detected) {
        angular_velocity_queue_.clear();
      } else if (is_queue_longer_than_1s) {
        angular_velocity_queue_.pop_front();
      } else {
        break;
      }
    }

    // Transform angular velocity to base frame
    geometry_msgs::msg::Vector3Stamped angular_velocity;
    angular_velocity.vector = msg->angular_velocity;
    angular_velocity.header = msg->header;

    // Insert in queue (no transform for now - will be handled by distortion filter if needed)
    auto it = std::lower_bound(
      angular_velocity_queue_.begin(), angular_velocity_queue_.end(), msg->header.stamp,
      [](const auto & av, const auto & stamp) { return rclcpp::Time(av.header.stamp) < stamp; });
    angular_velocity_queue_.insert(it, angular_velocity);
  }

  // Remove messages older than first_point_stamp
  auto it = std::lower_bound(
    angular_velocity_queue_.begin(), angular_velocity_queue_.end(), first_point_stamp,
    [](const auto & angular_velocity, const std::uint64_t stamp) {
      return rclcpp::Time(angular_velocity.header.stamp).nanoseconds() < stamp;
    });

  angular_velocity_queue_.erase(angular_velocity_queue_.begin(), it);
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::dag::CudaPointcloudPreprocessorDagNode)
