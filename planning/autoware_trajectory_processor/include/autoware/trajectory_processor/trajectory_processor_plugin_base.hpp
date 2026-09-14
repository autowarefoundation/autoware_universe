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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PLUGIN_BASE_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PLUGIN_BASE_HPP_

#include "autoware/trajectory_processor/trajectory_processor_context.hpp"
#include "autoware/trajectory_processor/trajectory_processor_data.hpp"
#include "autoware/trajectory_processor/trajectory_processor_parameters.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::trajectory_processor::plugin
{

using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/// @brief Publisher usable from either supported node type.
template <typename MessageT>
class PublisherHandle
{
public:
  PublisherHandle() = default;

  template <typename PublisherT>
  explicit PublisherHandle(PublisherT publisher)
  : publish_([publisher](const MessageT & msg) { publisher->publish(msg); }),
    subscription_count_([publisher]() { return publisher->get_subscription_count(); })
  {
  }

  void operator()(const MessageT & msg) const { publish_(msg); }

  [[nodiscard]] size_t get_subscription_count() const
  {
    return subscription_count_ ? subscription_count_() : 0;
  }

  explicit operator bool() const { return static_cast<bool>(publish_); }

private:
  std::function<void(const MessageT &)> publish_;
  std::function<size_t()> subscription_count_;
};

/// @brief Planning factor interface usable from either supported node type.
class PlanningFactorAdapter
{
public:
  PlanningFactorAdapter(rclcpp::Node * node, const std::string & name)
  : rclcpp_impl_(
      std::make_unique<autoware::planning_factor_interface::PlanningFactorInterfaceT<rclcpp::Node>>(
        node, name))
  {
  }

  PlanningFactorAdapter(autoware::agnocast_wrapper::Node * node, const std::string & name)
  : agnocast_impl_(
      std::make_unique<autoware::planning_factor_interface::PlanningFactorInterfaceT<
        autoware::agnocast_wrapper::Node>>(node, name))
  {
  }

  template <typename... Args>
  void add(Args &&... args)
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->add(std::forward<Args>(args)...);
    } else {
      agnocast_impl_->add(std::forward<Args>(args)...);
    }
  }

  void publish()
  {
    if (rclcpp_impl_) {
      rclcpp_impl_->publish();
    } else {
      agnocast_impl_->publish();
    }
  }

  [[nodiscard]] std::vector<PlanningFactor> get_factors() const
  {
    return rclcpp_impl_ ? rclcpp_impl_->get_factors() : agnocast_impl_->get_factors();
  }

private:
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterfaceT<rclcpp::Node>>
    rclcpp_impl_;
  std::unique_ptr<
    autoware::planning_factor_interface::PlanningFactorInterfaceT<autoware::agnocast_wrapper::Node>>
    agnocast_impl_;
};

/// @brief Outcome of processing one candidate trajectory.
enum class ProcessingResult { Unchanged, Modified };

/// @brief Common pluginlib interface for trajectory modifier and optimizer plugins.
class TrajectoryProcessorPluginBase
{
public:
  /// @brief Construct an uninitialized plugin instance.
  TrajectoryProcessorPluginBase() = default;
  /// @brief Destroy the plugin through the common interface.
  virtual ~TrajectoryProcessorPluginBase() = default;

  TrajectoryProcessorPluginBase(const TrajectoryProcessorPluginBase &) = delete;
  TrajectoryProcessorPluginBase & operator=(const TrajectoryProcessorPluginBase &) = delete;
  TrajectoryProcessorPluginBase(TrajectoryProcessorPluginBase &&) = delete;
  TrajectoryProcessorPluginBase & operator=(TrajectoryProcessorPluginBase &&) = delete;

  /// @brief Initialize a plugin with distinct class and pipeline instance identities.
  void initialize(
    std::string class_name, std::string instance_name, rclcpp::Node * node_ptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    rclcpp_node_ptr_ = node_ptr;
    initialize_common(
      std::move(class_name), std::move(instance_name), std::move(time_keeper), std::move(context),
      params);
  }

  void initialize(
    std::string class_name, std::string instance_name, autoware::agnocast_wrapper::Node * node_ptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    agnocast_node_ptr_ = node_ptr;
    initialize_common(
      std::move(class_name), std::move(instance_name), std::move(time_keeper), std::move(context),
      params);
  }

  /// @brief Initialize a plugin whose instance identity is its class name.
  void initialize(
    const std::string & class_name, rclcpp::Node * node_ptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    initialize(
      class_name, class_name, node_ptr, std::move(time_keeper), std::move(context), params);
  }

  void initialize(
    const std::string & class_name, autoware::agnocast_wrapper::Node * node_ptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    initialize(
      class_name, class_name, node_ptr, std::move(time_keeper), std::move(context), params);
  }

  /// @brief Process one candidate trajectory using data shared across its plugin pipeline.
  virtual ProcessingResult process(
    TrajectoryPoints & trajectory_points, TrajectoryProcessorData & data) = 0;

  /// @brief Apply an updated snapshot of processor parameters.
  virtual void update_params(const TrajectoryProcessorParams & params) = 0;

  /// @brief Return the unique pipeline instance name.
  [[nodiscard]] const std::string & get_name() const { return instance_name_; }
  /// @brief Return the unqualified plugin class name.
  [[nodiscard]] const std::string & get_short_name() const { return short_name_; }

  /// @brief Publish plugin-specific debug data when supported.
  virtual void publish_debug_data([[maybe_unused]] const std::string & ns) const {}

  /// @brief Publish accumulated planning factors when supported.
  virtual void publish_planning_factor()
  {
    if (planning_factor_interface_) {
      planning_factor_interface_->publish();
    }
  }

  /// @brief Return planning factors currently accumulated by the plugin.
  [[nodiscard]] std::vector<PlanningFactor> get_planning_factors() const
  {
    if (planning_factor_interface_) {
      return planning_factor_interface_->get_factors();
    }
    return {};
  }

protected:
  /// @brief Perform derived-plugin initialization after common state is stored.
  virtual void on_initialize(const TrajectoryProcessorParams & params) = 0;

  /// @brief Invoke @p func with the hosting node, whichever node type backs this plugin.
  template <typename Func>
  decltype(auto) with_node(Func && func) const
  {
    if (agnocast_node_ptr_ != nullptr) {
      return std::forward<Func>(func)(agnocast_node_ptr_);
    }
    return std::forward<Func>(func)(rclcpp_node_ptr_);
  }

  /// @brief Return the hosting node's logger.
  [[nodiscard]] rclcpp::Logger get_logger() const
  {
    return with_node([](auto * node) { return node->get_logger(); });
  }
  /// @brief Return the hosting node's clock.
  [[nodiscard]] rclcpp::Clock::SharedPtr get_clock() const
  {
    return with_node([](auto * node) { return node->get_clock(); });
  }
  /// @brief Return the hosting node's current time.
  [[nodiscard]] rclcpp::Time now() const { return get_clock()->now(); }

  /// @brief Create a publisher on the hosting node and return it as a node-independent callable.
  template <typename MessageT>
  PublisherHandle<MessageT> make_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    return with_node([&](auto * node) -> PublisherHandle<MessageT> {
      return PublisherHandle<MessageT>(node->template create_publisher<MessageT>(topic_name, qos));
    });
  }

  /// @brief Create a polling subscriber on the hosting node.
  template <typename MessageT>
  typename autoware::agnocast_wrapper::polling::PollingSubscriber<MessageT>::SharedPtr
  make_polling_subscriber(const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    using Result =
      typename autoware::agnocast_wrapper::polling::PollingSubscriber<MessageT>::SharedPtr;
    return with_node([&](auto * node) -> Result {
      using NodeT = std::remove_pointer_t<decltype(node)>;
      if constexpr (std::is_same_v<NodeT, autoware::agnocast_wrapper::Node>) {
        return autoware::agnocast_wrapper::polling::create_polling_subscriber<MessageT>(
          node, topic_name, qos);
      } else {
        return std::make_shared<
          autoware::agnocast_wrapper::polling::ROS2PollingSubscriber<MessageT>>(
          node, topic_name, qos);
      }
    });
  }

  /// @brief Create the planning factor interface bound to the hosting node.
  void init_planning_factor_interface(const std::string & name)
  {
    planning_factor_interface_ =
      with_node([&](auto * node) { return std::make_unique<PlanningFactorAdapter>(node, name); });
  }

  /// @brief Return the shared processing time keeper.
  [[nodiscard]] std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const
  {
    return time_keeper_;
  }

  std::unique_ptr<PlanningFactorAdapter> planning_factor_interface_;
  std::shared_ptr<TrajectoryProcessorContext> context_;
  bool enabled_{true};
  double trajectory_time_step_{0.1};

private:
  /// @brief Extract the final component of a namespace-qualified class name.
  static std::string get_unqualified_name(const std::string & name)
  {
    const auto separator = name.find_last_of(':');
    return separator == std::string::npos ? name : name.substr(separator + 1);
  }

  void initialize_common(
    std::string class_name, std::string instance_name,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    class_name_ = std::move(class_name);
    instance_name_ = std::move(instance_name);
    short_name_ = get_unqualified_name(class_name_);
    time_keeper_ = std::move(time_keeper);
    context_ = std::move(context);

    RCLCPP_DEBUG(
      get_logger(), "Instantiated trajectory processor plugin '%s' as '%s'", class_name_.c_str(),
      instance_name_.c_str());
    on_initialize(params);
  }

  std::string class_name_{"unnamed_plugin"};
  std::string instance_name_{"unnamed_plugin"};
  std::string short_name_{"unnamed_plugin"};
  rclcpp::Node * rclcpp_node_ptr_{nullptr};
  autoware::agnocast_wrapper::Node * agnocast_node_ptr_{nullptr};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_processor::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PLUGIN_BASE_HPP_
