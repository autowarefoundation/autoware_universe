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

#include <string>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED

#include "autoware_utils/ros/polling_subscriber.hpp"

#include <agnocast/agnocast.hpp>

#include <cstdlib>
#include <memory>
#include <type_traits>

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<    \
    MessageT, autoware::agnocast_wrapper::OwnershipType::Unique>
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) \
  autoware::agnocast_wrapper::message_ptr<    \
    MessageT, autoware::agnocast_wrapper::OwnershipType::Shared>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::Publisher<MessageT>::SharedPtr
#define AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT) \
  typename autoware::agnocast_wrapper::PollingSubscriber<MessageT>::SharedPtr

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  autoware::agnocast_wrapper::create_subscription<message_type>(this, topic, qos, callback, options)
#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  autoware::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  autoware::agnocast_wrapper::create_publisher<message_type>(this, arg1, arg2, arg3)
#define AUTOWARE_CREATE_POLLING_SUBSCRIBER(message_type, topic, qos) \
  autoware::agnocast_wrapper::create_polling_subscriber<message_type>(this, topic, qos)

#define AUTOWARE_SUBSCRIPTION_OPTIONS agnocast::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS agnocast::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) publisher->allocate_output_message_unique()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) publisher->allocate_output_message_shared()

namespace autoware::agnocast_wrapper
{

enum class OwnershipType { Unique, Shared };

template <typename MessageT, OwnershipType Ownership>
class message_interface;

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Unique>
{
public:
  message_interface() = default;

  virtual ~message_interface() = default;

  message_interface(const message_interface & r) = delete;
  message_interface & operator=(const message_interface & r) = delete;

  message_interface(message_interface && r) = default;
  message_interface & operator=(message_interface && r) = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::unique_ptr<MessageT> move_ros2_ptr() && noexcept = 0;
};

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Shared>
{
public:
  virtual ~message_interface() = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::shared_ptr<MessageT> move_ros2_ptr() && noexcept = 0;
};

template <typename MessageT, OwnershipType Ownership>
class agnocast_message : public message_interface<MessageT, Ownership>
{
  using ros2_ptr_t = std::conditional_t<
    Ownership == OwnershipType::Unique, std::unique_ptr<MessageT>, std::shared_ptr<MessageT>>;

  agnocast::ipc_shared_ptr<MessageT> ptr_;

public:
  explicit agnocast_message(agnocast::ipc_shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return std::move(ptr_);
  }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  ros2_ptr_t move_ros2_ptr() && noexcept override { return ros2_ptr_t{}; }
};

template <typename MessageT, OwnershipType Ownership>
class ros2_message : public message_interface<MessageT, Ownership>
{
  using ros2_ptr_t = std::conditional_t<
    Ownership == OwnershipType::Unique, std::unique_ptr<MessageT>, std::shared_ptr<MessageT>>;

  ros2_ptr_t ptr_;

public:
  explicit ros2_message(ros2_ptr_t && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  ros2_ptr_t move_ros2_ptr() && noexcept override { return std::move(ptr_); }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return agnocast::ipc_shared_ptr<MessageT>{};
  }
};

template <typename MessageT, OwnershipType Ownership>
class message_ptr
{
  using ros2_ptr_t = std::conditional_t<
    Ownership == OwnershipType::Unique, std::unique_ptr<MessageT>, std::shared_ptr<MessageT>>;

  std::shared_ptr<message_interface<MessageT, Ownership>> ptr_;

  template <typename U>
  friend class AgnocastPublisher;
  template <typename U>
  friend class ROS2Publisher;

private:
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept
  {
    return std::move(*(std::move(ptr_))).move_agnocast_ptr();
  }

  auto move_ros2_ptr() && noexcept { return std::move(*(std::move(ptr_))).move_ros2_ptr(); }

public:
  message_ptr() : ptr_(nullptr) {}

  explicit message_ptr(agnocast::ipc_shared_ptr<MessageT> && ptr)
  : ptr_(std::make_unique<agnocast_message<MessageT, Ownership>>(std::move(ptr)))
  {
  }

  explicit message_ptr(ros2_ptr_t && ptr)
  : ptr_(std::make_unique<ros2_message<MessageT, Ownership>>(std::move(ptr)))
  {
  }

  MessageT & operator*() const noexcept { return ptr_->as_ref(); }

  MessageT * operator->() const noexcept { return ptr_->as_ptr(); }

  explicit operator bool() const noexcept { return static_cast<bool>(ptr_->as_ptr()); }

  MessageT * get() const noexcept { return ptr_->as_ptr(); }
};

// Defaults to zero if the environment variable is missing or invalid.
inline int get_ENABLE_AGNOCAST()
{
  const char * env = std::getenv("ENABLE_AGNOCAST");
  if (env) {
    return std::atoi(env);
  }
  return 0;
}

inline bool use_agnocast()
{
  static const int sv = get_ENABLE_AGNOCAST();
  return sv == 1;
}

template <typename MessageT>
class Subscription
{
  typename rclcpp::Subscription<MessageT>::SharedPtr ros2_sub_{nullptr};
  typename agnocast::Subscription<MessageT>::SharedPtr agnocast_sub_{nullptr};

public:
  using SharedPtr = std::shared_ptr<Subscription<MessageT>>;

  template <typename Func>
  explicit Subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
    const agnocast::SubscriptionOptions & options)
  {
    static_assert(
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&> ||
        std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_SHARED_PTR(MessageT) &&>,
      "callback should be invocable with an rvalue reference to either AUTOWARE_MESSAGE_UNIQUE_PTR "
      "or AUTOWARE_MESSAGE_SHARED_PTR");

    constexpr auto ownership =
      std::is_invocable_v<std::decay_t<Func>, AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) &&>
        ? OwnershipType::Unique
        : OwnershipType::Shared;

    if (use_agnocast()) {
      agnocast_sub_ = agnocast::create_subscription<MessageT>(
        node, topic_name, qos,
        [callback = std::forward<Func>(callback)](agnocast::ipc_shared_ptr<MessageT> && msg) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        },
        options);
    } else {
      rclcpp::SubscriptionOptions ros2_options;
      ros2_options.callback_group = options.callback_group;
      ros2_sub_ = node->create_subscription<MessageT>(
        topic_name, qos,
        [callback = std::forward<Func>(callback)](std::unique_ptr<MessageT> msg) {
          callback(message_ptr<MessageT, ownership>(std::move(msg)));
        },
        ros2_options);
    }
  }
};

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos, Func && callback,
  const agnocast::SubscriptionOptions & options)
{
  return std::make_shared<Subscription<MessageT>>(
    node, topic_name, qos, std::forward<Func>(callback), options);
}

template <typename MessageT, typename Func>
typename Subscription<MessageT>::SharedPtr create_subscription(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  Func && callback, const agnocast::SubscriptionOptions & options)
{
  return std::make_shared<Subscription<MessageT>>(
    node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)),
    std::forward<Func>(callback), options);
}

template <typename MessageT>
class PollingSubscriber
{
public:
  using SharedPtr = std::shared_ptr<PollingSubscriber<MessageT>>;

  virtual ~PollingSubscriber() = default;

  virtual AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() = 0;
  virtual AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() = 0;
};

template <typename MessageT>
class AgnocastPollingSubscriber : public PollingSubscriber<MessageT>
{
  typename agnocast::PollingSubscriber<MessageT>::SharedPtr subscriber_;

public:
  explicit AgnocastPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(agnocast::create_subscription<MessageT>(node, topic_name, qos))
  {
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() override
  {
    auto data = subscriber_->take_data();
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(data));
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() override
  {
    auto data = subscriber_->take_data();
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(data));
  }
};

template <typename MessageT>
class ROS2PollingSubscriber : public PollingSubscriber<MessageT>
{
  typename autoware_utils::InterProcessPollingSubscriber<MessageT>::SharedPtr subscriber_;

public:
  explicit ROS2PollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : subscriber_(
      autoware_utils::InterProcessPollingSubscriber<MessageT>::create_subscription(
        node, topic_name, qos))
  {
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) takeData() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(subscriber_->take_data()));
  }

  AUTOWARE_MESSAGE_SHARED_PTR(const MessageT) take_data() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(const MessageT)(std::move(subscriber_->take_data()));
  }
};

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  } else {
    return std::make_shared<ROS2PollingSubscriber<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)));
  }
}

template <typename MessageT>
typename PollingSubscriber<MessageT>::SharedPtr create_polling_subscriber(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPollingSubscriber<MessageT>>(node, topic_name, qos);
  } else {
    return std::make_shared<ROS2PollingSubscriber<MessageT>>(node, topic_name, qos);
  }
}

template <typename MessageT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MessageT>>;

  virtual ~Publisher() = default;

  virtual AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() = 0;
  virtual AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() = 0;

  virtual void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) = 0;
  virtual void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) = 0;

  virtual uint32_t get_subscription_count() const = 0;
};

template <typename MessageT>
class AgnocastPublisher : public Publisher<MessageT>
{
  typename agnocast::Publisher<MessageT>::SharedPtr publisher_;

public:
  explicit AgnocastPublisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options)
  : publisher_(agnocast::create_publisher<MessageT>(node, topic_name, qos, options))
  {
  }

  AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() override
  {
    return AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT){publisher_->borrow_loaned_message()};
  }

  AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(MessageT){publisher_->borrow_loaned_message()};
  }

  void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message)
  {
    publisher_->publish(std::move(message).move_agnocast_ptr());
  }

  void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message)
  {
    publisher_->publish(std::move(message).move_agnocast_ptr());
  }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
};

template <typename MessageT>
class ROS2Publisher : public Publisher<MessageT>
{
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_{nullptr};

public:
  explicit ROS2Publisher(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
    const agnocast::PublisherOptions & options)
  {
    rclcpp::PublisherOptions ros2_options;
    ros2_options.qos_overriding_options = options.qos_overriding_options;
    publisher_ = node->create_publisher<MessageT>(topic_name, qos, ros2_options);
  }

  AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) allocate_output_message_unique() override
  {
    return AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT){std::make_unique<MessageT>()};
  }

  AUTOWARE_MESSAGE_SHARED_PTR(MessageT) allocate_output_message_shared() override
  {
    return AUTOWARE_MESSAGE_SHARED_PTR(MessageT){std::make_shared<MessageT>()};
  }

  void publish(AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) && message) override
  {
    publisher_->publish(std::move(message).move_ros2_ptr());
  }

  void publish(AUTOWARE_MESSAGE_SHARED_PTR(MessageT) && message) override
  {
    publisher_->publish(*message);
  }

  uint32_t get_subscription_count() const override { return publisher_->get_subscription_count(); }
};

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  agnocast::PublisherOptions options;
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(node, topic_name, qos, options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(node, topic_name, qos, options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth)
{
  agnocast::PublisherOptions options;
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos,
  const agnocast::PublisherOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(node, topic_name, qos, options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(node, topic_name, qos, options);
  }
}

template <typename MessageT>
typename Publisher<MessageT>::SharedPtr create_publisher(
  rclcpp::Node * node, const std::string & topic_name, const size_t qos_history_depth,
  const agnocast::PublisherOptions & options)
{
  if (use_agnocast()) {
    return std::make_shared<AgnocastPublisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  } else {
    return std::make_shared<ROS2Publisher<MessageT>>(
      node, topic_name, rclcpp::QoS(rclcpp::KeepLast(qos_history_depth)), options);
  }
}

}  // namespace autoware::agnocast_wrapper

#else

#include "autoware_utils/ros/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

#define AUTOWARE_MESSAGE_UNIQUE_PTR(MessageT) std::unique_ptr<MessageT>
#define AUTOWARE_MESSAGE_SHARED_PTR(MessageT) std::shared_ptr<MessageT>
#define AUTOWARE_SUBSCRIPTION_PTR(MessageT) typename rclcpp::Subscription<MessageT>::SharedPtr
#define AUTOWARE_PUBLISHER_PTR(MessageT) typename rclcpp::Publisher<MessageT>::SharedPtr
#define AUTOWARE_POLLING_SUBSCRIBER_PTR(MessageT) \
  typename autoware_utils::InterProcessPollingSubscriber<MessageT>::SharedPtr

#define AUTOWARE_CREATE_SUBSCRIPTION(message_type, topic, qos, callback, options) \
  this->create_subscription<message_type>(topic, qos, callback, options)
#define AUTOWARE_CREATE_PUBLISHER2(message_type, arg1, arg2) \
  this->create_publisher<message_type>(arg1, arg2)
#define AUTOWARE_CREATE_PUBLISHER3(message_type, arg1, arg2, arg3) \
  this->create_publisher<message_type>(arg1, arg2, arg3)
#define AUTOWARE_CREATE_POLLING_SUBSCRIBER(message_type, topic, qos) \
  autoware_utils::InterProcessPollingSubscriber<message_type>::create_subscription(this, topic, qos)

#define AUTOWARE_SUBSCRIPTION_OPTIONS rclcpp::SubscriptionOptions
#define AUTOWARE_PUBLISHER_OPTIONS rclcpp::PublisherOptions

#define ALLOCATE_OUTPUT_MESSAGE_UNIQUE(publisher) \
  std::make_unique<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()
#define ALLOCATE_OUTPUT_MESSAGE_SHARED(publisher) \
  std::make_shared<typename std::remove_reference<decltype(*publisher)>::type::ROSMessageType>()

#endif
