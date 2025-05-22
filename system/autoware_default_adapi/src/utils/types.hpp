// Copyright 2022 TIER IV, Inc.
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

#ifndef UTILS__TYPES_HPP_
#define UTILS__TYPES_HPP_

#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>

namespace autoware::default_adapi
{

template <class T>
using Publisher = typename autoware::component_interface_utils::Publisher<T>;
template <class T>
using Subscription = typename autoware::component_interface_utils::Subscription<T>;
template <class T>
using Client = typename autoware::component_interface_utils::Client<T>;
template <class T>
using Service = typename autoware::component_interface_utils::Service<T>;

template <class T>
using PollingSubscription = autoware_utils_rclcpp::InterProcessPollingSubscriber<T>;

template <class SpecT, class T = typename SpecT::Message>
typename PollingSubscription<T>::SharedPtr create_polling_subscription(rclcpp::Node * node)
{
  using autoware::component_interface_utils::get_qos;
  return PollingSubscription<T>::create_subscription(node, SpecT::name, get_qos<SpecT>());
}

// TODO(Takagi, Isamu): remove this
template <class T>
using Pub = typename autoware::component_interface_utils::Publisher<T>::SharedPtr;
template <class T>
using Sub = typename autoware::component_interface_utils::Subscription<T>::SharedPtr;
template <class T>
using Cli = typename autoware::component_interface_utils::Client<T>::SharedPtr;
template <class T>
using Srv = typename autoware::component_interface_utils::Service<T>::SharedPtr;

}  // namespace autoware::default_adapi

#endif  // UTILS__TYPES_HPP_
