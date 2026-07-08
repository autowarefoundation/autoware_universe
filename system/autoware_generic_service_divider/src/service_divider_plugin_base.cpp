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

#include "generic_service_divider/service_divider_plugin_base.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace generic_service_divider
{

void ServiceDividerPluginBase::setup_service_division()
{
  const auto type = service_type();
  const auto outputs = output_services();

  service_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  for (const auto & output_cfg : outputs) {
    OutputClientEntry entry;
    entry.config = output_cfg;
    entry.callback_group =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rcl_client_options_t client_options = rcl_client_get_default_options();
    entry.client = std::make_shared<GenericClient>(
      node_->get_node_base_interface().get(), node_->get_node_graph_interface(), output_cfg.name,
      type, client_options);
    node_->get_node_services_interface()->add_client(
      std::dynamic_pointer_cast<rclcpp::ClientBase>(entry.client), entry.callback_group);

    output_clients_.push_back(std::move(entry));
  }

  try_start_input_service();

  if (!input_service_started_) {
    server_wait_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), [this]() { try_start_input_service(); });
  }
}

void ServiceDividerPluginBase::try_start_input_service()
{
  std::lock_guard<std::mutex> lock(service_start_mutex_);
  if (input_service_started_) {
    return;
  }

  const auto type = service_type();
  const auto input_name = input_service_name();

  std::size_t ready_count = 0;
  std::vector<std::string> not_ready_services;
  not_ready_services.reserve(output_clients_.size());
  for (const auto & entry : output_clients_) {
    if (entry.client->service_is_ready()) {
      ++ready_count;
      continue;
    }
    not_ready_services.push_back(entry.config.name);
  }

  if (!not_ready_services.empty()) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < not_ready_services.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << not_ready_services[i];
    }
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Service divider: waiting for output service servers before advertising '%s' "
      "(ready=%zu/%zu, waiting=[%s])",
      input_name.c_str(), ready_count, output_clients_.size(), oss.str().c_str());
    return;
  }

  rcl_service_options_t service_options = rcl_service_get_default_options();
  input_service_ = std::make_shared<GenericService>(
    node_->get_node_base_interface()->get_shared_rcl_node_handle(), input_name, type,
    std::bind(
      &ServiceDividerPluginBase::handle_request, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3),
    service_options);
  node_->get_node_services_interface()->add_service(
    std::dynamic_pointer_cast<rclcpp::ServiceBase>(input_service_), service_callback_group_);

  input_service_started_ = true;
  if (server_wait_timer_) {
    server_wait_timer_->cancel();
    server_wait_timer_.reset();
  }

  RCLCPP_INFO(
    node_->get_logger(), "Service divider: %s -> %zu outputs (type: %s)", input_name.c_str(),
    output_clients_.size(), type.c_str());
}

void ServiceDividerPluginBase::handle_request(
  std::shared_ptr<GenericService> service, std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<void> request)
{
  auto pending = std::make_shared<PendingDivision>();
  pending->request_header = request_header;
  pending->service = service;

  int awaiting = 0;
  for (const auto & entry : output_clients_) {
    pending->completed[entry.config.name] = false;
    pending->timed_out[entry.config.name] = false;
    ++awaiting;
  }
  pending->awaiting_count = awaiting;

  int64_t pending_id;
  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    pending_id = next_pending_id_++;
    pending_divisions_[pending_id] = pending;
  }

  const auto request_detail = format_request(request.get());
  RCLCPP_INFO(
    node_->get_logger(), "Service divider[%ld]: call received on '%s'%s%s", pending_id,
    input_service_name().c_str(), request_detail.empty() ? "" : " request=",
    request_detail.empty() ? "" : request_detail.c_str());

  for (auto & entry : output_clients_) {
    const auto & name = entry.config.name;
    const int timeout_ms = entry.config.timeout_ms;

    RCLCPP_INFO(
      node_->get_logger(),
      "Service divider[%ld]: forwarding call to '%s' (primary=%s, timeout_ms=%d)", pending_id,
      name.c_str(), entry.config.primary ? "true" : "false", timeout_ms);

    auto timer = node_->create_wall_timer(
      std::chrono::milliseconds(timeout_ms), [this, pending, name, pending_id]() {
        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          if (pending->completed[name]) {
            return;
          }
          pending->timed_out[name] = true;
          pending->completed[name] = true;
          pending->awaiting_count--;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "Service divider[%ld]: timeout waiting for response from '%s'", pending_id,
          name.c_str());
        try_finalize_response(pending);
      });

    {
      std::lock_guard<std::mutex> lock(pending->mutex);
      pending->timeout_timers.push_back(timer);
    }

    try {
      entry.client->async_send_request(
        request, [this, pending, name, pending_id](GenericClient::SharedFuture future) {
          auto response = future.get();
          const auto response_detail = format_response(response.get());
          {
            std::lock_guard<std::mutex> lock(pending->mutex);
            if (pending->completed[name]) {
              return;  // Already timed out
            }
            pending->responses[name] = response;
            pending->completed[name] = true;
            pending->awaiting_count--;
          }
          RCLCPP_INFO(
            node_->get_logger(), "Service divider[%ld]: response from '%s'%s%s", pending_id,
            name.c_str(), response_detail.empty() ? "" : " response=",
            response_detail.empty() ? "" : response_detail.c_str());
          try_finalize_response(pending);
        });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Service divider[%ld]: failed to send request to '%s': %s",
        pending_id, name.c_str(), e.what());
      {
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->completed[name] = true;
        pending->timed_out[name] = true;
        pending->awaiting_count--;
      }
      try_finalize_response(pending);
    }
  }
}

void ServiceDividerPluginBase::try_finalize_response(std::shared_ptr<PendingDivision> pending)
{
  std::lock_guard<std::mutex> lock(pending->mutex);
  if (pending->awaiting_count > 0) {
    return;
  }

  for (auto & timer : pending->timeout_timers) {
    timer->cancel();
  }
  pending->timeout_timers.clear();

  bool all_success = true;
  std::string primary_name;
  std::shared_ptr<void> primary_response;

  for (const auto & entry : output_clients_) {
    const auto & name = entry.config.name;

    if (pending->timed_out[name]) {
      RCLCPP_ERROR(node_->get_logger(), "Service divider: '%s' timed out", name.c_str());
      all_success = false;
      continue;
    }

    auto it = pending->responses.find(name);
    if (it == pending->responses.end()) {
      all_success = false;
      continue;
    }

    if (!is_response_success(it->second.get())) {
      RCLCPP_WARN(node_->get_logger(), "Service divider: '%s' returned failure", name.c_str());
      all_success = false;
    }

    if (entry.config.primary) {
      primary_name = name;
      primary_response = it->second;
    }
  }

  if (all_success && primary_response) {
    RCLCPP_INFO(
      node_->get_logger(), "Service divider: all outputs succeeded, returning primary response '%s'",
      primary_name.c_str());
    pending->service->send_response(*pending->request_header, primary_response);
  } else if (primary_response && !all_success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Service divider: at least one output failed/timed out, returning error response (primary='%s')",
      primary_name.c_str());
    auto error_resp = create_error_response("One or more output services failed or timed out");
    pending->service->send_response(*pending->request_header, error_resp);
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Service divider: primary service did not respond, returning error");
    auto error_resp = create_error_response("Primary service did not respond");
    pending->service->send_response(*pending->request_header, error_resp);
  }

  // Clean up from pending map (find by pointer identity)
  {
    std::lock_guard<std::mutex> map_lock(pending_map_mutex_);
    for (auto it = pending_divisions_.begin(); it != pending_divisions_.end(); ++it) {
      if (it->second == pending) {
        pending_divisions_.erase(it);
        break;
      }
    }
  }
}

}  // namespace generic_service_divider
