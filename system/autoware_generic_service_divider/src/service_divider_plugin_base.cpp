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

#include "rcpputils/join.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace generic_service_divider
{

ServiceDividerPluginBase::StartupDiagnosticInfo
ServiceDividerPluginBase::get_startup_diagnostic_info()
{
  std::lock_guard<std::mutex> lock(service_start_mutex_);

  StartupDiagnosticInfo info;
  info.input_service_started = input_service_started_;
  info.input_service_name = input_service_name();
  info.total_output_service_count = output_clients_.size();

  for (const auto & entry : output_clients_) {
    if (entry.client->service_is_ready()) {
      ++info.ready_output_service_count;
    } else {
      info.waiting_output_services.push_back(entry.config.name);
    }
  }

  return info;
}

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

    entry.client = autoware::agnocast_wrapper::create_generic_client(
      node_.get(), output_cfg.name, type, rclcpp::ServicesQoS(), entry.callback_group);

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

  const auto not_ready_services = collect_not_ready_output_services();
  if (!not_ready_services.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Service divider: waiting for output service servers before advertising '%s' "
      "(ready=%zu/%zu, waiting=[%s])",
      input_service_name().c_str(), output_clients_.size() - not_ready_services.size(),
      output_clients_.size(), rcpputils::join(not_ready_services, ", ").c_str());
    return;
  }

  advertise_input_service();
}

std::vector<std::string> ServiceDividerPluginBase::collect_not_ready_output_services() const
{
  std::vector<std::string> not_ready_services;
  not_ready_services.reserve(output_clients_.size());
  for (const auto & entry : output_clients_) {
    if (!entry.client->service_is_ready()) {
      not_ready_services.push_back(entry.config.name);
    }
  }
  return not_ready_services;
}

void ServiceDividerPluginBase::advertise_input_service()
{
  const auto type = service_type();
  const auto input_name = input_service_name();

  input_service_ = autoware::agnocast_wrapper::create_generic_service(
    node_.get(), input_name, type,
    [this](AUTOWARE_GENERIC_SERVICE_PTR service, std::shared_ptr<void> request) {
      handle_request(service, request);
    },
    rclcpp::ServicesQoS(), service_callback_group_);

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
  AUTOWARE_GENERIC_SERVICE_PTR service, std::shared_ptr<void> request)
{
  auto pending = std::make_shared<PendingDivision>();
  pending->request = request;
  pending->service = service;

  for (const auto & entry : output_clients_) {
    pending->completed[entry.config.name] = false;
    pending->timed_out[entry.config.name] = false;
  }
  pending->awaiting_count = static_cast<int>(output_clients_.size());

  const int64_t pending_id = register_pending_division(pending);

  const auto request_detail = format_request(request.get());
  RCLCPP_INFO(
    node_->get_logger(), "Service divider[%ld]: call received on '%s'%s%s", pending_id,
    input_service_name().c_str(), request_detail.empty() ? "" : " request=",
    request_detail.empty() ? "" : request_detail.c_str());

  if (output_clients_.empty()) {
    // No forward_request() call will ever run to drive try_finalize_response() (it is only
    // invoked from a forward_request()-armed timeout timer or response callback), so a plugin
    // misconfigured with zero output services would otherwise leave this call unanswered forever
    // instead of failing loudly. evaluate_outputs() naturally reports no primary response in this
    // case (there is no entry to mark primary), so the caller gets a clean error instead of a
    // hang.
    try_finalize_response(pending);
    return;
  }

  for (auto & entry : output_clients_) {
    forward_request(entry, pending, pending_id, request);
  }
}

int64_t ServiceDividerPluginBase::register_pending_division(
  const std::shared_ptr<PendingDivision> & pending)
{
  std::lock_guard<std::mutex> lock(pending_map_mutex_);
  const int64_t pending_id = next_pending_id_++;
  pending_divisions_[pending_id] = pending;
  return pending_id;
}

bool ServiceDividerPluginBase::mark_output_completed(
  const std::shared_ptr<PendingDivision> & pending, const std::string & name, bool timed_out,
  std::shared_ptr<void> response)
{
  std::lock_guard<std::mutex> lock(pending->mutex);
  if (pending->completed[name]) {
    return false;
  }
  if (response) {
    pending->responses[name] = std::move(response);
  }
  pending->timed_out[name] = timed_out;
  pending->completed[name] = true;
  pending->awaiting_count--;
  return true;
}

void ServiceDividerPluginBase::forward_request(
  OutputClientEntry & entry, const std::shared_ptr<PendingDivision> & pending, int64_t pending_id,
  const std::shared_ptr<void> & request)
{
  const auto name = entry.config.name;
  const int timeout_ms = entry.config.timeout_ms;

  RCLCPP_INFO(
    node_->get_logger(),
    "Service divider[%ld]: forwarding call to '%s' (primary=%s, timeout_ms=%d)", pending_id,
    name.c_str(), entry.config.primary ? "true" : "false", timeout_ms);

  auto timer = node_->create_wall_timer(
    std::chrono::milliseconds(timeout_ms), [this, pending, name, pending_id]() {
      if (!mark_output_completed(pending, name, true, nullptr)) {
        return;
      }
      RCLCPP_WARN(
        node_->get_logger(), "Service divider[%ld]: timeout waiting for response from '%s'",
        pending_id, name.c_str());
      try_finalize_response(pending);
    });

  {
    std::lock_guard<std::mutex> lock(pending->mutex);
    pending->timeout_timers.push_back(timer);
  }

  try {
    // Borrow this specific client's own request buffer and fill it via the plugin's
    // copy_request(), rather than handing it the shared `request` object directly: this output
    // client's async_send_request() only accepts a buffer this specific client borrowed via its
    // own create_request() (a hard requirement on the Agnocast path, where `request` belongs to
    // the input service's own borrowed shared-memory buffer, not this client's -- see
    // GenericClient::async_send_request()'s doc comment in autoware_agnocast_wrapper).
    auto forwarded_request = entry.client->create_request();
    try {
      copy_request(forwarded_request.get(), request.get());
    } catch (...) {
      entry.client->cancel_request(forwarded_request);
      throw;
    }
    entry.client->async_send_request(
      forwarded_request, [this, pending, name, pending_id](
                           autoware::agnocast_wrapper::GenericClient::SharedFuture future) {
        auto response = future.get();
        const auto response_detail = format_response(response.get());
        if (!mark_output_completed(pending, name, false, response)) {
          return;  // Already timed out
        }
        RCLCPP_INFO(
          node_->get_logger(), "Service divider[%ld]: response from '%s'%s%s", pending_id,
          name.c_str(), response_detail.empty() ? "" : " response=",
          response_detail.empty() ? "" : response_detail.c_str());
        try_finalize_response(pending);
      });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Service divider[%ld]: failed to send request to '%s': %s", pending_id,
      name.c_str(), e.what());
    mark_output_completed(pending, name, true, nullptr);
    try_finalize_response(pending);
  }
}

void ServiceDividerPluginBase::try_finalize_response(std::shared_ptr<PendingDivision> pending)
{
  std::lock_guard<std::mutex> lock(pending->mutex);
  if (pending->awaiting_count > 0 || pending->finalized) {
    return;
  }
  pending->finalized = true;

  for (auto & timer : pending->timeout_timers) {
    timer->cancel();
  }
  pending->timeout_timers.clear();

  send_final_response(pending, evaluate_outputs(pending));
  erase_pending_division(pending);
}

ServiceDividerPluginBase::DivisionOutcome ServiceDividerPluginBase::evaluate_outputs(
  const std::shared_ptr<PendingDivision> & pending) const
{
  DivisionOutcome outcome;

  for (const auto & entry : output_clients_) {
    const auto & name = entry.config.name;

    if (pending->timed_out[name]) {
      RCLCPP_ERROR(node_->get_logger(), "Service divider: '%s' timed out", name.c_str());
      outcome.all_success = false;
      continue;
    }

    auto it = pending->responses.find(name);
    if (it == pending->responses.end()) {
      outcome.all_success = false;
      continue;
    }

    if (!is_response_success(it->second.get())) {
      RCLCPP_WARN(node_->get_logger(), "Service divider: '%s' returned failure", name.c_str());
      outcome.all_success = false;
    }

    if (entry.config.primary) {
      outcome.primary_name = name;
      outcome.primary_response = it->second;
    }
  }

  return outcome;
}

std::shared_ptr<void> ServiceDividerPluginBase::build_response(
  const std::shared_ptr<PendingDivision> & pending, const std::shared_ptr<void> & source)
{
  // Borrow this service's own response buffer and fill it via the plugin's copy_response(),
  // rather than sending `source` directly: none of outcome.primary_response (received through a
  // different GenericClient) or create_error_response()'s freshly-allocated object was obtained
  // from this service's own create_response(pending->request), which send_response() requires on
  // the Agnocast path (see its doc comment in autoware_agnocast_wrapper) -- Agnocast has no way to
  // publish an arbitrary heap object through a service's shared-memory channel.
  auto response = pending->service->create_response(pending->request);
  try {
    copy_response(response.get(), source.get());
  } catch (...) {
    pending->service->cancel_response(pending->request, response);
    throw;
  }
  return response;
}

void ServiceDividerPluginBase::send_final_response(
  const std::shared_ptr<PendingDivision> & pending, const DivisionOutcome & outcome)
{
  if (!outcome.primary_response) {
    RCLCPP_ERROR(
      node_->get_logger(), "Service divider: primary service did not respond, returning error");
    auto response =
      build_response(pending, create_error_response("Primary service did not respond"));
    pending->service->send_response(pending->request, response);
    return;
  }

  if (!outcome.all_success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Service divider: at least one output failed/timed out, returning error response "
      "(primary='%s')",
      outcome.primary_name.c_str());
    auto response = build_response(
      pending, create_error_response("One or more output services failed or timed out"));
    pending->service->send_response(pending->request, response);
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Service divider: all outputs succeeded, returning primary response '%s'",
    outcome.primary_name.c_str());
  auto response = build_response(pending, outcome.primary_response);
  pending->service->send_response(pending->request, response);
}

void ServiceDividerPluginBase::erase_pending_division(
  const std::shared_ptr<PendingDivision> & pending)
{
  // Find by pointer identity, since the callbacks only carry the shared pointer.
  std::lock_guard<std::mutex> map_lock(pending_map_mutex_);
  for (auto it = pending_divisions_.begin(); it != pending_divisions_.end(); ++it) {
    if (it->second == pending) {
      pending_divisions_.erase(it);
      break;
    }
  }
}

}  // namespace generic_service_divider
